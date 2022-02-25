use rp_pico::{
    hal::{
        gpio::{
            bank0::{Gpio14, Gpio15},
            Disabled, DynPin, FunctionPio0, OutputDriveStrength, OutputSlewRate, Pin, PullDown,
        },
        pio::{PIOBuilder, PIOExt, PinDir, ShiftDirection},
    },
    pac::{PIO0, RESETS},
};

pub fn setup_pio(
    resets: &mut RESETS,
    pio0: PIO0,
    mut swdclk: Pin<Gpio15, Disabled<PullDown>>,
    mut swdio: Pin<Gpio14, Disabled<PullDown>>,
) -> () {
    // High speed IO
    swdio.set_drive_strength(OutputDriveStrength::TwelveMilliAmps);
    swdio.set_slew_rate(OutputSlewRate::Fast);
    swdclk.set_drive_strength(OutputDriveStrength::TwelveMilliAmps);
    swdclk.set_slew_rate(OutputSlewRate::Fast);
    let swdclk = swdclk.into_push_pull_output();

    //// PIO

    let program = pio_proc::pio!(
        32,
        "
; Goals:
;
; * Send/receive data at max speed (F/2)
; * Allow for sending or receiving, or use 2 PIO programs
;
; SWD Protocol steps:
; 1. Send request byte
; 2. Read ACK, return if error (send 32 clocks if error, as per standard)
; 3. Send/Receive 32 bit word + parity
; 4. Perform extra trailing clocks as per the standard
;

; - side x is the swd clock signal
; - uses auto push/pull with 8 bits limit
; - IRQ 0 is done (TODO: check this is fine)
; - IRQ 1 is error (TODO: check this is fine)

.side_set 1 ; each instruction may set 1 bit

; TODO

; 1. send request
pull                        side 0
set x, 7                    side 0

set pindirs 1               side 0 ; output
req:
    out pins, 1             side 0
    jmp x-- req             side 1

; 2. receive ack
set pindirs 0               side 0

; turnaround
nop                         side 1
nop                         side 0

; read ACK (nop while testing with logic analyzer)
nop side 1 ; jmp pin ack_ok              side 1 ; if first bit is 1, ack maybe OK
           ; jmp ack_error               side 0 ; if 0, jump to error checking
           ; ack_ok:
nop side 0 ; nop                         side 0
nop side 1 ; jmp pin ack_error_protocol  side 1 ; if second bit is 1, protocol error
set x, 31                   side 0              ; prepare for read: 32bit data 
nop side 1 ; jmp pin ack_error_protocol  side 1 ; if third bit is 1, protocol error

; if there is no data in the output register, it's a read operation
pull noblock                side 0
jmp !osre write_data        side 0 

read_data:   ; we are in read from before, no need to set pindirs again
    read_loop:
        in pins, 1          side 1
        jmp x-- read_loop   side 0

    push                    side 0

    ; read parity
    in pins, 1              side 1
    push                    side 0

    set x, 7                side 1
    jmp trail_loop          side 0

write_data:
    ; turnaround (write)
    set pindirs 1           side 1 ; output mode and turnaround clock

    write_loop:
        out pins, 1         side 0
        jmp !osre write_loop  side 1

    pull  side 0
    out pins, 1         side 0 ; clock out parity
    set x, 7            side 1
    jmp trail_loop                    side 0

ack_error:

; TODO, figure out if WAIT or FAULT

ack_error_protocol:

irq 1                       side 0 ; error
set x, 31                   side 0

; trailing clocks
trail_loop:
    set pindirs 0           side 0 ; input
    jmp x-- trail_loop      side 1

done:
    irq wait 0              side 0
        "
    );

    // Initialize and start PIO

    // configure LED pin for Pio0.
    let swdclk: Pin<_, FunctionPio0> = swdclk.into_mode();
    let swdclk: DynPin = swdclk.into();
    let swdio: Pin<_, FunctionPio0> = swdio.into_mode();
    let swdio: DynPin = swdio.into();
    // PIN id for use inside of PIO
    // let pio_pin_id = 25;

    let (mut pio, sm0, _, _, _) = pio0.split(resets);
    let installed = pio.install(&program.program).unwrap();
    let div = 0f32; // as slow as possible (0 is interpreted as 65536)
    let (mut sm, mut rx_fifo, mut tx_fifo) = PIOBuilder::from_program(installed)
        .side_set_pin_base(swdclk.id().num)
        .out_shift_direction(ShiftDirection::Right)
        .out_pins(swdio.id().num, 1)
        .in_pin_base(swdio.id().num)
        .set_pins(swdio.id().num, 1)
        // .autopull(true)
        // .pull_threshold(8)
        .autopush(true)
        .push_threshold(8)
        .clock_divisor(div)
        .build(sm0);

    // The GPIO pin needs to be configured as an output.
    sm.set_pindirs([
        (swdio.id().num, PinDir::Input),
        (swdclk.id().num, PinDir::Output),
    ]);

    assert!(tx_fifo.write(0xaa));
    assert!(tx_fifo.write(0xf0f0f0f0u32));
    // assert!(tx_fifo.write(0xf0u8));
    // assert!(tx_fifo.write(0xf0u8));
    // assert!(tx_fifo.write(0xf0u8));
    assert!(tx_fifo.write(0));

    sm.start();
}
