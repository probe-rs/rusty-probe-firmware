use embedded_hal::digital::v2::OutputPin;
use rp_pico::{
    hal::{
        gpio::{
            bank0::{Gpio14, Gpio15},
            Disabled, DynPin, FunctionPio0, OutputDriveStrength, OutputSlewRate, Pin, PullDown,
        },
        pio::{PIOBuilder, PIOExt, PinDir, PinState, ShiftDirection},
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
    let mut swdclk = swdclk.into_push_pull_output();
    swdclk.set_low().ok();

    //// PIO

    let program = pio_proc::pio_asm!(
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
;
; - side x is the swd clock signal
; - uses auto pull with 32 bits limit
; - uses manual push with 32 bits limit
; - IRQ 0 is done flag
;
;
; Usage (sending):
;
; txfifo.write(request_bytes);
; txfifo.write(data);
; txfifo.write(parity);
; // clear done IRQ to start
; // wait for done IRQ
; let ack = rxfifo.read().unwrap(); // check if it's OK
;
; Usage (receiving):
;
; txfifo.write(request_bytes);
; // clear done IRQ to start
; // wait for done IRQ
; let ack = rxfifo.read().unwrap(); // check if it's OK
; let data = rxfifo.read().unwrap();
; let parity = rxfifo.read().unwrap(); // check if it's OK

.side_set 1 ; each instruction may set 1 bit

; start; wait for the clearing of the done flag
irq wait 0 rel                  side 0

; 1. send request (8)
set x, 7                        side 0

set pindirs 1                   side 0 ; output
req:
    out pins, 1                 side 0
    jmp x-- req                 side 1

out null, 32                    side 0 ; discard the rest

; 2. receive ack

; turnaround
set pindirs 0                   side 1 ; input

set x, 2                        side 0
ack_loop:
    in pins, 1                  side 1
    jmp x-- ack_loop            side 0

; push ack 
push                            side 0

; 3. read/write data 

set x, 31                       side 0 ; prepare for 32bits of data 
; if there is no data in the output register, it's a read operation
jmp !osre write_data            side 0

read_data:   ; we are in read from before, no need to set pindirs again
    read_loop:
        in pins, 1              side 1
        jmp x-- read_loop       side 0

    push                        side 0

    ; read parity
    in pins, 1                  side 1
    push                        side 0

    jmp trail_clocks            side 0

write_data:
    ; turnaround (write)
    set pindirs 1               side 1 ; output mode and turnaround clock

    write_loop:
        out pins, 1             side 0
        jmp x-- write_loop      side 1

    out pins, 1                 side 0 ; clock out parity
    out null, 32                side 1 ; discard the rest

; 4. read/write data 

; trailing clocks
trail_clocks:
set y, 7                        side 0 ; y register is used for trailing clocks

trail_loop:
    set pindirs 0               side 0 ; input
    jmp y-- trail_loop          side 1
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
    let (mut sm, mut rx_fifo, mut tx_fifo) = PIOBuilder::from_program(installed)
        .side_set_pin_base(swdclk.id().num)
        .out_shift_direction(ShiftDirection::Right)
        .out_pins(swdio.id().num, 1)
        .in_pin_base(swdio.id().num)
        .set_pins(swdio.id().num, 1)
        .autopull(true)
        .autopush(false)
        .pull_threshold(32)
        .push_threshold(32)
        .clock_divisor_fixed_point(0, 0) // as slow as possible
        .build(sm0);

    // The GPIO pin needs to be configured as an output.
    sm.set_pins([(swdclk.id().num, PinState::Low)]);
    sm.set_pindirs([
        (swdio.id().num, PinDir::Input),
        (swdclk.id().num, PinDir::Output),
    ]);

    let _sm = sm.start();

    let irq = &pio.irq0();

    //
    // Test RX
    //

    assert!(tx_fifo.write(0xaa));
    // assert!(tx_fifo.write(0xf0f0f0f0u32));
    // assert!(tx_fifo.write(0));

    while !irq.raw().sm0() {} // Wait for start

    defmt::info!("Started RX");
    pio.clear_irq(1);

    while !irq.raw().sm0() {} // Wait for done

    defmt::info!("ack:    0x{:X}", rx_fifo.read().unwrap());
    defmt::info!("data:   0x{:X}", rx_fifo.read().unwrap());
    defmt::info!("parity: 0x{:X}", rx_fifo.read().unwrap());
    assert!(rx_fifo.read().is_none());

    //
    // Test TX
    //

    assert!(tx_fifo.write(0xaa));
    assert!(tx_fifo.write(0xf0f0f0f0u32));
    assert!(tx_fifo.write(0));

    defmt::info!("");
    defmt::info!("Started TX");
    pio.clear_irq(1);

    while !irq.raw().sm0() {} // Wait for done

    defmt::info!("ack:    0x{:X}", rx_fifo.read().unwrap());
    assert!(rx_fifo.read().is_none());
}
