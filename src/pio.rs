use embedded_hal::digital::v2::OutputPin;
use rp2040_hal::{
    gpio::{
        bank0::*, DynPinId, FunctionNull, FunctionPio0, FunctionSioOutput, OutputDriveStrength,
        OutputSlewRate, Pin, PullDown, PullNone,
    },
    pac::{PIO0, RESETS},
    pio::{PIOBuilder, PIOExt, PinDir, PinState, ShiftDirection},
};

pub fn setup_pio(
    resets: &mut RESETS,
    pio0: PIO0,
    mut swdclk: Pin<Gpio11, FunctionNull, PullDown>,
    mut swdio: Pin<Gpio10, FunctionNull, PullDown>,
    mut swdio_dir: Pin<Gpio12, FunctionNull, PullDown>,
) -> () {
    // High speed IO
    swdio.set_drive_strength(OutputDriveStrength::TwelveMilliAmps);
    swdio.set_slew_rate(OutputSlewRate::Fast);
    swdio_dir.set_drive_strength(OutputDriveStrength::TwelveMilliAmps);
    swdio_dir.set_slew_rate(OutputSlewRate::Fast);
    swdclk.set_drive_strength(OutputDriveStrength::TwelveMilliAmps);
    swdclk.set_slew_rate(OutputSlewRate::Fast);
    let mut swdclk = swdclk.into_push_pull_output();
    swdclk.set_low().ok();
    let mut swdio_dir = swdio_dir.into_push_pull_output();
    swdio_dir.set_low().ok();

    //// PIO
    let program = pio_proc::pio_file!("./src/swd.pio");

    // Initialize and start PIO

    let swdclk: Pin<_, FunctionPio0, _> = swdclk.into_function();
    let swdclk: Pin<DynPinId, FunctionSioOutput, PullNone> = swdclk
        .into_push_pull_output()
        .into_pull_type()
        .into_dyn_pin();
    let swdio: Pin<_, FunctionPio0, _> = swdio.into_function();
    let swdio: Pin<DynPinId, FunctionSioOutput, PullNone> = swdio
        .into_push_pull_output()
        .into_pull_type()
        .into_dyn_pin();
    let swdio_dir: Pin<_, FunctionPio0, _> = swdio_dir.into_function();
    let swdio_dir: Pin<DynPinId, FunctionSioOutput, PullNone> = swdio_dir
        .into_push_pull_output()
        .into_pull_type()
        .into_dyn_pin();

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
        .clock_divisor_fixed_point(0, 0) // As slow as possible
        .build(sm0);

    // The GPIO pin needs to be configured as an output.
    sm.set_pins([
        (swdclk.id().num, PinState::Low),
        (swdio_dir.id().num, PinState::Low),
    ]);
    sm.set_pindirs([
        (swdio.id().num, PinDir::Input),
        (swdclk.id().num, PinDir::Output),
        (swdio_dir.id().num, PinDir::Output),
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
