type SerialPort = Box<dyn serialport::SerialPort>;

fn try_transmit(target: &mut SerialPort, host: &mut SerialPort, data: &[u8]) {
    target.write_all(data).unwrap();
    let mut host_buffer = vec![0u8; data.len()];
    host.read_exact(&mut host_buffer).unwrap();
    assert_eq!(data, &host_buffer);
}

fn main() {
    let mut target_serial = serialport::new("/dev/ttyACM0", 115200).open().expect("failed to open target serial");
    target_serial.set_timeout(std::time::Duration::from_secs(1));
    let mut host_serial = serialport::new("/dev/ttyUSB0", 115200).open().expect("failed to open host serial");
    host_serial.set_timeout(std::time::Duration::from_secs(1));

    let test_data: Vec<u32> = (0..512/4).collect();
    let test_data_bytes = unsafe{ std::slice::from_raw_parts(test_data.as_ptr() as *const u8, 64) };
    try_transmit(&mut target_serial, &mut host_serial, test_data_bytes);
}