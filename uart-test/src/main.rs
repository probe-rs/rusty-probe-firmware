type SerialPort = Box<dyn serialport::SerialPort>;

fn try_transmit(target: &mut SerialPort, host: &mut SerialPort, data: &[u8]) {
    target.write_all(data).unwrap();
    let mut host_buffer = vec![0u8; data.len()];
    host.read_exact(&mut host_buffer).unwrap();
    assert_eq!(data, &host_buffer);
}

enum Command {
    TransmitCount(u32),
    Bootload,
}

impl Command {
    fn from_command_line(arg: &String) -> Self {
        if arg == "bootload" {
            Self::Bootload
        } else {
            Self::TransmitCount(arg.parse::<u32>().expect("invalid count"))
        }
    }
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    if args.len() != 5 {
        eprintln!("Usage: {} <target_tty> <host_tty> <baud_rate> bootload|<count>", args[0]);
        std::process::exit(1);
    }

    let target_path = args[1].as_str();
    let host_path = args[2].as_str();
    let baud_rate = args[3].parse::<u32>().expect("invalid baud rate");
    let command = Command::from_command_line(&args[4]);

    let mut target_serial = serialport::new(target_path, baud_rate).open().expect("failed to open target serial");
    target_serial.set_timeout(std::time::Duration::from_secs(1));

    if let Command::TransmitCount(count) = command {
        let mut host_serial = serialport::new(host_path, baud_rate).open().expect("failed to open host serial");
        host_serial.set_timeout(std::time::Duration::from_secs(1));
        let test_data: Vec<u32> = (0..count).collect();
        let test_data_bytes = unsafe{ std::slice::from_raw_parts(test_data.as_ptr() as *const u8, (count * 4) as usize) };
        try_transmit(&mut target_serial, &mut host_serial, test_data_bytes);
    } else {
        target_serial.write_all(&0xDABAD000u32.to_be_bytes());
    }
}