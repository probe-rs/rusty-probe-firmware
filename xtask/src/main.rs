use std::{
    io::Write,
    time::{Duration, Instant},
};

use clap::Parser;
use serialport::SerialPort;
use xshell::{cmd, Shell};

#[derive(Debug, clap::Parser)]
pub enum Command {
    /// Reboot the Rusty Probe into the USB bootloader
    Reboot {
        /// If the serial port that is specified can not be opened
        /// (usually because it is not present), exit with success
        #[clap(long)]
        succeed_if_absent: bool,
    },
    /// Flash a firmware elf
    ///
    /// Requires that `elf2uf2-rs` is installed and the rp2040's USB
    /// filesystem is mounted.
    Flash {
        /// Path to the ELF file to be flash
        elf: String,
    },

    /// Reboot the Rusty Probe into the USB bootloader, and flash
    /// a firmware ELF
    ///
    /// Requires that `elf2uf2-rs` is installed, and that the rp2040's
    /// USB filesystem is automounted.
    RebootAndFlash {
        /// Path to the ELF file to be flashed
        elf: String,
    },

    /// Read log data from the Rusty Probe, and print the
    /// data to `stdout`. The output data can be pumped into
    /// a tool like [`defmt-print`]
    ///
    /// [`defmt-print`]: https://crates.io/crates/defmt-print
    ReadLog,

    /// Reboot the Rusty Probe into the USB bootloader, flash
    /// a firmware ELF, wait for it to restart, and write log data
    /// to `stdout`
    #[clap(alias = "reboot-flash-readlog")]
    Debug {
        /// Path to the ELF file to be flashed
        elf: String,
    },
}

#[derive(Debug, clap::Parser)]
pub struct Cli {
    /// The name of the serial port to use for sending commands or
    /// reading log data
    #[cfg_attr(
        target_os = "linux",
        clap(short, long, default_value = "/dev/ttyACM0", global = true)
    )]
    #[clap(short, long, global = true)]
    serial_port: String,

    /// How long we should wait for the rp2040's USB filesystem
    /// to be mounted while flashing (in milliseconds).
    #[clap(global = true, long, short, default_value = "5000")]
    timeout: u64,

    #[clap(subcommand)]
    command: Command,
}

fn reboot(serial_port: &mut Box<dyn SerialPort>) -> anyhow::Result<()> {
    eprintln!("Restarting probe into USB bootloader");
    serial_port.write_all(&0xDABAD000u32.to_be_bytes())?;
    Ok(())
}

fn read_log(serial_port: &mut Box<dyn SerialPort>) -> anyhow::Result<()> {
    let mut stdout = std::io::stdout();
    let mut buf = [0u8; 2048];

    loop {
        let data = match serial_port.read(&mut buf) {
            Ok(data) => data,
            Err(e) => {
                if e.kind() != std::io::ErrorKind::TimedOut {
                    break Err(e.into());
                } else {
                    continue;
                }
            }
        };

        stdout.write_all(&buf[..data])?;
        stdout.flush()?;
    }
}

fn flash(elf: String, timeout_ms: u64) -> anyhow::Result<()> {
    if let Err(e) = std::fs::File::open(&elf) {
        return Err(anyhow::anyhow!("Could not open ELF file: {e}"));
    }

    eprintln!("Flashing Rusty Probe");

    let sh = Shell::new()?;

    let start = Instant::now();

    let success = loop {
        if start.elapsed() > Duration::from_millis(timeout_ms) {
            break false;
        }

        if cmd!(sh, "elf2uf2-rs -d {elf}")
            .ignore_stdout()
            .ignore_stderr()
            .quiet()
            .run()
            .is_ok()
        {
            break true;
        }

        std::thread::sleep(Duration::from_secs(1));
    };

    if success {
        Ok(())
    } else {
        Err(anyhow::anyhow!("Flash attempt timed out"))
    }
}

fn main() -> anyhow::Result<()> {
    let opts = Cli::parse();

    let serial_port = || {
        serialport::new(&opts.serial_port, 115200)
            .timeout(std::time::Duration::from_millis(100))
            .open()
            .map_err(|e| anyhow::anyhow!(format!("Failed to open serial port: {e}")))
    };

    match opts.command {
        Command::Reboot { succeed_if_absent } => match serial_port() {
            Ok(mut serial_port) => reboot(&mut serial_port),
            Err(e) if succeed_if_absent => {
                eprintln!("Could not open Serial Port ({e}). Exiting with success");
                Ok(())
            }
            Err(e) => Err(e),
        },
        Command::Flash { elf } => flash(elf, opts.timeout),
        Command::RebootAndFlash { elf } => {
            if let Ok(mut port) = serial_port() {
                reboot(&mut port)?;
            } else {
                eprintln!(
                    "Could not find probe to restart... Assuming it is already in USB bootloader."
                );
            };

            flash(elf, opts.timeout)
        }
        Command::ReadLog => {
            let mut serial_port = serial_port()?;
            read_log(&mut serial_port)
        }
        Command::Debug { elf } => {
            if let Ok(mut port) = serial_port() {
                reboot(&mut port)?;
            } else {
                eprintln!(
                    "Could not find probe to restart... Assuming it is already in USB bootloader."
                );
            };

            flash(elf, opts.timeout)?;

            let start = Instant::now();
            loop {
                if start.elapsed() > Duration::from_secs(5) {
                    return Err(anyhow::anyhow!("Acquiring log serial port timed out"));
                }

                if let Ok(mut port) = serial_port() {
                    // Sleep a little while so that some startup data can be accumulated
                    // in the serial port.
                    //
                    // Not entirely certain that this really helps, but ¯\_(ツ)_/¯
                    std::thread::sleep(Duration::from_millis(200));
                    break read_log(&mut port);
                }
            }
        }
    }
}
