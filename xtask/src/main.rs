use std::{
    io::Write,
    process::Stdio,
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
    /// data to `stdout` or into `defmt-print`
    ReadLog {
        /// If set, this value is used as path to the ELF that `defmt-print`
        /// is started with, and the log sent into `defmt-print`.
        #[clap(short, long, env = "XTASK_DEFMT_PRINT")]
        defmt_elf_path: Option<String>,

        /// Increase `defmt-print`'s verbosity
        #[clap(short, env = "XTASK_DEFMT_VERBOSE")]
        verbose: bool,
    },

    /// Reboot the Rusty Probe into the USB bootloader, flash
    /// a firmware ELF, wait for it to restart, and write log data
    /// to `stdout` or into `defmt-print`.
    #[clap(alias = "reboot-flash-readlog")]
    Debug {
        /// Path to the ELF file to be flashed
        elf: String,
        /// If set, `defmt-print` is started and the log data
        /// is sent into it.
        #[clap(short, long, env = "XTASK_DEFMT_PRINT")]
        defmt_print: bool,
        /// Increase `defmt-print`'s verbosity
        #[clap(short, env = "XTASK_DEFMT_VERBOSE")]
        verbose: bool,
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
    #[clap(short, long, global = true, env = "XTASK_SERIAL")]
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

fn read_log(
    port: &mut Box<dyn SerialPort>,
    defmt_print: Option<(String, bool)>,
) -> anyhow::Result<()> {
    let buf = &mut [0u8; 2048];

    fn read_data<'a>(
        port: &mut Box<dyn SerialPort>,
        buffer: &'a mut [u8],
    ) -> anyhow::Result<&'a [u8]> {
        match port.read(buffer) {
            Ok(data) => Ok(&buffer[..data]),
            Err(e) => {
                if e.kind() != std::io::ErrorKind::TimedOut {
                    Err(anyhow::anyhow!(e))
                } else {
                    Ok(&buffer[..0])
                }
            }
        }
    }

    if let Some((elf, verbose)) = defmt_print {
        eprintln!("Starting defmt-print");

        // Don't use `xtask` here because we want to access stdin of
        // the child process.
        let mut command = std::process::Command::new("defmt-print");

        if verbose {
            command.arg("--show-skipped-frames");
        }

        let mut child = command
            .args(["-e".to_owned(), format!("{elf}")])
            .stdin(Stdio::piped())
            .spawn()
            .map_err(|e| anyhow::anyhow!("Starting `defmt-print` failed: {e}"))?;

        let mut stdin = child.stdin.take().unwrap();

        loop {
            let data: &[u8] = read_data(port, buf)?;
            stdin.write_all(data)?;
            stdin.flush()?;
        }
    } else {
        let mut stdout = std::io::stdout();

        loop {
            let data = read_data(port, buf)?;
            stdout.write_all(data)?;
            stdout.flush()?;
        }
    };
}

fn flash(elf: &str, timeout_ms: u64) -> anyhow::Result<()> {
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
        Command::Flash { elf } => flash(&elf, opts.timeout),
        Command::RebootAndFlash { elf } => {
            if let Ok(mut port) = serial_port() {
                reboot(&mut port)?;
            } else {
                eprintln!(
                    "Could not find probe to restart... Assuming it is already in USB bootloader."
                );
            };

            flash(&elf, opts.timeout)
        }
        Command::ReadLog {
            defmt_elf_path,
            verbose,
        } => {
            let mut serial_port = serial_port()?;
            read_log(&mut serial_port, defmt_elf_path.map(|p| (p, verbose)))
        }
        Command::Debug {
            elf,
            defmt_print,
            verbose,
        } => {
            if let Ok(mut port) = serial_port() {
                reboot(&mut port)?;
            } else {
                eprintln!(
                    "Could not find probe to restart... Assuming it is already in USB bootloader."
                );
            };

            flash(&elf, opts.timeout)?;

            let defmt_print = if defmt_print { Some(elf) } else { None };

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
                    break read_log(&mut port, defmt_print.map(|p| (p, verbose)));
                }
            }
        }
    }
}
