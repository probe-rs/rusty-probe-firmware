use anyhow::{Context, Result, anyhow};
use nusb::{
    MaybeFuture,
    io::{EndpointRead, EndpointWrite},
    transfer::{Bulk, Direction},
};
use std::{
    io::{Read, Write},
    time::Duration,
};

pub struct UsbBridge {
    in_ep: EndpointRead<Bulk>,
    out_ep: EndpointWrite<Bulk>,
}

impl UsbBridge {
    pub fn new(vendor_id: u16, product_id: u16, serial: Option<String>) -> Result<Self> {
        let (in_ep, out_ep) = Self::get_probe_endpoints(vendor_id, product_id, serial)?;
        Ok(Self { in_ep, out_ep })
    }

    fn get_probe_endpoints(
        vendor_id: u16,
        product_id: u16,
        serial: Option<String>,
    ) -> Result<(EndpointRead<Bulk>, EndpointWrite<Bulk>)> {
        let devices: Vec<_> = nusb::list_devices()
            .wait()
            .context("Failed to list USB devices")?
            .filter(|d| d.vendor_id() == vendor_id && d.product_id() == product_id)
            .collect();
        if devices.len() == 0 {
            return Err(anyhow!(
                "Target device VID:0x{:04X} PID:0x{:04X} not found",
                vendor_id,
                product_id
            ));
        }
        if serial.is_none() && devices.len() > 1 {
            let serials = devices
                .into_iter()
                .map(|d| d.serial_number().unwrap_or("<UNKNOWN>").to_owned())
                .collect::<Vec<_>>()
                .join("\n    ");
            return Err(anyhow!(
                "More than one device found with VID:0x{:04X} PID:0x{:04X}; choose a serial number:\n    {}",
                vendor_id,
                product_id,
                serials
            ));
        }

        let device = if devices.len() == 1 {
            devices[0].clone()
        } else {
            devices
                .into_iter()
                .find(|d| d.serial_number() == serial.as_deref())
                .ok_or_else(|| anyhow!("No probe device had serial number {:?}", serial))?
        }
        .open()
        .wait()
        .context("Failed to open USB device")?;

        let config = device
            .active_configuration()
            .context("Failed to read active USB configuration")?;

        for interface in config.interfaces() {
            for alt in interface.alt_settings() {
                if let Some(str_idx) = alt.string_index()
                    && let Ok(name) = device
                        .get_string_descriptor(str_idx, 0x0409, Duration::from_millis(500))
                        .wait()
                    && name == "Rusty-Probe Debug Interface"
                {
                    let mut in_ep = None;
                    let mut out_ep = None;

                    for ep in alt.endpoints() {
                        match ep.direction() {
                            Direction::In => in_ep = Some(ep.address()),
                            Direction::Out => out_ep = Some(ep.address()),
                        }
                    }

                    let in_addr = in_ep.ok_or_else(|| anyhow!("Missing IN bulk endpoint"))?;
                    let out_addr = out_ep.ok_or_else(|| anyhow!("Missing OUT bulk endpoint"))?;

                    let claimed_interface = device
                        .claim_interface(interface.interface_number())
                        .wait()
                        .context("Failed to claim target USB interface")?;

                    return Ok((
                        claimed_interface
                            .endpoint(in_addr)
                            .unwrap()
                            .reader(4096)
                            .with_num_transfers(2),
                        claimed_interface
                            .endpoint(out_addr)
                            .unwrap()
                            .writer(4096)
                            .with_num_transfers(2)
                            .with_write_timeout(Duration::from_millis(260)),
                    ));
                }
            }
        }

        Err(anyhow!(
            "Interface 'Rusty-Probe Debug Interface' not found on the device"
        ))
    }

    pub fn write_all(&mut self, buf: &[u8]) -> std::io::Result<()> {
        self.out_ep.write_all(buf)
    }

    pub fn flush_writes(&mut self) -> std::io::Result<()> {
        self.out_ep.flush_end()
    }

    pub fn read(&mut self, buf: &mut [u8]) -> std::io::Result<usize> {
        self.in_ep.read(buf)
    }
}
