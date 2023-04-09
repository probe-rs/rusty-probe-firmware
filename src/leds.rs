use dap_rs::dap::{DapLeds, HostStatus};
use embedded_hal::digital::v2::OutputPin;
use rp2040_hal::gpio::{bank0::*, Pin, PushPullOutput};

pub struct BoardLeds {
    green: Pin<Gpio27, PushPullOutput>,
    red: Pin<Gpio28, PushPullOutput>,
    blue: Pin<Gpio29, PushPullOutput>,
    rgb: (bool, bool, bool),
}

impl BoardLeds {
    pub fn new(
        red: Pin<Gpio28, PushPullOutput>,
        green: Pin<Gpio27, PushPullOutput>,
        blue: Pin<Gpio29, PushPullOutput>,
    ) -> Self {
        let mut me = Self {
            red,
            green,
            blue,
            rgb: (false, false, false),
        };
        me.rgb(false, false, false);
        me
    }

    fn set_red(&mut self, level: bool) {
        self.red.set_state((!level).into()).ok();
    }

    fn set_green(&mut self, level: bool) {
        self.green.set_state((!level).into()).ok();
    }

    fn set_blue(&mut self, level: bool) {
        self.blue.set_state((!level).into()).ok();
    }

    pub fn state(&self) -> (bool, bool, bool) {
        self.rgb
    }

    pub fn rgb(&mut self, r: bool, g: bool, b: bool) {
        self.rgb = (r, g, b);
        self.set_red(r);
        self.set_green(g);
        self.set_blue(b);
    }

    pub fn red(&mut self) {
        self.rgb(true, false, false);
    }

    /// R + G = yellow
    pub fn yellow(&mut self) {
        self.rgb(true, true, false);
    }

    pub fn green(&mut self) {
        self.rgb(false, true, false);
    }

    // G + B = light blue
    pub fn light_blue(&mut self) {
        self.rgb(false, true, true);
    }

    pub fn blue(&mut self) {
        self.rgb(false, false, true);
    }

    // R + B = purple-ish
    pub fn pink(&mut self) {
        self.rgb(true, false, true);
    }

    // R + G + B = white/purple
    pub fn white(&mut self) {
        self.rgb(true, true, true);
    }

    /// Off
    pub fn off(&mut self) {
        self.rgb(false, false, false);
    }
}

#[derive(Debug, Clone, Copy, PartialEq, defmt::Format)]
pub enum Vtarget {
    Voltage1V8,
    Voltage3V3,
}

use core::task::Poll;
use core::{
    marker::PhantomData,
    sync::atomic::{AtomicU8, Ordering},
};
use rtic_common::waker_registration::CriticalSectionWakerRegistration;

pub struct HostStatusToken {
    // To make sure no one else can construct it
    _field: PhantomData<()>,
}

pub struct LedManager {
    leds: BoardLeds,
    dap_leds: Option<HostStatusToken>,
}

impl LedManager {
    pub fn new(leds: BoardLeds) -> Self {
        Self {
            leds,
            dap_leds: Some(HostStatusToken {
                _field: Default::default(),
            }),
        }
    }

    fn vtarget_storage() -> &'static AtomicU8 {
        static VTARGET_STORAGE: AtomicU8 = AtomicU8::new(0);
        &VTARGET_STORAGE
    }

    fn waker() -> &'static CriticalSectionWakerRegistration {
        static REG: CriticalSectionWakerRegistration = CriticalSectionWakerRegistration::new();
        &REG
    }

    fn host_status_storage() -> &'static AtomicU8 {
        static HOST_STATUS_STORAGE: AtomicU8 = AtomicU8::new(0);
        &HOST_STATUS_STORAGE
    }

    fn current_host_status() -> Option<HostStatus> {
        let value = Self::host_status_storage().load(Ordering::Relaxed);

        if value == 1 {
            Some(HostStatus::Connected(false))
        } else if value == 2 {
            Some(HostStatus::Connected(true))
        } else if value == 3 {
            Some(HostStatus::Running(false))
        } else if value == 4 {
            Some(HostStatus::Running(true))
        } else {
            None
        }
    }

    pub fn set_host_status(host_status: HostStatus) {
        let value = match host_status {
            HostStatus::Connected(false) => 1,
            HostStatus::Connected(true) => 2,
            HostStatus::Running(false) => 3,
            HostStatus::Running(true) => 4,
        };

        Self::host_status_storage().store(value, Ordering::Relaxed);

        // Updating the host status is always a state transition,
        // so we want to wake the waker.
        Self::waker().wake();
    }

    pub fn host_status_token(&mut self) -> Option<HostStatusToken> {
        self.dap_leds.take()
    }

    pub fn current_vtarget() -> Option<Vtarget> {
        let value = Self::vtarget_storage().load(Ordering::Relaxed);

        if value == 1 {
            Some(Vtarget::Voltage1V8)
        } else if value == 2 {
            Some(Vtarget::Voltage3V3)
        } else {
            None
        }
    }

    pub fn set_current_vtarget(vtarget: Option<Vtarget>) {
        let new_value = if vtarget == Some(Vtarget::Voltage1V8) {
            1
        } else if vtarget == Some(Vtarget::Voltage3V3) {
            2
        } else {
            0
        };

        let current_vtarget = Self::current_vtarget();
        Self::vtarget_storage().store(new_value, Ordering::Relaxed);

        // If Vtarget has changed, we may want to make a change to the LEDs
        if current_vtarget != vtarget {
            Self::waker().wake();
        }
    }

    fn set(&mut self) -> bool {
        let current_vtarget = Self::current_vtarget();
        let current_host_status = Self::current_host_status();

        let is_activity = match (current_vtarget, current_host_status) {
            (Some(current_vtarget), None)
            | (Some(current_vtarget), Some(HostStatus::Connected(false))) => {
                match current_vtarget {
                    Vtarget::Voltage1V8 => self.leds.yellow(),
                    Vtarget::Voltage3V3 => self.leds.white(),
                }
                false
            }
            (_, Some(HostStatus::Connected(true))) => {
                self.leds.blue();
                true
            }
            (_, Some(HostStatus::Running(false))) => {
                self.leds.light_blue();
                true
            }
            (_, Some(HostStatus::Running(true))) => {
                self.leds.green();
                true
            }
            (None, _) => {
                self.leds.red();
                false
            }
        };

        is_activity
    }

    async fn update(&mut self) {
        let is_activity = self.set();
        if is_activity {
            // Wait for a little bit so the status can actually be seen.
            use rtic_monotonics::rp2040::{ExtU64, Timer};

            Timer::delay(50.millis()).await;
        }
    }

    pub async fn run(&mut self) -> ! {
        loop {
            let mut polled = false;

            // Set the LEDs to whatever the current state is.
            self.set();

            // Wait for an update to occur
            core::future::poll_fn(|ctx| {
                if !polled {
                    polled = true;
                    Self::waker().register(ctx.waker());
                    Poll::Pending
                } else {
                    Poll::Ready(())
                }
            })
            .await;

            self.update().await;
        }
    }
}

impl DapLeds for HostStatusToken {
    fn react_to_host_status(&mut self, host_status: HostStatus) {
        LedManager::set_host_status(host_status);
    }
}
