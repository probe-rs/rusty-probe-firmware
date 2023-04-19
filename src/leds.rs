use core::marker::PhantomData;
use core::num::NonZeroU8;
use core::sync::atomic::{AtomicU8, Ordering};
use core::task::Poll;

use dap_rs::dap::DapLeds;
use embedded_hal::digital::v2::OutputPin;
use rp2040_hal::gpio::{bank0::*, Pin, PushPullOutput};
use rtic_common::waker_registration::CriticalSectionWakerRegistration;

#[derive(Debug, Clone, Copy, PartialEq, defmt::Format)]
enum HostStatus {
    Connected(bool),
    Running(bool),
}

impl From<dap_rs::dap::HostStatus> for HostStatus {
    fn from(value: dap_rs::dap::HostStatus) -> Self {
        match value {
            dap_rs::dap::HostStatus::Connected(c) => Self::Connected(c),
            dap_rs::dap::HostStatus::Running(c) => Self::Running(c),
        }
    }
}

impl From<HostStatus> for NonZeroU8 {
    fn from(value: HostStatus) -> Self {
        let val = match value {
            HostStatus::Connected(false) => 1,
            HostStatus::Connected(true) => 2,
            HostStatus::Running(false) => 3,
            HostStatus::Running(true) => 4,
        };
        unsafe { NonZeroU8::new_unchecked(val) }
    }
}

impl From<NonZeroU8> for HostStatus {
    fn from(value: NonZeroU8) -> Self {
        match value.get() {
            1 => HostStatus::Connected(false),
            2 => HostStatus::Connected(true),
            3 => HostStatus::Running(false),
            4 => HostStatus::Running(true),
            _ => panic!("Invalid value"),
        }
    }
}

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

/// A struct that manages the onboard LEDs
///
/// Current color scheme:
/// * Host not connected
///     * No Vtarget detected: Red
///     * 1.8V Vtarget detected: Pink
///     * 3.3V Vtarget detected: White
/// * Host Connected: Yellow
/// * Not running: blue
/// * Running: green
///
/// After each state change a short delay is introduced to ensure that
/// the color can be noticed, unless it updates to a new state other than
/// Host not connected.
pub struct LedManager {
    leds: BoardLeds,
}

impl LedManager {
    pub fn new(leds: BoardLeds) -> Self {
        Self { leds }
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

        NonZeroU8::new(value).map(HostStatus::from)
    }

    pub fn set_host_status(host_status: Option<dap_rs::dap::HostStatus>) {
        let host_status: Option<HostStatus> = host_status.map(Into::into);
        let value: u8 = host_status
            .map(NonZeroU8::from)
            .map(NonZeroU8::get)
            .unwrap_or(0);

        Self::host_status_storage().store(value, Ordering::Relaxed);

        defmt::debug!("Host status is now {}", host_status);

        // Updating the host status is always a state transition,
        // so we want to wake the waker.
        Self::waker().wake();
    }

    pub fn host_status_token(&mut self) -> HostStatusToken {
        HostStatusToken {
            _phantom: Default::default(),
        }
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

        Self::vtarget_storage().store(new_value, Ordering::Relaxed);

        Self::waker().wake();
    }

    fn set(&mut self) -> bool {
        let current_vtarget = Self::current_vtarget();
        let current_host_status = Self::current_host_status();

        let is_activity = match (current_vtarget, current_host_status) {
            (Some(current_vtarget), None)
            | (Some(current_vtarget), Some(HostStatus::Connected(false))) => {
                match current_vtarget {
                    Vtarget::Voltage1V8 => self.leds.pink(),
                    Vtarget::Voltage3V3 => self.leds.white(),
                }
                false
            }
            (_, Some(HostStatus::Connected(true))) => {
                self.leds.yellow();
                true
            }
            (_, Some(HostStatus::Running(false))) => {
                self.leds.blue();
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
            use futures_util::{future::select, pin_mut};

            // Wait for a little bit so the status can actually be seen, unless
            // it changes to another non-idle state.
            use rtic_monotonics::rp2040::{ExtU64, Timer};

            let wait_for_nonactive_status = async {
                loop {
                    let (new_status, _) = self.wait_for_change().await;
                    if new_status != Some(HostStatus::Connected(false))
                        && new_status != Some(HostStatus::Running(false))
                    {
                        break;
                    }
                }
            };
            pin_mut!(wait_for_nonactive_status);

            let timeout = Timer::delay(100.millis());
            pin_mut!(timeout);

            select(wait_for_nonactive_status, timeout).await;
        } else {
            let _ = self.wait_for_change().await;
        }
    }

    #[must_use]
    async fn wait_for_change(&mut self) -> (Option<HostStatus>, Option<Vtarget>) {
        let current_vtarget = Self::current_vtarget();
        let current_host_status = Self::current_host_status();

        // Wait for an update to occur
        core::future::poll_fn(|ctx| {
            Self::waker().register(ctx.waker());

            let new_vtarget = Self::current_vtarget();
            let new_host_status = Self::current_host_status();

            if new_vtarget != current_vtarget {
                // A change in Vtarget means that a cable was re- or disconnected, in
                // which case knowing the core status is impossible, so we just reset
                // it to the default.
                Self::set_host_status(None);

                Poll::Ready((new_host_status, new_vtarget))
            } else if new_host_status != current_host_status {
                Poll::Ready((new_host_status, new_vtarget))
            } else {
                Poll::Pending
            }
        })
        .await
    }

    pub async fn run(&mut self) -> ! {
        loop {
            // Set the LEDs to whatever the current state is.
            self.update().await;
        }
    }
}

/// A token that can be used to update the DAP Host
/// status.
#[derive(Debug, Clone, Copy)]
pub struct HostStatusToken {
    _phantom: PhantomData<()>,
}

impl DapLeds for HostStatusToken {
    fn react_to_host_status(&mut self, host_status: dap_rs::dap::HostStatus) {
        LedManager::set_host_status(Some(host_status));
    }
}
