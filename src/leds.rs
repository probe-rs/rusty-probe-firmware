use embedded_hal::digital::v2::{OutputPin, ToggleableOutputPin};
use rp2040_hal::gpio::{bank0::*, Pin, PushPullOutput};

pub struct BoardLeds {
    green: Pin<Gpio27, PushPullOutput>,
    red: Pin<Gpio28, PushPullOutput>,
    blue: Pin<Gpio29, PushPullOutput>,
}

impl BoardLeds {
    pub fn new(
        red: Pin<Gpio28, PushPullOutput>,
        green: Pin<Gpio27, PushPullOutput>,
        blue: Pin<Gpio29, PushPullOutput>,
    ) -> Self {
        let mut me = Self { red, green, blue };
        me.rgb(false, false, false);
        me
    }

    pub fn red(&mut self, level: bool) {
        self.red.set_state((!level).into()).ok();
    }

    pub fn toggle_red(&mut self) {
        self.red.toggle().ok();
    }

    pub fn green(&mut self, level: bool) {
        self.green.set_state((!level).into()).ok();
    }

    pub fn toggle_green(&mut self) {
        self.green.toggle().ok();
    }

    pub fn blue(&mut self, level: bool) {
        self.blue.set_state((!level).into()).ok();
    }
    pub fn toggle_blue(&mut self) {
        self.blue.toggle().ok();
    }

    pub fn rgb(&mut self, r: bool, g: bool, b: bool) {
        self.red(r);
        self.green(g);
        self.blue(b);
    }
}
