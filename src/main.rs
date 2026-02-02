use std::sync::Arc;
use std::sync::atomic::AtomicU8;

mod cmds;
mod repl;
mod shell;
use cmds::*;
mod fxcore;
mod leds;
mod power;

fn main() {
    log_server::init_wait().unwrap();
    log::set_max_level(log::LevelFilter::Info);
    log::info!("my PID is {}", xous::process::id());
    #[cfg(feature = "duart-debug-hal")]
    bao1x_hal::claim_duart();

    let tt = ticktimer::Ticktimer::new().unwrap();
    shell::start_shell();

    tt.sleep_ms(500).ok(); // pause for the system to startup
    let usb = usb_bao1x::UsbHid::new();
    usb.serial_console_input_injection();

    let led_value: Arc<AtomicU8> = Arc::new(AtomicU8::new(13));
    tt.sleep_ms(500).ok();
    leds::start_leds(led_value.clone());

    power::power_manager(led_value.clone());
}
