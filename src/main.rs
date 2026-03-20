use std::sync::Arc;
use std::sync::atomic::AtomicU8;

mod bio;
mod cmds;
mod repl;
mod shell;
use cmds::*;
// mod fxcore;
mod leds;
mod power;

// .\baosign.ps1 -Config baosec-lite -Target bunnie@10.0.245.164:code/testjig/images/

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

    // setup the VBUS/VBAT measurement pins
    let iox = bao1x_api::IoxHal::new();
    let adc = bao1x_hal_service::Adc::new();
    use bao1x_api::IoSetup;
    iox.setup_pin(
        bao1x_api::IoxPort::PA,
        4,
        Some(bao1x_api::IoxDir::Input),
        Some(bao1x_api::IoxFunction::Gpio),
        Some(bao1x_api::IoxEnable::Enable),
        Some(bao1x_api::IoxEnable::Disable),
        None,
        None,
    );
    // safety - we have manually checked there are no conflicts with this mapping
    unsafe { adc.enable_channel(bao1x_hal::udma::AdcExtChannel::Adc3) };

    let led_value: Arc<AtomicU8> = Arc::new(AtomicU8::new(13));
    tt.sleep_ms(500).ok();
    leds::start_leds(led_value.clone());

    power::power_manager(led_value.clone());
}
