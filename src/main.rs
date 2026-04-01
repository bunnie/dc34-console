use std::sync::{
    Arc,
    atomic::{AtomicBool, Ordering},
};
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

    tt.sleep_ms(500).ok();
    leds::start_leds();

    let run_led_fade = Arc::new(AtomicBool::new(false));

    std::thread::spawn({
        let run_led_fade = run_led_fade.clone();
        move || {
            let xns = xous_names::XousNames::new().unwrap();
            let gfx = ux_api::service::gfx::Gfx::new(&xns).unwrap();

            const MIN: u8 = 0;
            const MAX: u8 = bao1x_hal::sh1107::DEFAULT_BRIGHTNESS;
            // Number of steps across the full fade - tune this alongside the sleep duration
            // to control the overall fade speed.
            const STEPS: u8 = MAX;
            const INC: u8 = 2;
            // Gamma value: 2.2 is standard sRGB. Increase for a longer "dark" phase;
            // decrease toward 1.0 to flatten back toward linear.
            const GAMMA: f32 = 2.4;

            // Precompute the lookup table at startup rather than doing powf() every tick.
            let lut: Vec<u8> = (0..=STEPS)
                .map(|i| {
                    let t = i as f32 / STEPS as f32; // 0.0 ..= 1.0 linear
                    let corrected = t.powf(GAMMA); // apply gamma
                    (corrected * MAX as f32).round() as u8 // scale to hardware range
                })
                .collect();

            let mut up = false;
            let mut t: u8 = 0; // linear animation parameter, 0 ..= STEPS

            let mut was_fading = run_led_fade.load(Ordering::SeqCst);
            loop {
                let do_fade = run_led_fade.load(Ordering::SeqCst);
                if do_fade {
                    std::thread::sleep(std::time::Duration::from_millis(80));

                    if up {
                        t = t.saturating_add(INC).min(STEPS);
                        if t == STEPS {
                            up = false;
                        }
                    } else {
                        t = t.saturating_sub(INC).max(MIN);
                        if t == MIN {
                            up = true;
                        }
                    }

                    gfx.brightness(lut[t as usize]).unwrap();
                } else {
                    if was_fading {
                        gfx.brightness(MAX).unwrap();
                        t = MAX;
                    }
                    std::thread::sleep(std::time::Duration::from_millis(500));
                }
                was_fading = do_fade;
            }
        }
    });

    power::power_manager(run_led_fade);
}
