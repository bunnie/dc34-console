use std::sync::{
    Arc,
    atomic::{AtomicBool, Ordering},
};

use bao1x_api::{IoIrq, IoxHal, IoxValue};
use bao1x_hal::lis2dh12::{Lis2dh12, Orientation, regs};
use bao1x_hal::{axp2101::VbusIrq, i2c::I2c};
use chrono::Utc;
use dc34_api::*;
use num_traits::ToPrimitive;

/*
Power manager policy notes

Goals:
- aggressively enter wfi sleep - screen inactive, lights running
  - wake from wfi sleep should be keypress only
  - wfi sleep control time is set by application
- after a longer period of time, go into deep sleep
  - no lights running
  - wake by accelerometer interrupt or keypress
  - deep sleep control time is set by power manager
*/

const POWER_POLL_INTERVAL_MS: usize = 2500;
// this gives some margin for the keypress to "catch up" in case both motion and
// keypress interrupts are simultaneously fired as a wakeup event
const MOTION_IRQ_MARGIN_MS: u64 = 1000;
const WFI_IDLE_SEC_INIT: u64 = 60;
const WFI_MIN_SEC: usize = 5;
const DEEP_SLEEP_SEC: i64 = 5 * 60; // 15 * 60; // 15 minutes of total quiescence in deployment

fn setup_accel(accel: &mut Lis2dh12, i2c: &mut I2c) -> Result<(), xous::Error> {
    let saved_ctrl3 = accel.read_register(i2c, regs::CTRL_REG3)?;

    // Latch both INT1 (motion) and INT2 (orientation) until their SRC regs are read
    // 0x08 = LIR_INT1, 0x02 = LIR_INT2
    accel.write_register(i2c, regs::CTRL_REG5, 0x08 | 0x02)?;

    // -- INT1: motion detection -----------------------------------------------
    // OR combination, all axes high/low enabled
    accel.write_register(i2c, regs::INT1_CFG, 0x7F)?;

    // -- sensitivity tuning for wake ------------------------------------------
    /* // original tuning - fairly sensitive to taps, but misses longer, slower motions
    // Threshold: 16mg/LSB at ±2g - low enough to catch gentle movement
    accel.write_register(i2c, regs::INT1_THS, 10)?;
    // Minimum duration before interrupt fires
    accel.write_register(i2c, regs::INT1_DURATION, 1)?;
    */

    // --- newer tuning ---
    // At 25Hz, DURATION=2 → 80ms minimum - should be better at rejecting
    // brief non-walking transients without missing steps
    // CTRL_REG1: 25Hz, normal mode, XYZ enabled
    // [7:4]=0011 (25Hz), [2:0]=111
    accel.write_register(i2c, regs::CTRL_REG1, 0x37)?;
    accel.write_register(i2c, regs::INT1_THS, 20)?;
    accel.write_register(i2c, regs::INT1_DURATION, 2)?;
    /*
    Tuning loop:
      Start at THS=20, DUR=2
      - If breathing triggers → raise THS to 22, not duration
      - If slow walking misses → lower THS to 18 first, then DUR
      - If chair creaks/fabric rustle triggers it → raise DUR to 4–5, not THS
    Threshold controls amplitude sensitivity, duration controls how sustained the motion
    needs to be.
    */
    // -- end sensitivity tuning for wake --------------------------------------

    // -- INT2: 6D orientation detection ---------------------------------------
    // AOI=0, 6D=1, all six directions enabled - fires on any face change
    accel.write_register(i2c, regs::INT2_CFG, 0x7F)?;
    // ~500mg threshold - high enough to stay stable while wearing, low enough
    // to detect a deliberate flip. Tune between 0x10 (250mg) and 0x30 (750mg).
    accel.write_register(i2c, regs::INT2_THS, 0x20)?;
    // Small debounce: at 100Hz, value=2 → 20ms minimum hold before triggering.
    // Prevents a mid-flip transient from double-firing.
    accel.write_register(i2c, regs::INT2_DURATION, 2)?;

    // -- Shared config ---------------------------------------------------------
    accel.set_interrupt_polarity(i2c, bao1x_hal::lis2dh12::InterruptPolarity::ActiveHigh)?;

    // CTRL_REG2: HPF enabled for INT1 only (HPEN1=1, HPEN2=0).
    // INT2/6D intentionally gets raw gravity - that's its reference signal.
    accel.write_register(i2c, regs::CTRL_REG2, 0b00_00_0001)?;
    // Read REFERENCE to reset the HPF and zero it against current orientation
    accel.read_register(i2c, regs::REFERENCE)?;

    // CTRL_REG3: route both IA1 (motion) and IA2 (orientation) to INT1 pin
    // bit6=I1_IA1, bit5=I1_IA2
    accel.write_register(i2c, regs::CTRL_REG3, saved_ctrl3 | 0x40 | 0x20)?;

    // Clear any pending interrupts on both engines
    let _ = accel.read_register(i2c, regs::INT1_SRC)?;
    let _ = accel.read_register(i2c, regs::INT2_SRC)?;

    Ok(())
}

fn accel_enable_int(accel: &mut Lis2dh12, i2c: &mut I2c, enable: bool) -> Result<(), xous::Error> {
    // Clear any pending interrupts on both engines
    let _ = accel.read_register(i2c, regs::INT1_SRC)?;
    let _ = accel.read_register(i2c, regs::INT2_SRC)?;
    let saved_ctrl3 = accel.read_register(i2c, regs::CTRL_REG3)?;
    if enable {
        log::info!("enable accel");
        // bit6=I1_IA1 (motion), bit5=I1_IA2 (orientation)
        accel.write_register(i2c, regs::CTRL_REG3, saved_ctrl3 | 0x40 | 0x20)?;
        accel.reset_highpass(i2c)?;
    } else {
        // always allow orientation interrupts; disable motion
        log::info!("disable accel");
        accel.write_register(i2c, regs::CTRL_REG3, saved_ctrl3 & !(0x40/* | 0x20 */))?;
    }
    Ok(())
}

pub fn power_manager(run_led_fade: Arc<AtomicBool>, plugged_in: Arc<AtomicBool>) -> ! {
    let xns = xous_names::XousNames::new().unwrap();
    let tt = ticktimer::Ticktimer::new().unwrap();

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
    let dummy = adc.read_raw(bao1x_hal::udma::AdcSource::Ext(bao1x_hal::udma::AdcExtChannel::Adc3), Some(8));
    log::info!("ADC pipe-clearing value {}", dummy);

    let mut i2c = I2c::new();
    // Initialize
    let mut accel = Lis2dh12::new(&mut i2c).ok();
    if let Some(a) = &mut accel {
        setup_accel(a, &mut i2c).unwrap();
    } else {
        log::warn!("No accelerometer found!");
    }

    let susres = susres::Susres::new_without_hook(&xns).unwrap();
    let gfx = ux_api::service::gfx::Gfx::new(&xns).unwrap();

    let iox_hal = IoxHal::new();
    let sid = xns.register_name(POWER_MANAGER_SERVER, None).unwrap();

    let kbd = bao1x_api::keyboard::Keyboard::new(&xns).unwrap();
    kbd.register_listener(POWER_MANAGER_SERVER, PowerManagerOp::KeyPress.to_u32().unwrap() as usize);

    let rtc = bao1x_hal_service::Rtc::new();
    let rovers = rtc.set_wakeup(Utc::now() + chrono::Duration::seconds(DEEP_SLEEP_SEC)).unwrap_or(0);
    if rovers > 1 {
        log::warn!("Rollover case for RTC not handled, wakeup will fail. rovers: {}", rovers);
    }
    let mut alarm_set = true;

    let mut orientation = Orientation::FaceUp;
    if let Some(a) = accel.as_mut() {
        let _ = iox_hal.set_irq_pin(
            bao1x_api::IoxPort::PC,
            15,
            bao1x_api::IoxValue::Low,
            POWER_MANAGER_SERVER,
            PowerManagerOp::MotionIrq.to_usize().unwrap(),
        );
        log::info!("Accelerometer interrupt pin setup");
        if let Ok(o) = a.get_orientation(&mut i2c) {
            orientation = o;
        }
    }
    let vbus_io = (bao1x_api::IoxPort::PA, 4u8);
    let mut vbus_state = iox.get_gpio_pin_value(vbus_io.0, vbus_io.1);
    plugged_in.store(vbus_state == IoxValue::High, Ordering::SeqCst);
    let vbus_irq_index = iox_hal
        .set_irq_pin(
            vbus_io.0,
            vbus_io.1,
            !vbus_state,
            POWER_MANAGER_SERVER,
            PowerManagerOp::VbusIrq.to_usize().unwrap(),
        )
        .expect("Couldn't claim Vbus IRQ");

    let cid = xous::connect(sid).unwrap();
    std::thread::spawn({
        let cid = cid;
        move || {
            let tt = ticktimer::Ticktimer::new().unwrap();
            loop {
                tt.sleep_ms(POWER_POLL_INTERVAL_MS).ok();
                xous::try_send_message(
                    cid,
                    xous::Message::new_scalar(PowerManagerOp::Poll.to_usize().unwrap(), 0, 0, 0, 0),
                )
                .ok();
            }
        }
    });

    let usb = usb_bao1x::UsbHid::new();

    let mut pwr_mgr_enabled = false;
    let mut wfi_awaiting_keypress = false;
    let mut idle_sec = WFI_IDLE_SEC_INIT;

    let mut last_action_time_ms = tt.elapsed_ms();
    let mut msg_opt = None;
    loop {
        xous::reply_and_receive_next(sid, &mut msg_opt).unwrap();
        let opcode = {
            let msg = msg_opt.as_mut().unwrap();
            num_traits::FromPrimitive::from_usize(msg.body.id()).unwrap_or(PowerManagerOp::Invalid)
        };
        log::debug!("{:?}", opcode);
        match opcode {
            PowerManagerOp::Enable => {
                if let Some(scalar) = msg_opt.as_mut().unwrap().body.scalar_message_mut() {
                    if scalar.arg1 != 0 {
                        pwr_mgr_enabled = true;
                    } else {
                        pwr_mgr_enabled = false;
                    }
                    if scalar.arg2 > WFI_MIN_SEC {
                        idle_sec = scalar.arg2 as u64;
                    }
                }
            }
            PowerManagerOp::Poll => {
                let now_ms = tt.elapsed_ms();
                // disable power management if VBUS is plugged in
                if !pwr_mgr_enabled || vbus_state == IoxValue::High {
                    // this effectively disables the if statement below by claiming
                    // an action has *always* happened
                    last_action_time_ms = now_ms;

                    if alarm_set {
                        // clear the RTC alarm. The alarm_set flag just makes things a little
                        // more efficient so we're not redundantly clearing the alarm every
                        // poll
                        rtc.clear_wakeup();
                        alarm_set = false;
                    }
                }
                if !wfi_awaiting_keypress && (now_ms - last_action_time_ms > idle_sec * 1000)
                    || wfi_awaiting_keypress && (now_ms - last_action_time_ms > MOTION_IRQ_MARGIN_MS)
                {
                    gfx.set_power(false).unwrap();
                    wfi_awaiting_keypress = true; // this tells the KeyPress handler we have to turn on the screen

                    susres.initiate_suspend().unwrap();
                    // we idled, until a button was pressed

                    // brief delay for everything to catch up
                    tt.sleep_ms(100).ok();
                    last_action_time_ms = now_ms;

                    // screen wake-up is delegated to KeyPress handler -
                    // this prevents the screen glitch on RTC wake event
                }
            }
            PowerManagerOp::MotionIrq => {
                if let Some(a) = &mut accel {
                    let int1_src = a.read_int1_source(&mut i2c).unwrap();
                    // must read to clear any pending interrupt
                    let _int2_src = a.read_int2_source(&mut i2c).unwrap();
                    if int1_src.active {
                        /* log::info!("Motion confirmed! {:?}", a.read_accel_mg(&mut i2c).unwrap()); */
                        a.reset_highpass(&mut i2c).unwrap();
                        // only enable deep sleep if we're on battery power
                        if vbus_state == IoxValue::Low {
                            // this pushes the alarm date out by the deep sleep time horizon
                            let rovers = rtc
                                .set_wakeup(Utc::now() + chrono::Duration::seconds(DEEP_SLEEP_SEC))
                                .unwrap_or(0);
                            if rovers > 1 {
                                log::warn!(
                                    "Rollover case for RTC not handled, wakeup will fail. rovers: {}",
                                    rovers
                                );
                            }
                            alarm_set = true;
                        }
                    }
                    // always check this in both interrupt cases
                    if let Ok(o) = a.get_orientation(&mut i2c) {
                        if orientation != o && o != Orientation::Unknown {
                            log::info!("New orientation: {:?}", o);
                            orientation = o;
                            gfx.flip_screen(o == Orientation::FaceDown).ok();
                            kbd.flip_orientation(o == Orientation::FaceDown);
                            kbd.inject_key('🔁');
                        }
                    }
                }
            }
            PowerManagerOp::VbusIrq => {
                // check the current value, because we can have chatter after interrupts
                vbus_state = iox.get_gpio_pin_value(vbus_io.0, vbus_io.1);
                plugged_in.store(vbus_state == IoxValue::High, Ordering::SeqCst);
                // flip the edge trigger to opposite the current state
                iox_hal.update_irq_pin(POWER_MANAGER_SERVER, vbus_irq_index, Some(!vbus_state), None);

                // notify the USB stack of state changes
                xous::send_message(
                    usb.cid(),
                    xous::Message::new_blocking_scalar(
                        usb_bao1x::api::Opcode::PmicIrq.to_usize().unwrap(),
                        (if vbus_state == IoxValue::High { VbusIrq::Insert } else { VbusIrq::Remove }).into(),
                        0,
                        0,
                        0,
                    ),
                )
                .ok();
            }
            PowerManagerOp::KeyPress => {
                if let Some(scalar) = msg_opt.as_ref().unwrap().body.scalar_message() {
                    let k = char::from_u32(scalar.arg1 as u32).unwrap_or('\u{0000}');
                    if k == '⏰' {
                        log::info!("Deep sleep trigger hit!");

                        // ensure accelerometer interrupts are enabled, that's the primary source of waking
                        if let Some(a) = &mut accel {
                            accel_enable_int(a, &mut i2c, true).unwrap();
                        }
                        // turn off screen
                        gfx.set_power(false).unwrap();

                        // TODO: wrap this in something more ergonomic
                        let conn = xns
                            .request_connection_blocking(susres::api::SERVER_NAME_SUSRES)
                            .expect("Can't connect to SUSRES");
                        match xous::send_message(
                            conn,
                            xous::Message::new_blocking_scalar(
                                susres::api::Opcode::PlatformSpecific.to_usize().unwrap(),
                                bao1x_hal_service::api::ClockOp::DeepSleep.to_usize().unwrap(),
                                0,
                                0,
                                0,
                            ),
                        ) {
                            Ok(xous::Result::Scalar1(result)) => {
                                if result == 1 {
                                    log::info!("Should be in deep sleep!");
                                } else {
                                    log::error!("Couldn't initiate deep sleep")
                                }
                            }
                            _ => panic!("Couldn't send deep sleep message to susres"),
                        }
                        // -- Execution should have diverged here - system is off --
                    } else {
                        // delegating this to here prevents the screen from glitching on
                        // during wakeup due to RTC event
                        if wfi_awaiting_keypress {
                            // turn on screen
                            gfx.set_power(true).unwrap();
                            wfi_awaiting_keypress = false;
                        }

                        // only enable deep sleep if we're on battery power
                        if vbus_state == IoxValue::Low {
                            // this pushes the alarm date out by the deep sleep time horizon
                            let rovers = rtc
                                .set_wakeup(Utc::now() + chrono::Duration::seconds(DEEP_SLEEP_SEC))
                                .unwrap_or(0);
                            if rovers > 1 {
                                log::warn!(
                                    "Rollover case for RTC not handled, wakeup will fail. rovers: {}",
                                    rovers
                                );
                            }
                            alarm_set = true;
                        }
                    }
                    if k == '🔁' {
                        log::info!("tilt keep-on");
                    }
                }
                last_action_time_ms = tt.elapsed_ms();
            }
            PowerManagerOp::SetFadeMode => {
                if let Some(scalar) = msg_opt.as_ref().unwrap().body.scalar_message() {
                    // only do the fading effect if we're on battery (it's a power saving feature)
                    if scalar.arg1 != 0 {
                        run_led_fade.store(true, Ordering::SeqCst);
                    } else {
                        run_led_fade.store(false, Ordering::SeqCst);
                    }
                }
            }
            PowerManagerOp::GetAccelId => {
                if let Some(scalar) = msg_opt.as_mut().unwrap().body.scalar_message_mut() {
                    if let Some(a) = accel.as_ref() {
                        scalar.arg1 = 1;
                        let id = a.read_who_am_i(&mut i2c).unwrap_or(0);
                        scalar.arg2 = id as usize;
                    } else {
                        scalar.arg1 = 0;
                    }
                }
            }
            PowerManagerOp::GetVbat => {
                if let Some(scalar) = msg_opt.as_mut().unwrap().body.scalar_message_mut() {
                    let vbat_raw = adc.read_raw(
                        bao1x_hal::udma::AdcSource::Ext(bao1x_hal::udma::AdcExtChannel::Adc3),
                        Some(8),
                    );
                    let vbat_mv = (bao1x_hal::udma::Adc::raw_to_voltage(vbat_raw) * 1000.0f32) as usize;
                    scalar.arg1 = 1;
                    scalar.arg2 = vbat_mv;
                }
            }
            PowerManagerOp::GetVbus => {
                if let Some(scalar) = msg_opt.as_mut().unwrap().body.scalar_message_mut() {
                    scalar.arg1 = 1;
                    scalar.arg2 =
                        if iox.get_gpio_pin_value(bao1x_api::IoxPort::PA, 4) == bao1x_api::IoxValue::High {
                            1
                        } else {
                            0
                        };
                }
            }
            // system has powered on, enable interrupts & management
            PowerManagerOp::Boot => {
                if let Some(_scalar) = msg_opt.as_mut().unwrap().body.scalar_message_mut() {
                    pwr_mgr_enabled = true;
                    // set initial orientation
                    if let Some(a) = accel.as_mut() {
                        if let Ok(o) = a.get_orientation(&mut i2c) {
                            log::info!("Initial orientation: {:?}", o);
                            orientation = o;
                            gfx.flip_screen(o == Orientation::FaceDown).ok();
                            kbd.flip_orientation(o == Orientation::FaceDown);
                        }

                        // enable the interrupts on boot
                        accel_enable_int(a, &mut i2c, true).unwrap();
                    }
                }
            }
            PowerManagerOp::Invalid => {
                log::error!("Invalid power manager operation: {:?}", opcode);
            }
        }
    }
}
