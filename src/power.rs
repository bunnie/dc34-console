use std::sync::{
    Arc,
    atomic::{AtomicBool, Ordering},
};
use std::time::Duration;

use bao1x_api::{IoIrq, IoxHal};
use bao1x_hal::i2c::I2c;
use bao1x_hal::lis2dh12::{Lis2dh12, Orientation, regs};
use dc34_api::*;
use num_traits::ToPrimitive;

/*
pub fn start_power_management() {
    std::thread::spawn(move || {
        power_manager();
    });
}
*/

const POWER_POLL_INTERVAL_MS: usize = 2500;
const WFI_IDLE_SEC_INIT: u64 = 60;
const WFI_MIN_SEC: usize = 5;

fn setup_accel(accel: &mut Lis2dh12, i2c: &mut I2c) -> Result<(), xous::Error> {
    let saved_ctrl3 = accel.read_register(i2c, regs::CTRL_REG3)?;

    accel.write_register(i2c, regs::CTRL_REG5, 0x08)?;

    // INT1_CFG: OR combination, all axes high/low enabled
    accel.write_register(i2c, regs::INT1_CFG, 0x7F)?;
    // INT1_THS: threshold 16mg/LSB at ±2g
    accel.write_register(i2c, regs::INT1_THS, 10)?;
    // INT1_DURATION: 0 (no minimum duration)
    accel.write_register(i2c, regs::INT1_DURATION, 1)?;
    // set polarity
    accel.set_interrupt_polarity(i2c, bao1x_hal::lis2dh12::InterruptPolarity::ActiveHigh)?;
    // CTRL_REG3: Enable IA1 on INT1 pin
    accel.write_register(i2c, regs::CTRL_REG3, saved_ctrl3 | 0x40)?;

    accel.write_register(i2c, regs::CTRL_REG2, 0b00_00_0001)?;
    accel.read_register(i2c, regs::REFERENCE)?;

    // Wait a bit for new samples
    std::thread::sleep(Duration::from_millis(150));

    // Clear any pending interrupt by reading INT1_SRC
    let _ = accel.read_register(i2c, regs::INT1_SRC)?;

    // Check if interrupt is active
    // let src = accel.read_register(i2c, regs::INT1_SRC)?;
    // let triggered = (src & 0x40) != 0;
    // log::info!("debug_test_int1_simple: INT1_SRC = 0x{:02X}, triggered = {}", src, triggered);

    Ok(())
}

pub fn power_manager(run_led_fade: Arc<AtomicBool>) -> ! {
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
        log::warn!("No accelerometer found, disabling wakeup features");
    }

    let susres = susres::Susres::new_without_hook(&xns).unwrap();
    let gfx = ux_api::service::gfx::Gfx::new(&xns).unwrap();

    let iox_hal = IoxHal::new();
    let sid = xns.register_name(POWER_MANAGER_SERVER, None).unwrap();

    let kbd = bao1x_api::keyboard::Keyboard::new(&xns).unwrap();
    kbd.register_listener(POWER_MANAGER_SERVER, PowerManagerOp::KeyPress.to_u32().unwrap() as usize);

    if accel.is_some() {
        log::info!("Accelerometer interrupt enabled");
        iox_hal.set_irq_pin(
            bao1x_api::IoxPort::PC,
            15,
            bao1x_api::IoxValue::High,
            POWER_MANAGER_SERVER,
            PowerManagerOp::MotionIrq.to_usize().unwrap(),
        );
    }

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

    let mut enabled = true;
    let mut idle_sec = WFI_IDLE_SEC_INIT;
    // let mut display_on = true;

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
                        enabled = true;
                    } else {
                        enabled = false;
                    }
                    if scalar.arg2 > WFI_MIN_SEC {
                        idle_sec = scalar.arg2 as u64;
                    }
                }
            }
            PowerManagerOp::Poll => {
                let now_ms = tt.elapsed_ms();
                if !enabled {
                    // this effectively disables the if statement below by claiming
                    // an action has *always* happened
                    last_action_time_ms = now_ms;
                }
                if now_ms - last_action_time_ms > idle_sec * 1000 {
                    /*
                    if display_on {
                        gfx.set_power(false).unwrap();
                        display_on = false;
                    }
                    */

                    gfx.set_power(false).unwrap();

                    susres.initiate_suspend().unwrap();
                    // we idled, until a button was pressed
                    tt.sleep_ms(100).ok();
                    gfx.set_power(true).unwrap();
                    last_action_time_ms = now_ms;
                }
            }
            PowerManagerOp::MotionIrq => {
                if let Some(a) = &mut accel {
                    last_action_time_ms = tt.elapsed_ms();
                    let source = a.read_int1_source(&mut i2c).unwrap();
                    if source.active {
                        /*
                        if !display_on {
                            gfx.set_power(true);
                            display_on = true;
                        }
                        */
                        log::debug!(
                            "Motion confirmed! {:?} {:?}",
                            source,
                            a.read_accel_mg(&mut i2c).unwrap()
                        );
                        a.reset_highpass(&mut i2c).unwrap();
                    }
                }
            }
            PowerManagerOp::KeyPress => {
                last_action_time_ms = tt.elapsed_ms();
            }
            PowerManagerOp::SetFadeMode => {
                if let Some(scalar) = msg_opt.as_ref().unwrap().body.scalar_message() {
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
            PowerManagerOp::Invalid => {
                log::error!("Invalid power manager operation: {:?}", opcode);
            }
        }
    }
}
