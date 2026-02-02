use std::sync::Arc;
use std::sync::atomic::{AtomicU8, Ordering};
use std::time::Duration;

use bao1x_api::{IoIrq, IoxHal};
use bao1x_hal::i2c::I2c;
use bao1x_hal::lis2dh12::{Lis2dh12, Orientation, regs};
use num_traits::ToPrimitive;
use utralib::generated::utra;
/*
pub fn start_power_management() {
    std::thread::spawn(move || {
        power_manager();
    });
}
*/

const POWER_POLL_INTERVAL_MS: usize = 2500;
const WFI_IDLE_SEC: u64 = 10;

pub const POWER_MANAGER_SERVER: &'static str = "_phx_pwr_mgr_";

#[derive(Debug, Copy, Clone, num_derive::FromPrimitive, num_derive::ToPrimitive)]
pub enum PowerManagerOp {
    Poll,
    MotionIrq,
    KeyPress,
    Invalid,
}

fn setup_accel(accel: &mut Lis2dh12, i2c: &mut I2c) -> Result<(), xous::Error> {
    let saved_ctrl3 = accel.read_register(i2c, regs::CTRL_REG3)?;

    accel.write_register(i2c, regs::CTRL_REG5, 0x08)?;

    // INT1_CFG: OR combination, all axes high/low enabled
    accel.write_register(i2c, regs::INT1_CFG, 0x7F)?;
    // INT1_THS: threshold 16mg/LSB at ±2g
    accel.write_register(i2c, regs::INT1_THS, 10)?;
    // INT1_DURATION: 0 (no minimum duration)
    accel.write_register(i2c, regs::INT1_DURATION, 0)?;
    // CTRL_REG3: Enable IA1 on INT1 pin
    accel.write_register(i2c, regs::CTRL_REG3, saved_ctrl3 | 0x40)?;

    accel.write_register(i2c, regs::CTRL_REG2, 0b00_00_0001)?;
    accel.read_register(i2c, regs::REFERENCE)?;

    // Wait a bit for new samples
    std::thread::sleep(Duration::from_millis(150));

    // Clear any pending interrupt by reading INT1_SRC
    let _ = accel.read_register(i2c, regs::INT1_SRC)?;

    // Check if interrupt is active
    let src = accel.read_register(i2c, regs::INT1_SRC)?;
    let triggered = (src & 0x40) != 0;

    log::info!("debug_test_int1_simple: INT1_SRC = 0x{:02X}, triggered = {}", src, triggered);

    Ok(())
}

pub fn power_manager(led_value: Arc<AtomicU8>) -> ! {
    let xns = xous_names::XousNames::new().unwrap();
    let tt = ticktimer::Ticktimer::new().unwrap();

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
        iox_hal.set_irq_pin(
            bao1x_api::IoxPort::PC,
            15,
            bao1x_api::IoxValue::Low,
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

    let mut test_state = true;
    let mut display_on = true;

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
            PowerManagerOp::Poll => {
                let now_ms = tt.elapsed_ms();
                if now_ms - last_action_time_ms > WFI_IDLE_SEC * 1000 {
                    if test_state {
                        led_value.store(13, Ordering::SeqCst);
                    } else {
                        led_value.store(0, Ordering::SeqCst);
                    }
                    test_state = !test_state;

                    if display_on {
                        gfx.set_power(false).unwrap();
                        display_on = false;
                    }
                    /*
                    gfx.set_power(false).unwrap();
                    susres.initiate_suspend().unwrap();
                    // we idled, until a button was pressed
                    tt.sleep_ms(100).ok();
                    gfx.set_power(true).unwrap();
                    */
                }
            }
            PowerManagerOp::MotionIrq => {
                if let Some(a) = &mut accel {
                    last_action_time_ms = tt.elapsed_ms();
                    let source = a.read_int1_source(&mut i2c).unwrap();
                    if source.active {
                        if !display_on {
                            gfx.set_power(true);
                            display_on = true;
                        }
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
            PowerManagerOp::Invalid => {
                log::error!("Invalid power manager operation: {:?}", opcode);
            }
        }
    }
}
