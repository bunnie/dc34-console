pub const LED_SERVER: &'static str = "_phx_led_";
#[derive(Debug, Copy, Clone, num_derive::FromPrimitive, num_derive::ToPrimitive)]
pub enum LedManagerOp {
    Autogamy,
    Invalid,
}

pub fn start_leds() {
    std::thread::spawn(move || {
        leds();
    });
}

fn leds() {
    let tt = ticktimer::Ticktimer::new().unwrap();
    let xns = xous_names::XousNames::new().unwrap();

    let sid = xns.register_name(LED_SERVER, None).unwrap();

    let mut lightgenes =
        crate::bio::lightgenes::Lightgenes::new(arbitrary_int::u5::new(15), 10, None).unwrap();

    let mut msg_opt = None;
    loop {
        xous::reply_and_receive_next(sid, &mut msg_opt).unwrap();
        let opcode = {
            let msg = msg_opt.as_mut().unwrap();
            num_traits::FromPrimitive::from_usize(msg.body.id()).unwrap_or(LedManagerOp::Invalid)
        };
        match opcode {
            LedManagerOp::Autogamy => {
                lightgenes.autogamy();
                lightgenes.express();
            }
            LedManagerOp::Invalid => {
                log::error!("Invalid LED manager operation: {:?}", opcode);
            }
        };
    }
}
