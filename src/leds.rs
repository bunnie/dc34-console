use dc34_api::{BadgeType, LedManagerOp};

use crate::bio::lightgenes::Haploid;

pub fn start_leds() {
    std::thread::spawn(move || {
        leds();
    });
}

fn leds() {
    let tt = ticktimer::Ticktimer::new().unwrap();
    let xns = xous_names::XousNames::new().unwrap();

    let sid = xns.register_name(dc34_api::LED_SERVER, None).unwrap();

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
            LedManagerOp::Syngamy => {
                if let Some(scalar) = msg_opt.as_mut().unwrap().body.scalar_message_mut() {
                    if let Some(sperm) = Haploid::deserialize_u32(&[
                        scalar.arg1 as u32,
                        scalar.arg2 as u32,
                        scalar.arg3 as u32,
                        scalar.arg4 as u32,
                    ]) {
                        // 64 is a roughly 25% chance that any gene gets a single-bit flip mutation in grey
                        // code
                        //
                        // TODO: increase rate if coming from the same badge type to help increase
                        // diversity among inbred pools
                        lightgenes.syngamy(sperm, 64);
                        lightgenes.express();
                    } else {
                        log::warn!("Couldn't deserialize gene in call to Syngamy, ignoring")
                    }
                }
            }
            LedManagerOp::Force => {
                if let Some(scalar) = msg_opt.as_mut().unwrap().body.scalar_message_mut() {
                    if let Some(phenotype) = Haploid::deserialize_u32(&[
                        scalar.arg1 as u32,
                        scalar.arg2 as u32,
                        scalar.arg3 as u32,
                        scalar.arg4 as u32,
                    ]) {
                        lightgenes.force(phenotype);
                    } else {
                        log::warn!("Couldn't deserialize gene in call to Force, ignoring")
                    }
                }
            }
            LedManagerOp::GeneInit => {
                if let Some(scalar) = msg_opt.as_mut().unwrap().body.scalar_message_mut() {
                    let badge_type = BadgeType::try_from(scalar.arg1 as u8).unwrap_or(BadgeType::None);
                    lightgenes.gene.0 = [Haploid::from_type(&badge_type), Haploid::from_type(&badge_type)];
                    log::info!("Init to {:?}: gene {:?}", badge_type, lightgenes.gene);
                    lightgenes.express();
                }
            }
            LedManagerOp::Invalid => {
                log::error!("Invalid LED manager operation: {:?}", opcode);
            }
        };
    }
}
