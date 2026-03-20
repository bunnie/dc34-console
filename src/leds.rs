use std::{
    panic::UnwindSafe,
    sync::{
        Arc,
        atomic::{AtomicU8, Ordering},
    },
};

use bio_lib::ws2812::rgb_to_u32;

pub fn start_leds(led_value: Arc<AtomicU8>) {
    std::thread::spawn(move || {
        leds(led_value);
    });
}

fn leds(led_value: Arc<AtomicU8>) {
    let tt = ticktimer::Ticktimer::new().unwrap();

    let mut lightgenes =
        crate::bio::lightgenes::Lightgenes::new(arbitrary_int::u5::new(15), 10, 1, None).unwrap();

    lightgenes.run(None);

    // idle, so objects don't drop
    loop {
        tt.sleep_ms(10_000).ok();
    }
}
