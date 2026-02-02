use std::sync::{
    Arc,
    atomic::{AtomicU8, Ordering},
};

use bio_lib::ws2812::rgb_to_u32;

pub fn start_leds(led_value: Arc<AtomicU8>) {
    std::thread::spawn(move || {
        leds(led_value);
    });
}

fn leds(led_value: Arc<AtomicU8>) {
    let tt = ticktimer::Ticktimer::new().unwrap();

    let mut ws2812 =
        bio_lib::ws2812::Ws2812::new(bio_lib::ws2812::LedVariant::C, arbitrary_int::u5::new(15), None)
            .unwrap();

    let mut hues: Vec<f32> = Vec::new();
    for i in 0..8 {
        hues.push((i as f32 * 30.0) % 360.0);
    }
    let mut strip = vec![0u32; 8];

    loop {
        // convert and update values
        for (i, led) in hues.iter_mut().enumerate() {
            let value = led_value.load(Ordering::SeqCst) as f32 / 255.0;
            let (r, g, b) = hsv_to_rgb(*led, 0.8, value);
            // Pack into 24-bit RGB value
            let rgb_value: u32 = rgb_to_u32(r, g, b);
            strip[i] = rgb_value;
            *led += 1.0;
            if *led >= 360.0 {
                *led = 0.0;
            }
        }
        // send
        ws2812.send(&strip);
        tt.sleep_ms(10).ok();
    }
}

fn hsv_to_rgb(hue: f32, saturation: f32, value: f32) -> (u8, u8, u8) {
    let c = value * saturation;
    let h = hue / 60.0;
    let x = c * (1.0 - ((h % 2.0) - 1.0).abs());
    let m = value - c;

    let (r, g, b) = if h < 1.0 {
        (c, x, 0.0)
    } else if h < 2.0 {
        (x, c, 0.0)
    } else if h < 3.0 {
        (0.0, c, x)
    } else if h < 4.0 {
        (0.0, x, c)
    } else if h < 5.0 {
        (x, 0.0, c)
    } else {
        (c, 0.0, x)
    };

    (((r + m) * 255.0) as u8, ((g + m) * 255.0) as u8, ((b + m) * 255.0) as u8)
}
