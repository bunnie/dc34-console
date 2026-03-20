#![no_std]
#![no_main]

use core::arch::asm;

mod panic_handler {
    use core::panic::PanicInfo;
    #[panic_handler]
    fn handle_panic(_arg: &PanicInfo) -> ! { loop {} }
}

#[inline(always)]
pub fn read_fifo0() -> u32 {
    let value: u32;
    unsafe {
        core::arch::asm!(
            "mv {}, x15",
            out(reg) value,
        );
    }
    value
}

#[inline(always)]
pub fn set_gpio(value: u32) {
    unsafe {
        core::arch::asm!(
            "mv x14, {}",
            in(reg) value,
        );
    }
}

#[unsafe(export_name = "rust_entry")]
pub unsafe extern "C" fn rust_entry() -> ! {
    let mut foo = 0;
    loop {
        let arg = read_fifo0();
        foo += arg;
        set_gpio(foo);
    }
}
