use core::arch::asm;

use crate::platform;
// Assembly stubs for entering into the loader, and exiting it.

#[link_section = ".text.init"]
#[export_name = "_start"]
pub extern "C" fn _start() {
    unsafe {
        #[rustfmt::skip]
        asm! (
            // Place the stack pointer at the end of RAM
            "mv          sp, {ram_top}",
            // subtract four from sp to make room for a DMA "gutter"
            "addi        sp, sp, -4",

        "50:",
            // Install a machine mode trap handler
            "la          t0, abort",
            "csrw        mtvec, t0",

            // Start Rust
            "j   rust_entry",

            ram_top = in(reg) (platform::RAM_BASE + platform::RAM_SIZE),
            options(noreturn)
        );
    }
}
