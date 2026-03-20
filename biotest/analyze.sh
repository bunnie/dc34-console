riscv-none-elf-nm -r --size-sort --print-size target/riscv32emc-unknown-none-elf/release/biotest | rustfilt > biotest.txt
riscv-none-elf-objdump -h target/riscv32emc-unknown-none-elf/release/biotest >> biotest.txt
riscv-none-elf-objdump -S -l -d target/riscv32emc-unknown-none-elf/release/biotest | rustfilt >> biotest.txt