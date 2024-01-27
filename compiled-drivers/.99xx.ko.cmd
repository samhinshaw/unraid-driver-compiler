cmd_/drivers/Linux/99xx.ko := ld -r -m elf_x86_64 -z noexecstack --build-id=sha1  -T scripts/module.lds -o /drivers/Linux/99xx.ko /drivers/Linux/99xx.o /drivers/Linux/99xx.mod.o;  true
