#!/bin/sh
#sudo dd if=arch/mips/boot/uImage of=/dev/mmcblk0 seek=4194304 bs=1 conv=notrunc
sudo dd if=arch/mips/boot/uImage of=/dev/sdc seek=4194304 bs=1 conv=notrunc
sync
