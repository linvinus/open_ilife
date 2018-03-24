#!/bin/sh
arm-none-eabi-gdb build/ch.elf -ex 'target extended-remote chuwi:3333'
#arm-none-eabi-gdb build/ch.elf -ex 'target extended-remote R4:3333'
