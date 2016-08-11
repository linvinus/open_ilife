#!/bin/sh
arm-none-eabi-gdb build/ch.elf -ex 'target extended-remote localhost:3333'
