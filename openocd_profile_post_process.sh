#!/bin/bash

board=esp32s3empty
cpu_id=cpu0
run_id=v27

elf=./build/${board}/esp-idf_build/ardupilot.elf
gprof_elf=./build/${board}/esp-idf_build/ardupilot_gprof.elf
gdb=xtensa-esp32s3-elf-gdb

profile_file=./${board}_${run_id}_${cpu_id}.flat.profile

cp ${elf} ${gprof_elf}
xtensa-esp32s3-elf-objcopy -I elf32-xtensa-le --rename-section .flash.text=.text ${gprof_elf}
xtensa-esp32s3-elf-gprof ${gprof_elf} gmon.out > ${profile_file}

