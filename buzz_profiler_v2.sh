#!/bin/bash

# https://poormansprofiler.org/

board=esp32s3empty
cpu_id=cpu0
nsamples=1000
run_id=v27
sleeptime=1

elf=./build/${board}/esp-idf_build/ardupilot.elf
gdbdev=`ls /dev/cu.*`
gdb=xtensa-esp32s3-elf-gdb

stacksfile=./${board}_${run_id}_${cpu_id}.stacks.log
gdberrfile=./${board}_${run_id}_${cpu_id}.gdberr.log

# run gdb in background so we can send it signals later;
# this specificially does NOT reset the device or halt it,
# we want to keep the app running while probing.

# disconnect any existing gdb session
pkill -SIGUSR1 -f xtensa-esp-elf-gdb
sleep 5

for x in $(seq 1 $nsamples)
  do
    $gdb $elf --nx --quiet --batch \
      -ex "target remote :3333" \
      -ex "set confirm off" \
      -ex "handle SIGINT stop print nopass" \
      -ex "catch signal SIGUSR1" \
      -ex "set print pretty" \
      -ex "set pagination 0" \
      -ex "set print asm-demangle on" \
      -ex "set remote hardware-watchpoint-limit 2" \
      -ex "set print thread-events off" \
      -ex "thread apply all bt" \
      -ex "c" \
      2> ${gdberrfile} \
      >> ${stacksfile} &

    sleep $sleeptime

    # SIGUSR1 terminates GDB and leaves the app running
    pkill -SIGUSR1 -f xtensa-esp-elf-gdb

    echo -e '\n\n' >> $stacksfile
    echo -ne "\r$x/$nsamples"
  done
