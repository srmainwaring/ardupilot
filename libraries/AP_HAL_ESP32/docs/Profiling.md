# Profiling

Guide to profiling ArduPilot code on the esp32.

## Links

- https://poormansprofiler.org/
- https://github.com/yoshinorim/quickstack


- https://github.com/brendangregg/FlameGraph
- https://docs.px4.io/main/en/debug/profiling.html
- https://review.openocd.org/c/openocd/+/8277/2/src/target/target.c



Open On-Chip Debugger
- https://openocd.org/
- https://sourceforge.net/p/openocd/code/ci/master/tree/doc/openocd.texi

```bash
brew install open-ocd
```

ESP32 S3 JTAG Debugging
- https://docs.espressif.com/projects/esp-idf/en/v5.3.1/esp32s3/api-guides/jtag-debugging/index.html#jtag-debugging


## Examples

### M5StampflyS3

- [wips/wip-esp32-stampfly+elrs+bmm150+gps](https://github.com/srmainwaring/ardupilot/tree/wips/wip-esp32-stampfly%2Belrs%2Bbmm150%2Bgps)

```bash
./waf configure --board esp32s3m5stampfly --debug
./waf copter
ESPBAUD=921600 ./waf copter --upload 
```

[Erase flash](https://docs.espressif.com/projects/esptool/en/latest/esp32s3/esptool/basic-commands.html)

```bash
esptool.py erase_flash
```

