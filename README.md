# IMU Kalman Filter

## Build

[Setup environment for ESP32 development](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/index.html#installation)

In new terminal, set up environment variables:

```bash
. $HOME/esp/esp-idf/export.sh
```

Configure project:

```bash
cd imu-kalman
idf.py set-target esp32
idf.py menuconfig
```

Build

```bash
idf.py build
```

Flash

First, determine serial port:

```bash
ls /dev/tty* # linux
ls /dev/cu* # mac
```

Then flash:

```bash
idf.py -p PORT [-b BAUD] flash
```
