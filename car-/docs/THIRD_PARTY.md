# Third-party notices

## ST VL53L1X ULD API

Files:

- `Core/Inc/vl53l1x/VL53L1X_api.h`
- `Core/Src/vl53l1x/VL53L1X_api.c`
- `Core/Inc/vl53l1x/vl53l1_platform.h`
- `Core/Src/vl53l1x/vl53l1_platform.c`

The VL53L1X core is dual-licensed by STMicroelectronics under its proprietary
license and the BSD 3-clause "New" or "Revised" License. This project uses the
BSD 3-clause option. The original file headers are retained.

The STM32 platform layer and initialization example were adapted from:

- <https://github.com/maudeve-it/VL53L1X-STM32> (MIT License, Copyright (c) 2023 Mauro De Vecchi)

The files were modified for:

- shared I2C2 bus with two VL53L1X devices, address passed per call;
- XSHUT/address assignment moved to `tof400.c`;
- timeout added to the ST `VL53L1X_SensorInit()` and temperature-update wait loops;
- no blocking forever when a sensor is absent.

## SSD1306 font

`Core/Inc/ssd1306_font.h` is a standard 5×7 ASCII bitmap font table for the
characters 32–90, used for the OLED status display.
