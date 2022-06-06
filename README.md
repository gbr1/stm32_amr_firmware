# stm32_amr_firmware

## How connect motor

```
left_front_joint (B) +-------+ right_front_joint (C)
                     | \   / |
                     |   ^   |
                     |   ^   |
                     | /   \ |
left_rear_joint (A)  +-------+ right_rear_joint (D)
```

## Info

You can read some data timing by checking these pins:

- `PA12`, motors controller 

- `PA8`, joints + imu + battery publishers

- `PB13`, data receiving and decoding

- `PB14`, message is not decoded

- `PA11`, joints references are decoded

<br>

Possible setups for imu:

- 2/4/8/16 g

- 250/500/1000/2000 °/s

---

## How to install the stm32duino core

1. Install Arduino IDE (tested on IDE 2 beta10 and IDE 1.8.13);
2. go into preferences and add `http://dan.drown.org/stm32duino/package_STM32duino_index.json` 
3. in board manager install stm32duino core for **STM32F1** boards
4. open terminal and type `$``cd /home/$USER/.arduino15/packages/stm32duino/tools/stm32tools/2021.5.31/linux/`
5. run `./install.sh`
6. close terminal and go back to Arduino IDE
7. select `tools -> board -> STM32F1 Boards (Arduino_STM32) -> "Generic STM32F103R series"`
8. select `tools -> variant -> STM32F103RC (48k RAM 256 Flash)`
9. select `tools -> cpu speed -> 72MHz`
10. select `tools -> optimize -> "smallest default"`
11. select `tools -> port -> /dev/ttyUSB0`

## How to upload

If you are on linux you need an STLINK v2 and you need to:<br>

1. select `tools -> upload method -> stlink`
2. press upload

If you are on macOS you can also upload just using the serial port.<br>

1. select `tools -> upload methos -> serial`
2. press upload

If you encounter issues, you can press **PC0** and **RESET** simultaneously, release only **RESET** and go in a sort of ***programming mode***.  You will see that onboard led fades.

Detach and reattach stlink usb cable and you should be able to load your firmware without any issues.

---

*NOTE: if you encounter some issues on uploading, try to select `tools -> variant -> STM32F103RE`*

*NOTE: it may necessary to erase memory of stm32 or load an empty program from serial if the board is bricked (a blink is provided in bin folder).*

*NOTE: is imu returns all zeros, reset the board*

---

> ***Copyright © 2022 G. Bruno under MIT license***