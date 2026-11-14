# Medusa: sound effects synth for Eurorack

Medusa is a sound effects synth with four instruments: Dub Siren, Ray Gun, Lead Synth and Disco. It uses a Waveshare RP2040-Zero and an ST7789 colour screen. You can use it as a Eurorack module or as a standalone synth.

![Medusa](https://github.com/user-attachments/assets/238fe3b8-64d0-45a7-8392-5b8dcfc98ec9)

## Download the firmware

<p align="center">
  <a href="https://github.com/lonesoulsurfer/Medusa_Sound_Effects_Modular_Synth/releases/latest"><img src="https://img.shields.io/badge/Download-latest%20firmware%20(.uf2)-2ea44f?style=for-the-badge" alt="Download the latest firmware" /></a>
</p>

> **Warning:** Do not use **Code → Download ZIP** to get the firmware. That ZIP has the source code only. It does not have the `.uf2` file. Get the `.uf2` file from the [latest release](https://github.com/lonesoulsurfer/Medusa_Sound_Effects_Modular_Synth/releases/latest).

## Build guide and video

- Step-by-step build guide on Instructables: [Medusa - a Sound Effects Synth](https://www.instructables.com/Medusa-a-Sound-Effects-Synth-Eurorack-Module-or-St/)
- Video on YouTube: [Medusa](https://youtu.be/EX9xcyuSX-U)

## Features

- 4 instruments, each with 4 to 6 variations
- 4 knobs: pitch, speed, modulation and echo feedback
- 8 keys, to play notes as on a small keyboard
- Delay with 5 delay times
- Mutate: random changes to the sound on each sync trigger
- Special functions: 8-step sequencer, 5-second recorder, reverse, infinite delay, second LFO, 5-oscillator drone and sidechain
- Sync input

The [user manual](docs/MANUAL.md) tells you how to use all the functions.

## What you need

### Hardware

- All parts are in the [parts list](Parts%20List%20Medusa.pdf).
- The schematic is [Schematic.png](Schematic.png).
- The Gerber files for the PCB and the front panel are in [Gerber files](Gerber%20files/). You can send them to a PCB maker.

### Software (only if you compile the firmware yourself)

- Arduino IDE 2
- Board package: **Raspberry Pi Pico/RP2040** by Earle Philhower
- Libraries (install them with the Library Manager):
  - **Adafruit GFX Library**
  - **Adafruit ST7735 and ST7789 Library**

## Install the firmware

A `.uf2` file is the firmware, ready to use. You do not need the Arduino IDE to install it. You copy it to the RP2040-Zero, as you copy a file to a USB stick.

### Step 1: Download the file

1. Go to the [latest release](https://github.com/lonesoulsurfer/Medusa_Sound_Effects_Modular_Synth/releases/latest).
2. In the **Assets** list, click the `.uf2` file. Your browser downloads it.

### Step 2: Copy the file to the RP2040-Zero

Use method A if you can reach the **BOOT** button on the RP2040-Zero. Use method B if the board is inside a case.

#### Method A: with the BOOT button

1. Disconnect the USB cable from Medusa.
2. Push and hold the **BOOT** button on the RP2040-Zero.
3. While you hold the button, connect the USB cable to your computer.
4. Release the button. A drive with the name **RPI-RP2** opens on your computer.
5. Drag the `.uf2` file onto the **RPI-RP2** drive.
6. Wait about 5 seconds. The drive closes, the board restarts, and Medusa starts.

#### Method B: without the BOOT button (Windows)

This method uses a tool from the Arduino IDE. You must install the Arduino IDE and the **Raspberry Pi Pico/RP2040** board package first.

1. Connect Medusa to your computer with the USB cable.
2. Close the Arduino IDE.
3. Find the COM port of Medusa. Open **Device Manager** and look under **Ports (COM & LPT)**. An example is `COM5`.
4. Open the folder that has the `.uf2` file in File Explorer.
5. Click in the address bar. Type `powershell` and push **Enter**. A PowerShell window opens in that folder.
6. Paste these lines. Change `COM5` to your COM port, and change the file name to the name of your file.
7. Push **Enter**.

```powershell
$base = "$env:LOCALAPPDATA\Arduino15\packages\rp2040"
$py = (Get-ChildItem "$base\tools\pqt-python3" -Recurse -Filter "python3*.exe" | Select-Object -First 1).FullName
$tool = (Get-ChildItem "$base\hardware\rp2040" -Recurse -Filter uf2conv.py | Sort-Object FullName -Descending | Select-Object -First 1).FullName
& $py -I $tool --serial COM5 --family RP2040 --deploy ".\Medusa_V2-v2.0.0.uf2"
```

8. Wait about 10 seconds. Medusa restarts with the new firmware.

## Compile the firmware (for developers)

1. Install the software in [Software](#software-only-if-you-compile-the-firmware-yourself).
2. Open `Arduino IDE - Medusa/Medusa_V2/Medusa_V2.ino`.
3. In the **Tools** menu, set these values:
   - **Board:** Waveshare RP2040 Zero
   - **Flash Size:** 2MB (no FS)
   - **CPU Speed:** 200 MHz
   - **Optimize:** Small (-Os) (standard)
   - **USB Stack:** Pico SDK
   - **Port:** the COM port of Medusa
4. Connect the RP2040-Zero.
5. Click **Upload**.

## Files in this repository

| Folder or file | Contents |
|----------------|----------|
| `Arduino IDE - Medusa/Medusa_V2/` | Firmware |
| `Front Panel Design/` | KiCad, SVG and DXF files for the front panel |
| `Gerber files/` | Gerber files for the PCB and the front panel, ready to order |
| `Parts List Medusa.pdf` | Parts list |
| `Schematic.png` | Schematic |
| `docs/MANUAL.md` | User manual |
| `CHANGELOG.md` | Changes in each firmware version |

![Medusa module](https://github.com/user-attachments/assets/63473aa4-63ea-4bd8-871f-995da8978576)

![Medusa front](https://github.com/user-attachments/assets/11cc97fb-e715-48a2-9f7d-79626728065f)

## Support

Do you like this project? You can support my work on [GitHub Sponsors](https://github.com/sponsors/lonesoulsurfer).

## Licence

- **Firmware** (`Arduino IDE - Medusa/`): [MIT License](LICENSE). You can use, change and sell it.
- **Hardware** (PCB, front panel and Gerber files): [CERN Open Hardware Licence v2, Strongly Reciprocal](LICENSE-HARDWARE). You can make, change and sell boards. If you share or sell a changed design, you must also share the changed design files under the same licence.

Copyright (c) 2025 Marcus (lonesoulsurfer).
