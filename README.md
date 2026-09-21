# Ensō (円相)

*[日本語版はこちら / Japanese version here](README_ja.md)*

**Ambient Sound Simulator for M5Stack Cardputer ADV.** Built with PlatformIO + the Arduino framework.

Ensō generates ambient soundscapes — rain, wind, ocean waves, a flowing stream, birdsong, rustling leaves, crickets, a campfire, distant thunder, a singing bowl, a wind chime, a ticking clock, a fan, white/pink noise — entirely from noise, filters, resonators and oscillators in real time. **No samples are used anywhere.** Meant for meditation, relaxation, or simply as background sound while you work or sleep.

A soft ring breathes at the center of the screen: its color shows which sounds are currently layered in, and its wobble shows how much they're moving in the moment.

I tell Claude my ideas and it does the coding.
This firmware exists because a user of C.P.S. had started building something close to this idea and asked whether I'd carry it forward once C.P.S. settled down — Ensō is that idea, finished.
> **This is currently 100% vibe-coded.** If you're not comfortable with AI-assisted or vibe-coded firmware, I understand if this isn't for you.

---

## About the name

Ensō (円相) is the Zen circle -- traditionally painted in a single, unbroken brushstroke, representing enlightenment, the universe, and a moment captured completely as it is. It felt right for a firmware built entirely around one circle breathing at the center of the screen.

There's a second layer to it, too: in Japanese, "ensō" is also the reading of 演奏 (ensō), meaning "a musical performance" -- the act of playing. The same word, said the same way, points at both an unbroken circle and the act of playing. For a firmware that turns numbers into sound in real time, that felt like too good a coincidence to pass up.

---

## Features

| Category | Details |
|---|---|
| **Sound layers** | 15 layers, grouped by genre, each with its own volume, color and 3 parameters — **Water:** Rain, Stream, Waves · **Weather:** Wind, Thunder · **Life:** Leaves, Birds, Crickets · **Fire:** Fire · **Meditative:** Bowl (singing bowl), Chime (wind chime) · **Mechanical:** Clock, Fan · **Noise:** White, Pink |
| **Exact timing** | Clock and Fan run on a sample-accurate clock with no timing jitter at all — a second hand that wavers stops sounding like a second hand. Every other layer fluctuates naturally, the way real ambient sound does |
| **Scenes** | 13 preset scenes (Forest, Riverside, Ocean, Rainy Day, Thunderstorm, Campfire, Night, Zen Garden, Meditation, Study Room, Cool Room, Pink Noise, White Noise), plus unlimited custom scenes saved to SD |
| **Crossfading** | Switching scenes morphs volumes and parameters smoothly from the old scene to the new one, rather than muting and reloading |
| **Drift scene** | A scene that never settles — rolls a fresh random mix on selection, then keeps quietly evolving it (a parameter nudged, a level moved, a sound easing in or fading out) for as long as it stays selected |
| **Random scene** | `R` builds a new scene ("Elsewhere") from 2–4 natural layers with plausible settings. Mechanical and noise layers are excluded |
| **Space** | A light reverb over the whole mix. Bowl and Chime in particular are a different instrument with it on |
| **Session timer** | A timed sitting with a bell at the start, the end, and optionally a set interval — independent of the Bowl layer, so it sounds whatever scene is loaded |
| **LED** | Drives the on-board RGB LED from a chosen layer's activity, as a free-running breathing pulse, or synced to the breathing guide when one is running |
| **Breathing guide** | `Shift`+`;`/`.` cycles the main screen's ring between its normal reactive mode and three breathing patterns (Coherent 5.5, Box 4-4-4-4, Relax 4-7-8). The circle grows and shrinks with the breath |
| **Shake** | Shake the device to fire a configurable action — Random Scene, Sleep On, Display Off, or LED On/Off. `Shift`+`S` runs the same action from the keyboard, for boards without an IMU |
| **Sleep timer** | Fades the mix out and powers down after a set time |
| **Battery** | Percentage shown on the main screen |
| **Auto-save** | Settings and scene state save to SD a moment after you change anything |

---

## Hardware

| Item | Value |
|---|---|
| Supported device | M5Stack Cardputer ADV |
| MCU | ESP32-S3 (dual-core Xtensa LX7, 240 MHz) |
| Audio | Internal speaker + 3.5 mm jack (mono output — see *Known limitations*) |
| IMU | BMI270 6-axis (used for the Shake gesture) |
| LED | WS2812 RGB, on-board |
| SD slot | SPI — SCK=GPIO40, MISO=GPIO39, MOSI=GPIO14, CS=GPIO12 |

> Developed and tested on Cardputer ADV. `Shift`+`S` gives boards without an IMU a keyboard equivalent for the Shake action, but the firmware as a whole hasn't been verified on the original Cardputer.

---

## Installation

### Method 1 — Install via M5Burner (easiest)

No compiling required.

1. Download and install [M5Burner](https://docs.m5stack.com/en/uiflow/M5Burner) from the official site
2. Connect your Cardputer ADV via USB-C
3. Search for "Ensō" inside M5Burner
4. Select the correct COM port and press **Burn**
5. Ensō launches automatically once flashing finishes

> Insert a FAT32-formatted SD card to get automatic settings save and custom scenes.

### Method 2 — Install via Launcher FW

If your Cardputer ADV runs **Launcher FW**, you can install without compiling.

#### Method 2a — OTA (easiest)

1. Launch Launcher FW
2. Search for "Ensō" in its OTA feature
3. Select it and download/install
4. Ensō launches automatically once installed

#### Method 2b — Copy the .bin to SD manually

1. Download the latest `.bin` from the [Releases](https://github.com/oldtokage-collab/Enso/releases) page
2. Copy it to the **root** of your SD card (not a subfolder)
3. Insert the SD card and boot into Launcher FW
4. Select the file in Launcher's file browser and flash it
5. Ensō launches automatically once flashing finishes

> A FAT32 micro SD card is needed both for Method 2b and for Ensō's own settings.

### Method 3 — Build from source (PlatformIO)

**You'll need:**
- [VSCode](https://code.visualstudio.com/) + the **PlatformIO IDE** extension
- A Cardputer ADV connected via USB-C

**Steps:**
1. Clone or download this repository
2. Open the `Enso` folder in VSCode (`File › Open Folder`)
3. `platformio.ini` is detected automatically; required libraries download on first build
4. Click **Upload** (→) in the bottom toolbar

On a successful build, `.pio/build/m5stack-cardputer-adv/` contains:

| File | Purpose |
|---|---|
| `firmware.bin` | App only (used by PlatformIO's Upload button) |
| `merge.bin` | **Combined image** (bootloader + partitions + app), for M5Burner or single-file flashing tools |

> **If the device won't enter flashing mode:** power off → hold G0 → power on → release G0.

### First boot (any install method)

On first boot, Ensō creates `/ENSO/` (and `/ENSO/scene/`) on the SD card automatically — a FAT32-formatted micro SD card is needed for this. Settings and scene changes save to `/ENSO/settings.json` a moment after each change. Without an SD card inserted, Ensō still runs fine on its built-in defaults; nothing is saved between power cycles.

---

## Operation

### Mode switching

| Key | Action |
|---|---|
| `Tab` | Cycle menus: Main → Edit → Settings → Main |
| `Space` | Play / pause |
| `H` | Toggle the help overlay for the current screen |
| `G0` | Display off |

### Main screen

The main screen plays the current scene: the ring at its center shows, in color and motion, what's playing. Battery percentage, and the sleep/session countdowns when running, sit in the top-right.

| Key | Action |
|---|---|
| `,` / `/` | Switch scene — crossfades in immediately, no confirmation needed |
| `;` / `.` | Master volume |
| `Shift` + `;` / `.` | Cycle the breathing guide: reactive rings → Coherent 5.5 → Box 4-4-4-4 → Relax 4-7-8 |
| `R` | Random scene ("Elsewhere") |
| `L` | LED on / off |
| `Shift` + `S` | Run the configured Shake action from the keyboard |

### Edit screen

Lists all 15 layers, grouped by genre, with a volume bar and Mute/Solo indicator for each.

| Key | Action |
|---|---|
| `;` / `.` | Select layer |
| `,` / `/` | Layer volume |
| `Enter` | Open the layer's parameter screen |
| `M` | Mute layer |
| `S` | Solo layer (session only, not saved) |
| `R` | Random scene |
| `L` | LED on / off |

**Layer parameters** (opened with `Enter`): each layer has 3 parameters plus a Color swatch.

| Key | Action |
|---|---|
| `;` / `.` | Select parameter |
| `,` / `/` | Adjust value / cycle palette color |
| `Tab` or `` ` `` | Back to the layer list |

### Settings screen

| Key | Action |
|---|---|
| `;` / `.` | Select item |
| `Enter` | Open |

**Display** — brightness, auto-dim delay and level, auto-off delay.

**LED** — on/off, source (a layer's activity, or a free-running period), color (a palette entry, or "follow the source layer"), brightness, speed, sway.

**Space** — reverb amount, size, damping. Amount 0 turns it off.

**Session** — sitting duration, an optional interval bell, bell pitch, and Start/Stop.

**Sleep Timer** — on/off, minutes until power-down, optional fade-out over the last minute.

**Shake** — on/off, sensitivity, and the action to fire: Random Scene, Sleep On, Display Off, or LED On/Off.

**Scenes** — manage the scene list:

| Key | Action |
|---|---|
| `;` / `.` | Move selection |
| `Enter` | Show / hide the scene in the main-screen browse list |
| `,` / `/` | Reorder |
| `N` | Save the current mix as a new custom scene |
| `R` | Rename a custom scene |
| `D` | Delete a custom scene |

Presets and the Drift scene can't be renamed, deleted, or overwritten.

**About** — version, SD/IMU detection, battery level.

---

## Signal path

```
Layer generators
(noise, filters, resonators, oscillators — one per sound)
    │
    ▼
Layer mix (per-layer volume, mute / solo)
    │
    ▼
Master volume
    │
    ▼
Space (reverb, optional)
    │
    ▼
Session bell (mixed in at start / end / interval)
    │
    ▼
Dither + 16-bit output
    │
    ▼
Speaker
```

---

## Project structure

```
Enso/
├── platformio.ini    # Build configuration
├── merge_bin.py       # Post-build script: produces merge.bin for M5Burner
└── src/
    └── main.cpp        # All source code (single file)
```

---

## Dependencies

Managed automatically by PlatformIO.

| Library | Version |
|---|---|
| `m5stack/M5Cardputer` | latest |
| `m5stack/M5Unified` | latest |
| `m5stack/M5GFX` | latest |
| `bblanchon/ArduinoJson` | ^7.0.0 |
| `adafruit/Adafruit NeoPixel` | ^1.12.0 |

`SD` and `SPI` ship with the Arduino-ESP32 core, so they don't need a `lib_deps` entry.

---

## Known limitations / ideas for later

- Audio output is mono. This board's 3.5 mm jack doesn't provide independent left/right channels, so there's no true stereo effect on this hardware.
- No real-time clock, so any timing feature is relative ("in N minutes"), never a fixed time of day.
- Display is 240×135px; the layout is fairly tight.
- Developed and tested on Cardputer ADV. The original Cardputer isn't officially verified.

---

## Credits

Concept and specification by a user of C.P.S., who had started building something close to this idea but wasn't able to see it through — Ensō exists to carry that idea forward. Firmware built with Claude (Anthropic).

## License

[CC BY-NC-SA 4.0](https://creativecommons.org/licenses/by-nc-sa/4.0/) — free to use, modify, and share for personal, non-commercial purposes, with attribution. Commercial use is not permitted. This firmware was built from an idea a community member shared rather than one of my own, so it didn't feel right to license it for commercial use.
