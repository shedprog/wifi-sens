# wifi-sens

ESP32-S3 Wi-Fi CSI sender/receiver firmware plus the `esp_csi_tool.py` desktop viewer.

## Project layout

- `csi_send/` - PlatformIO sender firmware. It sends ESP-NOW packets on a fixed Wi-Fi channel.
- `csi_recv/` - PlatformIO receiver firmware. It listens on the same channel, enables CSI, and prints `CSI_DATA` / `RADAR_DADA` lines over serial.
- `tools/` - Python Qt GUI for reading the receiver serial stream, plotting CSI/radar data, and writing captures under `tools/log/` and `tools/data/`.

## Requirements

- Two ESP32-S3 boards.
- Python 3.10+ recommended.
- PlatformIO CLI or VS Code PlatformIO.
- Linux serial access to `/dev/ttyACM*` or `/dev/ttyUSB*`.

Install the Python and PlatformIO dependencies:

```bash
cd wifi-sens
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

If serial ports require permission on Linux:

```bash
sudo usermod -aG dialout "$USER"
```

Log out and back in after changing the group.

## Configure

Both boards must use the same Wi-Fi channel. The default is channel `11`.

- Receiver channel: `csi_recv/platformio.ini`
- Sender channel and send rate: `csi_send/platformio.ini`

The default sender MAC is `1A:00:00:00:00:00` in both firmware projects. If you change any `CSI_SEND_MAC*` build flag, change it in both projects.

Default ports in this repo:

- Sender upload/monitor port: `/dev/ttyACM0`
- Receiver upload/monitor port: `/dev/ttyACM1`

Edit the `upload_port` and `monitor_port` values in each `platformio.ini` if your boards appear on different ports.

## Build and upload

Connect the sender board first, then upload:

```bash
pio run -d csi_send -t upload
```

Connect the receiver board, then upload:

```bash
pio run -d csi_recv -t upload
```

Optional receiver serial check:

```bash
pio device monitor -d csi_recv -p /dev/ttyACM1 -b 2000000
```

You should see receiver status lines and `CSI_DATA` once the sender is running.

## Run the CSI GUI

Start the GUI against the receiver serial port:

```bash
python tools/esp_csi_tool.py -p /dev/ttyACM1 -t LLTF
```

The GUI creates runtime files under:

- `tools/log/`
- `tools/data/`

Those folders are ignored by Git.

If PyQt5 fails to start on Linux with an `xcb` plugin error, install the missing Qt/X11 system packages for your distribution and rerun the same command.
