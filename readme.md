# NeuroPawn LSL Streamer

Barebones Python GUI for NeuroPawn Knight / Knight IMU boards.

It connects to the board over serial, sends channel and RLD commands, waits 1 second after every command, parses incoming packets, and publishes the data to an LSL stream.

Website: [neuropawn.tech](https://neuropawn.tech)

Project and board documentation:
- [NeuroPawn website](https://neuropawn.tech)
- [BrainFlow NeuroPawn README](https://github.com/NeuroPawn/brainflow/blob/docs-v2/README.md)
- [Packet format source](https://github.com/NeuroPawn/brainflow/blob/docs-v2/src/board_controller/neuropawn/knight.cpp)

## Features

- Serial connection to NeuroPawn Knight boards
- Channel on/off controls
- RLD on/off controls
- Gain selection
- Software notch filter at 50 Hz or 60 Hz
- Software bandpass filter from 1 to 40 Hz
- Auto-detects non-IMU vs IMU packet format from live packets
- LSL receiver GUI with live scrolling visualizer

## Board formats

The streamer auto-detects the board type by packet length:

- Non-IMU board:
  - 21-byte frame total
  - 8 EXG channels
  - Packet layout:
    - start byte: 0xA0
    - sample number
    - 8x 16-bit EXG values
    - LOFF STATP
    - LOFF STATN
    - end byte: 0xC0

- IMU board:
  - 57-byte frame total
  - 8 EXG channels plus 9 IMU values
  - Packet layout:
    - start byte: 0xA0
    - counter
    - 8x 16-bit EXG values
    - LOFF STATP
    - LOFF STATN
    - 9x float32 IMU values:
      - AccelX, AccelY, AccelZ
      - GyroX, GyroY, GyroZ
      - MagX, MagY, MagZ
    - end byte: 0xC0

## Command syntax

Commands are sent as ASCII strings over the serial port.

- Enable channel and set gain:
  - chon_{channel}_{gain}
- Disable channel:
  - choff_{channel}
- Enable RLD:
  - rldadd_{channel}
- Disable RLD:
  - rldremove_{channel}

Valid gain values:

- 1
- 2
- 3
- 4
- 6
- 8
- 12

Each command is followed by a 2 second pause.

## Files

- knight_lsl_gui.py  
  Serial reader, packet parser, IMU auto-detection, LSL streamer, and control GUI.

- knight_lsl_receiver.py  
  LSL receiver and live visualizer.

- requirements.txt  
  Python dependencies.

## Install

```bash
pip install -r requirements.txt
```

## Run

Start the streamer first:

```bash
python knight_lsl_gui.py
```

Then start the visualizer in a second terminal:

```bash
python knight_lsl_receiver.py
```

The streamer publishes an LSL stream named `NeuroPawnKnight`. The receiver
auto-detects the stream and shows a live scrolling plot.