# Quickstart

A Linux terminal-based keyboard controller for autonomous driving, built on the
Apollo Cyber RT framework.

---

## Prerequisites

- Linux system with Python 3
- Apollo Cyber RT environment properly installed and sourced (ensure `cyber_py3`
  is available)
- Compiled Apollo control protobuf messages (`control_cmd_pb2`)

---

## How to Run

```bash
python3 whl_can.py
```

This will start the program with non-blocking keyboard listening and send
control commands in real time.

---

## Control Keys

| Key     | Function                               |
| ------- | -------------------------------------- |
| `w`     | Accelerate (increase speed)            |
| `s`     | Decelerate (decrease speed)            |
| `a`     | Turn left (increase steering)          |
| `d`     | Turn right (decrease steering)         |
| `m`     | Change gear (cycle P → R → N → D)      |
| `b`     | Increase brake                         |
| `B`     | Decrease brake                         |
| (space) | Emergency stop (speed zero, max brake) |
| `q`     | Quit program                           |

---
