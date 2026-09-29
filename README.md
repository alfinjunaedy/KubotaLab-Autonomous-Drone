# Autonomous Drone

<p align="center">
  <img src="imgs/demo.gif" alt="Autonomous flight demo" />
</p>

Compact autonomous quadcopter built around a Raspberry Pi Zero 2 W. It estimates its position from an IMU, optical flow, and an altitude ToF sensor, then closes the loop on position and velocity onboard — no external positioning system required.

## Hardware

| | |
|---|---|
| Dimensions | 205 (L) × 205 (W) × 110 (H) mm |
| Weight | 877 g |
| Battery | Li-ion 6S pack (~5 min flight time) |
| Onboard controller | Raspberry Pi Zero 2 W |
| Sensors | IMU · optical flow · altitude ToF · A/D converter |
| Payload | iPhone 16 Pro |
| Flight controller | 4-in-1 ESC (Betaflight) |

<p align="center">
  <img src="imgs/iso.png" width="50%" alt="Isometric CAD view" />
  <img src="imgs/top.png" width="50%" alt="Top CAD view" />
</p>

## Repository layout

```
├── cad/Assembly.step    # robot CAD design
├── imgs/                # demo.gif · iso.png · top.png
├── includes/            # include files
├── log/                 # binary flight logs (*.bin)
├── src/                 # source files
├── main.cpp             # main code
├── Makefile
└── plot_log_bin.py      # plot flight logs with Python
```

## Environment setup

The controller runs **Raspbian GNU/Linux 12 (Bookworm), armv7l 32-bit**. Prepare the system once before building the code.

**1. System packages** — SSH (remote access), Avahi (mDNS discovery), Samba (file shares for pulling `log/*.bin`), and monitoring tools:

```bash
sudo apt install openssh-server
sudo apt install samba
sudo apt install avahi-daemon
```

**2. [WiringPi](https://github.com/WiringPi/WiringPi)** — GPIO library, built and installed from source:

```bash
git clone https://github.com/WiringPi/WiringPi.git
cd WiringPi
./build debian
cd debian-template
sudo chmod +x wiringpi_3.2_armhf.deb
sudo apt install ./wiringpi_3.2_armhf.deb
sudo systemctl enable pigpiod
```

**3. Boot config** — edit `/boot/firmware/config.txt` and add the following below `[all]`:

```
dtoverlay=vc4-kms-v3d
enable_uart=1
dtoverlay=disable-bt
dtoverlay=dwc2,dr_mode=host
dtoverlay=i2c-gpio,bus=3,i2c_gpio_sda=4,i2c_gpio_scl=17,i2c_gpio_delay_us=2
over_voltage=2
arm_freq=1200
core_freq=500
```

This overclocks the Pi to 1.2 GHz, enables the UART (Bluetooth disabled to free it), puts USB into host mode, and bit-bangs I²C bus 3 on GPIO 4 (SDA) / GPIO 17 (SCL).

## Build

From the repository root:

```bash
make clean && make
```

## Software design

```
┌────────────────────────────────────────────────┐
│ Application — C++ (threads 0 / 1 / 2)          │
├────────────────────────────────────────────────┤
│ OS — Raspbian 12 Bookworm (32-bit)             │
├────────────────────────────────────────────────┤
│ Hardware — sensors · Pi Zero 2 W · 4-in-1 ESC  │
└────────────────────────────────────────────────┘
```

All threads run simultaneously:

- **Thread 0** — position control (100 Hz), CRFS communication (100 Hz)
- **Thread 1** — sensor readings (IMU 100 Hz, A/D conv 100 Hz, optical flow 10 Hz), position estimation (10 Hz), data logging (100 Hz)
- **Thread 2** — ToF reading (15 Hz), supervisor at 15 Hz (internal monitoring & management of flight and emergency states)

## Control design

```mermaid
flowchart LR
    subgraph sensors [Sensors]
        IMU[IMU]
        OF[Optical flow]
        TOF[ToF 8x8]
    end

    PE[Position estimation<br/>xyz · v_xyz · rpy · rates]

    subgraph control [Control]
        PC[Position control]
        VC[Velocity control]
        OC[Orientation control]
        GE[Ground effect compensation]
        SUM(("+"))
        FC[FC · motor mixing<br/>Betaflight 4-in-1 ESC]
    end

    USER([Target xyz · yaw])
    MOTORS([Motors])

    IMU --> PE
    OF --> PE
    TOF --> PE

    USER -->|target xyz| PC
    USER -->|target yaw| OC
    PC -->|target vel| VC

    VC -->|roll-pitch-thrust| SUM
    OC -->|yaw| SUM
    TOF -->|8x8 range| GE
    IMU --> GE
    GE -->|Δ roll-pitch-yaw-thrust| SUM
    SUM -->|roll-pitch-yaw-thrust| FC

    FC --> MOTORS

    PE -->|xyz| PC
    PE -->|v_xyz| VC
    PE -->|rpy · rates| OC
```

| Module | In | Out | Feedback |
|---|---|---|---|
| Position control | target xyz | target velocity | current xyz |
| Velocity control | target velocity | roll-pitch-thrust | current velocity |
| Orientation control | target yaw | target yaw | yaw, yaw rate |
| Position estimation | ToF 8×8, IMU, optical flow | xyz, v_xyz, rpy, rates | — |
| Ground effect | ToF 8×8, IMU | Δ roll-pitch-yaw-thrust (compensation) | — |
| FC | roll-pitch-yaw-thrust | motor mixing | — |

## Flight stages

| Stage | Code |
|---|---|
| Landed | 0 |
| Takeoff | 1 |
| Hover | 2 |
| Move | 3 |
| Landing | 4 |

## Emergency stages

**Emergency landing**

| Condition | Code |
|---|---|
| Battery ≤ 20 V | -1 |
| Altitude > 1.65 m | -2 |
| Sensor frequency < 50% of nominal | -3 |

**Emergency stop**

| Condition | Code |
|---|---|
| Linear velocity > ±0.65 m/s | -11 |
| Roll-pitch angle > ±25° | -12 |
| Sensor frequency = 0 Hz | -13 |
| Altitude > 1.95 m | -14 |
| Button | -15 |
| Angular velocity > ±180 °/s | -16 |

## Flight data

All flight data are stored in binary log files (`log/*.bin`). Plot them with:

```bash
python plot_log_bin.py [logfilename].bin
```

## License

Released under the [MIT License](LICENSE) — applies to the code, CAD, and documentation.
