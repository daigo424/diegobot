
# Diegobot

[日本語表記はこちら](README.ja.md)

A self-built differential-drive AMR (autonomous mobile robot). ROS 2 Jazzy + micro-ROS control the drivetrain, with teleop from a smartphone.

The long-term goal is to build something like a TurtleBot3 Waffle.

See [doc/wiring.md](doc/wiring.md) for hardware details.


https://github.com/user-attachments/assets/dc853809-cab7-45fa-803e-697c4c420df1

| Top side | Bottom side | Inside |
|-|-|-|
| <img width="4284" height="5712" alt="IMG_3719" src="https://github.com/user-attachments/assets/3d513cac-4771-48f2-9cec-5c678de6467a" /> | <img width="4284" height="5712" alt="IMG_3721" src="https://github.com/user-attachments/assets/99f35c13-2766-4b2a-8f3e-169f8383f291" /> | <img width="4284" height="5712" alt="IMG_3720" src="https://github.com/user-attachments/assets/ebad4de7-7a94-47d7-bd92-819fb143bd88" /> |

## Layout

```text
diegobot/
├── edge/
│   ├── docker/         # ROS 2 Jazzy image, rosbridge, micro-ROS Agent compose definitions
│   ├── workspace/      # ROS 2 workspace (colcon)
│   ├── pico/           # Pico firmware (PlatformIO). Production + verification test firmwares → edge/pico/README.md
│   ├── esp32/          # Legacy ESP32 firmware (superseded by Pico)
│   └── provisioning/   # Raspberry Pi initial-setup automation + BLE Wi-Fi provisioning daemon → edge/provisioning/README.md
├── mobile/              # Flutter app "diegoctl" (BLE provisioning + joystick teleop)
├── doc/                 # Hardware documentation (wiring, etc.)
└── Makefile             # Entry point for dev/deploy operations
```

## Basic operations on the dev machine

```sh
make build          # Build the ROS 2 image
make up             # Start the containers (diegobot / rosbridge / micro-ros-agent)
make down           # Stop them
make login          # Enter the diegobot container
make vendor-import  # Fetch third-party packages listed in edge/workspace/vendor.repos (run after a fresh clone or when vendor.repos changes)
```

Motor check:

```sh
make teleop-twist-keyboard   # Drive by sending /cmd_vel from the keyboard
```

The `micro-ros-agent` service in `edge/docker/docker-compose.yml` takes the host's USB serial port (Pico) from `HOST_MICRO_ROS_PORT` in `.env`. The `/dev/ttyACM*` number can shift on replug/reflash, so check it each time.

## Pico firmware

`edge/pico/` holds a PlatformIO project with the production firmware plus several verification-test firmwares for individual motors etc. See [edge/pico/README.md](edge/pico/README.md) for how to switch between them and what each one does.

```sh
cd edge/pico
pio run -t upload
```

## Deploying to the Raspberry Pi

```sh
make pi-rsync   # Copy the code over
make pi-ssh     # SSH into the Pi
```

See [edge/provisioning/README.md](edge/provisioning/README.md) for first-time setup on the Pi (installing Docker, making the BLE provisioning daemon persistent, etc.).

To cross-build the container image on the dev machine and ship it to the Pi (useful when building on the Pi itself is slow or runs out of memory):

```sh
make pi-push-image
```

## Mobile app (diegoctl)

`mobile/` contains a Flutter app with a BLE Wi-Fi provisioning screen and a joystick teleop screen that publishes `/cmd_vel` over rosbridge.

```sh
make android-install   # Build a release APK and install it on the connected device
```
