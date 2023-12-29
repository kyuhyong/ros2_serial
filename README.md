# serial_com

Serial port interface example for ROS2 using asio library

# Package info

- Package name: serial_com
- ROS distro: Foxy (tested)
- OS: Ubuntu 20.04 (tested)

# Installation

## Dependency

Apt install some pacakges

```
sudo apt install libasio-dev libboost-dev
```

## Colcon build

Clone this package into your ros2 workspace and try colcon build.

```
cd to ros2_workspace/src
git clone https://github.com/kyuhyong/ros2_serial
cd ..
colcon build --pacakge-select serial_com
```

# Launch package

Connect **usb-to-serial port** on your pc and check its path

Modify **config/serial.yaml** file per setup.

- port name
- baud rate

If all settings are correct, launch serial_com by entering below.

```
ros2 launch serial_com serial.launch.py
```

# About AsioSerial

AsioSerial handles opening, transmitting and receiving packets through any serial port.

## Receiving from RX

- Before calling any receive function, call **set_callback_read_until()** to bind a callback function which will be triggered when there is data to read.
- Call **set_end_of_read_char(char)** to set end_of_read_char (EOC) of your choice. (Default is "**\n**")
- **open(string port_name, int baud_rate)** will open port.
- To receive packes until EOC met, call **start_async_receive()**.
- To receive any packets, just call **start_receive()**

## Transmitting to TX

- To send string message: **write_some(string)**
- To send array of bytes: **write_bytes(** char*, **int)**
