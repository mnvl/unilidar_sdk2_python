# Python binding integration test

This test talks to a real Unitree lidar device and prints parsed data.

## Run

```bash
python -m unittest -v tests/test_device_stream.py
```

## Environment variables

- `UNITREE_LIDAR_TEST_MODE`: `serial` (default) or `udp`
- `UNITREE_LIDAR_TEST_MAX_WAIT`: max seconds to wait for packets (default `15`)
- `UNITREE_LIDAR_TEST_PRINT_PACKETS`: max packet prints (default `20`)

Serial mode:

- `UNITREE_LIDAR_SERIAL_PORT` (default `/dev/ttyACM0`)
- `UNITREE_LIDAR_SERIAL_BAUDRATE` (default `4000000`)

UDP mode:

- `UNITREE_LIDAR_UDP_LIDAR_IP` (default `192.168.1.62`)
- `UNITREE_LIDAR_UDP_LIDAR_PORT` (default `6101`)
- `UNITREE_LIDAR_UDP_LOCAL_IP` (default `192.168.1.2`)
- `UNITREE_LIDAR_UDP_LOCAL_PORT` (default `6201`)

Shared:

- `UNITREE_LIDAR_CLOUD_SCAN_NUM` (default `18`)
- `UNITREE_LIDAR_USE_SYSTEM_TIMESTAMP` (`1` default, set `0` to use lidar timestamp)
- `UNITREE_LIDAR_RANGE_MIN` (default `0`)
- `UNITREE_LIDAR_RANGE_MAX` (default `100`)
