# Unitree Lidar SDK2 Python Bindings

Python bindings for Unitree Lidar SDK2 with prebuilt native SDK static libraries in this repository.

## Install

From GitHub:

```bash
pip install git+https://github.com/mnvl/unilidar_sdk2_python.git
```

From local repository clone:

```bash
pip install .
```

The build automatically links against:

- `lib/x86_64/libunilidar_sdk2.a` on `x86_64`
- `lib/aarch64/libunilidar_sdk2.a` on `aarch64`

## Quick start

```python
from unitree_lidar_sdk import LidarReader, LIDAR_POINT_DATA_PACKET_TYPE, LIDAR_IMU_DATA_PACKET_TYPE

reader = LidarReader()
reader.initialize_serial(port="/dev/ttyACM0")

while True:
    packet_type = reader.run_parse()
    if packet_type == LIDAR_POINT_DATA_PACKET_TYPE:
        cloud = reader.get_point_cloud()
        if cloud:
            print(cloud["stamp"], len(cloud["points"]))
    elif packet_type == LIDAR_IMU_DATA_PACKET_TYPE:
        imu = reader.get_imu_data()
        if imu:
            print(imu["quaternion"])
```

## Integration test with real device

```bash
python -m unittest -v tests/test_device_stream.py
```

Use `UNITREE_LIDAR_TEST_MODE=udp` for UDP mode. See `tests/README.md` for all environment variables.
