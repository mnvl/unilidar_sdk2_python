import os
import time
import unittest

import numpy as np

from unitree_lidar_sdk import (
    LIDAR_2D_POINT_DATA_PACKET_TYPE,
    LIDAR_IMU_DATA_PACKET_TYPE,
    LIDAR_POINT_DATA_PACKET_TYPE,
    LIDAR_VERSION_PACKET_TYPE,
    LidarReader,
)


MODE = os.getenv("UNITREE_LIDAR_TEST_MODE", "serial").strip().lower()
MAX_WAIT_SECONDS = float(os.getenv("UNITREE_LIDAR_TEST_MAX_WAIT", "15"))
MAX_PRINT_PACKETS = int(os.getenv("UNITREE_LIDAR_TEST_PRINT_PACKETS", "20"))


class TestUnitreeLidarDeviceStream(unittest.TestCase):
    def setUp(self):
        self.reader = LidarReader()

    def tearDown(self):
        try:
            self.reader.close_serial()
        except Exception:
            pass
        try:
            self.reader.close_udp()
        except Exception:
            pass

    def _initialize_reader(self):
        if MODE == "serial":
            port = os.getenv("UNITREE_LIDAR_SERIAL_PORT", "/dev/ttyACM0")
            baudrate = int(os.getenv("UNITREE_LIDAR_SERIAL_BAUDRATE", "4000000"))
            cloud_scan_num = int(os.getenv("UNITREE_LIDAR_CLOUD_SCAN_NUM", "18"))
            use_system_timestamp = os.getenv("UNITREE_LIDAR_USE_SYSTEM_TIMESTAMP", "1") != "0"
            range_min = float(os.getenv("UNITREE_LIDAR_RANGE_MIN", "0"))
            range_max = float(os.getenv("UNITREE_LIDAR_RANGE_MAX", "100"))

            ret = self.reader.initialize_serial(
                port=port,
                baudrate=baudrate,
                cloud_scan_num=cloud_scan_num,
                use_system_timestamp=use_system_timestamp,
                range_min=range_min,
                range_max=range_max,
            )
            self.assertEqual(ret, 0, f"initialize_serial failed on {port}, ret={ret}")
            print(f"[init] serial port={port}, baudrate={baudrate}, ret={ret}")
            return

        if MODE == "udp":
            lidar_port = int(os.getenv("UNITREE_LIDAR_UDP_LIDAR_PORT", "6101"))
            lidar_ip = os.getenv("UNITREE_LIDAR_UDP_LIDAR_IP", "192.168.1.62")
            local_port = int(os.getenv("UNITREE_LIDAR_UDP_LOCAL_PORT", "6201"))
            local_ip = os.getenv("UNITREE_LIDAR_UDP_LOCAL_IP", "192.168.1.2")
            cloud_scan_num = int(os.getenv("UNITREE_LIDAR_CLOUD_SCAN_NUM", "18"))
            use_system_timestamp = os.getenv("UNITREE_LIDAR_USE_SYSTEM_TIMESTAMP", "1") != "0"
            range_min = float(os.getenv("UNITREE_LIDAR_RANGE_MIN", "0"))
            range_max = float(os.getenv("UNITREE_LIDAR_RANGE_MAX", "100"))

            ret = self.reader.initialize_udp(
                lidar_port=lidar_port,
                lidar_ip=lidar_ip,
                local_port=local_port,
                local_ip=local_ip,
                cloud_scan_num=cloud_scan_num,
                use_system_timestamp=use_system_timestamp,
                range_min=range_min,
                range_max=range_max,
            )
            self.assertEqual(ret, 0, f"initialize_udp failed, ret={ret}")
            print(
                "[init] udp "
                f"lidar={lidar_ip}:{lidar_port} local={local_ip}:{local_port}, ret={ret}"
            )
            return

        self.fail("UNITREE_LIDAR_TEST_MODE must be 'serial' or 'udp'")

    def test_stream_and_display(self):
        self._initialize_reader()

        deadline = time.monotonic() + MAX_WAIT_SECONDS
        found = {"cloud": 0, "cloud2d": 0, "imu": 0, "version": 0}
        printed = 0

        while time.monotonic() < deadline:
            packet_type = self.reader.run_parse()

            if packet_type == LIDAR_POINT_DATA_PACKET_TYPE:
                cloud = self.reader.get_point_cloud()
                if cloud:
                    found["cloud"] += 1
                    if printed < MAX_PRINT_PACKETS:
                        points = cloud["points"]
                        self.assertIsInstance(points, np.ndarray)
                        self.assertEqual(points.ndim, 2)
                        self.assertEqual(points.shape[1], 6)
                        first = points[0].tolist() if len(points) > 0 else None
                        print(
                            "[cloud3d] "
                            f"stamp={cloud['stamp']:.6f} id={cloud['id']} "
                            f"points={len(points)} first={first}"
                        )
                        printed += 1

            elif packet_type == LIDAR_2D_POINT_DATA_PACKET_TYPE:
                # Current binding does not expose dedicated 2D cloud accessor.
                found["cloud2d"] += 1
                if printed < MAX_PRINT_PACKETS:
                    print("[cloud2d] packet received")
                    printed += 1

            elif packet_type == LIDAR_IMU_DATA_PACKET_TYPE:
                imu = self.reader.get_imu_data()
                if imu:
                    found["imu"] += 1
                    if printed < MAX_PRINT_PACKETS:
                        print(
                            "[imu] "
                            f"seq={imu['seq']} stamp={imu['stamp_sec']}.{imu['stamp_nsec']} "
                            f"q={imu['quaternion']} av={imu['angular_velocity']} "
                            f"la={imu['linear_acceleration']}"
                        )
                        printed += 1

            elif packet_type == LIDAR_VERSION_PACKET_TYPE:
                sdk = self.reader.get_sdk_version()
                fw = self.reader.get_lidar_firmware_version()
                hw = self.reader.get_lidar_hardware_version()
                found["version"] += 1
                if printed < MAX_PRINT_PACKETS:
                    print(f"[version] sdk={sdk} fw={fw} hw={hw}")
                    printed += 1

        print(f"[summary] counts={found}")
        received_any = any(count > 0 for count in found.values())
        self.assertTrue(
            received_any,
            f"No lidar packets received within {MAX_WAIT_SECONDS} seconds (mode={MODE})",
        )


if __name__ == "__main__":
    unittest.main(verbosity=2)
