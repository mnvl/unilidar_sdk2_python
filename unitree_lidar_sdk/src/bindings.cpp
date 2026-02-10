#include <array>
#include <stdexcept>
#include <string>

#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "unitree_lidar_protocol.h"
#include "unitree_lidar_sdk.h"

namespace py = pybind11;
using namespace unilidar_sdk2;

namespace {

std::array<uint8_t, 4> to_array4(const std::vector<uint8_t> &values, const char *name)
{
    if (values.size() != 4) {
        throw std::invalid_argument(std::string(name) + " must contain 4 elements");
    }
    return {values[0], values[1], values[2], values[3]};
}

std::array<uint8_t, 6> to_array6(const std::vector<uint8_t> &values, const char *name)
{
    if (values.size() != 6) {
        throw std::invalid_argument(std::string(name) + " must contain 6 elements");
    }
    return {values[0], values[1], values[2], values[3], values[4], values[5]};
}

py::dict to_py_imu(const LidarImuData &imu)
{
    py::dict out;
    out["seq"] = imu.info.seq;
    out["stamp_sec"] = imu.info.stamp.sec;
    out["stamp_nsec"] = imu.info.stamp.nsec;

    out["quaternion"] = std::vector<float>{
        imu.quaternion[0], imu.quaternion[1], imu.quaternion[2], imu.quaternion[3]};
    out["angular_velocity"] = std::vector<float>{
        imu.angular_velocity[0], imu.angular_velocity[1], imu.angular_velocity[2]};
    out["linear_acceleration"] = std::vector<float>{
        imu.linear_acceleration[0], imu.linear_acceleration[1], imu.linear_acceleration[2]};
    return out;
}

py::dict to_py_cloud(const PointCloudUnitree &cloud)
{
    py::dict out;
    out["stamp"] = cloud.stamp;
    out["id"] = cloud.id;
    out["ring_num"] = cloud.ringNum;

    // points columns: x, y, z, intensity, time, ring
    py::array_t<float> points(
        {static_cast<py::ssize_t>(cloud.points.size()), static_cast<py::ssize_t>(6)});
    auto points_view = points.mutable_unchecked<2>();
    for (py::ssize_t i = 0; i < points_view.shape(0); ++i) {
        const auto &p = cloud.points[static_cast<size_t>(i)];
        points_view(i, 0) = p.x;
        points_view(i, 1) = p.y;
        points_view(i, 2) = p.z;
        points_view(i, 3) = static_cast<float>(p.intensity);
        points_view(i, 4) = p.time;
        points_view(i, 5) = static_cast<float>(p.ring);
    }
    out["points"] = points;
    return out;
}

} // namespace

class LidarReader {
public:
    LidarReader() : reader_(createUnitreeLidarReader())
    {
        if (reader_ == nullptr) {
            throw std::runtime_error("createUnitreeLidarReader() returned nullptr");
        }
    }

    int initialize_serial(
        const std::string &port = "/dev/ttyACM0",
        uint32_t baudrate = 4000000,
        uint16_t cloud_scan_num = 18,
        bool use_system_timestamp = true,
        float range_min = 0,
        float range_max = 100)
    {
        return reader_->initializeSerial(
            port, baudrate, cloud_scan_num, use_system_timestamp, range_min, range_max);
    }

    int initialize_udp(
        uint16_t lidar_port = 6101,
        const std::string &lidar_ip = "192.168.1.62",
        uint16_t local_port = 6201,
        const std::string &local_ip = "192.168.1.2",
        uint16_t cloud_scan_num = 18,
        bool use_system_timestamp = true,
        float range_min = 0,
        float range_max = 100)
    {
        return reader_->initializeUDP(
            lidar_port,
            lidar_ip,
            local_port,
            local_ip,
            cloud_scan_num,
            use_system_timestamp,
            range_min,
            range_max);
    }

    bool close_serial() { return reader_->closeSerial(); }

    bool close_udp() { return reader_->closeUDP(); }

    int run_parse() { return reader_->runParse(); }

    void clear_buffer() { reader_->clearBuffer(); }

    py::object get_point_cloud() const
    {
        PointCloudUnitree cloud;
        if (!reader_->getPointCloud(cloud)) {
            return py::none();
        }
        return to_py_cloud(cloud);
    }

    py::object get_imu_data() const
    {
        LidarImuData imu;
        if (!reader_->getImuData(imu)) {
            return py::none();
        }
        return to_py_imu(imu);
    }

    py::object get_sdk_version() const
    {
        std::string value;
        if (!reader_->getVersionOfSDK(value)) {
            return py::none();
        }
        return py::str(value);
    }

    py::object get_lidar_firmware_version() const
    {
        std::string value;
        if (!reader_->getVersionOfLidarFirmware(value)) {
            return py::none();
        }
        return py::str(value);
    }

    py::object get_lidar_hardware_version() const
    {
        std::string value;
        if (!reader_->getVersionOfLidarHardware(value)) {
            return py::none();
        }
        return py::str(value);
    }

    py::object get_time_delay() const
    {
        double value;
        if (!reader_->getTimeDelay(value)) {
            return py::none();
        }
        return py::float_(value);
    }

    py::object get_dirty_percentage() const
    {
        float value;
        if (!reader_->getDirtyPercentage(value)) {
            return py::none();
        }
        return py::float_(value);
    }

    void send_user_ctrl_cmd(uint32_t cmd_type, uint32_t cmd_value)
    {
        LidarUserCtrlCmd cmd{cmd_type, cmd_value};
        reader_->sendUserCtrlCmd(cmd);
    }

    void set_lidar_work_mode(uint32_t mode) { reader_->setLidarWorkMode(mode); }

    void sync_lidar_timestamp() { reader_->syncLidarTimeStamp(); }

    void reset_lidar() { reader_->resetLidar(); }

    void stop_lidar_rotation() { reader_->stopLidarRotation(); }

    void start_lidar_rotation() { reader_->startLidarRotation(); }

    void set_lidar_ip_address_config(
        const std::vector<uint8_t> &lidar_ip,
        const std::vector<uint8_t> &user_ip,
        const std::vector<uint8_t> &gateway,
        const std::vector<uint8_t> &subnet_mask,
        uint16_t lidar_port,
        uint16_t user_port)
    {
        const auto lidar_ip_arr = to_array4(lidar_ip, "lidar_ip");
        const auto user_ip_arr = to_array4(user_ip, "user_ip");
        const auto gateway_arr = to_array4(gateway, "gateway");
        const auto subnet_arr = to_array4(subnet_mask, "subnet_mask");

        LidarIpAddressConfig cfg{};
        for (size_t i = 0; i < 4; ++i) {
            cfg.lidar_ip[i] = lidar_ip_arr[i];
            cfg.user_ip[i] = user_ip_arr[i];
            cfg.gateway[i] = gateway_arr[i];
            cfg.subnet_mask[i] = subnet_arr[i];
        }
        cfg.lidar_port = lidar_port;
        cfg.user_port = user_port;
        reader_->setLidarIpAddressConfig(cfg);
    }

    void set_lidar_mac_address_config(const std::vector<uint8_t> &mac)
    {
        const auto mac_arr = to_array6(mac, "mac");
        LidarMacAddressConfig cfg{};
        for (size_t i = 0; i < 6; ++i) {
            cfg.mac[i] = mac_arr[i];
        }
        reader_->setLidarMacAddressConfig(cfg);
    }

    size_t get_buffer_cached_size() const { return reader_->getBufferCachedSize(); }

    size_t get_buffer_read_size() const { return reader_->getBufferReadSize(); }

private:
    UnitreeLidarReader *reader_;
};

PYBIND11_MODULE(_native, m)
{
    m.doc() = "Python bindings for Unitree Lidar SDK2";

    py::class_<LidarReader>(m, "LidarReader")
        .def(py::init<>())
        .def("initialize_serial",
             &LidarReader::initialize_serial,
             py::arg("port") = "/dev/ttyACM0",
             py::arg("baudrate") = 4000000,
             py::arg("cloud_scan_num") = 18,
             py::arg("use_system_timestamp") = true,
             py::arg("range_min") = 0,
             py::arg("range_max") = 100)
        .def("initialize_udp",
             &LidarReader::initialize_udp,
             py::arg("lidar_port") = 6101,
             py::arg("lidar_ip") = "192.168.1.62",
             py::arg("local_port") = 6201,
             py::arg("local_ip") = "192.168.1.2",
             py::arg("cloud_scan_num") = 18,
             py::arg("use_system_timestamp") = true,
             py::arg("range_min") = 0,
             py::arg("range_max") = 100)
        .def("close_serial", &LidarReader::close_serial)
        .def("close_udp", &LidarReader::close_udp)
        .def("run_parse", &LidarReader::run_parse)
        .def("clear_buffer", &LidarReader::clear_buffer)
        .def("get_point_cloud", &LidarReader::get_point_cloud)
        .def("get_imu_data", &LidarReader::get_imu_data)
        .def("get_sdk_version", &LidarReader::get_sdk_version)
        .def("get_lidar_firmware_version", &LidarReader::get_lidar_firmware_version)
        .def("get_lidar_hardware_version", &LidarReader::get_lidar_hardware_version)
        .def("get_time_delay", &LidarReader::get_time_delay)
        .def("get_dirty_percentage", &LidarReader::get_dirty_percentage)
        .def("send_user_ctrl_cmd", &LidarReader::send_user_ctrl_cmd)
        .def("set_lidar_work_mode", &LidarReader::set_lidar_work_mode)
        .def("sync_lidar_timestamp", &LidarReader::sync_lidar_timestamp)
        .def("reset_lidar", &LidarReader::reset_lidar)
        .def("stop_lidar_rotation", &LidarReader::stop_lidar_rotation)
        .def("start_lidar_rotation", &LidarReader::start_lidar_rotation)
        .def("set_lidar_ip_address_config", &LidarReader::set_lidar_ip_address_config)
        .def("set_lidar_mac_address_config", &LidarReader::set_lidar_mac_address_config)
        .def("get_buffer_cached_size", &LidarReader::get_buffer_cached_size)
        .def("get_buffer_read_size", &LidarReader::get_buffer_read_size);

    m.attr("LIDAR_POINT_DATA_PACKET_TYPE") = py::int_(LIDAR_POINT_DATA_PACKET_TYPE);
    m.attr("LIDAR_2D_POINT_DATA_PACKET_TYPE") = py::int_(LIDAR_2D_POINT_DATA_PACKET_TYPE);
    m.attr("LIDAR_IMU_DATA_PACKET_TYPE") = py::int_(LIDAR_IMU_DATA_PACKET_TYPE);
    m.attr("LIDAR_VERSION_PACKET_TYPE") = py::int_(LIDAR_VERSION_PACKET_TYPE);

    m.attr("USER_CMD_RESET_TYPE") = py::int_(USER_CMD_RESET_TYPE);
    m.attr("USER_CMD_STANDBY_TYPE") = py::int_(USER_CMD_STANDBY_TYPE);
    m.attr("USER_CMD_VERSION_GET") = py::int_(USER_CMD_VERSION_GET);
    m.attr("USER_CMD_LATENCY_TYPE") = py::int_(USER_CMD_LATENCY_TYPE);
    m.attr("USER_CMD_CONFIG_RESET") = py::int_(USER_CMD_CONFIG_RESET);
    m.attr("USER_CMD_CONFIG_GET") = py::int_(USER_CMD_CONFIG_GET);
    m.attr("USER_CMD_CONFIG_AUTO_STANDBY") = py::int_(USER_CMD_CONFIG_AUTO_STANDBY);
}
