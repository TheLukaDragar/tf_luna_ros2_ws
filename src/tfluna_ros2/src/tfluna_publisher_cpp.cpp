/**
 * @file tfluna_publisher_cpp.cpp
 * @brief High-performance C++ TF-Luna LiDAR publisher for ROS2
 * 
 * Features:
 * - Optimized serial communication with error handling
 * - Dual publishing: sensor_msgs/Range and sensor_msgs/PointCloud2
 * - Foxglove-optimized PointCloud2 with intensity and temperature fields
 * - RAII resource management
 * - Configurable parameters
 * - Robust error recovery
 * 
 * @author Generated for Jetson Orin Nano
 * @date 2024
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/serial.h>

#include <array>
#include <vector>
#include <string>
#include <memory>
#include <chrono>
#include <cstring>
#include <algorithm>

namespace tfluna_ros2
{

/**
 * @brief TF-Luna sensor data structure
 */
struct TFLunaData
{
    float distance_m{0.0f};
    uint16_t strength{0};
    float temperature_c{0.0f};
    bool valid{false};
    
    TFLunaData() = default;
    TFLunaData(float dist, uint16_t str, float temp) 
        : distance_m(dist), strength(str), temperature_c(temp), valid(true) {}
};

/**
 * @brief RAII Serial port wrapper
 */
class SerialPort
{
public:
    explicit SerialPort(const std::string& device, int baudrate)
        : device_(device), baudrate_(baudrate), fd_(-1)
    {
        open();
    }
    
    ~SerialPort()
    {
        close();
    }
    
    // Non-copyable, movable
    SerialPort(const SerialPort&) = delete;
    SerialPort& operator=(const SerialPort&) = delete;
    SerialPort(SerialPort&& other) noexcept : device_(std::move(other.device_)), 
                                              baudrate_(other.baudrate_), fd_(other.fd_)
    {
        other.fd_ = -1;
    }
    
    bool is_open() const { return fd_ >= 0; }
    
    int get_fd() const { return fd_; }
    
    ssize_t read(void* buffer, size_t size)
    {
        if (!is_open()) return -1;
        return ::read(fd_, buffer, size);
    }
    
    void flush_input()
    {
        if (is_open()) {
            tcflush(fd_, TCIFLUSH);
        }
    }

private:
    void open()
    {
        fd_ = ::open(device_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd_ < 0) {
            throw std::runtime_error("Failed to open serial port: " + device_);
        }
        
        // Configure serial port
        struct termios tty{};
        if (tcgetattr(fd_, &tty) != 0) {
            ::close(fd_);
            fd_ = -1;
            throw std::runtime_error("Failed to get serial attributes");
        }
        
        // Set baud rate
        speed_t speed;
        switch (baudrate_) {
            case 9600: speed = B9600; break;
            case 19200: speed = B19200; break;
            case 38400: speed = B38400; break;
            case 57600: speed = B57600; break;
            case 115200: speed = B115200; break;
            case 230400: speed = B230400; break;
            default:
                ::close(fd_);
                fd_ = -1;
                throw std::runtime_error("Unsupported baud rate: " + std::to_string(baudrate_));
        }
        
        cfsetispeed(&tty, speed);
        cfsetospeed(&tty, speed);
        
        // 8N1 configuration
        tty.c_cflag &= ~PARENB;   // No parity
        tty.c_cflag &= ~CSTOPB;   // 1 stop bit
        tty.c_cflag &= ~CSIZE;    // Clear size bits
        tty.c_cflag |= CS8;       // 8 data bits
        tty.c_cflag &= ~CRTSCTS;  // No hardware flow control
        tty.c_cflag |= CREAD | CLOCAL; // Enable receiver, ignore modem control lines
        
        // Raw input mode
        tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        tty.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL);
        tty.c_oflag &= ~OPOST;
        
        // Non-blocking read with timeout
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 1; // 0.1 second timeout
        
        if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
            ::close(fd_);
            fd_ = -1;
            throw std::runtime_error("Failed to set serial attributes");
        }
        
        // Flush buffers
        tcflush(fd_, TCIOFLUSH);
    }
    
    void close()
    {
        if (fd_ >= 0) {
            ::close(fd_);
            fd_ = -1;
        }
    }
    
    std::string device_;
    int baudrate_;
    int fd_;
};

/**
 * @brief High-performance TF-Luna ROS2 publisher
 */
class TFLunaPublisher : public rclcpp::Node
{
public:
    explicit TFLunaPublisher(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node("tfluna_publisher_cpp", options)
    {
        // Declare parameters with validation
        declare_parameters();
        
        // Get parameters
        get_parameters();
        
        // Validate parameters
        validate_parameters();
        
        // Initialize serial connection
        try {
            serial_port_ = std::make_unique<SerialPort>(serial_port_name_, baud_rate_);
            RCLCPP_INFO(get_logger(), "Serial connection established: %s", serial_port_name_.c_str());
        } catch (const std::exception& e) {
            RCLCPP_FATAL(get_logger(), "Failed to initialize serial port: %s", e.what());
            throw;
        }
        
        // Create publishers with Foxglove-compatible QoS
        auto qos = rclcpp::QoS(10)
            .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
            .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE)
            .history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);
        
        range_publisher_ = create_publisher<sensor_msgs::msg::Range>("tfluna/range", qos);
        pointcloud_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("tfluna/pointcloud", qos);
        
        // Create timer with high precision
        auto timer_period = std::chrono::duration<double>(1.0 / publish_rate_);
        timer_ = create_wall_timer(timer_period, [this]() { timer_callback(); });
        
        // Initialize message templates for performance
        initialize_message_templates();
        
        RCLCPP_INFO(get_logger(), "TF-Luna C++ publisher started");
        RCLCPP_INFO(get_logger(), "Publishing at %.1f Hz on topics: tfluna/range and tfluna/pointcloud", publish_rate_);
        RCLCPP_INFO(get_logger(), "Serial: %s @ %d baud", serial_port_name_.c_str(), baud_rate_);
    }

private:
    void declare_parameters()
    {
        // Serial connection parameters
        declare_parameter("serial_port", "/dev/ttyTHS2");
        declare_parameter("baud_rate", 115200);
        
        // ROS2 parameters
        declare_parameter("frame_id", "tfluna_link");
        declare_parameter("publish_rate", 50.0);
        
        // Sensor specifications (TF-Luna datasheet)
        declare_parameter("field_of_view", 0.0349);  // 2 degrees in radians
        declare_parameter("min_range", 0.2);         // meters
        declare_parameter("max_range", 8.0);         // meters at 90% reflectivity
        
        // Performance parameters
        declare_parameter("read_timeout_ms", 100);
        declare_parameter("max_read_attempts", 3);
    }
    
    void get_parameters()
    {
        serial_port_name_ = get_parameter("serial_port").as_string();
        baud_rate_ = get_parameter("baud_rate").as_int();
        frame_id_ = get_parameter("frame_id").as_string();
        publish_rate_ = get_parameter("publish_rate").as_double();
        field_of_view_ = get_parameter("field_of_view").as_double();
        min_range_ = get_parameter("min_range").as_double();
        max_range_ = get_parameter("max_range").as_double();
        read_timeout_ms_ = get_parameter("read_timeout_ms").as_int();
        max_read_attempts_ = get_parameter("max_read_attempts").as_int();
    }
    
    void validate_parameters()
    {
        if (publish_rate_ <= 0.0 || publish_rate_ > 250.0) {
            throw std::invalid_argument("publish_rate must be between 0 and 250 Hz");
        }
        if (min_range_ < 0.0 || max_range_ <= min_range_) {
            throw std::invalid_argument("Invalid range parameters");
        }
        if (baud_rate_ <= 0) {
            throw std::invalid_argument("Invalid baud rate");
        }
    }
    
    void initialize_message_templates()
    {
        // Pre-configure Range message template
        range_template_.header.frame_id = frame_id_;
        range_template_.radiation_type = sensor_msgs::msg::Range::INFRARED;
        range_template_.field_of_view = static_cast<float>(field_of_view_);
        range_template_.min_range = static_cast<float>(min_range_);
        range_template_.max_range = static_cast<float>(max_range_);
        
        // Pre-configure PointCloud2 message template
        pointcloud_template_.header.frame_id = frame_id_;
        pointcloud_template_.height = 1;
        pointcloud_template_.width = 1;
        pointcloud_template_.is_bigendian = false;
        pointcloud_template_.point_step = 20; // 5 fields * 4 bytes each
        pointcloud_template_.row_step = 20;
        pointcloud_template_.is_dense = true;
        
        // Define point cloud fields
        pointcloud_template_.fields.resize(5);
        pointcloud_template_.fields[0].name = "x";
        pointcloud_template_.fields[0].offset = 0;
        pointcloud_template_.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
        pointcloud_template_.fields[0].count = 1;
        
        pointcloud_template_.fields[1].name = "y";
        pointcloud_template_.fields[1].offset = 4;
        pointcloud_template_.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
        pointcloud_template_.fields[1].count = 1;
        
        pointcloud_template_.fields[2].name = "z";
        pointcloud_template_.fields[2].offset = 8;
        pointcloud_template_.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
        pointcloud_template_.fields[2].count = 1;
        
        pointcloud_template_.fields[3].name = "intensity";
        pointcloud_template_.fields[3].offset = 12;
        pointcloud_template_.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
        pointcloud_template_.fields[3].count = 1;
        
        pointcloud_template_.fields[4].name = "temperature";
        pointcloud_template_.fields[4].offset = 16;
        pointcloud_template_.fields[4].datatype = sensor_msgs::msg::PointField::FLOAT32;
        pointcloud_template_.fields[4].count = 1;
        
        // Pre-allocate data buffer
        pointcloud_template_.data.resize(20);
    }
    
    void timer_callback()
    {
        auto sensor_data = read_tfluna_data();
        
        if (!sensor_data.valid) {
            return;
        }
        
        auto timestamp = now();
        
        // Publish Range message
        publish_range_message(sensor_data, timestamp);
        
        // Publish PointCloud2 message
        publish_pointcloud_message(sensor_data, timestamp);
        
        // Log occasionally for monitoring
        log_data_occasionally(sensor_data);
    }
    
    TFLunaData read_tfluna_data()
    {
        constexpr size_t PACKET_SIZE = 9;
        constexpr uint8_t HEADER1 = 0x59;
        constexpr uint8_t HEADER2 = 0x59;
        
        try {
            // Check if enough data is available (similar to Python's in_waiting > 8)
            int bytes_available = 0;
            if (ioctl(serial_port_->get_fd(), FIONREAD, &bytes_available) == 0 && bytes_available > 8) {
                
                // Read exactly 9 bytes (same as Python approach)
                std::array<uint8_t, PACKET_SIZE> buffer{};
                ssize_t bytes_read = serial_port_->read(buffer.data(), PACKET_SIZE);
                
                if (bytes_read == PACKET_SIZE) {
                    // Clear any remaining data in buffer (same as Python reset_input_buffer)
                    serial_port_->flush_input();
                    
                    // Check for valid header (same as Python)
                    if (buffer[0] == HEADER1 && buffer[1] == HEADER2) {
                        // Extract distance (same as Python: bytes[2] + bytes[3] * 256)
                        uint16_t distance_cm = buffer[2] + buffer[3] * 256;
                        float distance_m = distance_cm / 100.0f;
                        
                        // Extract signal strength (same as Python: bytes[4] + bytes[5] * 256)
                        uint16_t strength = buffer[4] + buffer[5] * 256;
                        
                        // Extract temperature (same as Python: (temp_raw / 8.0) - 256.0)
                        uint16_t temp_raw = buffer[6] + buffer[7] * 256;
                        float temperature_c = (temp_raw / 8.0f) - 256.0f;
                        
                        // Basic validation (avoid obviously invalid readings)
                        if (distance_cm > 0 && distance_cm < 65535 && strength > 0) {
                            return TFLunaData(distance_m, strength, temperature_c);
                        }
                    }
                }
            }
        } catch (const std::exception& e) {
            RCLCPP_DEBUG(get_logger(), "Error reading sensor data: %s", e.what());
        }
        
        return TFLunaData{}; // Invalid data
    }
    
    void publish_range_message(const TFLunaData& data, const rclcpp::Time& timestamp)
    {
        auto range_msg = range_template_; // Copy template for performance
        range_msg.header.stamp = timestamp;
        
        // Handle out-of-range measurements per REP-117
        if (data.distance_m < min_range_) {
            range_msg.range = -std::numeric_limits<float>::infinity();
        } else if (data.distance_m > max_range_) {
            range_msg.range = std::numeric_limits<float>::infinity();
        } else {
            range_msg.range = data.distance_m;
        }
        
        range_publisher_->publish(range_msg);
    }
    
    void publish_pointcloud_message(const TFLunaData& data, const rclcpp::Time& timestamp)
    {
        auto pointcloud_msg = pointcloud_template_; // Copy template
        pointcloud_msg.header.stamp = timestamp;
        
        // Point coordinates (sensor points along positive X-axis)
        float x = data.distance_m;
        float y = 0.0f;
        float z = 0.0f;
        
        // Normalize intensity (0-1 range for better visualization)
        float intensity = std::min(data.strength / 65535.0f, 1.0f);
        float temperature = data.temperature_c;
        
        // Pack point data efficiently
        std::memcpy(&pointcloud_msg.data[0], &x, sizeof(float));
        std::memcpy(&pointcloud_msg.data[4], &y, sizeof(float));
        std::memcpy(&pointcloud_msg.data[8], &z, sizeof(float));
        std::memcpy(&pointcloud_msg.data[12], &intensity, sizeof(float));
        std::memcpy(&pointcloud_msg.data[16], &temperature, sizeof(float));
        
        pointcloud_publisher_->publish(pointcloud_msg);
    }
    
    void log_data_occasionally(const TFLunaData& data)
    {
        static auto last_log_time = std::chrono::steady_clock::now();
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(current_time - last_log_time);
        
        if (elapsed.count() >= 2) { // Log every 2 seconds
            RCLCPP_INFO(get_logger(), 
                "Distance: %.2fm, Strength: %u, Temp: %.1f°C", 
                data.distance_m, data.strength, data.temperature_c);
            last_log_time = current_time;
        }
    }
    
    // Parameters
    std::string serial_port_name_;
    int baud_rate_;
    std::string frame_id_;
    double publish_rate_;
    double field_of_view_;
    double min_range_;
    double max_range_;
    int read_timeout_ms_;
    int max_read_attempts_;
    
    // Hardware
    std::unique_ptr<SerialPort> serial_port_;
    
    // ROS2 components
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr range_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // Message templates for performance
    sensor_msgs::msg::Range range_template_;
    sensor_msgs::msg::PointCloud2 pointcloud_template_;
};

} // namespace tfluna_ros2

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(tfluna_ros2::TFLunaPublisher)

/**
 * @brief Main entry point for standalone execution
 */
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<tfluna_ros2::TFLunaPublisher>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("tfluna_publisher_cpp"), "Fatal error: %s", e.what());
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}
