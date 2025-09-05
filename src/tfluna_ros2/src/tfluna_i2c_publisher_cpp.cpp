/**
 * @file tfluna_i2c_publisher_cpp.cpp
 * @brief High-performance C++ TF-Luna LiDAR I2C publisher for ROS2
 * 
 * Features:
 * - Optimized I2C communication with error handling
 * - Dual publishing: sensor_msgs/Range and sensor_msgs/PointCloud2
 * - Foxglove-optimized PointCloud2 with intensity and temperature fields
 * - RAII resource management
 * - Configurable parameters
 * - Robust error recovery
 * - Multi-sensor support via I2C addressing
 * 
 * @author Generated for Jetson Orin Nano
 * @date 2024
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <std_msgs/msg/header.hpp>

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <thread>

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
 * @brief RAII I2C device wrapper
 */
class I2CDevice
{
public:
    explicit I2CDevice(int bus_number, uint8_t device_address)
        : bus_number_(bus_number), device_address_(device_address), fd_(-1)
    {
        open();
    }
    
    ~I2CDevice()
    {
        close();
    }
    
    // Non-copyable, movable
    I2CDevice(const I2CDevice&) = delete;
    I2CDevice& operator=(const I2CDevice&) = delete;
    I2CDevice(I2CDevice&& other) noexcept 
        : bus_number_(other.bus_number_), device_address_(other.device_address_), fd_(other.fd_)
    {
        other.fd_ = -1;
    }
    
    bool is_open() const { return fd_ >= 0; }
    
    /**
     * @brief Read data from I2C device
     */
    bool read_data(uint8_t* buffer, size_t length)
    {
        if (!is_open()) return false;
        
        ssize_t result = ::read(fd_, buffer, length);
        return result == static_cast<ssize_t>(length);
    }
    
    /**
     * @brief Write command to I2C device
     */
    bool write_command(uint8_t command)
    {
        if (!is_open()) return false;
        
        return ::write(fd_, &command, 1) == 1;
    }
    
    /**
     * @brief Read register from I2C device (Arduino Wire library style)
     */
    int read_register(uint8_t reg)
    {
        if (!is_open()) return -1;
        
        // Step 1: Write register address (like Wire.beginTransmission + Wire.write + Wire.endTransmission)
        if (::write(fd_, &reg, 1) != 1) {
            return -1;
        }
        
        // Step 2: Read one byte from that register (like Wire.requestFrom + Wire.read)
        uint8_t data;
        if (::read(fd_, &data, 1) == 1) {
            return static_cast<int>(data);
        }
        
        return -1;
    }

private:
    void open()
    {
        std::string device_path = "/dev/i2c-" + std::to_string(bus_number_);
        fd_ = ::open(device_path.c_str(), O_RDWR);
        
        if (fd_ < 0) {
            throw std::runtime_error("Failed to open I2C device: " + device_path);
        }
        
        // Set I2C slave address
        if (ioctl(fd_, I2C_SLAVE, device_address_) < 0) {
            ::close(fd_);
            fd_ = -1;
            throw std::runtime_error("Failed to set I2C slave address: 0x" + 
                                   std::to_string(device_address_));
        }
    }
    
    void close()
    {
        if (fd_ >= 0) {
            ::close(fd_);
            fd_ = -1;
        }
    }
    
    int bus_number_;
    uint8_t device_address_;
    int fd_;
};

/**
 * @brief High-performance TF-Luna I2C ROS2 publisher
 */
class TFLunaI2CPublisher : public rclcpp::Node
{
public:
    explicit TFLunaI2CPublisher(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node("tfluna_i2c_publisher_cpp", options)
    {
        // Declare parameters with validation
        declare_parameters();
        
        // Get parameters
        get_parameters();
        
        // Validate parameters
        validate_parameters();
        
        // Initialize I2C connection
        try {
            i2c_device_ = std::make_unique<I2CDevice>(i2c_bus_, i2c_address_);
            RCLCPP_INFO(get_logger(), "I2C connection established: bus %d, address 0x%02X", 
                       i2c_bus_, i2c_address_);
        } catch (const std::exception& e) {
            RCLCPP_FATAL(get_logger(), "Failed to initialize I2C device: %s", e.what());
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
        
        RCLCPP_INFO(get_logger(), "TF-Luna I2C C++ publisher started");
        RCLCPP_INFO(get_logger(), "Publishing at %.1f Hz on topics: tfluna/range and tfluna/pointcloud", publish_rate_);
        RCLCPP_INFO(get_logger(), "I2C: Bus %d, Address 0x%02X", i2c_bus_, i2c_address_);
    }

private:
    void declare_parameters()
    {
        // I2C connection parameters
        declare_parameter("i2c_bus", 7);  // I2C1 on Jetson Orin Nano (pins 3/5)
        declare_parameter("i2c_address", 0x10);  // Default TF-Luna I2C address
        
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
        i2c_bus_ = get_parameter("i2c_bus").as_int();
        i2c_address_ = static_cast<uint8_t>(get_parameter("i2c_address").as_int());
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
        if (i2c_bus_ < 0 || i2c_bus_ > 255) {
            throw std::invalid_argument("Invalid I2C bus number");
        }
        if (i2c_address_ < 0x08 || i2c_address_ > 0x77) {
            throw std::invalid_argument("Invalid I2C address (must be 0x08-0x77)");
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
        try {
            // Based on Arduino TFLI2C library - read individual registers
            // TFL_DIST_LO=0x00, TFL_DIST_HI=0x01, TFL_FLUX_LO=0x02, TFL_FLUX_HI=0x03, 
            // TFL_TEMP_LO=0x04, TFL_TEMP_HI=0x05
            constexpr uint8_t TFL_DIST_LO = 0x00;
            constexpr uint8_t TFL_DIST_HI = 0x01;
            constexpr uint8_t TFL_FLUX_LO = 0x02;
            constexpr uint8_t TFL_FLUX_HI = 0x03;
            constexpr uint8_t TFL_TEMP_LO = 0x04;
            constexpr uint8_t TFL_TEMP_HI = 0x05;
            
            for (int attempt = 0; attempt < max_read_attempts_; ++attempt) {
                // Read individual registers like the Arduino library does
                std::array<uint8_t, 6> dataArray{};
                bool read_success = true;
                
                // Read distance registers
                int dist_lo = i2c_device_->read_register(TFL_DIST_LO);
                int dist_hi = i2c_device_->read_register(TFL_DIST_HI);
                if (dist_lo < 0 || dist_hi < 0) {
                    read_success = false;
                } else {
                    dataArray[0] = static_cast<uint8_t>(dist_lo);
                    dataArray[1] = static_cast<uint8_t>(dist_hi);
                }
                
                // Read signal strength registers
                int flux_lo = i2c_device_->read_register(TFL_FLUX_LO);
                int flux_hi = i2c_device_->read_register(TFL_FLUX_HI);
                if (flux_lo < 0 || flux_hi < 0) {
                    read_success = false;
                } else {
                    dataArray[2] = static_cast<uint8_t>(flux_lo);
                    dataArray[3] = static_cast<uint8_t>(flux_hi);
                }
                
                // Read temperature registers
                int temp_lo = i2c_device_->read_register(TFL_TEMP_LO);
                int temp_hi = i2c_device_->read_register(TFL_TEMP_HI);
                if (temp_lo < 0 || temp_hi < 0) {
                    read_success = false;
                } else {
                    dataArray[4] = static_cast<uint8_t>(temp_lo);
                    dataArray[5] = static_cast<uint8_t>(temp_hi);
                }
                
                if (read_success) {
                    // Parse data exactly like Arduino library
                    uint16_t distance_cm = dataArray[0] + (dataArray[1] << 8);
                    uint16_t strength = dataArray[2] + (dataArray[3] << 8);
                    uint16_t temp_raw = dataArray[4] + (dataArray[5] << 8);
                    
                    // Convert distance to meters
                    float distance_m = distance_cm / 100.0f;
                    
                    // Temperature conversion (from Arduino library - raw value in 0.01 degrees C)
                    float temperature_c = temp_raw / 100.0f;
                    
                    // Signal strength validation (from Arduino library)
                    if (strength >= 100 && strength != 0xFFFF) {
                        // Valid reading
                        if (distance_cm > 0 && distance_cm < 65535) {
                            return TFLunaData(distance_m, strength, temperature_c);
                        }
                    }
                }
                
                // Small delay between attempts
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        } catch (const std::exception& e) {
            RCLCPP_DEBUG(get_logger(), "Error reading I2C sensor data: %s", e.what());
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
    int i2c_bus_;
    uint8_t i2c_address_;
    std::string frame_id_;
    double publish_rate_;
    double field_of_view_;
    double min_range_;
    double max_range_;
    int read_timeout_ms_;
    int max_read_attempts_;
    
    // Hardware
    std::unique_ptr<I2CDevice> i2c_device_;
    
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
RCLCPP_COMPONENTS_REGISTER_NODE(tfluna_ros2::TFLunaI2CPublisher)

/**
 * @brief Main entry point for standalone execution
 */
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<tfluna_ros2::TFLunaI2CPublisher>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("tfluna_i2c_publisher_cpp"), "Fatal error: %s", e.what());
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}
