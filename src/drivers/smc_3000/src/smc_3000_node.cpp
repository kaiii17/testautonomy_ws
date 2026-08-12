#include <iostream>
#include <chrono>
#include <functional>
#include <memory>
#include <vector>
#include <string>
#include <sstream>
#include <cstring>

#include <termios.h>
#include <fcntl.h>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>
#include <smc_3000_msgs/msg/drpva_a.hpp>   // DRPVA message
#include <smc_3000_msgs/msg/imuatt_a.hpp>  // IMUATTA message
#include <smc_3000_msgs/msg/nmea.hpp>      // NMEA message (GNGGA)

// CRC32 polynomial used by DRPVA and IMUATTA
#define CRC32_POLYNOMIAL 0xEDB88320L

class MultiProtocolNode : public rclcpp::Node
{
public:
    MultiProtocolNode() : Node("multi_protocol_node")
    {
        // Declare parameters
        this->declare_parameter<std::string>("device", "/dev/ttyAMA1");
        this->declare_parameter<int>("baudrate", 115200);

        this->get_parameter("device", device_);
        this->get_parameter("baudrate", baudrate_);

        RCLCPP_INFO(this->get_logger(), "Device: %s, baudrate: %d", device_.c_str(), baudrate_);

        // Open serial port
        uart_fd_ = open(device_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        if (uart_fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open %s", device_.c_str());
            return;
        }

        // Configure serial port speed
        tcgetattr(uart_fd_, &options_);
        switch (baudrate_)
        {
        case 9600:    options_.c_cflag = B9600 | CS8 | CLOCAL | CREAD; break;
        case 38400:   options_.c_cflag = B38400 | CS8 | CLOCAL | CREAD; break;
        case 115200:  options_.c_cflag = B115200 | CS8 | CLOCAL | CREAD; break;
        case 230400:  options_.c_cflag = B230400 | CS8 | CLOCAL | CREAD; break;
        case 460800:  options_.c_cflag = B460800 | CS8 | CLOCAL | CREAD; break;
        case 921600:  options_.c_cflag = B921600 | CS8 | CLOCAL | CREAD; break;
        default:
            // Unknown baudrate, defaulting to 115200
            options_.c_cflag = B115200 | CS8 | CLOCAL | CREAD;
            break;
        }
        options_.c_iflag = IGNPAR;
        options_.c_oflag = 0;
        options_.c_lflag = 0;
        tcflush(uart_fd_, TCIFLUSH);
        tcsetattr(uart_fd_, TCSANOW, &options_);
        RCLCPP_INFO(this->get_logger(), "Serial port ready.");

        // Create publishers for DRPVA, IMUATTA, and NMEA
        drpva_pub_   = this->create_publisher<smc_3000_msgs::msg::DrpvaA>("smc3000/drpva", 10);
        imuatta_pub_ = this->create_publisher<smc_3000_msgs::msg::ImuattA>("smc3000/imuatta", 10);
        nmea_pub_    = this->create_publisher<smc_3000_msgs::msg::Nmea>("smc3000/gngga", 10);

        // Read serial data every 1ms
        uart_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1),
            std::bind(&MultiProtocolNode::readSerialData, this));

        RCLCPP_INFO(this->get_logger(), "Node constructed and timer started.");
    }

    ~MultiProtocolNode()
    {
        if (uart_fd_ >= 0) {
            close(uart_fd_);
        }
    }

private:
    // Serial/parameter members
    std::string device_;
    int baudrate_;
    int uart_fd_;
    struct termios options_;

    // Receive buffer
    std::string buffer_;

    // Message objects
    smc_3000_msgs::msg::DrpvaA  drpva_msg_;
    smc_3000_msgs::msg::ImuattA imuatta_msg_;
    smc_3000_msgs::msg::Nmea    nmea_msg_;

    // Publishers
    rclcpp::Publisher<smc_3000_msgs::msg::DrpvaA>::SharedPtr  drpva_pub_;
    rclcpp::Publisher<smc_3000_msgs::msg::ImuattA>::SharedPtr imuatta_pub_;
    rclcpp::Publisher<smc_3000_msgs::msg::Nmea>::SharedPtr    nmea_pub_;

    // Timer for periodic serial reading
    rclcpp::TimerBase::SharedPtr uart_timer_;

    // Periodically read serial data and attempt to parse three message types
    void readSerialData()
    {
        char buff[1024];
        int n = read(uart_fd_, buff, sizeof(buff)-1);
        if (n > 0)
        {
            buff[n] = '\0';
            RCLCPP_INFO(this->get_logger(), "Received: %s", buff);
            
            buffer_ += buff;
            RCLCPP_DEBUG(this->get_logger(), "Current buffer: '%s'", buffer_.c_str());

            while (true)
            {
                // Find start position of a message (# or $)
                std::size_t startPos = buffer_.find_first_of("#$");
                if (startPos == std::string::npos)
                {
                    buffer_.clear();
                    break;
                }

                char msgType = buffer_[startPos];
                bool parsed = false;
                std::size_t endPos = std::string::npos;
                std::string message;

                if (msgType == '#')
                {
                    // DRPVA or IMUATTA messages end with '*'
                    endPos = buffer_.find("*", startPos);
                    if (endPos == std::string::npos)
                        break;

                    // Include CRC (8 hex digits) and following delimiter
                    endPos += 1 + 8;
                    if (endPos > buffer_.size())
                        break;

                    message = buffer_.substr(startPos, endPos - startPos);
                }
                else if (msgType == '$')
                {
                    // GNGGA messages end with '*'
                    endPos = buffer_.find("*", startPos);
                    if (endPos == std::string::npos)
                        break;

                    // Include XOR checksum (2 hex digits) and following delimiter
                    endPos += 1 + 2;
                    if (endPos > buffer_.size())
                        break;

                    message = buffer_.substr(startPos, endPos - startPos);
                }

                if (msgType == '#')
                {
                    if (message.find("DRPVAA") == 1)
                    {
                        if(parseDrpvaData(message))
                        {
                            drpva_pub_->publish(drpva_msg_);
                            RCLCPP_DEBUG(this->get_logger(), "Parsed DRPVA message.");
                            parsed = true;
                        }
                    }
                    else if (message.find("IMUATTA") == 1)
                    {
                        if(parseImuattAData(message))
                        {
                            imuatta_pub_->publish(imuatta_msg_);
                            RCLCPP_DEBUG(this->get_logger(), "Parsed IMUATTA message.");
                            parsed = true;
                        }
                    }
                }
                else if (msgType == '$')
                {
                    if(parseGnggaData(message))
                    {
                        nmea_pub_->publish(nmea_msg_);
                        RCLCPP_DEBUG(this->get_logger(), "Parsed GNGGA message.");
                        parsed = true;
                    }
                }

                if (parsed)
                {
                    buffer_ = buffer_.substr(endPos);
                }
                else
                {
                    buffer_ = buffer_.substr(startPos + 1);
                }

                const size_t MAX_BUFFER_SIZE = 4096;
                if (buffer_.size() > MAX_BUFFER_SIZE) {
                    buffer_.clear();
                    break;
                }
            }

            RCLCPP_DEBUG(this->get_logger(), "Remaining buffer: '%s'", buffer_.c_str());
        }
    }

    // Parse DRPVA message (CRC32)
    bool parseDrpvaData(const std::string& message)
    {
        if (message.find("DRPVAA") != 1) {
            return false;
        }

        std::size_t starPos = message.find("*");
        if (starPos == std::string::npos || starPos + 9 > message.size()) {
            return false;
        }

        std::string payload = message.substr(1, starPos - 1);
        unsigned long calcCrc = calculateBlockCRC32(payload.size(), payload.c_str());

        std::string checkStr = message.substr(starPos + 1, 8);
        unsigned long providedCrc = 0;
        {
            std::stringstream ss;
            ss << std::hex << checkStr;
            ss >> providedCrc;
        }

        if (calcCrc != providedCrc) {
            return false;
        }

        std::vector<std::string> splittedData;
        std::stringstream ss(payload);
        std::string item;
        while (std::getline(ss, item, ',')) {
            size_t semicolonPos = item.find(';');
            if (semicolonPos != std::string::npos) {
                std::string beforeSem = item.substr(0, semicolonPos);
                std::string afterSem = item.substr(semicolonPos + 1);
                if (!beforeSem.empty()) splittedData.push_back(beforeSem);
                if (!afterSem.empty()) splittedData.push_back(afterSem);
            }
            else {
                splittedData.push_back(item);
            }
        }

        if (splittedData.empty() || splittedData[0] != "DRPVAA") {
            return false;
        }

        try {
            int idx = 0;
            drpva_msg_.header.sync = splittedData[idx++];           // "DRPVAA"
            drpva_msg_.header.message = splittedData[idx++];
            drpva_msg_.header.port = std::stoi(splittedData[idx++]);
            drpva_msg_.header.idle_time = std::stof(splittedData[idx++]);
            drpva_msg_.header.time_status = splittedData[idx++];
            drpva_msg_.header.week = std::stoul(splittedData[idx++]);
            drpva_msg_.header.seconds = std::stof(splittedData[idx++]);
            drpva_msg_.header.receiver_status = std::stoul(splittedData[idx++]);
            drpva_msg_.header.bds_time = std::stoul(splittedData[idx++]);
            drpva_msg_.header.utc_time = std::stol(splittedData[idx++]);

            drpva_msg_.sol_status = splittedData[idx++];
            drpva_msg_.pos_type = splittedData[idx++];
            drpva_msg_.datum_id = splittedData[idx++];
            drpva_msg_.reserved[0] = std::stoi(splittedData[idx++]);
            drpva_msg_.reserved[1] = std::stoi(splittedData[idx++]);
            drpva_msg_.reserved[2] = std::stoi(splittedData[idx++]);
            drpva_msg_.reserved[3] = std::stoi(splittedData[idx++]);
            drpva_msg_.dr_age = std::stof(splittedData[idx++]);
            drpva_msg_.sol_age = std::stof(splittedData[idx++]);
            drpva_msg_.lat = std::stod(splittedData[idx++]);
            drpva_msg_.lon = std::stod(splittedData[idx++]);
            drpva_msg_.hgt = std::stod(splittedData[idx++]);
            drpva_msg_.undulation = std::stof(splittedData[idx++]);
            drpva_msg_.lat_sig = std::stof(splittedData[idx++]);
            drpva_msg_.lon_sig = std::stof(splittedData[idx++]);
            drpva_msg_.hgt_sig = std::stof(splittedData[idx++]);
            drpva_msg_.ve = std::stod(splittedData[idx++]);
            drpva_msg_.vn = std::stod(splittedData[idx++]);
            drpva_msg_.vu = std::stod(splittedData[idx++]);
            drpva_msg_.ve_sig = std::stof(splittedData[idx++]);
            drpva_msg_.vn_sig = std::stof(splittedData[idx++]);
            drpva_msg_.vu_sig = std::stof(splittedData[idx++]);
            drpva_msg_.heading = std::stod(splittedData[idx++]);
            drpva_msg_.pitch = std::stod(splittedData[idx++]);
            drpva_msg_.roll = std::stod(splittedData[idx++]);
            drpva_msg_.heading_sig = std::stof(splittedData[idx++]);
            drpva_msg_.pitch_sig = std::stof(splittedData[idx++]);
            drpva_msg_.roll_sig = std::stof(splittedData[idx++]);
            drpva_msg_.reserved1[0] = std::stol(splittedData[idx++]);
            drpva_msg_.reserved1[1] = std::stol(splittedData[idx++]);
            drpva_msg_.reserved1[2] = std::stol(splittedData[idx++]);
            drpva_msg_.reserved1[3] = std::stol(splittedData[idx++]);
            drpva_msg_.reserved2[0] = std::stof(splittedData[idx++]);
            drpva_msg_.reserved2[1] = std::stof(splittedData[idx++]);
            drpva_msg_.reserved2[2] = std::stof(splittedData[idx++]);
            drpva_msg_.reserved2[3] = std::stof(splittedData[idx++]);
            drpva_msg_.reserved3[0] = std::stod(splittedData[idx++]);
            drpva_msg_.reserved3[1] = std::stod(splittedData[idx++]);
            drpva_msg_.reserved3[2] = std::stod(splittedData[idx++]);
            drpva_msg_.reserved3[3] = std::stod(splittedData[idx++]);
        }
        catch(const std::exception& e) {
            return false;
        }

        return true;
    }

    // Parse IMUATTA message (CRC32)
    bool parseImuattAData(const std::string& message)
    {
        if (message.find("IMUATTA") != 1) {
            return false;
        }

        std::size_t starPos = message.find("*");
        if (starPos == std::string::npos || starPos + 9 > message.size()) {
            return false;
        }

        std::string payload = message.substr(1, starPos - 1);
        unsigned long calcCrc = calculateBlockCRC32(payload.size(), payload.c_str());

        std::string checkStr = message.substr(starPos + 1, 8);
        unsigned long providedCrc = 0;
        {
            std::stringstream ss;
            ss << std::hex << checkStr;
            ss >> providedCrc;
        }

        if (calcCrc != providedCrc) {
            return false;
        }

        std::vector<std::string> splittedData;
        std::stringstream ss(payload);
        std::string item;
        while (std::getline(ss, item, ',')) {
            size_t semicolonPos = item.find(';');
            if (semicolonPos != std::string::npos) {
                std::string beforeSem = item.substr(0, semicolonPos);
                std::string afterSem = item.substr(semicolonPos + 1);
                if (!beforeSem.empty()) splittedData.push_back(beforeSem);
                if (!afterSem.empty()) splittedData.push_back(afterSem);
            }
            else {
                splittedData.push_back(item);
            }
        }

        if (splittedData.empty() || splittedData[0] != "IMUATTA") {
            return false;
        }

        try {
            int idx = 0;
            imuatta_msg_.header.sync = splittedData[idx++]; // "IMUATTA"
            imuatta_msg_.header.idle_time = std::stof(splittedData[idx++]);
            imuatta_msg_.header.message = splittedData[idx++];
            imuatta_msg_.header.time_status = splittedData[idx++];
            imuatta_msg_.header.week = std::stoul(splittedData[idx++]);
            imuatta_msg_.header.seconds = std::stof(splittedData[idx++]);
            imuatta_msg_.header.receiver_status = std::stoul(splittedData[idx++]);
            imuatta_msg_.header.port = std::stol(splittedData[idx++]);
            imuatta_msg_.header.utc_time = std::stol(splittedData[idx++]);
            imuatta_msg_.header.bds_time = std::stoul(splittedData[idx++]);

            imuatta_msg_.sol_status = splittedData[idx++];
            imuatta_msg_.pos_type = splittedData[idx++];
            imuatta_msg_.sol_age = std::stof(splittedData[idx++]);
            imuatta_msg_.dr_age = std::stof(splittedData[idx++]);
            imuatta_msg_.roll = std::stod(splittedData[idx++]) * (360.0 / 32767.0);             //unit: 360/32767 [deg]
            imuatta_msg_.pitch = std::stod(splittedData[idx++]) * (360.0 / 32767.0);            //unit: 360/32767 [deg]
            imuatta_msg_.heading = std::stod(splittedData[idx++]) * (360.0 / 32767.0);           //unit: 360/32767 [deg]
            imuatta_msg_.acc_x = std::stod(splittedData[idx++]) * (80.0 / 32767.0);              //unit: 80/32767[m/s^2]
            imuatta_msg_.acc_y = std::stod(splittedData[idx++]) * (80.0 / 32767.0);              //unit: 80/32767[m/s^2]
            imuatta_msg_.acc_z = std::stod(splittedData[idx++]) * (80.0 / 32767.0);              //unit: 80/32767[m/s^2]
            imuatta_msg_.gyro_x = std::stod(splittedData[idx++]) * (500.0 / 32767.0);            //unit: 500/32767[deg/s]
            imuatta_msg_.gyro_y = std::stod(splittedData[idx++]) * (500.0 / 32767.0);            //unit: 500/32767[deg/s]
            imuatta_msg_.gyro_z = std::stod(splittedData[idx++]) * (500.0 / 32767.0);            //unit: 500/32767[deg/s]
            imuatta_msg_.reserved[0] = std::stoi(splittedData[idx++]);
            imuatta_msg_.reserved[1] = std::stoi(splittedData[idx++]);
        }
        catch(const std::exception& e) {
            return false;
        }

        return true;
    }

    // Parse GNGGA message (XOR checksum)
    bool parseGnggaData(const std::string& message)
    {
        if (message.find("GNGGA") != 1) {
            return false;
        }

        std::size_t starPos = message.find("*");
        if (starPos == std::string::npos || starPos + 3 > message.size()) {
            return false;
        }

        std::string payload = message.substr(1, starPos - 1);
        unsigned int calcXor = calculateXorChecksum(payload);

        std::string checkStr = message.substr(starPos + 1, 2);
        unsigned long providedXor = 0;
        {
            std::stringstream ss;
            ss << std::hex << checkStr;
            ss >> providedXor;
        }

        if (calcXor != providedXor) {
            return false;
        }

        std::vector<std::string> splittedData;
        std::stringstream ss(payload);
        std::string item;
        while (std::getline(ss, item, ',')) {
            splittedData.push_back(item);
        }

        if (splittedData.empty() || splittedData[0] != "GNGGA") {
            return false;
        }

        try {
            int idx = 0;
            nmea_msg_.sync = splittedData[idx++];  // "GNGGA"
            nmea_msg_.utc_time = splittedData[idx++];
            nmea_msg_.latitude = std::stod(splittedData[idx++]);
            nmea_msg_.latitude_direction = splittedData[idx++];
            nmea_msg_.longitude = std::stod(splittedData[idx++]);
            nmea_msg_.longitude_direction = splittedData[idx++];
            nmea_msg_.fix_quality = std::stoi(splittedData[idx++]);
            nmea_msg_.num_satellites = std::stoi(splittedData[idx++]);
            nmea_msg_.hdop = std::stod(splittedData[idx++]);
            nmea_msg_.altitude = std::stod(splittedData[idx++]);
            nmea_msg_.altitude_unit = splittedData[idx++];
            nmea_msg_.geoid_separation = std::stod(splittedData[idx++]);
            nmea_msg_.geoid_separation_unit = splittedData[idx++];
            nmea_msg_.dgps_age = std::stod(splittedData[idx++]);
            nmea_msg_.dgps_station_id = std::stoi(splittedData[idx++]);
        }
        catch(const std::exception& e) {
            return false;
        }

        return true;
    }

    // CRC32 helper functions for DRPVA/IMUATTA
    unsigned long crc32Value(int i)
    {
        unsigned long ulCRC = i;
        for(int j = 0; j < 8; j++)
        {
            if(ulCRC & 1)
                ulCRC = (ulCRC >> 1) ^ CRC32_POLYNOMIAL;
            else
                ulCRC >>= 1;
        }
        return ulCRC;
    }

    unsigned long calculateBlockCRC32(unsigned long count, const char* buf)
    {
        unsigned long ulCRC = 0;
        while(count--)
        {
            unsigned long ulTemp1 = (ulCRC >> 8) & 0x00FFFFFFL;
            unsigned long ulTemp2 = crc32Value(((int)ulCRC ^ *buf++) & 0xFF);
            ulCRC = ulTemp1 ^ ulTemp2;
        }
        return ulCRC;
    }

    // XOR checksum for GNGGA
    unsigned int calculateXorChecksum(const std::string& s)
    {
        unsigned int xorVal = 0;
        for(char c : s) {
            xorVal ^= static_cast<unsigned char>(c);
        }
        return xorVal;
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MultiProtocolNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

