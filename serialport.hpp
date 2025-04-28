// SerialPort.hpp
#ifndef SERIAL_PORT_HPP
#define SERIAL_PORT_HPP

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cerrno>
#include <stdexcept>
#include <poll.h>
#include <string.h>
#include <system_error>

class SerialException : public std::runtime_error {
public:
    explicit SerialException(const std::string& msg) : std::runtime_error(msg) {}
};

class SerialPort {
    int fd = -1;
    std::string port;
    
public:
    // 串口配置结构体
    struct Config {
        speed_t baudrate;
        uint8_t data_bits;
        uint8_t stop_bits;
        char parity;
        bool flow_control;
        bool clear_buffer;
        uint8_t vmin;
        uint8_t vtime;

        Config(): baudrate(B9600), data_bits(8), stop_bits(1), parity('N'),
              flow_control(false), clear_buffer(true), vmin(1), vtime(0) {}
    };

    SerialPort() = default;

    explicit SerialPort(const std::string& portName, const Config& config = Config()) {
        open(portName, config);
    }

    ~SerialPort() {
        close();
    }

    void open(const std::string& portName, const Config& config = Config()) {
        if (isOpen()) close();

        fd = ::open(portName.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        if (fd < 0) {
            throw SerialException("Failed to open port " + portName + ": " + strerror(errno));
        }
        
        // clearBothBuffers();

        try {
            configure(config);
            port = portName;
        } catch (...) {
            ::close(fd);
            fd = -1;
            throw;
        }
    }

    bool isOpen() const {
        return fd >= 0;
    }

    void close() {
        if (isOpen()) {
            ::close(fd);
            fd = -1;
            port.clear();
        }
    }

    void configure(const Config& config) {
        if (!isOpen()) {
            throw SerialException("Port not open");
        }

        termios tty{};
        if (tcgetattr(fd, &tty) != 0) {
            throw SerialException("tcgetattr failed: " + std::string(strerror(errno)));
        }

        // 设置输入输出波特率
        cfsetispeed(&tty, config.baudrate);
        cfsetospeed(&tty, config.baudrate);

        // 控制模式标志
        tty.c_cflag &= ~PARENB; // 清除奇偶校验使能
        tty.c_cflag &= ~CSTOPB; // 清除双停止位
        tty.c_cflag &= ~CSIZE;  // 清除数据位掩码
        tty.c_cflag |= (CLOCAL | CREAD); // 保持程序对端口的独占访问

        // 数据位
        switch (config.data_bits) {
            case 5: tty.c_cflag |= CS5; break;
            case 6: tty.c_cflag |= CS6; break;
            case 7: tty.c_cflag |= CS7; break;
            case 8: tty.c_cflag |= CS8; break;
            default: throw SerialException("Invalid data bits: " + std::to_string(config.data_bits));
        }

        // 奇偶校验
        switch (config.parity) {
            case 'N': break;
            case 'E': tty.c_cflag |= PARENB; break;
            case 'O': tty.c_cflag |= (PARENB | PARODD); break;
            default: throw SerialException("Invalid parity: " + std::string(1, config.parity));
        }

        // 停止位
        if (config.stop_bits == 2) {
            tty.c_cflag |= CSTOPB;
        } else if (config.stop_bits != 1) {
            throw SerialException("Invalid stop bits: " + std::to_string(config.stop_bits));
        }

        // 流控制
        if (config.flow_control) {
            tty.c_cflag |= CRTSCTS;
        } else {
            tty.c_cflag &= ~CRTSCTS;
        }

        // 本地模式标志
        tty.c_lflag = 0; // 禁用规范输入，回显等

        // 输入模式标志
        tty.c_iflag = 0;
        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // 禁用软件流控制
        tty.c_iflag &= ~(INLCR | ICRNL); // 禁用NL/CR转换

        // 输出模式标志
        tty.c_oflag = 0; // 禁用输出处理

        // 超时设置
        tty.c_cc[VMIN] = config.vmin;
        tty.c_cc[VTIME] = config.vtime;

        if(config.clear_buffer) {
            cfsetispeed(&tty, B50);
            cfsetospeed(&tty, B50);
            if (tcsetattr(fd, TCSANOW, &tty) != 0) {
                throw SerialException("tcsetattr failed: " + std::string(strerror(errno)));
            }
            cfsetispeed(&tty, config.baudrate);
            cfsetospeed(&tty, config.baudrate);
        }

        if (tcsetattr(fd, TCSANOW, &tty) != 0) {
            throw SerialException("tcsetattr failed: " + std::string(strerror(errno)));
        }
    }

    size_t write(const uint8_t* data, size_t length) {
        if (!isOpen()) {
            throw SerialException("Port not open");
        }

        ssize_t result = ::write(fd, data, length);
        if (result < 0) {
            throw SerialException("Write failed: " + std::string(strerror(errno)));
        }
        return static_cast<size_t>(result);
    }

    void clearBothBuffers() {
        if (!isOpen()) {
            throw SerialException("Port not open");
        }
        usleep(200);
        if (tcflush(fd, TCIOFLUSH) != 0) {
            throw SerialException("Clear buffers failed: " + std::string(strerror(errno)));
        }
    }

    bool waitForReadyRead(int timeout_ms = -1) {
        if (!isOpen()) {
            throw SerialException("Port not open");
        }

        struct pollfd pfd;
        pfd.fd = fd;
        pfd.events = POLLIN;

        int ret = poll(&pfd, 1, timeout_ms);
        if (ret < 0) {
            throw SerialException("poll failed: " + std::string(strerror(errno)));
        }
        
        if (ret == 0) return false; // 超时
        
        // 检查是否有错误事件
        if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) {
            throw SerialException("Device error detected");
        }
        
        return (pfd.revents & POLLIN) != 0;
    }

    size_t read(uint8_t* buffer, size_t max_length) {
        if (!isOpen()) {
            throw SerialException("Port not open");
        }

        ssize_t result = ::read(fd, buffer, max_length);
        if (result < 0) {
            throw SerialException("Read failed: " + std::string(strerror(errno)));
        }
        return static_cast<size_t>(result);
    }

    // 便捷方法
    void writeString(const std::string& str) {
        write(reinterpret_cast<const uint8_t*>(str.data()), str.size());
    }

    std::string readString(size_t max_length = 256) {
        std::string buffer(max_length, '\0');
        size_t read_bytes = read(reinterpret_cast<uint8_t*>(&buffer[0]), max_length);
        buffer.resize(read_bytes);
        return buffer;
    }

    // 禁止拷贝
    SerialPort(const SerialPort&) = delete;
    SerialPort& operator=(const SerialPort&) = delete;
};

#endif // SERIAL_PORT_HPP
