/**
 * ============================================================================
 * SerialPort — Pure POSIX serial port wrapper (no ROS dependency)
 * ============================================================================
 *
 * serial::Serial (ROS package) 대체용 POSIX termios 구현체.
 * Linux 전용. open/close/read/write/available/flush 제공.
 *
 * ============================================================================
 */

#ifndef MOBILE_SDK__SERIAL_PORT_HPP_
#define MOBILE_SDK__SERIAL_PORT_HPP_

#include <cstddef>
#include <cstdint>
#include <string>

namespace mobile_sdk
{

class SerialPort
{
public:
    SerialPort();
    ~SerialPort();

    // Non-copyable, movable
    SerialPort(const SerialPort&) = delete;
    SerialPort& operator=(const SerialPort&) = delete;
    SerialPort(SerialPort&& other) noexcept;
    SerialPort& operator=(SerialPort&& other) noexcept;

    /**
     * @brief Open serial port with given device path and baudrate.
     * @return true on success.
     */
    bool open(const std::string& port, int baudrate = 19200);

    /**
     * @brief Close serial port.
     */
    void close();

    /**
     * @brief Check if port is open.
     */
    bool isOpen() const;

    /**
     * @brief Number of bytes available for reading.
     */
    size_t available() const;

    /**
     * @brief Read up to @p size bytes into @p buffer.
     * @return Number of bytes actually read, or -1 on error.
     */
    ssize_t read(uint8_t* buffer, size_t size) const;

    /**
     * @brief Write @p size bytes from @p buffer.
     * @return Number of bytes actually written, or -1 on error.
     */
    ssize_t write(const uint8_t* buffer, size_t size) const;

    /**
     * @brief Flush output buffer (tcdrain).
     */
    void flush() const;

    /**
     * @brief Get the port path used for open().
     */
    const std::string& port() const { return port_; }

private:
    int fd_;
    std::string port_;

    /// Convert integer baudrate to termios speed constant.
    /// Returns the baud constant (unsigned int) for use with cfsetispeed/cfsetospeed.
    static unsigned int baudrateToSpeed(int baudrate);
};

}  // namespace mobile_sdk

#endif  // MOBILE_SDK__SERIAL_PORT_HPP_
