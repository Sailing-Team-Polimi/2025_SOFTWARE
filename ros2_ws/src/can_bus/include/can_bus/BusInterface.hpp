#pragma once

#include <vector>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <sstream>
#include <iomanip>
#include <mutex>
#include <string>
#include <algorithm>
#include <chrono>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <cerrno>
#include <system_error>
#include "rclcpp/rclcpp.hpp"

namespace bus
{

    /**
     * @brief Flexible CAN data frame with dynamic header and payload.
     *
     * • Default header size in bytes is DefaultHeaderSize (1 byte).
     * • Header and payload sizes are dynamic and set at runtime.
     */
    class CanPackage
    {
    public:
        /**
         * @brief Default header size (in bytes).
         */
        static constexpr std::size_t DefaultHeaderSize = 1;

        /**
         * @brief Default construct with a zeroed header of DefaultHeaderSize and empty payload.
         */
        CanPackage() = default;

        /**
         * @brief Construct with explicit header and payload.
         *
         * @param header Vector of header bytes.
         * @param payload Vector of payload bytes.
         */
        CanPackage(const std::vector<uint8_t> &header,
                   const std::vector<uint8_t> &payload)
            : header_{header}, payload_{payload} {}

        /**
         * @brief Construct from raw buffer using DefaultHeaderSize.
         *
         * @param buffer Raw byte stream (header + payload).
         */
        explicit CanPackage(const std::vector<uint8_t> &buffer)
        {
            set(buffer, DefaultHeaderSize);
        }

        /**
         * @brief Construct from raw buffer with specified header length.
         *
         * @param buffer Raw byte stream (header + payload).
         * @param headerLen Number of initial bytes to treat as header.
         */
        CanPackage(const std::vector<uint8_t> &buffer, std::size_t headerLen)
        {
            set(buffer, headerLen);
        }

        /**
         * @brief Get a const reference to the header bytes.
         * @return Header vector.
         */
        [[nodiscard]] const std::vector<uint8_t> &getHeader() const noexcept
        {
            return header_;
        }

        /**
         * @brief Get a const reference to the payload bytes.
         * @return Payload vector.
         */
        [[nodiscard]] const std::vector<uint8_t> &getPayload() const noexcept
        {
            return payload_;
        }

        /**
         * @brief Get the header size in bytes.
         * @return header_.size().
         */
        [[nodiscard]] std::size_t headerSize() const noexcept
        {
            return header_.size();
        }

        /**
         * @brief Get the payload size in bytes.
         * @return payload_.size().
         */
        [[nodiscard]] std::size_t payloadSize() const noexcept
        {
            return payload_.size();
        }

        /**
         * @brief Get the total packet size (header + payload).
         * @return Sum of header and payload sizes.
         */
        [[nodiscard]] std::size_t packageSize() const noexcept
        {
            return headerSize() + payloadSize();
        }

        /**
         * @brief Split a raw buffer into header and payload at headerLen.
         *
         * @param buffer Raw byte stream.
         * @param headerLen Number of bytes to treat as header.
         */
        void set(const std::vector<uint8_t> &buffer, std::size_t headerLen)
        {
            if (headerLen > buffer.size())
            {
                RCLCPP_WARN(rclcpp::get_logger("bus"),
                            "Header length %zu exceeds buffer size %zu",
                            headerLen, buffer.size());
                return;
            }
            header_.assign(buffer.begin(), buffer.begin() + headerLen);
            payload_.assign(buffer.begin() + headerLen, buffer.end());
        }

        /**
         * @brief Serialize header and payload into a single raw buffer.
         * @return Concatenated vector (header first, then payload).
         */
        [[nodiscard]] std::vector<uint8_t> buffer() const
        {
            std::vector<uint8_t> out;
            out.reserve(packageSize());
            out.insert(out.end(), header_.begin(), header_.end());
            out.insert(out.end(), payload_.begin(), payload_.end());
            return out;
        }

        /**
         * @brief Log the full package (header and payload) in hex via RCLCPP_INFO.
         */
        void logPackage() const
        {
            std::ostringstream ss;
            ss << "Header:";
            for (auto b : header_)
                ss << ' ' << std::hex << std::setw(2) << std::setfill('0') << +b;
            ss << " | Payload:";
            for (auto b : payload_)
                ss << ' ' << std::hex << std::setw(2) << std::setfill('0') << +b;
            RCLCPP_INFO(rclcpp::get_logger("bus"), "%s", ss.str().c_str());
        }

        /**
         * @brief Interpret the header bytes as a single big-endian signed 64-bit integer.
         * @throws std::runtime_error if header size > 8 bytes.
         */
        int64_t headerAsInt() const
        {
            return bytesToInt64(header_.data(), header_.size());
        }

        /**
         * @brief Interpret the entire payload as a single big-endian signed 64-bit integer.
         * @throws std::runtime_error if payload size > 8 bytes.
         */
        int64_t payloadAsInt() const
        {
            return bytesToInt64(payload_.data(), payload_.size());
        }

        /**
         * @brief Split the payload into chunks of k bytes and interpret each as a big-endian int64.
         *
         * @param k Number of bytes per integer chunk.
         * @return Vector of int64_t, one per chunk.
         * @throws std::invalid_argument if k == 0.
         * @throws std::runtime_error if payload size is not a multiple of k or k > 8.
         */
        std::vector<int64_t> payloadAsInts(std::size_t k) const
        {
            if (k == 0)
            {
                throw std::invalid_argument("Chunk size must be > 0");
            }
            if (k > 8)
            {
                throw std::runtime_error("Chunk size cannot exceed 8 bytes");
            }
            if (payload_.size() % k != 0)
            {
                throw std::runtime_error("Payload size is not a multiple of chunk size");
            }

            std::vector<int64_t> result;
            result.reserve(payload_.size() / k);
            for (std::size_t offset = 0; offset < payload_.size(); offset += k)
            {
                result.push_back(bytesToInt64(&payload_[offset], k));
            }
            return result;
        }

    private:
        std::vector<uint8_t> header_{DefaultHeaderSize, 0};
        std::vector<uint8_t> payload_;

        /**
         * @brief Convert up to 8 bytes in big-endian order to a signed int64.
         * @param data Pointer to the first byte.
         * @param len  Number of bytes (0 <= len <= 8).
         * @throws std::runtime_error if len > 8.
         */
        static int64_t bytesToInt64(const uint8_t *data, std::size_t len)
        {
            if (len > sizeof(int64_t))
            {
                throw std::runtime_error("Cannot convert more than 8 bytes to int64");
            }
            int64_t acc = 0;
            for (std::size_t i = 0; i < len; ++i)
            {
                acc = (acc << 8) | data[i];
            }
            // If you need to treat the highest bit as a sign bit:
            if (len > 0 && (data[0] & 0x80))
            {
                // sign-extend
                acc |= -(int64_t(1) << (len * 8));
            }
            return acc;
        }
    };

    /**
     * @brief Singleton managing serial I/O with COBS‑framed CanPackage.
     */
    class SerialInterface
    {
    public:
        SerialInterface(const SerialInterface &) = delete;
        SerialInterface &operator=(const SerialInterface &) = delete;

        /**
         * @brief Get the singleton instance, opening port on first call.
         *
         * @param port Serial device path (default "/dev/ttyS0").
         * @param baud Baud rate constant (e.g., B115200).
         * @return Reference to the single instance.
         *
         * @note Exceptions (std::system_error) thrown by this constructor should be
         *       caught at the application level to retry or fallback to another port.
         */
        static SerialInterface &getInstance(const std::string &port = "/dev/ttyS0",
                                            int baud = B115200)
        {
            static SerialInterface instance(port, baud);
            return instance;
        }

        /**
         * @brief Send a CanPackage over serial with COBS framing.
         *
         * @param pkg Package to send.
         */
        void send(const CanPackage &pkg)
        {
            auto raw = pkg.buffer();
            auto framed = encodeCOBS(raw);
            std::lock_guard<std::mutex> lock(mutex_);
            ::write(fd_, framed.data(), framed.size());
        }

        /**
         * @brief Read available COBS‑framed data, decode, and build packages.
         *
         * This loop exits on timeout specified by ReadTimeout or if read() returns <= 0.
         *
         * @return Vector of decoded CanPackage objects.
         */
        std::vector<CanPackage> readAll()
        {
            std::vector<CanPackage> packets;
            uint8_t buf[256];

            while (true)
            {
                ssize_t n = ::read(fd_, buf, sizeof(buf));
                if (n > 0) // process each incoming byte
                {
                    for (ssize_t i = 0; i < n; ++i)
                    {
                        uint8_t byte = buf[i];
                        if (byte == COBS_BYTE)
                        {
                            // complete packet → decode, store, clear the frame buffer
                            auto raw = decodeCOBS(frame_);
                            packets.emplace_back(raw);
                            frame_.clear();
                        }
                        else
                        {
                            // still in the middle of a packet
                            frame_.push_back(byte);
                        }
                    }
                    // keep looping in case more data is buffered
                }
                else if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) // no more data *right now* → exit and return what we've got
                {
                    break;
                }
                else if (n == 0) // (EOF on some serial setups) – treat like no more data
                {
                    break;
                }
                else // real error (e.g. n<0 but errno!=EAGAIN)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("bus"),
                                "Serial read error: %s", strerror(errno));
                    break;
                }
            }

            return packets;
        }


    private:
        /**
         * @brief Private constructor opens and configures the serial port.
         *
         * Throws std::system_error if opening or configuring the port fails.
         * Application should catch these exceptions to implement retries or fallback.
         */
        SerialInterface(const std::string &port, int baud)
        {
            fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
            if (fd_ < 0)
                throw std::system_error(errno, std::generic_category(), "open serial port");

            struct termios tty{};
            if (::tcgetattr(fd_, &tty) != 0)
                throw std::system_error(errno, std::generic_category(), "tcgetattr");

            tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
            tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
            tty.c_cflag &= ~CSIZE; // Clear all the size bits, then use one of the statements below
            tty.c_cflag |= CS8; // 8 bits per byte (most common)
            tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
            tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

            tty.c_lflag &= ~ICANON;
            tty.c_lflag &= ~ECHO; // Disable echo
            tty.c_lflag &= ~ECHOE; // Disable erasure
            tty.c_lflag &= ~ECHONL; // Disable new-line echo
            tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP

            tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
            tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

            tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
            tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
            
            tty.c_cc[VMIN] = 0;
            tty.c_cc[VTIME] = 0;

            cfsetospeed(&tty, baud);
            cfsetispeed(&tty, baud);

            if (::tcsetattr(fd_, TCSANOW, &tty) != 0)
                throw std::system_error(errno, std::generic_category(), "tcsetattr");
        }

        /**
         * @brief Destructor closes the serial port.
         */
        ~SerialInterface()
        {
            if (fd_ >= 0)
                ::close(fd_);
        }

        /**
         * @brief COBS delimiter byte.
         */
        static constexpr uint8_t COBS_BYTE = 0;

        /**
         * @brief COBS encode a raw buffer, appending COBS_BYTE terminator.
         * @param data Raw bytes to encode.
         * @return Vector of COBS-encoded bytes including final delimiter.
         */        
        static std::vector<uint8_t> encodeCOBS(const std::vector<uint8_t> &data)
        {
            const uint8_t MARKER = 69;                            // the special “over-end” value
            size_t len = data.size();
            // output needs room for +1 leading code and +1 trailing zero
            std::vector<uint8_t> out(len + 2);

            // lastZero points to the code‐byte we need to fill next
            size_t lastZero = 0;
            out[0] = MARKER;                                      // initial code placeholder

            // copy/input‐scan loop
            for (size_t i = 0; i < len; ++i)
            {
                if (data[i] != 0x00)
                {
                    out[i + 1] = data[i];
                }
                else
                {
                    out[i + 1] = MARKER;                          // stuff a marker in place of the zero
                    // write the distance into the previous code slot
                    out[lastZero] = static_cast<uint8_t>((i + 1) - lastZero);
                    lastZero = i + 1;                             // next code slot is here
                }
            }

            out[len + 1] = 0x00;                                  // final zero terminator
            return out;
        }


        /**
         * @brief Decode a COBS-encoded buffer back to raw data.
         * @param data COBS-encoded bytes excluding the final delimiter.
         * @return Decoded raw bytes.
         */
        static std::vector<uint8_t> decodeCOBS(const std::vector<uint8_t>& data)
        {
            if (data.empty()) return {};

            std::vector<uint8_t> out;
            out.reserve(data.size());

            // The first byte tells us where the first zero goes
            size_t nextZero = data[0];

            // Start reading at index 1, stop at the trailing 0x00
            for (size_t i = 1; i < data.size() && data[i] != 0x00; ++i)
            {
                if (i == nextZero)
                {
                    // time to inject a zero
                    out.push_back(0);
                    // advance to the next code‐position
                    nextZero += data[i];
                }
                else
                {
                    // just a literal byte
                    out.push_back(data[i]);
                }
            }

            return out;
        }




        int fd_;
        std::mutex mutex_;
        std::vector<uint8_t> frame_;  
    };

} // namespace buss