#pragma once
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>

#include <string>
#include <stdexcept>
#include <iostream>

class BtSerial {
public:
  BtSerial(const std::string& dev = "/dev/rfcomm0", speed_t baud = B115200)
  : fd_(-1)
  {
    fd_ = open(dev.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd_ < 0) {
      throw std::runtime_error("open failed: " + std::string(strerror(errno)));
    }

    struct termios tty{};
    if (tcgetattr(fd_, &tty) != 0) {
      close(fd_);
      throw std::runtime_error("tcgetattr failed: " + std::string(strerror(errno)));
    }

    cfsetospeed(&tty, baud);
    cfsetispeed(&tty, baud);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD | CSTOPB | CRTSCTS);

    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP |
                     INLCR | IGNCR | ICRNL |
                     IXON | IXOFF | IXANY);

    tty.c_oflag &= ~OPOST;
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);

    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
      close(fd_);
      throw std::runtime_error("tcsetattr failed: " + std::string(strerror(errno)));
    }
  }

  ~BtSerial() {
    if (fd_ >= 0) close(fd_);
  }

  bool writeString(const std::string& msg) {
    const char* data = msg.c_str();
    size_t total = 0;
    size_t len = msg.size();

    while (total < len) {
      ssize_t n = write(fd_, data + total, len - total);
      if (n < 0) {
        if (errno == EINTR) continue;
        std::cerr << "write failed: " << strerror(errno) << std::endl;
        return false;
      }
      total += static_cast<size_t>(n);
    }

    tcdrain(fd_);
    return true;
  }

  bool readLine(std::string& line)
  {
    line.clear();
    char ch = 0;

    while (true) {
      ssize_t n = ::read(fd_, &ch, 1);
      if (n < 0) {
        if (errno == EINTR) continue;
        std::cerr << "read failed: " << strerror(errno) << std::endl;
        return false;
      }
      if (n == 0) {
        return false;
      }

      if (ch == '\r') continue;

      if (ch == '\n') {
        if (!line.empty()) return true;
        continue;
      }

      line.push_back(ch);

      if (line.size() > 512) {
        std::cerr << "readLine: line too long, discarding" << std::endl;
        line.clear();
        return false;
      }
    }
  }

  bool isOpen() {
    return fd_ >= 0;
  }

private:
  int fd_;
};
