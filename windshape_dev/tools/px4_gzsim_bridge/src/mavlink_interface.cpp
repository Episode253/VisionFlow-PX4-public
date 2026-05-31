#include "mavlink_interface.h"

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <algorithm>
#include <netinet/tcp.h>
#include <poll.h>
#include <cstring>
#include <cmath>

#define MAX_CONSECUTIVE_SPARE_MSG 10

MavlinkInterface::MavlinkInterface() {
  memset(fds_, 0, sizeof(fds_));
  fds_[LISTEN_FD].fd = -1;
  fds_[CONNECTION_FD].fd = -1;
  std::fill_n(input_is_motor_, n_out_max, false);
  input_reference_.resize(n_out_max);
  input_reference_.setZero();

  // Safe HITL defaults. PX4 needs valid IMU + mag + baro from the first
  // HIL_SENSOR frames, otherwise QGC reports: no baro, no compass, no heading,
  // no position estimate. These values are overwritten by Gazebo callbacks as
  // soon as sensor topics start publishing.
  accel_b_ = Eigen::Vector3d(0.0, 0.0, -9.80665);
  gyro_b_ = Eigen::Vector3d::Zero();
  mag_b_ = Eigen::Vector3d(0.2, 0.0, 0.4);       // Gauss, same as the known-good Python test
  temperature_ = 25.0;
  abs_pressure_ = 1013.25;                       // hPa
  pressure_alt_ = 0.0;                           // m
  diff_pressure_ = 0.0;
  imu_updated_ = true;
  mag_updated_ = true;
  baro_updated_ = true;
}

MavlinkInterface::~MavlinkInterface() {
  close();
}


speed_t MavlinkInterface::BaudToTermiosSpeed(unsigned int baudrate) const
{
  switch (baudrate) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 500000: return B500000;
    case 576000: return B576000;
    case 921600: return B921600;
#ifdef B1000000
    case 1000000: return B1000000;
#endif
#ifdef B1500000
    case 1500000: return B1500000;
#endif
#ifdef B2000000
    case 2000000: return B2000000;
#endif
#ifdef B3000000
    case 3000000: return B3000000;
#endif
    default:
      std::cerr << "Unsupported baudrate " << baudrate << ", falling back to 921600" << std::endl;
      return B921600;
  }
}

void MavlinkInterface::OpenSerialPort()
{
  simulator_socket_fd_ = ::open(device_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (simulator_socket_fd_ < 0) {
    std::cerr << "Opening serial device " << device_ << " failed: " << strerror(errno) << ", aborting" << std::endl;
    abort();
  }

  termios tio{};
  if (tcgetattr(simulator_socket_fd_, &tio) != 0) {
    std::cerr << "tcgetattr failed: " << strerror(errno) << ", aborting" << std::endl;
    abort();
  }

  cfmakeraw(&tio);
  const speed_t speed = BaudToTermiosSpeed(baudrate_);
  cfsetispeed(&tio, speed);
  cfsetospeed(&tio, speed);

  tio.c_cflag |= (CLOCAL | CREAD);
  tio.c_cflag &= ~CSTOPB;
  tio.c_cflag &= ~CRTSCTS;
  tio.c_cflag &= ~PARENB;
  tio.c_cflag &= ~CSIZE;
  tio.c_cflag |= CS8;
  tio.c_cc[VMIN] = 0;
  tio.c_cc[VTIME] = 1;

  if (tcsetattr(simulator_socket_fd_, TCSANOW, &tio) != 0) {
    std::cerr << "tcsetattr failed: " << strerror(errno) << ", aborting" << std::endl;
    abort();
  }

  tcflush(simulator_socket_fd_, TCIOFLUSH);
  memset(fds_, 0, sizeof(fds_));
  fds_[CONNECTION_FD].fd = simulator_socket_fd_;
  fds_[CONNECTION_FD].events = POLLIN | POLLOUT;

  std::cout << "Opening serial MAVLink device " << device_ << " @ " << baudrate_ << std::endl;
}


void MavlinkInterface::OpenQgcUdpForwarding()
{
  if (!qgc_udp_forward_enabled_) {
    return;
  }

  memset(&qgc_remote_addr_, 0, sizeof(qgc_remote_addr_));
  qgc_remote_addr_.sin_family = AF_INET;
  qgc_remote_addr_.sin_port = htons(qgc_udp_remote_port_);
  qgc_remote_addr_.sin_addr.s_addr = inet_addr(qgc_udp_addr_str_.c_str());
  if (qgc_remote_addr_.sin_addr.s_addr == INADDR_NONE) {
    std::cerr << "Invalid qgc_udp_addr: " << qgc_udp_addr_str_
              << ", disabling QGC UDP forwarding" << std::endl;
    qgc_udp_forward_enabled_ = false;
    return;
  }

  memset(&qgc_local_addr_, 0, sizeof(qgc_local_addr_));
  qgc_local_addr_.sin_family = AF_INET;
  qgc_local_addr_.sin_addr.s_addr = htonl(INADDR_ANY);
  qgc_local_addr_.sin_port = htons(qgc_udp_local_port_);

  qgc_udp_socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
  if (qgc_udp_socket_fd_ < 0) {
    std::cerr << "Creating QGC UDP forwarding socket failed: "
              << strerror(errno) << ", disabling QGC UDP forwarding" << std::endl;
    qgc_udp_forward_enabled_ = false;
    return;
  }

  int reuse = 1;
  setsockopt(qgc_udp_socket_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
#ifdef SO_REUSEPORT
  setsockopt(qgc_udp_socket_fd_, SOL_SOCKET, SO_REUSEPORT, &reuse, sizeof(reuse));
#endif

  if (bind(qgc_udp_socket_fd_, reinterpret_cast<struct sockaddr *>(&qgc_local_addr_),
           qgc_local_addr_len_) < 0) {
    std::cerr << "Binding QGC UDP forwarding socket on local port "
              << qgc_udp_local_port_ << " failed: " << strerror(errno)
              << ", disabling QGC UDP forwarding" << std::endl;
    ::close(qgc_udp_socket_fd_);
    qgc_udp_socket_fd_ = -1;
    qgc_udp_forward_enabled_ = false;
    return;
  }

  std::cout << "QGC UDP forwarding enabled: serial <-> UDP "
            << qgc_udp_addr_str_ << ":" << qgc_udp_remote_port_
            << " using local UDP port " << qgc_udp_local_port_
            << " [v8 complete-frame forwarding + serial GCS heartbeat]" << std::endl;
}

void MavlinkInterface::ForwardMavlinkMessageToQgc(const mavlink_message_t *message)
{
  if (!qgc_udp_forward_enabled_ || qgc_udp_socket_fd_ < 0 || message == nullptr) {
    return;
  }

  // QGC needs complete MAVLink frames in UDP datagrams. Do not forward raw
  // serial read chunks: a serial read can contain half a MAVLink frame or many
  // frames. Re-serialize each parsed MAVLink message into one UDP datagram.
  uint8_t packet[MAVLINK_MAX_PACKET_LEN];
  const uint16_t len = mavlink_msg_to_send_buffer(packet, message);
  if (len == 0) {
    return;
  }

  const ssize_t ret = sendto(qgc_udp_socket_fd_, packet, len, 0,
                             reinterpret_cast<struct sockaddr *>(&qgc_remote_addr_),
                             qgc_remote_addr_len_);

  static uint64_t qgc_forward_count = 0;
  if (ret < 0) {
    static int error_count = 0;
    if (error_count++ < 20) {
      std::cerr << "QGC UDP forwarding parsed MAVLink message failed: "
                << strerror(errno) << std::endl;
    }
  } else {
    ++qgc_forward_count;
    if (qgc_forward_count == 1 || qgc_forward_count % 100 == 0) {
      std::cout << "[QGC_FORWARD] sent " << qgc_forward_count
                << " complete MAVLink frames to "
                << qgc_udp_addr_str_ << ":" << qgc_udp_remote_port_
                << std::endl;
    }
  }
}

void MavlinkInterface::ProcessQgcUdpMessage()
{
  if (!qgc_udp_forward_enabled_ || qgc_udp_socket_fd_ < 0 || !use_serial_) {
    return;
  }

  uint8_t qgc_buf[4096];
  struct sockaddr_in sender_addr {};
  socklen_t sender_len = sizeof(sender_addr);
  const ssize_t ret = recvfrom(qgc_udp_socket_fd_, qgc_buf, sizeof(qgc_buf), 0,
                               reinterpret_cast<struct sockaddr *>(&sender_addr), &sender_len);
  if (ret <= 0) {
    return;
  }

  // QGC commands / parameter requests are raw MAVLink frames. Forward them
  // directly to the real flight controller serial link. We intentionally do
  // not parse or filter here; PX4 remains the authority on what to accept.
  if (!WriteAll(qgc_buf, static_cast<size_t>(ret))) {
    static int error_count = 0;
    if (error_count++ < 10) {
      std::cerr << "Forwarding QGC UDP packet to serial failed: "
                << strerror(errno) << std::endl;
    }
  }
}


bool MavlinkInterface::WriteAll(const uint8_t *buffer, size_t len)
{
  const std::lock_guard<std::mutex> lock(serial_write_mutex_);
  const auto start = std::chrono::steady_clock::now();
  size_t written = 0;

  while (written < len) {
    const ssize_t ret = ::write(fds_[CONNECTION_FD].fd, buffer + written, len - written);
    if (ret > 0) {
      written += static_cast<size_t>(ret);
      continue;
    }

    if (ret < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) {
      // Do not let one congested serial write block the sender thread forever.
      // Dropping one stale HIL sample is better than letting QGC lose heartbeat.
      if (std::chrono::steady_clock::now() - start > std::chrono::milliseconds(20)) {
        return false;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }

    return false;
  }

  return true;
}

void MavlinkInterface::Load()
{
  mavlink_addr_ = htonl(INADDR_ANY);
  if (mavlink_addr_str_ != "INADDR_ANY") {
    mavlink_addr_ = inet_addr(mavlink_addr_str_.c_str());
    if (mavlink_addr_ == INADDR_NONE) {
      std::cerr << "Invalid mavlink_addr: " << mavlink_addr_str_ << ", aborting" << std::endl;
      abort();
    }
  }
  if (ft_enabled_ && secondary_mavlink_addr_str_ != "INADDR_ANY") {
    secondary_mavlink_addr_ = inet_addr(secondary_mavlink_addr_str_.c_str());
    if (secondary_mavlink_addr_ == INADDR_NONE) {
      std::cerr << "Invalid secondary_mavlink_addr: " << secondary_mavlink_addr_ << ", aborting" << std::endl;
      abort();
    }
  }

  // initialize sender status to zero
  memset((char *)&sender_m_status_, 0, sizeof(sender_m_status_));

  if (use_serial_) {
    OpenSerialPort();
    OpenQgcUdpForwarding();
    receiver_thread_ = std::thread([this] () { ReceiveWorker(); });
    sender_thread_ = std::thread([this] () { SendWorker(); });
    return;
  }

  memset((char *)&remote_simulator_addr_, 0, sizeof(remote_simulator_addr_));
  remote_simulator_addr_.sin_family = AF_INET;
  remote_simulator_addr_len_ = sizeof(remote_simulator_addr_);

  memset((char *)&secondary_remote_simulator_addr_, 0, sizeof(secondary_remote_simulator_addr_));
  secondary_remote_simulator_addr_.sin_family = AF_INET;
  secondary_remote_simulator_addr_len_ = sizeof(secondary_remote_simulator_addr_);

  memset((char *)&local_simulator_addr_, 0, sizeof(local_simulator_addr_));
  local_simulator_addr_.sin_family = AF_INET;
  local_simulator_addr_len_ = sizeof(local_simulator_addr_);

  memset((char *)&secondary_local_simulator_addr_, 0, sizeof(secondary_local_simulator_addr_));
  secondary_local_simulator_addr_.sin_family = AF_INET;
  secondary_local_simulator_addr_len_ = sizeof(secondary_local_simulator_addr_);

  if (use_tcp_) {

      if ((simulator_socket_fd_ = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        std::cerr << "Creating TCP socket failed: " << strerror(errno) << ", aborting" << std::endl;
        abort();
      }

      int yes = 1;
      int result = setsockopt(simulator_socket_fd_, IPPROTO_TCP, TCP_NODELAY, &yes, sizeof(yes));
      if (result != 0) {
        std::cerr << "setsockopt failed: " << strerror(errno) << ", aborting" << std::endl;
        abort();
      }

      struct linger nolinger {};
      nolinger.l_onoff = 1;
      nolinger.l_linger = 0;

      result = setsockopt(simulator_socket_fd_, SOL_SOCKET, SO_LINGER, &nolinger, sizeof(nolinger));
      if (result != 0) {
        std::cerr << "setsockopt failed: " << strerror(errno) << ", aborting" << std::endl;
        abort();
      }

      // The socket reuse is necessary for reconnecting to the same address
      // if the socket does not close but gets stuck in TIME_WAIT. This can happen
      // if the server is suddenly closed, for example, if the robot is deleted in gazebo.
      int socket_reuse = 1;
      result = setsockopt(simulator_socket_fd_, SOL_SOCKET, SO_REUSEADDR, &socket_reuse, sizeof(socket_reuse));
      if (result != 0) {
        std::cerr << "setsockopt failed: " << strerror(errno) << ", aborting" << std::endl;
        abort();
      }

      // Same as above but for a given port
      result = setsockopt(simulator_socket_fd_, SOL_SOCKET, SO_REUSEPORT, &socket_reuse, sizeof(socket_reuse));
      if (result != 0) {
        std::cerr << "setsockopt failed: " << strerror(errno) << ", aborting" << std::endl;
        abort();
      }

    if (tcp_client_mode_) {
      // TCP client mode
      local_simulator_addr_.sin_addr.s_addr = htonl(INADDR_ANY);
      local_simulator_addr_.sin_port = htons(0);
      remote_simulator_addr_.sin_addr.s_addr = mavlink_addr_;
      remote_simulator_addr_.sin_port = htons(mavlink_tcp_port_);
      memset(fds_, 0, sizeof(fds_));
    } else {
      // TCP server mode
      local_simulator_addr_.sin_addr.s_addr = mavlink_addr_;
      local_simulator_addr_.sin_port = htons(mavlink_tcp_port_);

      if (bind(simulator_socket_fd_, (struct sockaddr *)&local_simulator_addr_, local_simulator_addr_len_) < 0) {
        std::cerr << "bind failed: " << strerror(errno) << ", aborting" << std::endl;
        abort();
      }

      errno = 0;
      if (listen(simulator_socket_fd_, 0) < 0) {
        std::cerr << "listen failed: " << strerror(errno) << ", aborting" << std::endl;
        abort();
      }

      memset(fds_, 0, sizeof(fds_));
      fds_[LISTEN_FD].fd = simulator_socket_fd_;
      fds_[LISTEN_FD].events = POLLIN; // only listens for new connections on tcp
    }
  } else {
    // When connecting to HITL, we specify the port where the mavlink traffic originates from.
    remote_simulator_addr_.sin_addr.s_addr = mavlink_addr_;
    remote_simulator_addr_.sin_port = htons(mavlink_udp_remote_port_);
    secondary_remote_simulator_addr_.sin_addr.s_addr = secondary_mavlink_addr_;
    secondary_remote_simulator_addr_.sin_port = htons(mavlink_udp_remote_port_);
    // Local
    local_simulator_addr_.sin_addr.s_addr = htonl(INADDR_ANY);
    local_simulator_addr_.sin_port = htons(mavlink_udp_local_port_);
    secondary_local_simulator_addr_.sin_addr.s_addr = htonl(INADDR_ANY);
    secondary_local_simulator_addr_.sin_port = htons(secondary_mavlink_udp_local_port_);

    std::cout << "Creating UDP socket for HITL input on local port : " << mavlink_udp_local_port_ << " and remote port " << mavlink_udp_remote_port_ << std::endl;

    if ((simulator_socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
      std::cerr << "Creating UDP socket failed: " << strerror(errno) << ", aborting" << std::endl;
      abort();
    }

    if (bind(simulator_socket_fd_, (struct sockaddr *)&local_simulator_addr_, local_simulator_addr_len_) < 0) {
      std::cerr << "bind failed: " << strerror(errno) << ", aborting" << std::endl;
      abort();
    }

    if (ft_enabled_) {
      std::cout << "Creating secondary UDP socket for HITL input on local port : " << secondary_mavlink_udp_local_port_ << " and remote port " << mavlink_udp_remote_port_ << std::endl;

      if ((simulator_second_socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        std::cerr << "Creating secondary UDP socket failed: " << strerror(errno) << ", aborting" << std::endl;
        abort();
      }

      if (bind(simulator_second_socket_fd_, (struct sockaddr *)&secondary_local_simulator_addr_, secondary_local_simulator_addr_len_) < 0) {
        std::cerr << "secondary bind failed: " << strerror(errno) << ", aborting" << std::endl;
        abort();
      }
    }

    memset(fds_, 0, sizeof(fds_));
    fds_[CONNECTION_FD].fd = simulator_socket_fd_;
    fds_[CONNECTION_FD].events = POLLIN | POLLOUT; // read/write
  }

  // Start mavlink message receiver thread
  receiver_thread_ = std::thread([this] () {
    ReceiveWorker();
  });
  // Start mavlink message sender thread
  sender_thread_ = std::thread([this] () {
    SendWorker();
  });
}


/*******************************************************
 * Receive buffer handling
 */

std::shared_ptr<mavlink_message_t> MavlinkInterface::PopRecvMessage() {
  std::shared_ptr<mavlink_message_t> msg(nullptr);
  const std::lock_guard<std::mutex> guard(receiver_buff_mtx_);
  if (!receiver_buffer_.empty()) {
    msg = receiver_buffer_.front();
    receiver_buffer_.pop();
  }
  return msg;
}

void MavlinkInterface::ProcessReceivedMessage(int ret, char *thrd_name) {

  if (ret < 0) {
    std::cerr << "[" << thrd_name << "] recv error: " << strerror(errno) << std::endl;
    if (errno == ECONNRESET) {
      close_conn_ = true;
    }
    return;
  } else if (ret == 0) {
    std::cerr << "[" << thrd_name << "] no data received after select trigger.." << std::endl;
    // No data..
    return;
  }

  // data received
  int len = ret;

  // Do not mirror arbitrary raw serial chunks to QGC. Serial reads may split a
  // MAVLink frame across two buffers, which creates malformed UDP datagrams for
  // QGC and can leave the UI stuck at "Disconnected". We parse first, then
  // forward complete MAVLink frames below.
  mavlink_status_t status{};
  mavlink_message_t message{};

  for(int i = 0; i < len; i++)
  {
    auto msg_received = static_cast<Framing>(mavlink_frame_char_buffer(&m_buffer_, &m_status_, buf_[i], &message, &status));
    if (msg_received == Framing::bad_crc || msg_received == Framing::bad_signature) {
      _mav_parse_error(&m_status_);
      m_status_.msg_received = MAVLINK_FRAMING_INCOMPLETE;
      m_status_.parse_state = MAVLINK_PARSE_STATE_IDLE;
      if (buf_[i] == MAVLINK_STX) {
        m_status_.parse_state = MAVLINK_PARSE_STATE_GOT_STX;
        m_buffer_.len = 0;
        mavlink_start_checksum(&m_buffer_);
      }
    }

    if (msg_received != Framing::incomplete) {
      // Forward every valid PX4 MAVLink message to QGC as a complete MAVLink
      // UDP frame. This keeps QGC parameter sync, console and vehicle state
      // alive while the bridge still filters what it consumes internally.
      if (use_serial_) {
        ForwardMavlinkMessageToQgc(&message);
      }

      // The bridge itself only consumes HEARTBEAT and HIL_ACTUATOR_CONTROLS.
      // PX4 also streams many telemetry messages at high rate; queuing all of
      // them overflows the receiver FIFO before PreUpdate can drain it.
      if (message.msgid != MAVLINK_MSG_ID_HEARTBEAT &&
          message.msgid != MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS) {
        continue;
      }

      auto msg = std::make_shared<mavlink_message_t>(message);
      const std::lock_guard<std::mutex> guard(receiver_buff_mtx_);
      if (receiver_buffer_.size() >= kMaxRecvBufferSize) {
        // Keep the newest messages. Dropping one stale message is safer than
        // clearing the whole FIFO and losing fresh actuator controls.
        receiver_buffer_.pop();
      }
      receiver_buffer_.push(msg);
    }
  }
}

void MavlinkInterface::ReceiveWorker() {
  struct sockaddr_in remote_addr;
  socklen_t remote_addr_len = sizeof(remote_addr);

  char thrd_name[64] = {0};
  sprintf(thrd_name, "MAV_Recver_%d", gettid());
  pthread_setname_np(pthread_self(), thrd_name);

  std::cout << "[" << thrd_name << "] starts" << std::endl;

  /*
  // Wait for connection:
  if ((fds_[CONNECTION_FD].fd <= 0) && use_tcp_) {
    // Client mode
    std::cout << "[" << thrd_name << "] Wait for TCP connection.." << std::endl;
    while (fds_[CONNECTION_FD].fd <= 0) {
      if (!tcp_client_mode_) acceptConnections();
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    std::cout << "[" << thrd_name << "] TCP connection detected" << std::endl;
  }
  */
  std::cout << "[" << thrd_name << "] Start receiving..." << std::endl;

  fd_set readfds;
  int maxfd;

  while(!close_conn_ && !gotSigInt_) {
    FD_ZERO(&readfds);
    FD_SET(simulator_socket_fd_, &readfds);
    maxfd = simulator_socket_fd_ + 1;

    if (ft_enabled_) {
      FD_SET(simulator_second_socket_fd_, &readfds);
      maxfd = std::max(maxfd, simulator_second_socket_fd_ + 1);
    }

    if (qgc_udp_forward_enabled_ && qgc_udp_socket_fd_ >= 0) {
      FD_SET(qgc_udp_socket_fd_, &readfds);
      maxfd = std::max(maxfd, qgc_udp_socket_fd_ + 1);
    }

    struct timeval tv = {1, 0}; // 1 second timeout
    int ret = select(maxfd, &readfds, nullptr, nullptr, &tv);
    if (ret < 0) {
      std::cerr << "[" << thrd_name << "] recv error: " << strerror(errno) << std::endl;
      if (errno == ECONNRESET) {
        close_conn_ = true;
      }
      continue;
    } else if (ret == 0) {
      // Timeout
      continue;
    }

    if (FD_ISSET(simulator_socket_fd_, &readfds)) {
      int ret = 0;
      if (use_serial_) {
        ret = static_cast<int>(::read(simulator_socket_fd_, buf_, sizeof(buf_)));
      } else {
        // Receive data from sock1
        ret = recvfrom(simulator_socket_fd_, buf_, sizeof(buf_), 0, (struct sockaddr *)&remote_addr, &remote_addr_len);
      }
      ProcessReceivedMessage(ret, thrd_name);
    }

    if (qgc_udp_forward_enabled_ && qgc_udp_socket_fd_ >= 0 &&
        FD_ISSET(qgc_udp_socket_fd_, &readfds)) {
      ProcessQgcUdpMessage();
    }

    if (ft_enabled_ && FD_ISSET(simulator_second_socket_fd_, &readfds)) {
      // Receive data from sock2
      int ret = recvfrom(simulator_second_socket_fd_, buf_, sizeof(buf_), 0, (struct sockaddr *)&remote_addr, &remote_addr_len);
      // ... process data
      ProcessReceivedMessage(ret, thrd_name);
    }
  }
  std::cout << "The thread [" << thrd_name << "] was shutdown." << std::endl;
}

/*******************************************************
 * Send buffer handling
 */

void MavlinkInterface::PushSendMessage(std::shared_ptr<mavlink_message_t> msg) {
  if (!msg) {
    return;
  }

  const uint32_t msgid = msg->msgid;
  const bool high_rate_hil_msg =
      msgid == MAVLINK_MSG_ID_HIL_SENSOR ||
      msgid == MAVLINK_MSG_ID_HIL_GPS ||
      msgid == MAVLINK_MSG_ID_HIL_STATE_QUATERNION ||
      msgid == MAVLINK_MSG_ID_ESC_STATUS ||
      msgid == MAVLINK_MSG_ID_ESC_INFO;

  {
    const std::lock_guard<std::mutex> guard(sender_buff_mtx_);

    // HIL_SENSOR / HIL_GPS / HIL_STATE are state samples. Old samples are
    // worthless once a newer sample of the same type exists. Coalescing avoids
    // queue growth during QGC parameter downloads or brief serial backpressure.
    if (high_rate_hil_msg && !sender_buffer_.empty()) {
      for (auto it = sender_buffer_.begin(); it != sender_buffer_.end(); ) {
        if ((*it) && (*it)->msgid == msgid) {
          it = sender_buffer_.erase(it);
        } else {
          ++it;
        }
      }
    }

    sender_buffer_.push_back(msg);

    while (sender_buffer_.size() > kMaxSendBufferSize) {
      bool dropped = false;

      // Prefer dropping stale high-rate state messages. Keep lower-rate command
      // or control messages when possible.
      for (auto it = sender_buffer_.begin(); it != sender_buffer_.end(); ++it) {
        if ((*it) && ((*it)->msgid == MAVLINK_MSG_ID_HIL_SENSOR ||
                     (*it)->msgid == MAVLINK_MSG_ID_HIL_GPS ||
                     (*it)->msgid == MAVLINK_MSG_ID_HIL_STATE_QUATERNION ||
                     (*it)->msgid == MAVLINK_MSG_ID_ESC_STATUS ||
                     (*it)->msgid == MAVLINK_MSG_ID_ESC_INFO)) {
          sender_buffer_.erase(it);
          dropped = true;
          break;
        }
      }

      if (!dropped) {
        sender_buffer_.pop_front();
      }

      static auto last_warn = std::chrono::steady_clock::now() - std::chrono::seconds(10);
      const auto now = std::chrono::steady_clock::now();
      if (received_first_actuator_ && now - last_warn > std::chrono::seconds(5)) {
        std::cerr << "PushSendMessage - sender queue under pressure, dropping stale HIL samples" << std::endl;
        last_warn = now;
      }
    }
  }

  sender_cv_.notify_one();
}

void MavlinkInterface::PushSendMessage(mavlink_message_t *msg) {
    auto msg_shared = std::make_shared<mavlink_message_t>(*msg);
    PushSendMessage(msg_shared);
}

void MavlinkInterface::SendGcsHeartbeatToSerial()
{
  if (!use_serial_ || fds_[CONNECTION_FD].fd <= 0) {
    return;
  }

  // Act as a minimal GCS heartbeat on the direct serial link. Some PX4 MAVLink
  // instances keep telemetry quiet until they see a GCS/companion heartbeat.
  // This also gives the real flight controller a stable peer even before QGC
  // sends its first UDP packet through this bridge.
  mavlink_message_t msg{};
  mavlink_heartbeat_t heartbeat{};
  heartbeat.type = MAV_TYPE_GCS;
  heartbeat.autopilot = MAV_AUTOPILOT_INVALID;
  heartbeat.base_mode = 0;
  heartbeat.custom_mode = 0;
  heartbeat.system_status = MAV_STATE_ACTIVE;
  heartbeat.mavlink_version = 3;

  mavlink_msg_heartbeat_encode_chan(255, 190, MAVLINK_COMM_0, &msg, &heartbeat);

  uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
  const int packet_len = mavlink_msg_to_send_buffer(buffer, &msg);
  if (packet_len > 0) {
    WriteAll(buffer, static_cast<size_t>(packet_len));
  }
}

void MavlinkInterface::SendWorker() {
  char thrd_name[64] = {0};
  sprintf(thrd_name, "MAV_Sender_%d", gettid());
  pthread_setname_np(pthread_self(), thrd_name);

  if ((fds_[CONNECTION_FD].fd <= 0) && tcp_client_mode_) {
    std::cout << "[" << thrd_name << "] Try to connect to PX4 TCP server.. " << std::endl;
    while (!tryConnect()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    std::cout << "[" << thrd_name << "] Client connected to PX4 TCP server" << std::endl;
  }

  auto last_gcs_heartbeat = std::chrono::steady_clock::now() - std::chrono::seconds(2);

  while(!close_conn_ && !gotSigInt_) {
    // Keep the PX4 MAVLink serial instance alive even when QGC has not yet sent
    // anything to the bridge. A plain condition_variable::wait() sleeps forever
    // when the send queue is empty, so use wait_for() to allow periodic link
    // heartbeats. This is the boring little detail that makes UDP forwarding
    // actually start instead of waiting politely until the heat death of Linux.
    {
      const auto now = std::chrono::steady_clock::now();
      if (use_serial_ && qgc_udp_forward_enabled_ &&
          now - last_gcs_heartbeat >= std::chrono::seconds(1)) {
        SendGcsHeartbeatToSerial();
        last_gcs_heartbeat = now;
      }
    }

    std::unique_lock<std::mutex> lock{sender_buff_mtx_};
    sender_cv_.wait_for(lock, std::chrono::milliseconds(20), [&]()
    {
      return close_conn_ || gotSigInt_ || !sender_buffer_.empty();
    });

    if (close_conn_ || gotSigInt_) {
      break;
    }

    if (sender_buffer_.empty()) {
      continue;
    }

    auto msg = sender_buffer_.front();
    if (msg) {
      sender_buffer_.pop_front();
      lock.unlock();
      send_mavlink_message(msg.get());
    } else {
      lock.unlock();
    }
  }

  std::cout << "The thread [" << thrd_name << "] was shutdown." << std::endl;
}

void MavlinkInterface::SendSensorMessages(uint64_t time_usec) {
  mavlink_hil_sensor_t sensor_msg{};
  sensor_msg.time_usec = time_usec;

  /* Workaround for MAVLink v2 zero-suppression bug.
     PX4 resets the sensor id on the receiver side. */
  sensor_msg.id = 1;

  // Always publish a complete HIL_SENSOR frame containing IMU + magnetometer +
  // barometer. Sending only "updated" sub-fields is fragile in HITL startup: if
  // Gazebo's mag/baro callbacks lag or the topic name is wrong, PX4 never
  // creates valid simulated compass/baro topics and QGC blocks arming. Keeping
  // the last known values in every frame matches the Python sanity test that
  // successfully produced sensor_accel/sensor_gyro/sensor_mag/sensor_baro.
  Eigen::Vector3d accel_b = accel_b_;
  Eigen::Vector3d gyro_b = gyro_b_;
  Eigen::Vector3d mag_b = mag_b_;
  double temperature = temperature_;
  double abs_pressure = abs_pressure_;
  double pressure_alt = pressure_alt_;
  double diff_pressure = diff_pressure_;
  bool include_diff_pressure = false;

  {
    const std::lock_guard<std::mutex> lock(sensor_msg_mutex_);

    mag_b = mag_b_;
    temperature = temperature_;
    abs_pressure = abs_pressure_;
    pressure_alt = pressure_alt_;
    diff_pressure = diff_pressure_;
    include_diff_pressure = diff_press_updated_;

    // Consume update markers, but keep the values for the next frame.
    imu_updated_ = false;
    mag_updated_ = false;
    baro_updated_ = false;
    diff_press_updated_ = false;
  }

  // Defensive fallbacks. A zero magnetic field or zero pressure is invalid for
  // PX4 preflight checks. Do not let an empty Gazebo callback poison the EKF.
  if (!std::isfinite(mag_b.x()) || !std::isfinite(mag_b.y()) || !std::isfinite(mag_b.z()) ||
      mag_b.norm() < 1e-6) {
    mag_b = Eigen::Vector3d(0.2, 0.0, 0.4);
  }

  if (!std::isfinite(abs_pressure) || abs_pressure < 300.0 || abs_pressure > 1200.0) {
    abs_pressure = 1013.25;
  }
  if (!std::isfinite(temperature)) {
    temperature = 25.0;
  }
  if (!std::isfinite(pressure_alt)) {
    pressure_alt = 0.0;
  }

  sensor_msg.xacc = accel_b.x();
  sensor_msg.yacc = accel_b.y();
  sensor_msg.zacc = accel_b.z();
  sensor_msg.xgyro = gyro_b.x();
  sensor_msg.ygyro = gyro_b.y();
  sensor_msg.zgyro = gyro_b.z();
  sensor_msg.xmag = mag_b.x();
  sensor_msg.ymag = mag_b.y();
  sensor_msg.zmag = mag_b.z();
  sensor_msg.abs_pressure = abs_pressure;
  sensor_msg.diff_pressure = include_diff_pressure ? diff_pressure : 0.0;
  sensor_msg.pressure_alt = pressure_alt;
  sensor_msg.temperature = temperature;

  sensor_msg.fields_updated = static_cast<uint16_t>(SensorSource::ACCEL) |
                              static_cast<uint16_t>(SensorSource::GYRO) |
                              static_cast<uint16_t>(SensorSource::MAG) |
                              static_cast<uint16_t>(SensorSource::BARO);

  if (include_diff_pressure) {
    sensor_msg.fields_updated |= static_cast<uint16_t>(SensorSource::DIFF_PRESS);
  }

  mavlink_message_t msg{};
  mavlink_msg_hil_sensor_encode_chan(254, 25, MAVLINK_COMM_0, &msg, &sensor_msg);
  FinalizeOutgoingMessage(&msg, 254, 25,
    MAVLINK_MSG_ID_HIL_SENSOR_MIN_LEN,
    MAVLINK_MSG_ID_HIL_SENSOR_LEN,
    MAVLINK_MSG_ID_HIL_SENSOR_CRC);
  PushSendMessage(&msg);
}

void MavlinkInterface::SendEscStatusMessages(uint64_t time_usec, struct StatusData::EscStatus &status) {
  mavlink_esc_status_t esc_status_msg{};
  mavlink_esc_info_t esc_info_msg{};
  static constexpr uint8_t batch_size = MAVLINK_MSG_ESC_STATUS_FIELD_RPM_LEN;
  static uint16_t counter = 0;

  esc_status_msg.time_usec = time_usec;
  for (int i = 0; i < batch_size; i++) {
    esc_status_msg.rpm[i] = status.esc[i].rpm;

    esc_info_msg.failure_flags[i] = 0;
    esc_info_msg.error_count[i] = 0;
    esc_info_msg.temperature[i] = 20;
  }

  mavlink_message_t msg{};
  mavlink_msg_esc_status_encode_chan(254, 25, MAVLINK_COMM_0, &msg, &esc_status_msg);
  auto msg_shared = std::make_shared<mavlink_message_t>(msg);
  PushSendMessage(msg_shared);

  esc_info_msg.counter = counter++;
  esc_info_msg.count = status.esc_count;
  esc_info_msg.connection_type = 0; // TODO: use two highest bits for selected input
  esc_info_msg.info = (1u << status.esc_count) - 1;

  mavlink_msg_esc_info_encode_chan(254, 25, MAVLINK_COMM_0, &msg, &esc_info_msg);
  msg_shared = std::make_shared<mavlink_message_t>(msg);
  PushSendMessage(msg_shared);
}

void MavlinkInterface::UpdateBarometer(const SensorData::Barometer &data) {
  const std::lock_guard<std::mutex> lock(sensor_msg_mutex_);
  temperature_ = data.temperature;
  abs_pressure_ = data.abs_pressure;
  pressure_alt_ = data.pressure_alt;

  baro_updated_ = true;
}

void MavlinkInterface::UpdateAirspeed(const SensorData::Airspeed &data) {
  const std::lock_guard<std::mutex> lock(sensor_msg_mutex_);
  diff_pressure_ = data.diff_pressure;

  diff_press_updated_ = true;
}

void MavlinkInterface::UpdateIMU(const SensorData::Imu &data) {
  // Imu is updated only before sending, so locking handled there
  //   by last_imu_message_mutex_
  accel_b_ = data.accel_b;
  gyro_b_ = data.gyro_b;

  imu_updated_ = true;
}

void MavlinkInterface::UpdateMag(const SensorData::Magnetometer &data) {
  const std::lock_guard<std::mutex> lock(sensor_msg_mutex_);
  mag_b_ = data.mag_b;

  mag_updated_ = true;
}

void MavlinkInterface::ReadMAVLinkMessages()
{
  if (gotSigInt_) {
    return;
  }

  received_actuator_ = false;

  if ((fds_[CONNECTION_FD].fd <= 0) && tcp_client_mode_) {
    return;
  }

  if (!enable_lockstep_ && IsRecvBuffEmpty()) {
    // Receive buffer is empty, exit
    return;
  }

  do {
    //std::cerr << "[MavlinkInterface] check message from msg buffer.. " << std::endl;
    auto msg = PopRecvMessage();
    if (msg) {
      //std::cout << "[MavlinkInterface] ReadMAVLinkMessages -> handle_message " << std::endl;
      handle_message(msg.get());
    }
  } while( (!enable_lockstep_ && !IsRecvBuffEmpty()) ||
           ( enable_lockstep_ && !received_actuator_ && received_first_actuator_) );

}

void MavlinkInterface::acceptConnections()
{
  if (fds_[CONNECTION_FD].fd > 0) {
    return;
  }

  // accepting incoming connections on listen fd
  int ret =
    accept(fds_[LISTEN_FD].fd, (struct sockaddr *)&remote_simulator_addr_, &remote_simulator_addr_len_);

  if (ret < 0) {
    if (errno != EWOULDBLOCK) {
      std::cerr << "accept error: " << strerror(errno) << std::endl;
    }
    return;
  }

  // assign socket to connection descriptor on success
  fds_[CONNECTION_FD].fd = ret; // socket is replaced with latest connection
  fds_[CONNECTION_FD].events = POLLIN;
}

bool MavlinkInterface::tryConnect()
{
  if (fds_[CONNECTION_FD].fd > 0) {
    return true;
  }

  // Try connecting to px4 simulator TCP server
  int ret = connect(simulator_socket_fd_, (struct sockaddr *)&remote_simulator_addr_, remote_simulator_addr_len_);

  if (ret < 0) {
    return false;
  }

  // assign socket to connection descriptor on success
  fds_[CONNECTION_FD].events = POLLIN | POLLOUT; // read/write
  fds_[CONNECTION_FD].fd = simulator_socket_fd_;

  return true;
}

void MavlinkInterface::handle_message(mavlink_message_t *msg)
{
  switch (msg->msgid) {
  case MAVLINK_MSG_ID_HEARTBEAT:
    handle_heartbeat(msg);
    break;
  case MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS:
    handle_actuator_controls(msg);
    break;
  }
}

void MavlinkInterface::handle_heartbeat(mavlink_message_t *msg)
{
  /*
  char thrd_name[64] = {0};
  sprintf(thrd_name, "thr_%d", gettid());
  pthread_setname_np(pthread_self(), thrd_name);

  mavlink_heartbeat_t hbeat;
  mavlink_msg_heartbeat_decode(msg, &hbeat);
  //bool armed = (hbeat.system_status == MAV_STATE_ACTIVE || hbeat.system_status == MAV_STATE_CRITICAL);
  int status = (int) hbeat.system_status;
  int compid = (int) msg->compid;

  std::cout << "HB from " << compid << " armed:" << (armed ? "ARMED" : "DISARMED") << " status:" << status <<
    " *armed1_:" << static_cast<void*>(&armed1_) << " *armed2_:" << static_cast<void*>(&armed2_) << std::endl;

  if (msg->compid == 1) {
    // Update from FC1
    // armed if state is either ACTIVE or CRITICAL, otherwise disarmed
    if (armed1_ != armed) {
      armed1_ = armed;
      std::cout << thrd_name << " FC1: " << (armed ? "ARMED" : "DISARMED") << "(" << status << ")"
        << " => STATE [" << (armed1_ ? "ARMED" : "DISARMED")
        << ", " << (armed2_ ? "ARMED" : "DISARMED") << "]" << std::endl;
    }

    // If FC1 is arming, make sure we use FC1
    if (armed) {
      if (use_redundant_) {
        std::cout << "Primary FC1 healthy and arming => Use primary FC1!" << std::endl;
        use_redundant_ = false;
      }
    }
  } else if (msg->compid == 2) {
    // Update from FC2
    // armed if state is either ACTIVE or CRITICAL, otherwise disarmed
    if (armed2_ != armed) {
      armed2_ = armed;
      std::cout << thrd_name << " FC2: " << (armed ? "ARMED" : "DISARMED") << "(" << status << ")"
        << " => STATE [" << (armed1_ ? "ARMED" : "DISARMED")
        << ", " << (armed2_ ? "ARMED" : "DISARMED") << "]" << std::endl;
    }
  } else {
    std::cout << "HB from unknown source" << std::endl;
    return;
  }

  received_heartbeats_ = true;

  if (!armed1_ && armed2_) {
    use_redundant_ = true;
    std::cout << "Primary FC1 disarmed while redundant still armed => Use redundant FC2!" << std::endl;
  }

  */
 received_heartbeats_ = true;
}

bool is_running(int fc, mavlink_hil_actuator_controls_t &controls) {
  for (int i=0; i<4; i++) {
    //std::cout << "FC" << fc << ": motor[" << i << "]: " << controls.controls[i] << std::endl;
    if (controls.controls[i] > 0.001) {
      return true;
    }
  }
  return false;
}

void MavlinkInterface::handle_actuator_controls(mavlink_message_t *msg)
{
  static int consecutive_spare_msg = 0;
  const std::lock_guard<std::mutex> lock(actuator_mutex_);
  mavlink_hil_actuator_controls_t controls{};
  mavlink_msg_hil_actuator_controls_decode(msg, &controls);

  if (msg->compid == 2) {
    // Message from redundant FC
    armed2_ = (controls.mode & MAV_MODE_FLAG_SAFETY_ARMED) || is_running(2, controls);

    //std::cout << "secondary FP2: " << (armed2_ ? "ARMED" : "DISARMED") << std::endl;
    if (consecutive_spare_msg < MAX_CONSECUTIVE_SPARE_MSG) {
      if (consecutive_spare_msg > (MAX_CONSECUTIVE_SPARE_MSG/2)) {
        std::cout << "FC1 missing count " << consecutive_spare_msg << std::endl;
      }
      consecutive_spare_msg++;
      if (consecutive_spare_msg == MAX_CONSECUTIVE_SPARE_MSG) {
        use_redundant_ = true;
        std::cout << "Primary FC1 timeout => Use redundant FC2" << std::endl;
      }
    }

    if (!use_redundant_) {
      return;
    }

  } else if (msg->compid == 1) {
    // Message from primary FC
    armed1_ = (controls.mode & MAV_MODE_FLAG_SAFETY_ARMED) || is_running(1, controls);
    //std::cout << "Primary FC1: " << (armed1_ ? "ARMED" : "DISARMED") << std::endl;
    if (ft_enabled_) {
      if (armed1_) {
        consecutive_spare_msg = 0;
        if (use_redundant_) {
          use_redundant_ = false;
          std::cout << "Primary FC1 armed => Use primary FC1" << std::endl;
        }
      } else {
        if (!use_redundant_) {
          std::cout << "Primary FC1 disarmed => Use redundant FC2" << std::endl;
        }
        use_redundant_ = true;
      }

      if (use_redundant_) {
        return;
      }
    }
  }

  // set rotor and servo speeds, controller targets
  input_reference_.resize(n_out_max);
  for (int i = 0; i < input_reference_.size(); i++) {
    input_is_motor_[i] = controls.flags & (1 << i);
    input_reference_[i] = controls.controls[i];
  }
  received_actuator_ = true;
  received_first_actuator_ = true;
}

void MavlinkInterface::send_mavlink_message(const mavlink_message_t *message)
{
  assert(message != nullptr);

  if (gotSigInt_ || close_conn_) {
    return;
  }

  uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
  int packetlen = mavlink_msg_to_send_buffer(buffer, message);

  if (fds_[CONNECTION_FD].fd > 0) {
    ssize_t len;
    if (use_serial_) {
      len = WriteAll(buffer, packetlen) ? static_cast<ssize_t>(packetlen) : -1;
    } else if (use_tcp_) {
      len = send(fds_[CONNECTION_FD].fd, buffer, packetlen, 0);
    } else {

      len = sendto(simulator_socket_fd_, buffer, packetlen, 0, (struct sockaddr *)&remote_simulator_addr_, remote_simulator_addr_len_);

      if (ft_enabled_) {
        ssize_t len2 = sendto(simulator_second_socket_fd_, buffer, packetlen, 0, (struct sockaddr *)&secondary_remote_simulator_addr_, secondary_remote_simulator_addr_len_);
        if (len < 0 && len2 < 0) {
          // neither one worked => error
          len = -1;
        } else {
          // at least one worked => success
          len = 0;
        }
      }
    }

    if (len < 0) {
      if (received_first_actuator_) {
        std::cerr << "Failed sending mavlink message: " << strerror(errno) << std::endl;
        if (errno == ECONNRESET || errno == EPIPE) {
          if (use_tcp_) { // udp socket remains alive
            std::cerr << "Closing connection." << std::endl;
            close_conn_ = true;
          }
        }
      }
    }
  }
}

void MavlinkInterface::close()
{
  close_conn_ = true;
  // Shutdown receiver side
  if (fds_[CONNECTION_FD].fd >= 0 && !use_serial_) {
    shutdown(fds_[CONNECTION_FD].fd, SHUT_RD);
  }
  if (simulator_second_socket_fd_ >= 0 && !use_serial_) {
    shutdown(simulator_second_socket_fd_, SHUT_RD);
  }

  if (receiver_thread_.joinable())
    receiver_thread_.join();

  if (sender_thread_.joinable()) {
    sender_cv_.notify_one();
    sender_thread_.join();
  }

  if (fds_[CONNECTION_FD].fd >= 0) {
    ::close(fds_[CONNECTION_FD].fd);
    fds_[CONNECTION_FD].fd = -1;
    fds_[CONNECTION_FD] = { 0, 0, 0 };
  }

  if (simulator_second_socket_fd_ >= 0) {
    ::close(simulator_second_socket_fd_);
    simulator_second_socket_fd_ = -1;
  }

  if (qgc_udp_socket_fd_ >= 0) {
    ::close(qgc_udp_socket_fd_);
    qgc_udp_socket_fd_ = -1;
  }

  received_first_actuator_ = false;
}



void MavlinkInterface::onSigInt() {
  gotSigInt_ = true;
  close();
}

Eigen::VectorXd MavlinkInterface::GetActuatorControls() {
  const std::lock_guard<std::mutex> lock(actuator_mutex_);
  return input_reference_;
}

bool MavlinkInterface::IsInputMotorAtIndex(int index) {
  const std::lock_guard<std::mutex> lock(actuator_mutex_);
  return input_is_motor_[index];
}

bool MavlinkInterface::GetArmedState() {
  const std::lock_guard<std::mutex> lock(actuator_mutex_);
  return (armed1_ || armed2_);
}

// Mavlink helper function to finalize message without global channel status
uint16_t MavlinkInterface::FinalizeOutgoingMessage(mavlink_message_t* msg, uint8_t system_id, uint8_t component_id, uint8_t min_length, uint8_t length, uint8_t crc_extra)
{
    const std::lock_guard<std::mutex> guard(mav_status_mutex_);
    return mavlink_finalize_message_buffer(msg, system_id, component_id, &sender_m_status_,
        min_length, length, crc_extra);
}
