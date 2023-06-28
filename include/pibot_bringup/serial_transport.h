#ifndef PIBOT_BRINGUP_SERIAL_TRANSPORT_H_
#define PIBOT_BRINGUP_SERIAL_TRANSPORT_H_

#include "transport.h"

namespace pibot {

class SerialTransport : public Transport {
 public:
  SerialTransport(std::string url, int32_t baudrate);
  ~SerialTransport();
  bool init();
  Buffer read();

  void write(Buffer& data);

  void set_timeout(int t);
  bool is_timeout();

 private:
  void mainRun();

  unsigned long m_timeout_us;

  bool m_timeoutFlag;
  int m_fd;
  std::string m_port;
  int32_t m_baudrate;
};

}  // namespace pibot

#endif /* TRANSPORT_SERIAL_H_ */
