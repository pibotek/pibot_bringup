#ifndef TRANSPORT_SERIAL_H_
#define TRANSPORT_SERIAL_H_

#include "transport.h"

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
  std::string m_port;
  int32_t m_baudrate;

  unsigned long m_timeout_us;
  int m_fd;
  bool m_timeoutFlag;
};

#endif /* TRANSPORT_SERIAL_H_ */
