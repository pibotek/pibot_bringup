#ifndef TRANSPORT_SERIAL_H_
#define TRANSPORT_SERIAL_H_

#include "transport.h"

class Serial_transport : public Transport
{
public:
	Serial_transport (std::string url, int32_t baudrate);
	~Serial_transport();
	bool init();
	Buffer read();

	void write(Buffer &data);

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


#endif /* TRANSPORT_SERIAL_H_ */
