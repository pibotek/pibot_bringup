#ifndef PIBOT_SIMPLE_DATAFRAME_MASTER_H_
#define PIBOT_SIMPLE_DATAFRAME_MASTER_H_

#include <string.h>
#include <memory>
#include "simple_dataframe.h"

class Transport;
class SimpleDataframe : public Dataframe {
 public:
  SimpleDataframe(std::shared_ptr<Transport> trans);
  ~SimpleDataframe();
  void register_notify(const MESSAGE_ID __attribute__((unused)) id, Notify __attribute__((unused)) * nf) {
  }

  bool data_recv(unsigned char c);
  bool data_parse();
  bool init();
  bool interact(const MESSAGE_ID id);

 private:
  bool recv_proc();

 private:
  bool send_message(const MESSAGE_ID id);
  bool send_message(const MESSAGE_ID id, unsigned char* data, unsigned char len);
  bool send_message(Message* msg);

 private:
  Message active_rx_msg;

  RECEIVE_STATE recv_state;
  std::shared_ptr<Transport> trans_;
};
#endif