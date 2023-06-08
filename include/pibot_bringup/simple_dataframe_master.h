#ifndef PIBOT_BRINGUP_SIMPLE_DATAFRAME_MASTER_H_
#define PIBOT_BRINGUP_SIMPLE_DATAFRAME_MASTER_H_

#include "simple_dataframe.h"
#include <string.h>

namespace pibot {

class Transport;
class SimpleDataframe : public Dataframe {
 public:
  SimpleDataframe(Transport* trans = 0);
  ~SimpleDataframe();
  void register_notify(const MESSAGE_ID id, Notify* _nf) {
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
  Transport* trans;
};

}  // namespace pibot

#endif