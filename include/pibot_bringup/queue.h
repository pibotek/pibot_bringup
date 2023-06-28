#ifndef PIBOT_BRINGUP_QUEUE_H_
#define PIBOT_BRINGUP_QUEUE_H_

namespace pibot {

class Queue {
 public:
  virtual bool put(unsigned char ch) = 0;
  virtual bool get(unsigned char& ch) = 0;

  virtual unsigned short size() = 0;
  virtual unsigned short max_size() = 0;
};

}  // namespace pibot

#endif