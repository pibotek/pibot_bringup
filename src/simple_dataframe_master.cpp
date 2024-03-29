#include "simple_dataframe_master.h"
#include "data_holder.h"
#include <ros/ros.h>
#include <stdio.h>

#include "transport.h"

namespace pibot {

SimpleDataframe::SimpleDataframe(Transport* _trans) : trans(_trans) {
  recv_state = STATE_RECV_FIX;
}

SimpleDataframe::~SimpleDataframe() {
}

bool SimpleDataframe::init() {
  trans->set_timeout(500);
  return true;
}

bool SimpleDataframe::data_recv(unsigned char c) {
  // printf("%02x ", c);
  switch (recv_state) {
    case STATE_RECV_FIX:
      if (c == FIX_HEAD) {
        memset(&active_rx_msg, 0, sizeof(active_rx_msg));
        active_rx_msg.head.flag = c;
        active_rx_msg.check += c;

        recv_state = STATE_RECV_ID;
      } else {
        recv_state = STATE_RECV_FIX;
      }
      break;
    case STATE_RECV_ID:
      if (c < ID_MESSGAE_MAX) {
        active_rx_msg.head.msg_id = c;
        active_rx_msg.check += c;
        recv_state = STATE_RECV_LEN;
      } else {
        recv_state = STATE_RECV_FIX;
      }
      break;
    case STATE_RECV_LEN:
      active_rx_msg.head.length = c;
      active_rx_msg.check += c;
      if (active_rx_msg.head.length == 0) {
        recv_state = STATE_RECV_CHECK;
      } else {
        recv_state = STATE_RECV_DATA;
      }
      break;
    case STATE_RECV_DATA:
      active_rx_msg.data[active_rx_msg.recv_count++] = c;
      active_rx_msg.check += c;
      if (active_rx_msg.recv_count >= active_rx_msg.head.length) {
        recv_state = STATE_RECV_CHECK;
      }
      break;
    case STATE_RECV_CHECK:
      recv_state = STATE_RECV_FIX;
      if (active_rx_msg.check == c) {
        // printf("\r\n");
        return true;
      }
      break;
    default:
      recv_state = STATE_RECV_FIX;
  }

  return false;
}

bool SimpleDataframe::data_parse() {
  MESSAGE_ID id = (MESSAGE_ID)active_rx_msg.head.msg_id;

  // printf("data_parse:id=%d\r\n", id);

  DataHolder* dh = DataHolder::get();
  switch (id) {
    case ID_GET_VERSION:
      memcpy(&dh->firmware_info, active_rx_msg.data, sizeof(dh->firmware_info));
      break;
    case ID_SET_ROBOT_PARAMTER:
      break;
    case ID_GET_ROBOT_PARAMTER:
      memcpy(&dh->parameter, active_rx_msg.data, sizeof(dh->parameter));
      break;
    case ID_INIT_ODOM:
      break;
    case ID_SET_VELOCITY:
      break;
    case ID_GET_ODOM:
      memcpy(&dh->odom, active_rx_msg.data, sizeof(dh->odom));
      break;
    case ID_GET_PID_DATA:
      memcpy(&dh->pid_data, active_rx_msg.data, sizeof(dh->pid_data));
      break;
    case ID_GET_IMU_DATA:
      memcpy(&dh->imu_data, active_rx_msg.data, sizeof(dh->imu_data));
      break;
    default:
      break;
  }

  return true;
}

bool SimpleDataframe::send_message(const MESSAGE_ID id) {
  Message msg(id);

  send_message(&msg);

  return true;
}

bool SimpleDataframe::send_message(const MESSAGE_ID id, unsigned char* data, unsigned char len) {
  Message msg(id, data, len);

  send_message(&msg);

  return true;
}

bool SimpleDataframe::send_message(Message* msg) {
  if (trans == 0) {
    return true;
  }

  Buffer data((unsigned char*)msg, (unsigned char*)msg + sizeof(msg->head) + msg->head.length + 1);
  trans->write(data);

  return true;
}

bool SimpleDataframe::interact(const MESSAGE_ID id) {
  // printf("make command:id=%d\r\n", id);

  DataHolder* dh = DataHolder::get();
  switch (id) {
    case ID_GET_VERSION:
      send_message(id);
      break;
    case ID_SET_ROBOT_PARAMTER:
      send_message(id, (unsigned char*)&dh->parameter, sizeof(dh->parameter));
      break;
    case ID_GET_ROBOT_PARAMTER:
      send_message(id);
      break;
    case ID_INIT_ODOM:
      send_message(id);
      break;
    case ID_SET_VELOCITY:
      send_message(id, (unsigned char*)&dh->velocity, sizeof(dh->velocity));
      break;
    case ID_GET_ODOM:
      send_message(id);
      break;
    case ID_GET_PID_DATA:
      send_message(id);
      break;
    case ID_GET_IMU_DATA:
      send_message(id);
    default:
      break;
  }

  if (!recv_proc()) {
    return false;
  }

  return true;
}

bool SimpleDataframe::recv_proc() {
  int i = 0;
  trans->set_timeout(150);
  bool got = false;
  while (ros::ok()) {
    Buffer data = trans->read();

    for (int i = 0; i < data.size(); i++) {
      if (data_recv(data[i])) {
        got = true;
        // std::cout << "ok" << std::endl;
        break;
      }
    }

    if (got) break;

    if (trans->is_timeout()) {
      ROS_WARN("timeout");
      return false;
    }
  }

  if (!data_parse()) {
    return false;
  }

  return true;
}

}  // namespace pibot