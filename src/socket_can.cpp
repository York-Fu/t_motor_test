#include "t_motor.h"
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -45.0f
#define V_MAX 45.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -18.0f
#define T_MAX 18.0f

#define PRINT_CAM_FRAME false

int socket_can;

bool can_init()
{
  struct sockaddr_can addr;
  struct can_frame frame;
  struct ifreq ifr;
  const char *ifname = "can0";

  if ((socket_can = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
  {
    perror("Error while opening socket");
    return false;
  }

  strcpy(ifr.ifr_name, ifname);
  ioctl(socket_can, SIOCGIFINDEX, &ifr);

  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  printf("%s at index %d\n", ifname, ifr.ifr_ifindex);

  if (bind(socket_can, (struct sockaddr *)&addr, sizeof(addr)) < 0)
  {
    perror("Error in socket bind");
    return false;
  }

  return true;
}

bool can_close()
{
  close(socket_can);
  return true;
}

int float_to_uint(float x, float x_min, float x_max, int bits)
{
  /// Converts a float to an unsigned int, given range and number of bits ///
  float span = x_max - x_min;
  if (x < x_min)
    x = x_min;
  else if (x > x_max)
    x = x_max;
  return (int)((x - x_min) * ((float)((1 << bits) - 1)) / span);
}

// int float_to_uint(float x, float x_min, float x_max, int bits)
// {
//   /// Converts a float to an unsigned int, given range and number of bits ///
//   float span = x_max - x_min;
//   float offset = x_min;
//   return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
// }

float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
  /// converts unsigned int to float, given range and number of bits ///
  float span = x_max - x_min;
  float offset = x_min;
  return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

void pack_cmd(uint8_t *msg, double_t p_des, double_t v_des, double_t kp, double_t kd, double_t t_ff)
{
  int32_t p, v, int_kp, int_kd, t;
  p = float_to_uint(p_des, P_MIN, P_MAX, 16);
  v = float_to_uint(v_des, V_MIN, V_MAX, 12);
  int_kp = float_to_uint(kp, KP_MIN, KP_MAX, 12);
  int_kd = float_to_uint(kd, KD_MIN, KD_MAX, 12);
  t = float_to_uint(t_ff, T_MIN, T_MAX, 12);

  msg[0] = p >> 8;
  msg[1] = p & 0xFF;
  msg[2] = v >> 4;
  msg[3] = ((v & 0xF) << 4) | (int_kp >> 8);
  msg[4] = int_kp & 0xFF;
  msg[5] = int_kd >> 4;
  msg[6] = ((int_kd & 0xF) << 4) | (t >> 8);
  msg[7] = t & 0xFF;

  // msg[0] = 0x80;
  // msg[1] = 0x21;
  // msg[2] = 0x7f;
  // msg[3] = 0xf0;
  // msg[4] = 0x31;
  // msg[5] = 0x00;
  // msg[6] = 0x07;
  // msg[7] = 0xff;
}

void unpack_reply(uint8_t *msg, uint8_t *id, double_t *pos, double_t *vel, double_t *current)
{
  int32_t p, v, i;
  *id = msg[0];
  p = (msg[1] << 8) | msg[2];
  v = (msg[3] << 4) | (msg[4] >> 4);
  i = ((msg[4] & 0xF) << 8) | (msg[5]);
  *pos = uint_to_float(p, P_MIN, P_MAX, 16);
  *vel = uint_to_float(v, V_MIN, V_MAX, 12);
  *current = uint_to_float(i, T_MIN, T_MAX, 12);
}

bool tmotor_setCtrlMode(uint8_t id)
{
  int32_t nbytes = 0;
  struct can_frame frame;
  frame.can_id = id;
  frame.can_dlc = 8;
  for (int i = 0; i < frame.can_dlc; i++)
  {
    frame.data[i] = 0xff;
  }
  frame.data[7] = 0xfc;
  nbytes = write(socket_can, &frame, sizeof(struct can_frame));
  // printf("write %d bytes\n", nbytes);
  if (nbytes == -1)
  {
    printf("Error set ctrl mode!\n");
    return false;
  }
  return true;
}

bool tmorot_resetCtrlMod(uint8_t id)
{
  int32_t nbytes = 0;
  struct can_frame frame;
  frame.can_id = 0x001;
  frame.can_dlc = 8;
  for (int i = 0; i < frame.can_dlc; i++)
  {
    frame.data[i] = 0xff;
  }
  frame.data[7] = 0xfd;
  nbytes = write(socket_can, &frame, sizeof(struct can_frame));
  // printf("write %d bytes\n", nbytes);
  if (nbytes == -1)
  {
    printf("Error reset ctrl mode!\n");
    return false;
  }
  return true;
}

bool tmorot_setZeroPosition(uint8_t id)
{
  int32_t nbytes = 0;
  struct can_frame frame;
  frame.can_id = 0x001;
  frame.can_dlc = 8;
  for (int i = 0; i < frame.can_dlc; i++)
  {
    frame.data[i] = 0xff;
  }
  frame.data[7] = 0xfe;
  nbytes = write(socket_can, &frame, sizeof(struct can_frame));
  // printf("write %d bytes\n", nbytes);
  if (nbytes == -1)
  {
    printf("Error set zero position!\n");
    return false;
  }
  return true;
}

bool tmotor_transmit(uint8_t id, TMotorParam_t *param)
{
  int32_t nbytes = 0;
  struct can_frame can_freme;
  can_freme.can_id = 0x001;
  can_freme.can_dlc = 8;
  pack_cmd(can_freme.data, param->p_des, param->v_des, param->kp, param->kd, param->t_ff);
#if PRINT_CAM_FRAME
  printf("send fream: ");
  for (int i = 0; i < can_freme.can_dlc; i++)
  {
    printf("0x%02X ", can_freme.data[i]);
  }
  printf("\n");
#endif
  nbytes = write(socket_can, &can_freme, sizeof(struct can_frame));
  if (nbytes == -1)
  {
    printf("Errot tmotor transmit!\n");
    return false;
  }
  return true;
}

bool tmotor_receive(uint8_t id, TMotorParam_t *param)
{
  int32_t nbytes = 0;
  struct can_frame can_freme;
  nbytes = read(socket_can, &can_freme, sizeof(struct can_frame));
  if (nbytes < 0)
  {
    printf("Errot tmotor receive!\n");
    return false;
  }
#if PRINT_CAM_FRAME
  printf("receive fream: ");
  for (int i = 0; i < can_freme.can_dlc; i++)
  {
    printf("0x%02X ", can_freme.data[i]);
  }
  printf("\n");
#endif
  uint8_t m_id;
  double_t p = 0, v = 0, I = 0;
  unpack_reply(can_freme.data, &m_id, &p, &v, &I);
  if (m_id != id)
  {
    return false;
  }
  param->position = p;
  param->velocity = v;
  param->current = I;
  return true;
}
