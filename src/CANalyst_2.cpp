#include "t_motor.h"
#include "controlcan.h"

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

DWORD DeviceIndex = 0, CANIndex = 0;

bool can_init()
{
  if (VCI_OpenDevice(VCI_USBCAN2, DeviceIndex, 0) == 1) //打开设备
  {
    printf("open deivce success!\n"); //打开设备成功
  }
  else
  {
    printf("open deivce error!\n");
    return false;
  }

  //初始化参数，严格参数二次开发函数库说明书。
  VCI_INIT_CONFIG config;
  config.AccCode = 0;
  config.AccMask = 0xFFFFFFFF;
  config.Filter = 1;     // 接收所有帧
  config.Timing0 = 0x00; // 波特率
  config.Timing1 = 0x14;
  config.Mode = 0; // 正常模式

  if (VCI_InitCAN(VCI_USBCAN2, DeviceIndex, CANIndex, &config) != 1)
  {
    printf("Init CAN1 error\n");
    VCI_CloseDevice(VCI_USBCAN2, DeviceIndex);
    return false;
  }

  if (VCI_StartCAN(VCI_USBCAN2, DeviceIndex, CANIndex) != 1)
  {
    printf("Start CAN1 error\n");
    VCI_CloseDevice(VCI_USBCAN2, DeviceIndex);
    return false;
  }
  return true;
}

bool can_close()
{
  int32_t result = 0;
  result = VCI_CloseDevice(VCI_USBCAN2, DeviceIndex);
  if (result < 1)
  {
    perror("close device failed!\n");
    return false;
  }
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

  // msg[0] = 0x7f;
  // msg[1] = 0xff;
  // msg[2] = 0x8e;
  // msg[3] = 0x30;
  // msg[4] = 0x00;
  // msg[5] = 0x33;
  // msg[6] = 0x37;
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
  int32_t result = 0;
  //需要发送的帧，结构体设置
  VCI_CAN_OBJ can_freme;
  can_freme.ID = id;
  can_freme.SendType = 1;
  can_freme.RemoteFlag = 0;
  can_freme.ExternFlag = 0;
  can_freme.DataLen = 8;
  for (int i = 0; i < can_freme.DataLen; i++)
  {
    can_freme.Data[i] = 0xff;
  }
  can_freme.Data[7] = 0xfc;
  result = VCI_Transmit(VCI_USBCAN2, DeviceIndex, CANIndex, &can_freme, 1);
  if (result < 1)
  {
    perror("set ctrl mode failed!\n");
    return false;
  }
  return true;
}

bool tmorot_resetCtrlMod(uint8_t id)
{
  int32_t result = 0;
  //需要发送的帧，结构体设置
  VCI_CAN_OBJ can_freme;
  can_freme.ID = id;
  can_freme.SendType = 0;
  can_freme.RemoteFlag = 0;
  can_freme.ExternFlag = 0;
  can_freme.DataLen = 8;
  for (int i = 0; i < can_freme.DataLen; i++)
  {
    can_freme.Data[i] = 0xff;
  }
  can_freme.Data[7] = 0xfd;
  result = VCI_Transmit(VCI_USBCAN2, DeviceIndex, CANIndex, &can_freme, 1);
  if (result < 1)
  {
    perror("reset ctrl mode failed!\n");
    return false;
  }
  return true;
}

bool tmorot_setZeroPosition(uint8_t id)
{
  int32_t result = 0;
  //需要发送的帧，结构体设置
  VCI_CAN_OBJ can_freme;
  can_freme.ID = id;
  can_freme.SendType = 0;
  can_freme.RemoteFlag = 0;
  can_freme.ExternFlag = 0;
  can_freme.DataLen = 8;
  for (int i = 0; i < can_freme.DataLen; i++)
  {
    can_freme.Data[i] = 0xff;
  }
  can_freme.Data[7] = 0xfe;
  result = VCI_Transmit(VCI_USBCAN2, DeviceIndex, CANIndex, &can_freme, 1);
  if (result < 1)
  {
    perror("set zero failed!\n");
    return false;
  }
  return true;
}

bool tmotor_transmit(uint8_t id, TMotorParam_t *param)
{
  int32_t result = 0;
  VCI_CAN_OBJ can_freme;
  can_freme.ID = id;
  can_freme.SendType = 1;
  can_freme.RemoteFlag = 0;
  can_freme.ExternFlag = 0;
  can_freme.DataLen = 8;
  pack_cmd(can_freme.Data, param->p_des, param->v_des, param->kp, param->kd, param->t_ff);
#if PRINT_CAM_FRAME
  printf("send fream: ");
  for (int i = 0; i < can_freme.DataLen; i++)
  {
    printf("0x%02X ", can_freme.Data[i]);
  }
  printf("\n");
#endif
  result = VCI_Transmit(VCI_USBCAN2, DeviceIndex, CANIndex, &can_freme, 1);
  if (result < 1)
  {
    perror("tmotor transmit failed!\n");
    return false;
  }
  return true;
}

bool tmotor_receive(uint8_t id, TMotorParam_t *param)
{
  int32_t result = 0;
  VCI_CAN_OBJ can_freme;
  result = VCI_Receive(VCI_USBCAN2, DeviceIndex, CANIndex, &can_freme, 1, 0);
  if (result < 1)
  {
    printf("tmotor receive failed! receive %d fream\n", result);
    return false;
  }
#if PRINT_CAM_FRAME
  printf("receive fream: ");
  for (int i = 0; i < can_freme.DataLen; i++)
  {
    printf("0x%02X ", can_freme.Data[i]);
  }
  printf("\n");
#endif
  uint8_t m_id;
  double_t p = 0, v = 0, I = 0;
  unpack_reply(can_freme.Data, &m_id, &p, &v, &I);
  if (m_id != id)
  {
    return false;
  }
  param->position = p;
  param->velocity = v;
  param->current = I;
  return true;
}
