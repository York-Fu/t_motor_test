#ifndef _t_motor_h_
#define _t_motor_h_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdio.h>
#include <stdint.h>
#include <math.h>

typedef struct
{
  double_t p_des;
  double_t v_des;
  double_t kp;
  double_t kd;
  double_t t_ff;
  double_t position;
  double_t velocity;
  double_t current;
} TMotorParam_t;

bool can_init();
bool can_close();
bool tmotor_setCtrlMode(uint8_t id);
bool tmorot_resetCtrlMod(uint8_t id);
bool tmorot_setZeroPosition(uint8_t id);
bool tmotor_transmit(uint8_t id, TMotorParam_t *param);
bool tmotor_receive(uint8_t id, TMotorParam_t *param);

#ifdef __cplusplus
}
#endif

#endif