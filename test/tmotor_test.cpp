#include "t_motor.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <pthread.h>
#include <ctime>
#include <cstdlib>
#include "unistd.h"
#include <signal.h>

uint8_t motorId = 1;
TMotorParam_t motorParam;

void setMotorPosition(double_t p_des, double_t kp)
{
  motorParam.p_des = p_des;
  motorParam.v_des = 0;
  motorParam.kp = kp;
  motorParam.kd = 0;
  motorParam.t_ff = 0;
  tmotor_transmit(motorId, &motorParam);
  tmotor_receive(motorId, &motorParam);
}

void setMotorVelocity(double_t v_des, double_t kd)
{
  motorParam.p_des = 0;
  motorParam.v_des = v_des;
  motorParam.kp = 0;
  motorParam.kd = kd;
  motorParam.t_ff = 0;
  tmotor_transmit(motorId, &motorParam);
  tmotor_receive(motorId, &motorParam);
}

void setMotorTorque(double_t t_ff)
{
  motorParam.p_des = 0;
  motorParam.v_des = 0;
  motorParam.kp = 0;
  motorParam.kd = 0;
  motorParam.t_ff = t_ff;
  tmotor_transmit(motorId, &motorParam);
  tmotor_receive(motorId, &motorParam);
}

void test()
{
  bool result = false;
  result = tmotor_setCtrlMode(motorId);
  if (result == false)
  {
    return;
  }
  printf("set ctrl mode success.\n");
  result = tmorot_setZeroPosition(motorId);
  if (result == false)
  {
    return;
  }
  printf("set zero pos success.\n");

  uint32_t count = 0;

  while (1)
  {
    // setMotorTorque(0.6);
    setMotorVelocity(5, 2);
    // setMotorPosition(count * 0.1, 10);
    count++;
    printf("p_des: %f, v_des: %f, kp: %f, kd: %f, t_ff: %f\n", motorParam.p_des, motorParam.v_des, motorParam.kp, motorParam.kd, motorParam.t_ff);
    printf("p: %f, v: %f, I: %f\n", motorParam.position, motorParam.velocity, motorParam.current);
    usleep(1000 * 100);
  }
}

void sigintHandler(int sig)
{
  printf("reset ctrl mode.\n");
  tmorot_resetCtrlMod(motorId);
  can_close();
  printf("signal exit.\n");
  exit(EXIT_SUCCESS);
}

int main(int argc, char **argv)
{
  signal(SIGINT, sigintHandler);
  bool result = false;
  result = can_init();
  if (result == false)
  {
    perror("device init failed!\n");
    return 1;
  }

  test();

  return 0;
}