# t motor test
测试t_motor电机MIT模式功能，编译时可选socket_can接口或CANalyst_2接口。
### build
```
mkdir build && cd build
cmake ..
make -j`nproc`
```
### run
```
./tmotor_test
```