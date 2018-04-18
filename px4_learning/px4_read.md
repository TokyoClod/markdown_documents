# PX4 源码学习之路

## Learning Plan

1. 理清楚[姿态控制](https://github.com/TokyoClod/Firmware/tree/master/src/modules/mc_att_control)、[位置控制](https://github.com/TokyoClod/Firmware/blob/master/src/modules/mc_pos_control/mc_pos_control_main.cpp)环
2. EKF滤波器对位置、姿态的估计[ekf2_main.cpp](https://github.com/TokyoClod/Firmware/blob/master/src/modules/ekf2/ekf2_main.cpp),[ecl-ekf](https://github.com/TokyoClod/ecl/tree/c4fc0bb6f8fce94eab9bb810e45df0f785b3c2f6)
3. 尝试新的控制策略、估计方法

## Attiltude Control(姿态内环)

### 