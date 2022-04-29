/**
 ************************************************************************
 * @file imu_mobile.h
 * @author Liana van Teijlingen
 * @date 20.04.2021
 * @brief Contains required definitions of imu_mobile.c
 **********************************************************************
 **/

#ifndef IMU_MOBILE_H
#define IMU_MOBILE_H

void init_lis(void);
double read_lis(int sensor);

#endif /* IMU_MOBILE_H */