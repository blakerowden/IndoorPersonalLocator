/*
 ************************************************************************
 * @file hci.h
 * @author Boston O'Neill, 45798737
 * @date 04/04/22
 * @brief Host controller interface Include file.
 **********************************************************************
 */

#ifndef HCI_H
#define HCI_H

/*Define HCI packet variables*/
#define DEV_PREAMBLE 0xAA

#define HCI_ReqLen_ID 0x1

#define HTS221_Temp_ID 0x01
#define HTS221_Hum_ID 0x02
#define LPS22_AirP_ID 0x03
#define CCS811_Dev_ID 0x04
#define LIS2DH_X_ID 0x05
#define LIS2DH_Y_ID 0x06
#define LIS2DH_Z_ID 0x07
#define RGB_LED_ID 0x08
#define BUZZ_Dev_ID 0x09
#define PB_Dev_ID 0x0A

/*Include Necessary Functions for bluetooth drivers*/
void print_reading(uint8_t data[14]);

void print_JSON(struct shell *shell, uint8_t data[14]);


#endif /*HCI_H*/