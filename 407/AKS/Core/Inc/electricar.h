#ifndef ELECTRICAR_
#define ELECTRICAR_

#include "main.h"

typedef enum
{
    IDLE,
    LEFT,
    RIGHT,
    ERR

} Angle_Typedef;

float GpsToDecimalDegrees(const float value, char quadrant);
float CalculateHeadingVehicle(const float new_lat,const float new_long,const float old_lat,const float old_long);
int IS_FLOAT(char* arr);
char cci2a(char *num);
char* ccf2a(char *num);
int IS_CONTAIN(char* num,char parameter);
char* CONVERT_ASCII(char* arr[],int count);
int float2twobyte(float f,int i);
char* cf2a(float num,char str[4]);
char ci2a(int num);
uint32_t MAP(uint32_t au32_IN, uint32_t au32_INmin, uint32_t au32_INmax, uint32_t au32_OUTmin, uint32_t au32_OUTmax);
int calculate_angle(int adc_val,int max_adc_val,char which_variable);
int binarray_to_decimal(uint8_t array[8]);
void decimal_to_binarray(int dec,uint8_t dest_array[8]);
void NEXTION_SEND(UART_HandleTypeDef huart,int what_do_you_want,char* ID,int variable,char myMessage[50]);

#endif
