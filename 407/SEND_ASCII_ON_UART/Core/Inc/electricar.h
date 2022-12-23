#ifndef ELECTRICAR_
#define ELECTRICAR_

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

#endif
