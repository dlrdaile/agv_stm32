#include "main.h"
class mecumnamu
{
public:
    mecumnamu(float a,float b);
    ~mecumnamu();
    void wheel2center(const float wheel_speed[], float *center_speed);
    void center2wheel(float *wheel_speed,const float center_speed[]);
    void center2wheel(float *wheel_speed,const int16_t center_speed[]);
private:
    float sum;
};