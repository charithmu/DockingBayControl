#include "Arduino.h"

class Motor
{
private:
    int in1, in2, pwm, stby, eLim, sLim;

public:
    enum MotPos
    {
        MID  = 0,
        START  = 1,
        END  = 2,
        ERROR = 3
    };

    Motor(int in1Pin, int in2Pin, int pwmPin, int stbyPin, int slPin, int elPin);

    void fwd();

    void rev();

    void brake();

    void stop();

    void off();

    MotPos checkLim();

    bool home();
};