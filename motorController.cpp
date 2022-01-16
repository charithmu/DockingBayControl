
#include "motorController.h"
#include "Arduino.h"

Motor::Motor(int in1Pin, int in2Pin, int pwmPin, int stbyPin, int slPin, int elPin)
{
    in1 = in1Pin;
    in2 = in2Pin;
    pwm = pwmPin;
    stby = stbyPin;
    eLim = elPin;
    sLim = slPin;

    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    pinMode(pwm, OUTPUT);
    pinMode(stby, OUTPUT);
    pinMode(eLim, INPUT);
    pinMode(sLim, INPUT);
}

void Motor::fwd()
{
    digitalWrite(stby, HIGH);
    digitalWrite(pwm, HIGH);
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
}

void Motor::rev()
{
    digitalWrite(stby, HIGH);
    digitalWrite(pwm, HIGH);
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
}

void Motor::brake()
{
    digitalWrite(stby, HIGH);
    digitalWrite(in1, HIGH);
    digitalWrite(in2, HIGH);
    digitalWrite(pwm, LOW);
}

void Motor::stop()
{
    digitalWrite(stby, HIGH);
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(pwm, HIGH);
}

void Motor::off()
{
    digitalWrite(stby, LOW);
}

/**
 * @brief Poll limit swithes to determine motor location.
 * 
 * @return Motor::MotPos
 */
Motor::MotPos Motor::checkLim()
{
    if ((digitalRead(sLim) == LOW) && (digitalRead(eLim) == LOW))
    {
        return ERROR;
    }
    else if ((digitalRead(sLim) == HIGH) && (digitalRead(eLim) == HIGH))
    {
        return MID;
    }
    else if ((digitalRead(sLim) == HIGH) && (digitalRead(eLim) == LOW))
    {
        return END;
    }
    else if ((digitalRead(sLim) == LOW) && (digitalRead(eLim) == HIGH))
    {
        return START;
    }

    return ERROR;
}

/**
 * @brief Try to move forward and backward to find the actual position of motor. And then move to unlock position.
 * 
 * @return true; if success
 * @return false; if any error
 */
bool Motor::home()
{

    uint32_t duration = 5000;
    uint32_t startTime = millis();

    MotPos currrentPos = ERROR;

    bool forwardCheck = true;
    bool backwardCheck = false;

    // Try and move in forward direction and check to hit end limit or timeout
    Serial.println("Forward check...");
    while (forwardCheck)
    {
        fwd();

        if (checkLim() == END)
        {
            Serial.println("end limit");
            brake();
            stop();
            currrentPos = END;
            break;
        }

        if ((millis() - startTime) > duration)
        {
            Serial.println("timeout!");
            brake();
            stop();
            backwardCheck = true; // enable backward check
            break;
        }
    }

    // Try and move in backward direction and check to hit start limit or timeout
    startTime = millis();
    Serial.println("Reverse check...");
    while (backwardCheck)
    {
        rev();

        if (checkLim() == START)
        {
            Serial.println("start limit");
            brake();
            stop();
            currrentPos = START;
            break;
        }

        if ((millis() - startTime) > duration)
        {
            Serial.println("timeout!");
            brake();
            stop();
            break;
        }
    }

    if (currrentPos == START) // if at locked position try to unlock
    {
        Serial.println("Unlocking");
        startTime = millis();
        while (true)
        {
            fwd();
            if (checkLim() == END) //unlocked
            {
                Serial.println("end limit");
                brake();
                stop();
                currrentPos = END;
                break;
            }

            if ((millis() - startTime) > duration) //cannot unlock
            {
                Serial.println("timeout!");
                brake();
                stop();
                currrentPos = ERROR;
                break;
            }
        }
    }

    if (currrentPos == END)
    {
        Serial.println("Homing Success!");
        return true;
    }
    else if (currrentPos == ERROR)
    {
        Serial.println("Homing Failed!");
        return false;
    }

    return false;
}