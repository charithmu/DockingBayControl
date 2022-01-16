#include <Ticker.h>
#include <Wire.h>
#include <Arduino.h>
#include "Adafruit_VL6180X.h"
#include "EspMQTTClient.h"
#include "motorController.h"

////////////////////////////////////
// Definitions
////////////////////////////////////

// motor controller pins
#define AIN1 26
#define AIN2 25
#define APWM 27
#define BIN1 15
#define BIN2 14
#define BPWM 4
#define STBY 33

//ToF sensor pins
#define XSHUT 32

//Limit Switches pins
#define ASL 23
#define AEL 19
#define BSL 5
#define BEL 18

//configurations
#define TRIG_DIST 15

//constants
const String ID = "BAY002S";

const String statusTopic = "fleet/dockingBay/" + ID + "/status";
const String commandTopic = "fleet/dockingBay/" + ID + "/command";
const String firstMsg = ID + " : Now Online";
const String lastMsg = ID + " : Going Offline";

enum DockBayState
{
  INIT,
  UNLOCKING,
  UNLOCKED,
  LOCKING,
  LOCKED,
  AUTO,
  ERROR
};

enum COMMAND
{
  C_NOP,
  C_INIT,
  C_LOCK,
  C_UNLOCK,
  C_AUTO
};

////////////////////////////////////
// Initializations
////////////////////////////////////

EspMQTTClient espmqttclient(
    "DroneLab",
    "dronelab",
    "10.42.0.1", // MQTT Broker server ip
    // "DroneLabMQTT",   // Can be omitted if not needed
    // "dronelab", // Can be omitted if not needed
    ID.c_str(), // Client name that uniquely identify your device
    1883        // The MQTT port, default to 1883. this line can be omitted
);

Ticker tofTimer;
Adafruit_VL6180X tof = Adafruit_VL6180X();

Motor motA = Motor(AIN1, AIN2, APWM, STBY, ASL, AEL);
Motor motB = Motor(BIN1, BIN2, BPWM, STBY, BSL, BEL);

DockBayState bayState = INIT;
DockBayState prevState = ERROR;

COMMAND cmd = C_NOP;

bool proximityTrigger = false;
bool tofOK = false;
bool verbose = true;

////////////////////////////////////
//  Forward Declarations
////////////////////////////////////

// uint8_t readRange();
// void evalFSM();
// void initESPMqtt();
// void onConnectionEstablished();
// void checkTOFRange();

////////////////////////////////////
//  SETUP
////////////////////////////////////
void setup()
{
  // wait for serial port to open on native usb devices
  Serial.begin(115200);
  while (!Serial)
  {
    delay(1);
  }

  // init MQTT
  initESPMqtt();

  //initialize ToF sensor
  // digitalWrite(XSHUT, LOW);
  // digitalWrite(XSHUT, HIGH);
  if (!tof.begin())
  {
    Serial.println("Failed to find ToF sensor");
    tofOK = false;
    //while (1); //stuck here
  }
  else
  {
    Serial.println("ToF Sensor found!");
    tofOK = true;
  }

  uint8_t result = readRange();
  Serial.println(result);

  //attach timer intruppt
  tofTimer.attach_ms(100, checkTOFRange);
}

////////////////////////////////////
//  MAIN LOOP
////////////////////////////////////
void loop()
{
  espmqttclient.loop();
  evalFSM();

  // Serial.print("mot A:");
  // Serial.println(motA.checkLim());
  // Serial.print("mot B:");
  // Serial.println(motB.checkLim());
  // delay(500);
  
  // uint8_t result = readRange();
  // Serial.println(result);
  // delay(500);

}

////////////////////////////////////
//  Functions
////////////////////////////////////

/**
 * @brief initialize espMQTT, Optional functionalities of EspMQTTClient
 * 
 */
void initESPMqtt()
{
  Serial.println("MQTT INIT...");
  espmqttclient.enableDebuggingMessages();                                   // Enable debugging messages sent to serial output
  espmqttclient.enableHTTPWebUpdater();                                      // Enable the web updater. User and password default to values of MQTTUsername and MQTTPassword. These can be overridded with enableHTTPWebUpdater("user", "password").
  espmqttclient.enableOTA();                                                 // Enable OTA (Over The Air) updates. Password defaults to MQTTPassword. Port is the default OTA port. Can be overridden with enableOTA("password", port).
  espmqttclient.enableLastWillMessage(statusTopic.c_str(), lastMsg.c_str()); // You can activate the retain flag by setting the third parameter to true
}

/**
 * @brief This function is called once everything is connected (Wifi and MQTT). MANDETORY!!
 * 
 */
void onConnectionEstablished()
{
  Serial.println("WIFI+MQTT READY!!");
  espmqttclient.subscribe(commandTopic, commandCallback);
  espmqttclient.publish(statusTopic, firstMsg); // You can activate the retain flag by setting the third parameter to true
}

/**
 * @brief mqtt subscriber callback
 * 
 * @param topic 
 * @param msg 
 */
void commandCallback(const String &topic, const String &msg)
{
  // Serial.println("In Subscriber callback..");
  if (msg == "lock")
  {
    cmd = C_LOCK;
  }
  else if (msg == "unlock")
  {
    cmd = C_UNLOCK;
  }
  else if (msg == "auto")
  {
    cmd = C_AUTO;
  }
  else if (msg == "init")
  {
    cmd = C_INIT;
  }
  else if (msg == "nop")
  {
    cmd = C_NOP;
  }
}

/**
 * @brief sensor read
 * 
 * @return uint8_t 
 */
uint8_t readRange()
{
  if (tofOK)
  {
    uint8_t range = tof.readRange();
    uint8_t status = tof.readRangeStatus();

    if (status == VL6180X_ERROR_NONE)
    {
      // Serial.print("Range: ");
      return range;
    }
    // If some error occurred, print it out!
    if ((status >= VL6180X_ERROR_SYSERR_1) && (status <= VL6180X_ERROR_SYSERR_5))
    {
      Serial.println("System error");
    }
    else if (status == VL6180X_ERROR_ECEFAIL)
    {
      Serial.println("ECE failure");
    }
    else if (status == VL6180X_ERROR_NOCONVERGE)
    {
      Serial.println("No convergence");
    }
    else if (status == VL6180X_ERROR_RANGEIGNORE)
    {
      Serial.println("Ignoring range");
    }
    else if (status == VL6180X_ERROR_SNR)
    {
      Serial.println("Signal/Noise error");
    }
    else if (status == VL6180X_ERROR_RAWUFLOW)
    {
      Serial.println("Raw reading underflow");
    }
    else if (status == VL6180X_ERROR_RAWOFLOW)
    {
      Serial.println("Raw reading overflow");
    }
    else if (status == VL6180X_ERROR_RANGEUFLOW)
    {
      Serial.println("Range reading underflow");
    }
    else if (status == VL6180X_ERROR_RANGEOFLOW)
    {
      Serial.println("Range reading overflow");
    }
    return -1;
  }
  else
  {
    Serial.println("ToF sensor not ready!");
    return -1;
  }
}

/**
 * @brief timer callback
 *
 */
void checkTOFRange()
{
  uint8_t result = readRange();

  // Serial.println("Distance: " + result);

  proximityTrigger = (result <= TRIG_DIST) ? true : false;
}

/**
 * @brief Finite State Machine evaluation
 * 
 */
void evalFSM()
{
  switch (bayState)
  {
  case INIT:
  {
    if (!(prevState == INIT))
    {
      Serial.println("DOCKING BAY STATE: INIT");
      String msg = ID + " : INIT";
      espmqttclient.publish(statusTopic, msg, false);

      prevState = INIT;
    }

    // init motor A by homing;
    Serial.println("Init Motor A..");
    Serial.println(motA.checkLim());

    if (motA.home())
    {
      Serial.println("Motor A initialized");
    }
    else
    {
      Serial.println("Failed to init motor A");
      bayState = ERROR;
      break;
    }
    Serial.println("DONE Init Motor A..");
    Serial.println(motA.checkLim());

    // init motor B by homing
    Serial.println("Init Motor B..");
    Serial.println(motB.checkLim());

    if (motB.home())
    {
      Serial.println("Motor B initialized");
    }
    else
    {
      Serial.println("Failed to init motor B");
      bayState = ERROR;
      break;
    }
    Serial.println("DONE Init Motor A..");
    Serial.println(motB.checkLim());

    bayState = UNLOCKED; // init Done
    break;
  }

  case UNLOCKED:
  {
    if (!(prevState == UNLOCKED))
    {
      Serial.println("DOCKING BAY STATE: UNLOCKED");
      String msg = ID + " : UNLOCKED";
      espmqttclient.publish(statusTopic, msg, false);

      prevState = UNLOCKED;
    }

    bayState = evalCommands(UNLOCKED);

    break;
  }
  case LOCKED:
  {

    if (!(prevState == LOCKED))
    {
      Serial.println("DOCKING BAY STATE: LOCKED");
      String msg = ID + " : LOCKED";
      espmqttclient.publish(statusTopic, msg, false);

      prevState = LOCKED;
    }

    bayState = evalCommands(LOCKED);

    break;
  }

  case UNLOCKING:
  {
    if (!(prevState == UNLOCKING))
    {
      Serial.println("DOCKING BAY STATE: UNLOCKING");
      String msg = ID + " : UNLOCKING";
      espmqttclient.publish(statusTopic, msg, false);

      prevState = UNLOCKING;
    }

    bool a = false;
    bool b = false;

    // mot A fwd until end limit
    if (motA.checkLim() == Motor::END)
    {
      motA.brake();
      motA.stop();
      a = true;
    }
    else
    {
      motA.fwd();
    }

    // mot A fwd until end limit
    if (motB.checkLim() == Motor::END)
    {
      motB.brake();
      motB.stop();
      b = true;
    }
    else
    {
      motB.fwd();
    }

    // only change state if both motA and motB are at end limits (unlock position)
    bayState = (a && b) ? UNLOCKED : UNLOCKING;

    break;
  }
  case LOCKING:
  {
    if (!(prevState == LOCKING))
    {
      Serial.println("DOCKING BAY STATE: LOCKING");
      String msg = ID + " : LOCKING";
      espmqttclient.publish(statusTopic, msg, false);

      prevState = LOCKING;
    }

    bool a = false;
    bool b = false;

    //mot A rev until start limit
    if (motA.checkLim() == Motor::START)
    {
      motA.brake();
      motA.stop();
      a = true;
    }
    else
    {
      motA.rev();
    }

    //mot B rev until start limit
    if (motB.checkLim() == Motor::START)
    {
      motB.brake();
      motB.stop();
      b = true;
    }
    else
    {
      motB.rev();
    }

    // only change state if both motA and motB are at start limits (lock position)
    bayState = (a && b) ? LOCKED : LOCKING;

    break;
  }

  case AUTO:
  {
    if (!(prevState == AUTO))
    {
      Serial.println("DOCKING BAY STATE: AUTO");
      String msg = ID + " : AUTO";
      espmqttclient.publish(statusTopic, msg, false);

      prevState = AUTO;
    }

    DockBayState nextState = evalCommands(AUTO);
    bayState = (nextState == AUTO && proximityTrigger == true) ? LOCKING : nextState; // if no external commands process triggers
    break;
  }
  case ERROR:
  {
    if (!(prevState == ERROR))
    {
      Serial.println("DOCKING BAY STATE: ERROR");
      String msg = ID + " : ERROR";
      espmqttclient.publish(statusTopic, msg, false);

      prevState = ERROR;
    }

    bayState = evalCommands(ERROR);
    break;
  }

  default:
    Serial.println("DOKCING BAY STATE: UNKNOWN!!!");
    break;
  }
}

DockBayState evalCommands(DockBayState currentState)
{
  DockBayState nextState = currentState;

  if (currentState == ERROR)
  {
    return (cmd == C_INIT) ? INIT : ERROR;
  }

  switch (cmd)
  {

  case C_NOP:
    nextState = currentState;
    break;

  case C_INIT:
    nextState = INIT;
    break;

  case C_AUTO:
    if (currentState == UNLOCKED)
    {
      nextState = AUTO;
    }
    else
    {
      nextState = currentState; //no change
    }
    break;

  case C_LOCK:
    if (currentState == LOCKED)
    {
      nextState = currentState; //no change
    }
    else
    {
      nextState = LOCKING;
    }

    break;

  case C_UNLOCK:
    if (currentState == UNLOCKED)
    {
      nextState = currentState; // no change
    }
    else
    {
      nextState = UNLOCKING;
    }
    break;

  default:
    break; //unknown command
  }

  cmd = C_NOP; // clear command
  return nextState;
}