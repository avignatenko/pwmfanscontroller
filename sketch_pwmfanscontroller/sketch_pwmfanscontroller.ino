#include <PWM.h>
#include <math.h>

class TemperatureController
{
  public:

    static const float tempTresh = 0.3;

    static const float TERMIST_B = 4300;

  public:

    TemperatureController(int in_tempSensorPin, float in_vIn = 5) : m_sensorPin(in_tempSensorPin), m_vIn(in_vIn)
    {

    }

    void Init()
    {
      lastTemp[0] = GetTemperature();
      lastTemp[1] = lastTemp[0];
      lastTemp[2] = lastTemp[0];
    }

    void Step()
    {
      float temperature = GetTemperature();

      lastTemp[0] = lastTemp[1];
      lastTemp[1] = lastTemp[2];
      lastTemp[2] = temperature;
    }

    float GetTemperature()
    {
      float voltage = analogRead(m_sensorPin) * m_vIn / 1023.0;
      float r1 = voltage / (m_vIn - voltage);

      float temperature = 1. / ( 1. / (TERMIST_B) * log(r1) + 1. / (25. + 273.) ) - 273;

      return temperature;
    }

    float GetTemperatureAveraged()
    {
      float tempAverage = (lastTemp[0] + lastTemp[1] + lastTemp[2]) / 3;

      return tempAverage;
    }

  private:

    float lastTemp[3];
    float m_vIn;
    int m_sensorPin;
};

class IInterruptHandler
{
  public:

    virtual void OnInterrupt() = 0;
};



class CInterrupt
{

  public:

    CInterrupt()
      : m_handlers0Size(0)
      , m_handlers1Size(0)
    {
      ::memset(m_handlers0, sizeof(IInterruptHandler*) * BUFFER_SIZE, NULL);
      ::memset(m_handlers1, sizeof(IInterruptHandler*) * BUFFER_SIZE, NULL);
    }

    void Init()
    {
      attachInterrupt(0, &CInterrupt::OnInterrupt0, RISING);
      attachInterrupt(1, &CInterrupt::OnInterrupt1, RISING);
    }

    void Attach(int in_interruptNumber, IInterruptHandler* in_handler)
    {
      Serial.print("Interrupt: Set ");
        
      Serial.print (in_interruptNumber, DEC);
      Serial.print ("\r\n");

      if (in_interruptNumber == 0)
      {
        m_handlers0[m_handlers0Size++] = in_handler;
      }
      else if (in_interruptNumber == 1)
      {
        m_handlers1[m_handlers1Size++] = in_handler;
      }

    }

  private:

    static void OnInterrupt0();

    static void OnInterrupt1();

  private:


    static const int BUFFER_SIZE = 10;
    int m_handlers0Size;
    int m_handlers1Size;
    IInterruptHandler* m_handlers0[BUFFER_SIZE];
    IInterruptHandler* m_handlers1[BUFFER_SIZE];
};

CInterrupt Interrupt;

void CInterrupt::OnInterrupt0()
{
  for (int i = 0; i < Interrupt.m_handlers0Size; ++i)
  {
    if (Interrupt.m_handlers0[i] != NULL)
      Interrupt.m_handlers0[i]->OnInterrupt();
  }
}

void CInterrupt::OnInterrupt1()
{
  for (int i = 0; i < Interrupt.m_handlers1Size; ++i)
  {
    if (Interrupt.m_handlers1[i] != NULL)
      Interrupt.m_handlers1[i]->OnInterrupt();
  }
}


class PWMFan: public IInterruptHandler
{
  public:

    static const float MIN_TEMP_DELTA = 3; // degrees
    static const float MAX_TEMP_DELTA = 6; // degrees
    static const float MAX_TEMP_NO_FAN = 28; // degrees
    
  public:

    PWMFan(int in_number, int in_motorPin, int in_pwmPin, int in_hallSensorPin, TemperatureController& in_controller, TemperatureController& in_outsideController)
      : m_number(in_number)
      , m_motorPin(in_motorPin)
      , m_pwmPin(in_pwmPin)
      , m_hallSensorPin(in_hallSensorPin)
      , m_controllerTemperature(in_controller)
      , m_controllerTemperatureOutside(in_outsideController)
      , m_enabledDC(false)
      , m_speedWarning(false)
      , m_lastRPMUpdateTime(0)
      , m_lowRMPWarning(false)
    {

    }

    void Init()
    {

      // init motor control
      pinMode(m_motorPin, OUTPUT);
      digitalWrite(m_motorPin, LOW);

      if (m_pwmPin >= 0)
      {
        // init motor speed control
        pinMode(m_pwmPin, OUTPUT);
  
        //sets the frequency for the specified pin
        bool success = SetPinFrequencySafe(m_pwmPin,  25 * 1000 /* hz */);
  
        Serial.print("Fan ");
        Serial.print(m_number, DEC);
        Serial.print(": ");
  
        Serial.print(String("Freq: ") + (success ? "true" : "false"));
      }
   
      // init hallsensor pin
      pinMode(m_hallSensorPin, INPUT_PULLUP);

      Interrupt.Attach(digitalPinToInterrupt(m_hallSensorPin), this);
    }

    void Step()
    {
      if (m_pwmPin >=  0)
      {
        float tempAverageOutside = m_controllerTemperatureOutside.GetTemperatureAveraged();
        float tempAverage = m_controllerTemperature.GetTemperatureAveraged();
        float tempDelta = tempAverage - tempAverageOutside - MIN_TEMP_DELTA;
  
        Serial.print("Fan ");
        Serial.print(m_number, DEC);
        Serial.print(": ");
  
        Serial.print("Temp av: " + String(tempAverage) + "\r\n");
  
        Serial.print("Fan ");
        Serial.print(m_number, DEC);
        Serial.print(": ");
  
        Serial.print("Temp avout: " + String(tempAverageOutside) + "\r\n");
  
        // enable engine at all?
        bool enableDC = ((tempDelta > TemperatureController::tempTresh) && tempAverage > MAX_TEMP_NO_FAN);
        if (enableDC)
        {
          digitalWrite(m_motorPin, HIGH);
          m_enabledDC = true;
        } else if (tempDelta < - TemperatureController::tempTresh)
        {
          digitalWrite(m_motorPin, LOW);
          m_enabledDC = false;
        }
  
        if (!m_enabledDC)
        {
          pwmWrite(m_pwmPin, 0);
          m_lowRMPWarning = false;
          return;
        }
  
        float speedCoeff = tempDelta / MAX_TEMP_DELTA;
        if (speedCoeff > 1)
          speedCoeff = 1;
        else if (speedCoeff < 0)
          speedCoeff = 0;
  
        float speed = pwmMin + speedCoeff * (pwmMax  - pwmMin);
  
  
        pwmWrite(m_pwmPin, speed);
  
        Serial.print("Fan ");
        Serial.print(m_number, DEC);
        Serial.print(": ");
  
        Serial.print(String("Speed: ") + String(speed) + "\n");
      }
    
      // update rotations info
      unsigned long currentTime = millis();
      if (currentTime > m_lastRPMUpdateTime)
      {
        unsigned long delta = currentTime - m_lastRPMUpdateTime;
        const int millisUpdatePeriod = 1000; // 1sec
        if (delta > millisUpdatePeriod)
        {
          float rpm = (float)m_rotations / delta * 1000 * 60 / 2;

          Serial.print("Fan ");
          Serial.print(m_number, DEC);
          Serial.print(": ");

          Serial.print (rpm, DEC);
          Serial.print (" rpm\r\n");

          m_rotations = 0;
          m_lastRPMUpdateTime = currentTime;

          m_lowRMPWarning = (rpm < 100);
        }
      }
      else // overflow, wait for next measurement
      {
        m_lastRPMUpdateTime = currentTime;
        m_rotations = 0;
      }

    }

    bool LowRPMWarning()
    {
      return m_lowRMPWarning;
    }

    virtual void OnInterrupt()
    {
      m_rotations++;
    }

  private:

    static const float pwmMin = 20;
    static const float pwmMax = 255;

    int m_number;
    int m_motorPin;
    int m_pwmPin;
    int m_hallSensorPin;
    TemperatureController& m_controllerTemperature;
    TemperatureController& m_controllerTemperatureOutside;
    bool m_enabledDC;
    bool m_speedWarning;
    volatile int m_rotations;
    unsigned long m_lastRPMUpdateTime;
    bool m_lowRMPWarning;
};


class WarningLed
{
  public:

    WarningLed(int in_led)
      : m_led(in_led)
    {

    }

    void Init()
    {
      pinMode(m_led, OUTPUT);

    }

    void Signal(bool in_signal)
    {
      digitalWrite(m_led, in_signal ? HIGH : LOW);
    }

  private:

    int m_led;
};

const int TEMP_SENSOR_PIN1 = 0;
const int TEMP_SENSOR_PIN2 = 1;
const int TEMP_SENSOR_PIN3 = 2;

const float TEMP_SENSOR_VOLTAGE  = 5.0;

TemperatureController controllerTemperature1(TEMP_SENSOR_PIN1, TEMP_SENSOR_VOLTAGE);
TemperatureController controllerTemperature2(TEMP_SENSOR_PIN2, TEMP_SENSOR_VOLTAGE);
TemperatureController controllerTemperatureOutSide(TEMP_SENSOR_PIN3, TEMP_SENSOR_VOLTAGE);


const int MOTOR1_PIN = 8;
const int PWM1_PIN = 9;
const int HALLSENSOR1_PIN = 2;

PWMFan fan1(0, MOTOR1_PIN, PWM1_PIN, HALLSENSOR1_PIN, controllerTemperature1, controllerTemperatureOutSide);


const int MOTOR2_PIN = 10;
const int PWM2_PIN = 11;
const int HALLSENSOR2_PIN = 3;

// second fan will use the same PWM, that's why -1 there
PWMFan fan2(1, MOTOR2_PIN, -1, HALLSENSOR2_PIN, controllerTemperature1, controllerTemperatureOutSide);


const int WARNING_LED_PIN = 13;

WarningLed warningLed(WARNING_LED_PIN);

void setup()
{
  Serial.begin(9600);

  //initialize all timers except for 0, to save time keeping functions
  InitTimersSafe();

  Interrupt.Init();

  fan1.Init();
  fan2.Init();
  
  warningLed.Init();
  controllerTemperature1.Init();
  controllerTemperature2.Init();
  controllerTemperatureOutSide.Init();
}


void loop()
{

  controllerTemperature1.Step();
  controllerTemperature2.Step();
  controllerTemperatureOutSide.Step();
  fan1.Step();
  fan2.Step();

  bool warning = (fan1.LowRPMWarning() || fan2.LowRPMWarning());
  warningLed.Signal(warning);

  delay(1000);
}

