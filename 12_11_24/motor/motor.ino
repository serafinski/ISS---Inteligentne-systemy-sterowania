// Include dla TRSensor
#include "TRSensors.h"

// Liczba sensorów
#define NUM_SENSORS 5

// MOTOR 1

// DOSTOSOWAC PORTY W ZALEZNOSCI OD ROBOTA
#define PIN_M1_DIRECTION_FORWARD 15
#define PIN_M1_DIRECTION_REVERSE 14
#define PIN_M1_SPEED 5

// MOTOR 2

// DOSTOSOWAC PORTY W ZALEZNOSCI OD ROBOTA
#define PIN_M2_DIRECTION_FORWARD 16
#define PIN_M2_DIRECTION_REVERSE 17
#define PIN_M2_SPEED 6

// PINY INTERUPT
#define M1_PIN_ENCODER_INTERRUPT 2
#define M2_PIN_ENCODER_INTERRUPT 3

#define DIRECTION_SWITCH_DELAY 20

// CO ILE MA MIERZYC
#define MEASUREMENT_TIME_MS 1000

// SERIAL BY MOZNA BYLO COKOLWIEK WYPISAC
#define SERIAL_BAUD_RATE 9600

// WYLICZONE Z REGRESJI
// Motor 1: y = 0.38 * PWM + -6.38
// Motor 2: y = 0.41 * PWM + -8.60

// A MOTOR 1
#define A_M1 0.44
// B MOTOR 1
#define B_M1 -5.09
// A MOTOR 2
#define A_M2 0.40
// B MOTOR 2
#define B_M2 -3.85

// Sensory od 0 do 5 są połączone do analog'owych inputów 0 do 5
TRSensors trs = TRSensors();
unsigned int sensorValues[NUM_SENSORS];

// ENUM KIERUNEK MOTORU
enum Direction {
  FORWARD,
  REVERSE,
  DISABLED
};

class Motor
{
  // Konwencyjnie public przed private
  public:
    // Konstruktor
    Motor(int id_forward_pin, int id_reverse_pin, int id_speed_pin)
    {
      pin_direction_forward = id_forward_pin;
      pin_direction_reverse = id_reverse_pin;
      pin_speed = id_speed_pin;
      // inicjalizacja countera z 0
      counter = 0;
    }
    
    //Inicjalizacja
    void initalize()
    {
      pinMode(pin_direction_forward, OUTPUT);
      digitalWrite(pin_direction_forward, LOW);
      pinMode(pin_direction_reverse, OUTPUT);
      digitalWrite(pin_direction_reverse, LOW);
      pinMode(pin_speed, OUTPUT);
      analogWrite(pin_speed, 0);
    }

    // ENUM - FORWARD, REVERSE, DISABLED
    void set_direction(Direction dir)
    {
      if(dir == DISABLED)
      {
        digitalWrite(pin_direction_forward, LOW);
        digitalWrite(pin_direction_reverse, LOW);
      }
      else if(dir == FORWARD)
      {
        digitalWrite(pin_direction_reverse, LOW);
        delay(DIRECTION_SWITCH_DELAY);
        digitalWrite(pin_direction_forward, HIGH); 
      }
      else if(dir == REVERSE)
      {
        digitalWrite(pin_direction_forward, LOW);
        delay(DIRECTION_SWITCH_DELAY);
        digitalWrite(pin_direction_reverse, HIGH);
      }
    }
    // INT 0-255
    void set_speed(int speed)
    {
      analogWrite(pin_speed, speed);
    }

    // STOP - W przeciwienstwie do disabled nie zachowuje ostatniej predkosci
    void stop()
    {
      set_direction(DISABLED);
      set_speed(0);
    }

    void incrementCounter()
    {
      counter++;
    }

    //Getter
    int getCounterAndReset()
    {
      int tmp = counter;
      counter = 0;
      return tmp;
    }

  // Konwencyjnie private po public
  private:
    int pin_direction_forward;
    int pin_direction_reverse;
    int pin_speed;
    int counter;
};

// Inicjalizacja motorow
Motor motor1(PIN_M1_DIRECTION_FORWARD, PIN_M1_DIRECTION_REVERSE, PIN_M1_SPEED);
Motor motor2(PIN_M2_DIRECTION_FORWARD, PIN_M2_DIRECTION_REVERSE, PIN_M2_SPEED);

void encoder_handler_m1()
{
  motor1.incrementCounter();
}

void encoder_handler_m2()
{
  motor2.incrementCounter();
}

// Uniwersalna funkcja obliczająca PWM na podstawie prędkości
void set_speed_by_velocity(Motor &motor, float velocity, float a, float b)
{
    int pwm = (velocity - b) / a;

    if (pwm < 0) pwm = 0;
    if (pwm > 255) pwm = 255;

    motor.set_speed(pwm);
}

// Setup
void setup() 
{
  Serial.begin(SERIAL_BAUD_RATE);

  motor1.initalize();
  motor2.initalize();

  attachInterrupt(digitalPinToInterrupt(M1_PIN_ENCODER_INTERRUPT), encoder_handler_m1, RISING);
  attachInterrupt(digitalPinToInterrupt(M2_PIN_ENCODER_INTERRUPT), encoder_handler_m2, RISING);


  Serial.println("Kalibrowowanie sensorow...");
  for (int i = 0; i < 400; i++)  // kalibracja trwa około 10 sekund
  {
    trs.calibrate();  // odczytuje wszystkie sensory 10 razy
    delay(10);
  }
  Serial.println("Kalbracja skonczona.");

  // // Print the calibration minimum values measured when emitters were on
  // for (int i = 0; i < NUM_SENSORS; i++)
  // {
  //   Serial.print(trs.calibratedMin[i]);
  //   Serial.print(' ');
  // }
  // Serial.println();

  // // Print the calibration maximum values measured when emitters were on
  // for (int i = 0; i < NUM_SENSORS; i++)
  // {
  //   Serial.print(trs.calibratedMax[i]);
  //   Serial.print(' ');
  // }
  // Serial.println();
  // delay(1000);
}

// Nieskończona pętla
void loop()
{
  unsigned int position = trs.readLine(sensorValues);

  // // Drukowanie wartości sensorów i pozycji linii
  // for (unsigned char i = 0; i < NUM_SENSORS; i++)
  // {
  //   Serial.print(sensorValues[i]);
  //   Serial.print('\t');
  // }
  // Serial.println(position);

  // Logika podążania za linią
  int base_speed = 5; // Bazowa prędkość
  int max_speed_diff = 1; // Maksymalna różnica prędkości do skrętu
  int error = position - 2500; // Centrum jest na 2500
  int speed_diff = error * 0.01;

  Serial.print("Speed Difference: ");
  Serial.println(speed_diff);

  int left_speed = base_speed + speed_diff;
  int right_speed = base_speed - speed_diff;

  left_speed = constrain(left_speed, 0, 255);
  right_speed = constrain(right_speed, 0, 255);

  // Ustawienie kierunków i prędkości motorów na podstawie wyliczonych wartości
  motor1.set_direction(FORWARD);
  motor2.set_direction(FORWARD);

  set_speed_by_velocity(motor1, left_speed, A_M1, B_M1);
  set_speed_by_velocity(motor2, right_speed, A_M2, B_M2);

  delay(10);
}