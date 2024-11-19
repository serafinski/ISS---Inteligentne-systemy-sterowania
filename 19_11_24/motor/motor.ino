// Include dla TRSensor
#include "TRSensors.h"

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

// A MOTOR 1
#define A_M1 0.38
// A MOTOR 2
#define A_M2 0.41

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

// PID Controller Class
class PIDController 
{
  public:
    PIDController(double kp, double ki, double kd) {
      Kp = kp;
      Ki = ki;
      Kd = kd;
      integral = 0.0;
      previousError = 0.0;
    }

    double calculate(double target, double current) {
      double error = target - current;
      integral += error;
      double derivative = error - previousError;
      double output = Kp * error + Ki * integral + Kd * derivative;
      previousError = error;
      return output;
    }

    void reset() {
      integral = 0.0;
      previousError = 0.0;
    }

  private:
    double Kp, Ki, Kd;
    double integral;
    double previousError;
};

// Inicjalizacja motorow
Motor motor1(PIN_M1_DIRECTION_FORWARD, PIN_M1_DIRECTION_REVERSE, PIN_M1_SPEED);
Motor motor2(PIN_M2_DIRECTION_FORWARD, PIN_M2_DIRECTION_REVERSE, PIN_M2_SPEED);

PIDController pid1(A_M1, 0.001, 0.1);
PIDController pid2(A_M2, 0.001, 0.1);

// Kontroler PID dla podążania za linią
PIDController linePID(0.4, 0.0, 0.1); // Ustaw odpowiednie wartości Kp, Ki, Kd

// Inicjalizacja czujnika liniowego
TRSensors trs = TRSensors();

void encoder_handler_m1()
{
  motor1.incrementCounter();
}

void encoder_handler_m2()
{
  motor2.incrementCounter();
}

// Funkcja do poruszania robota do przodu o określoną liczbę impulsów
void move_forward()
{
  motor1.set_direction(FORWARD);
  motor2.set_direction(FORWARD);

  // Prędkość
  float velocity = 80;

  while (true)
  {
    // Odczyt linii
    unsigned int sensorValues[5];
    unsigned int position = trs.readLine(sensorValues);

    // Oblicz błąd (pozycja linii względem środka)
    int error = position - 2000;

    // Oblicz korektę z PID
    double correction = linePID.calculate(0, error);

    // Odczyt aktualnych prędkości silników
    int currentSpeedM1 = motor1.getCounterAndReset();
    int currentSpeedM2 = motor2.getCounterAndReset();

    // Oblicz PWM dla silników z korekcją
    double pwmM1 = pid1.calculate(velocity, currentSpeedM1);
    double pwmM2 = pid2.calculate(velocity, currentSpeedM2);

    // Ograniczenie wartości PWM do zakresu 0-255
    pwmM1 = constrain(pwmM1, 0, 255);
    pwmM2 = constrain(pwmM2, 0, 255);

    // Ustawienie PWM na silnikach
    motor1.set_speed(static_cast<int>(pwmM1));
    motor2.set_speed(static_cast<int>(pwmM2));

    delay(10);
  }
  
  // Zatrzymaj oba motory po osiągnięciu celu
  motor1.stop();
  motor2.stop();
  
  pid1.reset();
  pid2.reset();
  linePID.reset();
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
}

// Nieskończona pętla
void loop()
{
    move_forward();
}