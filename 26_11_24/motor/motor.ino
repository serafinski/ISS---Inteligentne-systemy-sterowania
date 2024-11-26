// Include dla TRSensor i OneWire
#include <OneWire.h>
#include <TRSensors.h>

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

#define COUNTS_PER_ROTATION 500
#define NUM_SENSORS 5

// SERIAL BY MOZNA BYLO COKOLWIEK WYPISAC
#define SERIAL_BAUD_RATE 9600

// A MOTOR 1
#define A_M1 = 0.33

// A MOTOR 2
#define A_M2 = 0.40

// zmienne do PID
#define P = 0.4
#define I = 0.05
#define D = 1.45

TRSensors trs = TRSensors();
unsigned int sensorValues[NUM_SENSORS];

enum Direction {
  FORWARD,
  REVERSE,
  DISABLE
};

class Motor {
  // Konwencyjnie public przed private
  public:
    // Konstruktor
    Motor(int id_forward_pin, int id_reverse_pin, int id_speed_pin)
    {
      pin_direction_forward = id_forward_pin;
      pin_direction_reverse = id_reverse_pin;
      pin_speed = id_speed_pin;
    }
    
    //Inicjalizacja  
    void initialize() {
      pinMode(pin_direction_forward, OUTPUT);
      digitalWrite(pin_direction_forward, LOW);
      pinMode(pin_direction_reverse, OUTPUT);
      digitalWrite(pin_direction_reverse, LOW);
      pinMode(pin_speed, OUTPUT);
      analogWrite(pin_speed, 0);
    }
    
    // ENUM - FORWARD, REVERSE, DISABLED
    void set_direction(Direction dir) {
      if (dir == FORWARD)
      {
        digitalWrite(pin_direction_reverse, LOW);
        delay(DIRECTION_SWITCH_DELAY);
        digitalWrite(pin_direction_forward, HIGH);
      }
      else if (dir == REVERSE) 
      {
        digitalWrite(pin_direction_forward, LOW);
        delay(DIRECTION_SWITCH_DELAY);
        digitalWrite(pin_direction_reverse, HIGH);
      } 
      else 
      {
        digitalWrite(pin_direction_forward, LOW);
        digitalWrite(pin_direction_reverse, LOW);
      }
    }
    // INT 0-255
    void set_speed(int speed) {
      analogWrite(pin_speed, constrain(speed, 0, 255));
    }
    
    void stop() {
      set_direction(DISABLE);
      set_speed(0);
    }

  private:
    int pin_direction_forward, 
    int pin_direction_reverse, 
    int pin_speed;
};

// Tworzymy obiekty dla obu silników
Motor motor1(PIN_M1_DIRECTION_FORWARD, PIN_M1_DIRECTION_REVERSE, PIN_M1_SPEED);
Motor motor2(PIN_M2_DIRECTION_FORWARD, PIN_M2_DIRECTION_REVERSE, PIN_M2_SPEED);

volatile int left_counter = 0;
volatile int right_counter = 0;

void encoder_handler_m1() {
  left_counter++;
}

void encoder_handler_m2() {
  right_counter++;
}

// Funkcja do ruchu na określoną liczbę obrotów
void move_forward() {

  int left_speed = A_M1 * 255;   
  int right_speed = A_M2 * 255;
  
  motor1.set_direction(FORWARD);
  motor2.set_direction(FORWARD);
  motor1.set_speed(left_speed);
  motor2.set_speed(right_speed);
}

float previous_error = 0;
float integral = 0;
unsigned int previous_position = 0;


void setup() {
  Serial.begin(SERIAL_BAUD_RATE);

  for (int i = 0; i < 400; i++)  // make the calibration take about 10 seconds
  {
    trs.calibrate();
  }

  motor1.initialize();
  motor2.initialize();
  
  attachInterrupt(digitalPinToInterrupt(M1_PIN_ENCODER_INTERRUPT), encoder_handler_m1, RISING);
  attachInterrupt(digitalPinToInterrupt(M2_PIN_ENCODER_INTERRUPT), encoder_handler_m2, RISING);
}

void loop() {
  move_forward();
  
  unsigned int position = trs.readLine(sensorValues);

  if (position > 10000) {
    // Obracanie się wokół własnej osi w lewo lub w prawo w zależności od poprzedniej pozycji linii
    if (previous_position > 2500) {
      // Obracanie w lewo
      while (position > 5000) {
        motor1.set_direction(REVERSE);
        motor2.set_direction(FORWARD);
        motor1.set_speed(100);
        motor2.set_speed(100);
        position = trs.readLine(sensorValues);
      }
    } else {
      // Obracanie w prawo
      while (position > 5000) {
        motor1.set_direction(FORWARD);
        motor2.set_direction(REVERSE);
        motor1.set_speed(100);
        motor2.set_speed(100);
        position = trs.readLine(sensorValues);
      }
    }
  } else {
    // błąd
    int setpoint = 2500;
    int error = setpoint - position;
    
    // całka błędu
    integral += error;
    
    // pochodna błędu
    int derivative = error - previous_error;
    
    // PID
    float correction = P * error + I * integral + D * derivative;
    
    int base_speed = 100;
    int left_speed = base_speed + correction / 20;
    int right_speed = base_speed - correction / 20;
    
    motor1.set_speed(left_speed);
    motor2.set_speed(right_speed);
    
    previous_error = error;
  }
  
  previous_position = position;
}