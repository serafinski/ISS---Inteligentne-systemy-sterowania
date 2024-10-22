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
#define A_M1 0.38
// B MOTOR 1
#define B_M1 -6.38
// A MOTOR 2
#define A_M2 0.41
// B MOTOR 2
#define B_M2 -8.60

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

// Funkcja do poruszania robota do przodu o określoną liczbę impulsów
void move_forward(int impulses)
{
  motor1.set_direction(FORWARD);
  motor2.set_direction(FORWARD);

  // Prędkość
  float velocity = 50;

  // Ustaw prędkości dla obu motorów
  set_speed_by_velocity(motor1, velocity, A_M1, B_M1);
  set_speed_by_velocity(motor2, velocity, A_M2, B_M2);

  // Czekaj, aż oba motory pokonają odpowiednią liczbę impulsów
  int pulses_m1 = 0;
  int pulses_m2 = 0;

  while (pulses_m1 < impulses || pulses_m2 < impulses)
  {
    pulses_m1 += motor1.getCounterAndReset();
    pulses_m2 += motor2.getCounterAndReset();
    delay(10);
  }
  
  // Zatrzymaj oba motory po osiągnięciu celu
  motor1.stop();
  motor2.stop();
}

// Funkcja do skrętu robota w prawo o określoną liczbę impulsów
void turn_right(int impulses)
{
  // USTAWIC OPDOWIEDNIO BO ZALEZY OD ROBOTA
  motor1.set_direction(REVERSE);
  motor2.set_direction(FORWARD);

  float velocity = 50;

  set_speed_by_velocity(motor1, velocity, A_M1, B_M1);
  set_speed_by_velocity(motor2, velocity, A_M2, B_M2);

  int pulses_m1 = 0;
  int pulses_m2 = 0;

  while (pulses_m1 < impulses || pulses_m2 < impulses)
  {
    pulses_m1 += motor1.getCounterAndReset();
    pulses_m2 += motor2.getCounterAndReset();
    delay(10); 
  }

  motor1.stop();
  motor2.stop();
}

// Setup
void setup() 
{
  Serial.begin(SERIAL_BAUD_RATE);

  motor1.initalize();
  motor2.initalize();

  attachInterrupt(digitalPinToInterrupt(M1_PIN_ENCODER_INTERRUPT), encoder_handler_m1, RISING);
  attachInterrupt(digitalPinToInterrupt(M2_PIN_ENCODER_INTERRUPT), encoder_handler_m2, RISING);
}

// Nieskończona pętla
void loop()
{
  if (Serial.available() > 0)
  {
    String command = Serial.readStringUntil('\n');
    // Pierwszy znak określa akcję
    char action = command.charAt(0);
    // Odczytaj liczbę
    int value = command.substring(2).toInt(); 

    if (action == 'M')
    {
      // Ruch o określoną liczbę impulsów do przodu
      move_forward(value);
    }
    else if (action == 'T')
    {
      // Skręt o określoną liczbę impulsów w prawo
      turn_right(value); 
    }
  }
}