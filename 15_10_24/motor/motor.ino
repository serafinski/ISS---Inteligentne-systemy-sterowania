// MOTOR 1
#define PIN_M1_DIRECTION_FORWARD 14
#define PIN_M1_DIRECTION_REVERSE 15
#define PIN_M1_SPEED 5

// MOTOR 2
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

int counter;

void encoder_handler_m1()
{
  motor1.incrementCounter();
}

void encoder_handler_m2()
{
  motor2.incrementCounter();
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

// Nieskonczona petla 
void loop()
{
  motor1.set_direction(FORWARD);
  motor2.set_direction(FORWARD);


  Serial.println("Speed;Motor_1;Motor_2");
  for (int speed = 0; speed <=255; speed += 10)
  {
    motor1.set_speed(speed);
    motor2.set_speed(speed);

    delay(MEASUREMENT_TIME_MS);
    
    int motor1_pulses = motor1.getCounterAndReset();
    int motor2_pulses = motor2.getCounterAndReset();

    Serial.print(speed);
    Serial.print(";");
    Serial.print(motor1_pulses);
    Serial.print(";");
    Serial.println(motor2_pulses);
  }
}
