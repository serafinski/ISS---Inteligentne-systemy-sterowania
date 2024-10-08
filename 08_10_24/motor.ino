// MOTOR 1
#define PIN_M1_DIRECTION_FORWARD 14
#define PIN_M1_DIRECTION_REVERSE 15
#define PIN_M1_SPEED 5

// MOTOR 2
#define PIN_M2_DIRECTION_FORWARD 16
#define PIN_M2_DIRECTION_REVERSE 17
#define PIN_M2_SPEED 6


#define DIRECTION_SWITCH_DELAY 20

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
        digitalWrite(PIN_M1_DIRECTION_FORWARD, LOW);
        digitalWrite(PIN_M1_DIRECTION_REVERSE, LOW);
      }
      else if(dir == FORWARD)
      {
        digitalWrite(PIN_M1_DIRECTION_REVERSE, LOW);
        delay(DIRECTION_SWITCH_DELAY);
        digitalWrite(PIN_M1_DIRECTION_FORWARD, HIGH); 
      }
      else if(dir == REVERSE)
      {
        digitalWrite(PIN_M1_DIRECTION_FORWARD, LOW);
        delay(DIRECTION_SWITCH_DELAY);
        digitalWrite(PIN_M1_DIRECTION_REVERSE, HIGH);
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

  // Konwencyjnie private po public
  private:
    int pin_direction_forward;
    int pin_direction_reverse;
    int pin_speed;
};

// Inicjalizacja motorow
Motor motor1(PIN_M1_DIRECTION_FORWARD, PIN_M1_DIRECTION_REVERSE, PIN_M1_SPEED);
Motor motor2(PIN_M2_DIRECTION_FORWARD, PIN_M2_DIRECTION_REVERSE, PIN_M2_SPEED);

// Setup
void setup() 
{
  motor1.initalize();
  motor2.initalize();
}

// Nieskonczona petla 
void loop()
{
  motor1.set_direction(FORWARD);
  motor1.set_speed(150);
  delay(2000);
  
  motor1.stop();
  delay(2000);

  motor2.set_direction(FORWARD);
  motor2.set_speed(150);
  delay(2000);
  
  motor2.stop();
  delay(2000); 
}
