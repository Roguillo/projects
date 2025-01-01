#include <Arduino.h>
#include <TimerOne.h>
#include <FastLED.h>

// I/O pin for RGB LEDs
#define LED_PIN 5

//button pin
#define BUTTON 7

//temperature & humidity sensor pin
#define TH_PIN 11

//7-segment display pins
#define A 22
#define B 10
#define C 8
#define D 12
#define E 4
#define F 21
#define G 9
#define DP 6
#define ST 23
#define ND 20
#define RD 13

const int NUM_LEDS = 14,
BRIGHTNESS = 10;

//RGB LED array
CRGB leds[NUM_LEDS];

//variable for changing LED color
int hue = 0;

/**
 *increments hue and resets once it reaches 255
 */
void gradient() {
  hue++;
  
  if (hue >= 255) {
    hue = 0;
  }
}

//button handler variables
volatile bool button_pressed = false;
volatile unsigned long start = 0;

//temperature unit: true for celsius, false for fahrenheit
bool unit = true;




// "dice" LED numbers in array
/*
| 00 | || | 01 |
| 02 | 03 | 04 |
| 05 | || | 06 |
| || | || | || |
| 07 | || | 08 |
| 09 | 10 | 11 |
| 12 | || | 13 |
*/

/**
 * leds[]          main LED array
 * arr_size        amount of LEDs to turn on
 * indices[]       numbers of LEDs to turn on   
 * 
 * turns on the specifed LEDs to color determined by hue variable, position in LEDs given, and amount of LEDs given
 */
void set_num(CRGB leds[], int arr_size, const int indices[]) {
  for (int i = 0; i < arr_size; i++) {
    int index = indices[i];
    leds[index] = CHSV(hue + (i * (255 / arr_size)), 255, 255);
  }

  FastLED.show();
}


//LED arrangements for numbers 1 - 6 on both "dice"
void one_one() {
  const int indices[] = {3};
  set_num(leds, 1, indices);
}

void one_two() {
  const int indices[] = {0, 6};
  set_num(leds, 2, indices);
}

void one_three() {
  const int indices[] = {1, 3, 5};
  set_num(leds, 3, indices);
}

void one_four() {
  const int indices[] = {0, 1, 5, 6};
  set_num(leds, 4, indices);
}

void one_five() {
  const int indices[] = {0, 1, 3, 5, 6};
  set_num(leds, 5, indices);
}

void one_six() {
  const int indices[] = {0, 1, 2, 4, 5, 6};
  set_num(leds, 6, indices);
}

void two_one() {
  const int indices[] = {10};
  set_num(leds, 1, indices);
}

void two_two() {
  const int indices[] = {7, 13};
  set_num(leds, 2, indices);
}

void two_three() {
  const int indices[] = {8, 10, 12};
  set_num(leds, 3, indices);
}

void two_four() {
  const int indices[] = {7, 8, 12, 13};
  set_num(leds, 4, indices);
}

void two_five() {
  const int indices[] = {7, 8, 10, 12, 13};
  set_num(leds, 5, indices);
}

void two_six() {
  const int indices[] = {7, 8, 9, 11, 12, 13};
  set_num(leds, 6, indices);
}


//function pointer array for all die numbers (1 - 6) on both dice
void (*dado[12])() = {one_one, one_two, one_three, one_four, one_five, one_six, two_one, two_two, two_three, two_four, two_five, two_six};


/**
 * simulates a dice throw at random
 */
void dice() {
  dado[random(0, 6)]();
  dado[random(6, 12)]();

  delay(1000);

  for(int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB::Black;
    FastLED.show();
    delay(18);
  }
}



//segments with corresponding letters
/*
| | A | |
| F | B |
| | G | |
| E | C |
| | D | |
*/

/**
 * clears 7-segment display
 */
void dig_clear() {
  digitalWrite(A, LOW);
  digitalWrite(B, LOW);
  digitalWrite(C, LOW);
  digitalWrite(D, LOW);
  digitalWrite(E, LOW);
  digitalWrite(F, LOW);
  digitalWrite(G, LOW);
  digitalWrite(DP, LOW);
  digitalWrite(ST, LOW);
  digitalWrite(ND, LOW);
  digitalWrite(RD, LOW);
}


/**
 * num          digit to turn on (0, 1, or 2)
 *
 * sets the digit on the 7-segment display to switch on
 */
void dig(int num) {
  switch(num) {
    case(0):
      digitalWrite(ST, HIGH);
      break;

    case(1):
      digitalWrite(ND, HIGH);
      break;

    case(2):
      digitalWrite(RD, HIGH);
      break;
  }
}


//Segment arrangements for numbers 0 - 9, C, F, and decimal point (DP) on the 7-segment display
void zero(int num) {
  digitalWrite(A, HIGH);
  digitalWrite(B, HIGH);
  digitalWrite(C, HIGH);
  digitalWrite(D, HIGH);
  digitalWrite(E, HIGH);
  digitalWrite(F, HIGH);
  dig(num);
}

void one(int num) {
  digitalWrite(B, HIGH);
  digitalWrite(C, HIGH);
  dig(num);
}

void two(int num) {
  digitalWrite(A, HIGH);
  digitalWrite(B, HIGH);
  digitalWrite(G, HIGH);
  digitalWrite(E, HIGH);
  digitalWrite(D, HIGH);
  dig(num);
}

void three(int num) {
  digitalWrite(A, HIGH);
  digitalWrite(B, HIGH);
  digitalWrite(G, HIGH);
  digitalWrite(C, HIGH);
  digitalWrite(D, HIGH);
  dig(num);
}

void four(int num) {
  digitalWrite(F, HIGH);
  digitalWrite(B, HIGH);
  digitalWrite(G, HIGH);
  digitalWrite(C, HIGH);
  dig(num);
}

void five(int num) {
  digitalWrite(A, HIGH);
  digitalWrite(F, HIGH);
  digitalWrite(G, HIGH);
  digitalWrite(C, HIGH);
  digitalWrite(D, HIGH);
  dig(num);
}

void six(int num) {
  digitalWrite(A, HIGH);
  digitalWrite(F, HIGH);
  digitalWrite(G, HIGH);
  digitalWrite(E, HIGH);
  digitalWrite(C, HIGH);
  digitalWrite(D, HIGH);
  dig(num);
}

void seven(int num) {
  digitalWrite(A, HIGH);
  digitalWrite(B, HIGH);
  digitalWrite(C, HIGH);
  dig(num);
}

void eight(int num) {
  digitalWrite(A, HIGH);
  digitalWrite(B, HIGH);
  digitalWrite(C, HIGH);
  digitalWrite(D, HIGH);
  digitalWrite(E, HIGH);
  digitalWrite(F, HIGH);
  digitalWrite(G, HIGH);
  dig(num);
}

void nine(int num) {
  digitalWrite(A, HIGH);
  digitalWrite(F, HIGH);
  digitalWrite(B, HIGH);
  digitalWrite(G, HIGH);
  digitalWrite(C, HIGH);
  digitalWrite(D, HIGH);
  dig(num);
}

void point(int num) {
  digitalWrite(DP, HIGH);
  dig(num);
}

void see(int num) {
  digitalWrite(A, HIGH);
  digitalWrite(F, HIGH);
  digitalWrite(E, HIGH);
  digitalWrite(D, HIGH);
  dig(num);
}

void ef(int num) {
  digitalWrite(A, HIGH);
  digitalWrite(F, HIGH);
  digitalWrite(G, HIGH);
  digitalWrite(E, HIGH);
  dig(num);
}


//function pointer array for all digits (0 - 9) and decimal point
void (*digs[11])(int) = {zero, one, two, three, four, five, six, seven, eight, nine, point};


/**
 * num          number to display (0 - 9)
 * dig          digit to display number on (0 - 2)
 * 
 * displays the specified number at the specifed digit position on the 7-segment display
 */
void sevseg(int num, int dig) {
  digs[num](dig);
  delay(4);
  dig_clear();
}



/**
 * quick animation to play before and after showing random number with rngsus()
 */
void ani() {
  digitalWrite(G, HIGH);
  digitalWrite(ST, HIGH);
  delay(83);

  digitalWrite(ND, HIGH);
  delay(83);

  digitalWrite(RD, HIGH);
  delay(83);

  digitalWrite(ST, LOW);
  delay(83);

  digitalWrite(ND, LOW);
  delay(83);

  digitalWrite(RD, LOW);
  digitalWrite(G, LOW);
  delay(83);
}


/**
 * generates a random integer between 1 and 999, and shows it on the 7-segment display
 */
void rngsus() {
  int num = random(1, 999);
  
  ani();

  for(int i = 0; i < 60; i++) {
    digs[((num % 1000) - (num % 100)) / 100](0);
    delay(6);
    dig_clear();

    digs[((num % 100) - (num % 10)) / 10](1);
    delay(6);
    dig_clear();

    digs[num % 10](2);
    delay(6);
    dig_clear();
  }

  ani();
}



/**
 * when the button is pressed, saves timestamp and changes the button_pressed flag to true
 */
void button_handler() {
  if(!(button_pressed)) {
    start = millis();
    button_pressed = true;
  }
}



/**
 * intializes temp & humidity sensor
 */
void dht_start() {
  pinMode(TH_PIN, OUTPUT);
  digitalWrite(TH_PIN, LOW);

  delay(18);

  digitalWrite(TH_PIN, HIGH);

  delayMicroseconds(30);

  pinMode(TH_PIN, INPUT);
}


/**
 * humidity             as a percentage
 * temperature          in degrees celsius
 *
 * reads temperature and humidity data
 */
int dht_read(uint8_t *humidity, uint8_t *temperature) {
  uint8_t data[5] = {0};

  while (digitalRead(TH_PIN) == HIGH);
  while (digitalRead(TH_PIN) == LOW);
  while (digitalRead(TH_PIN) == HIGH);

  for (int i = 0; i < 40; i++) {
    while (digitalRead(TH_PIN) == LOW);
    unsigned long start_time = micros();
    while (digitalRead(TH_PIN) == HIGH);
    if (micros() - start_time > 50) {
      data[i / 8] |= (1 << (7 - (i % 8)));
    }
  }

  if (data[4] != (data[0] + data[1] + data[2] + data[3])) {
    return -1;
  }

  *humidity = data[0];
  *temperature = data[2];
  return 0;
}




void setup() {
  Serial.begin(9600);

  delay(2000);

  pinMode(LED_PIN, OUTPUT);
  pinMode(TH_PIN, INPUT);

  pinMode(BUTTON, INPUT_PULLUP);

  FastLED.addLeds<WS2812,LED_PIN,RGB>(leds, NUM_LEDS);
  FastLED.setBrightness(BRIGHTNESS);

  attachInterrupt(digitalPinToInterrupt(BUTTON), button_handler, FALLING);

  Timer1.initialize(75 * 1000);
  Timer1.attachInterrupt(gradient);

  pinMode(A, OUTPUT);
  pinMode(B, OUTPUT);
  pinMode(C, OUTPUT);
  pinMode(D, OUTPUT);
  pinMode(E, OUTPUT);
  pinMode(F, OUTPUT);
  pinMode(G, OUTPUT);
  pinMode(DP, OUTPUT);
  pinMode(ST, OUTPUT);
  pinMode(ND, OUTPUT);
  pinMode(RD, OUTPUT);
}


/*
>continuously alternates temperature (in fahrenheit) and humidity data at 1 second intervals on the 7-segment display 
>short button press displays dice throw on RGB LEDs
>long button press displays random integer between 1 - 999 on 7-segment display
>longer button press toggles between celsius and fahrenheit
>wait at least a second between presses
*/
void loop() {
  uint8_t humidity, temperature;

  dht_start();

  if (dht_read(&humidity, &temperature) == 0) {
    int temp;
    unit ? temp = 10*temperature : temp = 10*(1.8*temperature + 32);

    for(int i = 0; i < 60; i++) {
      sevseg(((temp % 1000) - (temp % 100)) / 100, 0);
      sevseg(((temp % 100) - (temp % 10)) / 10, 1);
      sevseg(10, 1);
      sevseg(temp % 10, 2);
    }

    for(int i = 0; i < 60; i++) {
      sevseg(0, 0);
      sevseg(((humidity % 100) - (temp % 10)) / 10, 1);
      sevseg(humidity % 10, 2);
      sevseg(10, 2);
    }
  }

  if(button_pressed) {
    unsigned long duration = millis() - start;

    if(digitalRead(BUTTON) == HIGH) {
      button_pressed = false;

      if(duration < 2000) {
        dice();

      } else if((duration > 2000) && (duration < 4000)) {
        rngsus();

      } else {
        if(unit) {
          for(int i = 0; i < 60; i++) {
            digitalWrite(G, HIGH);
            digitalWrite(ST, HIGH);
            delay(6);
            dig_clear();

            ef(1);
            delay(6);
            dig_clear();

            digitalWrite(G, HIGH);
            digitalWrite(RD, HIGH);
            delay(6);
            dig_clear();
          }

        } else {
            for(int i = 0; i < 60; i++) {
            digitalWrite(G, HIGH);
            digitalWrite(ST, HIGH);
            delay(6);
            dig_clear();

            see(1);
            delay(6);
            dig_clear();

            digitalWrite(G, HIGH);
            digitalWrite(RD, HIGH);
            delay(6);
            dig_clear();
          }
        }

        unit = !unit;
      }
    }
  }
}