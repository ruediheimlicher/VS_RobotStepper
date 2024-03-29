#include <Arduino.h>
/*********
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp-now-one-to-many-esp32-esp8266/

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*********/

// MAC: 48:3f:da:a4:36:57 MINI D1

#include <ESP8266WiFi.h>
#include <espnow.h>
//#include <Servo.h>

//#include <Ticker.h>
#include "elapsedMillis.h"
#include "expo.h"





#define batt A0

#define NUM_STEPPERS 2

#define MAX_TICKS 3400
#define MIN_TICKS 1700

#define MAX_DELAY 4000
#define MIN_DELAY 400
uint16_t maxdelay = 0;
uint16_t mindelay = 0;

#define MAX_STEPS 2000 // Anlaufen des Steppers
#define MIN_STEPS 1000 // max speed Stepper
#define STEPSREGION 500

#define IMPULSBREITE 20

uint16_t steppermittearray[NUM_STEPPERS] = {}; // Werte fuer Mitte
uint16_t mitte0 = 0;
uint16_t mitte0_raw = 0;
uint16_t mitte1 = 0;
uint16_t mitte1_raw = 0;

uint16_t delta_0 = 0;
uint16_t delta_1 = 0;

volatile uint16_t stepsarray[NUM_STEPPERS] = {1500, 1500}; // steps
volatile uint8_t dir_status = 0;                           // Richtung bit0: stepper0

volatile uint16_t stepperimpulsarray[NUM_STEPPERS] = {1500, 1500}; //

uint16_t err1[16] = {};


#define MITTE 1500

#define MITTE 1500

/*
// bread
#define STEPPER0_DIR 15
#define STEPPER0_STEP 13
#define STEPPER0_EN 9

#define STEPPER1_DIR 12
#define STEPPER1_STEP 14
#define STEPPER1_EN 10

*/
// Print
// A
#define STEPPER0_DIR 9
#define STEPPER0_STEP 15
#define STEPPER0_EN 10

// B
#define STEPPER1_DIR 12
#define STEPPER1_STEP 14
#define STEPPER1_EN 13





#define STEPPER0_DIR_BIT 0
#define STEPPER0_EN_BIT 1


#define STEPPER1_DIR_BIT 2
#define STEPPER1_EN_BIT 3

#define NULLBAND 100




#define AVERAGE 4
uint16_t delay0 = 0; // Impulsdelay stepper 0
uint16_t delay1 = 0;
uint16_t delay0_raw = 0;
uint16_t delay0_raw_array[AVERAGE] = {};
uint8_t average0pos = 0;

uint16_t delay1_raw = 0;

uint16_t delay0_map = 0;
uint16_t delay1_map = 0;

int16_t ausschlag0 = 0;
int16_t ausschlag0_inv = 0;
int16_t ausschlag0_map = 0;

int16_t ausschlag1 = 0;
int16_t ausschlag1_map = 0;

uint16_t steps0 = 0;
uint16_t steps1 = 0;

uint8_t nullband = 0;

float stepfaktor = 1; // Ticks to Steps ratio
uint16_t stepperwertbereich = 0;

uint8_t firstruncounter = 8;

uint8_t loopstatus = 0;

#define TON_ON          0
#define TON_OFF          1

#define FIRSTRUN_BIT    7

uint16_t maxwinkel = 180;

uint8_t buttonstatus = 0;
uint8_t tonindex = 0;

uint8_t starttonindex = 0;
void playTon(int ton);
#define START_TON 3
#define LICHT_ON 2

#define TON_PIN 0

uint16_t tonperiode = 0; // 10 khz
int8_t ton_on = 0;
uint16_t signalton = 22;

elapsedMillis tonposition;

uint16_t ubatt = 0;

uint8_t broadcastAddress[] = {0x48, 0x3F, 0xDA, 0xA4, 0x36, 0x57};

int deg0 = 0;
int deg1 = 0;

long unsigned int ledintervall = 1000;
elapsedMillis ledmillis;
elapsedMillis tonmillis;
elapsedMillis pausemillis;
elapsedMicros tonmicros;
int tonfolge[3] = {554, 329, 440};
int periodenfolge[3] = {75, 127, 95};
int startmelodieD[4] = {107, 85, 70,53};
int startperiodenfolge[4] = {107, 85, 70,53};



volatile uint8_t stepperindex = 0;
volatile uint16_t stepperimpulsdauer = 0;
volatile uint16_t stepperzeitsumme = 0; // cumulate stepperimpulses

#define PAKETINTERVALL 20000
#define SERVO_PAUSE_BIT 7
#define TIMERFAKTOR 5                              // Umrechnung Impulszeit
volatile uint16_t paketintervall = PAKETINTERVALL; // 20 ms
volatile int interrupts;


bool recordSteps = false;
bool playRecordedSteps = false;

unsigned long previousTimeInMilli = millis();

uint16_t MAP(uint16_t au32_IN, uint16_t au32_INmin, uint16_t au32_INmax, uint16_t au32_OUTmin, uint16_t au32_OUTmax)
{
  return ((((au32_IN - au32_INmin) * (au32_OUTmax - au32_OUTmin)) / (au32_INmax - au32_INmin)) + au32_OUTmin);
}
void playTon(uint8_t ton)
{
  // Cis: 554
  // e: 330
  // a: 440
  tone(TON_PIN, tonfolge[ton], 800);
  // tone(TON_PIN,440,800);
}

void playStartmelodie(uint8_t ton)
{
 
  tone(TON_PIN, startmelodieD[ton], 800);
}

// Structure example to receive data
// Must match the sender structure
typedef struct canal_struct
{
  uint16_t lx;
  uint16_t ly;
  uint16_t rx;
  uint16_t ry;

  uint8_t digi;

  uint16_t x;
  uint16_t y;
} canal_struct;

// Create a struct_message called canaldata
canal_struct canaldata;

canal_struct outdata;

#define ESP_NOW_SEND_SUCCESS 0

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, uint8_t status)
{
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// callback function that will be executed when data is received
void OnDataRecv(uint8_t *mac, uint8_t *incomingData, uint8_t len)
{
  // blink
  memcpy(&canaldata, incomingData, sizeof(canaldata));

  // Serial.print("Bytes received: ");
  // Serial.printf("%d %d %d %d %d %d \n",mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);

  // Serial.printf("lx: %d lx. %d\n", canaldata.lx, canaldata.ly);
  // Serial.print(canaldata.lx);
  // Serial.print(" ");
  // Serial.print("ly: ");
  // Serial.print(canaldata.ly);
  // Serial.print(" int ");
  // Serial.print(interrupts);
  // Serial.print(" ");
  // Serial.print("digi: ");
  // Serial.println(canaldata.digi);

  // lx

  //
  ausschlag0 = 0;

  average0pos++;
  average0pos &= 0x03;
  delay0_raw_array[average0pos] = (canaldata.rx);
  
  delay0_raw = (canaldata.rx); // sum0/AVERAGE;

  delay0_map = map(delay0_raw, MIN_STEPS, MAX_STEPS, MIN_DELAY, MAX_DELAY);
  ausschlag0 = abs(delay0_map - mitte0); // nur Betrag

  if (abs(ausschlag0) < NULLBAND)
  {
    dir_status |= (1 << STEPPER0_EN_BIT);
    digitalWrite(STEPPER0_EN, HIGH);
    delay0 = 0;
  }
  else
  {

    dir_status &= ~(1 << STEPPER0_EN_BIT);
  }
   if (ausschlag0 > MAX_DELAY - mitte0)
  {
    ausschlag0 = MAX_DELAY - mitte0;
  }

  ausschlag0_map = map(ausschlag0, 0, MAX_DELAY - mitte0, MAX_DELAY, MIN_DELAY);

  steps0 = ausschlag0_map;
  
  if (delay0_map < mitte0) // ccw
  {
    dir_status &= ~(1 << STEPPER0_DIR_BIT);
  }
  else
  {
    dir_status |= (1 << STEPPER0_DIR_BIT);
  }

  // steps0 = ausschlag0_map;

  if (steps0 < MIN_DELAY)
  {
    steps0 = MIN_DELAY;
  }
  // Stepper 1
  ausschlag1 = 0;
  delay1_raw = canaldata.ry;
  if (delay1_raw < MIN_STEPS)
  {
    delay1_raw = MIN_STEPS;
  }
  
  delay1_map = map(delay1_raw, MIN_STEPS, MAX_STEPS, MIN_DELAY, MAX_DELAY);
  // delay1_map = map(delay1_raw, mitte1_raw - STEPSREGION, mitte1_raw + STEPSREGION, MIN_DELAY, MAX_DELAY);
  
  ausschlag1 = abs(delay1_map - mitte1); // nur Betrag
  if (ausschlag1 > MAX_DELAY - mitte1)
  {
    ausschlag1 = MAX_DELAY - mitte1;
  }
  ausschlag1_map = map(ausschlag1, 0, MAX_DELAY - mitte1, MAX_DELAY, MIN_DELAY);

  if (abs(ausschlag1) < NULLBAND)
  {
    dir_status |= (1 << STEPPER1_EN_BIT);
    digitalWrite(STEPPER1_EN, HIGH);
    delay1 = 0;
  }
  else
  {
    dir_status &= ~(1 << STEPPER1_EN_BIT);
  }
  
  if (delay1_map < mitte1) // ccw
  {
    dir_status &= ~(1 << STEPPER1_DIR_BIT);
  }
  else
  {
    dir_status |= (1 << STEPPER1_DIR_BIT);
  }

  steps1 = ausschlag1_map;
  if(steps1 > 0x4000)
  {
    err1[0] = delay1_raw;
    err1[1] = delay1_map;
    err1[2] = ausschlag1;
    err1[3] = ausschlag1_map;
  }
  if (steps1 < MIN_DELAY)
  {
    steps1 = MIN_DELAY;
  }
  // Mitte bestimmen
  if ((dir_status & (1 << FIRSTRUN_BIT)) && (firstruncounter--))
  {
    mitte0_raw += delay0_raw;
    steppermittearray[0] += delay0_raw;
    mitte0 += delay0_map;

   // Serial.printf("delay0_raw: %d delay0_map: %d mitte0: %d\n", delay0_raw, delay0_map, mitte0);

    mitte1_raw += delay1_raw;
    steppermittearray[1] += delay1_raw;
    mitte1 += delay1_map;
    if (firstruncounter == 0)
    {
      steppermittearray[0] /= 8;
      Serial.printf("mitte0 summe: %d \t", mitte0);
      mitte0 /= 8;
      mitte0_raw /= 8;
      Serial.printf("mitte0 mittel: %d \n", mitte0);

      steppermittearray[1] /= 8;
      mitte1 /= 8;
      mitte1_raw /= 8;
      Serial.printf("steppermittel 0: %d mitte0: %d \t steppermittel 1: %d\n", steppermittearray[0], mitte0, steppermittearray[1]);
      dir_status &= ~(1 << FIRSTRUN_BIT);

      delta_0 = min((mitte0 - MIN_DELAY),(MAX_DELAY - mitte0));
      delta_1 = min((mitte1 - MIN_DELAY),(MAX_DELAY - mitte1));

      
    
    }
  } // if FIRSTRUN

  // Serial.printf("delay0_raw: %d steps0: %d \t delay1: %d steps1: %d\n", delay0_raw, steps0, delay1_raw, steps1);

  if (canaldata.digi & (1 << START_TON))
  {
    if (!(buttonstatus & (1 << START_TON)))
    {
      // Serial.println("digi start");
      // Serial.println(canaldata.digi);
      buttonstatus |= (1 << START_TON);
      tonindex = 0;
    }
  }
}

void IRAM_ATTR stepperISR()
{
  if (delay0)
  {
    if (delay0 == IMPULSBREITE) // Impuls starten
    {
      digitalWrite(STEPPER0_STEP, HIGH);
    }
    delay0--;
  }
  if (delay1)
  {
    if (delay1 == IMPULSBREITE) // Impuls starten
    {
      digitalWrite(STEPPER1_STEP, HIGH);
    }
    delay1--;
  }

  
  if(tonperiode)
    {
      tonperiode--;
    }
    if(tonperiode == 0)
    {
      ton_on ^= (1<<0);
      tonperiode = signalton;
      //if (buttonstatus & (1 << START_TON))
      if (loopstatus & (1<<TON_ON))
      {
        digitalWrite(TON_PIN, ton_on);
      }
    }
}

void setup()
{
  // Initialize Serial Monitor
  Serial.begin(115200);
  Serial.println();
  Serial.print("ESP Board MAC Address:  ");
  Serial.println(WiFi.macAddress());

  pinMode(TON_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(A0, INPUT);
  // https://community.platformio.org/t/esp8266-gibberish-serial-monitor-output/30027
  stepsarray[0] = 400;

  pinMode(STEPPER0_DIR, OUTPUT);
  digitalWrite(STEPPER0_DIR, LOW);
  pinMode(STEPPER0_STEP, OUTPUT);
  digitalWrite(STEPPER0_STEP, LOW);
  pinMode(STEPPER0_EN, OUTPUT);
  digitalWrite(STEPPER0_EN, HIGH); // OFF

  pinMode(STEPPER1_DIR, OUTPUT);
  digitalWrite(STEPPER1_DIR, LOW);
  pinMode(STEPPER1_STEP, OUTPUT);
  digitalWrite(STEPPER1_STEP, LOW);
  pinMode(STEPPER1_EN, OUTPUT);
  digitalWrite(STEPPER1_EN, HIGH); // OFF

  // timer1_isr_init();

  timer1_enable(TIM_DIV1, TIM_EDGE, TIM_LOOP);
  timer1_write(1000);
  timer1_attachInterrupt(stepperISR);

  stepfaktor = (MAX_STEPS - MIN_STEPS) / (MAX_TICKS - MIN_TICKS);

  dir_status |= (1 << FIRSTRUN_BIT);

  // Set device as a Wi-Fi Station;
  WiFi.mode(WIFI_STA);

  WiFi.disconnect();

  // Init ESP-NOW
  if (esp_now_init() != 0)
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);
  esp_now_register_recv_cb(OnDataRecv);
  mitte0 = 0;
  err1[0] = 0;
  err1[1] = 0;
//playStartmelodie(0);
loopstatus |= (1<<FIRSTRUN_BIT);
 tonmillis = 0;
//tone(TON_PIN, 440, 800);
}

void loop()
{

// Startmelodie
  
  if(loopstatus & (1<<FIRSTRUN_BIT))
  {
   
    //Serial.printf("firstrun start\n");
    if (tonindex == 0) // vorgang beginnt
    {
      //Serial.printf("************ FIRSTRUN_BIT start index: %d signalton: %d\n", tonindex, signalton);

      tonmillis = 0;
      //Serial.print("************ TON start index: ");
      //Serial.print(tonindex);
      signalton = startperiodenfolge[tonindex];
      tonperiode = signalton;
      loopstatus |= (1<<TON_ON);
      //Serial.printf("************ TON start index: %d signalton: %d\n", tonindex, signalton);
      tonindex++;
    }
    if(tonindex) // vorgang im Gang
    {
      if (tonmillis > 300) // lang genug
      {
          
        loopstatus &= ~(1<<TON_ON); // ton off
        pausemillis = 0; // Beginn pause
          //Serial.print("************ TON next index: ");
          //Serial.println(tonindex);
        if(tonmillis > 350) // next ton
        {
          
          if (tonindex == 4) // end
          {
            buttonstatus &= ~(1<<START_TON);
            loopstatus &= ~(1<<TON_ON);
            loopstatus &= ~(1<<FIRSTRUN_BIT);
            tonindex = 0;
          }
          else
          {
            signalton = startperiodenfolge[tonindex];
            loopstatus |= (1<<TON_ON); // Ton ON
            //Serial.printf("************ TON next index: %d signalton: %d\n", tonindex, signalton);
            tonmillis = 0; 
            tonindex++;
          }
        
        } // tonmillis > 850
      } // tonmillis > 800

      


    } // if tonindex
    //Serial.printf("firstrun end\n");
  } // end FIRSTRUN
  
  // end Startmelodie
  else
  {
  if (delay0 == 0)
  {
    {
      digitalWrite(STEPPER0_DIR, (dir_status & (1 << STEPPER0_DIR_BIT)));
      digitalWrite(STEPPER0_EN, (dir_status & (1 << STEPPER0_EN_BIT)));
      digitalWrite(STEPPER0_STEP, LOW);
      delay0 = steps0; // stepsarray[0];
    }
  }
  if (delay1 == 0)
  {
    {
      digitalWrite(STEPPER1_DIR, (dir_status & (1 << STEPPER1_DIR_BIT)));
      digitalWrite(STEPPER1_EN, (dir_status & (1 << STEPPER1_EN_BIT)));
      digitalWrite(STEPPER1_STEP, LOW);
      delay1 = steps1; // stepsarray[1];
    }
  }
}
  if (tonmicros > 4)
  {
    // digitalWrite(4,!digitalRead(4));
    tonmicros = 0;
  }

  
  if (ledmillis >= ledintervall)
  {
    // int  faktor0 = int(6400 +6000 * (sin(deg0*PI/180)));
    // stepsarray[0] = faktor0;
    //deg0 += 16;

    // int  faktor1 = int(6400 +6000 * (sin(deg1*PI/180)));
    // stepsarray[1] = faktor1;
    //deg1 += 18;

    //Serial.printf("m0: %d del0_raw: %d   steps0: %d   del0_map: %d   a0: %d  a0_map: %d\t m1: %d del1_raw: %d steps1: %d del1_map: %d a1: %d a1_map: %d dir_status: %2X\t", mitte0, delay0_raw, steps0, delay0_map, ausschlag0, ausschlag0_map, mitte1, delay1_raw, steps1, delay1_map, ausschlag1, ausschlag1_map, dir_status);
    //Serial.printf("\n");

    if(err1[0]> 0)
    {
       Serial.printf("err1: %d %d %d %d\n",err1[0], err1[1],err1[2], err1[3]);
    }
    ledmillis = 0;
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    ubatt = analogRead(A0);
    outdata.x = ubatt;
    //
    
    //

    //  Serial.println("led");
  }

  if (buttonstatus & (1 << START_TON))
  {
    if (tonindex == 0) // vorgang beginnt
    {
      tonmillis = 0;
      Serial.print("************ TON start index: ");
      //Serial.print(tonindex);
      signalton = periodenfolge[tonindex];
      loopstatus |= (1<<TON_ON);
      //Serial.printf("************ TON start index: %d signalton: %d\n", tonindex, signalton);
      tonindex++;
    }
    if(tonindex) // vorgang im Gang
    {
      if (tonmillis > 800) // lang genug
      {
          
        loopstatus &= ~(1<<TON_ON); // ton off
        pausemillis = 0; // Beginn pause
          //Serial.print("************ TON next index: ");
          //Serial.println(tonindex);
        if(tonmillis > 850) // next ton
        {
          
          if (tonindex == 3) // end
          {
            buttonstatus &= ~(1<<START_TON);
            loopstatus &= ~(1<<TON_ON);
            tonindex = 0;
          }
          else
          {
            signalton = periodenfolge[tonindex];
            loopstatus |= (1<<TON_ON); // Ton ON
            //Serial.printf("************ TON next index: %d signalton: %d\n", tonindex, signalton);
            tonmillis = 0; 
            tonindex++;
          }
        
        } // tonmillis > 850
      } // tonmillis > 800

      


    } // if tonindex
    /*
    if (tonindex < 3)
    {
      if (tonmillis > 850)
      {
        //Serial.print("************ TON next index: ");
        //Serial.println(tonindex);
        signalton = periodenfolge[tonindex];
        Serial.printf("************ TON next index: %d signalton: %d\n", tonindex, signalton);
        //playTon(tonindex);
        tonindex++;
        tonmillis = 0; 
      }
    }
    else if (tonindex >= 4)
    {
      Serial.println("************ TON END");
      buttonstatus &= ~(1 << START_TON);
      tonindex = 0;
    }
    */
  }
}
