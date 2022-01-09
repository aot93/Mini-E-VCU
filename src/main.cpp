/*
  Copyright (c) 2021 Historic Electric Ltd
  Permission is hereby granted, free of charge, to any person obtaining
  a copy of this software and associated documentation files (the
  "Software"), to deal in the Software without restriction, including
  without limitation the rights to use, copy, modify, merge, publish,
  distribute, sublicense, and/or sell copies of the Software, and to
  permit persons to whom the Software is furnished to do so, subject to
  the following conditions:
  The above copyright notice and this permission notice shall be included
  in all copies or substantial portions of the Software.


  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.cur
  IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
  CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
  TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <Metro.h>
#include <Watchdog_t4.h>
#include <ADC.h>
#include <ADC_util.h>
#include <EEPROM.h>

//CAN Setup

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Can2;
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can3;

#define NUM_TX_MAILBOXES 10
#define NUM_RX_MAILBOXES 10

CAN_message_t msg;

signed long loopTime = 0;

void canRX_289(const CAN_message_t &msg); //Inverter RPM, Battery and Torque
void canRX_299(const CAN_message_t &msg); //Inverter Temps
void canRX_351(const CAN_message_t &msg); //BMS Status
void canRX_355(const CAN_message_t &msg); //BMS Status
void canRX_356(const CAN_message_t &msg); //BMS HV Voltage
void canRX_377(const CAN_message_t &msg); //Outlander Charger Low voltage stats
void canRX_389(const CAN_message_t &msg); //Outlander Charger HV stats
void canRX_732(const CAN_message_t &msg); //Inverter Current
void canRX_733(const CAN_message_t &msg); //Inverter Temps
void dashComms();                         //update the dash-board
void bmsComms();                          //Comms to the BMS - Set Key on

void dogFood();
void menu();
void readPins();
void readPedal();
void inverterComms();
void tempCheck();
void showInfo();
void loadDefault();
void saveVarsToEEPROM();
void stateHandler();

//Metro Timers

Metro timer50_1 = Metro(50);     //inverter timer
Metro timer50_2 = Metro(50);     //De-bounce check
Metro timer100_1 = Metro(100);   //2nd inverter timer
Metro timer100_2 = Metro(96);    //longer Debounce
Metro timer100_3 = Metro(110);   //Temp handler
Metro timer500_1 = Metro(50);    //pedal debug timer
Metro timer1000_1 = Metro(1000); //General 1s timer
Metro timer2000_1 = Metro(2000); //Serial update timer
Metro timer30s_1 = Metro(30000); //30Sec Timer to check DC-DC State
Metro timer10_1 = Metro(10);     //Dash coms timer - needs to be fast for stepper motor

//Watchdog Setup
WDT_T4<WDT1> wdt;

void wdtCallback()
{
  Serial.println("FEED THE DOG SOON, OR RESET!");
}

//ADC setup

ADC *adc = new ADC();

// Define Outputs
#define OUT1 6    //NEG Contactor
#define OUT2 9    //PRE Charge Contactor
#define OUT3 10   //Drive Contactor
#define OUT4 11   //Brake Lights
#define OUT5 12   //Pump
#define OUT6 24   //FAN
#define OUT7 25   // No connection
#define OUT8 28   //DC -DC Enable
#define OUT9 29   //Temp Gauge
#define OUT10 33  //RED LED
#define OUT11 36  //Green LED
#define OUT12 37  //Reverse Lights
#define LEDpin 13 //Builtin LED

//Define Inputs
#define ISO_IN1 2  //FWD
#define ISO_IN2 3  //START
#define ISO_IN3 4  //Brake
#define ISO_IN4 5  //REV
#define ISO_IN5 26 // MAP 2 ECO
#define ISO_IN6 27 // MAP 3 SPORT
#define ISO_IN7 32 // IGNITION
#define ISO_IN8 21 // PP Detect

#define POT_A 41 //POT A
#define POT_B 40 //POT B

//VCU Staus

#define boot 1
#define ready 2
#define driveNeutral 3
#define driveForward 4
#define driveReverse 5
#define charging 6
#define error 7

uint8_t VCUstatusChangeCounter = 0;    //used for hysterisys to stop jitter on VCU status
uint8_t VCUstatusChangeThreshold = 60; // n loops

uint8_t VCUstatus = 1;

uint8_t contactorState = 0; //state of contactors

//Setup Variables

uint8_t brake_pedal; //Brake lights

uint8_t start;
uint8_t ppDetect;    //Prox pilot pin
uint8_t ignition;    //ignition
uint8_t dir_FWD;     //Drive Switch is set to forward
uint8_t dir_REV;     //dRIVE Sitch is to Reverse
uint8_t dir_NEUTRAL; //Set Neutral as the default state
uint8_t BMS_Status;  //BMS Status
uint8_t BMS_SOC;
uint8_t inverterFunction = 0x00;
uint8_t BMS_keyOn = 0;

float BMS_avgtmp; //BMS Battery AVG Temp
float currentact; //BMS Current
float BMS_packvoltage;
int BMS_discurrent;

unsigned long pretimer1;

int chargerHVbattryVolts = 0;
int chargerACvolts = 0;
float charger12vbattryVolts = 0;
float charger12vCurrent = 0;
int chargerTemp1 = 0;
int chargerTemp2 = 0;
int chargerTemp3 = 0;
int chargerTemp4 = 0;
int chargerHVcurrent = 0;
uint8_t chargerStatus;
int avgChargerTemp = 0;

int motorRPM = 0;
int motorTempPeak = 0;
int motorTemp1 = 0;
int motorTemp2 = 0;
uint16_t motorHVbatteryVolts = 0;
int motorTorque = 0;
int motorCurrent1 = 0;
int motorCurrent2 = 0;
int avgMotorTemp = 0;

int inverterTemp1 = 0;
int inverterTemp2 = 0;
int avgInverterTemp = 0;

byte torqueHibyte = 0;
byte torqueLoByte = 0;
int torqueRequest = 0;
int targetTorque = 0;
int curentTorque = 0;
int throttlePosition = 0;
uint8_t pedalDebug = 0;
uint8_t inverterEnable = 1;
int regenTarget = 0;       // target regen torque
uint32_t regenTimer = 0;   // timer for regen delay
uint32_t regenDelay = 250; //delay before regen Starts
uint8_t regenState = 0;    //Are we requesting regen
uint32_t brakeDelay = 0;
int regenRequest = 0;

int incomingByte;
uint8_t menuLoad = 0;
uint8_t showStats = 1;
uint8_t pumpState = 0;
uint8_t fanState = 0;
//EEPROM Stored Vars

uint8_t tempGaugeMin = EEPROM.read(1); //Temp Gauge min PWM
uint8_t tempGaugeMax = EEPROM.read(2); //Temp Gauge Max PWM

uint8_t pumpOnTemp = EEPROM.read(3);
uint8_t pumpoffTemp = EEPROM.read(4);

uint8_t fanOnTemp = EEPROM.read(5);
uint8_t fanOffTemp = EEPROM.read(6);

int maxTorque = EEPROM.read(31) * 255 + EEPROM.read(30);
int minTorque = EEPROM.read(8);                              //Used for creeping
int tpslowOffset = EEPROM.read(21) * 255 + EEPROM.read(20);  //Value when foot off pedal
int tpshighOffset = EEPROM.read(23) * 255 + EEPROM.read(22); //Value when foot on pedal
int torqueIncrement = EEPROM.read(11);                       //used for default rpm based regen
uint8_t setTpsLow = 0;
uint8_t setTpsHigh = 0;

uint8_t active_map = 1;   //Active Pedal map
uint8_t map2;         //Eco Map
uint8_t map3;         //Sport MAp

//Setup the peddal map arrays..

byte idx_j, idx_k; //index of tps,rpm bins
int pedal_offset;

const int num_rpm_bins = 21;
const int tpsbins[21] = {0, 3, 5, 8, 10, 13, 18, 20, 23, 25, 28, 32, 34, 40, 50, 60, 70, 80, 90, 100, 101};

const int rpmbins[num_rpm_bins] = {
    250, 500, 625, 750, 1000, 1250, 1500, 2000, 2500, 3000, 3500, 4000, 4500, 5000, 5500, 6000, 6500, 7000, 7500, 8000, 10000};

const int pedal_map_one[21][22] = {   //Normal
    //map 1..
    /*250*/ {0, 0, 6, 6, 6, 5, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5},
    /*500*/ {-10, 0, 6, 6, 6, 5, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5},
    /*625*/ {-20, 0, 6, 6, 6, 5, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5},
    /*750*/ {-30, 0, 6, 6, 6, 5, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5},
    /*1000*/ {-50, 0, 6, 6, 6, 5, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5},
    /*1250*/ {-70, 0, 6, 6, 6, 5, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5},
    /*1500*/ {-90, 0, 6, 6, 6, 5, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5},
    /*2000*/ {-110, 0, 6, 6, 6, 5, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5},
    /*2500*/ {-130, 0, 6, 6, 6, 5, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5},
    /*3000*/ {-150, 0, 6, 6, 6, 5, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5},
    /*3500*/ {-150, 0, 6, 6, 6, 5, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5},
    /*4000*/ {-150, 0, 6, 6, 6, 5, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5},
    /*4500*/ {-150, 0, 6, 6, 6, 5, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5},
    /*5000*/ {-160, 0, 6, 6, 6, 5, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5},
    /*5500*/ {-180, 0, 6, 6, 6, 5, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5},
    /*6000*/ {-200, 0, 6, 6, 6, 5, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5},
    /*6500*/ {-200, 0, 6, 6, 6, 5, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5},
    /*7000*/ {-225, 0, 6, 6, 6, 5, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5},
    /*7500*/ {-250, 0, 6, 6, 6, 5, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5},
    /*8000*/ {-300, 0, 6, 6, 6, 5, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5},
    /*10000*/ {-300, 0, 6, 6, 6, 5, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5},
};

const int pedal_map_two[21][22] = {   //ECO
    //map 2..
    /*250*/ {0, 0, 6, 6, 6, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6},
    /*500*/ {-10, 0, 6, 6, 6, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6},
    /*625*/ {-20, 0, 6, 6, 6, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6},
    /*750*/ {-30, 0, 6, 6, 6, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6},
    /*1000*/ {-50, 0, 6, 6, 6, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6},
    /*1250*/ {-70, 0, 6, 6, 6, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6},
    /*1500*/ {-90, 0, 6, 6, 6, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6},
    /*2000*/ {-110, 0, 0, 6, 6, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6},
    /*2500*/ {-130, 0, 0, 6, 6, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6},
    /*3000*/ {-150, 0, 0, 6, 6, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6},
    /*3500*/ {-150, 0, 0, 6, 6, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6},
    /*4000*/ {-150, 0, 0, 3, 6, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6},
    /*4500*/ {-150, 0, 0, 3, 6, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6},
    /*5000*/ {-160, 0, 0, 3, 6, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6},
    /*5500*/ {-180, 0, 0, 3, 6, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6},
    /*6000*/ {-200, 0, 0, 3, 6, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6},
    /*6500*/ {-200, 0, 0, 3, 6, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6},
    /*7000*/ {-225, 0, 0, 3, 6, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6},
    /*7500*/ {-250, 0, 0, 3, 6, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6},
    /*8000*/ {-300, 0, 0, 3, 6, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6},
    /*10000*/ {-300, 0, 0, 3, 6, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6},
};

const int pedal_map_three[21][22] = {  //Sport
    //map 3..
    /*250*/ {0, 0, 7, 7, 7, 6, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 6},
    /*500*/ {-10, 0, 7, 7, 7, 6, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 6},
    /*625*/ {-20, 0, 7, 7, 7, 6, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 6},
    /*750*/ {-30, 0, 7, 7, 7, 6, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 6},
    /*1000*/ {-50, 0, 7, 7, 7, 6, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 6},
    /*1250*/ {-70, 0, 7, 7, 7, 6, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 6},
    /*1500*/ {-90, 0, 7, 7, 7, 6, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 6},
    /*2000*/ {-110, 0, 7, 7, 7, 6, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 6},
    /*2500*/ {-130, 0, 7, 7, 7, 6, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 6},
    /*3000*/ {-150, 0, 7, 7, 7, 6, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 6},
    /*3500*/ {-150, 0, 7, 7, 7, 6, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 6},
    /*4000*/ {-150, 0, 7, 7, 7, 6, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 6},
    /*4500*/ {-150, 0, 7, 7, 7, 6, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 6},
    /*5000*/ {-160, 0, 7, 7, 7, 6, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 6},
    /*5500*/ {-180, 0, 7, 7, 7, 6, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 6},
    /*6000*/ {-200, 0, 7, 7, 7, 6, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 6},
    /*6500*/ {-200, 0, 7, 7, 7, 6, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 6},
    /*7000*/ {-225, 0, 7, 7, 7, 6, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 6},
    /*7500*/ {-250, 0, 7, 7, 7, 6, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 6},
    /*8000*/ {-300, 0, 7, 7, 7, 6, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 6},
    /*10000*/ {-300, 0, 7, 7, 7, 6, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 6},
};

void setup()
{

  //Setup Can

  Can1.begin();
  Can2.begin();
  Can3.begin();
  Can1.setBaudRate(500000);
  Can2.setBaudRate(500000);
  Can3.setBaudRate(50000);

  Can1.setMaxMB(NUM_TX_MAILBOXES + NUM_RX_MAILBOXES);
  for (int i = 0; i < NUM_RX_MAILBOXES; i++)
  {
    Can1.setMB((FLEXCAN_MAILBOX)i, RX, STD);
  }
  Can2.setMaxMB(NUM_TX_MAILBOXES + NUM_RX_MAILBOXES);
  for (int i = 0; i < NUM_RX_MAILBOXES; i++)
  {
    Can2.setMB((FLEXCAN_MAILBOX)i, RX, STD);
  }
  Can3.setMaxMB(NUM_TX_MAILBOXES + NUM_RX_MAILBOXES);
  for (int i = 0; i < NUM_RX_MAILBOXES; i++)
  {
    Can3.setMB((FLEXCAN_MAILBOX)i, RX, STD);
  }

  for (int i = NUM_RX_MAILBOXES; i < (NUM_TX_MAILBOXES + NUM_RX_MAILBOXES); i++)
  {
    Can1.setMB((FLEXCAN_MAILBOX)i, TX, STD);
    Can2.setMB((FLEXCAN_MAILBOX)i, TX, STD);
    Can3.setMB((FLEXCAN_MAILBOX)i, TX, STD);
  }

  Can1.setMBFilter(REJECT_ALL);
  Can1.enableMBInterrupts();
  Can2.setMBFilter(REJECT_ALL);
  Can2.enableMBInterrupts();
  Can3.setMBFilter(REJECT_ALL);
  Can3.enableMBInterrupts();
  Can1.onReceive(MB0, canRX_377); // Charger LV Stats
  Can1.onReceive(MB1, canRX_355); //BMS Status
  Can1.onReceive(MB2, canRX_389); //Charger HV Stats
  Can2.onReceive(MB3, canRX_289); //Inverter RPM Battery and Torque
  Can2.onReceive(MB4, canRX_299); //Inverter Temps
  Can2.onReceive(MB5, canRX_732); //Inverter current
  Can2.onReceive(MB6, canRX_733); //Inverter Temps
  Can1.onReceive(MB7, canRX_356); //BMS HV Voltage
  Can1.onReceive(MB8, canRX_351); //BMS HV Voltage
  Can1.setMBFilter(MB0, 0x377);
  Can1.setMBFilter(MB1, 0x355);
  Can1.setMBFilter(MB2, 0x389);
  Can1.setMBFilter(MB7, 0x356);
  Can1.setMBFilter(MB8, 0x351);
  Can2.setMBFilter(MB3, 0x289);
  Can2.setMBFilter(MB4, 0x299);
  Can2.setMBFilter(MB5, 0x732);
  Can2.setMBFilter(MB6, 0x733);

  Can1.mailboxStatus();
  Can2.mailboxStatus();
  Can3.mailboxStatus();

  //Setup ADC

  adc->adc0->setAveraging(16);                                    // set number of averages
  adc->adc0->setResolution(16);                                   // set bits of resolution
  adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::MED_SPEED); // change the conversion speed
  adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::MED_SPEED);     // change the sampling speed

  adc->adc1->setAveraging(16);                                    // set number of averages
  adc->adc1->setResolution(16);                                   // set bits of resolution
  adc->adc1->setConversionSpeed(ADC_CONVERSION_SPEED::MED_SPEED); // change the conversion speed
  adc->adc1->setSamplingSpeed(ADC_SAMPLING_SPEED::MED_SPEED);     // change the sampling speed

  //Set up Outputs
  pinMode(OUT1, OUTPUT);   //Neg Cnct
  pinMode(OUT2, OUTPUT);   //Pre Cnct
  pinMode(OUT3, OUTPUT);   //Drive cnct
  pinMode(OUT4, OUTPUT);   //Brake lights
  pinMode(OUT5, OUTPUT);   //Pump
  pinMode(OUT6, OUTPUT);   //Fan
  pinMode(OUT7, OUTPUT);   //No conection
  pinMode(OUT8, OUTPUT);   //DC-DC Enable
  pinMode(OUT9, OUTPUT);   //Temp Gauge
  pinMode(OUT10, OUTPUT);  //Dash Red LED
  pinMode(OUT11, OUTPUT);  //Dash Green LED
  pinMode(OUT12, OUTPUT);  //Rev Lights
  pinMode(LEDpin, OUTPUT); //Builtin LED

  //Setup Inputs
  pinMode(ISO_IN1, INPUT); //FWD
  pinMode(ISO_IN2, INPUT); //start from key
  pinMode(ISO_IN3, INPUT); //Brake
  pinMode(ISO_IN4, INPUT); //REV
  pinMode(ISO_IN5, INPUT); //IGN
  pinMode(ISO_IN6, INPUT); //Map 1
  pinMode(ISO_IN7, INPUT); //MAP 2
  pinMode(ISO_IN8, INPUT); // PP Detect

  pinMode(POT_A, INPUT); //Throtle Pot A
  pinMode(POT_B, INPUT); //Throtle Pot B

  // Write initial Vals to Outputs
  digitalWrite(OUT1, LOW);
  digitalWrite(OUT2, LOW);
  digitalWrite(OUT3, LOW);
  digitalWrite(OUT4, LOW);
  digitalWrite(OUT5, LOW);
  digitalWrite(OUT6, LOW);
  digitalWrite(OUT7, LOW);
  digitalWrite(OUT8, LOW);
  analogWrite(OUT9, LOW);
  digitalWrite(OUT10, HIGH);
  digitalWrite(OUT11, LOW);
  digitalWrite(OUT12, LOW);
  digitalWrite(LEDpin, HIGH);

  //Watch Dog setup
  WDT_timings_t config;
  config.trigger = 1; //in seconds, 0->128
  config.timeout = 5; //in seconds, 0->128 
  config.callback = wdtCallback;
  wdt.begin(config);

  Serial.begin(9600);
  Serial.println("Mini-E VCU Starting Up.....");
  Serial.print("VCU Status: ");
  Serial.println(VCUstatus);
  delay(1000);
}

ADC::Sync_result result;

void loop()
{
  long loopTimestart = micros();
  dogFood(); //Reset the Watchdog

  Can1.events(); //Call CAN bus bus interupts
  Can2.events();
  Can3.events(); //Currently this only send.

  dashComms();  
  tempCheck();  
  stateHandler();
  readPins();

  if (pedalDebug == 1)
  {
    readPedal();
  }

  if (Serial.available() > 0)
  {
    incomingByte = Serial.read();
    if (incomingByte == 115) //press s for menu
    {
      menuLoad = 0;
      showStats = 0;
      menu();
    }
    else
    {
      menu();
    }
  }

  if (timer50_2.check() == 1)
  {

    bmsComms();
  }

  if ((timer2000_1.check() == 1) && showStats == 1)
  {

    showInfo();
  }
  loopTime = micros();
  loopTime = loopTime - loopTimestart;
}

//---------------end of loop -----------------------

void menu()
{
  if (menuLoad == 0)
  { //Serial Menu
    Serial.println("----------------Mini E VCU Settings Menu----------------");
    Serial.println();
    Serial.println("1 - Set Pump on Temp");
    Serial.println("2 - Set Pump off Temp");
    Serial.println("3 - Temp Gauge PWM Test");
    Serial.println("4 - Set Temp Gauge Low Value");
    Serial.println("5 - Set Temp Gauge High Value");
    Serial.println("6 - Set Pedal Lo Offset");
    Serial.println("7 - Set Pedal High Offset");
    Serial.println("8 - Set Max Tourqe");
    Serial.println("9 - Set Map");
    Serial.println("0 - Set Torque Increment");
    Serial.println("F - Fan On / Off");
    Serial.println("M - Pump on / Off");
    Serial.println("P - Show Pedal debug info");
    Serial.println("X - Enable / Disable inverter");
    Serial.println("L - Load Default Values");
    Serial.println("V - VCU Status");
    Serial.println("I - Input Test");
    Serial.println("C - Contactor Test - ONLY WITH HV OFF!!");
    Serial.println("R - Reset Board");
    Serial.println("S - Show this menu");
    Serial.println("Q - Quit and Save");
    menuLoad = 1;
  }

  switch (incomingByte)
  {
  case '1':
  {
    int value = Serial.parseInt();
    if (value > 100)
    {
      value = 100;
    }
    pumpOnTemp = value;
    Serial.print("Pump on Set to: ");
    Serial.println(value);
    menuLoad = 0;

    break;
  }
  case '2':
  {
    int value = Serial.parseInt();
    if (value > 100)
    {
      value = 100;
    }
    pumpoffTemp = value;
    Serial.print("Pump off Set to: ");
    Serial.println(value);
    menuLoad = 0;

    break;
  }

  case '3':
  {
    int value = Serial.parseInt();
    if (value > 255)
    {
      value = 255;
    }
    analogWrite(OUT9, value);
    Serial.print("Gauge Set to: ");
    Serial.println(value);
    menuLoad = 0;

    break;
  }

  case '4':
  {
    int value = Serial.parseInt();
    if (value > 255)
    {
      value = 255;
    }
    if (value < 0)
    {
      value = 0;
    }
    tempGaugeMin = value;
    Serial.print("Gauge Min Set to: ");
    Serial.println(value);
    menuLoad = 0;
  }
  break;

  case '5':
  {
    int value = Serial.parseInt();
    if (value > 255)
    {
      value = 255;
    }
    if (value < 0)
    {
      value = 0;
    }
    tempGaugeMax = value;
    Serial.print("Gauge Max Set to: ");
    Serial.println(value);
    menuLoad = 0;
  }

  break;

  case '6':
    setTpsLow = 1;
    readPedal();
    Serial.print(" ADC1 Reading: ");
    Serial.print(" Pedal Low Offset Set to: ");
    Serial.println(tpslowOffset);
    menuLoad = 0;

    break;

  case '7':
    setTpsHigh = 1;
    readPedal();
    Serial.print(" ADC1 Reading: ");
    Serial.print(" Pedal High Offset Set to: ");
    Serial.println(tpshighOffset);
    menuLoad = 0;

    break;

  case '8':
  {
    int value = Serial.parseInt();
    if (value > 2000)
    {
      value = 2000;
    }
    if (value < 0)
    {
      value = 0;
    }
    maxTorque = value;
    Serial.print("Max Toruqe: ");
    Serial.println(value);
    menuLoad = 0;

    break;
  }

  case '9':
  {
    int value = Serial.parseInt();
    if (value > 4)
    {
      value = 4;
    }
    if (value < 1)
    {
      value = 1;
    }
    active_map = value;
    Serial.print("Map Set to: ");
    Serial.println(value);
    menuLoad = 0;
  }

  case '0':
  {
    int value = Serial.parseInt();
    if (value > 100)
    {
      value = 100;
    }
    if (value < 0)
    {
      value = 0;
    }
    torqueIncrement = value;

    Serial.print("Torque Increment Set to: ");
    Serial.println(value);
    menuLoad = 0;
  }

  case 'd':
    showInfo();
    menuLoad = 0;

    break;

  case 'f':
    if (fanState == 0)
    {
      fanState = 2; //2 = Debug
      digitalWrite(OUT6, HIGH);
      Serial.print("FAN ON");
    }
    else
    {
      fanState = 0;
      digitalWrite(OUT6, LOW);
      Serial.print("FAN OFF");
    }
    break;

    break;

  case 'l':
    loadDefault();
    menuLoad = 0;

    break;

  case 'c':
    if (VCUstatus == 2)
    {
      Serial.println("NEG ON");
      digitalWrite(OUT1, HIGH);
      delay(1000);
      Serial.println("NEG OFF");
      digitalWrite(OUT1, LOW);
      delay(500);
      Serial.println("PRE ON");
      digitalWrite(OUT2, HIGH);
      delay(1000);
      Serial.println("PRE OFF");
      digitalWrite(OUT2, LOW);
      delay(500);
      Serial.println("MAIN ON");
      digitalWrite(OUT3, HIGH);
      delay(1000);
      Serial.println("MAIN OFF");
      digitalWrite(OUT3, LOW);
    }
    else
    {
      Serial.println("HV MUST BE OFF");
    }
    break;

  case 'm':
    if (pumpState == 0)
    {
      pumpState = 2; //2 = Debug
      digitalWrite(OUT5, HIGH);
      Serial.print("PUMP ON");
    }
    else
    {
      pumpState = 0;
      digitalWrite(OUT5, LOW);
      Serial.print("PUMP OFF");
    }
    break;

  case 'p':
    if (pedalDebug == 0)
    {
      pedalDebug = 1;
      showStats = 0;
    }
    else
    {
      pedalDebug = 0;
      showStats = 1;
    }
    break;

  case 'x':
    if (inverterFunction == 0x00)
    {
      inverterFunction = 0x03;
      inverterEnable = 1;
      Serial.println("Inverter Enable");
    }
    else

    {
      inverterFunction = 0x00;
      inverterEnable = 0;
      Serial.println("Inverter Disable");
    }

    break;

  case 'v':
    Serial.print("");
    Serial.print("VCU Status: ");
    Serial.println(VCUstatus);
    menuLoad = 0;

    break;

  case 'i':
    readPins();
    Serial.println("Reading Pins...");
    Serial.print("ISO 2 - Start Signal:");
    Serial.println(start);
    Serial.print("ISO 3 - Brake Pedal:");
    Serial.println(brake_pedal);
    Serial.print("ISO 1 - Forward:");
    Serial.println(dir_FWD);
    Serial.print("ISO 4 - Reverse:");
    Serial.println(dir_REV);
    Serial.print("ISO  8 - Charge Plug Detect:");
    Serial.println(ppDetect);
    Serial.print("ISO 6 - MAP 2:");
    Serial.println(map2);
    Serial.print("ISO 7 - MAP 3:");
    Serial.println(map3);
    Serial.print("Thotle POT A:");
    Serial.print(result.result_adc0);
    Serial.print(" , Thotle POT B:");
    Serial.println(result.result_adc1);
    menuLoad = 0;

    break;

  case 'r':
    wdt.reset();
    break;

  case 'q':

    saveVarsToEEPROM();
    showStats = 1;
    menuLoad = 0;
    break;
  }
}

void readPins()
{

  dir_FWD = digitalRead(ISO_IN1);
  start = digitalRead(ISO_IN2);
  brake_pedal = digitalRead(ISO_IN3);
  dir_REV = digitalRead(ISO_IN4);
  map2 = digitalRead(ISO_IN5);
  map3 = digitalRead(ISO_IN6);
  ignition = digitalRead(ISO_IN7);
  ppDetect = digitalRead(ISO_IN8);
}

void readPedal()

//Sync Analog read
//compare the result

{
  ADC::Sync_result result = adc->analogSyncRead(POT_A, POT_B);
  result.result_adc0 = (uint16_t)result.result_adc0;
  result.result_adc1 = (uint16_t)result.result_adc1;
  float comparisonResult = ((result.result_adc1 * 1.0) / (result.result_adc0 * 1.0));
  if (comparisonResult != 0)
  {
    if ((comparisonResult > 1.0) && (comparisonResult < 2.0))
    {
     
      throttlePosition = map(result.result_adc1, tpslowOffset, tpshighOffset, 0, 100);
      if (throttlePosition < 2)
      {
        throttlePosition = 0;
      }
      if (throttlePosition > 100)
      {
        throttlePosition = 100;
      }
    }

    // if ((comparisonResult < 1.0 || (comparisonResult > 2.0)))
    // {
    //   // Serial.println("--!PEDDAL MISMATCH!--");
    //   // throttlePosition = 0;
    // }
  }

  if (adc->adc0->fail_flag != ADC_ERROR::CLEAR)
  {
    Serial.print("ADC0: ");
    Serial.println(getStringADCError(adc->adc0->fail_flag));
  }
  if (adc->adc1->fail_flag != ADC_ERROR::CLEAR)
  {
    Serial.print("ADC1: ");
    Serial.println(getStringADCError(adc->adc1->fail_flag));
  }

  if ((timer500_1.check() == 1) && pedalDebug == 1)
  {
    Serial.print("ADC0 Reading: ");
    Serial.print(result.result_adc0);
    Serial.print(" ADC1 Reading: ");
    Serial.print(result.result_adc1);
    Serial.print(" Comparison Result: ");
    Serial.print(comparisonResult);

    Serial.print(" Throtle Position: ");
    Serial.println(throttlePosition);
  }

  if (setTpsLow == 1)
  {
    Serial.print(" ADC1 Reading: ");
    Serial.print(result.result_adc1);
    tpslowOffset = result.result_adc1;
    setTpsLow = 0;
  }
  if (setTpsHigh == 1)
  {
    Serial.print(" ADC1 Reading: ");
    Serial.print(result.result_adc1);
    tpshighOffset = result.result_adc1;
    setTpsHigh = 0;
  }
}

void dogFood()
{
  wdt.feed(); //Feed the Watchdog
}

//CAN Functions

void canRX_289(const CAN_message_t &msg)
{
  motorTorque = ((((msg.buf[0] * 256) + msg.buf[1]) - 10000) / 10); //Motor Torque -200 / + 200nm
  motorRPM = (msg.buf[2] * 256 + msg.buf[3] - 20000);               //289_RrRPM,Rear RPM,220289,C*256+D-20000,-10000,10000,RPM,
  motorHVbatteryVolts = (msg.buf[4] * 256 + msg.buf[5]);            //Inverter HV
}

void canRX_299(const CAN_message_t &msg)
{
  //inverter Temps
  motorTempPeak = (msg.buf[0] - 40);
  inverterTemp1 = (msg.buf[1] - 40);
  inverterTemp2 = (msg.buf[4] - 40);

  avgInverterTemp = (inverterTemp1 + inverterTemp2) / 2;
}

void canRX_351(const CAN_message_t &msg)
{
  BMS_discurrent = (((msg.buf[5] * 256) + msg.buf[4]) / 10);
}

void canRX_355(const CAN_message_t &msg)
{
  BMS_SOC = ((msg.buf[1] * 256) + msg.buf[0]);
  BMS_avgtmp = (((msg.buf[5] / 10.0) * 256) + (msg.buf[4] / 10.0));
  BMS_Status = msg.buf[6];

  if (BMS_Status == 5)
  {
    if (VCUstatusChangeCounter > VCUstatusChangeThreshold)

      VCUstatusChangeCounter = 0;
    VCUstatus = 7;
  }
  else
  {
    VCUstatusChangeCounter++;
  }
}

void canRX_356(const CAN_message_t &msg)
{

  BMS_packvoltage = (((msg.buf[1] * 256) / 100.0) + (msg.buf[0] / 100.0));

  int amps = (msg.buf[3] * 256) + (msg.buf[2]);
  currentact = ((amps - 3000) / 10.0);
}

void canRX_377(const CAN_message_t &msg)
{
  charger12vbattryVolts = float(((msg.buf[0] * 256) + msg.buf[1]) * 0.01);
  charger12vCurrent = float(((msg.buf[2] * 256) + msg.buf[3]) * 0.01);
  chargerTemp1 = msg.buf[4] - 40;
  chargerTemp2 = msg.buf[5] - 40;
  chargerTemp3 = msg.buf[6] - 40;
  chargerStatus = msg.buf[7];
  avgChargerTemp = (chargerTemp1 + chargerTemp2 + chargerTemp3 + chargerTemp4) / 4;
}

void canRX_389(const CAN_message_t &msg)
{
  chargerHVbattryVolts = msg.buf[0] * 2; //Charger HV Battery Voltage
  chargerACvolts = msg.buf[1];
  chargerHVcurrent = msg.buf[2];
  chargerTemp4 = msg.buf[4] - 40;
}

void canRX_732(const CAN_message_t &msg)
{
  // inverter Current
  motorCurrent1 = (msg.buf[0] * 256 + msg.buf[1] - 1000);
  motorCurrent2 = (msg.buf[2] * 256 + msg.buf[3] - 1000);
}

void canRX_733(const CAN_message_t &msg)
{
  //inverter Temps
  motorTemp1 = (msg.buf[0] - 40);
  motorTemp2 = (msg.buf[2] - 40);
  avgMotorTemp = (motorTemp1 + motorTemp2 + motorTempPeak) / 3;
}

void inverterComms()
{
  if (timer50_1.check() == 1)
  {

    if (regenState == 0 && VCUstatus == 4 && motorRPM > 2000) // Increment Torque delivery
    {

      if (curentTorque < targetTorque)
      {
        curentTorque += torqueIncrement;
        torqueRequest = curentTorque;
      }

      if (curentTorque >= targetTorque)
      {
        torqueRequest = targetTorque;
        curentTorque = targetTorque;
      }
    }
    else if (regenState == 0 && VCUstatus == 4 && motorRPM < 2000)
    {
      torqueRequest = targetTorque;
      curentTorque = torqueRequest;
    }

    if (active_map == 3)  // No need to increment in 'sport' mode
    {
      torqueRequest = targetTorque;
      curentTorque = torqueRequest;
    }

    if (regenState == 1 && VCUstatus == 4)

    {
      if (regenTarget < regenRequest) // increment Regen
      {
        regenRequest -= 5;
        torqueRequest = regenRequest;
        // Serial.println("Regen inc.");
      }
      else
      regenRequest = regenTarget;
      torqueRequest = regenRequest;
    }

    if (regenState == 2 && VCUstatus == 4)

    {
      if (regenTarget > regenRequest) // increment Regen off
      {
        regenRequest += 30;
        torqueRequest = regenRequest;
        // Serial.println("Regen Dec.");
      }
    }

    if (torqueRequest > (2000))
    {
      torqueRequest = 0;
      Serial.println("--!OVER TOURQUE!--");
    }
    if (torqueRequest < (-1000))
    {
      torqueRequest = 0;
      Serial.println("--!UNDER TOURQUE!--");
    }

    torqueRequest += 10000;

    if (BMS_discurrent < currentact) //Decrese tourque if we are over current - Crude needs work..
    {
      torqueRequest -= 20;
      Serial.println("--!OVER CURRENT!--");
      if (torqueRequest < 0)
      {
        torqueRequest = 0;
      }
    }

    if (pedalDebug == 1)
    {
      Serial.print("Offset: ");
      Serial.print(pedal_offset);
      Serial.print(" Tourque Request: ");
      Serial.println(torqueRequest - 10000);
      Serial.print("Regen Target:");
      Serial.print(regenTarget);
      Serial.print(" Regen Request: ");
      Serial.print(regenRequest);
      Serial.print(" Motor Torque ");
      Serial.println(motorTorque);
    }
    if (inverterEnable != 1)
    {
      inverterFunction = 0x00;
    }
    torqueLoByte = lowByte(torqueRequest);
    torqueHibyte = highByte(torqueRequest);
    msg.id = 0x287;
    msg.len = 8;
    msg.buf[0] = 0;
    msg.buf[1] = 0;
    msg.buf[2] = torqueHibyte;
    msg.buf[3] = torqueLoByte;
    msg.buf[4] = 0;
    msg.buf[5] = 0;
    msg.buf[6] = inverterFunction;
    msg.buf[7] = 0;
    Can2.write(msg);
    torqueRequest = 0;
  }

  if (timer100_1.check() == 1)
  {
    msg.id = 0x371;
    msg.len = 8;
    msg.buf[0] = 48;
    msg.buf[1] = 0;
    msg.buf[2] = 0;
    msg.buf[3] = 0;
    msg.buf[4] = 0;
    msg.buf[5] = 0;
    msg.buf[6] = 0;
    msg.buf[7] = 0;
    Can2.write(msg);
    delay(1);
    msg.id = 0x285;
    msg.len = 8;
    msg.buf[0] = 0;
    msg.buf[1] = 0;
    msg.buf[2] = 20;
    msg.buf[3] = 57;
    msg.buf[4] = 143;
    msg.buf[5] = 254;
    msg.buf[6] = 12;
    msg.buf[7] = 16;
    Can2.write(msg);
    delay(1);

    msg.id = 0x286;
    msg.len = 8;
    msg.buf[0] = 0;
    msg.buf[1] = 0;
    msg.buf[2] = 0;
    msg.buf[3] = 61;
    msg.buf[4] = 0;
    msg.buf[5] = 0;
    msg.buf[6] = 33;
    msg.buf[7] = 0;
    Can2.write(msg);
  }
}

void dashComms()
{

  if (timer10_1.check() == 1)
  {
    msg.id = 0x555;
    msg.len = 8;
    msg.buf[0] = highByte(motorRPM);
    msg.buf[1] = lowByte(motorRPM);
    msg.buf[2] = VCUstatus;
    msg.buf[3] = BMS_Status;
    msg.buf[4] = BMS_SOC;
    msg.buf[5] = chargerHVbattryVolts / 2;
    msg.buf[6] = avgChargerTemp;
    msg.buf[7] = avgMotorTemp;
    Can3.write(msg);

    int amps = ((currentact * 10) + 3000);
    msg.id = 0x558;
    msg.len = 8;
    msg.buf[0] = lowByte(amps);
    msg.buf[1] = highByte(amps);
    msg.buf[2] = lowByte(int16_t(BMS_avgtmp * 10));
    msg.buf[3] = highByte(int16_t(BMS_avgtmp * 10));
    msg.buf[4] = lowByte(uint16_t(BMS_packvoltage) * 100);
    msg.buf[5] = highByte(uint16_t(BMS_packvoltage) * 100);
    msg.buf[6] = torqueHibyte;
    msg.buf[7] = torqueLoByte;
    Can3.write(msg);

    msg.id = 0x560;
    msg.len = 8;
    msg.buf[0] = active_map;
    msg.buf[1] = 0;;
    msg.buf[2] = 0;;
    msg.buf[3] = 0;
    msg.buf[4] = 0;
    msg.buf[5] = 0;
    msg.buf[6] = 0;
    msg.buf[7] = 0;
    Can3.write(msg);
  }
}

void bmsComms()
{

  msg.id = 0x456;
  msg.len = 8;
  msg.buf[0] = BMS_keyOn;
  msg.buf[1] = BMS_keyOn;
  msg.buf[2] = BMS_keyOn;
  msg.buf[3] = BMS_keyOn;
  msg.buf[4] = BMS_keyOn;
  msg.buf[5] = BMS_keyOn;
  msg.buf[6] = BMS_keyOn;
  msg.buf[7] = BMS_keyOn;
  Can1.write(msg);
}

void tempCheck()
{
  if (timer100_3.check() == 1)
  {

    if (avgChargerTemp || avgInverterTemp || avgMotorTemp > pumpOnTemp) //Temp Handling
    {

      if (pumpState == 0)
      {
        digitalWrite(OUT5, HIGH);
        pumpState = 1;
      }
    }
    if (VCUstatus != 6)
    {
      if (avgChargerTemp && avgMotorTemp && avgInverterTemp < pumpoffTemp)
      {
        if (pumpState == 1)
        {
          digitalWrite(OUT5, LOW);
          pumpState = 0;
        }
      }
    }

    if (VCUstatus == 6)
    {
      if (avgChargerTemp < pumpoffTemp)
      {

        if (pumpState == 1)
        {
          digitalWrite(OUT5, LOW);
          pumpState = 0;
        }
      }
    }

    if ((avgChargerTemp || avgInverterTemp || avgMotorTemp) > fanOnTemp)
    {
      if (fanState == 0)
      {
        digitalWrite(OUT6, HIGH);
        fanState = 1;
      }
    }
    if ((avgChargerTemp && avgInverterTemp && avgMotorTemp) < fanOffTemp)
    {
      if (fanState == 1)
      {
        digitalWrite(OUT6, LOW);
        fanState = 0;
      }
    }

    // Send To Temp Gaauge
    uint8_t tempGauge = (avgChargerTemp + avgMotorTemp + avgInverterTemp) / 3;
    tempGauge = map(tempGauge, 0, 100, tempGaugeMin, tempGaugeMax);
    analogWrite(OUT9, tempGauge);
  }
}

void showInfo()
{
  Serial.println();
  Serial.println();
  Serial.println();
  Serial.print("VCU Staus: ");
  Serial.print(VCUstatus);
  Serial.print( "Active MAp:");
  Serial.print(active_map);
  Serial.print("  BMS Status: ");
  Serial.print(BMS_Status);
  Serial.print("  Charger Status: ");
  Serial.print(chargerStatus, HEX);
  Serial.print(" BMS Current: ");
  Serial.print(currentact);
  Serial.print(" BMS HV: ");
  Serial.print(BMS_packvoltage);
  Serial.print(" BMS TEMP: ");
  Serial.print(BMS_avgtmp);
  Serial.print(" BMS Dischrge Limit");
  Serial.println(BMS_discurrent);
  Serial.print("SOC %: ");
  Serial.print(BMS_SOC);
  Serial.print(" DC-DC Volts: ");
  Serial.print(charger12vbattryVolts);
  Serial.print(" DC-DC Current: ");
  Serial.print(charger12vCurrent);
  Serial.print(" HV charger Current: ");
  Serial.print(chargerHVcurrent);
  Serial.print(" HV Battery Voltage: ");
  Serial.print(chargerHVbattryVolts);
  Serial.print(" Charger AC volts: ");
  Serial.println(chargerACvolts);
  Serial.print("Charger Temp1: ");
  Serial.print(chargerTemp1);
  Serial.print("  Charger Temp2: ");
  Serial.print(chargerTemp2);
  Serial.print("  Charger Temp3: ");
  Serial.print(chargerTemp3);
  Serial.print("  Charger Temp4: ");
  Serial.print(chargerTemp4);
  Serial.print("  Avg Charger Temp: ");
  Serial.println(avgChargerTemp);
  Serial.print("Inverter Temp 1: ");
  Serial.print(inverterTemp1);
  Serial.print(" Inverter Temp 2: ");
  Serial.print(inverterTemp2);
  Serial.print(" Avg Inverter Temp: ");
  Serial.println(avgInverterTemp);
  Serial.print("  Torque Request: ");
  Serial.println(torqueRequest);
  Serial.print("RPM: ");
  Serial.print(motorRPM);
  Serial.print(" Motor Peak Temp: ");
  Serial.print(motorTempPeak);
  Serial.print(" Mtr Temp 1: ");
  Serial.print(motorTemp1);
  Serial.print(" Mtr Temp 2: ");
  Serial.print(motorTemp2);
  Serial.print("Avg motor Temp: ");
  Serial.println(avgMotorTemp);
  Serial.print("Motor HV: ");
  Serial.print(motorHVbatteryVolts);
  Serial.print("  motor Torque: ");
  Serial.print(motorTorque);
  Serial.print("  Motor Amps1: ");
  Serial.print(motorCurrent1);
  Serial.print("  Motor amps2: ");
  Serial.println(motorCurrent2);
  Serial.print("Loop Time: ");
  Serial.print(loopTime);
}

void loadDefault() // Load the defaul values
{
  tempGaugeMin = 18;  //Temp Gauge min PWM
  tempGaugeMax = 128; //Temp Gauge Max PWM

  pumpOnTemp = 36;
  pumpoffTemp = 35;

  fanOnTemp = 71;
  fanOffTemp = 70;

  maxTorque = 200;      //Not used curently
  minTorque = 0;        //Not used curently
  tpslowOffset = 1000;  //Value when foot off pedal
  tpshighOffset = 3790; //Value when foot on pedal
  torqueIncrement = 20; //Value to ramp tourqe by
  active_map = 1;

  Serial.println("Loaded Default Vlues");
}

void saveVarsToEEPROM() //Save Values to EEPROM
{
  Serial.println("Writing Values to EEPROM...");
  EEPROM.update(1, tempGaugeMin);
  EEPROM.update(2, tempGaugeMax);
  EEPROM.update(3, pumpOnTemp);
  EEPROM.update(4, pumpoffTemp);
  EEPROM.update(5, fanOffTemp);
  EEPROM.update(6, fanOnTemp);

  EEPROM.update(8, minTorque);
  EEPROM.update(20, lowByte(tpslowOffset));
  EEPROM.update(21, highByte(tpslowOffset));
  EEPROM.update(22, lowByte(tpshighOffset));
  EEPROM.update(23, highByte(tpshighOffset));
  EEPROM.update(30, lowByte(maxTorque));
  EEPROM.update(31, highByte(maxTorque));
  EEPROM.update(11, torqueIncrement);
  Serial.println("Finished Writing Values to EEPROM...");
}

void stateHandler()
{
  switch (VCUstatus)
  {
  case boot:
  {

    VCUstatus = 2;

    Serial.print("VCU Status: ");
    Serial.println(VCUstatus);
    break;
  }
  case ready:
  {
    if (BMS_Status == 3)
    {
      if (VCUstatusChangeCounter > VCUstatusChangeThreshold)
      {
        VCUstatusChangeCounter = 0;
        VCUstatus = charging;
        Serial.print("VCU Status: ");
        Serial.println(VCUstatus);
      }
      else
      {
        VCUstatusChangeCounter++;
      }
    }

    if ((start == 0) && (brake_pedal == 0) && (BMS_Status == 1))
    {
      if ((dir_FWD == 1) && (dir_REV == 1))
      {
        VCUstatus = driveNeutral;
        Serial.print("VCU Status: ");
        Serial.println(VCUstatus);
      }
    }
    break;
  }
  case driveNeutral:
  {
    if (digitalRead(OUT12) == HIGH)
    {
      digitalWrite(OUT12, LOW); //Turn off the reversing lights
    }
    if (digitalRead(OUT4) == HIGH)
    {
      digitalWrite(OUT4, LOW); //Turn off the Brake lights
    }
    if (digitalRead(OUT1) == LOW) //Check if Neg Contactor is off then start the sequence
    {

      digitalWrite(OUT1, HIGH); //Negative Contactor
      Serial.println("Negative On!...");
      pretimer1 = millis();
    }
    if (pretimer1 + 500 < millis())
    {
      if ((digitalRead(OUT2) == LOW) && (digitalRead(OUT3) == LOW))
      {
        digitalWrite(OUT2, HIGH);
        Serial.println("PreCharge On!...");
        pretimer1 = millis();
      }
    }
    if (pretimer1 + 1000 < millis())
    {
      if ((motorHVbatteryVolts > 280) && (digitalRead(OUT3) == LOW)) //Check that the preCharge has finished
      {
        digitalWrite(OUT3, HIGH); //Turn on drive Contactor
        Serial.println("Main On!...");
        pretimer1 = millis();
      }
    }

    if ((pretimer1 + 500 < millis()) && (digitalRead(OUT2) == HIGH) && (digitalRead(OUT3) == HIGH))
    {
      digitalWrite(OUT2, LOW);
      Serial.println("PreCharge OFF!...");
      digitalWrite(OUT8, HIGH); //DC-DC Enable on
      Serial.println("DC-DC Enabled...");
      Serial.print("VCU Status: ");
      Serial.println(VCUstatus);
    }

    if ((digitalRead(OUT3) == HIGH) && (digitalRead(OUT2) == LOW) && (digitalRead(OUT1) == HIGH))
    {

      digitalWrite(OUT10, LOW);  //Red dash LED Off
      digitalWrite(OUT11, HIGH); //Green dash LED On
      if (motorRPM < 500)
      {                          //Dont shut down inverter above 2mph
        BMS_keyOn = 0;           //BMS Key off...Inverter Disable
        inverterFunction = 0x00; // Inverter Disable
      }
    }
    if ((dir_FWD == 0) && (dir_REV == 1) && (BMS_Status != 5))
    {
      if (VCUstatusChangeCounter > VCUstatusChangeThreshold)
      {
        VCUstatusChangeCounter = 0;
        VCUstatus = driveForward;
      }
      else
      {
        VCUstatusChangeCounter++;
      }
    }
    if ((dir_FWD == 1) && (dir_REV == 0) && (BMS_Status != 5))
    {

      if (motorRPM < 500)
      {
        if (VCUstatusChangeCounter > VCUstatusChangeThreshold)
        {
          VCUstatusChangeCounter = 0;
          VCUstatus = driveReverse;
        }
        else
        {
          VCUstatusChangeCounter++;
        }
      }
    }

    break;
  }

  case driveForward:
  {
    if ((dir_FWD == 1) && (dir_REV == 1))
    {

      if (VCUstatusChangeCounter > VCUstatusChangeThreshold)
      {
        VCUstatusChangeCounter = 0;
        VCUstatus = driveNeutral;
        digitalWrite(OUT12, LOW); //Turn off Rev Lights
      }
      else
      {
        VCUstatusChangeCounter++;
      }
    }
 
    readPedal();
    BMS_keyOn = 1; //Key on for BMS

    digitalWrite(OUT12, LOW); //Turn off Rev Lights
    // Pedal Map Handler

    byte j = 0; //index of tps
    byte k = 0; //index of throttle pos
    while (throttlePosition > tpsbins[k])
    {
      k++;
    }
    while (motorRPM > rpmbins[j] && j <= num_rpm_bins - 1)
    {
      j++;
    }
    if (j > num_rpm_bins - 1)
    {
      j = num_rpm_bins - 1;
    };
    idx_k = k;
    idx_j = j;

    if (map2 == 0 && map3 == 1)
    {
      active_map = 2;
    }

    if (map2 == 1 && map3 == 0)
    {
      active_map = 3;
    }

    if (map2 == 1 && map3 == 1)
    {
      active_map = 1;
    }

    switch (active_map)
    {
    case 1: // copy this code for other maps..
    {
      pedal_offset = pedal_map_one[idx_j][idx_k];
      break;
    }

    case 2: // copy this code for other maps..
    {
      pedal_offset = pedal_map_two[idx_j][idx_k];
      break;
    }

    case 3: // copy this code for other maps..
    {
      pedal_offset = pedal_map_three[idx_j][idx_k];
      break;
    }
    }

    if (pedal_offset > 1 && regenState != 0)
    {

      regenState = 2;
      regenTarget = 0;

      if (regenRequest >= -9)
      {
        regenState = 0;
        regenTarget = 0;
        regenRequest = 0;
        brakeDelay = 0;
        digitalWrite(OUT4, LOW); //Brake Lights off
      }
    }
    else if (pedal_offset > 1 && regenState != 2)
    {
      inverterFunction = 0x03;                              //Enable inverter
      targetTorque = (throttlePosition * pedal_offset) * 2; //*2 because we are 200nm Max torque
      regenState = 0;
      regenTarget = 0;
      regenRequest = 0;
    }

    if (pedal_offset < 0)
    {

      inverterFunction = 0x03;
      regenState = 1;
      regenTarget = pedal_offset * 2;
      if (brake_pedal == 1 && brakeDelay == 0)
      {
        brakeDelay = millis();
      }
      if (brake_pedal == 1 && brakeDelay + 1000 < millis() && regenRequest < -9)
      {
        digitalWrite(OUT4, HIGH);
        brakeDelay = 0;
      }
    }

    if (pedal_offset == 0)
    {
      torqueRequest = 0;
      targetTorque = 0;
    }

    if (brake_pedal == 0 && motorRPM < 100)
    {
      torqueRequest = 0;       //0 Torque if the brake is presed and we are nearly stopped
      inverterFunction = 0x00; //Shut off inverter
      regenState = 0;
      regenTarget = 0;
      regenRequest = 0;
      regenTimer = 0;
      brakeDelay = 0;
      digitalWrite(OUT4, LOW); //Brake Lights off
    }
    inverterComms();

    break;
  }

  case driveReverse:
  {
    if ((dir_FWD == 1) && (dir_REV == 1))
    {
      
      if (VCUstatusChangeCounter > VCUstatusChangeThreshold)
      {
        VCUstatusChangeCounter = 0;
        VCUstatus = driveNeutral;
        digitalWrite(OUT4, LOW);  //Turn off Barke Lights
        digitalWrite(OUT12, LOW); //Turn off Rev Lights
      }
      else
      {
        VCUstatusChangeCounter++;
      }
    }

    if ((dir_FWD == 0) && (dir_REV == 1))
    {

      if (VCUstatusChangeCounter > VCUstatusChangeThreshold)
      {
        VCUstatusChangeCounter = 0;
        VCUstatus = driveForward;
        digitalWrite(OUT12, LOW); //Turn off Rev Lights
      }
      else
      {
        VCUstatusChangeCounter++;
      }
    }
    readPedal();
    BMS_keyOn = 1;             //enable the inverter
    digitalWrite(OUT12, HIGH); //Turn on the reversing lights

    if (throttlePosition > 5)
    {
      torqueRequest = throttlePosition * -6; // lets make the pedal less responsive
      inverterFunction = 0x03;
      if (motorRPM < -2000)
      {
        torqueRequest = 0;
      }
    }

    if (brake_pedal == 0)
    {
      torqueRequest = 0; //0 Torque if the brake is presed
    }
    inverterComms();

    break;
  }
  case charging:
  {
    inverterFunction = 0x00;      // disable the inverter
    if (digitalRead(OUT1) == LOW) //Check if Neg Contactor is off then start the sequence
    {

      digitalWrite(OUT1, HIGH); //Negative Contactor
      Serial.println("Negative On!...");
      pretimer1 = millis();
    }
    if (pretimer1 + 500 < millis())
    {
      if ((digitalRead(OUT2) == LOW) && (digitalRead(OUT3) == LOW))
      {
        digitalWrite(OUT2, HIGH);
        Serial.println("PreCharge On!...");
        pretimer1 = millis();
      }
    }
    if (pretimer1 + 1000 < millis())
    {
      if ((chargerHVbattryVolts > 250) && digitalRead(OUT3) == LOW)
      {
        digitalWrite(OUT3, HIGH); //Turn on drive Contactor
        Serial.println("Main On!...");
        pretimer1 = millis();
      }
    }

    if ((pretimer1 + 500 < millis()) && (digitalRead(OUT2) == HIGH) && (digitalRead(OUT3) == HIGH))
    {
      digitalWrite(OUT2, LOW);
      Serial.println("PreCharge OFF!...");
      digitalWrite(OUT8, HIGH); //DC-DC Enable on
      Serial.println("DC-DC Enabled...");
      Serial.print("VCU Status: ");
      Serial.println(VCUstatus);
    }
    //Check the contactor state
    if ((digitalRead(OUT3) == HIGH) && (digitalRead(OUT2) == LOW) && (digitalRead(OUT1) == HIGH))
    {
      if (BMS_Status == 3)
      {

        BMS_keyOn = 0;            //Disable the inverter drive
        digitalWrite(OUT10, LOW); //Red LED Off
        digitalWrite(OUT2, LOW);  //We don't want any more smoke!
        if (timer1000_1.check() == 1)
        {
          digitalWrite(OUT11, !digitalRead(OUT11)); // Flash the Green LED
        }
        if (charger12vbattryVolts < 12.0)
        {
          digitalWrite(OUT8, HIGH); //DC-DC Enable on (charge the aux battery as well..)
        }

        if (timer30s_1.check() == 1)
        { //No need to keep the DC-DC on if the battery is charged
          if (charger12vCurrent < .5)
          {
            digitalWrite(OUT8, LOW);
          }
        }
      }
      else if (BMS_Status != 3)
      {
        if (timer1000_1.check() == 1)
        {
          Serial.println("Waiting for BMS!");
          BMS_keyOn = 0; //Disable the inverter drive
        }
      }
    }
    break;
  }

  case error:
  {
    BMS_keyOn = 0;
    inverterFunction = 0x00;
    Serial.println("ERROR !!!");

    break;
  }
  }
}
