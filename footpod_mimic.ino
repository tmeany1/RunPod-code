//For ESP32 - set board to DOIT_ESP32 DEVKIT V1
//
//
#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <math.h>

// variables will change:
uint16_t    inst_speed = 40;                                                 /**< Instantaneous Speed. */
uint8_t     inst_cadence = 1;                                               /**< Instantaneous Cadence. */
uint16_t    inst_stride_length = 1;                                         /**< Instantaneous Stride Length. */
uint32_t    total_distance = 10;

float kmph=0.0;
float old_kmph=0.0;
float mps=0.0;

unsigned long current_time=0;
unsigned long last_transmit_time=0;

byte fakePos[1] = {1};

bool _BLEClientConnected = false;
  
#define RSCService BLEUUID((uint16_t)0x1814)
BLECharacteristic RSCMeasurementCharacteristics(BLEUUID((uint16_t)0x2A53), BLECharacteristic::PROPERTY_NOTIFY);
BLECharacteristic sensorPositionCharacteristic(BLEUUID((uint16_t)0x2A5D), BLECharacteristic::PROPERTY_READ);

BLEDescriptor RSCDescriptor(BLEUUID((uint16_t)0x2901));
BLEDescriptor sensorPositionDescriptor(BLEUUID((uint16_t)0x2901));

void(* resetFunc) (void) = 0;//declare reset function at address 0
// call with resetFunc(); if no connection requests in the last hour

class MyServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      _BLEClientConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      _BLEClientConnected = false;
    }
};

void InitBLE() {
  BLEDevice::init("FootpodMimic");
  // CBLE Server
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Do some BLE Setup
  BLEService *pRSC = pServer->createService(RSCService);

  pRSC->addCharacteristic(&RSCMeasurementCharacteristics);
  RSCDescriptor.setValue("Send all your RCSM rubbish here");
  RSCMeasurementCharacteristics.addDescriptor(&RSCDescriptor);
  RSCMeasurementCharacteristics.addDescriptor(new BLE2902());

  pRSC->addCharacteristic(&sensorPositionCharacteristic);

  pServer->getAdvertising()->addServiceUUID(RSCService);

  pRSC->start();

  pServer->getAdvertising()->start();
}

// Interrupt service routine for button press routine
// 18km/h on the treadmill corresponds to a 32ms period so use a digital debounce to ingore anything less than 25ms
//
void IRAM_ATTR button_isr() {  

  unsigned long time_since_last_push;
  
  current_time = millis();
  time_since_last_push = current_time-button1.time_pressed;

  // IT EVENTUALLY GETS TO BE 25!!!! but it still means the de-glitching worked
  if (time_since_last_push> 25 ){
    button1.numberKeyPresses += 1;
    button1.pressed = true;

    button1.previous_time_pressed=button1.time_pressed;
    button1.time_pressed = current_time;
    button1.period=button1.time_pressed - button1.previous_time_pressed;

    //Serial.print("Button pushed - period = ");
    //Serial.println(button1.period/1000.0);
    //Serial.printf("Time since last push %d\n",time_since_last_push);
  }
}

void setup() {

   //Setup the PWM being used for testing
   // setup a PWM output to generate a test signal
   // the number of the LED pin
   const int ledPin = 12;  // 12 corresponds to GPIO12
   const int freq = 100;
   const int ledChannel = 0;
   const int resolution = 8;

  ledcSetup(ledChannel, freq, resolution);
  ledcAttachPin(ledPin, ledChannel);
  ledcWrite(ledChannel, 10); // 10/256 high
  
  // initialize the Serial Monitor
  Serial.begin(115200);
  Serial.println("Booting in setup routine");
  
  // Initialize the Bluetooth 
  InitBLE();

  // For information on the pullups see https://people.eecs.berkeley.edu/~boser/courses/49_sp_2019/N_gpio.html#:~:text=The%20values%20of%20the%20pull,k%CE%A9%20(pull%2Ddown).
  // Pullups are in the range 30k to 80k so a 1k series resistance to give protection should be ok => 100mV input for low and 3.3V for high
  // Pulled up to 3.3V
  pinMode(button1.PIN, INPUT_PULLUP);
  attachInterrupt(button1.PIN, button_isr, FALLING);
      
  delay(2000);  
}

// How often to send the speed e.g. every 0.5 seconds or if has changed by a certain amount
void loop(){
  
  // WHAT HAPPENS IF BUTTON NEVER PRESSED - will keep old speed but instead speed should be zero
  if (button1.pressed) {
      //Serial.printf("Button 1 has been pressed %u times\n", button1.numberKeyPresses);
      button1.pressed = false;

      old_kmph = kmph;
      
      // Period = 322mS => 2km/h, period = 32mS => 18km/h - Speed = distance/time. Optimize to be correct at 10km/h
      kmph= 10.0/11.6*0.626/(button1.period/1000.0);
  }
  
  // Assume treadmill stopped if no button presses in the last 1 second)
  if ((millis()-button1.time_pressed) > 1000) kmph = 0.0;
 
  mps=kmph/3.6;
  inst_speed =mps*256; // WHY 256???????????, inst_speed is a 16 bit integer??? - check the RSCP profile

  // Transmit once per second
  // sending the latest speed estimation
  current_time=millis();
  if ((current_time-last_transmit_time)>1000){

     //Serial.printf("Button 1 period %d\n", button1.period);
     Serial.printf("kmph %5.3f\n", kmph);
     //Serial.print("kmph=");
     //Serial.println(kmph);
     //Serial.printf("mps %5.3f\n", mps);
     //Serial.printf("inst_speed %d\n", inst_speed);
     //Serial.printf("Number of button pushes %d\n",button1.numberKeyPresses);

    //Create the bytearray to send to Zwift via BLE
    //CSCP-cycling speed and cadence profile-described at 
    //RSCP-Running speed and cadence profile-described at 
    byte charArray[10] = {3,
      (unsigned byte)inst_speed, (unsigned byte)(inst_speed >> 8),
      (unsigned byte)inst_cadence,
      (unsigned byte)inst_stride_length, (unsigned byte)(inst_stride_length >> 8),
      (unsigned byte)total_distance, (unsigned byte)(total_distance >> 8), (unsigned byte)(total_distance >> 16), (unsigned byte)(total_distance >> 24)};

    RSCMeasurementCharacteristics.setValue(charArray,10);

    RSCMeasurementCharacteristics.notify();

    sensorPositionCharacteristic.setValue(fakePos, 1);

    last_transmit_time = current_time;
  }
  
} // end of main loop
