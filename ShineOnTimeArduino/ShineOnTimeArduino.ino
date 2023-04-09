#include <ArduinoBLE.h>
#include <RTCZero.h>

//IO
#define LED_PIN 10
#define POTENTIOMETER_PIN A0
#define POTENTIOMETER_SAMPLE_SIZE 8

//OTHER
#define MIN_BRIGHTNESS 1
#define BRIGHTNESS_INCREMENT_DELAY_x10ms 586 //Time to turn the light fully on = t_delay x n_increments = 586x10ms x 2^8 = 25 minutes
#define BLINK_PERIOD_x10ms 20 //200ms
#define NUM_BLINKS 4 //even number
#define LOOP_PERIOD_MS 10

#define CURRENT_TIME_UUID "67112ab4-c0e4-11ed-afa1-0242ac120002"
#define ALARM_TIME_UUID "67112e06-c0e4-11ed-afa1-0242ac120002"
#define HOUR_UUID "67113022-c0e4-11ed-afa1-0242ac120002"
#define MINUTE_UUID "67113d88-c0e4-11ed-afa1-0242ac120002"
#define SECOND_UUID "671145d0-c0e4-11ed-afa1-0242ac120002"
#define ALARM_ON_UUID "67114fa0-c0e4-11ed-afa1-0242ac120002"


BLEService alarmService(ALARM_TIME_UUID);
BLEService timeService(CURRENT_TIME_UUID);

BLEUnsignedCharCharacteristic alarmHourCharacteristic(HOUR_UUID, BLERead | BLEWrite);
BLEUnsignedCharCharacteristic alarmMinuteCharacteristic(MINUTE_UUID, BLERead | BLEWrite);
BLEBoolCharacteristic         alarmOnCharacteristic(ALARM_ON_UUID, BLERead | BLEWrite);
BLEUnsignedCharCharacteristic timeHourCharacteristic(HOUR_UUID, BLERead | BLEWrite);
BLEUnsignedCharCharacteristic timeMinuteCharacteristic(MINUTE_UUID, BLERead | BLEWrite);
BLEUnsignedCharCharacteristic timeSecondCharacteristic(SECOND_UUID, BLERead | BLEWrite);

BLEDevice central;

RTCZero clock;

uint16_t pot_avg;
uint8_t brightness;

//flags
bool alarm = false;
uint8_t blink = 0;
bool connected = false;


//State-machine for controlling LED-Brightness
void brightnessControllSM(){
  enum State {CTRL_BY_POTENTIOMETER, CTRL_BY_ALARM, BLINK};
  static State state = CTRL_BY_POTENTIOMETER;
  static uint8_t num_blinks = 0;
  static uint8_t blink_brightness = 0xFF;
  uint8_t user_brightness = pot_avg*pot_avg/0xFF; //non linear scale
  switch(state){
    default:
    case State::CTRL_BY_POTENTIOMETER:
      brightness = user_brightness; //DO
      if(alarm){ //EVENT           
        brightness = 0; //ACTION
        state = State::CTRL_BY_ALARM;
      }
      if(blink){
        state = State::BLINK;
        blink_brightness = brightness > 0 ? brightness : 0xFF;
      }
      break;
    case State::CTRL_BY_ALARM:
      if(     !((millis()/10)%BRIGHTNESS_INCREMENT_DELAY_x10ms) 
          &&  brightness < 0xFF ){
        brightness++;
      }
      if( (user_brightness > MIN_BRIGHTNESS) || !alarm){ //EVENT
        state = State::CTRL_BY_POTENTIOMETER;
        //
        alarm = false;
      }
      if(blink){
        state = State::BLINK;
        blink_brightness = brightness > 0 ? brightness : 0xFF;
      }
      break;
    case State::BLINK:
      if(!((millis()/10)%BLINK_PERIOD_x10ms)){
        brightness = brightness > 0 ? 0 : blink_brightness;
        blink--;
      }
      if(blink == 0){
          state = alarm ? State::CTRL_BY_ALARM : CTRL_BY_POTENTIOMETER;
      }
      break;
  }

}
void alarmMatchISR(){
  alarm = true;
  clock.disableAlarm();
  alarmOnCharacteristic.writeValue(false);
}

void setup() {
  // put your setup code here, to run once:
  pinMode(LED_PIN,OUTPUT);
  analogReadResolution(8);
  Serial.begin(9600);

  clock.begin();
  clock.attachInterrupt(alarmMatchISR);

  if (!BLE.begin()) {
    Serial.println("starting Bluetooth® Low Energy module failed!");

    while (1);
  }

  // set advertised local name and service UUID:
  BLE.setLocalName("ShineOnTime");
  BLE.setAdvertisedService(alarmService);

  // add the characteristic to the service
  alarmService.addCharacteristic(alarmHourCharacteristic);
  alarmService.addCharacteristic(alarmMinuteCharacteristic);
  alarmService.addCharacteristic(alarmOnCharacteristic);

  timeService.addCharacteristic(timeHourCharacteristic);
  timeService.addCharacteristic(timeMinuteCharacteristic);
  timeService.addCharacteristic(timeSecondCharacteristic);

  // add service
  BLE.addService(alarmService);
  BLE.addService(timeService);
  
 // start advertising
  BLE.advertise();

  Serial.println("ShineOnTime BLE Alarm - setup complete");

}

void loop() {

  brightnessControllSM();
  analogWrite(LED_PIN, brightness);

  pot_avg = 0;
  for(int i = 0; i < POTENTIOMETER_SAMPLE_SIZE; i++){
    pot_avg += analogRead(POTENTIOMETER_PIN);
  }
  pot_avg /= POTENTIOMETER_SAMPLE_SIZE;

  if(!connected){
    central = BLE.central();
  }
  if(central){
    if(!connected){ //connection is new
      connected = true;
      Serial.print("Connected to central: ");
      // print the central's MAC address:
      Serial.println(central.address());
      // Make the Light blink to show 
      blink = 6;
    }
    if(central.connected()){
      if(timeHourCharacteristic.written()){
        clock.setHours(timeHourCharacteristic.value());
      }
      if(timeMinuteCharacteristic.written()){
        clock.setMinutes(timeMinuteCharacteristic.value());
      }
      if(timeSecondCharacteristic.written()){
        clock.setSeconds(timeSecondCharacteristic.value());
      }
      if(alarmHourCharacteristic.written()){
        clock.setAlarmHours(alarmHourCharacteristic.value());
      }
      if(alarmMinuteCharacteristic.written()){
        clock.setAlarmMinutes(alarmMinuteCharacteristic.value());
      }
      if(alarmOnCharacteristic.written()){
        //Alarm enabled
        if(alarmOnCharacteristic.value()){
          Serial.println("alarm set.");
          clock.setAlarmSeconds(0);
          clock.enableAlarm(clock.MATCH_HHMMSS);
          blink = 2;
        }else{
          //Alarm disabled
          clock.disableAlarm();
          alarm = false;
          blink = 4;
        }
      }
    }else{
      connected = false;
    }
  }
  while(millis()%LOOP_PERIOD_MS); //wait till next loop
}
