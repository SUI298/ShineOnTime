#include <ArduinoBLE.h>
#include <RTCZero.h>

//IO
#define LED_PIN 10
#define POTENTIOMETER_PIN A0

//OTHER
#define MIN_BRIGHTNESS 16
#define BRIGHTNESS_INCREMENT_DELAY_x10ms 586 //Time to turn the light fully on = t_delay x n_increments = 586x10ms x 2^8 = 25 minutes
#define BLINK_PERIOD_x10ms 20 //200ms
#define NUM_BLINKS 4 //even number
#define LOOP_PERIOD_MS 10

#define CURRENT_TIME_UUID "67112ab4-c0e4-11ed-afa1-0242ac120002"
#define ALARM_TIME_UUID "67112e06-c0e4-11ed-afa1-0242ac120002"
#define HOUR_UUID "67113022-c0e4-11ed-afa1-0242ac120002"
#define MINUTE_UUID "67113d88-c0e4-11ed-afa1-0242ac120002"
#define SECOND_UUID "671145d0-c0e4-11ed-afa1-0242ac120002"

class RunningAverage{
  #define N 100
  uint8_t values[N];
  uint8_t index;
  public:
  RunningAverage(uint8_t initial_val = 0){
    index = 0;
    for(int i = 0; i < N; i++){
      values[i] = initial_val;
    }
  }
  void addVal(uint8_t val){
    values[index++] = val;
    index %= N;    
  }
  uint8_t value(){
    uint16_t sum = 0;
    for(int i = 0; i < N; i++){
      sum += values[i];
    }
    return sum/N;
  }

};

BLEService alarmService(ALARM_TIME_UUID);
BLEService timeService(CURRENT_TIME_UUID);

BLEUnsignedCharCharacteristic alarmHourCharacteristic(HOUR_UUID, BLERead | BLEWrite);
BLEUnsignedCharCharacteristic alarmMinuteCharacteristic(MINUTE_UUID, BLERead | BLEWrite);
BLEUnsignedCharCharacteristic timeHourCharacteristic(HOUR_UUID, BLERead | BLEWrite);
BLEUnsignedCharCharacteristic timeMinuteCharacteristic(MINUTE_UUID, BLERead | BLEWrite);
BLEUnsignedCharCharacteristic timeSecondCharacteristic(SECOND_UUID, BLERead | BLEWrite);

BLEDevice central;

RTCZero clock;

RunningAverage pot_avg;
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
  uint8_t potentiometer_val = pot_avg.value();
  switch(state){
    default:
    case State::CTRL_BY_POTENTIOMETER:
      brightness = constrain(map(potentiometer_val, MIN_BRIGHTNESS, 0xFF, 0, 0xFF),0 ,0xFF); //DO
      if(alarm){ //EVENT           
        Serial.println("ALARM ALARM ALARM");
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
      if( potentiometer_val > MIN_BRIGHTNESS){ //EVENT
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
  // put your main code here, to run repeatedly:
  brightnessControllSM();
  pot_avg.addVal(analogRead(POTENTIOMETER_PIN)); 
  analogWrite(LED_PIN, brightness);
  Serial.println(millis());

  if(!connected){
    central = BLE.central();
  }
  if(central){
    if(!connected){
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
        Serial.println("alarm set.");
        clock.setAlarmMinutes(alarmMinuteCharacteristic.value());
        clock.setAlarmSeconds(0);
        clock.enableAlarm(clock.MATCH_HHMMSS);
        blink = 2;
      }
    }else{
      connected = false;
    }
  }

  delay(1);
  while(millis()%LOOP_PERIOD_MS); //wait till next loop
}
