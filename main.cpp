#define LED_BUILDIN 2
#define SERVO_PIN 18
#define MOTOR_PIN 23

#define softSpacing 60 //time [ms] between soft start/end steps
#define constatntSpeedTime 5000 //const speed for time [ms]

#include <Wire.h>
#include <BluetoothSerial.h>
#include <AS5600.h>

BluetoothSerial SerialBT;
AS5600 as5600;   //  use default Wire

void softStart();
void softEnd();
int BT_read_speed();
void softStart(int targetSpeed, int stepsSpacing);
void softEnd(int initSpeed, int stepsSpacing);
void constant_Speed(int speed, unsigned long timeOfConstSpeed);

void setup(){
    //Serial init
    Serial.begin(115200);
    //BT init
    SerialBT.begin("ALO_Synergia");

    pinMode(LED_BUILDIN, OUTPUT);
  	digitalWrite(LED_BUILDIN, HIGH);
    pinMode(21, OUTPUT);
	  digitalWrite(19, LOW);

    //servo PWM
  	ledcSetup(1, 50, 8);
	  ledcAttachPin(SERVO_PIN, 1);

    //dc motor PWM
	  ledcSetup(2, 10000, 8);
  	ledcAttachPin(MOTOR_PIN, 2);

    //AS5600 encoder settings
	  Wire.begin();
	  as5600.begin(4);  //  set direction pin.
	  as5600.setDirection(AS5600_CLOCK_WISE);  //  default, just be explicit
	  int as5600Status = as5600.isConnected();
	  Serial.print("Connect: ");
	  Serial.println(as5600Status);
	  delay(1000);
}

void loop(){
    //wait for BT connection
    while(!SerialBT.connected()){}
    Serial.print("Connected to ");
    Serial.println(SerialBT.getBtAddressString());

    //send info to user
    SerialBT.println("Wprowadz predkosc w deg/s aby rozpoczac (max 1200):");
    //sequence always starts with getting speed via BT, wait till != 0
    int speed = 0;
    while(speed==0){
        speed = BT_read_speed();
    }
    Serial.print("Received speed: ");
    Serial.println(speed);

    softStart(speed, softSpacing);
    constant_Speed(speed, constatntSpeedTime);
    softEnd(speed, softSpacing);

}

int BT_read_speed(){ 
  String messageReceived = "";
  char c = 'a';
  int speed = 0;
  while (SerialBT.available()) {
    c = SerialBT.read();
    if ((c != '\n')&&(c != '\r')){
      messageReceived += String(c);
    }
    delay(2);
  }
  if(messageReceived!=""){
    //Serial.println(messageReceived);
    speed = messageReceived.toInt();
    //Serial.println(speed);
  }
  if(speed>1200){
    speed = 1200;
  }
  if(speed<0){
    speed = 0;
  }
  return speed; //in range from 0 to 1200
}

void softStart(int targetSpeed, int stepsSpacing){

  //convert speed to required range - from 150 (motor stop, 0 deg/s) to 255 (motor full speed, 1200 deg/s)
  //it's estimated value, wrong due to non-linearity of motor characteristics but close enough to start, it'll be regulated then
  targetSpeed = map(targetSpeed, 0, 1200, 150, 255);

    unsigned long last_millis = millis();
  	for (int i = 150; i <= targetSpeed; i++) {
		if (millis() - last_millis >= stepsSpacing) {
			ledcWrite(2, i);
      Serial.println(i);
			last_millis = millis();
		}
		else {
			i--;
		}	
		Serial.print(i);
		Serial.print("\t");
		as5600_test();
	}
}

void softEnd(int initSpeed, int stepsSpacing){
  //same as in softStart, close enough estimation
  initSpeed = map(initSpeed, 0, 1200, 150, 255);
  unsigned long last_millis = millis();
for (int i = initSpeed; i >= 150; i--) {

		if (millis() - last_millis >= stepsSpacing) {
			ledcWrite(2, i);
			last_millis = millis();
      //Serial.println(i);
		}
		else {
			i++;
		}
	
		Serial.print(i);
		Serial.print("\t");
		as5600_test();
		
	}
}

void constant_Speed(int speed, unsigned long timeOfConstSpeed){
  unsigned long previousMillis = millis();
  while(millis()-previousMillis<timeOfConstSpeed){
    //int x = P_regulator(int speed);
    //P regulator will return value of required ledc argument, for now its simply map, regulator is to be done
    int x = map(speed, 0, 1200, 150, 255);
    ledcWrite(2, x);
    as5600_test();
    //Serial.println(x);
  }
}

void as5600_test() {
	uint16_t last_angle = 0;
	unsigned long last_millis = 0;
	//  Serial.print(millis());
    //  Serial.print("\t");
	Serial.print(as5600.readAngle());
	Serial.print("\t");
	
	//uint16_t angular_velocity = (as5600.readAngle()-

	//Serial.println(as5600.rawAngle());
	Serial.println(as5600.getAngularSpeed());
	//  Serial.println(as5600.rawAngle() * AS5600_RAW_TO_DEGREES);
}

//for debug purposes:

void esp_test_blink_led() {

	int delay_time = 1000;

	delay(delay_time);
	digitalWrite(LED_BUILDIN, HIGH);
	delay(delay_time);
	digitalWrite(LED_BUILDIN, LOW);
}
void servo_test_sweep() {
	int dutyCycle = 0;

	for (dutyCycle = 2; dutyCycle <= 32; dutyCycle++) {

		ledcWrite(1, dutyCycle);
		delay(100);
		Serial.println(dutyCycle);
	}
	for (dutyCycle = 32; dutyCycle >= 2; dutyCycle--) {
		ledcWrite(1, dutyCycle);
		delay(100);
		Serial.println(dutyCycle);
	}
}



