#include <SPI.h>
#include <mcp2515.h>
#include <Servo.h> 

Servo servo;

const uint8_t canId = 0x31;

MCP2515 mcp2515(8);
struct can_frame canMsg;
struct can_frame measureMsg;
constexpr char kMeasurementFrameId = 0x01;
constexpr char kMeasurementRateDivisor = 100;

#define LED_PIN 6
#define SERVO_PIN 9
#define MEASUREMENT_PIN A6

void setup(void) {
    //Serial.begin(9600);
    //delay(3000);
    //Serial.println("started");
  
    mcp2515.reset();
    mcp2515.setBitrate(CAN_100KBPS, MCP_8MHZ);
    mcp2515.setNormalMode();

    
    //servo.attach(SERVO_PIN);
    //servo.writeMicroseconds(1500);
    pinMode(6, OUTPUT);
    pinMode(A6, OUTPUT);
}

char timeoutCount = 0;
char measurementRateDivisorCount = 0;
long int measurementValue = 0;

void loop(void) {
    if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
        if ((canMsg.can_id >> 5) == canId) {
            int micro = (canMsg.data[0] << 8) + canMsg.data[1];
            
            //Serial.println(micro);
            servo.attach(SERVO_PIN);
            servo.writeMicroseconds(micro);
            timeoutCount = 0;
            digitalWrite(6, !digitalRead(6));
        }  
    }
    
    if (timeoutCount < 51) timeoutCount++;
    if (timeoutCount == 50) {
        servo.detach();
        Serial.println("detach");
        digitalWrite(SERVO_PIN, LOW);
    }

    ++measurementRateDivisorCount;
    measurementValue += analogRead(A6);

    if (measurementRateDivisorCount >= kMeasurementRateDivisor) {
      measurementValue /= kMeasurementRateDivisor;
      measureMsg.can_id = (canId << 5) | kMeasurementFrameId;
      measureMsg.can_dlc = 2;
      measureMsg.data[0] = (measurementValue >> 8) & 0xFF;
      measureMsg.data[1] = measurementValue & 0xFF;
      mcp2515.sendMessage(&measureMsg);
      measurementValue = 0;
      measurementRateDivisorCount = 0;
    }

    delay(10);
}
