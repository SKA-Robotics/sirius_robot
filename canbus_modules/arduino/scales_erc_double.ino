#include <Arduino.h>
#include <SPI.h>
#include <mcp2515.h>
#include "HX711.h"


const int LOADCELL_DOUT_PIN = 2;
const int LOADCELL_SCK_PIN  = 3;

const int LOADCELL_DOUT_PIN_2 = 2;
const int LOADCELL_SCK_PIN_2  = 3;

HX711 scale;
HX711 scale2;

MCP2515 mcp2515(10);
struct can_frame canMsg;
struct can_frame canMsg2;

// TODO: adjust calibration factor
float calibration_factor = -459.542;
float calibration_factor2 = -459.542;

void setup() {
  Serial.begin(57600);
  Serial.println("Initializing scale and CAN");

  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  scale.set_scale(calibration_factor);
  scale.tare();

  scale2.begin(LOADCELL_DOUT_PIN2, LOADCELL_SCK_PIN2);
  scale2.set_scale(calibration_factor2);
  scale2.tare();

  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_16MHZ);
  mcp2515.setNormalMode();

  Serial.println("Weight sender ready");
}

void loop() {
  // average of 10 readings
  float weight = scale.get_units(10);
  float weight2 = scale2.get_units(10);


  canMsg.can_id  = 0x71;  // TODO: adjust can_id
  canMsg.can_dlc = 4;

  canMsg2.can_id  = 0x72;  // TODO: adjust can_id
  canMsg2.can_dlc = 4;

  uint8_t* bytes = reinterpret_cast<uint8_t*>(&weight); 
  for (int i = 0; i < 4; i++) {
    canMsg.data[i] = bytes[i];
  }

  uint8_t* bytes2 = reinterpret_cast<uint8_t*>(&weight2); 
  for (int i = 0; i < 4; i++) {
    canMsg2.data[i] = bytes2[i];
  }

  if (mcp2515.sendMessage(&canMsg) != MCP2515::ERROR_OK) {
    Serial.println("CAN send failed!");
  }
  if (mcp2515.sendMessage(&canMsg2) != MCP2515::ERROR_OK) {
    Serial.println("CAN2 send failed!");
  }

  Serial.print("raw: ");
  Serial.print(scale.get_value(10));
  Serial.print("  g: ");
  Serial.println(scale.get_units(10));


  Serial.print("2 raw: ");
  Serial.print(scale2.get_value(10));
  Serial.print(" 2 g: ");
  Serial.println(scale2.get_units(10));

  delay(500);  // send every 0.5s
}
