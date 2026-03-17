#include <Arduino.h>
#include <SPI.h>
#include <mcp2515.h>
#include "HX711.h"


const int LOADCELL_DOUT_PIN = 2;
const int LOADCELL_SCK_PIN  = 3;

HX711 scale;

MCP2515 mcp2515(10);
struct can_frame canMsg;

// TODO: adjust calibration factor
float calibration_factor = -459.542;

void setup() {
  Serial.begin(57600);
  Serial.println("Initializing scale and CAN");

  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  scale.set_scale(calibration_factor);
  scale.tare();

  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_16MHZ);
  mcp2515.setNormalMode();

  Serial.println("Weight sender ready");
}

void loop() {
  // average of 10 readings
  float weight = scale.get_units(10);

  canMsg.can_id  = 0x70;  // TODO: adjust can_id
  canMsg.can_dlc = 4;

  uint8_t* bytes = reinterpret_cast<uint8_t*>(&weight); 
  for (int i = 0; i < 4; i++) {
    canMsg.data[i] = bytes[i];
  }

  if (mcp2515.sendMessage(&canMsg) != MCP2515::ERROR_OK) {
    Serial.println("CAN send failed!");
  }

  delay(500);  // send every 0.5s
}
