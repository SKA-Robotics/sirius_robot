#include <SPI.h>
#include <mcp2515.h>

const uint8_t canId = 0x30;

MCP2515 mcp2515(8);
struct can_frame canMsg;

#define LED_PIN 6

void setup(void) {
    mcp2515.reset();
    mcp2515.setBitrate(CAN_100KBPS, MCP_8MHZ);
    mcp2515.setNormalMode();

    pinMode(LED_PIN, OUTPUT);
}

void loop(void) {
    for (char i = 0; i < 4; i++) {
        int port;
        switch(i) {
            case 0:
                port = A0;
                break;
            case 1:
                port = A1;
                break;
            case 2:
                port = A2;
                break;
            case 3:
                port = A3;
                break;
        }
        long int value = 0;
        for (char j = 0; j < 100; j++) {
            value += analogRead(port);
        }
        value /= 100;

        char dataOffset = i << 1;

        canMsg.data[dataOffset] = value >> 8;
        canMsg.data[dataOffset + 1] = value & 0xFF;
    }
    canMsg.can_id  = (canId << 5);
    canMsg.can_dlc = 8;

    mcp2515.sendMessage(&canMsg);
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(500);
}
