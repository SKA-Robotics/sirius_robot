#include <Wire.h>
#include <MCP342x.h>
#include <SPI.h>
#include <mcp2515.h>

const uint8_t canId = 0x30;


// 0x68 is the default address for all MCP342x devices
uint8_t address = 0x6E;
MCP342x adc = MCP342x(address);

MCP2515 mcp2515(8);
struct can_frame canMsg;

#define LED_PIN 6

void setup(void) {
    Wire.begin();
    
    MCP342x::generalCallReset();
    delay(1); // MC342x needs 300us to settle, wait 1ms
    
    // Check device present
    Wire.requestFrom(address, (uint8_t)1);
    if (!Wire.available()) {
        while (1)
        ;
    }

    mcp2515.reset();
    mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
    mcp2515.setNormalMode();
}

void loop(void) {
    for (char i = 0; i < 4; i++) {
        long int value;
        MCP342x::Config status;
        MCP342x::Channel channel;

        switch(i) {
            case 0:
                channel = MCP342x::channel1;
                break;
            case 1:
                channel = MCP342x::channel2;
                break;
            case 2:
                channel = MCP342x::channel3;
                break;
            case 3:
                channel = MCP342x::channel4;
                break;
        }
        uint8_t err = adc.convertAndRead(channel, MCP342x::oneShot,
                MCP342x::resolution18, MCP342x::gain1,
                1000000, value, status);

        if (err) value = 0xFFFFFFFF;

        char dataOffset = (i & 1) << 2;

        canMsg.data[dataOffset] = value >> 24;
        canMsg.data[dataOffset + 1] = (value >> 16) & 0xFF;
        canMsg.data[dataOffset + 2] = (value >> 8) & 0xFF;
        canMsg.data[dataOffset + 3] = value & 0xFF;

        if (dataOffset == 4) {
            canMsg.can_id  = (canId << 5) | (i >> 1);
            canMsg.can_dlc = 8;

            mcp2515.sendMessage(&canMsg);
        }
    }
}
