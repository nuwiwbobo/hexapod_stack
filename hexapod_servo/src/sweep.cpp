#include <dynamixel_sdk/dynamixel_sdk.h>
#include <cstdio>
#include <thread>
#include <chrono>

const char* PORT   = "/dev/ttyUSB0";
const int   BAUD   = 1000000;
const int   ID     = 1;

// AX-18A register addresses
const int ADDR_TORQUE_ENABLE = 24;
const int ADDR_TORQUE_LIMIT  = 34;
const int ADDR_MOVING_SPEED  = 32;
const int ADDR_GOAL_POS      = 30;
const int ADDR_PRESENT_POS   = 36;

int main() {
    auto* port   = dynamixel::PortHandler::getPortHandler(PORT);
    auto* packet = dynamixel::PacketHandler::getPacketHandler(1.0);

    if (!port->openPort())      { puts("Failed to open port"); return 1; }
    port->setBaudRate(BAUD);

    uint8_t err = 0;

    // Low speed and low torque for safety
    packet->write2ByteTxRx(port, ID, ADDR_MOVING_SPEED, 100, &err);  // ~10% speed
    packet->write2ByteTxRx(port, ID, ADDR_TORQUE_LIMIT, 300, &err);  // ~30% torque
    packet->write1ByteTxRx(port, ID, ADDR_TORQUE_ENABLE, 1,  &err);
    printf("Torque ON — starting sweep\n");

    // center=512, ±128 ticks ≈ ±45°
    int targets[] = {512, 640, 512, 384, 512};
    for (int target : targets) {
        packet->write2ByteTxRx(port, ID, ADDR_GOAL_POS, target, &err);
        printf("→ target: %4d  ", target);
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));

        uint16_t pos = 0;
        packet->read2ByteTxRx(port, ID, ADDR_PRESENT_POS, &pos, &err);
        printf("actual: %4d  error: %+d ticks\n", pos, (int)pos - target);
    }

    packet->write1ByteTxRx(port, ID, ADDR_TORQUE_ENABLE, 0, &err);
    printf("Torque OFF\n");
    port->closePort();
}
