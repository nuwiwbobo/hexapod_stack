// ping_read.cpp — drop in hexapod_servo/src/ or compile standalone
#include <dynamixel_sdk/dynamixel_sdk.h>
#include <cstdio>

int main() {
  const char* PORT    = "/dev/ttyUSB0";
  const int   BAUD    = 1000000;
  const int   ID      = 1;       // change to your servo ID
  const int   ADDR_PRESENT_POS = 36;  // AX register map

  auto* portHandler   = dynamixel::PortHandler::getPortHandler(PORT);
  auto* packetHandler = dynamixel::PacketHandler::getPacketHandler(1.0);

  if (!portHandler->openPort())  { puts("Failed to open port"); return 1; }
  portHandler->setBaudRate(BAUD);

  // Ping
  uint8_t err = 0;
  uint16_t model = 0;
  int rc = packetHandler->ping(portHandler, ID, &model, &err);
  printf("Ping: rc=%d  model=%d  err=%d\n", rc, model, err);
  if (rc != COMM_SUCCESS) return 1;

  // Read present position (2 bytes)
  uint16_t pos = 0;
  rc = packetHandler->read2ByteTxRx(portHandler, ID, ADDR_PRESENT_POS, &pos, &err);
  printf("Present position: %d ticks  (center=512)\n", pos);

  portHandler->closePort();
}

// AX-18A key registers
const int ADDR_TORQUE_ENABLE = 24;
const int ADDR_TORQUE_LIMIT  = 34;  // 0-1023
const int ADDR_GOAL_POS      = 30;
const int ADDR_PRESENT_POS   = 36;

// Enable with low torque limit (20%)
packetHandler->write2ByteTxRx(portHandler, ID, ADDR_TORQUE_LIMIT, 200, &err);
packetHandler->write1ByteTxRx(portHandler, ID, ADDR_TORQUE_ENABLE, 1,   &err);

// Sweep center(512) ± 128 ticks ≈ ±45°
for (int target : {512, 640, 512, 384, 512}) {
    packetHandler->write2ByteTxRx(portHandler, ID, ADDR_GOAL_POS, target, &err);
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));
    uint16_t pos = 0;
    packetHandler->read2ByteTxRx(portHandler, ID, ADDR_PRESENT_POS, &pos, &err);
    printf("Target: %d  Actual: %d\n", target, pos);
}

packetHandler->write1ByteTxRx(portHandler, ID, ADDR_TORQUE_ENABLE, 0, &err);
