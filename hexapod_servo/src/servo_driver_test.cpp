#include <dynamixel_sdk/dynamixel_sdk.h>
#include <cstdio>
#include <cmath>
#include <thread>
#include <chrono>
#include <vector>

// ── inline ServoDriver (no colcon needed) ──────────────────────────────────
struct ServoConfig {
    int id; int ticks; int center;
    double max_radians; double offset; int sign;
};

uint16_t radianToTick(double rad, const ServoConfig& s) {
    double res = (double)s.ticks / s.max_radians;
    int t = (int)std::round(s.center + s.sign * (rad - s.offset) * res);
    if (t < 0)        t = 0;
    if (t > s.ticks)  t = s.ticks;
    return (uint16_t)t;
}

double tickToRadian(uint16_t tick, const ServoConfig& s) {
    double res = (double)s.ticks / s.max_radians;
    return (tick - s.center) / ((double)s.sign * res) + s.offset;
}
// ──────────────────────────────────────────────────────────────────────────

const char* PORT = "/dev/ttyUSB0";
const int   BAUD = 1000000;

const int ADDR_TORQUE_ENABLE = 24;
const int ADDR_TORQUE_LIMIT  = 34;
const int ADDR_MOVING_SPEED  = 32;
const int ADDR_GOAL_POS      = 30;
const int ADDR_PRESENT_POS   = 36;

int main() {
    ServoConfig cfg;
    cfg.id          = 1;
    cfg.ticks       = 1024;
    cfg.center      = 512;
    cfg.max_radians = 2.0 * M_PI;
    cfg.offset      = 0.0;
    cfg.sign        = 1;

    fprintf(stderr, "Creating handlers...\n"); fflush(stderr);
    auto* port   = dynamixel::PortHandler::getPortHandler(PORT);
    auto* packet = dynamixel::PacketHandler::getPacketHandler(1.0);
    if (!port)   { fprintf(stderr, "NULL port handler\n");   return 1; }
    if (!packet) { fprintf(stderr, "NULL packet handler\n"); return 1; }
    fprintf(stderr, "Handlers OK\n"); fflush(stderr);

    if (!port->openPort()) { fprintf(stderr, "openPort failed\n"); return 1; }
    fprintf(stderr, "openPort OK\n"); fflush(stderr);

    if (!port->setBaudRate(BAUD)) { fprintf(stderr, "setBaudRate failed\n"); return 1; }
    fprintf(stderr, "setBaudRate OK\n"); fflush(stderr);

    uint8_t err = 0;
    int rc;

    rc = packet->write2ByteTxRx(port, cfg.id, ADDR_MOVING_SPEED, 100, &err);
    fprintf(stderr, "write MOVING_SPEED rc=%d err=%d\n", rc, err); fflush(stderr);

    rc = packet->write2ByteTxRx(port, cfg.id, ADDR_TORQUE_LIMIT, 300, &err);
    fprintf(stderr, "write TORQUE_LIMIT rc=%d err=%d\n", rc, err); fflush(stderr);

    rc = packet->write1ByteTxRx(port, cfg.id, ADDR_TORQUE_ENABLE, 1, &err);
    fprintf(stderr, "write TORQUE_ENABLE rc=%d err=%d\n", rc, err); fflush(stderr);

    fprintf(stderr, "Torque ON — starting sweep\n"); fflush(stderr);

    std::vector<double> targets = {0.0, M_PI/4, 0.0, -M_PI/4, 0.0};
    for (double target_rad : targets) {
        uint16_t cmd_tick = radianToTick(target_rad, cfg);
        fprintf(stderr, "Sending tick %d for %.4f rad\n", cmd_tick, target_rad); fflush(stderr);

        rc = packet->write2ByteTxRx(port, cfg.id, ADDR_GOAL_POS, cmd_tick, &err);
        fprintf(stderr, "  write GOAL_POS rc=%d err=%d\n", rc, err); fflush(stderr);

        std::this_thread::sleep_for(std::chrono::milliseconds(2000));

        uint16_t act_tick = 0;
        rc = packet->read2ByteTxRx(port, cfg.id, ADDR_PRESENT_POS, &act_tick, &err);
        fprintf(stderr, "  read PRESENT_POS rc=%d err=%d tick=%d\n", rc, err, act_tick); fflush(stderr);

        double act_rad = tickToRadian(act_tick, cfg);
        printf("target=%+.4f rad  cmd=%4d  actual=%4d  got=%+.4f rad  err=%+.4f rad\n",
               target_rad, cmd_tick, act_tick, act_rad, act_rad - target_rad);
        fflush(stdout);
    }

    packet->write1ByteTxRx(port, cfg.id, ADDR_TORQUE_ENABLE, 0, &err);
    fprintf(stderr, "Torque OFF\n");
    port->closePort();
    return 0;
}
