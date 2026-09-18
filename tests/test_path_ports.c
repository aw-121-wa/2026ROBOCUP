#include "disc_task_config.h"
#include "path_ports.h"
#include "path_mission.h"
#include "chassis_control.h"
#include <stdio.h>
#include <string.h>

UART_HandleTypeDef huart4 = {0, 4}, huart6 = {0, 6}, huart7 = {0, 7};
ChassisConfig chassis_config = {
    .wheel_radius_mm = 35, .half_track_mm = 128.5f, .half_wheelbase_mm = 130.5f};
static ChassisState state;
static uint32_t now;
static uint8_t *rx4, *rx7;
static char wire[80];
static unsigned holds, turn_positions;
static bool moving, rfid_init_failure;

uint32_t HAL_GetTick(void) { return now; }
uint32_t PathSession_Create(void) { return 123; }
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *u, uint8_t *b, uint16_t n) {
    if (n != 1) return HAL_ERROR;
    if (u == &huart4) rx4 = b;
    else if (u == &huart7) { rx7 = b; if (rfid_init_failure) return HAL_ERROR; }
    else return HAL_ERROR;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *u, uint8_t *b, uint16_t n) {
    if (u == &huart4) {
        if (n >= sizeof(wire)) return HAL_ERROR;
        memcpy(wire, b, n); wire[n] = 0;
    } else if (u == &huart6) {
        if (n == 13 && b[1] == 0xfd) ++turn_positions;
    } else return HAL_ERROR;
    return HAL_OK;
}
unsigned HAL_GPIO_ReadPin(void *port, uint16_t pin) {
    return ((port == GPIOD && (pin == GPIO_PIN_0 || pin == GPIO_PIN_1)) ||
            (port == GPIOD && pin == GPIO_PIN_10)) ? GPIO_PIN_RESET : GPIO_PIN_SET;
}
const ChassisState *Chassis_GetState(void) { return &state; }
bool Chassis_MotionBusy(void) { return moving; }
bool Chassis_IsSettled(void) { return !moving; }
float Chassis_ContinuousYaw(void) { return 0; }
void Chassis_Hold(void) { moving = false; ++holds; }
bool Chassis_Move(float x, float y, float v, float a, float d) {
    (void)x; (void)y; (void)v; (void)a; (void)d;
    if (!state.armed || moving) return false;
    moving = true; return true;
}
bool Chassis_Rotate(float deg) { return Chassis_Move(deg, 0, 1, 1, 1); }
bool Chassis_Body(float x, float y, float w) {
    (void)x; (void)y; (void)w; moving = true; return state.armed;
}
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *u) {
    return (u == &huart4 || u == &huart7) ? HAL_OK : HAL_ERROR;
}

#define CHECK(x) do { if (!(x)) { printf("FAIL %d: %s\n",__LINE__,#x); return 1; } } while(0)
static void tick(void) { now += 5; PathPorts_Tick(); }
static void reply(const char *line) {
    for (const char *p = line; *p; ++p) { *rx4 = (uint8_t)*p; PathPorts_RxComplete(&huart4); }
}
static void raw(uint8_t value) { *rx7 = value; PathPorts_RxComplete(&huart7); }
static void make_id(uint32_t value, uint8_t out[12]) {
    uint8_t f[12] = {4,12,2,32,0,4,0,(uint8_t)(value>>24),(uint8_t)(value>>16),
                     (uint8_t)(value>>8),(uint8_t)value,0};
    f[11] = 255; for (unsigned i=0;i<11;i++) f[11] ^= f[i];
    memcpy(out, f, 12);
}
static void id(uint32_t value) { uint8_t f[12]; make_id(value, f); for (unsigned i=0;i<12;i++) raw(f[i]); }

static int start_disc(void) {
    PathPorts_Init();
    CHECK(PathPorts_Ping()); tick(); CHECK(!strcmp(wire, "PING\r\n"));
    reply("PONG\r\n"); tick(); CHECK(path_diagnostics.accepted_ids == 1);
    state.armed = true;
    id(0x01020304); tick(); /* Capture is closed before a disc action gate. */
    CHECK(PathPorts_Disc()); tick(); reply("PONG\r\n"); tick(); tick();
    CHECK(!strcmp(wire, "DISC_START\r\n"));
    reply("DISC_ACK\r\n"); tick();
    CHECK(path_diagnostics.rfid_count == 0);
    CHECK(path_diagnostics.disc_waiting_rfid == 0);
    CHECK(path_diagnostics.disc_action_allowed == 1);
    return 0;
}
static int action_done(uint8_t index) {
    char line[32]; snprintf(line, sizeof(line), "DISC_ACTION_DONE %u\r\n", index);
    reply(line); tick();
    CHECK(path_diagnostics.disc_action_done_index == index);
    CHECK(path_diagnostics.disc_waiting_rfid == index);
    CHECK(path_diagnostics.disc_action_allowed == 0);
    return 0;
}
static int scan(uint8_t index, uint32_t uid) {
    id(uid); tick(); tick();
    char expected[32]; snprintf(expected, sizeof(expected), "DISC_RFID_OK %u\r\n", index);
    CHECK(!strcmp(wire, expected));
    CHECK(path_diagnostics.disc_rfid_confirmed_index == index);
    CHECK(path_diagnostics.disc_waiting_rfid == 0);
    CHECK(path_diagnostics.disc_action_allowed == (index < DISC_REQUIRED_RFID_COUNT));
    return 0;
}
static int complete_gate(uint8_t index, uint32_t uid) {
    CHECK(action_done(index) == 0); CHECK(scan(index, uid) == 0); return 0;
}

int main(int argc, char **argv) {
    CHECK(argc == 2);
    rfid_init_failure = !strcmp(argv[1], "rfid_init");
    CHECK(start_disc() == 0);

    if (!strcmp(argv[1], "frames")) {
        CHECK(action_done(1) == 0);
        uint8_t f[12]; make_id(0x4596b78a, f);
        for (unsigned i=0;i<6;i++) raw(f[i]);
        tick(); CHECK(path_diagnostics.rfid_count == 0);
        for (unsigned i=6;i<12;i++) raw(f[i]);
        tick(); tick();
        CHECK(path_diagnostics.rfid_count == 1 && !strcmp(wire, "DISC_RFID_OK 1\r\n"));
    } else if (!strcmp(argv[1], "rfid")) {
        CHECK(complete_gate(1, 3) == 0);
        CHECK(action_done(2) == 0); id(3); tick();
        CHECK(path_diagnostics.rfid_count == 1 && path_diagnostics.disc_waiting_rfid == 2);
        CHECK(scan(2, 1) == 0);
        id(9); tick(); CHECK(path_diagnostics.rfid_count == 2); /* closed: no preauthorization */
        CHECK(complete_gate(3, 9) == 0);
        CHECK(complete_gate(4, 2) == 0);
        CHECK(complete_gate(5, 7) == 0);
        uint32_t saved[8] = {0}; const uint32_t expected[] = {3,1,9,2,7};
        CHECK(PathPorts_CopyIds(saved, 8) == 5);
        CHECK(!memcmp(saved, expected, sizeof(expected)));
        reply("DISC_DONE\r\n"); tick(); CHECK(path_diagnostics.result == PATH_DONE);
    } else if (!strcmp(argv[1], "overflow")) {
        CHECK(action_done(1) == 0); id(11); id(22); tick(); tick();
        CHECK(path_diagnostics.rfid_count == 1);
        CHECK(action_done(2) == 0); tick();
        CHECK(path_diagnostics.rfid_count == 1 && path_diagnostics.disc_waiting_rfid == 2);
        CHECK(scan(2, 22) == 0);
    } else if (!strcmp(argv[1], "stop") || !strcmp(argv[1], "pending_stop")) {
        CHECK(action_done(1) == 0); PathPorts_Cancel(); tick();
        CHECK(path_diagnostics.result == PATH_CANCELED);
        CHECK(path_diagnostics.disc_waiting_rfid == 0 && path_diagnostics.disc_action_allowed == 0);
        CHECK(!strcmp(wire, "DISC_CANCEL\r\n"));
        id(5); tick(); CHECK(path_diagnostics.rfid_count == 0);
    } else if (!strcmp(argv[1], "timeout")) {
        CHECK(action_done(1) == 0); now += DISC_TASK_TIMEOUT_MS; tick();
        CHECK(path_diagnostics.result == PATH_TIMEOUT);
        CHECK(path_diagnostics.disc_waiting_rfid == 0 && path_diagnostics.disc_action_allowed == 0);
    } else if (!strcmp(argv[1], "late")) {
        for (uint8_t i=1;i<=5;i++) CHECK(complete_gate(i, i) == 0);
        now += DISC_TASK_TIMEOUT_MS; tick(); reply("DISC_DONE\r\n"); tick();
        CHECK(path_diagnostics.result == PATH_TIMEOUT);
    } else if (!strcmp(argv[1], "error")) {
        CHECK(action_done(1) == 0); reply("DISC_ERROR\r\n"); tick();
        CHECK(path_diagnostics.result == PATH_ERROR);
        CHECK(path_diagnostics.disc_waiting_rfid == 0 && path_diagnostics.disc_action_allowed == 0);
    } else if (!strcmp(argv[1], "link_fault")) {
        CHECK(action_done(1) == 0); id(44); tick(); /* RFID_OK is queued, not sent. */
        CHECK(path_diagnostics.rfid_count == 1);
        PathPorts_Error(&huart4); tick();
        CHECK(strcmp(wire, "DISC_RFID_OK 1\r\n") != 0);
        CHECK(path_diagnostics.result == PATH_ERROR);
        CHECK(path_diagnostics.disc_action_allowed == 0);
    } else if (!strcmp(argv[1], "stationary")) {
        for (uint8_t i=1;i<=5;i++) CHECK(complete_gate(i, 100+i) == 0);
        reply("DISC_DONE\r\n"); tick(); CHECK(path_diagnostics.result == PATH_DONE);
        for (unsigned i=0;i<1000;i++) tick();
        CHECK(turn_positions == 5);
    } else if (!strcmp(argv[1], "missing")) {
        for (uint8_t i=1;i<=4;i++) CHECK(complete_gate(i, i) == 0);
        reply("DISC_DONE\r\n"); tick(); CHECK(path_diagnostics.result == PATH_ERROR);
    } else if (!strcmp(argv[1], "rfid_fault") || !strcmp(argv[1], "rfid_init") || !strcmp(argv[1], "zero")) {
        if (!strcmp(argv[1], "rfid_fault")) PathPorts_Error(&huart7);
        reply("DISC_DONE\r\n"); tick(); CHECK(path_diagnostics.result == PATH_ERROR);
        CHECK(path_diagnostics.rfid_count == 0 && path_diagnostics.disc_waiting_rfid == 0);
        if (!strcmp(argv[1], "rfid_fault")) CHECK(path_diagnostics.rfid_fault & 16);
        if (!strcmp(argv[1], "rfid_init")) CHECK(path_diagnostics.rfid_fault & 1);
    } else {
        CHECK(0);
    }
    puts("ZHY adapter test passed"); return 0;
}
