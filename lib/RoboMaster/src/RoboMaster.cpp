#include "RoboMaster.hpp"
#include <FlexCAN_T4.h>

// Teensy 4.1 CAN1 を使用（ピン22=CRX1, 23=CTX1）
static FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;

// 内部保持
static MotorState g_state[9];         // index 1..8 を使用
static int16_t    g_target[9] = {0};  // 送信キュー（1..8）
static bool       g_initialized = false;

// サチュレーション（DJIスケール）
static inline int16_t clampCurrent(MotorType t, int32_t v) {
  if (t == MotorType::M2006) {           // C610
    if (v > 10000) v = 10000;
    if (v < -10000) v = -10000;
  } else {                                // M3508 / C620
    if (v > 16384) v = 16384;
    if (v < -16384) v = -16384;
  }
  return (int16_t)v;
}

// CAN受信コールバック
static void onCanReceive(const CAN_message_t &msg) {
  // 0x201..0x208: 各モータのフィードバック
  if (msg.id >= 0x201 && msg.id <= 0x208 && msg.len >= 7) {
    uint8_t id = msg.id - 0x200; // 0x201 -> 1
    if (id >= 1 && id <= 8) {
      g_state[id].enc  = (int16_t)((msg.buf[0] << 8) | msg.buf[1]); // 0..8191
      g_state[id].rpm  = (int16_t)((msg.buf[2] << 8) | msg.buf[3]);
      g_state[id].cur  = (int16_t)((msg.buf[4] << 8) | msg.buf[5]);
      g_state[id].temp = msg.buf[6];
    }
  }
}

static void sendGroup(uint8_t idBase) {
  const uint32_t arb = (idBase <= 4) ? 0x200 : 0x1FF;

  CAN_message_t m;
  m.id = arb;
  m.flags.extended = 0;
  m.len = 8;

  for (int k = 0; k < 4; ++k) {
    uint8_t id = idBase + k;
    int16_t v = g_target[id];
    m.buf[2*k + 0] = (uint8_t)((v >> 8) & 0xFF); // big-endian
    m.buf[2*k + 1] = (uint8_t)(v & 0xFF);
  }
  Can1.write(m);
}

// ---- 公開API実装 ----
void CAN_Init(uint32_t baud, const MotorConfig* configs, size_t num_configs) {
  // 型情報の初期化
  for (uint8_t i = 1; i <= 8; ++i) {
    g_state[i] = MotorState{};
    g_target[i] = 0;
  }
  for (size_t i = 0; i < num_configs; ++i) {
    if (configs[i].id >= 1 && configs[i].id <= 8) {
      g_state[configs[i].id].type = configs[i].type;
    }
  }

  // FlexCAN 初期化
  Can1.begin();
  Can1.setBaudRate(baud);
  Can1.setMaxMB(16);
  Can1.enableFIFO();
  Can1.enableFIFOInterrupt();
  Can1.onReceive(onCanReceive);
  Can1.mailboxStatus();

  g_initialized = true;

  // 立ち上がり安全化
  CAN_ZeroAllAndFlush();
}

void CAN_Poll() {
  if (!g_initialized) return;
  Can1.events(); // 受信処理実行
}

void CAN_SetMotorCurrent(uint8_t id, int16_t value) {
  if (!g_initialized) return;
  if (id < 1 || id > 8) return;
  g_target[id] = clampCurrent(g_state[id].type, value);
}

void CAN_FlushCurrents() {
  if (!g_initialized) return;
  // 1–4
  sendGroup(1);
  // 5–8
  sendGroup(5);
}

bool CAN_GetState(uint8_t id, MotorState& out) {
  if (id < 1 || id > 8) return false;
  out = g_state[id];
  return true;
}

void CAN_ZeroAllAndFlush() {
  for (uint8_t i = 1; i <= 8; ++i) g_target[i] = 0;
  CAN_FlushCurrents();
}
