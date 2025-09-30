#pragma once
#include <Arduino.h>
#include <stdint.h>
#include <stddef.h>

enum class MotorType : uint8_t { M2006, M3508 };

struct MotorConfig {
  uint8_t id;        // 1..8
  MotorType type;    // M2006 or M3508
};

// モータの受信状態（0x201..0x208 フィードバック）
struct MotorState {
  int16_t enc = 0;   // 0..8191 (raw)
  int16_t rpm = 0;   // signed
  int16_t cur = 0;   // signed (feedback)
  uint8_t temp = 0;  // ℃
  MotorType type = MotorType::M3508;
};

// ---- 公開API ----
// 初期化：ボーレートと接続モータの一覧を渡す
void CAN_Init(uint32_t baud, const MotorConfig* configs, size_t num_configs);

// 受信処理の実行（loop内で高頻度に呼ぶ）
void CAN_Poll();

// 目標電流を1台ぶんセット（値はDJI規定スケール／16bit、符号付き、後述）
void CAN_SetMotorCurrent(uint8_t id, int16_t value);

// まとめて送信（ID1-4は0x200、ID5-8は0x1FFで各4台パック送信）
void CAN_FlushCurrents();

// モータ状態の参照（true=有効ID）
bool CAN_GetState(uint8_t id, MotorState& out);

// 全台ゼロ電流を即時送信（安全停止に）
void CAN_ZeroAllAndFlush();
