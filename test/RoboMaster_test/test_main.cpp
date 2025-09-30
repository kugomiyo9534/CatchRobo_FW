#include <Arduino.h>
#include <unity.h>

#include "RoboMaster.hpp"

// --- 接続しているモータ（例） ---
// ID1,2: M3508（C620） / ID3,4: M2006（C610）
static const MotorConfig kMotors[] = {
  {1, MotorType::M3508},
  {2, MotorType::M3508},
  {3, MotorType::M2006},
  {4, MotorType::M2006},
  // {5, MotorType::M3508}, {6, MotorType::M3508}, {7, MotorType::M2006}, {8, MotorType::M2006},
};

static const uint32_t kBaud = 1000000; // 1Mbps

// デモ用
elapsedMillis gTicker;
int gPhase = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 1500) {}

  CAN_Init(kBaud, kMotors, sizeof(kMotors)/sizeof(kMotors[0]));
  Serial.println("CAN initialized.");

  // 念のためゼロ出力
  CAN_ZeroAllAndFlush();
}

void loop() {
  // 受信処理
  CAN_Poll();

  // --- デモ：2秒ごとに正逆切替して送信 ---
  if (gTicker > 2000) {
    gTicker = 0;

    if (gPhase % 2 == 0) {
      CAN_SetMotorCurrent(1,  8000);
      CAN_SetMotorCurrent(2,  8000);
      CAN_SetMotorCurrent(3,  3000);
      CAN_SetMotorCurrent(4,  3000);
    } else {
      CAN_SetMotorCurrent(1, -8000);
      CAN_SetMotorCurrent(2, -8000);
      CAN_SetMotorCurrent(3, -3000);
      CAN_SetMotorCurrent(4, -3000);
    }
    gPhase++;

    CAN_FlushCurrents();

    // 受信状態をモニタ出力（任意）
    for (uint8_t id = 1; id <= 4; ++id) {
      MotorState s;
      if (CAN_GetState(id, s)) {
        Serial.print("ID"); Serial.print(id);
        Serial.print(" enc:"); Serial.print(s.enc);
        Serial.print(" rpm:"); Serial.print(s.rpm);
        Serial.print(" curFb:"); Serial.print(s.cur);
        Serial.print(" tmp:"); Serial.print(s.temp);
        Serial.println();
      }
    }
  }

  // 実運用ではタイマ割り込みや一定周期で CAN_FlushCurrents() を回してください（~1kHz 推奨）
}
