#include <Arduino.h>
#include <wiring_private.h>

// MAX485 回路の RXD(RO)→D2、TXD(DI)←D4。サーボは ID 1、1 Mbps。
constexpr uint8_t SERVO_ID = 1;
constexpr uint16_t SPEED = 300;

// 1 Mbps は SPRESENSE 標準 SoftwareSerial の対象外のため、GPIO を直接操作する。
#define DEMCR (*(volatile uint32_t *)0xE000EDFC)
#define DWT_CTRL (*(volatile uint32_t *)0xE0001000)
#define DWT_CYCCNT (*(volatile uint32_t *)0xE0001004)
volatile uint32_t *txPin;
uint32_t bitCycles;

void waitUntil(uint32_t time) {
  while (static_cast<int32_t>(DWT_CYCCNT - time) < 0) {}
}

// 8N1: スタートビット、データ 8 ビット、ストップビットを送る。
void sendByte(uint8_t value) {
  uint32_t time = DWT_CYCCNT;
  *txPin = GPIO_OUTPUT_LOW;
  for (int bit = 0; bit < 8; ++bit) {
    time += bitCycles;
    waitUntil(time);
    *txPin = (value & 1) ? GPIO_OUTPUT_HIGH : GPIO_OUTPUT_LOW;
    value >>= 1;
  }
  time += bitCycles;
  waitUntil(time);
  *txPin = GPIO_OUTPUT_HIGH;
  waitUntil(time + bitCycles);
}

// 応答を使わず書き込み指令を送る。EEPROM の設定は変更しない。
void sendWrite(const uint8_t *params, uint8_t count) {
  uint8_t sum = SERVO_ID + count + 2 + 3;
  noInterrupts();
  sendByte(0xff); sendByte(0xff); sendByte(SERVO_ID);
  sendByte(count + 2); sendByte(3);
  for (uint8_t i = 0; i < count; ++i) { sendByte(params[i]); sum += params[i]; }
  sendByte(~sum);
  interrupts();
}

// 加速度、目標位置、時間、速度を連続して書き込む。
void moveTo(int position) {
  const uint8_t params[] = {41, 50, uint8_t(position), uint8_t(position >> 8),
                            0, 0, uint8_t(SPEED), uint8_t(SPEED >> 8)};
  sendWrite(params, sizeof(params));
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_D04, OUTPUT);
  digitalWrite(PIN_D04, HIGH);
  txPin = reinterpret_cast<volatile uint32_t *>(get_gpio_regaddr(pin_convert(PIN_D04)));
  DEMCR |= 1u << 24;
  DWT_CYCCNT = 0;
  DWT_CTRL |= 1;
  bitCycles = clockCyclesPerMicrosecond();
  delay(1000);
  // 1. トルクを有効にする。
  const uint8_t torque[] = {40, 1};
  sendWrite(torque, sizeof(torque));
  // 2. 位置 2048 を基準に、約18度動かして戻す。
  Serial.println("Moving to 2048");
  moveTo(2048);
  delay(8000);
  Serial.println("Moving to 2248");
  moveTo(2248);
  delay(3000);
  Serial.println("Moving to 2048");
  moveTo(2048);
  delay(3000);
  Serial.println("Commands sent; visual confirmation required");
}

// 起動時に一度だけ動かす。リセットすると再実行する。
void loop() { delay(1000); }
