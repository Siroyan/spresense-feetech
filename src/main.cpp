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
volatile uint32_t *rxPin;

// 受信波形の変化時刻。8バイトの応答は最大80回の変化がある。
uint32_t edgeTime[120];
uint8_t edgeLevel[120];

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

// 3. 現在位置を問い合わせ、波形を記録してから 8N1 の各ビットを読む。
int readPosition() {
  const uint8_t request[] = {0xff, 0xff, SERVO_ID, 4, 2, 56, 2,
                             uint8_t(~(SERVO_ID + 4 + 2 + 56 + 2))};
  int count = 0;
  noInterrupts();
  for (uint8_t byte : request) sendByte(byte);
  uint32_t start = DWT_CYCCNT;
  int initial = (*rxPin & GPIO_INPUT_MASK) != 0;
  int last = initial;
  // 1 Mbps の連続した応答を逃さないよう、まず信号が変わった時刻を記録する。
  while (uint32_t(DWT_CYCCNT - start) < bitCycles * 500) {
    int level = (*rxPin & GPIO_INPUT_MASK) != 0;
    if (level != last) {
      if (count == 120) break;
      edgeTime[count] = DWT_CYCCNT - start;
      edgeLevel[count++] = level;
      last = level;
    }
  }
  interrupts();
  if (count == 120) return -1;

  // 送信直後の不要な変化を飛ばし、有効なヘッダーから応答を探す。
  for (int candidate = 0; candidate < count; ++candidate) {
    if (edgeLevel[candidate] != 0) continue;
    uint8_t reply[8] = {};
    int edge = candidate;
    bool valid = true;
    for (int byte = 0; byte < 8 && valid; ++byte) {
      while (edge < count && edgeLevel[edge] != 0) ++edge;
      if (edge == count) { valid = false; break; }
      uint32_t byteStart = edgeTime[edge++];
      for (int bit = 0; bit < 9; ++bit) {
        uint32_t sample = byteStart + bitCycles * (bit + 1) + bitCycles / 2;
        int level = initial;
        for (int i = 0; i < count && edgeTime[i] <= sample; ++i) level = edgeLevel[i];
        if (bit < 8 && level) reply[byte] |= 1u << bit;
        if (bit == 8 && !level) valid = false;
      }
      while (edge < count && edgeTime[edge] < byteStart + bitCycles * 9) ++edge;
    }
    uint8_t sum = 0;
    for (int i = 2; i < 8; ++i) sum += reply[i];
    if (valid && reply[0] == 0xff && reply[1] == 0xff && reply[2] == SERVO_ID &&
        reply[3] == 4 && reply[4] == 0 && sum == 0xff) {
      int position = reply[5] | (reply[6] << 8);
      return position <= 4095 ? position : -1;
    }
  }
  return -1;
}

void printPosition() {
  int position = readPosition();
  if (position < 0) Serial.println("Position read failed");
  else { Serial.print("Position: "); Serial.println(position); }
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_D02, INPUT);
  pinMode(PIN_D04, OUTPUT);
  digitalWrite(PIN_D04, HIGH);
  txPin = reinterpret_cast<volatile uint32_t *>(get_gpio_regaddr(pin_convert(PIN_D04)));
  rxPin = reinterpret_cast<volatile uint32_t *>(get_gpio_regaddr(pin_convert(PIN_D02)));
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
  printPosition();
  Serial.println("Moving to 2248");
  moveTo(2248);
  delay(3000);
  printPosition();
  Serial.println("Moving to 2048");
  moveTo(2048);
  delay(3000);
  printPosition();
  Serial.println("Finished");
}

// 起動時に一度だけ動かす。リセットすると再実行する。
void loop() { delay(1000); }
