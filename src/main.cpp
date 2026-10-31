#include <Arduino.h>
#include <SoftwareSerial.h>

// サーボ ID 1、115200 bps。D2 は受信、D4 は送信。
constexpr uint8_t SERVO_ID = 1;
constexpr uint16_t SPEED = 300;
SoftwareSerial servo(PIN_D02, PIN_D04);

// FEETECH のパケット: FF FF ID 長さ 命令 パラメータ チェックサム。
void sendPacket(uint8_t instruction, const uint8_t *params, uint8_t count) {
  uint8_t checksum = SERVO_ID + count + 2 + instruction;
  servo.write(0xff);
  servo.write(0xff);
  servo.write(SERVO_ID);
  servo.write(count + 2);
  servo.write(instruction);
  for (uint8_t i = 0; i < count; ++i) {
    servo.write(params[i]);
    checksum += params[i];
  }
  servo.write(uint8_t(~checksum));
}

void moveTo(int position) {
  // レジスタ41から、加速度・位置・時間・速度を順に書き込む。
  const uint8_t params[] = {41, 50, uint8_t(position), uint8_t(position >> 8),
                            0, 0, uint8_t(SPEED), uint8_t(SPEED >> 8)};
  sendPacket(3, params, sizeof(params));
}

int readPosition() {
  while (servo.available()) servo.read();
  const uint8_t params[] = {56, 2};  // 現在位置を2バイト読む。
  sendPacket(2, params, sizeof(params));

  // 先頭の FF は回路の待機電位で崩れることがあるため、2番目の FF から検証する。
  uint8_t reply[7] = {};
  int count = 0;
  unsigned long start = millis();
  while (millis() - start < 50) {
    if (!servo.available()) continue;
    uint8_t byte = servo.read();
    if (count < 7) reply[count++] = byte;
    if (count < 7) continue;
    uint8_t sum = 0;
    for (int i = 1; i < 7; ++i) sum += reply[i];
    if (reply[0] == 0xff && reply[1] == SERVO_ID &&
        reply[2] == 4 && reply[3] == 0 && sum == 0xff) {
      int position = reply[4] | (reply[5] << 8);
      return position <= 4095 ? position : -1;
    }
    for (int i = 1; i < 7; ++i) reply[i - 1] = reply[i];
    count = 6;
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
  servo.begin(115200);
  digitalWrite(PIN_D04, HIGH);  // 送信していない間は High。
  delay(1000);

  // 1. トルクを有効にする。
  const uint8_t torque[] = {40, 1};
  sendPacket(3, torque, sizeof(torque));
  // 2. 中央から左右それぞれ約90度動かし、中央に戻す。
  Serial.println("Moving to 2048");
  moveTo(2048);
  delay(8000);
  printPosition();
  Serial.println("Moving to 3072");
  moveTo(3072);
  delay(9000);
  printPosition();
  Serial.println("Moving to 1024");
  moveTo(1024);
  delay(9000);
  printPosition();
  Serial.println("Moving to 2048");
  moveTo(2048);
  delay(9000);
  printPosition();
  Serial.println("Finished");
}

void loop() { delay(1000); }  // リセットすると再実行する。
