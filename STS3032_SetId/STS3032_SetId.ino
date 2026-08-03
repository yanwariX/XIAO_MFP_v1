// FEETECH STS3032 ID変更用スケッチ
// 注意:IDを変更したいサーボを1台だけ接続してください。

#include <Arduino.h>
#include <SCServo.h>

// ============================================================
// ★割り当てるサーボ番号
static const uint8_t kNewServoId = 2;   
// ============================================================


static const unsigned long kUsbSerialBaud = 115200;
static const unsigned long kServoBusBauds[] = {
  1000000,
  115200
};
static const unsigned long kPingTimeoutMs = 20;

// Seeed XIAO用バスサーボドライバボードの標準配線です。
static const int kServoBusRxPin = D7;
static const int kServoBusTxPin = D6;

// ============================================================
// サーボバス設定
// ============================================================
#if defined(CONFIG_IDF_TARGET_ESP32C3) || defined(CONFIG_IDF_TARGET_ESP32C6) || defined(CONFIG_IDF_TARGET_ESP32S3)
  #define SERVO_SERIAL Serial0
#else
  #define SERVO_SERIAL Serial1
#endif

SMS_STS g_servoBus;

static void beginServoBus(unsigned long baud) {
  SERVO_SERIAL.end();
  delay(50);
  SERVO_SERIAL.begin(baud, SERIAL_8N1, kServoBusRxPin, kServoBusTxPin);
  g_servoBus.pSerial = &SERVO_SERIAL;
  g_servoBus.IOTimeOut = kPingTimeoutMs;
  delay(500);
}

static int scanServoId() {
  int foundId = -1;
  uint8_t foundCount = 0;

  for (uint16_t id = 0; id <= 253; ++id) {
    if ((id % 32) == 0) {
      Serial.print("  scanning ID ");
      Serial.println(id);
    }

    const int result = g_servoBus.Ping(static_cast<uint8_t>(id));
    if (result != -1) {
      Serial.print("Found servo ID: ");
      Serial.println(result);
      foundId = result;
      ++foundCount;
    }
    delay(2);
  }

  if (foundCount == 0) {
    return -1;
  }
  if (foundCount > 1) {
    return -2;
  }
  return foundId;
}

static bool pingId(uint8_t id, const char *label) {
  const int result = g_servoBus.Ping(id);
  Serial.print(label);
  Serial.print(" ID ");
  Serial.print(id);
  Serial.print(": ");

  if (result == -1) {
    Serial.println("not found");
    return false;
  }

  Serial.print("found, response ID=");
  Serial.println(result);
  return true;
}

static void changeServoId(uint8_t currentId, uint8_t newId) {
  if (currentId == newId) {
    Serial.println("Servo already has the target ID. No change needed.");
    return;
  }

  Serial.print("Change ID ");
  Serial.print(currentId);
  Serial.print(" -> ");
  Serial.println(newId);

  Serial.println("Unlock EPROM...");
  g_servoBus.unLockEprom(currentId);
  delay(100);

  Serial.println("Writing new ID...");
  g_servoBus.writeByte(currentId, SMS_STS_ID, newId);
  delay(100);

  Serial.println("Lock EPROM...");
  g_servoBus.LockEprom(newId);
  delay(500);

  if (pingId(newId, "After change")) {
    Serial.println("ID change completed.");
  } else {
    Serial.println("ID write command was sent, but the new ID did not respond.");
    Serial.println("Power-cycle the servo and run this sketch again.");
  }
}

void setup() {
  Serial.begin(kUsbSerialBaud);
  delay(3000);

  Serial.println();
  Serial.println("STS3032 ID changer auto-scan");
  Serial.println("Connect only one servo before running this sketch.");
  Serial.print("Target new ID: ");
  Serial.println(kNewServoId);

  for (size_t i = 0; i < (sizeof(kServoBusBauds) / sizeof(kServoBusBauds[0])); ++i) {
    const unsigned long baud = kServoBusBauds[i];
    Serial.print("Scanning at baud ");
    Serial.println(baud);

    beginServoBus(baud);
    const int foundId = scanServoId();

    if (foundId >= 0) {
      changeServoId(static_cast<uint8_t>(foundId), kNewServoId);
      return;
    }

    if (foundId == -2) {
      Serial.println("Multiple servo IDs responded. Disconnect all but one servo and try again.");
      return;
    }

    Serial.println("No servo found at this baud.");
  }

  Serial.println("ID change aborted.");
  Serial.println("Check servo power, wiring, board pins, and whether the servo is connected alone.");
}

void loop() {
}
