// MultiFunPlayer -> Seeed XIAO ESP32C3 c6 -> FEETECH STS3032 2台制御
// FTServo / SCServo 系ライブラリを導入

struct ServoRuntime;
struct ManualChannelState;

#include <Arduino.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <SCServo.h>

// ============================================================
// ユーザー設定: 主に調整する場所
// ============================================================
// Wi-Fi / UDP 接続設定
static const char *kWifiSsid = "YOUR_SSID";
static const char *kWifiPassword = "YOUR_PASSWORD";
static const char *kWifiHostname = "xiao";      // MFPでのホストネーム
static const uint16_t kUdpPort = 8889;          // MFPでのポート番号
static const bool kEnableBrowserMonitor = true;  // http://<hostname>.local でモニタ画面を表示

// サーボ1設定
static const uint8_t kServo1Id = 1;           // サーボ番号
static const char *kServo1AxisName = "A3";    // MFPのチャンネル割り当て
static const float kServo1MaxRpm = 80.0f;     // 最高速度設定(5V時の実用上限は85程度)
static const int8_t kServo1Direction = 1;     // 回転方向　-1で反転

// サーボ2設定
static const uint8_t kServo2Id = 2;           // サーボ番号
static const char *kServo2AxisName = "A4";    // MFPのチャンネル割り当て
static const float kServo2MaxRpm = 80.0f;     // 最高速度設定(5V時の実用上限は85程度)
static const int8_t kServo2Direction = 1;     // 回転方向　-1で反転

// ブラウザコントローラー初期値
static const float kManualMinTurnsDefault = 0.3f;   // min回転数
static const float kManualMaxTurnsDefault = 2.0f;   // max回転数
static const float kManualMinSpeedPercentDefault = 20.0f;   // min回転速度
static const float kManualMaxSpeedPercentDefault = 50.0f;   // max回転速度
static const float kManualMinPauseSecondsDefault = 0.0f;   // min休止時間
static const float kManualMaxPauseSecondsDefault = 0.3f;   // max休止時間
static const float kManualPauseChancePercentDefault = 30.0f;   // 休止挿入率

// ============================================================
// 固定設定 / 詳細設定
// ============================================================
// 通常はここから下を変更しなくて大丈夫です。

// USB シリアル入力設定
static const bool kEnableUsbSerialInput = true;
static const unsigned long kUsbSerialBaud = 115200;

// Wi-Fi / UDP 詳細設定
static const bool kEnableWifiUdp = true;
static const unsigned long kWifiRetryIntervalMs = 10000;
static const unsigned long kWaitForWifiOnBootMs = 15000;

// STS 閉ループ速度モードへの変換係数
// 値を上げると、同じ RPM 指定でもより強い速度コマンドを送ります。
static const float kServoRatedMaxRpm = 85.0f;
static const int kClosedLoopCommandAtRatedMaxRpm = 5700;
static const int kClosedLoopMinCommand = 10;
static const int kClosedLoopMaxCommand = 9999;
static const float kClosedLoopCommandScale = 1.0f;

// 停止と反転の安定化設定
static const int kCenterDeadbandTCode = 50;
static const int kDirectionChangeMinCommand = 60;

// Seeed XIAO 用バスサーボドライバの標準配線は D7=RX, D6=TX です。
// 配線を変えていないならそのままで大丈夫です。
static const int SERVO_BUS_RX_PIN = D7;
static const int SERVO_BUS_TX_PIN = D6;

// Seeed のサンプルでは STS のボーレートは 1000000 が一般的です。
// FT ソフト等で変更済みなら合わせてください。
static const unsigned long SERVO_BUS_BAUD = 1000000;

// true にすると起動時に WheelMode() を実行し、閉ループ速度モードへ切り替えます。
// すでに mode 1 へ設定済みで EEPROM 書き込みを減らしたい場合は false にできます。
static const bool CONFIGURE_CLOSED_LOOP_MODE_ON_BOOT = true;

// ============================================================
// ファームウェア情報
// ============================================================
static const char *kFirmwareId = "XIAO_STS3032_MFP_v1.0";
static const char *kTCodeVersion = "TCode v0.3";

// ============================================================
// 動作チューニング
// ============================================================
static const unsigned long kControlUpdateHz = 100;
// 0 にするとタイムアウトを無効化し、次の funscript 値が届くまで直前の値を保持します。
// たとえば 0 秒から 10 秒まで 70 が続く区間でも、その間ずっと 70 相当で回転します。
static const unsigned long kAxisTimeoutMs = 0;

// 実測値をシリアルへ出す場合の設定です。
// MFP と USB シリアル接続する場合は false のままにしてください。
static const bool kEnableFeedbackTelemetry = false;
static const unsigned long kFeedbackIntervalMs = 500;
static const uint16_t kHttpPort = 80;
static const unsigned long kMonitorTelemetryIntervalMs = 200;

// 電源投入直後はサーボ側の起動が間に合わず Ping に失敗することがあります。
// その対策として、起動時に少し待ってから初期化し、オフラインだった場合も
// 定期的に再試行します。
static const unsigned long kServoStartupDelayMs = 1500;
static const unsigned long kServoRetryIntervalMs = 1000;

// ============================================================
// サーボ個別設定（内部用）
// ============================================================
struct ServoConfig {
  const char *axisName;
  uint8_t id;
  float maxRpm;
  int8_t directionSign;
};

static const ServoConfig SERVO_CONFIGS[2] = {
  { kServo1AxisName, kServo1Id, kServo1MaxRpm, kServo1Direction },
  { kServo2AxisName, kServo2Id, kServo2MaxRpm, kServo2Direction }
};

// ============================================================
// サーボバス用 UART の選択
// ============================================================
#if defined(CONFIG_IDF_TARGET_ESP32C3) || defined(CONFIG_IDF_TARGET_ESP32C6) || defined(CONFIG_IDF_TARGET_ESP32S3)
  #define SERVO_SERIAL Serial0
#else
  #define SERVO_SERIAL Serial1
#endif

// ============================================================
// 内部定数
// ============================================================
static const int CHANNEL_COUNT = 10;
static const int TCODE_MIN = 0;
static const int TCODE_MAX = 9999;
static const int TCODE_CENTER = 5000;
static const size_t TCODE_LINE_BUFFER = 128;
static const uint16_t UDP_BUFFER_SIZE = 512;

enum class ReplySource {
  None,
  Serial,
  Udp
};

struct ReplyContext {
  ReplySource source;
  IPAddress udpIp;
  uint16_t udpPort;
};

static void stopAllServos();
static bool allManualFeedbackReady();
static void resetManualChannel(ManualChannelState &channel);
static void stopManualControl(bool stopServos);
static void startManualControlFromRequest(bool independent,
                                          float minTurns,
                                          float maxTurns,
                                          float minSpeedPercent,
                                          float maxSpeedPercent,
                                          float minPauseSeconds,
                                          float maxPauseSeconds,
                                          float pauseChancePercent);
static void sendReplyLine(const ReplyContext &context, const String &line);
static String buildD2Response();
static void handleMonitorRoot();
static void handleMonitorTelemetryJson();
static void handleManualStart();
static void handleManualStop();
static void pollHttpClients();

static const char kMonitorPageHtml[] PROGMEM = R"HTML(
<!doctype html>
<html lang="ja">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>STS3032 Monitor</title>
  <style>
    :root {
      color-scheme: dark;
      --bg: #12161d;
      --panel: #1b2430;
      --line: #2c3948;
      --text: #eef3f8;
      --muted: #9eb0c2;
      --good: #7dd3a5;
      --warn: #f6c177;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      font-family: "Segoe UI", sans-serif;
      background: linear-gradient(180deg, #0d1117 0%, #151c25 100%);
      color: var(--text);
    }
    .wrap {
      max-width: 980px;
      margin: 0 auto;
      padding: 24px 16px 40px;
    }
    h1 {
      margin: 0 0 8px;
      font-size: 28px;
    }
    .meta {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(220px, 1fr));
      gap: 12px;
      margin-bottom: 16px;
    }
    .panel {
      background: rgba(27, 36, 48, 0.92);
      border: 1px solid var(--line);
      border-radius: 14px;
      padding: 14px 16px;
      box-shadow: 0 10px 24px rgba(0, 0, 0, 0.18);
    }
    .servo-grid {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(300px, 1fr));
      gap: 14px;
    }
    .servo-head {
      display: flex;
      justify-content: space-between;
      align-items: baseline;
      gap: 12px;
      margin-bottom: 10px;
    }
    .servo-title {
      font-size: 22px;
      font-weight: 700;
    }
    .state {
      font-size: 13px;
      color: var(--muted);
    }
    .state.online { color: var(--good); }
    .state.offline { color: var(--warn); }
    .kv {
      display: grid;
      grid-template-columns: 1fr auto;
      gap: 8px 14px;
      align-items: baseline;
      margin-top: 8px;
    }
    .k {
      color: var(--muted);
      font-size: 14px;
    }
    .v {
      font-variant-numeric: tabular-nums;
      font-size: 18px;
      font-weight: 600;
    }
    .kv .v {
      display: flex;
      justify-content: flex-end;
      white-space: nowrap;
    }
    .readout {
      display: grid;
      grid-template-columns: minmax(2ch, 2ch) minmax(5ch, 5ch) minmax(5ch, 5ch);
      gap: 0 0.6ch;
      align-items: baseline;
    }
    .readout .sign {
      text-align: right;
    }
    .readout .num {
      text-align: right;
    }
    .readout .unit {
      text-align: left;
    }
    .controller {
      margin-bottom: 16px;
    }
    .controller-head {
      display: flex;
      justify-content: space-between;
      gap: 12px;
      align-items: center;
      margin-bottom: 12px;
    }
    .controller-title {
      font-size: 20px;
      font-weight: 700;
    }
    .control-rows {
      display: grid;
      gap: 10px;
    }
    .control-row {
      display: grid;
      grid-template-columns: 7em minmax(72px, 1fr) auto minmax(72px, 1fr) auto minmax(72px, 1fr) auto;
      gap: 8px;
      align-items: center;
    }
    .control-row.actions-row {
      grid-template-columns: 7em auto auto minmax(10em, auto) 1fr;
    }
    .control-label,
    label {
      display: grid;
      gap: 5px;
      color: var(--muted);
      font-size: 13px;
    }
    .control-label {
      display: block;
      font-weight: 700;
    }
    .range-mark,
    .unit-mark {
      color: var(--muted);
      font-size: 14px;
      white-space: nowrap;
    }
    input {
      width: 100%;
      border: 1px solid var(--line);
      border-radius: 8px;
      padding: 8px 10px;
      background: #111821;
      color: var(--text);
      font: inherit;
      font-variant-numeric: tabular-nums;
    }
    .actions {
      display: flex;
      align-items: center;
      gap: 10px;
      flex-wrap: wrap;
      margin-top: 12px;
    }
    button {
      border: 1px solid var(--line);
      border-radius: 8px;
      padding: 9px 14px;
      background: #223044;
      color: var(--text);
      font: inherit;
      font-weight: 700;
      cursor: pointer;
    }
    button.primary {
      background: #2d5b48;
      border-color: #3f8b69;
    }
    button.danger {
      background: #5a2e35;
      border-color: #9c4e5d;
    }
    button:disabled,
    input:disabled {
      opacity: 0.45;
      cursor: not-allowed;
    }
    .toggle {
      display: inline-flex;
      align-items: center;
      gap: 8px;
      color: var(--muted);
      font-size: 14px;
    }
    .toggle input {
      width: auto;
    }
    @media (max-width: 640px) {
      .control-row,
      .control-row.actions-row {
        grid-template-columns: 1fr 1fr;
      }
      .control-label {
        grid-column: 1 / -1;
      }
      .range-mark,
      .unit-mark {
        align-self: center;
      }
    }
  </style>
</head>
<body>
  <div class="wrap">
    <h1>STS3032 Monitor</h1>
    <div class="meta">
      <div class="panel"><div class="k">接続先</div><div class="v" id="host">-</div></div>
    </div>
    <div class="panel controller">
      <div class="controller-head">
        <div class="controller-title">ブラウザコントローラー</div>
        <div class="state" id="manualState">standby</div>
      </div>
      <div class="control-rows" id="manualInputs">
        <div class="control-row">
          <div class="control-label">回転数</div>
          <input id="minTurns" type="number" min="0" max="100" step="0.1" value="0.3">
          <span class="range-mark">～</span>
          <input id="maxTurns" type="number" min="0" max="100" step="0.1" value="2">
        </div>
        <div class="control-row">
          <div class="control-label">回転速度</div>
          <input id="minSpeed" type="number" min="0" max="100" step="1" value="20">
          <span class="range-mark">～</span>
          <input id="maxSpeed" type="number" min="0" max="100" step="1" value="50">
          <span class="unit-mark">%</span>
        </div>
        <div class="control-row">
          <div class="control-label">休止挿入</div>
          <input id="minPause" type="number" min="0" max="10" step="0.1" value="0.0">
          <span class="range-mark">～</span>
          <input id="maxPause" type="number" min="0" max="10" step="0.1" value="0.3">
          <span class="unit-mark">秒</span>
          <input id="pauseChance" type="number" min="0" max="100" step="10" value="30">
          <span class="unit-mark">%</span>
        </div>
      <div class="control-row actions-row">
        <div class="control-label">操作</div>
          <button class="primary" id="startBtn" type="button">start</button>
          <button class="danger" id="stopBtn" type="button">stop</button>
          <label class="toggle"><input id="independent" type="checkbox">左右別動作</label>
        </div>
      </div>
    </div>
    <div class="servo-grid" id="servos"></div>
  </div>
  <script>
    const fields = ['minTurns', 'maxTurns', 'minSpeed', 'maxSpeed', 'minPause', 'maxPause', 'pauseChance'];
    const startBtn = document.getElementById('startBtn');
    const stopBtn = document.getElementById('stopBtn');
    const independent = document.getElementById('independent');
    const manualState = document.getElementById('manualState');
    let manualDefaultsApplied = false;
    const applyManualDefaults = (defaults) => {
      if (manualDefaultsApplied || !defaults) return;
      fields.forEach((id) => {
        if (defaults[id] !== undefined) {
          document.getElementById(id).value = defaults[id];
        }
      });
      manualDefaultsApplied = true;
    };
    const params = () => {
      const data = new URLSearchParams();
      fields.forEach((id) => data.set(id, document.getElementById(id).value));
      data.set('independent', independent.checked ? '1' : '0');
      return data;
    };
    const postManual = async (path, body = null) => {
      const options = { method: 'POST' };
      if (body) {
        options.headers = { 'Content-Type': 'application/x-www-form-urlencoded' };
        options.body = body;
      }
      const response = await fetch(path, options);
      return response.json();
    };
    const updateController = (data) => {
      const running = Boolean(data.manualRequested || data.manualActive);
      const canStart = Boolean(data.manualAvailable) && !running;
      fields.forEach((id) => { document.getElementById(id).disabled = running; });
      independent.disabled = running;
      startBtn.disabled = !canStart;
      stopBtn.disabled = !running;
      manualState.textContent = running ? 'ブラウザ制御中' : data.manualAvailable ? 'UDP待機' : 'フィードバック待ち';
      manualState.className = `state ${data.manualAvailable ? 'online' : 'offline'}`;
    };
    startBtn.addEventListener('click', async () => {
      const data = await postManual('/api/manual/start', params());
      updateController(data);
    });
    stopBtn.addEventListener('click', async () => {
      const data = await postManual('/api/manual/stop');
      updateController(data);
    });
    const enableWheelSpin = (input) => {
      input.addEventListener('wheel', (event) => {
        if (document.activeElement !== input || input.disabled) return;
        event.preventDefault();
        input.stepUp(event.deltaY < 0 ? 1 : -1);
        input.dispatchEvent(new Event('input', { bubbles: true }));
      }, { passive: false });
    };
    fields.map((id) => document.getElementById(id)).forEach(enableWheelSpin);
    const renderReadout = (signText = '', numText = '-', unitText = '') => {
      return `<span class="readout"><span class="sign">${signText}</span><span class="num">${numText}</span><span class="unit">${unitText}</span></span>`;
    };
    const fmt = (value, digits = 1, unit = '') => {
      if (value === null || value === undefined || Number.isNaN(value)) return renderReadout('', '-', unit);
      return renderReadout('', Number(value).toFixed(digits), unit);
    };
    const fmtRpm = (value) => {
      if (value === null || value === undefined || Number.isNaN(value)) return renderReadout('', '-', 'RPM');
      const rawRpm = Number(value);
      const rpm = Math.abs(rawRpm) < 0.15 ? 0 : rawRpm;
      const sign = rpm > 0.05 ? '+' : rpm < -0.05 ? '-' : '';
      return renderReadout(sign, Math.abs(rpm).toFixed(1), 'RPM');
    };
    const fmtInt = (value, unit = '') => {
      if (value === null || value === undefined || Number.isNaN(value)) return renderReadout('', '-', unit);
      return renderReadout('', String(Math.round(Number(value))), unit);
    };
    const renderServo = (servo) => {
      const stateClass = servo.feedbackValid ? 'online' : 'offline';
      const stateText = servo.feedbackValid ? 'online' : 'no feedback';
      return `
        <div class="panel">
          <div class="servo-head">
            <div class="servo-title">${servo.axis} / ID ${servo.id}</div>
            <div class="state ${stateClass}">${stateText}</div>
          </div>
          <div class="kv">
            <div class="k">目標回転数</div><div class="v">${fmtRpm(servo.targetRpm)}</div>
            <div class="k">実回転数</div><div class="v">${fmtRpm(servo.derivedRpm)}</div>
            <div class="k">累計回転数(右)</div><div class="v">${fmt(servo.rightTurnsSinceBoot, 1, ' turns')}</div>
            <div class="k">累計回転数(左)</div><div class="v">${fmt(servo.leftTurnsSinceBoot, 1, ' turns')}</div>
            <div class="k">電圧</div><div class="v">${fmt(servo.voltageV, 1, ' V')}</div>
            <div class="k">温度</div><div class="v">${fmtInt(servo.temperatureC, ' °C')}</div>
          </div>
        </div>`;
    };
    const refresh = async () => {
      try {
        const response = await fetch('/api/telemetry', { cache: 'no-store' });
        const data = await response.json();
        document.getElementById('host').textContent = location.host;
        applyManualDefaults(data.manualDefaults);
        updateController(data);
        document.getElementById('servos').innerHTML = data.servos.map(renderServo).join('');
      } catch (error) {
        document.getElementById('servos').innerHTML = '<div class="panel"><div class="v">通信エラー</div></div>';
      }
    };
    refresh();
    setInterval(refresh, 500);
  </script>
</body>
</html>
)HTML";

// ============================================================
// 補助関数
// ============================================================
static inline int clampInt(int value, int minValue, int maxValue) {
  if (value < minValue) return minValue;
  if (value > maxValue) return maxValue;
  return value;
}

static inline int signOfInt(int value) {
  if (value > 0) return 1;
  if (value < 0) return -1;
  return 0;
}

static inline unsigned long controlIntervalMs() {
  return (1000UL + kControlUpdateHz - 1UL) / kControlUpdateHz;
}

static inline int axisIndexFromName(const char *axisName) {
  if (axisName == nullptr) return -1;
  if (axisName[0] != 'A' || axisName[1] < '0' || axisName[1] > '9') return -1;
  return axisName[1] - '0';
}

// ============================================================
// TCode 軸状態
// ============================================================
struct AxisState {
  int value;
  unsigned long lastUpdateMs;
};

class TCodeState {
public:
  TCodeState() : lineLength(0) {
    for (int i = 0; i < CHANNEL_COUNT; ++i) {
      axes[i].value = TCODE_CENTER;
      axes[i].lastUpdateMs = 0;
    }
    lineBuffer[0] = '\0';
    replyContext.source = ReplySource::Serial;
    replyContext.udpPort = 0;
  }

  void setReplyContextSerial() {
    replyContext.source = ReplySource::Serial;
    replyContext.udpPort = 0;
  }

  void setReplyContextUdp(const IPAddress &remoteIp, uint16_t remotePort) {
    replyContext.source = ReplySource::Udp;
    replyContext.udpIp = remoteIp;
    replyContext.udpPort = remotePort;
  }

  void inputByte(uint8_t byteValue) {
    const char c = static_cast<char>(byteValue);

    if (c == '\r') {
      return;
    }

    if (c == '\n') {
      if (lineLength > 0) {
        lineBuffer[lineLength] = '\0';
        executeLine(lineBuffer);
        resetLine();
      }
      return;
    }

    if (lineLength >= (TCODE_LINE_BUFFER - 1)) {
      resetLine();
      return;
    }

    lineBuffer[lineLength++] = c;
    lineBuffer[lineLength] = '\0';
  }

  int readAxis(const char *axisName) const {
    const int index = axisIndexFromName(axisName);
    if (index < 0 || index >= CHANNEL_COUNT) {
      return TCODE_CENTER;
    }

    const AxisState &axis = axes[index];
    if (kAxisTimeoutMs > 0 && axis.lastUpdateMs > 0) {
      const unsigned long age = millis() - axis.lastUpdateMs;
      if (age > kAxisTimeoutMs) {
        return TCODE_CENTER;
      }
    } else if (axis.lastUpdateMs == 0) {
      return TCODE_CENTER;
    }

    return axis.value;
  }

  void centerAll() {
    const unsigned long now = millis();
    for (int i = 0; i < CHANNEL_COUNT; ++i) {
      axes[i].value = TCODE_CENTER;
      axes[i].lastUpdateMs = now;
    }
  }

private:
  AxisState axes[CHANNEL_COUNT];
  char lineBuffer[TCODE_LINE_BUFFER];
  size_t lineLength;
  ReplyContext replyContext;

  void resetLine() {
    lineLength = 0;
    lineBuffer[0] = '\0';
  }

  void executeLine(char *line) {
    char *savePtr = nullptr;
    char *token = strtok_r(line, " ", &savePtr);
    while (token != nullptr) {
      handleToken(token);
      token = strtok_r(nullptr, " ", &savePtr);
    }
  }

  void handleToken(const char *token) {
    if (token == nullptr || token[0] == '\0') {
      return;
    }

    if (token[0] == 'D') {
      if (strcmp(token, "D0") == 0) {
        sendReplyLine(replyContext, String("D0 ") + kFirmwareId);
      } else if (strcmp(token, "D1") == 0) {
        sendReplyLine(replyContext, String("D1 ") + kTCodeVersion);
      } else if (strcmp(token, "D2") == 0) {
        sendReplyLine(replyContext, buildD2Response());
      } else if (strcmp(token, "DSTOP") == 0) {
        for (int i = 0; i < CHANNEL_COUNT; ++i) {
          axes[i].value = TCODE_CENTER;
          axes[i].lastUpdateMs = millis();
        }
        stopAllServos();
      }
      return;
    }

    if (token[0] != 'A' || token[1] < '0' || token[1] > '9') {
      return;
    }

    const int channel = token[1] - '0';
    const int value = clampInt(atoi(token + 2), TCODE_MIN, TCODE_MAX);
    axes[channel].value = value;
    axes[channel].lastUpdateMs = millis();
  }
};

// ============================================================
// サーボ実行時状態
// ============================================================
struct ServoRuntime {
  bool online;
  int lastCommandUnits;
  int lastNonZeroDirection;
  float targetRpm;
  float measuredRpm;
  unsigned long lastFeedbackMs;
  bool feedbackValid;
  int positionRaw;
  float positionDeg;
  long relativePositionTicks;
  float turnsSinceBoot;
  float rightTurnsSinceBoot;
  float leftTurnsSinceBoot;
  int speedUnits;
  int loadRaw;
  float loadPercent;
  int voltageRaw;
  float voltageVolts;
  int temperatureC;
  bool hasPositionSample;
  int lastPositionRaw;
  unsigned long lastPositionSampleMs;
  float derivedRpm;
};

enum class ManualPhase {
  Idle,
  Running,
  Pause
};

struct ManualChannelState {
  ManualPhase phase;
  long segmentStartTicks;
  float targetTurns;
  float targetRpm;
  unsigned long pauseUntilMs;
};

struct ManualControlState {
  bool requested;
  bool active;
  bool independent;
  float minTurns;
  float maxTurns;
  float minSpeedPercent;
  float maxSpeedPercent;
  float minPauseSeconds;
  float maxPauseSeconds;
  float pauseChancePercent;
  ManualChannelState channels[2];
};

static void resetServoTelemetry(ServoRuntime &runtime);
static void updateDerivedTelemetry(ServoRuntime &runtime, int rawPosition, unsigned long now);

static TCodeState g_tcode;
static SMS_STS g_servoBus;
static WiFiUDP g_udp;
static WebServer g_httpServer(kHttpPort);
static uint8_t g_udpBuffer[UDP_BUFFER_SIZE];
static ServoRuntime g_servoRuntime[2];
static ManualControlState g_manualControl;

static bool g_udpReady = false;
static bool g_httpReady = false;
static bool g_mdnsReady = false;
static unsigned long g_lastWifiRetryMs = 0;
static unsigned long g_lastControlMs = 0;
static unsigned long g_lastServoRetryMs = 0;
static unsigned long g_lastUdpPacketMs = 0;

static inline float positionRawToDegrees(int rawPosition) {
  float normalized = fmodf(static_cast<float>(rawPosition), 4096.0f);
  if (normalized < 0.0f) {
    normalized += 4096.0f;
  }
  return normalized * (360.0f / 4096.0f);
}

static inline bool telemetryPollingEnabled() {
  return kEnableFeedbackTelemetry || kEnableBrowserMonitor;
}

static inline unsigned long telemetryIntervalMs() {
  return kEnableBrowserMonitor ? kMonitorTelemetryIntervalMs : kFeedbackIntervalMs;
}

static inline float clampFloat(float value, float minValue, float maxValue) {
  if (value < minValue) return minValue;
  if (value > maxValue) return maxValue;
  return value;
}

static float randomFloatInRange(float minValue, float maxValue) {
  if (maxValue < minValue) {
    const float temp = minValue;
    minValue = maxValue;
    maxValue = temp;
  }
  const long scaled = random(0, 10001);
  const float ratio = static_cast<float>(scaled) / 10000.0f;
  return minValue + ((maxValue - minValue) * ratio);
}

static bool randomPercent(float percent) {
  const float clamped = clampFloat(percent, 0.0f, 100.0f);
  return randomFloatInRange(0.0f, 100.0f) < clamped;
}

static void resetServoTelemetry(ServoRuntime &runtime) {
  runtime.targetRpm = 0.0f;
  runtime.measuredRpm = 0.0f;
  runtime.lastFeedbackMs = 0;
  runtime.feedbackValid = false;
  runtime.positionRaw = 0;
  runtime.positionDeg = 0.0f;
  runtime.relativePositionTicks = 0;
  runtime.turnsSinceBoot = 0.0f;
  runtime.rightTurnsSinceBoot = 0.0f;
  runtime.leftTurnsSinceBoot = 0.0f;
  runtime.speedUnits = 0;
  runtime.loadRaw = 0;
  runtime.loadPercent = 0.0f;
  runtime.voltageRaw = 0;
  runtime.voltageVolts = 0.0f;
  runtime.temperatureC = 0;
  runtime.hasPositionSample = false;
  runtime.lastPositionRaw = 0;
  runtime.lastPositionSampleMs = 0;
  runtime.derivedRpm = 0.0f;
}

static void updateDerivedTelemetry(ServoRuntime &runtime, int rawPosition, unsigned long now) {
  runtime.positionRaw = rawPosition;
  runtime.positionDeg = positionRawToDegrees(rawPosition);

  if (!runtime.hasPositionSample) {
    runtime.hasPositionSample = true;
    runtime.lastPositionRaw = rawPosition;
    runtime.lastPositionSampleMs = now;
    runtime.relativePositionTicks = 0;
    runtime.turnsSinceBoot = 0.0f;
    runtime.rightTurnsSinceBoot = 0.0f;
    runtime.leftTurnsSinceBoot = 0.0f;
    runtime.derivedRpm = 0.0f;
    return;
  }

  int delta = rawPosition - runtime.lastPositionRaw;
  if (delta > 2048) {
    delta -= 4096;
  } else if (delta < -2048) {
    delta += 4096;
  }

  const unsigned long dtMs = now - runtime.lastPositionSampleMs;
  runtime.relativePositionTicks += delta;
  runtime.turnsSinceBoot = static_cast<float>(runtime.relativePositionTicks) / 4096.0f;

  if (delta > 0) {
    runtime.rightTurnsSinceBoot += static_cast<float>(delta) / 4096.0f;
  } else if (delta < 0) {
    runtime.leftTurnsSinceBoot += static_cast<float>(-delta) / 4096.0f;
  }

  if (dtMs > 0) {
    runtime.derivedRpm =
        (static_cast<float>(delta) / 4096.0f) * (60000.0f / static_cast<float>(dtMs));
  }

  runtime.lastPositionRaw = rawPosition;
  runtime.lastPositionSampleMs = now;
}

static void appendJsonFloat(String &json, float value, uint8_t digits) {
  json += String(static_cast<double>(value), static_cast<unsigned int>(digits));
}

static void appendJsonFloatOrNull(String &json, bool valid, float value, uint8_t digits) {
  if (valid) {
    appendJsonFloat(json, value, digits);
  } else {
    json += "null";
  }
}

static void appendJsonIntOrNull(String &json, bool valid, int value) {
  if (valid) {
    json += String(value);
  } else {
    json += "null";
  }
}

static String buildMonitorTelemetryJson() {
  String json;
  json.reserve(1600);
  json += "{\"hostname\":\"";
  json += kWifiHostname;
  json += "\",\"ip\":\"";
  json += WiFi.localIP().toString();
  json += "\",\"udpActive\":";
  json += (g_udpReady && g_lastUdpPacketMs > 0) ? "true" : "false";
  json += ",\"browserControl\":";
  json += g_manualControl.requested ? "true" : "false";
  json += ",\"manualRequested\":";
  json += g_manualControl.requested ? "true" : "false";
  json += ",\"manualActive\":";
  json += g_manualControl.active ? "true" : "false";
  json += ",\"manualAvailable\":";
  json += allManualFeedbackReady() ? "true" : "false";
  json += ",\"manualIndependent\":";
  json += g_manualControl.independent ? "true" : "false";
  json += ",\"manualDefaults\":{";
  json += "\"minTurns\":";
  appendJsonFloat(json, kManualMinTurnsDefault, 1);
  json += ",\"maxTurns\":";
  appendJsonFloat(json, kManualMaxTurnsDefault, 1);
  json += ",\"minSpeed\":";
  appendJsonFloat(json, kManualMinSpeedPercentDefault, 0);
  json += ",\"maxSpeed\":";
  appendJsonFloat(json, kManualMaxSpeedPercentDefault, 0);
  json += ",\"minPause\":";
  appendJsonFloat(json, kManualMinPauseSecondsDefault, 1);
  json += ",\"maxPause\":";
  appendJsonFloat(json, kManualMaxPauseSecondsDefault, 1);
  json += ",\"pauseChance\":";
  appendJsonFloat(json, kManualPauseChancePercentDefault, 0);
  json += '}';
  json += ",\"servos\":[";

  for (uint8_t i = 0; i < 2; ++i) {
    if (i > 0) {
      json += ',';
    }

    const ServoRuntime &runtime = g_servoRuntime[i];
    const bool valid = runtime.online && runtime.feedbackValid;

    json += "{\"axis\":\"";
    json += SERVO_CONFIGS[i].axisName;
    json += "\",\"id\":";
    json += String(SERVO_CONFIGS[i].id);
    json += ",\"online\":";
    json += runtime.online ? "true" : "false";
    json += ",\"feedbackValid\":";
    json += valid ? "true" : "false";
    json += ",\"targetRpm\":";
    appendJsonFloat(json, runtime.targetRpm, 2);
    json += ",\"estimatedRpm\":";
    appendJsonFloatOrNull(json, valid, runtime.measuredRpm, 2);
    json += ",\"derivedRpm\":";
    appendJsonFloatOrNull(json, valid, runtime.derivedRpm, 2);
    json += ",\"speedUnits\":";
    appendJsonIntOrNull(json, valid, runtime.speedUnits);
    json += ",\"positionRaw\":";
    appendJsonIntOrNull(json, valid, runtime.positionRaw);
    json += ",\"positionDeg\":";
    appendJsonFloatOrNull(json, valid, runtime.positionDeg, 2);
    json += ",\"turnsSinceBoot\":";
    appendJsonFloatOrNull(json, valid, runtime.turnsSinceBoot, 3);
    json += ",\"rightTurnsSinceBoot\":";
    appendJsonFloatOrNull(json, valid, runtime.rightTurnsSinceBoot, 3);
    json += ",\"leftTurnsSinceBoot\":";
    appendJsonFloatOrNull(json, valid, runtime.leftTurnsSinceBoot, 3);
    json += ",\"voltageV\":";
    appendJsonFloatOrNull(json, valid, runtime.voltageVolts, 1);
    json += ",\"loadPercent\":";
    appendJsonFloatOrNull(json, valid, runtime.loadPercent, 1);
    json += ",\"temperatureC\":";
    appendJsonIntOrNull(json, valid, runtime.temperatureC);
    json += '}';
  }

  json += "]}";
  return json;
}

static void handleMonitorRoot() {
  g_httpServer.send(200, "text/html; charset=utf-8", kMonitorPageHtml);
}

static void handleMonitorTelemetryJson() {
  g_httpServer.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate");
  g_httpServer.send(200, "application/json; charset=utf-8", buildMonitorTelemetryJson());
}

static float httpArgFloat(const char *name, float defaultValue) {
  if (!g_httpServer.hasArg(name)) {
    return defaultValue;
  }
  return g_httpServer.arg(name).toFloat();
}

static bool httpArgBool(const char *name, bool defaultValue) {
  if (!g_httpServer.hasArg(name)) {
    return defaultValue;
  }
  const String value = g_httpServer.arg(name);
  return value == "1" || value == "true" || value == "on";
}

static void sendManualStatusJson(int statusCode) {
  g_httpServer.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate");
  g_httpServer.send(statusCode, "application/json; charset=utf-8", buildMonitorTelemetryJson());
}

static void handleManualStart() {
  if (!allManualFeedbackReady()) {
    stopManualControl(true);
    sendManualStatusJson(409);
    return;
  }

  startManualControlFromRequest(
      httpArgBool("independent", false),
      httpArgFloat("minTurns", kManualMinTurnsDefault),
      httpArgFloat("maxTurns", kManualMaxTurnsDefault),
      httpArgFloat("minSpeed", kManualMinSpeedPercentDefault),
      httpArgFloat("maxSpeed", kManualMaxSpeedPercentDefault),
      httpArgFloat("minPause", kManualMinPauseSecondsDefault),
      httpArgFloat("maxPause", kManualMaxPauseSecondsDefault),
      httpArgFloat("pauseChance", kManualPauseChancePercentDefault));
  sendManualStatusJson(200);
}

static void handleManualStop() {
  stopManualControl(true);
  sendManualStatusJson(200);
}

static void configureHttpServer() {
  if (!kEnableBrowserMonitor) {
    return;
  }

  g_httpServer.on("/", handleMonitorRoot);
  g_httpServer.on("/api/telemetry", handleMonitorTelemetryJson);
  g_httpServer.on("/api/manual/start", HTTP_POST, handleManualStart);
  g_httpServer.on("/api/manual/stop", HTTP_POST, handleManualStop);
}

static void pollHttpClients() {
  if (!g_httpReady) {
    return;
  }
  g_httpServer.handleClient();
}

static void sendReplyLine(const ReplyContext &context, const String &line) {
  if (context.source == ReplySource::Udp && context.udpPort != 0) {
    g_udp.beginPacket(context.udpIp, context.udpPort);
    g_udp.print(line);
    g_udp.print('\n');
    g_udp.endPacket();
    return;
  }

  Serial.println(line);
}

static String buildD2Response() {
  String response = "D2";
  for (uint8_t i = 0; i < 2; ++i) {
    response += ' ';
    response += SERVO_CONFIGS[i].axisName;
    response += "-0000-9999";
  }
  return response;
}

static bool tryInitServoAtIndex(uint8_t servoIndex) {
  if (servoIndex >= 2) {
    return false;
  }

  ServoRuntime &runtime = g_servoRuntime[servoIndex];
  const uint8_t id = SERVO_CONFIGS[servoIndex].id;

  if (g_servoBus.Ping(id) == -1) {
    runtime.online = false;
    return false;
  }

  runtime.online = true;
  runtime.lastCommandUnits = 32767;
  runtime.lastNonZeroDirection = 0;
  resetServoTelemetry(runtime);

  if (CONFIGURE_CLOSED_LOOP_MODE_ON_BOOT) {
    g_servoBus.WheelMode(id);
    delay(5);
  }

  g_servoBus.EnableTorque(id, 1);
  g_servoBus.WriteSpe(id, 0, 0);
  return true;
}

static void retryOfflineServosIfNeeded() {
  const unsigned long now = millis();
  if ((now - g_lastServoRetryMs) < kServoRetryIntervalMs) {
    return;
  }
  g_lastServoRetryMs = now;

  for (uint8_t i = 0; i < 2; ++i) {
    if (!g_servoRuntime[i].online) {
      tryInitServoAtIndex(i);
    }
  }
}

// ============================================================
// TCode -> RPM -> STS 速度コマンド変換
// ============================================================
static float tcodeToSignedRpm(int tcodeValue, float maxRpm) {
  const int clamped = clampInt(tcodeValue, TCODE_MIN, TCODE_MAX);
  const float limitedMax = maxRpm < 0.0f ? 0.0f : maxRpm;
  const int delta = clamped - TCODE_CENTER;

  if (limitedMax <= 0.0f || abs(delta) <= kCenterDeadbandTCode) {
    return 0.0f;
  }

  if (delta > 0) {
    const float ratio = static_cast<float>(clamped - (TCODE_CENTER + kCenterDeadbandTCode)) /
                        static_cast<float>(TCODE_MAX - (TCODE_CENTER + kCenterDeadbandTCode));
    return ratio * limitedMax;
  }

  const float ratio = static_cast<float>((TCODE_CENTER - kCenterDeadbandTCode) - clamped) /
                      static_cast<float>((TCODE_CENTER - kCenterDeadbandTCode) - TCODE_MIN);
  return -ratio * limitedMax;
}

static int rpmToSpeedUnits(float rpm) {
  const float absRpm = fabsf(rpm);
  if (absRpm <= 0.0f) {
    return 0;
  }

  const float normalized = absRpm / kServoRatedMaxRpm;
  float command = normalized * static_cast<float>(kClosedLoopCommandAtRatedMaxRpm);
  command *= kClosedLoopCommandScale;

  int rounded = static_cast<int>(command + 0.5f);
  if (rounded > 0 && rounded < kClosedLoopMinCommand) {
    rounded = kClosedLoopMinCommand;
  }
  if (rounded > kClosedLoopMaxCommand) {
    rounded = kClosedLoopMaxCommand;
  }

  return (rpm >= 0.0f) ? rounded : -rounded;
}

static float speedUnitsToRpm(int speedUnits) {
  const float command = fabsf(static_cast<float>(speedUnits));
  const float estimatedRpm =
      (command / static_cast<float>(kClosedLoopCommandAtRatedMaxRpm)) * kServoRatedMaxRpm;
  return (speedUnits >= 0) ? estimatedRpm : -estimatedRpm;
}

// ============================================================
// サーボ制御
// ============================================================
static void sendServoSpeed(uint8_t servoIndex, int speedUnits, float targetRpm) {
  if (servoIndex >= 2) {
    return;
  }

  ServoRuntime &runtime = g_servoRuntime[servoIndex];
  if (!runtime.online) {
    return;
  }

  const int requestedDirection = signOfInt(speedUnits);
  if (requestedDirection != 0 &&
      runtime.lastNonZeroDirection != 0 &&
      requestedDirection != runtime.lastNonZeroDirection &&
      abs(speedUnits) < kDirectionChangeMinCommand) {
    speedUnits = requestedDirection * kDirectionChangeMinCommand;
  }

  if (runtime.lastCommandUnits == speedUnits) {
    runtime.targetRpm = targetRpm;
    return;
  }

  runtime.lastCommandUnits = speedUnits;
  runtime.targetRpm = targetRpm;
  g_servoBus.WriteSpe(SERVO_CONFIGS[servoIndex].id,
                      static_cast<s16>(speedUnits),
                      0);

  if (speedUnits != 0) {
    runtime.lastNonZeroDirection = signOfInt(speedUnits);
  }
}

static void stopAllServos() {
  for (uint8_t i = 0; i < 2; ++i) {
    sendServoSpeed(i, 0, 0.0f);
  }
}

static void resetManualChannel(ManualChannelState &channel) {
  channel.phase = ManualPhase::Idle;
  channel.segmentStartTicks = 0;
  channel.targetTurns = 0.0f;
  channel.targetRpm = 0.0f;
  channel.pauseUntilMs = 0;
}

static void stopManualControl(bool stopServos) {
  g_manualControl.requested = false;
  g_manualControl.active = false;
  g_tcode.centerAll();
  for (uint8_t i = 0; i < 2; ++i) {
    resetManualChannel(g_manualControl.channels[i]);
  }
  if (stopServos) {
    stopAllServos();
  }
}

static void initManualControl() {
  g_manualControl.requested = false;
  g_manualControl.active = false;
  g_manualControl.independent = false;
  g_manualControl.minTurns = kManualMinTurnsDefault;
  g_manualControl.maxTurns = kManualMaxTurnsDefault;
  g_manualControl.minSpeedPercent = kManualMinSpeedPercentDefault;
  g_manualControl.maxSpeedPercent = kManualMaxSpeedPercentDefault;
  g_manualControl.minPauseSeconds = kManualMinPauseSecondsDefault;
  g_manualControl.maxPauseSeconds = kManualMaxPauseSecondsDefault;
  g_manualControl.pauseChancePercent = kManualPauseChancePercentDefault;
  for (uint8_t i = 0; i < 2; ++i) {
    resetManualChannel(g_manualControl.channels[i]);
  }
}

static bool manualFeedbackReady(uint8_t servoIndex) {
  if (servoIndex >= 2) {
    return false;
  }
  const ServoRuntime &runtime = g_servoRuntime[servoIndex];
  return runtime.online && runtime.feedbackValid && runtime.hasPositionSample;
}

static float manualSpeedPercentToRpm(uint8_t servoIndex, float speedPercent, int direction) {
  const float limitedPercent = clampFloat(speedPercent, 0.0f, 100.0f);
  float rpm = SERVO_CONFIGS[servoIndex].maxRpm * (limitedPercent / 100.0f);
  rpm *= static_cast<float>(direction);
  rpm *= static_cast<float>(SERVO_CONFIGS[servoIndex].directionSign);
  return rpm;
}

static void beginManualSegment(uint8_t servoIndex, float turns, float speedPercent, int direction) {
  ManualChannelState &channel = g_manualControl.channels[servoIndex];
  channel.phase = ManualPhase::Running;
  channel.segmentStartTicks = g_servoRuntime[servoIndex].relativePositionTicks;
  channel.targetTurns = clampFloat(turns, 0.0f, 100.0f);
  channel.targetRpm = manualSpeedPercentToRpm(servoIndex, speedPercent, direction);
  channel.pauseUntilMs = 0;
}

static void beginRandomManualSegment(uint8_t servoIndex) {
  const float turns = randomFloatInRange(g_manualControl.minTurns, g_manualControl.maxTurns);
  const float speedPercent =
      randomFloatInRange(g_manualControl.minSpeedPercent, g_manualControl.maxSpeedPercent);
  const int direction = random(0, 2) == 0 ? -1 : 1;
  beginManualSegment(servoIndex, turns, speedPercent, direction);
}

static void beginLinkedManualSegments() {
  const float turns = randomFloatInRange(g_manualControl.minTurns, g_manualControl.maxTurns);
  const float speedPercent =
      randomFloatInRange(g_manualControl.minSpeedPercent, g_manualControl.maxSpeedPercent);
  const int direction = random(0, 2) == 0 ? -1 : 1;
  for (uint8_t i = 0; i < 2; ++i) {
    beginManualSegment(i, turns, speedPercent, direction);
  }
}

static void beginManualPause(uint8_t servoIndex, unsigned long now) {
  ManualChannelState &channel = g_manualControl.channels[servoIndex];
  if (randomPercent(g_manualControl.pauseChancePercent)) {
    const float seconds =
        randomFloatInRange(g_manualControl.minPauseSeconds, g_manualControl.maxPauseSeconds);
    channel.phase = ManualPhase::Pause;
    channel.pauseUntilMs = now + static_cast<unsigned long>(seconds * 1000.0f);
    channel.targetRpm = 0.0f;
  } else {
    beginRandomManualSegment(servoIndex);
  }
}

static void beginLinkedManualPause(unsigned long now) {
  if (randomPercent(g_manualControl.pauseChancePercent)) {
    const float seconds =
        randomFloatInRange(g_manualControl.minPauseSeconds, g_manualControl.maxPauseSeconds);
    const unsigned long untilMs = now + static_cast<unsigned long>(seconds * 1000.0f);
    for (uint8_t i = 0; i < 2; ++i) {
      g_manualControl.channels[i].phase = ManualPhase::Pause;
      g_manualControl.channels[i].pauseUntilMs = untilMs;
      g_manualControl.channels[i].targetRpm = 0.0f;
    }
  } else {
    beginLinkedManualSegments();
  }
}

static void startManualControlFromRequest(bool independent,
                                          float minTurns,
                                          float maxTurns,
                                          float minSpeedPercent,
                                          float maxSpeedPercent,
                                          float minPauseSeconds,
                                          float maxPauseSeconds,
                                          float pauseChancePercent) {
  g_manualControl.independent = independent;
  g_manualControl.minTurns = clampFloat(minTurns, 0.0f, 100.0f);
  g_manualControl.maxTurns = clampFloat(maxTurns, 0.0f, 100.0f);
  if (g_manualControl.maxTurns < g_manualControl.minTurns) {
    const float temp = g_manualControl.minTurns;
    g_manualControl.minTurns = g_manualControl.maxTurns;
    g_manualControl.maxTurns = temp;
  }
  g_manualControl.minSpeedPercent = clampFloat(minSpeedPercent, 0.0f, 100.0f);
  g_manualControl.maxSpeedPercent = clampFloat(maxSpeedPercent, 0.0f, 100.0f);
  if (g_manualControl.maxSpeedPercent < g_manualControl.minSpeedPercent) {
    const float temp = g_manualControl.minSpeedPercent;
    g_manualControl.minSpeedPercent = g_manualControl.maxSpeedPercent;
    g_manualControl.maxSpeedPercent = temp;
  }
  g_manualControl.minPauseSeconds = clampFloat(minPauseSeconds, 0.0f, 10.0f);
  g_manualControl.maxPauseSeconds = clampFloat(maxPauseSeconds, 0.0f, 10.0f);
  if (g_manualControl.maxPauseSeconds < g_manualControl.minPauseSeconds) {
    const float temp = g_manualControl.minPauseSeconds;
    g_manualControl.minPauseSeconds = g_manualControl.maxPauseSeconds;
    g_manualControl.maxPauseSeconds = temp;
  }
  g_manualControl.pauseChancePercent = clampFloat(pauseChancePercent, 0.0f, 100.0f);
  g_manualControl.requested = true;
  g_manualControl.active = false;
  for (uint8_t i = 0; i < 2; ++i) {
    resetManualChannel(g_manualControl.channels[i]);
  }
}

static bool allManualFeedbackReady() {
  return manualFeedbackReady(0) && manualFeedbackReady(1);
}

static void applyManualControl() {
  const unsigned long now = millis();
  if (!g_manualControl.requested) {
    return;
  }

  if (!allManualFeedbackReady()) {
    stopManualControl(true);
    return;
  }

  if (!g_manualControl.active) {
    g_manualControl.active = true;
    if (g_manualControl.independent) {
      beginRandomManualSegment(0);
      beginRandomManualSegment(1);
    } else {
      beginLinkedManualSegments();
    }
  }

  if (g_manualControl.independent) {
    for (uint8_t i = 0; i < 2; ++i) {
      ManualChannelState &channel = g_manualControl.channels[i];
      if (channel.phase == ManualPhase::Running) {
        const float completedTurns =
            fabsf(static_cast<float>(g_servoRuntime[i].relativePositionTicks -
                                     channel.segmentStartTicks)) /
            4096.0f;
        if (completedTurns >= channel.targetTurns) {
          sendServoSpeed(i, 0, 0.0f);
          beginManualPause(i, now);
        }
      } else if (channel.phase == ManualPhase::Pause && now >= channel.pauseUntilMs) {
        beginRandomManualSegment(i);
      } else if (channel.phase == ManualPhase::Idle) {
        beginRandomManualSegment(i);
      }

      const int speedUnits = rpmToSpeedUnits(channel.targetRpm);
      sendServoSpeed(i, speedUnits, channel.targetRpm);
    }
    return;
  }

  bool linkedComplete = true;
  bool linkedPaused = true;
  for (uint8_t i = 0; i < 2; ++i) {
    ManualChannelState &channel = g_manualControl.channels[i];
    if (channel.phase == ManualPhase::Running) {
      linkedPaused = false;
      const float completedTurns =
          fabsf(static_cast<float>(g_servoRuntime[i].relativePositionTicks -
                                   channel.segmentStartTicks)) /
          4096.0f;
      if (completedTurns < channel.targetTurns) {
        linkedComplete = false;
      }
    } else if (channel.phase != ManualPhase::Pause) {
      linkedPaused = false;
      linkedComplete = false;
    }
  }

  if (linkedComplete && !linkedPaused) {
    stopAllServos();
    beginLinkedManualPause(now);
  } else if (linkedPaused && now >= g_manualControl.channels[0].pauseUntilMs) {
    beginLinkedManualSegments();
  }

  for (uint8_t i = 0; i < 2; ++i) {
    ManualChannelState &channel = g_manualControl.channels[i];
    const int speedUnits = rpmToSpeedUnits(channel.targetRpm);
    sendServoSpeed(i, speedUnits, channel.targetRpm);
  }
}

static void refreshServoFeedbackIfNeeded() {
  if (!telemetryPollingEnabled()) {
    return;
  }

  const unsigned long now = millis();
  for (uint8_t i = 0; i < 2; ++i) {
    ServoRuntime &runtime = g_servoRuntime[i];
    if (!runtime.online) {
      continue;
    }
    if ((now - runtime.lastFeedbackMs) < telemetryIntervalMs()) {
      continue;
    }

    runtime.lastFeedbackMs = now;
    if (g_servoBus.FeedBack(SERVO_CONFIGS[i].id) == -1) {
      runtime.feedbackValid = false;
      continue;
    }

    runtime.feedbackValid = true;
    runtime.speedUnits = g_servoBus.ReadSpeed(-1);
    runtime.measuredRpm = speedUnitsToRpm(runtime.speedUnits);
    runtime.loadRaw = g_servoBus.ReadLoad(-1);
    runtime.loadPercent = fabsf(static_cast<float>(runtime.loadRaw)) / 10.0f;
    runtime.voltageRaw = g_servoBus.ReadVoltage(-1);
    runtime.voltageVolts = static_cast<float>(runtime.voltageRaw) / 10.0f;
    runtime.temperatureC = g_servoBus.ReadTemper(-1);
    updateDerivedTelemetry(runtime, g_servoBus.ReadPos(-1), now);

    if (kEnableFeedbackTelemetry) {
      Serial.print("ID ");
      Serial.print(SERVO_CONFIGS[i].id);
      Serial.print(" target=");
      Serial.print(runtime.targetRpm, 2);
      Serial.print("rpm speed=");
      Serial.print(runtime.measuredRpm, 2);
      Serial.print("rpm derived=");
      Serial.print(runtime.derivedRpm, 2);
      Serial.print("rpm voltage=");
      Serial.print(runtime.voltageVolts, 1);
      Serial.print("V load=");
      Serial.print(runtime.loadPercent, 1);
      Serial.print("% temp=");
      Serial.print(runtime.temperatureC);
      Serial.println("C");
    }
  }
}

static void updateServoTargets() {
  if (g_manualControl.requested || g_manualControl.active) {
    applyManualControl();
    if (g_manualControl.requested) {
      return;
    }
  }

  for (uint8_t i = 0; i < 2; ++i) {
    const ServoConfig &cfg = SERVO_CONFIGS[i];
    const int axisValue = g_tcode.readAxis(cfg.axisName);
    float rpm = tcodeToSignedRpm(axisValue, cfg.maxRpm);
    rpm *= static_cast<float>(cfg.directionSign);

    const int speedUnits = rpmToSpeedUnits(rpm);
    sendServoSpeed(i, speedUnits, rpm);
  }
}

// ============================================================
// Wi-Fi / UDP 入力
// ============================================================
static bool wifiConfigured() {
  return kEnableWifiUdp &&
         kWifiSsid != nullptr &&
         kWifiPassword != nullptr &&
         kWifiSsid[0] != '\0';
}

static void beginWifiConnection() {
  if (!wifiConfigured()) {
    return;
  }

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setHostname(kWifiHostname);
  WiFi.begin(kWifiSsid, kWifiPassword);
  g_lastWifiRetryMs = millis();
}

static void ensureUdpStarted() {
  if (!wifiConfigured()) {
    return;
  }

  if (WiFi.status() != WL_CONNECTED) {
    if (g_udpReady) {
      g_udp.stop();
      g_udpReady = false;
      g_lastUdpPacketMs = 0;
    }
    if (g_httpReady) {
      g_httpServer.stop();
      g_httpReady = false;
    }
    if (g_mdnsReady) {
      MDNS.end();
      g_mdnsReady = false;
    }
    return;
  }

  if (!g_udpReady) {
    g_udp.begin(kUdpPort);
    g_udpReady = true;
  }

  if (kEnableBrowserMonitor && !g_httpReady) {
    g_httpServer.begin();
    g_httpReady = true;
  }

  if (!g_mdnsReady) {
    if (MDNS.begin(kWifiHostname)) {
      MDNS.addService("tcode", "udp", kUdpPort);
      if (kEnableBrowserMonitor) {
        MDNS.addService("http", "tcp", kHttpPort);
      }
      g_mdnsReady = true;
    }
  }
}

static void maintainWifi() {
  if (!wifiConfigured()) {
    return;
  }

  if (WiFi.status() == WL_CONNECTED) {
    ensureUdpStarted();
    return;
  }

  g_udpReady = false;
  g_lastUdpPacketMs = 0;
  g_httpReady = false;
  if (g_mdnsReady) {
    MDNS.end();
    g_mdnsReady = false;
  }

  const unsigned long now = millis();
  if ((now - g_lastWifiRetryMs) >= kWifiRetryIntervalMs) {
    WiFi.disconnect();
    WiFi.begin(kWifiSsid, kWifiPassword);
    g_lastWifiRetryMs = now;
  }
}

static void pollUdpInput() {
  if (!g_udpReady) {
    return;
  }

  int packetSize = g_udp.parsePacket();
  while (packetSize > 0) {
    g_lastUdpPacketMs = millis();
    g_tcode.setReplyContextUdp(g_udp.remoteIP(), g_udp.remotePort());
    int length = g_udp.read(g_udpBuffer, sizeof(g_udpBuffer));
    if (length > 0 && !g_manualControl.requested) {
      for (int i = 0; i < length; ++i) {
        g_tcode.inputByte(g_udpBuffer[i]);
      }
    }
    packetSize = g_udp.parsePacket();
  }
}

// ============================================================
// USB シリアル入力
// ============================================================
static void pollSerialInput() {
  if (!kEnableUsbSerialInput) {
    return;
  }

  g_tcode.setReplyContextSerial();
  while (Serial.available() > 0) {
    const uint8_t byteValue = static_cast<uint8_t>(Serial.read());
    if (!g_manualControl.requested) {
      g_tcode.inputByte(byteValue);
    }
  }
}

// ============================================================
// サーボ初期化
// ============================================================
static void initServoBus() {
  SERVO_SERIAL.begin(SERVO_BUS_BAUD, SERIAL_8N1, SERVO_BUS_RX_PIN, SERVO_BUS_TX_PIN);
  g_servoBus.pSerial = &SERVO_SERIAL;
  delay(kServoStartupDelayMs);

  for (uint8_t i = 0; i < 2; ++i) {
    g_servoRuntime[i].online = false;
    g_servoRuntime[i].lastCommandUnits = 32767;
    g_servoRuntime[i].lastNonZeroDirection = 0;
    resetServoTelemetry(g_servoRuntime[i]);
    tryInitServoAtIndex(i);
  }
}

// ============================================================
// Arduino 初期化 / メインループ
// ============================================================
void setup() {
  if (kEnableUsbSerialInput || kEnableFeedbackTelemetry) {
    Serial.begin(kUsbSerialBaud);
    delay(100);
  }

  randomSeed(static_cast<unsigned long>(micros()));
  initManualControl();
  configureHttpServer();
  initServoBus();
  beginWifiConnection();

  if (wifiConfigured()) {
    const unsigned long startMs = millis();
    while (WiFi.status() != WL_CONNECTED &&
           (millis() - startMs) < kWaitForWifiOnBootMs) {
      delay(50);
    }
  }

  ensureUdpStarted();
  stopAllServos();
  g_lastControlMs = millis();
}

void loop() {
  pollSerialInput();
  maintainWifi();
  pollHttpClients();
  pollUdpInput();
  retryOfflineServosIfNeeded();

  const unsigned long now = millis();
  if ((now - g_lastControlMs) >= controlIntervalMs()) {
    g_lastControlMs = now;
    updateServoTargets();
    refreshServoFeedbackIfNeeded();
  }
}
