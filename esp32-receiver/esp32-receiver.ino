// RCJ Vision 受信サンプル (XIAO ESP32S3 / Arduino IDE)
// Tools -> USB CDC On Boot: Enabled にして書き込む。
// スマホ(OTG) → このUSBポートへ、1行1パケットのNMEA風テキストが届く。
//
//   $IMU,t_ms,heading,roll,pitch,ax,ay,az,gx,gy,gz,egoVx,egoVy*CS
//   $OBJ,t_ms,cls,id,cam,x,y,vx,vy,x500,y500,tca_ms,dca_mm,cross_ms,crossX,conf*CS
//     cls: B=ボール Y=黄ゴール U=青ゴール A=味方 O=相手
//     x,y: スマホ原点[mm] (+Y=背面カメラ方向, +X=右) / v:[mm/s] / x500,y500: 0.5秒後の予測
//     tca_ms/dca_mm: スマホ原点への最接近までの時間と距離
//     cross_ms/crossX: Y=0ラインを横切るまでの時間とそのX (横切らない場合は -1)
//   CS は '$' と '*' の間の文字のXOR(16進2桁)

struct Obj {
  char cls;
  int id, cam, x, y, vx, vy, x500, y500, tca, dca, crossMs, crossX, conf;
  uint32_t seenMs;
};

struct Imu {
  float heading, roll, pitch, ax, ay, az, gx, gy, gz, evx, evy;
  uint32_t seenMs;
};

Obj ball;       // 最新のボール
Imu imu;
uint32_t okCount = 0, badCount = 0;

static bool checkLine(const String &line, String &body) {
  int st = line.indexOf('$');
  int en = line.lastIndexOf('*');
  if (st < 0 || en < st || en + 3 > (int)line.length()) return false;
  body = line.substring(st + 1, en);
  uint8_t cs = 0;
  for (size_t i = 0; i < body.length(); i++) cs ^= (uint8_t)body[i];
  return cs == (uint8_t)strtol(line.substring(en + 1, en + 3).c_str(), nullptr, 16);
}

static void handle(const String &body) {
  if (body.startsWith("OBJ,")) {
    Obj o;
    char cls;
    long t;
    int n = sscanf(body.c_str(), "OBJ,%ld,%c,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
                   &t, &cls, &o.id, &o.cam, &o.x, &o.y, &o.vx, &o.vy, &o.x500, &o.y500,
                   &o.tca, &o.dca, &o.crossMs, &o.crossX, &o.conf);
    if (n == 15) {
      o.cls = cls;
      o.seenMs = millis();
      if (cls == 'B') ball = o;   // ここでロボット制御に使う
    }
  } else if (body.startsWith("IMU,")) {
    long t;
    int n = sscanf(body.c_str(), "IMU,%ld,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f",
                   &t, &imu.heading, &imu.roll, &imu.pitch, &imu.ax, &imu.ay, &imu.az,
                   &imu.gx, &imu.gy, &imu.gz, &imu.evx, &imu.evy);
    if (n == 12) imu.seenMs = millis();
  }
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  // 起動確認: 3回点滅 (XIAO ESP32S3 のLEDは負論理: LOW=点灯)
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_BUILTIN, LOW); delay(150);
    digitalWrite(LED_BUILTIN, HIGH); delay(150);
  }
  Serial.setRxBufferSize(2048);  // begin() より前に設定
  Serial.setTxTimeoutMs(0);      // ホストが読まなくても送信でブロックしない
  Serial.begin(115200);          // ネイティブUSB CDC(ボーレートは無視される)
}

String line;

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      String body;
      if (checkLine(line, body)) { handle(body); okCount++; } else if (line.length()) { badCount++; }
      line = "";
    } else if (c != '\r' && line.length() < 200) {
      line += c;
    }
  }

  // 動作確認: 有効なパケットを受信中は点滅、ボールが見えている間は点灯
  uint32_t now = millis();
  bool ballSeen = (now - ball.seenMs) < 300 && ball.seenMs != 0;
  if (ballSeen) digitalWrite(LED_BUILTIN, LOW);
  else digitalWrite(LED_BUILTIN, (okCount / 30) % 2 ? LOW : HIGH);

  // 1秒ごとに状況を返す(スマホ側は読み捨てる。PC接続時のデバッグ用)
  static uint32_t last = 0;
  if (now - last > 1000) {
    last = now;
    Serial.printf("ok=%lu bad=%lu ball=(%d,%d) v=(%d,%d) heading=%.0f\n",
                  (unsigned long)okCount, (unsigned long)badCount,
                  ball.x, ball.y, ball.vx, ball.vy, imu.heading);
  }
}
