// --- L298N 驅動板 1 (控制馬達 1) ---
const int ENA1 = 25; 
const int IN1  = 26;  
const int IN2  = 27; 

// --- L298N 驅動板 2 (控制馬達 2) ---
const int ENA2 = 33; // 接到第二塊板子的 ENA
const int IN3  = 21; // 接到第二塊板子的 IN1
const int IN4  = 14; // 接到第二塊板子的 IN2

// --- 編碼器引腳定義 ---
const int enc1A = 34; // 馬達 1 編碼器 (黃)
const int enc1B = 35; // 馬達 1 編碼器 (白)
const int enc2A = 18; // 馬達 2 編碼器 (黃)
const int enc2B = 19; // 馬達 2 編碼器 (白)

// --- PWM 設定 (相容 v3.0) ---
const int freq = 5000;
const int res  = 8;

// --- 變數定義 ---
volatile long ticks1 = 0;
volatile long ticks2 = 0;
char command;

int ch1 = 1;
int ch2 = 2;


// --- 中斷服務程序 (ISR) ---
void IRAM_ATTR readEnc1() {
  if (digitalRead(enc1A) == digitalRead(enc1B)) ticks1++; else ticks1--;
}
void IRAM_ATTR readEnc2() {
  if (digitalRead(enc2A) == digitalRead(enc2B)) ticks2++; else ticks2--;
}

void setup() {
  Serial.begin(115200);

  // 初始化引腳
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(enc1A, INPUT); pinMode(enc1B, INPUT);
  pinMode(enc2A, INPUT); pinMode(enc2B, INPUT);

  // 【更新】ESP32 Arduino v3.0 PWM 初始化方式
  // 直接將引腳與頻率、解析度綁定，系統會自動分配內部頻道
  ledcSetup(ch1, freq, res);
  ledcSetup(ch2, freq, res);      
  ledcAttachPin(ENA1, ch1);
  ledcAttachPin(ENA2, ch2);

  // 設置中斷
  attachInterrupt(digitalPinToInterrupt(enc1A), readEnc1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(enc2A), readEnc2, CHANGE);

  Serial.println("System Ready! (Two Drivers Mode - ESP32 v3.0)");
}

void loop() {
  if (Serial.available() > 0) {
    command = Serial.read();

    if (command == 'w') {      // 前進
      controlMotors(255, 255);
      Serial.println("Forward");
    } 
    else if (command == 's') { // 後退
      controlMotors(-255, -255);
      Serial.println("Backward");
    } 
    else if (command == 'a') { // 左轉
      controlMotors(255, -255);
      Serial.println("Turn Left");
    } 
    else if (command == 'd') { // 右轉
      controlMotors(-255, 255);
      Serial.println("Turn Right");
    } 
    else if (command == 'r') { // 停止
      controlMotors(0, 0);
      Serial.println("Stop");
    }
  }

  // 定期顯示編碼器數據
  static unsigned long lastTime = 0;
  if (millis() - lastTime > 500) {
    Serial.printf("Ticks M1: %ld | Ticks M2: %ld\n", ticks1, ticks2);
    lastTime = millis();
  }
}

// --- 控制馬達的通用函式 ---
void controlMotors(int speed1, int speed2) {
  // 控制馬達 1 (Driver 1)
  if (speed1 > 0) {
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
    ledcWrite(ch1, speed1);
  } else if (speed1 < 0) {
    digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
    ledcWrite(ch1, abs(speed1));
  } else {
    digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
    ledcWrite(ch1, 0);
  }

  // 控制馬達 2 (Driver 2)
  if (speed2 > 0) {
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
    ledcWrite(ch2, speed2); 
  } else if (speed2 < 0) {
    digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
    ledcWrite(ch2, abs(speed2));
  } else {
    digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
    ledcWrite(ch2, 0);
  }
}