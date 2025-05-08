#define IR_RECEIVE_PIN 15  // 紅外線接收器 OUT 腳接到這腳

int lastValue = HIGH;
unsigned long lastChange = 0;

void setup() {
  pinMode(IR_RECEIVE_PIN, INPUT);
  Serial.begin(115200);
  delay(1000);
  Serial.println("🔍 開始監控 IR 波形變化（GPIO 15）");
}

void loop() {
  int currentValue = digitalRead(IR_RECEIVE_PIN);

  if (currentValue != lastValue) {
    unsigned long now = micros();
    unsigned long duration = now - lastChange;
    lastChange = now;

    Serial.print("變化：");
    Serial.print(currentValue == LOW ? "📥 LOW  " : "📤 HIGH ");
    Serial.print(" | 間隔：");
    Serial.print(duration);
    Serial.println(" μs");

    lastValue = currentValue;
  }
}
