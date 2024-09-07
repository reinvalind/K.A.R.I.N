#define BLYNK_TEMPLATE_ID "TMPL6MwNPwUAe"
#define BLYNK_TEMPLATE_NAME "MBC HME"
#define BLYNK_AUTH_TOKEN "NNZswf32h6oXEfupi8b7klPLz7H3TfTM"

// Include the library files
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>

char auth[] = "NNZswf32h6oXEfupi8b7klPLz7H3TfTM"; // Enter your Auth token
char ssid[] = "Iven";                           // Enter your WIFI name
char pass[] = "cenz5262";                      // Enter your WIFI password

BlynkTimer timer;

// Water Quality Sensor Variables
#define TdsSensorPin A0
#define VREF 3.3
#define SCOUNT  30
int analogBuffer[SCOUNT];
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0;
int copyIndex = 0;
float averageVoltage = 0;
float tdsValue = 0;
float temperature = 23;

// Ultrasonic Sensor Variables
#define trig D7
#define echo D6
#define relay 5
#define pumpPin D2  // Pin for the additional pump
#define additionalPumpPin D3  // Pin for the additional pump connected to relay
#define additionalPumpPin2 D0  // Pin for the additional pump 2


int MaxLevel = 20;
int Level1 = 5;
int Level2 = (MaxLevel * 65) / 100;
int Level3 = (MaxLevel * 55) / 100;
int Level4 = (MaxLevel * 45) / 100;
int Level5 = (MaxLevel * 35) / 100;
int switchStatus = 0;
int additionalPumpStatus = 0;  // Variable to store additional pump status
int additionalPumpStatus2 = 0;  // Variable to store additional pump 2 status


// Threshold value for pump activation
int thresholdValue = 600;

void setup() {
  Serial.begin(115200);
  pinMode(TdsSensorPin, INPUT);
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);
  pinMode(relay, OUTPUT);
  pinMode(pumpPin, OUTPUT);  // Set the pump pin as an output
  pinMode(additionalPumpPin, OUTPUT);  // Set the additional pump pin as an output
  digitalWrite(relay, HIGH);
  digitalWrite(pumpPin, HIGH);  // Initially turn off the pump
  digitalWrite(additionalPumpPin, HIGH);  // Initially turn off the additional pump
  // Set pin D0 as an output for additional pump 2
  pinMode(additionalPumpPin2, OUTPUT);
  digitalWrite(additionalPumpPin2, HIGH);  // Initially turn off the additional pump 2
  Blynk.begin(auth, ssid, pass, "blynk.cloud", 80);

  // Call the functions
  timer.setInterval(100L, readWaterQuality);
  timer.setInterval(100L, ultrasonic);

  // Tambahkan fungsi untuk membaca nilai ambang batas dari Blynk
  Blynk.virtualWrite(V3, thresholdValue);  // Kirim nilai awal ke widget di aplikasi Blynk
  Blynk.syncVirtual(V3);  // Synchronize nilai dari widget di aplikasi Blynk
}

void readWaterQuality() {
  analogBuffer[analogBufferIndex] = analogRead(TdsSensorPin);
  analogBufferIndex++;
  if (analogBufferIndex == SCOUNT) {
    analogBufferIndex = 0;
  }

  for (copyIndex = 0; copyIndex < SCOUNT; copyIndex++) {
    analogBufferTemp[copyIndex] = analogBuffer[copyIndex];
  }

  averageVoltage = getMedianNum(analogBufferTemp, SCOUNT) * (float)VREF / 1024.0;
  float compensationCoefficient = 1.0 + 0.02 * (temperature - 25.0);
  float compensationVoltage = averageVoltage / compensationCoefficient;
  tdsValue = (133.42 * compensationVoltage * compensationVoltage * compensationVoltage - 255.86 * compensationVoltage * compensationVoltage + 857.39 * compensationVoltage) * 0.5;

  Blynk.virtualWrite(V2, tdsValue);  // Kirim nilai TDS ke widget di aplikasi Blynk

  // Perbarui nilai ambang batas dari widget di aplikasi Blynk
  Blynk.run();

  // Additional condition for the pump
  if (tdsValue >= thresholdValue) {
    digitalWrite(pumpPin, HIGH);  // Turn off the pump
  } else {
    digitalWrite(pumpPin, LOW);   // Turn on the pump
  }
  
  // Additional condition for the additional pump 2
  if (tdsValue >= thresholdValue && thresholdValue != 0) {
    digitalWrite(additionalPumpPin2, LOW);  // Turn on additional pump 2
  } else {
    digitalWrite(additionalPumpPin2, HIGH);  // Turn off additional pump 2
  }

  // Control additional pump based on virtual pin V4
  if (additionalPumpStatus == 1) {
    digitalWrite(additionalPumpPin, LOW);  // Turn on the additional pump
  } else {
    digitalWrite(additionalPumpPin, HIGH);   // Turn off the additional pump
  }

  Serial.print("TDS Value:");
  Serial.print(tdsValue, 0);
  Serial.println("ppm");
}

int getMedianNum(int bArray[], int iFilterLen) {
  // Median filtering algorithm
  int bTab[iFilterLen];
  for (byte i = 0; i < iFilterLen; i++)
    bTab[i] = bArray[i];
  int i, j, bTemp;
  for (j = 0; j < iFilterLen - 1; j++) {
    for (i = 0; i < iFilterLen - j - 1; i++) {
      if (bTab[i] > bTab[i + 1]) {
        bTemp = bTab[i];
        bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
      }
    }
  }
  if ((iFilterLen & 1) > 0) {
    bTemp = bTab[(iFilterLen - 1) / 2];
  } else {
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
  }
  return bTemp;
}

void ultrasonic() {
  if (switchStatus == 1) {
    digitalWrite(trig, LOW);
    delayMicroseconds(4);
    digitalWrite(trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig, LOW);
    long t = pulseIn(echo, HIGH);
    int distance = t / 29 / 2;

    int blynkDistance = (distance - MaxLevel) * -1;
    if (distance <= MaxLevel) {
      Blynk.virtualWrite(V0, blynkDistance);
    } else {
      Blynk.virtualWrite(V0, 0);
    }

    Serial.println("Ultrasonic Distance: ");
    Serial.print(distance);
    Serial.println();

    if (Level1 <= distance) {
      digitalWrite(relay, LOW);
    } else if (Level2 <= distance && Level1 > distance) {
      digitalWrite(relay, HIGH);
    } else if (Level3 <= distance && Level2 > distance) {
      digitalWrite(relay, HIGH);
    } else if (Level4 <= distance && Level3 > distance) {
      digitalWrite(relay, HIGH);
    } else if (Level5 >= distance) {
      digitalWrite(relay, HIGH);
    }
  } else {
    digitalWrite(relay, HIGH);
  }
}

BLYNK_WRITE(V1) {
  // Fungsi ini akan dipanggil setiap kali nilai dari widget berubah
  switchStatus = param.asInt();
}

BLYNK_WRITE(V3) {
  // Fungsi ini akan dipanggil setiap kali nilai dari widget berubah
  thresholdValue = param.asInt();  // Perbarui nilai ambang batas dengan nilai dari widget
}

BLYNK_WRITE(V4) {
  // Fungsi ini akan dipanggil setiap kali nilai dari widget berubah
  additionalPumpStatus = param.asInt();  // Perbarui status pompa tambahan dengan nilai dari widget
}

void loop() {
  Blynk.run();
  timer.run();
}

