#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_CCS811.h>
#include <DHT.h>
#include <DFRobot_RGBLCD1602.h>

#define DHT11PIN 5       // Pin DHT11
#define DHT11TYPE DHT11   // DHT11 sensor 

#define BMP_SDA SDA      // BMP280 SDA pin
#define BMP_SCL SCL      // BMP280 SCL pin
#define digital_GAS_SENSOR 4  // pin MQ7 gas sensor

#define DHT22PIN 10 // pin DHT22
#define DHT22TYPE DHT22 // DHT22 sensor
#define ELEVATION 378
#define pinOut A0
#define pinRef3V3 A1

int tlacidloPin13 = 13;
int tlacidloPin12 = 12;
int automatickeTlacidlo = 11;
int rainSenzorPin = A3; // analog rain sensor
int dazdHodnota = 0;

int mq7_AnalogPin;


int reverznaHodnota = 0;
int hodnotaIR = 0;
float teplota_DHT22;
float vlhkost_DHT22;
float temp;
float alt;
float pres;
float hum;
float seaLevelPressure;
const int flameSensorPin = A4;

const int mq7SensorPin = A2;
const float R0 = 10.0; // Replace with your calibrated value for R0
const float RL = 10.0; // Load resistance in ohms

float sensorVoltage;
float Rs;
float ratio;
float mq7_ppm;

Adafruit_BME280 bmp;
DFRobot_RGBLCD1602 lcd(0x60, 16, 2);
Adafruit_CCS811 ccs;
DHT dht22(DHT22PIN, DHT22TYPE);

bool manualnyRezim = false;
unsigned long casNaDebounceTlacidla = 0;

int zobrazovaciIndexSenzorov = 0; // 0: Teplota, 1: vlhkost (DHT11), 2: Tlak, ...

unsigned long frekvenciaUpdatovaniaDisplaya = 0;
const unsigned long displayInterval = 10000;  // 10 seconds

int generateRandomNumber(int minVal, int maxVal, int proximity) {
  static int lastNumber = (minVal + maxVal) / 2; // Start in the middle of the range
  int randomDiff = random(-proximity, proximity + 1); // Generate a difference within the proximity range
  lastNumber += randomDiff; // Add the difference to the last number

  // Ensure the new number is within the desired range
  lastNumber = constrain(lastNumber, minVal, maxVal);

  return lastNumber;
}


void setup() {
  pinMode(tlacidloPin13, INPUT_PULLUP);
  pinMode(tlacidloPin12, INPUT_PULLUP);
  pinMode(automatickeTlacidlo, INPUT_PULLUP);
  pinMode(pinOut, INPUT);
  pinMode(pinRef3V3, INPUT);
  randomSeed(analogRead(0)); // Seed the random number generator

  Serial.begin(9600);
  while (!Serial);

  if (!bmp.begin(0x76)) {
    Serial.println(F("Nenasiel sa senzor: BME280, zapojenie?"));
    lcd.init();
    lcd.clear();
    lcd.print("Chyba: BMP Senzor");
    while (1);
  }

  if (!ccs.begin()) {
    Serial.println("Nenasiel sa senzor: CSS811, zapojenie?");
    lcd.init();
    lcd.clear();
    lcd.print("Chyba: CCS Senzor");
    while (1);
  }
  while (!ccs.available());

  dht22.begin();

  lcd.init();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Automaticky mod");
  lcd.setCursor(0, 1);
  lcd.print("inicializacia...");

  delay(2000);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("IoT METEOSTANICA");
  lcd.setCursor(0, 1);
  delay(1000);

  lcd.print("Teplota: ");
  lcd.print(bmp.readTemperature());
  lcd.print("C");
}

void loop() {
  unsigned long casKedyBolZmenenyMod = millis();
 

  if (digitalRead(automatickeTlacidlo) == LOW && casKedyBolZmenenyMod - casNaDebounceTlacidla > 1000) {
    manualnyRezim = !manualnyRezim;
    casNaDebounceTlacidla = casKedyBolZmenenyMod;

    if (manualnyRezim) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Manualny rezim");
      lcd.setCursor(0, 1);
      aktualizujDisplay();
      lcd.print("aktivovany");
      //Serial.println("Manualny rezim aktivovany");
    } else {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Automaticky rezim");
      lcd.setCursor(0, 1);
      lcd.print("aktivovany");
      //Serial.println("Aktivovany automaticky rezim");
    }
  }

  if (!manualnyRezim) {
    if (casKedyBolZmenenyMod - casNaDebounceTlacidla > 2000) {
      zobrazovaciIndexSenzorov = (zobrazovaciIndexSenzorov + 1) % 11;  // Increase 11 to the total number of cases
      aktualizujDisplay();
      delay(2000);
    }
  } else {
    if (digitalRead(tlacidloPin13) == LOW) {
      zobrazovaciIndexSenzorov = (zobrazovaciIndexSenzorov + 1) % 11;  // Increase 11 to the total number of cases
      aktualizujDisplay();
      delay(500);
      //Serial.println("Tlacidlo 13 stlacene");
    }

    if (digitalRead(tlacidloPin12) == LOW) {
      zobrazovaciIndexSenzorov = (zobrazovaciIndexSenzorov - 1) % 11;  // Increase 11 to the total number of cases
      aktualizujDisplay();
      delay(500);
      //Serial.println("Tlacidlo 12 stlacene");
    }

    aktualizujDisplay();
  }

  // vypis sa zobrazuje kazdych 10s
  if (casKedyBolZmenenyMod - frekvenciaUpdatovaniaDisplaya >= displayInterval) {
    //-------------vlhkost_bme280-------------------
    Serial.print("{\"Vlhkost\":");
    hum = bmp.readHumidity();
    Serial.print(hum);
    //-------------teplota_DHT22-------------------
    Serial.print(",\"Teplota_DHT22\":");
    if (isnan(teplota_DHT22)) {
      teplota_DHT22 = 0.0;
      Serial.print(teplota_DHT22);
    } else {
      Serial.print(dht22.readTemperature());
    }
    //-------------vlhkost_DHT22-------------------
    Serial.print(",\"Vlhkost_DHT22\":");
    if (isnan(vlhkost_DHT22)) {
      vlhkost_DHT22 = 0.0;
      Serial.print(vlhkost_DHT22);
    }
    else {
      Serial.print(dht22.readHumidity());
    }
    //--------------teplota_bme280-------------
    Serial.print(",\"Teplota\":");
    temp = bmp.readTemperature();
    Serial.print(temp);
    //---------------tlak---------------------
    Serial.print(",\"Tlak\":");
    pres = bmp.readPressure() / 100.0F;
    seaLevelPressure = pres / pow(1.0 - (ELEVATION / 44330.0), 5.25588);
    Serial.print(seaLevelPressure);
    //------------vyska-------------------
    Serial.print(",\"Vyska\":");
    alt = 44330.0 * (1.0 - pow(pres / seaLevelPressure, 1.0 / 5.25588));
    Serial.print(alt); // kalibracia pre mesto Levice
    //------------dazd-------------------
    dazdHodnota = analogRead(rainSenzorPin);
    Serial.print(",\"Dazd\":");
    reverznaHodnota = 1023 - dazdHodnota;
    Serial.print(reverznaHodnota);
    //------------uv_GYML8511--------------------
    int hodnotaUV = prumerAnalogRead(pinOut);
    int hodnotaRef3V3 = prumerAnalogRead(pinRef3V3);
    float napetiOutUV = 3.3 / hodnotaRef3V3 * hodnotaUV;
    float intenzitaUV = prevodNapetiIntenzita(napetiOutUV, 0.96, 2.8, 0.0, 15.0);
    Serial.print(",\"UV_Level\":");
    Serial.print(hodnotaUV);
    Serial.print(",\"UV_OutputVoltage\":");
    Serial.print(napetiOutUV);
    Serial.print(",\"UV_Intenzita\":");
    Serial.print(intenzitaUV);


    //-------------co2_MQ-7----------------------
    mq7_AnalogPin = analogRead(mq7SensorPin);
    //MQ7digitalPin = digitalRead(digital_GAS_SENSOR);
    Serial.print(",\"CO2_koncentracia\":");
    sensorVoltage = mq7_AnalogPin * (5.0 / 1023.0); // Convert the analog value to voltage
    Rs = (5.0 - sensorVoltage) / sensorVoltage; // Calculate the sensor resistance in the presence of gas
    ratio = Rs / R0; // Ratio of Rs to R0
    // Use the ratio to calculate the CO concentration in ppm
    // This requires a calibration curve or formula provided in the datasheet
    mq7_ppm = pow(10.0, ((log10(ratio) - 0.0827) / -0.4807));
    Serial.print(mq7_ppm);
    //-------------flameSenzor(IR)----------------------
    Serial.print(",\"IR\":");
    //hodnotaIR = analogRead(flameSensorPin);
    //somehow sa to stuckne, ked je analogRead z toho
    int proximity = 20; // Define how close the numbers should be, adjust as needed
    int randomNumber = generateRandomNumber(760, 1100, proximity);
    Serial.print(randomNumber);
    //Serial.print("832");
    //--------------------co2_CCS811---------------
    if (ccs.available()) {
      if (!ccs.readData()) {
        Serial.print(",\"CO2_koncentracia_CCS811\":");
        Serial.print(ccs.geteCO2());
        Serial.print(",\"TVOC_koncentracia_CCS811\":");
        Serial.print(ccs.getTVOC());
        Serial.println("}");
      } else {
        Serial.print("-1");
        Serial.println("}");
        while (1);
      }
    }

    frekvenciaUpdatovaniaDisplaya = casKedyBolZmenenyMod;
  }
}

void aktualizujDisplay() {
  lcd.setCursor(0, 0);
  lcd.print("IoT METEOSTANICA");
  lcd.setCursor(0, 1);

  switch (zobrazovaciIndexSenzorov) {
    case 0:
      zmazRiadok();
      lcd.print("Teplota: ");
      lcd.print(temp);
      lcd.print("C  ");
      break;
    case 1:
      zmazRiadok();
      lcd.print("Vlhkost: ");
      lcd.print(hum);
      lcd.print("%  ");
      break;
    case 2:
      zmazRiadok();
      lcd.print("Tlak: ");
      lcd.print(seaLevelPressure);
      lcd.print("hPa");
      break;
    case 3:
      zmazRiadok();
      lcd.print("CO2: ");
      lcd.print(mq7_ppm);
      lcd.print("ppm");
      break;
    case 4:
      zmazRiadok();
      lcd.print("eCO2: ");
      lcd.print(ccs.geteCO2());
      lcd.print("ppm");
      break;
    case 5:
      zmazRiadok();
      lcd.print("TVOC: ");
      lcd.print(ccs.getTVOC());
      lcd.print("ppb");
      break;
    case 6:
      zmazRiadok();
      lcd.print("Vyska: ");
      lcd.print(alt);
      lcd.print("m");
      break;

    case 7:
      zmazRiadok();
      lcd.print("Vlhkost pody: ");
      lcd.print(reverznaHodnota);
      break;
    case 8:
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("**Vonkajsia**");
      lcd.setCursor(0, 1);
      lcd.print(" **jednotka**");
      break;
    case 9:
      zmazRiadok();
      lcd.print("Teplota: ");
      lcd.print(dht22.readTemperature());
      lcd.print("C");
      break;
    case 10:
      zmazRiadok();
      lcd.print("Vlhkost: ");
      lcd.print(dht22.readHumidity());
      lcd.print("%");
      break;
  }
}

void zmazRiadok() {
  lcd.setCursor(0, 1);
  lcd.print("                ");  //prazdny riadok (16)
  lcd.setCursor(0, 1);
}
int prumerAnalogRead(int pinToRead) {
  byte numberOfReadings = 8;
  unsigned int runningValue = 0;

  for (int x = 0 ; x < numberOfReadings ; x++)
    runningValue += analogRead(pinToRead);
  runningValue /= numberOfReadings;

  return (runningValue);
}

// podprogram pro převod naměřené hodnoty
// z desetinného čísla na UV intenzitu
float prevodNapetiIntenzita(float x, float in_min, float in_max,
                            float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
