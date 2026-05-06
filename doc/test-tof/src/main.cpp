#include <Arduino.h>
#include <Wire.h>
#include <VL53L1X.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_SSD1306.h>

// Broches XSHUT pour les 3 capteurs ToF
const uint8_t shutPins[] = {13, 14, 27}; 
const uint8_t numToFs = 3;
VL53L1X tofs[numToFs];

// Objets pour l'IMU et l'Écran OLED
Adafruit_MPU6050 mpu;
Adafruit_SSD1306 display(128, 64, &Wire, -1);

void setup() {
  Serial.begin(115200);
  
  // Initialisation du bus I2C sur les broches 21/22
  Wire.begin(21, 22); 
  Wire.setClock(400000); // Mode rapide 400kHz pour la réactivité

  // 1. Initialisation de l'écran OLED (Adresse par défaut 0x3C)
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("Erreur : Ecran OLED non detecte");
  }
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);

  // 2. Initialisation de l'IMU MPU6050 (Adresse 0x68)
  if (!mpu.begin()) {
    Serial.println("Erreur : IMU MPU6050 non detectee");
  }

  // 3. Procédure d'adressage séquentiel des ToF[cite: 1]
  // On place tous les ToF en mode Shutdown (Hardware Standby)[cite: 1]
  for (int i=0; i < numToFs; i++) {
    pinMode(shutPins[i], OUTPUT);
    digitalWrite(shutPins[i], LOW);
  }
  delay(10);

  // On réveille et on change l'adresse de chaque ToF un par un[cite: 1]
  for (int i=0; i < numToFs; i++) {
    digitalWrite(shutPins[i], HIGH);
    delay(2); // Attente du boot du capteur (max 1.2ms)[cite: 1]
    
    tofs[i].setTimeout(500);
    if (!tofs[i].init()) {
      Serial.printf("Erreur d'initialisation ToF %d\n", i);
      while(1); // Bloque ici si un ToF est mal branche
    }
    
    // On change l'adresse 0x52 d'origine pour éviter les conflits[cite: 1]
    // Les nouvelles adresses seront : 0x54, 0x56, 0x58[cite: 1]
    tofs[i].setAddress(0x54 + (i * 2)); 
    
    // Configuration optimale pour un robot (Mode Court)[cite: 1]
    tofs[i].setDistanceMode(VL53L1X::Short); // Max ~1.3m, plus stable[cite: 1]
    tofs[i].setMeasurementTimingBudget(33000); // 33ms[cite: 1]
    tofs[i].startContinuous(50); // Mesure toutes les 50ms[cite: 1]
  }

  Serial.println("Systeme pret : Ecran, IMU et 3 ToF OK !");
}

void loop() {
  display.clearDisplay();
  display.setCursor(0,0);
  display.println("--- QUADRUPEDE DATA ---");

  // Lecture et affichage des distances ToF[cite: 1]
  for (int i=0; i < numToFs; i++) {
    uint16_t dist = tofs[i].read();
    display.printf("ToF %d: %d mm\n", i, dist);
    
    // Affichage série pour débug[cite: 1]
    Serial.printf("ToF%d: %d | ", i, dist);
  }
  Serial.println();

  // Lecture des données de l'IMU
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  display.printf("AccX: %.2f m/s2\n", a.acceleration.x);
  display.printf("AccY: %.2f m/s2\n", a.acceleration.y);

  display.display();
  delay(20); // Petite pause pour la fluidité de l'affichage
}