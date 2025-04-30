#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SPI.h>
#include <WiFi.h>
#include <esp_now.h>

#define BUZZER_PIN 0

#define LCD_MOSI 23
#define LCD_SCK 18
#define LCD_CS 15
#define LCD_DC 2
#define LCD_RST 4
#define LCD_BLK 32

const int textSize = 3;
const int colWidth = 160;
const int rowHeight = 56;   // Enough to fit 32px text + padding
const int screenWidth = 320;
const int screenHeight = 170;


uint32_t vehicle1Timer = 0;
uint32_t vehicle2Timer = 0;
uint32_t vehicle3Timer = 0;
uint32_t vehicle4Timer = 0;
uint32_t vehicle5Timer = 0;
uint32_t vehicle6Timer = 0;

uint32_t vehicle1Delay = 0;
uint32_t vehicle2Delay = 0;
uint32_t vehicle3Delay = 0;
uint32_t vehicle4Delay = 0;
uint32_t vehicle5Delay = 0;
uint32_t vehicle6Delay = 0;

bool StuckDetector1 = false;
bool StuckDetector2 = false;
bool StuckDetector3 = false;
bool StuckDetector4 = false;
bool StuckDetector5 = false;
bool StuckDetector6 = false;

bool previousStuckDetector2 = true;

uint32_t StuckDetector2Timer = 0; //because you're using diy charger
uint32_t previousStuckDetector2Timer = 0;

unsigned long previousMillis = 0;

Adafruit_ST7789 lcd = Adafruit_ST7789(LCD_CS, LCD_DC, LCD_RST);


void formatMacAddress(const uint8_t *macAddr, char *buffer, int maxLength) {
    snprintf(buffer, maxLength, "%02x:%02x:%02x:%02x:%02x:%02x",
             macAddr[0], macAddr[1], macAddr[2],
             macAddr[3], macAddr[4], macAddr[5]);
  }


void broadcast(const String &message) {
    uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    esp_now_peer_info_t peerInfo = {};
    memcpy(&peerInfo.peer_addr, broadcastAddress, 6);
  
    // Only add the peer if it's not already added
    if (!esp_now_is_peer_exist(broadcastAddress)) {
      esp_now_add_peer(&peerInfo);
    }
  
    // Send message
    esp_err_t result = esp_now_send(broadcastAddress, (const uint8_t *)message.c_str(), message.length());
  
    // Print results to serial monitor
    if (result != ESP_OK) {
      Serial.println("ESP-NOW send failed");
    }
}


void receiveCallback(const uint8_t *mac, const uint8_t *data, int len) {
    char buffer[ESP_NOW_MAX_DATA_LEN + 1];
    int msgLen = min(ESP_NOW_MAX_DATA_LEN, len);
    strncpy(buffer, (const char *)data, msgLen);
    buffer[msgLen] = 0;

    //Print All message to serial monitor. Comment out for normal use:
    /*
    Serial.println(buffer);  
    lcd.fillRect(0, 0, screenWidth, 20, ST77XX_BLACK);  // Clear area at top of screen (optional)
    lcd.setCursor(0, 0);
    lcd.setTextColor(ST77XX_WHITE);
    lcd.setTextSize(2);
    lcd.print(buffer);  // buffer holds the received ESP-NOW message
    */

    // Format MAC address
    char macStr[18];
    snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    //Serial.printf("Received from %s: %s\n", macStr, buffer); // Serial Print MAC address with messages received via ESP-NOW

    // Process and Serial Print Driving Status Messages:
    if (strcmp("StuckDetector1", buffer) == 0) {
        StuckDetector1 = true;
        broadcast("StuckDetector1");
        }
    //Stuck detector 2 is different because the charge setup is DIY
    if (strcmp("StuckDetector2", buffer) == 0) { 
        StuckDetector2 = true;
        //Serial.println("Track 2 Help message received");
        }

    if (strcmp("StuckDetector3", buffer) == 0) {
        StuckDetector3 = true;
        //Serial.println("Track 3 STUCK");
        delay(1);
    } 


    if (strcmp("Driving1", buffer) == 0) {
        StuckDetector1 = false;
        vehicle1Timer = millis(); // Reset timer when vehicle is moving
        //Serial.println("Track 1 Nominal");
        delay(3);
    }
    if (strcmp("Driving2", buffer) == 0) {
        StuckDetector2 = false;
        vehicle2Timer = millis(); // Reset timer when vehicle is moving
        previousStuckDetector2 = true;
        //Serial.println("Track 2 Nominal");
    }
    if (strcmp("Driving3", buffer) == 0) {
        StuckDetector3 = false;
        vehicle3Timer = millis(); // Reset timer when vehicle is moving
        //Serial.println("Track 3 Nominal");
    }
    if (strcmp("Driving4", buffer) == 0) {
        StuckDetector4 = false;
        vehicle4Timer = millis(); // Reset timer when vehicle is moving
        //Serial.println("Track 4 Nominal");
    }
    if (strcmp("Driving5", buffer) == 0) {
        StuckDetector5 = false;
        vehicle5Timer = millis(); // Reset timer when vehicle is moving
        //Serial.println("Track 5 Nominal");
    }
    if (strcmp("Driving6", buffer) == 0) {
        StuckDetector6 = false;
        vehicle6Timer = millis(); // Reset timer when vehicle is moving
        //Serial.println("Track 6 Nominal");
    }
}
  
void sentCallback(const uint8_t *macAddr, esp_now_send_status_t status) {
    char macStr[18];
    formatMacAddress(macAddr, macStr, 18);
    // Optionally handle send status here
}
  

void drawStaticLabels() {
    lcd.setTextColor(ST77XX_WHITE);
    lcd.setTextSize(textSize);
  
    for (int i = 0; i < 6; i++) {
      int col = i % 2;
      int row = i / 2;
  
      int x = col * colWidth + 10;   // extra padding for both columns
      int y = row * rowHeight + 8;   // vertical centering
  
      lcd.setCursor(x, y);
      lcd.printf("%d:", i + 1);
    }
  }


void updateDelayValue(uint8_t vehicleId, uint32_t delayValue, uint32_t &lastDelayValue) {
    if (delayValue != lastDelayValue) {
      int index = vehicleId - 1;
      int col = index % 2;
      int row = index / 2;
  
      int x = col * colWidth + 50;   // leave room for "n:"
      int y = row * rowHeight + 8;
  
      lcd.fillRect(x, y, colWidth - 50, rowHeight - 10, ST77XX_BLACK);
      lcd.setCursor(x, y);

      if (delayValue > 480) { //8 minutes
        lcd.setTextColor(ST77XX_RED);
      } else if (delayValue < 120) {
        lcd.setTextColor(ST77XX_GREEN);
      } else {
        lcd.setTextColor(ST77XX_WHITE);
      }

      lcd.setTextSize(textSize);
      if (delayValue > 86400) {
        delayValue = delayValue / 86400; // Convert to days
        lcd.printf("%lu Days", delayValue);
      } else if (delayValue > 3600) {
        delayValue = delayValue / 3600; // Convert to hours
        lcd.printf("%lu Hrs", delayValue);
      } else {
        lcd.print(delayValue);
      }
      
  
      lastDelayValue = delayValue;
    }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Setup Starting");
  pinMode(LCD_BLK, OUTPUT);
  digitalWrite(LCD_BLK, HIGH); // Turn on backlight

  lcd.init(170, 320);       // Width, Height
  lcd.setRotation(1);       // 1 = (landscape mode)
  lcd.fillScreen(ST77XX_BLACK);

  drawStaticLabels();



 // Set ESP32 in STA mode for ESP-NOW
 WiFi.mode(WIFI_STA);

 // Print MAC address
 Serial.print("MAC Address: ");
 Serial.println(WiFi.macAddress());

 // Disconnect from WiFi (to use ESP-NOW)
 WiFi.disconnect();

 // Initialize ESP-NOW
 if (esp_now_init() == ESP_OK) {
   Serial.println("ESP-NOW Init Success");
   esp_now_register_recv_cb(receiveCallback);  // Register receive callback
   esp_now_register_send_cb(sentCallback);     // Register send callback
 } else {
   Serial.println("ESP-NOW Init Failed");
   delay(1000);
   ESP.restart();
 }

 tone(BUZZER_PIN, 400, 1000);  // Initial tone for a buzzer sound

}



void loop() {
    unsigned long currentMillis = millis();

    // Run the block every 100ms
    if (currentMillis - previousMillis >= 100) { 
        previousMillis = currentMillis; 

        static uint32_t lastVehicle1Delay = 0;
        vehicle1Delay = (millis() - vehicle1Timer)/1000;
        updateDelayValue(1, vehicle1Delay, lastVehicle1Delay);

        static uint32_t lastVehicle2Delay = 0;
        vehicle2Delay = (millis() - vehicle2Timer)/1000;
        updateDelayValue(2, vehicle2Delay, lastVehicle2Delay);

        static uint32_t lastVehicle3Delay = 0;
        vehicle3Delay = (millis() - vehicle3Timer)/1000;
        updateDelayValue(3, vehicle3Delay, lastVehicle3Delay);

        static uint32_t lastVehicle4Delay = 0;
        vehicle4Delay = (millis() - vehicle4Timer)/1000;
        updateDelayValue(4, vehicle4Delay, lastVehicle4Delay);

        static uint32_t lastVehicle5Delay = 0;
        vehicle5Delay = (millis() - vehicle5Timer)/1000;
        updateDelayValue(5, vehicle5Delay, lastVehicle5Delay);

        static uint32_t lastVehicle6Delay = 0;
        vehicle6Delay = (millis() - vehicle6Timer)/1000;
        updateDelayValue(6, vehicle6Delay, lastVehicle6Delay);


        // Check stuck conditions
        if (StuckDetector3 || StuckDetector1) {
            tone(BUZZER_PIN, 200, 50); // Play tone if any detector is triggered
        } 

        // Stuck detector 2 is different because the charge setup is DIY
        if (StuckDetector2 && previousStuckDetector2) {
            StuckDetector2Timer = millis();  
            Serial.println("Track 2 HOPEFULLY JUST CHARGING");
            previousStuckDetector2 = false;
        }

        if (StuckDetector2 && !previousStuckDetector2 && (millis() > StuckDetector2Timer + 40000)) { 
            Serial.println("Track 2 STUCK!!!");
            tone(BUZZER_PIN, 300, 50); // Play tone
        }
    }
}
