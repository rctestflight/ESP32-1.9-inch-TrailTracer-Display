#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SPI.h>
#include <WiFi.h>
#include <esp_now.h>
#include <math.h>

#define LCD_MOSI 23
#define LCD_SCK 18
#define LCD_CS 15
#define LCD_DC 2
#define LCD_RST 4
#define LCD_BLK 32

int desiredVehicleID = 2; //set this to the desired vehicle ID. 1: front track. 2: west track

const int textSize = 3;
const int screenWidth = 320;
const int screenHeight = 170;

unsigned long previousMillis = 0;

unsigned long lastDesiredVehicleReceiveTime = 0;  // Track last receive time for desiredVehicleID

// Transition counter: Driving to Charging transitions
char lastVehicleStatus[32] = "";
int transitionCountThisHour = 0;
// Record timestamps of individual transitions so we can compute a sliding
// count of transitions that occurred within the previous hour.
const int TRANSITION_EVENTS_MAX = 256;
unsigned long transition_event_times[TRANSITION_EVENTS_MAX];
volatile int transition_event_count = 0;
// Last parsed PID output (numeric) for display
int lastPidOutput = 0;

// Transition series buffer (24h window, 1 sample/30min)
const int TRANS_GRAPH_POINTS_MAX = 24 * 2; // samples for 24 hours at 1/30min (48 samples)
const unsigned long TRANS_GRAPH_INTERVAL = 30UL * 60000UL; // 30 minutes
unsigned long trans_time[TRANS_GRAPH_POINTS_MAX];
int trans_val[TRANS_GRAPH_POINTS_MAX];
int trans_count = 0;
unsigned long lastTransSample = 0;

// Graph data for vBatt over time (30 minute window)
const int GRAPH_POINTS_MAX = 60;  // Store up to 60 samples for 30 minutes
const int GRAPH_UPDATE_INTERVAL = 30000;  // Update graph every 30 seconds
unsigned long graphBuffer_time[GRAPH_POINTS_MAX];
float graphBuffer_vBatt[GRAPH_POINTS_MAX];
int graphBuffer_count = 0;
unsigned long lastGraphUpdate = 0;
// Graph data for s1_radius (30 second window)
const int S1_GRAPH_POINTS_MAX = 120; // allow more frequent samples
unsigned long graphS1_time[S1_GRAPH_POINTS_MAX];
float graphS1_val[S1_GRAPH_POINTS_MAX];
int graphS1_count = 0;

Adafruit_ST7789 lcd = Adafruit_ST7789(LCD_CS, LCD_DC, LCD_RST);

// Forward declarations
void drawVBattGraph();
void updateDisplayFromData();

// Shared data populated by receiveCallback, consumed in loop()
volatile bool newDataAvailable = false;
struct SharedData {
  uint8_t vehicleID;
  char vehicleStatus[32];
  char vBatt[32];
  char vCharge[32];
  char iCharge[32];
  char charge_status[32];
  char status_bits[32];
  char temperature[32];
  char s1_radius[32];
  char v5v[32];
  int pid_output;
  bool lapFlag;
} sharedData;


void formatMacAddress(const uint8_t *macAddr, char *buffer, int maxLength) {
    snprintf(buffer, maxLength, "%02x:%02x:%02x:%02x:%02x:%02x",
             macAddr[0], macAddr[1], macAddr[2],
             macAddr[3], macAddr[4], macAddr[5]);
  }

void receiveCallback(const uint8_t *mac, const uint8_t *data, int len) {
    char buffer[ESP_NOW_MAX_DATA_LEN + 1];
    int msgLen = min(ESP_NOW_MAX_DATA_LEN, len);
    strncpy(buffer, (const char *)data, msgLen);
    buffer[msgLen] = 0;

    // Ignore messages that don't start with "-"
    if (buffer[0] != '-') {
        return;
    }

    // Parse the 11 or 12 space-separated values (lapFlag is optional)
    uint8_t vehicleID;
    char vehicleStatus[32], vBatt[32], vCharge[32], iCharge[32], charge_status[32], status_bits[32];
    char temperature[32], s1_radius[32], v5v[32], pid_output_str[32];
    int lapFlagInt = 0;

    int parsed = sscanf(buffer, "-%hhu %s %s %s %s %s %s %s %s %s %s %d",
        &vehicleID, vehicleStatus, vBatt, vCharge, iCharge, charge_status, status_bits, temperature, s1_radius, v5v, pid_output_str, &lapFlagInt);

    // Verify we got at least 11 values (vehicleID + 10 more). lapFlag is optional.
    if (parsed < 11) {
      return;
    }

    // If vehicleID is the one we want, copy parsed fields into sharedData and signal main loop
    if (vehicleID == desiredVehicleID) {
      // Update the last receive time for this vehicle ID
      lastDesiredVehicleReceiveTime = millis();

      // Copy parsed strings into shared buffer under brief critical section
      noInterrupts();
      sharedData.vehicleID = vehicleID;
      strncpy(sharedData.vehicleStatus, vehicleStatus, sizeof(sharedData.vehicleStatus) - 1);
      sharedData.vehicleStatus[sizeof(sharedData.vehicleStatus) - 1] = '\0';
      strncpy(sharedData.vBatt, vBatt, sizeof(sharedData.vBatt) - 1);
      sharedData.vBatt[sizeof(sharedData.vBatt) - 1] = '\0';
      strncpy(sharedData.vCharge, vCharge, sizeof(sharedData.vCharge) - 1);
      sharedData.vCharge[sizeof(sharedData.vCharge) - 1] = '\0';
      strncpy(sharedData.iCharge, iCharge, sizeof(sharedData.iCharge) - 1);
      sharedData.iCharge[sizeof(sharedData.iCharge) - 1] = '\0';
      strncpy(sharedData.charge_status, charge_status, sizeof(sharedData.charge_status) - 1);
      sharedData.charge_status[sizeof(sharedData.charge_status) - 1] = '\0';
      strncpy(sharedData.status_bits, status_bits, sizeof(sharedData.status_bits) - 1);
      sharedData.status_bits[sizeof(sharedData.status_bits) - 1] = '\0';
      strncpy(sharedData.temperature, temperature, sizeof(sharedData.temperature) - 1);
      sharedData.temperature[sizeof(sharedData.temperature) - 1] = '\0';
      strncpy(sharedData.s1_radius, s1_radius, sizeof(sharedData.s1_radius) - 1);
      sharedData.s1_radius[sizeof(sharedData.s1_radius) - 1] = '\0';
      strncpy(sharedData.v5v, v5v, sizeof(sharedData.v5v) - 1);
      sharedData.v5v[sizeof(sharedData.v5v) - 1] = '\0';
      sharedData.pid_output = (int)atof(pid_output_str);
      sharedData.lapFlag = lapFlagInt ? true : false;
      newDataAvailable = true;
      interrupts();
    }
}
  
void sentCallback(const uint8_t *macAddr, esp_now_send_status_t status) {
    char macStr[18];
    formatMacAddress(macAddr, macStr, 18);
}

void drawVBattGraph() {
  // Draw graph area: X-axis 30 min, Y-axis dynamic based on min/max vBatt
  int graphX = 0;
  int graphY = 50;  // Y position of the graph. Below the two text lines
  int graphWidth = screenWidth;
  int graphHeight = 130;
  int graphRight = graphX + graphWidth - 1;
  int graphBottom = graphY + graphHeight - 1;

  // X-axis window: 30 minutes
  unsigned long timeWindow = 30 * 60 * 1000UL;  // 30 minutes in milliseconds
  unsigned long now = millis();

  // Determine dynamic vMin/vMax from buffer
  float vMin = 10.0f;
  float vMax = 12.5f;
  if (graphBuffer_count > 0) {
    // compute min/max among buffered points
    float minv = graphBuffer_vBatt[0];
    float maxv = graphBuffer_vBatt[0];
    for (int i = 1; i < graphBuffer_count; i++) {
      if (graphBuffer_vBatt[i] < minv) minv = graphBuffer_vBatt[i];
      if (graphBuffer_vBatt[i] > maxv) maxv = graphBuffer_vBatt[i];
    }
    // add a small padding so lines aren't on the extreme edge
    float padding = (maxv - minv) * 0.1f; // 10% padding
    if (padding < 0.05f) padding = 0.05f;
    vMin = minv - padding;
    vMax = maxv + padding;
    // ensure sensible bounds
    if (vMin >= vMax) { vMin = minv - 0.1f; vMax = maxv + 0.1f; }
  }

  // Clear graph area
  lcd.fillRect(graphX, graphY, graphWidth, graphHeight, ST77XX_BLACK);

  // Overlay both series in the same plotting area. Each series keeps its own vertical scaling
  int plotY = graphY;
  int plotBottom = graphBottom;
  int plotHeight = graphHeight;

  // --- draw vBatt (white) using 30-minute window ---
  if (graphBuffer_count > 1) {
    unsigned long v_timeWindow = 30 * 60 * 1000UL;
    for (int i = 0; i < graphBuffer_count - 1; i++) {
      unsigned long age = now - graphBuffer_time[i];
      if (age > v_timeWindow) continue;
      unsigned long age2 = now - graphBuffer_time[i + 1];
      if (age2 > v_timeWindow) continue;

      float x1_frac = (float)(v_timeWindow - age) / (float)v_timeWindow;
      int x1 = graphX + (int)(x1_frac * (graphWidth - 1));
      float y1_frac = (graphBuffer_vBatt[i] - vMin) / (vMax - vMin);
      int y1 = plotBottom - (int)(y1_frac * plotHeight);

      float x2_frac = (float)(v_timeWindow - age2) / (float)v_timeWindow;
      int x2 = graphX + (int)(x2_frac * (graphWidth - 1));
      float y2_frac = (graphBuffer_vBatt[i + 1] - vMin) / (vMax - vMin);
      int y2 = plotBottom - (int)(y2_frac * plotHeight);

      // clamp
      if (x1 < graphX) x1 = graphX;
      if (x2 > graphRight) x2 = graphRight;
      if (y1 < plotY) y1 = plotY;
      if (y1 > plotBottom) y1 = plotBottom;
      if (y2 < plotY) y2 = plotY;
      if (y2 > plotBottom) y2 = plotBottom;

      lcd.drawLine(x1, y1, x2, y2, ST77XX_WHITE);
    }
  }

  // --- draw s1_radius (red) using 30-second window overlaid on same plot ---
  if (graphS1_count > 1) {
    unsigned long s1_timeWindow = 30 * 1000UL; // 30 seconds
    // compute min/max for s1 data
    float s1Min = graphS1_val[0];
    float s1Max = graphS1_val[0];
    for (int i = 1; i < graphS1_count; i++) {
      if (graphS1_val[i] < s1Min) s1Min = graphS1_val[i];
      if (graphS1_val[i] > s1Max) s1Max = graphS1_val[i];
    }
    float s1Padding = (s1Max - s1Min) * 0.1f;
    if (s1Padding < 0.001f) s1Padding = 0.001f;
    float s1_vMin = s1Min - s1Padding;
    float s1_vMax = s1Max + s1Padding;
    if (s1_vMin >= s1_vMax) { s1_vMin = s1Min - 0.1f; s1_vMax = s1Max + 0.1f; }

    for (int i = 0; i < graphS1_count - 1; i++) {
      unsigned long age = now - graphS1_time[i];
      if (age > s1_timeWindow) continue;
      unsigned long age2 = now - graphS1_time[i + 1];
      if (age2 > s1_timeWindow) continue;

      float x1_frac = (float)(s1_timeWindow - age) / (float)s1_timeWindow;
      int x1 = graphX + (int)(x1_frac * (graphWidth - 1));
      float y1_frac = (graphS1_val[i] - s1_vMin) / (s1_vMax - s1_vMin);
      int y1 = plotBottom - (int)(y1_frac * plotHeight);

      float x2_frac = (float)(s1_timeWindow - age2) / (float)s1_timeWindow;
      int x2 = graphX + (int)(x2_frac * (graphWidth - 1));
      float y2_frac = (graphS1_val[i + 1] - s1_vMin) / (s1_vMax - s1_vMin);
      int y2 = plotBottom - (int)(y2_frac * plotHeight);

      // clamp
      if (x1 < graphX) x1 = graphX;
      if (x2 > graphRight) x2 = graphRight;
      if (y1 < plotY) y1 = plotY;
      if (y1 > plotBottom) y1 = plotBottom;
      if (y2 < plotY) y2 = plotY;
      if (y2 > plotBottom) y2 = plotBottom;

      lcd.drawLine(x1, y1, x2, y2, ST77XX_RED);
    }
  }

  // --- draw transitions (blue) using 24-hour window overlaid on same plot ---
  if (trans_count > 1) {
    unsigned long trans_timeWindow = 24UL * 3600UL * 1000UL; // 24 hours
    // compute max for transitions
    int tMaxInt = trans_val[0];
    for (int i = 1; i < trans_count; i++) {
      if (trans_val[i] > tMaxInt) tMaxInt = trans_val[i];
    }
    float tMin = 5.0f; // Y axis minimum for transitions graph
    float tMax = (float)tMaxInt; // Y axis max for transitions graph. It automatically changes to match the largest value
    if (tMax <= tMin) tMax = tMin + 1.0f;

    for (int i = 0; i < trans_count - 1; i++) {
      unsigned long age = now - trans_time[i];
      if (age > trans_timeWindow) continue;
      unsigned long age2 = now - trans_time[i + 1];
      if (age2 > trans_timeWindow) continue;

      float x1_frac = (float)(trans_timeWindow - age) / (float)trans_timeWindow;
      int x1 = graphX + (int)(x1_frac * (graphWidth - 1));
      float y1_frac = ((float)trans_val[i] - tMin) / (tMax - tMin);
      int y1 = plotBottom - (int)(y1_frac * plotHeight);

      float x2_frac = (float)(trans_timeWindow - age2) / (float)trans_timeWindow;
      int x2 = graphX + (int)(x2_frac * (graphWidth - 1));
      float y2_frac = ((float)trans_val[i + 1] - tMin) / (tMax - tMin);
      int y2 = plotBottom - (int)(y2_frac * plotHeight);

      // clamp
      if (x1 < graphX) x1 = graphX;
      if (x2 > graphRight) x2 = graphRight;
      if (y1 < plotY) y1 = plotY;
      if (y1 > plotBottom) y1 = plotBottom;
      if (y2 < plotY) y2 = plotY;
      if (y2 > plotBottom) y2 = plotBottom;

      lcd.drawLine(x1, y1, x2, y2, ST77XX_BLUE);
    }
  }
}

// Called from main loop to consume sharedData and update the display/graph
void updateDisplayFromData() {
  if (!newDataAvailable) return;

  // Copy shared data into local variables quickly
  char vehicleStatusLoc[32];
  char vBattLoc[32];
  char vChargeLoc[32];
  char iChargeLoc[32];
  char charge_statusLoc[32];
  char status_bitsLoc[32];
  char temperatureLoc[32];
  char s1_radiusLoc[32];
  char v5vLoc[32];
  int pid_outputLoc;
  uint8_t vehicleIDLoc = 0;
  bool lapFlagLoc = false;

  noInterrupts();
  vehicleIDLoc = sharedData.vehicleID;
  lapFlagLoc = sharedData.lapFlag;
  strncpy(vehicleStatusLoc, sharedData.vehicleStatus, sizeof(vehicleStatusLoc) - 1);
  vehicleStatusLoc[sizeof(vehicleStatusLoc) - 1] = '\0';
  strncpy(vBattLoc, sharedData.vBatt, sizeof(vBattLoc) - 1);
  vBattLoc[sizeof(vBattLoc) - 1] = '\0';
  strncpy(vChargeLoc, sharedData.vCharge, sizeof(vChargeLoc) - 1);
  vChargeLoc[sizeof(vChargeLoc) - 1] = '\0';
  strncpy(iChargeLoc, sharedData.iCharge, sizeof(iChargeLoc) - 1);
  iChargeLoc[sizeof(iChargeLoc) - 1] = '\0';
  strncpy(charge_statusLoc, sharedData.charge_status, sizeof(charge_statusLoc) - 1);
  charge_statusLoc[sizeof(charge_statusLoc) - 1] = '\0';
  strncpy(status_bitsLoc, sharedData.status_bits, sizeof(status_bitsLoc) - 1);
  status_bitsLoc[sizeof(status_bitsLoc) - 1] = '\0';
  strncpy(temperatureLoc, sharedData.temperature, sizeof(temperatureLoc) - 1);
  temperatureLoc[sizeof(temperatureLoc) - 1] = '\0';
  strncpy(s1_radiusLoc, sharedData.s1_radius, sizeof(s1_radiusLoc) - 1);
  s1_radiusLoc[sizeof(s1_radiusLoc) - 1] = '\0';
  strncpy(v5vLoc, sharedData.v5v, sizeof(v5vLoc) - 1);
  v5vLoc[sizeof(v5vLoc) - 1] = '\0';
  pid_outputLoc = sharedData.pid_output;
  // mark consumed
  newDataAvailable = false;
  interrupts();

  // Now perform all LCD drawing in main loop context
  // Clear an area large enough for two lines of text with extra padding
  lcd.fillRect(0, 0, screenWidth, 65, ST77XX_BLACK);

  // First line: colored status, vBatt
  lcd.setCursor(0, 0);
  lcd.setTextSize(textSize);

  if (strcmp(vehicleStatusLoc, "0") == 0) {
    lcd.setTextColor(ST77XX_RED);
    lcd.print("Stuck");
  } else if (strcmp(vehicleStatusLoc, "1") == 0) {
    lcd.setTextColor(ST77XX_GREEN);
    lcd.print("Driving");
  } else if (strcmp(vehicleStatusLoc, "2") == 0) {
    lcd.setTextColor(ST77XX_YELLOW);
    lcd.print("Charging");
  } else {
    lcd.setTextColor(ST77XX_WHITE);
    lcd.print(vehicleStatusLoc);
  }

  lcd.setTextColor(ST77XX_WHITE);
  lcd.print(" ");
  lcd.print(vBattLoc);
  lcd.print("V");

  // Second line: iCharge + remaining values in size 2 font
  int firstLinePx = 8 * textSize + 4; // approximate pixel height of first line + padding
  lcd.setCursor(0, firstLinePx);
  lcd.setTextSize(2);
  lcd.setTextColor(ST77XX_WHITE);

  {
    char buf[16];
    float iflt = atof(iChargeLoc);
    snprintf(buf, sizeof(buf), "%.1f", iflt);
    lcd.print(buf);
    lcd.print("A");
  }
  lcd.print(" ");
  {
    char buf[16];
    float vflt = atof(vChargeLoc);
    snprintf(buf, sizeof(buf), "%.1f", vflt);
    lcd.print(buf);
    lcd.print("V");
  }
  lcd.print(" ");
  {
    float tf = atof(temperatureLoc);
    int t_int = (int)roundf(tf);
    lcd.print(t_int);
  }
  lcd.print("C");

  // Draw pid_output as a yellow 4x4 pixel square mapped across the full width
  // pid range: 800 (left) .. 2200 (right). Place directly below second text line.
  {
    int pid = pid_outputLoc;
    // Clamp to new range
    if (pid < 800) pid = 800;
    if (pid > 2200) pid = 2200;
    lastPidOutput = pid;
    float frac = (pid - 800.0f) / 1400.0f; // 0..1
    if (frac < 0.0f) frac = 0.0f;
    if (frac > 1.0f) frac = 1.0f;
    int squareWidth = 4;
    int maxX = screenWidth - squareWidth;
    int pidX = (int)(frac * (float)maxX);
    // Vertical position: directly below second line
    int pidY = firstLinePx + (8 * 2) + 2; // second line top + height (8*size2) + small padding
    // Safety clamp
    if (pidX < 0) pidX = 0;
    if (pidX > maxX) pidX = maxX;
    if (pidY < 0) pidY = 0;
    if (pidY > screenHeight - squareWidth) pidY = screenHeight - squareWidth;
    lcd.fillRect(pidX, pidY, squareWidth, squareWidth, ST77XX_YELLOW);
  }

  // Update graph buffer with new vBatt sample (timed)
  unsigned long now = millis();
  if (now - lastGraphUpdate >= GRAPH_UPDATE_INTERVAL) {
    lastGraphUpdate = now;
    float vBattVal = atof(vBattLoc);

    // Shift buffer if full
    if (graphBuffer_count >= GRAPH_POINTS_MAX) {
      for (int i = 0; i < GRAPH_POINTS_MAX - 1; i++) {
        graphBuffer_time[i] = graphBuffer_time[i + 1];
        graphBuffer_vBatt[i] = graphBuffer_vBatt[i + 1];
      }
      graphBuffer_count--;
    }

    // Add new point
    graphBuffer_time[graphBuffer_count] = now;
    graphBuffer_vBatt[graphBuffer_count] = vBattVal;
    graphBuffer_count++;

    // Redraw graph
    drawVBattGraph();
  }

  // Append s1_radius sample for short-window graph (timestamped every data update)
  {
    float s1val = atof(s1_radiusLoc);
    unsigned long t = millis();
    if (graphS1_count >= S1_GRAPH_POINTS_MAX) {
      for (int i = 0; i < S1_GRAPH_POINTS_MAX - 1; i++) {
        graphS1_time[i] = graphS1_time[i + 1];
        graphS1_val[i] = graphS1_val[i + 1];
      }
      graphS1_count--;
    }
    graphS1_time[graphS1_count] = t;
    graphS1_val[graphS1_count] = s1val;
    graphS1_count++;
    // Redraw graph since s1 changed (vBatt draw may also run on its interval)
    drawVBattGraph();
  }

  // Track transitions only when lapFlag is received as 1
  if (lapFlagLoc) {
    noInterrupts();
    if (transition_event_count >= TRANSITION_EVENTS_MAX) {
      // buffer full: drop oldest by shifting left one
      for (int i = 0; i < TRANSITION_EVENTS_MAX - 1; i++) {
        transition_event_times[i] = transition_event_times[i + 1];
      }
      transition_event_count = TRANSITION_EVENTS_MAX - 1;
    }
    transition_event_times[transition_event_count++] = now;
    interrupts();
  }
  
  // Update last status for display purposes
  if (strcmp(lastVehicleStatus, vehicleStatusLoc) != 0) {
    strncpy(lastVehicleStatus, vehicleStatusLoc, sizeof(lastVehicleStatus) - 1);
    lastVehicleStatus[sizeof(lastVehicleStatus) - 1] = '\0';
  }
}

void updateElapsedTimeDisplay() {
    // Update elapsed time display on the second line
    unsigned long elapsed_ms = millis() - lastDesiredVehicleReceiveTime;
    unsigned long elapsed_s = elapsed_ms / 1000;
    char timebuf[16];
    
    if (elapsed_s < 60) {
      // Print in seconds
      snprintf(timebuf, sizeof(timebuf), "%luS", elapsed_s);
    } else if (elapsed_s < 3600) {
      // Print in minutes
      unsigned long elapsed_m = elapsed_s / 60;
      snprintf(timebuf, sizeof(timebuf), "%luM", elapsed_m);
    } else {
      // Print in hours
      unsigned long elapsed_h = elapsed_s / 3600;
      snprintf(timebuf, sizeof(timebuf), "%luH", elapsed_h);
    }
    
    // Clear just the time area at the end of the second line and redraw
    int firstLinePx = 8 * textSize + 4;
    int timeX = 220;  // Position for far right of screen
    // Clear only the area covering the transition counter and elapsed time text (70 wide, 18 tall)
    lcd.fillRect(timeX, firstLinePx, 70, 18, ST77XX_BLACK);
    lcd.setCursor(timeX, firstLinePx);
    lcd.setTextSize(2);
    lcd.setTextColor(ST77XX_BLUE);

    // Display transitions per hour
    lcd.setTextColor(ST77XX_BLUE);
    lcd.print(transitionCountThisHour);
    lcd.print("L ");

    // Display elapsed time after transitions
    if (elapsed_s > 10) {
      lcd.setTextColor(ST77XX_RED);
    } else {
      lcd.setTextColor(ST77XX_WHITE);
    }
    lcd.print(timebuf);
}

void setup() {
  Serial.begin(115200);
  pinMode(LCD_BLK, OUTPUT);
  digitalWrite(LCD_BLK, HIGH); // Turn on backlight

  lcd.init(170, 320);       // Width, Height
  lcd.setRotation(1);       // 1 = (landscape mode)
  lcd.fillScreen(ST77XX_BLACK);

  // Set ESP32 in STA mode for ESP-NOW
  WiFi.mode(WIFI_STA);

  // Disconnect from WiFi (to use ESP-NOW)
  WiFi.disconnect();

  // Initialize ESP-NOW
  if (esp_now_init() == ESP_OK) {
    esp_now_register_recv_cb(receiveCallback);  // Register receive callback
    esp_now_register_send_cb(sentCallback);     // Register send callback
  } else {
    delay(1000);
    ESP.restart();
  }
}


void loop() {
  unsigned long currentMillis = millis();

  // Run the block every 100ms
  if (currentMillis - previousMillis >= 100) {
    previousMillis = currentMillis;
    
    // Prune events older than 1 hour and update transitionCountThisHour
    unsigned long now = millis();
    unsigned long cutoff = (now > 3600000UL) ? (now - 3600000UL) : 0;
    // Remove oldest entries until all remaining are within the last hour
    noInterrupts();
    while (transition_event_count > 0 && transition_event_times[0] < cutoff) {
      for (int i = 0; i < transition_event_count - 1; i++) {
        transition_event_times[i] = transition_event_times[i + 1];
      }
      transition_event_count--;
    }
    transitionCountThisHour = transition_event_count;
    interrupts();

    // Periodic sample of transitionCountThisHour (once per minute)
    if (now - lastTransSample >= TRANS_GRAPH_INTERVAL) {
      lastTransSample = now;
      if (trans_count >= TRANS_GRAPH_POINTS_MAX) {
        for (int i = 0; i < TRANS_GRAPH_POINTS_MAX - 1; i++) {
          trans_time[i] = trans_time[i + 1];
          trans_val[i] = trans_val[i + 1];
        }
        trans_count--;
      }
      trans_time[trans_count] = now;
      trans_val[trans_count] = transitionCountThisHour;
      trans_count++;
      // Redraw graph to include transitions
      drawVBattGraph();
    }

    // Draw solid red border if elapsed_s > 20
    unsigned long elapsed_ms = millis() - lastDesiredVehicleReceiveTime;
    unsigned long elapsed_s = elapsed_ms / 1000;
    int thickness = 3;
    if (elapsed_s > 20) {
      // Top
      lcd.fillRect(0, 0, screenWidth, thickness, ST77XX_RED);
      // Bottom
      lcd.fillRect(0, screenHeight - thickness, screenWidth, thickness, ST77XX_RED);
      // Left
      lcd.fillRect(0, 0, thickness, screenHeight, ST77XX_RED);
      // Right
      lcd.fillRect(screenWidth - thickness, 0, thickness, screenHeight, ST77XX_RED);
    }

    // Update elapsed time display
    updateElapsedTimeDisplay();

    // If new data is available from ESP-NOW callback, update display
    if (newDataAvailable) {
      updateDisplayFromData();
    }
  }
}
