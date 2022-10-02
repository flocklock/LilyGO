#define SD_MISO     2
#define SD_MOSI     15
#define SD_SCLK     14
#define SD_CS       13
#define LED_PIN     12
#define BAT_ADC     35

#define SerialMon Serial
#define SerialAT Serial1

#include <SPI.h>
#include <SD.h>
#include "FS.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <SimpleTimer.h>

SimpleTimer firstTimer(50);

Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
  int i = 0;

#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, SerialMon);

File file;
uint32_t Freq = 0;

float readBattery(uint8_t pin)
{
    int vref = 1100;
    uint16_t volt = analogRead(pin);
    float battery_voltage = ((float)volt / 4095.0) * 2.0 * 3.3 * (vref);
    return battery_voltage;
}

void appendFile(fs::FS &fs, const char * path, const char * message){
    Serial.printf("Appending to file: %s\n", path);

    File file = fs.open(path, FILE_APPEND);
    if(!file){
        Serial.println("Failed to open file for appending");
        return;
    }
    if(file.print(message)){
        Serial.println("Message appended");
    } else {
        Serial.println("Append failed");
    }
}

void setup() {
  Serial.begin(115200);
  delay(10);
  Freq = getCpuFrequencyMhz();
  Serial.print("CPU Freq = ");
  Serial.print(Freq);
  Serial.println(" MHz");
  Freq = getXtalFrequencyMhz();
  Serial.print("XTAL Freq = ");
  Serial.print(Freq);
  Serial.println(" MHz");
  Freq = getApbFrequency();
  Serial.print("APB Freq = ");
  Serial.print(Freq);
  Serial.println(" Hz");
  delay(1000);
  setCpuFrequencyMhz(80);
  delay(3000);
  Freq = getCpuFrequencyMhz();
  Serial.print("CPU Freq = ");
  Serial.print(Freq);
  Serial.println(" MHz");
  Freq = getXtalFrequencyMhz();
  Serial.print("XTAL Freq = ");
  Serial.print(Freq);
  Serial.println(" MHz");
  Freq = getApbFrequency();
  Serial.print("APB Freq = ");
  Serial.print(Freq);
  Serial.println(" Hz");
  SPI.begin(SD_SCLK, SD_MISO, SD_MOSI, SD_CS);
    if (!SD.begin(SD_CS)) {
        Serial.println("SDCard MOUNT FAIL");
    } else {
        uint32_t cardSize = SD.cardSize() / (1024 * 1024);
        String str = "SDCard Size: " + String(cardSize) + "MB";
        Serial.println(str);
    }

    /* Initialise the sensor */
  if(!accel.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.println("Ooops, no ADXL345 detected ... Check your wiring!");
    while(1);
  }

  accel.setRange(ADXL345_RANGE_16_G);
  accel.setDataRate(ADXL345_DATARATE_25_HZ);
  file = SD.open("/acc.txt", FILE_APPEND);
    //deleteFile(SD, "/acc.txt");
    //writeFile(SD, "/acc.txt", "Start\n");
}

void loop() {
  if (firstTimer.isReady()) {
  sensors_event_t event; 
  accel.getEvent(&event);
  
  std::string message = std::to_string(millis()) + "," + std::to_string(readBattery(BAT_ADC)).substr(0,4) + "," + std::to_string(event.acceleration.x).substr(0,6) + "," + std::to_string(event.acceleration.y).substr(0,5) + "," + std::to_string(event.acceleration.z).substr(0,5) + "\n";
  file.print(message.c_str());
  if (i == 500) {
    i = 0;
    file.close();
    file = SD.open("/acc.txt", FILE_APPEND);
  }
  i++;
  firstTimer.reset();
  //appendFile(SD, "/acc.txt", message.c_str());
  }
}
