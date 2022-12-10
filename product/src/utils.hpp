#define BAT_ADC     35
#define LED_PIN     12
//#define DEBUG

#define UART_BAUD           9600
#define PIN_DTR             25
#define PIN_TX              27
#define PIN_RX              26
#define PWR_PIN             4

#define uS_TO_S_FACTOR      1000000ULL
#define mS_TO_S_FACTOR      1000ULL

#ifdef DEBUG
#  define D(x) x
#else
#  define D(x)
#endif // DEBUG

event e[32];

enum ACTIVITY {
    GROUND,
    STILL,
    RUMINATE,
    GRAZE,
    WALK,
    OTHER,
    COUNT
};

struct results {
    float stdDevX = 0;
    float stdDevY = 0;
    float stdDevZ = 0;
    float meanX = 0;
    float meanY = 0;
    float meanZ = 0;
};
/*
struct activities {
    int ground = 0;
    int still = 0;
    int ruminate = 0;
    int graze = 0;
    int walk = 0;
    int other = 0;
};
*/

void blink(int interval, int duration) {
    int time = millis();
    while(millis() - time < duration) {
        digitalWrite(LED_PIN, LOW);
        delay(interval);
        digitalWrite(LED_PIN, HIGH);
        delay(interval);
    }
}

float readBattery()
{
    int vref = 1100;
    uint16_t volt = analogRead(BAT_ADC);
    float battery_voltage = ((float)volt / 4095.0) * 2.0 * 3.3 * (vref);
    return battery_voltage;
}
void lowBatteryCheck(float voltage) {
  if(voltage > 0.1 && voltage < 2.7) {
    delay(100);
    if(readBattery() > 2.7)
        return;
    esp_sleep_enable_timer_wakeup(43200 * uS_TO_S_FACTOR); // sleep half a day
    esp_deep_sleep_start();
  }
}

results stdDev(event* events) {
    results result;
    float sumX = 0.0, SDX = 0.0;
    float sumY = 0.0, SDY = 0.0;
    float sumZ = 0.0, SDZ = 0.0;
    for (int i = 0; i < 32; ++i) {
        sumX += events[i].X;
        sumY += events[i].Y;
        sumZ += events[i].Z;
    }
    result.meanX = sumX / 32;
    result.meanY = sumY / 32;
    result.meanZ = sumZ / 32;
    for (int i = 0; i < 10; ++i) {
        SDX += pow(events[i].X - result.meanX, 2);
        SDY += pow(events[i].Y - result.meanY, 2);
        SDZ += pow(events[i].Z - result.meanZ, 2);
    }
    result.stdDevX = sqrt(SDX / 32);
    result.stdDevY = sqrt(SDY / 32);
    result.stdDevZ = sqrt(SDZ / 32);
    return result;
}

ACTIVITY evaluate(event* events) {
    results result = stdDev(events);
    if (result.stdDevX <= 0.03)
        return GROUND;
    else if (result.stdDevX > 0.03 && result.stdDevX <= 0.15)
        return STILL;
    else if (result.stdDevX > 0.15 && result.stdDevX <= 0.6)
        return RUMINATE;
    else if (result.stdDevX > 0.6 && result.meanZ > 1)
        return GRAZE;
    else if (result.stdDevX > 0.6 && result.meanZ < 1)
        return WALK;
    else
        return OTHER;
}



