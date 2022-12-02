#define BAT_ADC     35
#define LED_PIN     12
#define DEBUG

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

