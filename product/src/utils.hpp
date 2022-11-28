#define BAT_ADC     35
#define LED_PIN     12
#define DEBUG

#define UART_BAUD           9600
#define PIN_DTR             25
#define PIN_TX              27
#define PIN_RX              26
#define PWR_PIN             4

#define uS_TO_S_FACTOR      1000000ULL


#ifdef DEBUG
#  define D(x) x
#else
#  define D(x)
#endif // DEBUG

event e[32];

