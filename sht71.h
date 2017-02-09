#define HUMIDITY_DDR	DDRB
#define HUMIDITY_PIN	PINB
#define HUMIDITY_PORT	PORTB
#define HUMIDITY_DATA	0x10
#define HUMIDITY_SCK	0x20
//adr command r/w
#define HUMIDITY_STATUS_REG_W 0x06 //000 0011 0
#define HUMIDITY_STATUS_REG_R 0x07 //000 0011 1
#define HUMIDITY_MEASURE_TEMP 0x03 //000 0001 1
#define HUMIDITY_MEASURE_HUMI 0x05 //000 0010 1
#define HUMIDITY_RESET 0x1e //000 1111 0
void HumidityInit(void);
void HumidityDelay(void);
void HumidityWriteByte(unsigned char value);
unsigned char HumidityReadByte(unsigned char ack);
void HumidityTransStart(void);
void HumidityConnectionReset(void);
void HumiditySoftReset(void);
unsigned char HumidityReadStatus(void);
void HumidityWriteStatus(unsigned char value);
unsigned int HumidityMeasure(unsigned char mode);
void HumidityCalc(float *p_humidity ,float *p_temperature);
void HumidityGet(float *p_humidity ,float *p_temperature);
void HumidityUpdate(void);
