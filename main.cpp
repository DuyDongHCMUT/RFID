#include <stdio.h>
#include "RFID.h"
#define SDA PC1
#define SCL PC0
#define SCL_CLOCK 100000L           // T?n s? SCL 100kHz
#define LCD_I2C_ADDR (0x27 << 1)     // ??a ch? I2C c?a LCD (0x27) d?ch tr?i 1 bit
#define LCD_BACKLIGHT 0x08           // M? ?? b?t ??n n?n LCD
#define ENABLE 0b00000100            // M? t?n hi?u ENABLE

void I2C_Init() {
	TWSR = 0x00;                               // Ch?n pre-scaler l? 1
	TWBR = 0x20;     // ??t bit rate
	TWCR = (1 << TWEN);                        // K?ch ho?t I2C (TWI)
}

void I2C_Start() {
	TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN); // G?i t?n hi?u START
	while (!(TWCR & (1 << TWINT)));                   // Ch? START ho?n th?nh
}

void I2C_Stop() {
	TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN); // G?i t?n hi?u STOP
}

void I2C_Write(uint8_t lcd) {
	TWDR = lcd;
	TWCR = (1 << TWINT) | (1 << TWEN);         // Ghi d? li?u
	while (!(TWCR & (1 << TWINT)));            // Ch? ghi ho?n t?t
}

void LCD_Send(uint8_t lcd) {
	I2C_Write(lcd | ENABLE | LCD_BACKLIGHT);  // G?i t?n hi?u ENABLE
	_delay_us(1);
	I2C_Write((lcd & ~ENABLE) | LCD_BACKLIGHT); // G? t?n hi?u ENABLE
	_delay_us(50);
}

void LCD_SendCmd(uint8_t cmd) {
	I2C_Start();
	I2C_Write(LCD_I2C_ADDR);                   // G?i ??a ch? I2C
	LCD_Send(cmd & 0xF0);                      // G?i n?a byte cao
	LCD_Send((cmd << 4) & 0xF0);               // G?i n?a byte th?p
	I2C_Stop();
}

void LCD_SendData(uint8_t lcd) {
	I2C_Start();
	I2C_Write(LCD_I2C_ADDR);                   // G?i ??a ch? I2C
	LCD_Send((lcd & 0xF0) | 0x01);            // G?i n?a byte cao, b?t RS
	LCD_Send(((lcd << 4) & 0xF0) | 0x01);     // G?i n?a byte th?p, b?t RS
	I2C_Stop();
}

void LCD_Init() {
	_delay_ms(50);                // ??i kh?i ??ng LCD
	LCD_SendCmd(0x02);            // Chuy?n con tr? v? ??u m?n h?nh
	LCD_SendCmd(0x28);            // Ch? ?? 4-bit, 2 d?ng, font 5x7
	LCD_SendCmd(0x0C);            // B?t m?n h?nh, t?t con tr?
	LCD_SendCmd(0x06);            // T? ??ng t?ng ??a ch? sau m?i l?n ghi
	LCD_SendCmd(0x01);            // X?a m?n h?nh
	_delay_ms(2);
}

void LCD_SendString(char *a) {
	while (*a) {
		LCD_SendData(*a++);     // G?i t?ng k? t? trong chu?i
	}
}
void sendz(uint8_t c)
{
	while(bit_is_clear(UCSRA,UDRE));// doi den khi bit UDRE = 1
	UDR = c;
}
void SPI_MasterInit(void)
{
	DDRB |= (1<<SCK_PIN)|(1<<MOSI_PIN)|(1<<SS);
	//PORTB|=(1<<MISO_PIN);
	SPCR |=	(1<<SPE)|(1<<MSTR)|(1<<SPR0);
	sbi(PORTB,SS);
}


void _SendString(char str[])
{
	int i =0;
	
	while (str[i] != 0x00)
	{
		sendz(str[i]);
		i++;
	}
}
void sendUID(uint8_t *data) {
	for (int i = 0; i < 4; i++) {
		sendz(data[i]);  // G?i t?ng byte c?a UID qua UART
	}
}
// Hàm kh?i t?o PWM cho Timer1
void PWM_Init() {
	// ??t chân PD5 (OC1A) làm output
	DDRD |= (1 << PD5);

	// Ch?n ch? ?? Fast PWM (Mode 14: ICR1 TOP)
	TCCR1A |= (1 << COM1A1) | (1 << WGM11); // Clear OC1A on Compare Match, Fast PWM
	TCCR1B |= (1 << WGM12) | (1 << WGM13) | (1 << CS11); // Prescaler = 8

	// ??t chu k? PWM (TOP) là 20ms
	ICR1 = 19999; // F_CPU = 8MHz, Prescaler = 8 -> TOP = (8*10^6) / (50*8) - 1

	// ??t ?? r?ng xung ban ??u (v? trí trung tính 1.5ms)
	OCR1A = 1000; // ?? r?ng xung: 1ms
}

// Hàm ?i?u ch?nh góc c?a servo (?? r?ng xung t? 1ms ??n 2ms)
void Servo_SetAngle(uint16_t angle) {
	// Quy ??i góc t? 0 ??n 180 thành ?? r?ng xung (1000 - 2000us)
	OCR1A = 1000 + (angle * 1000) / 180;
}

MFRC522 abc(4,0);

int main(void)
{
	SPI_MasterInit();    // Kh?i t?o SPI
	I2C_Init();          // Kh?i t?o I2C
	LCD_Init();          // Kh?i t?o LCD
	PWM_Init();          // Kh?i t?o PWM
	DDRA = 0xFF;         // C?u hình Port A làm output
	PORTA = 0x00;        // ??t ??u ra ban ??u là 0
	UBRRL = 103;         // C?u hình t?c ?? baud cho UART
	UCSRC = (1 << URSEL) | (1 << UCSZ1) | (1 << UCSZ0); // Ch? ?? 8-bit
	UCSRB = (1 << TXEN); // Kích ho?t truy?n UART

	abc.begin();         // Kh?i t?o MFRC522
	_SendString("START");
	LCD_SendString("Moi quet the");

	while(1)
	{
		// X? lý th? RFID
		uint8_t status;
		uint8_t data[MAX_LEN];
		status = abc.requestTag(MF1_REQIDL, data);
		if (status == MI_OK) {
			status = abc.antiCollision(data);
			sendUID(data);
			PORTA |= (1 << PA2); // B?t LED
			_delay_ms(50);       // ??i 0.5s
			PORTA &= ~(1 << PA2); // T?t LED
			_delay_ms(50);       // ??i 0.5s
			LCD_SendCmd(0x01);   // Xóa LCD
			_delay_ms(2);        // ??i xóa
			LCD_SendString("Moi ra");

			Servo_SetAngle(90);  // ??a servo v? 90 ??
			_delay_ms(1000);     // ??i 3 giây
			Servo_SetAngle(0);   // ??a servo v? 0 ??
			_delay_ms(500);     // ??i 1 giây

			LCD_SendCmd(0x01);   // Xóa LCD
			_delay_ms(2);        // ??i xóa
			LCD_SendString("Moi quet the"); // Hi?n th? l?i thông báo
			abc.selectTag(data);
			abc.haltTag();       // D?ng th? sau khi x? lý
		}
	}
}