#define F_CPU 16000000UL  // T?n s? CPU
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>  // Th? vi?n ng?t

// ??nh ngh?a các chân
#define IR1_PIN PC4
#define IR2_PIN PC0
#define RELAY1_PIN PD1
#define RELAY2_PIN PD2
#define RELAY3_PIN PD3
#define RELAY4_PIN PD4

#define LCD_RS PB5
#define LCD_EN PB4
#define LCD_D4 PB0
#define LCD_D5 PB1
#define LCD_D6 PB2
#define LCD_D7 PB3

volatile unsigned long millis_count = 0;  // Bi?n ??m mili giây
volatile int people_count = 0;            // Bi?n ??m s? ng??i

// **Hàm millis() thay th?**
unsigned long millis() {
	unsigned long ms;
	cli();  // T?m d?ng ng?t ?? tránh l?i khi ??c millis_count
	ms = millis_count;
	sei();  // B?t l?i ng?t
	return ms;
}

// **Hàm ?i?u khi?n LCD**
void lcd_command(uint8_t cmd) {
	PORTB = (PORTB & 0xF0) | (cmd >> 4);  // G?i 4 bit cao
	PORTB &= ~(1 << LCD_RS);              // Ch?n ch? ?? command
	PORTB |= (1 << LCD_EN);               // Kích ho?t EN
	_delay_us(1);
	PORTB &= ~(1 << LCD_EN);

	PORTB = (PORTB & 0xF0) | (cmd & 0x0F);  // G?i 4 bit th?p
	PORTB |= (1 << LCD_EN);                 // Kích ho?t EN
	_delay_us(1);
	PORTB &= ~(1 << LCD_EN);

	_delay_ms(2);
}

void lcd_data(uint8_t data) {
	PORTB = (PORTB & 0xF0) | (data >> 4);  // G?i 4 bit cao
	PORTB |= (1 << LCD_RS);                // Ch?n ch? ?? d? li?u
	PORTB |= (1 << LCD_EN);                // Kích ho?t EN
	_delay_us(1);
	PORTB &= ~(1 << LCD_EN);

	PORTB = (PORTB & 0xF0) | (data & 0x0F);  // G?i 4 bit th?p
	PORTB |= (1 << LCD_EN);                  // Kích ho?t EN
	_delay_us(1);
	PORTB &= ~(1 << LCD_EN);

	_delay_ms(2);
}

void lcd_init() {
	DDRB |= (1 << LCD_RS) | (1 << LCD_EN) | (1 << LCD_D4) | (1 << LCD_D5) | (1 << LCD_D6) | (1 << LCD_D7);

	_delay_ms(20);
	lcd_command(0x02);  // Ch? ?? 4-bit
	lcd_command(0x28);  // Giao di?n 2 dòng, 4-bit
	lcd_command(0x0C);  // B?t màn hình, t?t con tr?
	lcd_command(0x06);  // T? ??ng t?ng con tr?
	lcd_command(0x01);  // Xóa màn hình
	_delay_ms(2);
}

void lcd_set_cursor(uint8_t row, uint8_t col) {
	uint8_t addr = (row == 0 ? 0x00 : 0x40) + col;
	lcd_command(0x80 | addr);
}

void lcd_print(char *str) {
	while (*str) {
		lcd_data(*str++);
	}
}

// **Hàm ki?m tra tr?ng thái c?m bi?n**
int is_sensor_triggered(uint8_t pin) {
	return (PINC & (1 << pin)) == 0;  // Tr? v? 1 n?u c?m bi?n kích ho?t
}

// **Hàm ?i?u khi?n r?-le**
void control_relay() {
	PORTD &= ~((1 << RELAY1_PIN) | (1 << RELAY2_PIN) | (1 << RELAY3_PIN) | (1 << RELAY4_PIN));  // T?t t?t c?

	if (people_count >= 1 && people_count <= 6) {
		PORTD |= (1 << RELAY1_PIN);  // B?t ?èn 1
		} else if (people_count >= 7 && people_count <= 12) {
		PORTD |= (1 << RELAY1_PIN) | (1 << RELAY2_PIN);  // B?t ?èn 1, 2
		} else if (people_count >= 13 && people_count <= 18) {
		PORTD |= (1 << RELAY1_PIN) | (1 << RELAY2_PIN) | (1 << RELAY3_PIN);  // B?t ?èn 1, 2, 3
		} else if (people_count >= 19) {
		PORTD |= (1 << RELAY1_PIN) | (1 << RELAY2_PIN) | (1 << RELAY3_PIN) | (1 << RELAY4_PIN);  // B?t t?t c?
	}
}

// **C?p nh?t LCD**
void update_lcd() {
	char buffer[16];
	lcd_command(0x01);  // Xóa màn hình
	_delay_ms(2);       // Ch? LCD x? lý l?nh xóa
	lcd_set_cursor(0, 0);
	lcd_print("People Count:");  // Hi?n th? nhãn
	lcd_set_cursor(1, 0);
	sprintf(buffer, "%03d", people_count);  // Chuy?n ??i s? ng??i thành chu?i
	lcd_print(buffer);  // Hi?n th? s? ng??i
}

// **Hàm x? lý tín hi?u c?m bi?n**
void handle_sensors() {
	static int prev_ir1 = 0, prev_ir2 = 0;

	int ir1 = is_sensor_triggered(IR1_PIN);
	int ir2 = is_sensor_triggered(IR2_PIN);

	if (ir1 && !prev_ir1) {  // IR1 ???c kích ho?t tr??c
		if (!ir2) {  // IR2 không kích ho?t => Ng??i vào
			if (people_count < 256) people_count++;
			control_relay();
			update_lcd();
		}
	}

	if (ir2 && !prev_ir2) {  // IR2 ???c kích ho?t tr??c
		if (!ir1) {  // IR1 không kích ho?t => Ng??i ra
			if (people_count > 0) people_count--;
			control_relay();
			update_lcd();
		}
	}

	prev_ir1 = ir1;
	prev_ir2 = ir2;
}

// **C?u hình Timer0 ?? t?o ng?t m?i 1ms**
void timer0_init() {
	TCCR0A |= (1 << WGM01);  // Ch? ?? CTC
	TCCR0B |= (1 << CS01) | (1 << CS00);  // Prescaler 64
	OCR0A = 249;  // T?o ng?t m?i 1ms
	TIMSK0 |= (1 << OCIE0A);  // B?t ng?t so sánh
	sei();  // Cho phép ng?t toàn c?c
}

// **Ng?t Timer0**
ISR(TIMER0_COMPA_vect) {
	millis_count++;
}

// **Ch??ng trình chính**
int main() {
	DDRC &= ~((1 << IR1_PIN) | (1 << IR2_PIN));  // C?m bi?n là ??u vào
	DDRD |= ((1 << RELAY1_PIN) | (1 << RELAY2_PIN) | (1 << RELAY3_PIN) | (1 << RELAY4_PIN));  // R?-le là ??u ra

	lcd_init();
	timer0_init();  // Kh?i t?o Timer0
	update_lcd();

	while (1) {
		handle_sensors();
		_delay_ms(50);  // Gi?m t?i CPU
	}

	return 0;
}