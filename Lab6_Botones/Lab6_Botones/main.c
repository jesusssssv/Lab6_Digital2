#define F_CPU 16000000UL  // Frecuencia del ATmega328P @ 16 MHz

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdbool.h>

// Comandos por botón (1 byte)
#define CMD_UP        'U'
#define CMD_DOWN      'D'
#define CMD_RIGHT     'R'
#define CMD_LEFT      'L'
#define CMD_ACTION_A  'A'
#define CMD_ACTION_B  'B'
#define CMD_HEARTBEAT 'H'

// Pines
#define BTN_UP        (1 << PC3)    // PC3
#define BTN_DOWN      (1 << PB3)    // PB3
#define BTN_RIGHT     (1 << PB4)    // PB4
#define BTN_LEFT      (1 << PC2)    // PC2
#define BTN_ACTION_A  (1 << PD5)    // PD5
#define BTN_ACTION_B  (1 << PD2)    // PD2

// UART TX buffer
#define UART_BUFFER_SIZE 32
volatile char uart_tx_buffer[UART_BUFFER_SIZE];
volatile uint8_t uart_tx_head = 0;
volatile uint8_t uart_tx_tail = 0;
volatile bool uart_transmitting = false;

// Estados previos para detectar flanco descendente
volatile uint8_t last_portb = 0;
volatile uint8_t last_portc = 0;
volatile uint8_t last_portd = 0;

// --- UART ---
static inline void uart_send_char(char data);

void uart_init(void) {
	// 9600 bps con F_CPU=16 MHz: UBRR = 103
	UBRR0H = 0;
	UBRR0L = 103;
	UCSR0B = (1 << TXEN0) | (1 << TXCIE0);         // TX + IRQ TX complete
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);        // 8 data, 1 stop, no parity
}

void uart_send_string(const char* s) {
	while (*s) { uart_send_char(*s++); }
}

static inline void uart_start_if_idle(void) {
	if (!uart_transmitting && (uart_tx_head != uart_tx_tail)) {
		uart_transmitting = true;
		UDR0 = uart_tx_buffer[uart_tx_tail];
		uart_tx_tail = (uart_tx_tail + 1) % UART_BUFFER_SIZE;
	}
}

static inline void uart_send_char(char data) {
	uint8_t next_head = (uart_tx_head + 1) % UART_BUFFER_SIZE;
	while (next_head == uart_tx_tail) { /* espera si buffer lleno */ }
	uart_tx_buffer[uart_tx_head] = data;
	uart_tx_head = next_head;
	uart_start_if_idle();
}

ISR(USART_TX_vect) {
	if (uart_tx_head != uart_tx_tail) {
		UDR0 = uart_tx_buffer[uart_tx_tail];
		uart_tx_tail = (uart_tx_tail + 1) % UART_BUFFER_SIZE;
		} else {
		uart_transmitting = false;
	}
}

// --- PIN CHANGE ---
void setup_pin_change_interrupts(void) {
	// Habilitar PCINT para B, C, D
	PCICR  = (1 << PCIE0) | (1 << PCIE1) | (1 << PCIE2);
	// Máscaras
	PCMSK0 = (1 << PCINT3) | (1 << PCINT4);    // PB3, PB4
	PCMSK1 = (1 << PCINT10) | (1 << PCINT11);  // PC2, PC3
	PCMSK2 = (1 << PCINT18) | (1 << PCINT21);  // PD2, PD5
	// Estados iniciales
	last_portb = PINB;
	last_portc = PINC;
	last_portd = PIND;
}

// PORTB (PB3, PB4)
ISR(PCINT0_vect) {
	uint8_t current = PINB;
	uint8_t changed = current ^ last_portb;

	// DOWN (PB3) flanco descendente
	if ((changed & BTN_DOWN) && !(current & BTN_DOWN)) {
		uart_send_char(CMD_DOWN);
		uart_send_char('\n');
	}

	// RIGHT (PB4) flanco descendente
	if ((changed & BTN_RIGHT) && !(current & BTN_RIGHT)) {
		uart_send_char(CMD_RIGHT);
		uart_send_char('\n');
	}

	last_portb = current;
}

// PORTC (PC2, PC3)
ISR(PCINT1_vect) {
	uint8_t current = PINC;
	uint8_t changed = current ^ last_portc;

	// UP (PC3)
	if ((changed & BTN_UP) && !(current & BTN_UP)) {
		uart_send_char(CMD_UP);
		uart_send_char('\n');
	}

	// LEFT (PC2)
	if ((changed & BTN_LEFT) && !(current & BTN_LEFT)) {
		uart_send_char(CMD_LEFT);
		uart_send_char('\n');
	}

	last_portc = current;
}

// PORTD (PD2, PD5)
ISR(PCINT2_vect) {
	uint8_t current = PIND;
	uint8_t changed = current ^ last_portd;

	// ACTION A (PD5)
	if ((changed & BTN_ACTION_A) && !(current & BTN_ACTION_A)) {
		uart_send_char(CMD_ACTION_A);
		uart_send_char('\n');
	}

	// ACTION B (PD2)
	if ((changed & BTN_ACTION_B) && !(current & BTN_ACTION_B)) {
		uart_send_char(CMD_ACTION_B);
		uart_send_char('\n');
	}

	last_portd = current;
}

int main(void) {
	// Entradas + pull-up
	DDRB &= ~(BTN_DOWN | BTN_RIGHT);
	PORTB |= (BTN_DOWN | BTN_RIGHT);

	DDRC &= ~(BTN_UP | BTN_LEFT);
	PORTC |= (BTN_UP | BTN_LEFT);

	DDRD &= ~(BTN_ACTION_A | BTN_ACTION_B);
	PORTD |= (BTN_ACTION_A | BTN_ACTION_B);

	uart_init();
	setup_pin_change_interrupts();
	sei();

	_delay_ms(500);

	// Mensaje inicial (opcional)
	uart_send_string("START\n");

	uint16_t heartbeat_counter = 0;

	while (1) {
		if (++heartbeat_counter >= 1000) { // ~10 s (1000*10ms)
			uart_send_char(CMD_HEARTBEAT);
			uart_send_char('\n');
			heartbeat_counter = 0;
		}
		_delay_ms(10);
	}
}
