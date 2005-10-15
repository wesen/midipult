/*
 * Midipult software, atmega8535
 */

/* 15 + 18 = 33 knoepfe */
/* 56 potis */
#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/signal.h>

#define F_CPU     12000000UL /* 12 Mhz */
#include <avr/delay.h>


/*** UART zeugs ***/

#define RB_SIZE 16
#define MIDI_BAUD 31250UL    /* 31.25 kbps */

volatile char tx_buf[RB_SIZE];
volatile char tx_wr = 0, tx_rd = 0;
#define TX_INCR(a, b) (((a) + (b)) & (RB_SIZE - 1))
#define UART_TX_OK (UCSRA & _BV(UDRE))
#define TX_FREE (!(TX_INCR(tx_wr, 1) == tx_rd))

SIGNAL(SIG_UART_TRANS) {
  PORTD ^= _BV(6);
  if (tx_rd != tx_wr) {
    UDR = tx_buf[tx_rd];
    tx_rd = TX_INCR(tx_rd, 1);
  }
}

void uart_putc(char c) {
  if (UART_TX_OK && (tx_wr == tx_rd)) {
    UDR = c;
  } else {
    while (TX_INCR(tx_wr, 1) == tx_rd) ;
    tx_buf[tx_wr] = c;
    tx_wr = TX_INCR(tx_wr, 1);
  }
}

void uart_init(void) {
  tx_wr = tx_rd = 0;
  
  /* set baudrate */
  UBRRL = (((F_CPU) / (16 * MIDI_BAUD)) - 1);
  /* enable TX, interrupts */
  UCSRB = _BV(TXEN) | _BV(TXCIE);

  sei();
}

/*** UART zeugs ***/

void io_init(void) {
  /* disable watchdog */
  WDTCR = 0;

  /* DD = 1, output */
  /* PORT = 1 wenn DD = 0, dann pull-up */
  /* portd = output */
  DDRD = 0XFF;
  
  /* enable ADC */
  DDRA = 0;
  PORTA = 0xFF;
  ADMUX = _BV(ADLAR);
  ADCSRA = _BV(ADEN)|_BV(ADPS2)|_BV(ADPS1)|_BV(ADPS0);
  // ADCSRA = _BV(ADEN);

  uart_init();
}

/*** MIDI zeugs ***/

#define MIDI_NOTE_ON     0x9
#define MIDI_NOTE_OFF    0x8
#define MIDI_AFTERTOUCH  0xA
#define MIDI_CC          0xB
#define MIDI_PROG_CHANGE 0xC
#define MIDI_CHAN_PRESS  0xD
#define MIDI_PITCH_WHEEL 0xE

#define MIDI_STATUS_TYPE(c) (((c) & 0xF0) >> 4)
#define MIDI_STATUS_CHAN(c) ((c) & 0xF)

static inline midi_is_status(char c) {
  return (c & 0x80);
}

void midi_send_note_on(unsigned char channel, unsigned char note, unsigned char velocity) {
  uart_putc((MIDI_NOTE_ON << 4) | channel);
  uart_putc(note);
  uart_putc(velocity);
}

void midi_send_note_off(unsigned char channel, unsigned char note, unsigned char velocity) {
  uart_putc((MIDI_NOTE_OFF << 4) | channel);
  uart_putc(note);
  uart_putc(velocity);
}

void midi_send_cc(unsigned char channel, unsigned char controller, unsigned char value) {
  uart_putc((MIDI_CC << 4) | channel);
  uart_putc(controller);
  uart_putc(value);
}

/*** knoepfe und LEDs ***/

#define LED_BUTTON_SEL_PORT        C
#define PORTLED_BUTTON_SEL_PORT        PORTC
#define DDRLED_BUTTON_SEL_PORT  DDRC

#define LED_BUTTON_PORT     D
#define PORTLED_BUTTON_PORT PORTD
#define DDRLED_BUTTON_PORT  DDRD
#define PINLED_BUTTON_PORT  PIND

#define SET_PIN_OUTPUT(port, pin) { DDR ## port |= _BV(pin); }
#define SET_PIN_INPUT(port, pin) { DDR ## port &= ~(_BV(pin)); }
#define SET_PIN_INPUT_PULLUP(port, pin) { SET_PIN_INPUT(port, pin); PORT ## port |= _BV(pin); }
#define SET_PIN_LOW(port, pin) { PORT ## port &= ~(_BV(pin)); }
#define SET_PIN_HIGH(port, pin) { PORT ## port |= _BV(pin); }
#define PIN_VALUE(port, pin) ((PIN ## port & _BV(pin)) ? 1 : 0)

#define NUM_BUTTONS 30

#define BUTTON_STATUS(i) (button_status[(i)] & 0xF)
#define SET_BUTTON_STATUS(i, val) { button_status[(i)] = button_status[(i)] & 0xF0 | (val); }
#define LED_STATUS(i) ((button_status[(i)] >> 4) & 0xF)
#define SET_LED_STATUS(i, val) { button_status[(i)] = button_status[(i)] & 0xF | ((val) << 4); }

#define BUTTON_NOTHING  0
#define BUTTON_PRESSED  1
#define BUTTON_NORMAL   2
#define BUTTON_RELEASED 3

/* debounce:
   2 bits pro knopf
   0 -> 1 (bounce?) -> 2
   wenn knopf auf 1 dann nachricht senden (note on), wieder auf 0 (note off)
*/

#define BUTTON_ROWS 5
unsigned char button_status[BUTTON_ROWS * 8];

void init_buttons(void) {
  int i, j;
  for (i = 0; i < BUTTON_ROWS; i++)
    for (j = 0; j < 8; j++)
      button_status[i + j * BUTTON_ROWS] = BUTTON_NOTHING;
}

void ask_button(unsigned char button_pin, unsigned char sel_pin) {
  SET_PIN_LOW(LED_BUTTON_SEL_PORT, sel_pin);
  SET_PIN_INPUT_PULLUP(LED_BUTTON_PORT, button_pin + 2);
  unsigned char val = PIN_VALUE(LED_BUTTON_PORT, button_pin + 2);
  SET_PIN_HIGH(LED_BUTTON_SEL_PORT, sel_pin);
  unsigned char button = sel_pin * BUTTON_ROWS + button_pin;
  switch (button_status[button]) {
  case BUTTON_NOTHING:
    if (!val)
      button_status[button] = BUTTON_PRESSED;
    break;
  case BUTTON_PRESSED:
  case BUTTON_NORMAL:
    if (val)
      button_status[button] = BUTTON_RELEASED;
    break;
  case BUTTON_RELEASED:
    if (val)
      button_status[button] = BUTTON_NOTHING;
    else
      button_status[button] = BUTTON_PRESSED;
    break;
  }
}

void ask_buttons(void) {
  int i, j;

  /* all high */
  DDRLED_BUTTON_SEL_PORT = 0xFF;
  PORTLED_BUTTON_SEL_PORT = 0xFF;
  
  for (i = 0; i < BUTTON_ROWS; i++)
    for (j = 0; j < 8; j++)
      ask_button(i, j);
}

void midi_buttons(void) {
  int i, j;
  for (i = 0; i < BUTTON_ROWS; i++)
    for (j = 0; j < 8; j++) {
      unsigned char button = i + j * BUTTON_ROWS;
      switch (button_status[button]) {
      case BUTTON_NOTHING:
      case BUTTON_NORMAL:
	break;
      case BUTTON_PRESSED:
	midi_send_note_on(1, 30 + button, 128);
	button_status[button] = BUTTON_NORMAL;
	//	_delay_ms(10);
	break;
      case BUTTON_RELEASED:
	midi_send_note_off(1, 30 + button, 128);
	button_status[button] = BUTTON_NOTHING;
	//	_delay_ms(10);
	break;
      }
    }
}

#if 0
void set_led(unsigned char sel_pin, unsigned char led_pin) {
  SET_PIN_OUTPUT(LED_BUTTON_PORT, led_pin);
  SET_PIN_LOW(LED_BUTTON_PORT, led_pin);
  SET_PIN_OUTPUT(LED_BUTTON_SEL_PORT, sel_pin);
  if (LED_STATUS(sel_pin * led_pin)) {
    SET_PIN_HIGH(LED_BUTTON_SEL_PORT, sel_pin);
  } else {
    SET_PIN_LOW(LED_BUTTON_SEL_PORT, sel_pin);
  }
}

void set_leds(void) {
  int i, j;
  for (i = 0; i < 6; i++)
    for (j = 0; j < 8; j++)
      set_led(i, j);
}
#endif
		       
/*** ADCs ***/

#define POTI_SEL_PORT        B
#define PORTPOTI_SEL_PORT PORTB
#define DDRPOTI_SEL_PORT  DDRB

#define NUM_POTIS 64
unsigned char poti_status[NUM_POTIS];

void init_potis(void) {
  int i, j;
  for (i = 0; i < 8; i++)
    for (j = 0; j < 8; j++)
      poti_status[i * 8 + j] = 0x00;
}

volatile unsigned short last_adc;

#define MIN_THRESHOLD 0x2100
#define MAX_THRESHOLD 0xF000

unsigned char convert_to_cc(unsigned short adc) {
  if (adc < MIN_THRESHOLD) return 0;
  if (adc > MAX_THRESHOLD) return 0x7F;
  uint32_t bla = (uint32_t)((uint32_t)adc - (uint32_t)MIN_THRESHOLD);
  bla *= 0x7F;
  bla /= 0xDFF0;
  return bla;
}

void ask_poti(unsigned char sel_pin, unsigned char adc) {
  SET_PIN_LOW(POTI_SEL_PORT, sel_pin);
  _delay_us(50);
  ADMUX = _BV(ADLAR) | adc;
  ADCSRA |= _BV(ADSC);
  loop_until_bit_is_set(ADCSRA, ADIF);
  loop_until_bit_is_clear(ADCSRA, ADSC);
  last_adc = ADC;
  SET_PIN_HIGH(POTI_SEL_PORT, sel_pin);
  unsigned char val = convert_to_cc(last_adc);
  unsigned char poti = sel_pin + 8 * adc;
  unsigned char old = poti_status[poti] & 0x7f;
  if ((val != old) && ((val == 00) || (val == 0x7F) || (abs(val - old) > 1)))
    poti_status[poti] = 0x80 | val;
}

void ask_potis(void) {
  int i, j;

  DDRPOTI_SEL_PORT = 0xFF;
  PORTPOTI_SEL_PORT = 0xFF;

  for (i = 0; i < 8; i++)
    for (j = 0; j < 8; j++)
      ask_poti(i, j);
}

void midi_potis(void) {
  int i, j;
  for (i = 0; i < 8; i++)
    for (j = 0; j < 8; j++) {
      unsigned char poti = i * 8 + j;
      if (poti_status[poti] & 0x80) {
	midi_send_cc(1, poti, poti_status[poti] & 0x7F);
	poti_status[poti] &= 0x7F;
      }
    }
}

/*** Main ***/

int main(void) {
  io_init();

  init_buttons();
  init_potis();

  DDRPOTI_SEL_PORT = 0xFF;
  PORTPOTI_SEL_PORT = 0xFF;
  DDRA = 0x00;
  PORTA = 0xFF;
  ADMUX = 0;

  int i;
  for (i =0 ; i < 100; i++) {
    ask_buttons();
    ask_potis();
  }
  
  for (;;) {
    
    ask_buttons();

    midi_buttons();
    
    ask_potis();
    midi_potis();
  }
}
