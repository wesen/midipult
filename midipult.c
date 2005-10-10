/*
 * Midipult software, atmega8535
 */

#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/signal.h>

#define F_CPU     12000000UL /* 12 Mhz */
#include <avr/delay.h>

#define MIDI_BAUD 31250UL    /* 31.25 kbps */

#define RB_SIZE 16

typedef struct {
  char buf[RB_SIZE];
  unsigned int start, len;
} rb_t;

static rb_t tx_rb;
static rb_t rx_rb;

void rb_put(rb_t *rb, char c);
void uart_put(void);

/*** signal zeugs ***/

SIGNAL(SIG_UART_RECV) {
  if (!rb_full(&rx_rb)) {
    char c = UDR;
    rb_put(&rx_rb, c);
  }
}

SIGNAL(SIG_UART_TRANS) {
  PORTD ^= _BV(6);
  uart_put();
}

/*
EMPTY_INTERRUPT( SIG_INTERRUPT0);
EMPTY_INTERRUPT( SIG_INTERRUPT1);
EMPTY_INTERRUPT( SIG_OUTPUT_COMPARE2);
EMPTY_INTERRUPT( SIG_OVERFLOW2);
EMPTY_INTERRUPT( SIG_INPUT_CAPTURE1);
EMPTY_INTERRUPT( SIG_OUTPUT_COMPARE1A);
EMPTY_INTERRUPT( SIG_OUTPUT_COMPARE1B);
EMPTY_INTERRUPT( SIG_OVERFLOW1);
EMPTY_INTERRUPT( SIG_OVERFLOW0);
EMPTY_INTERRUPT( SIG_SPI);
EMPTY_INTERRUPT( SIG_UART_DATA);
EMPTY_INTERRUPT( SIG_ADC);
EMPTY_INTERRUPT( SIG_EEPROM_READY);
EMPTY_INTERRUPT( SIG_COMPARATOR);
EMPTY_INTERRUPT( SIG_2WIRE_SERIAL);
EMPTY_INTERRUPT( SIG_INTERRUPT2);
EMPTY_INTERRUPT( SIG_OUTPUT_COMPARE0);
EMPTY_INTERRUPT( SIG_SPM_READY);
*/

/*** ring buffer zeugs ***/

void rb_init(rb_t *rb) {
  rb->start = rb->len = 0;
}

char rb_get(rb_t *rb) {
  char res = 0;
  if (rb->len > 0) {
    res = rb->buf[rb->start];
    rb->start = (rb->start + 1) % RB_SIZE;
    rb->len--;
  }
  return res;
}

void rb_put(rb_t *rb, char c) {
  if (rb->len < RB_SIZE)
    rb->buf[(rb->start + rb->len++) % RB_SIZE] = c;
}

int rb_full(rb_t *rb) {
  return (rb->len < RB_SIZE);
}

int rb_empty(rb_t *rb) {
  return (rb->len == 0);
}

unsigned int rb_len(rb_t *rb) {
  return rb->len;
}

unsigned int rb_free(rb_t *rb) {
  return (RB_SIZE - rb->len);
}


#define loop_until_rb_free(rb, len) do {} while(rb_free((rb)) < (len))

/*** UART zeugs ***/

void io_init(void) {
  /* disable watchdog */
  WDTCR = 0;

  /* DD = 1, output */
  /* PORT = 1 wenn DD = 0, dann pull-up */
  /* portd = output */
  DDRD = 0XFF;
  
  /* set baudrate */
  UBRRL = (((F_CPU) / (16 * MIDI_BAUD)) - 1);
  /* enable TX, RX, interrupts */
  UCSRB = _BV(TXEN) | _BV(RXEN) | _BV(RXCIE) | _BV(TXCIE);

  /* XXX initialize ADCs, pull-ups, etc... */


  /* enable interrupts */
  sei();
}

void uart_put(void) {
  if (!rb_empty(&tx_rb)) {
    char c = rb_get(&tx_rb);
    UDR = c;
  }
}

void uart_flush_tx(void) {
  loop_until_bit_is_set(UCSRA, UDRE);
  uart_put();
}

/*** MIDI zeugs ***/

#define MIDI_NOTE_ON     0x8
#define MIDI_NOTE_OFF    0x9
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
  loop_until_rb_free(&tx_rb, 3);
  rb_put(&tx_rb, (MIDI_NOTE_ON << 4) | channel);
  rb_put(&tx_rb, note);
  rb_put(&tx_rb, velocity);
  uart_flush_tx();
}

void midi_send_note_off(unsigned char channel, unsigned char note, unsigned char velocity) {
  loop_until_rb_free(&tx_rb, 3);
  rb_put(&tx_rb, (MIDI_NOTE_OFF << 4) | channel);
  rb_put(&tx_rb, note);
  rb_put(&tx_rb, velocity);
  uart_flush_tx();
}

/*** knoepfe und LEDs ***/

#define SET_PIN_OUTPUT(port, pin) { DDR ## port |= _BV(pin); }
#define SET_PIN_INPUT(port, pin) { DDR ## port &= ~(_BV(pin)); }
#define SET_PIN_INPUT_PULLUP(port, pin) { SET_PIN_INPUT(port, pin); PORT ## port |= _BV(pin); }
#define SET_PIN_LOW(port, pin) { PORT ## port &= ~(_BV(pin)); }
#define SET_PIN_HIGH(port, pin) { PORT ## port |= _BV(pin); }
#define PIN_VALUE(port, pin) ((PIN ## port & _BV(pin)) ? 1 : 0)

#define SEL_PIN        2
#define LED_BUTTON_PIN 3

unsigned char led_status = 0;
unsigned char button_status = 0;

void ask_button(void) {
  SET_PIN_OUTPUT(D, SEL_PIN);
  SET_PIN_LOW(D, SEL_PIN);
  SET_PIN_INPUT_PULLUP(D, LED_BUTTON_PIN);
  _delay_us(10);
  button_status = !PIN_VALUE(D, LED_BUTTON_PIN);
}
void set_led(void) {
  SET_PIN_OUTPUT(D, LED_BUTTON_PIN);
  SET_PIN_LOW(D, LED_BUTTON_PIN);
  SET_PIN_OUTPUT(D, SEL_PIN);
  if (led_status) {
    SET_PIN_HIGH(D, SEL_PIN);
  } else {
    SET_PIN_LOW(D, SEL_PIN);
  }
}

/*** Main ***/

int main(void) {
  rb_init(&tx_rb);
  rb_init(&rx_rb);
  io_init();
  
  for (;;) {
    PORTD ^= _BV(4);
    if (rb_empty(&tx_rb)) {
      if (button_status)
	rb_put(&tx_rb, 0xFF);
      else
	rb_put(&tx_rb, 0x00);
      uart_flush_tx();
    } else {
      uart_flush_tx();
    }
    ask_button();
    led_status = button_status;
    set_led();
  }
}
