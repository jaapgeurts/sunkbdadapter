/*
SoftwareSerial.cpp (formerly NewSoftSerial.cpp) - 
Multi-instance software serial library for Arduino/Wiring
-- Interrupt-driven receive and other improvements by ladyada
   (http://ladyada.net)
-- Tuning, circular buffer, derivation from class Print/Stream,
   multi-instance support, porting to 8MHz processors,
   various optimizations, PROGMEM delay tables, inverse logic and 
   direct port writing by Mikal Hart (http://www.arduiniana.org)
-- Pin change interrupt macros by Paul Stoffregen (http://www.pjrc.com)
-- 20MHz processor support by Garrett Mace (http://www.macetech.com)
-- ATmega1280/2560 support by Brett Hagman (http://www.roguerobotics.com/)

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

The latest version of this library can always be found at
http://arduiniana.org.
*/

// When set, _DEBUG co-opts pins 11 and 13 for debugging with an
// oscilloscope or logic analyzer.  Beware: it also slightly modifies
// the bit times, so don't rely on it too much at high baud rates
#define _DEBUG 0
#define _DEBUG_PIN1 11
#define _DEBUG_PIN2 13

#define LOW (0x0)
#define HIGH (0x1)

#define TX_ARD 8
#define RX_ARD 9
#define RX_PIN PB5
#define TX_PIN PB4
//
// Includes
//

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <SoftwareSerial.h>
#include <util/delay_basic.h>
#include <stdlib.h>


#define PIN_A8 (26)
#define PIN_A10 (28)

// TODO Cleanup this arduino junk
static const uint8_t A8 = PIN_A8;   // D8
static const uint8_t A10 = PIN_A10; // D10

#define digitalPinToPCICR(p) ((((p) >= 8 && (p) <= 11) || ((p) >= 14 && (p) <= 17) || ((p) >= A8 && (p) <= A10)) ? (&PCICR) : ((uint8_t *)0))
#define digitalPinToPCICRbit(p) 0
#define digitalPinToPCMSK(p) ((((p) >= 8 && (p) <= 11) || ((p) >= 14 && (p) <= 17) || ((p) >= A8 && (p) <= A10)) ? (&PCMSK0) : ((uint8_t *)0))
#define digitalPinToPCMSKbit(p) (((p) >= 8 && (p) <= 11) ? (p)-4 : ((p) == 14 ? 3 : ((p) == 15 ? 1 : ((p) == 16 ? 2 : ((p) == 17 ? 0 : (p - A8 + 4))))))

// static data
static uint8_t _receive_buffer[_SS_MAX_RX_BUFF];
static volatile uint8_t _receive_buffer_tail = 0;
static volatile uint8_t _receive_buffer_head = 0;
static sserial_t *active_object = NULL;

void ss_setRxIntMsk(sserial_t* data,bool enable);
uint8_t ss_rx_pin_read(sserial_t* data);
void ss_handle_interrupt(void);

//
// Debugging
//
// This function generates a brief pulse
// for debugging or measuring on an oscilloscope.
#if _DEBUG
inline void DebugPulse(uint8_t pin, uint8_t count)
{
  volatile uint8_t *pport = portOutputRegister(digitalPinToPort(pin));

  uint8_t val = *pport;
  while (count--)
  {
    *pport = val | digitalPinToBitMask(pin);
    *pport = val;
  }
}
#else
inline void DebugPulse(uint8_t a, uint8_t b)
{
}
#endif

//
// Private methods
//

/* static */
inline static void ss_tunedDelay(sserial_t* data,uint16_t delay)
{
  _delay_loop_2(delay);
}

// This function sets the current object as the "listening"
// one and returns true if it replaces another
bool ss_listen(sserial_t* data)
{
  if (!data->_rx_delay_stopbit)
    return false;

  if (active_object != data)
  {
    if (active_object)
      ss_stopListening(active_object);

    data->_buffer_overflow = false;
    _receive_buffer_head = _receive_buffer_tail = 0;
    active_object = data;

    ss_setRxIntMsk(data,true);
    return true;
  }

  return false;
}

// Stop listening. Returns true if we were actually listening.
bool ss_stopListening(sserial_t* data)
{
  if (active_object == data)
  {
    ss_setRxIntMsk(data,false);
    active_object = NULL;
    return true;
  }
  return false;
}

//
// The receive routine called by the interrupt handler
//
void ss_recv(sserial_t* data)
{

#if GCC_VERSION < 40302
  // Work-around for avr-gcc 4.3.0 OSX version bug
  // Preserve the registers that the compiler misses
  // (courtesy of Arduino forum user *etracer*)
  asm volatile(
      "push r18 \n\t"
      "push r19 \n\t"
      "push r20 \n\t"
      "push r21 \n\t"
      "push r22 \n\t"
      "push r23 \n\t"
      "push r26 \n\t"
      "push r27 \n\t" ::);
#endif

  uint8_t d = 0;

  // If RX line is high, then we don't see any start bit
  // so interrupt is probably not for us
  if (data->_inverse_logic ? ss_rx_pin_read(data) : !ss_rx_pin_read(data))
  {
    // Disable further interrupts during reception, this prevents
    // triggering another interrupt directly after we return, which can
    // cause problems at higher baudrates.
    ss_setRxIntMsk(data,false);

    // Wait approximately 1/2 of a bit width to "center" the sample
    ss_tunedDelay(data,data->_rx_delay_centering);
    DebugPulse(_DEBUG_PIN2, 1);

    // Read each of the 8 bits
    for (uint8_t i = 8; i > 0; --i)
    {
      ss_tunedDelay(data,data->_rx_delay_intrabit);
      d >>= 1;
      DebugPulse(_DEBUG_PIN2, 1);
      if (ss_rx_pin_read(data))
        d |= 0x80;
    }

    if (data->_inverse_logic)
      d = ~d;

    // if buffer full, set the overflow flag and return
    uint8_t next = (_receive_buffer_tail + 1) % _SS_MAX_RX_BUFF;
    if (next != _receive_buffer_head)
    {
      // save new data in buffer: tail points to where byte goes
      _receive_buffer[_receive_buffer_tail] = d; // save new byte
      _receive_buffer_tail = next;
    }
    else
    {
      DebugPulse(_DEBUG_PIN1, 1);
      data->_buffer_overflow = true;
    }

    // skip the stop bit
    ss_tunedDelay(data,data->_rx_delay_stopbit);
    DebugPulse(_DEBUG_PIN1, 1);

    // Re-enable interrupts when we're sure to be inside the stop bit
    ss_setRxIntMsk(data,true);
  }

#if GCC_VERSION < 40302
  // Work-around for avr-gcc 4.3.0 OSX version bug
  // Restore the registers that the compiler misses
  asm volatile(
      "pop r27 \n\t"
      "pop r26 \n\t"
      "pop r23 \n\t"
      "pop r22 \n\t"
      "pop r21 \n\t"
      "pop r20 \n\t"
      "pop r19 \n\t"
      "pop r18 \n\t" ::);
#endif
}

uint8_t ss_rx_pin_read(sserial_t* data)
{
  return PINB & _BV(RX_PIN);
}

//
// Interrupt handling
//

/* static */
inline void ss_handle_interrupt(void)
{
  if (active_object)
  {
    ss_recv(active_object);
  }
}

#if defined(PCINT0_vect)
ISR(PCINT0_vect)
{
  ss_handle_interrupt();
}
#endif

#if defined(PCINT1_vect)
ISR(PCINT1_vect, ISR_ALIASOF(PCINT0_vect));
#endif

#if defined(PCINT2_vect)
ISR(PCINT2_vect, ISR_ALIASOF(PCINT0_vect));
#endif

#if defined(PCINT3_vect)
ISR(PCINT3_vect, ISR_ALIASOF(PCINT0_vect));
#endif

//
// Constructor
//
sserial_t* ss_create(bool inverse_logic) 
{

  sserial_t* t = calloc(1,sizeof(sserial_t));
  t->_inverse_logic = inverse_logic;
  //#define TX 8

  //  digitalWrite(tx, _inverse_logic ? LOW : HIGH);
  if(t->_inverse_logic)
    PORTB &= ~_BV(TX_PIN);
  else
    PORTB |= _BV(TX_PIN); 

  //  pinMode(tx, OUTPUT);
  DDRB |= _BV(DDB4);

  //#define RX 9
  //pinMode(rx, INPUT);
  DDRB &= ~(_BV(DDB5));
  if (!t->_inverse_logic)
    //digitalWrite(rx, HIGH);  // pullup for normal logic!
    PORTB |= _BV(RX_PIN);

  return t;
}

//
// Destructor
//
void ss_free(sserial_t* data)
{
  ss_end(data);
}

uint16_t ss_subtract_cap(sserial_t* data,uint16_t num, uint16_t sub)
{
  if (num > sub)
    return num - sub;
  else
    return 1;
}

//
// Public methods
//

void ss_begin(sserial_t* data,long speed)
{
  data->_rx_delay_centering = data->_rx_delay_intrabit =data-> _rx_delay_stopbit = data->_tx_delay = 0;

  // Precalculate the various delays, in number of 4-cycle delays
  uint16_t bit_delay = (F_CPU / speed) / 4;

  // 12 (gcc 4.8.2) or 13 (gcc 4.3.2) cycles from start bit to first bit,
  // 15 (gcc 4.8.2) or 16 (gcc 4.3.2) cycles between bits,
  // 12 (gcc 4.8.2) or 14 (gcc 4.3.2) cycles from last bit to stop bit
  // These are all close enough to just use 15 cycles, since the inter-bit
  // timings are the most critical (deviations stack 8 times)
  data->_tx_delay = ss_subtract_cap(data,bit_delay, 15 / 4);

  // Only setup rx when we have a valid PCINT for this pin
  if (digitalPinToPCICR((int8_t)RX_ARD))
  {
#if GCC_VERSION > 40800
    // Timings counted from gcc 4.8.2 output. This works up to 115200 on
    // 16Mhz and 57600 on 8Mhz.
    //
    // When the start bit occurs, there are 3 or 4 cycles before the
    // interrupt flag is set, 4 cycles before the PC is set to the right
    // interrupt vector address and the old PC is pushed on the stack,
    // and then 75 cycles of instructions (including the RJMP in the
    // ISR vector table) until the first delay. After the delay, there
    // are 17 more cycles until the pin value is read (excluding the
    // delay in the loop).
    // We want to have a total delay of 1.5 bit time. Inside the loop,
    // we already wait for 1 bit time - 23 cycles, so here we wait for
    // 0.5 bit time - (71 + 18 - 22) cycles.
    data->_rx_delay_centering = ss_subtract_cap(data,bit_delay / 2, (4 + 4 + 75 + 17 - 23) / 4);

    // There are 23 cycles in each loop iteration (excluding the delay)
    data->_rx_delay_intrabit = ss_subtract_cap(data,bit_delay, 23 / 4);

    // There are 37 cycles from the last bit read to the start of
    // stopbit delay and 11 cycles from the delay until the interrupt
    // mask is enabled again (which _must_ happen during the stopbit).
    // This delay aims at 3/4 of a bit time, meaning the end of the
    // delay will be at 1/4th of the stopbit. This allows some extra
    // time for ISR cleanup, which makes 115200 baud at 16Mhz work more
    // reliably
    data->_rx_delay_stopbit = ss_subtract_cap(data,bit_delay * 3 / 4, (37 + 11) / 4);
#else // Timings counted from gcc 4.3.2 output
    // Note that this code is a _lot_ slower, mostly due to bad register
    // allocation choices of gcc. This works up to 57600 on 16Mhz and
    // 38400 on 8Mhz.
    data->_rx_delay_centering = ss_subtract_cap(data,bit_delay / 2, (4 + 4 + 97 + 29 - 11) / 4);
    data->_rx_delay_intrabit = ss_subtract_cap(data,bit_delay, 11 / 4);
    data->_rx_delay_stopbit = ss_subtract_cap(data,bit_delay * 3 / 4, (44 + 17) / 4);
#endif

    // Enable the PCINT for the entire port here, but never disable it
    // (others might also need it, so we disable the interrupt by using
    // the per-pin PCMSK register).
    *digitalPinToPCICR((int8_t)RX_ARD) |= _BV(digitalPinToPCICRbit(RX_ARD));
    // Precalculate the pcint mask register and value, so setRxIntMask
    // can be used inside the ISR without costing too much time.
    data->_pcint_maskreg = digitalPinToPCMSK(RX_ARD);
    data->_pcint_maskvalue = _BV(digitalPinToPCMSKbit(RX_ARD));

    ss_tunedDelay(data,data->_tx_delay); // if we were low this establishes the end
  }

#if _DEBUG
  pinMode(_DEBUG_PIN1, OUTPUT);
  pinMode(_DEBUG_PIN2, OUTPUT);
#endif

  ss_listen(data);
}

void ss_setRxIntMsk(sserial_t* data,bool enable)
{
  if (enable)
    *(data->_pcint_maskreg) |= data->_pcint_maskvalue;
  else
    *(data->_pcint_maskreg) &= ~(data->_pcint_maskvalue);
}

void ss_end(sserial_t* data)
{
  ss_stopListening(data);
}

bool overflow(sserial_t* data)
{
  bool ret = data->_buffer_overflow;
  if (ret)
    data->_buffer_overflow = false;
  return ret;
}

bool ss_isListening(sserial_t* data)
{
  return data == active_object;
}

// Read data from buffer
int ss_read(sserial_t* data)
{
  if (!ss_isListening(data))
    return -1;

  // Empty buffer?
  if (_receive_buffer_head == _receive_buffer_tail)
    return -1;

  // Read from "head"
  uint8_t d = _receive_buffer[_receive_buffer_head]; // grab next byte
  _receive_buffer_head = (_receive_buffer_head + 1) % _SS_MAX_RX_BUFF;
  return d;
}

int ss_available(sserial_t* data)
{
  if (!ss_isListening(data))
    return 0;

  return (_receive_buffer_tail + _SS_MAX_RX_BUFF - _receive_buffer_head) % _SS_MAX_RX_BUFF;
}

size_t ss_write(sserial_t* data,uint8_t b)
{
  if (data->_tx_delay == 0)
  {
    //setWriteError();
    return 0;
  }

  // By declaring these as local variables, the compiler will put them
  // in registers _before_ disabling interrupts and entering the
  // critical timing sections below, which makes it a lot easier to
  // verify the cycle timings

  uint8_t oldSREG = SREG;
  bool inv = data->_inverse_logic;
  uint16_t delay = data->_tx_delay;

  if (inv)
    b = ~b;

  cli(); // turn off interrupts for a clean txmit

  // Write the start bit
  if (inv)
    PORTB |= _BV(TX_PIN);
  else
    PORTB &= ~_BV(TX_PIN);

  ss_tunedDelay(data,delay);

  // Write each of the 8 bits
  for (uint8_t i = 8; i > 0; --i)
  {
    if (b & 1)            // choose bit
      PORTB |= _BV(TX_PIN); // send 1
    else
      PORTB &= ~(_BV(TX_PIN)); // send 0

    ss_tunedDelay(data,delay);
    b >>= 1;
  }

  // restore pin to natural state
  if (inv)
    PORTB &= ~(_BV(TX_PIN));
  else
    PORTB |= _BV(TX_PIN);

  SREG = oldSREG; // turn interrupts back on
  ss_tunedDelay(data,data->_tx_delay);

  return 1;
}

size_t ss_write_bytes(sserial_t* data,uint8_t* byte, size_t nbem)
{
  for (int i=0;i<nbem;i++)
    ss_write(data,byte[i]);
}



void ss_flush(sserial_t* data)
{
  // There is no tx buffering, simply return
}

int ss_peek(sserial_t* data)
{
  if (!ss_isListening(data))
    return -1;

  // Empty buffer?
  if (_receive_buffer_head == _receive_buffer_tail)
    return -1;

  // Read from "head"
  return _receive_buffer[_receive_buffer_head];
}
