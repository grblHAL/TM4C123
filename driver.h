/*
  driver.h - An embedded CNC Controller with rs274/ngc (g-code) support

  Grbl driver code for Texas Instruments Tiva C (TM4C123GH6PM) ARM processor

  Part of grblHAL

  Copyright (c) 2016-2021 Terje Io
  Copyright (c) 2011-2015 Sungeun K. Jeon
  Copyright (c) 2009-2011 Simen Svale Skogsrud

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

//
// NOTE: do NOT change configuration here - edit my_machine.h instead!
//

#ifndef __DRIVER_H__
#define __DRIVER_H__

#include "tiva.h"

#ifndef OVERRIDE_MY_MACHINE
#include "my_machine.h"
#endif

#include "grbl/driver_opts.h"

#ifndef CNC_BOOSTERPACK_SHORTS
#define CNC_BOOSTERPACK_SHORTS  0
#endif
#ifndef CNC_BOOSTERPACK_A4998
#define CNC_BOOSTERPACK_A4998   0
#endif

#define CNC_BOOSTERPACK         0

#define timerBase(t) timerB(t)
#define timerB(t) t ## _BASE
#define timerPeriph(t) timerP(t)
#define timerP(t) SYSCTL_PERIPH_ ## t
#define timerINT(t, i) timerI(t, i)
#define timerI(t, i) INT_ ## t ## i

// Define GPIO output mode options

#define GPIO_SHIFT0  0
#define GPIO_SHIFT1  1
#define GPIO_SHIFT2  2
#define GPIO_SHIFT3  3
#define GPIO_SHIFT4  4
#define GPIO_SHIFT5  5
#define GPIO_MAP     8
#define GPIO_BITBAND 9

// timer definitions

#define STEPPER_TIM WTIMER0
#define STEPPER_TIMER_PERIPH timerPeriph(STEPPER_TIM)
#define STEPPER_TIMER_BASE timerBase(STEPPER_TIM)
#define STEPPER_TIMER_INT timerINT(STEPPER_TIM, A)

#define PULSE_TIM TIMER0
#define PULSE_TIMER_PERIPH timerPeriph(PULSE_TIM)
#define PULSE_TIMER_BASE timerBase(PULSE_TIM)
#define PULSE_TIMER_INT timerINT(PULSE_TIM, A)

#define DEBOUNCE_TIM TIMER4
#define DEBOUNCE_TIMER_PERIPH timerPeriph(DEBOUNCE_TIM)
#define DEBOUNCE_TIMER_BASE timerBase(DEBOUNCE_TIM)
#define DEBOUNCE_TIMER_INT timerINT(DEBOUNCE_TIM, A)

#define SPINDLE_PWM_TIM TIMER3
#define SPINDLE_PWM_TIMER_PERIPH timerPeriph(SPINDLE_PWM_TIM)
#define SPINDLE_PWM_TIMER_BASE timerBase(SPINDLE_PWM_TIM)
//#define SPINDLE_PWM_TIMER_INT timerINT(SPINDLE_PWM_TIM, A)

#define DIGITAL_IN(port, pin) !!GPIOPinRead(port, 1<<pin)
#define DIGITAL_OUT(port, pin, on) GPIOPinWrite(port, 1<<pin, (on) ? 1<<pin : 0);

#ifdef BOARD_CNC_BOOSTERPACK
#include "cnc_boosterpack_map.h"
#elif defined(BOARD_MY_MACHINE)
#include "my_machine_map.h"
#else
#error No board!
#endif

// Adjust STEP_PULSE_LATENCY to get accurate step pulse length when required, e.g if using high step rates.
// The default value is calibrated for 10 microseconds length.
// NOTE: step output mode, number of axes and compiler optimization settings may all affect this value.
#ifndef STEP_PULSE_LATENCY
#define STEP_PULSE_LATENCY 1.3f // microseconds
#endif

#if I2C_STROBE_ENABLE || (TRINAMIC_ENABLE && TRINAMIC_I2C)
#define I2C_ENABLE 1
#else
#define I2C_ENABLE 0
#endif

// End configuration

#if TRINAMIC_ENABLE
#ifndef TRINAMIC_MIXED_DRIVERS
#define TRINAMIC_MIXED_DRIVERS 1
#endif
#include "motors/trinamic.h"
#include "trinamic/common.h"
#endif

#if PPI_ENABLE
#define PPI_ENABLE_TIM TIMER2
#define PPI_ENABLE_TIMER_PERIPH timerPeriph(PPI_ENABLE_TIM)
#define PPI_ENABLE_TIMER_BASE timerBase(PPI_ENABLE_TIM)
#define PPI_ENABLE_TIMER_INT timerINT(PPI_ENABLE_TIM, A)
#endif

#ifndef SPINDLE_DIRECTION_BIT
#define SPINDLE_DIRECTION_BIT (1<<SPINDLE_DIRECTION_PIN)
#endif
#ifndef SPINDLE_ENABLE_BIT
#define SPINDLE_ENABLE_BIT (1<<SPINDLE_ENABLE_PIN)
#endif

#ifndef COOLANT_FLOOD_BIT
#define COOLANT_FLOOD_BIT (1<<COOLANT_FLOOD_PIN)
#endif
#ifndef COOLANT_MIST_BIT
#define COOLANT_MIST_BIT (1<<COOLANT_MIST_PIN)
#endif

#ifndef PROBE_BIT
#define PROBE_BIT (1<<PROBE_PIN)
#endif

#ifdef CONTROL_PORT
#ifndef RESET_PORT
#define RESET_PORT          CONTROL_PORT
#endif
#ifndef FEED_HOLD_PORT
#define FEED_HOLD_PORT      CONTROL_PORT
#endif
#ifndef CYCLE_START_PORT
#define CYCLE_START_PORT    CONTROL_PORT
#endif
#ifdef SAFETY_DOOR_PIN
#if defined(ENABLE_SAFETY_DOOR_INPUT_PIN) && !defined(SAFETY_DOOR_PORT)
#define SAFETY_DOOR_PORT    CONTROL_PORT
#endif
#endif
#endif

#ifndef RESET_BIT
#define RESET_BIT (1<<RESET_PIN)
#endif

#ifndef FEED_HOLD_BIT
#define FEED_HOLD_BIT (1<<FEED_HOLD_PIN)
#endif

#ifndef CYCLE_START_BIT
#define CYCLE_START_BIT (1<<CYCLE_START_PIN)
#endif

#ifndef RESET_BIT
#define FEED_HOLD_BIT (1<<RESET_PIN)
#endif

#ifndef SAFETY_DOOR_BIT
#ifdef SAFETY_DOOR_PIN
#define SAFETY_DOOR_BIT (1<<SAFETY_DOOR_PIN)
#else
#define SAFETY_DOOR_BIT 0
#endif
#endif

#ifndef CONTROL_MASK
#define CONTROL_MASK    (RESET_BIT|FEED_HOLD_BIT|CYCLE_START_BIT|SAFETY_DOOR_BIT)
#endif

typedef struct {
    pin_function_t id;
    uint32_t port;
    uint8_t pin;
    uint16_t bit;
    pin_group_t group;
    volatile bool active;
    volatile bool debounce;
    pin_irq_mode_t irq_mode;
    pin_mode_t cap;
    ioport_interrupt_callback_ptr interrupt_callback;
    const char *description;
} input_signal_t;

typedef struct {
    pin_function_t id;
    uint32_t port;
    uint8_t pin;
    pin_group_t group;
    pin_mode_t mode;
    const char *description;
} output_signal_t;

typedef struct {
    uint8_t n_pins;
    union {
        input_signal_t *inputs;
        output_signal_t *outputs;
    } pins;
} pin_group_pins_t;

#ifdef HAS_IOPORTS
void ioports_init(pin_group_pins_t *aux_inputs, pin_group_pins_t *aux_outputs);
void ioports_event (input_signal_t *input);
#endif

#endif
