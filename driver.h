/*
  driver.h - An embedded CNC Controller with rs274/ngc (g-code) support

  Grbl driver code for Texas Instruments Tiva C (TM4C123GH6PM) ARM processor

  Part of grblHAL

  Copyright (c) 2016-2024 Terje Io
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

typedef struct {
    pin_function_t id;
    uint32_t port;
    uint8_t pin;
    uint16_t bit;
    pin_group_t group;
    volatile bool active;
    volatile bool debounce;
    pin_cap_t cap;
    pin_mode_t mode;
    ioport_interrupt_callback_ptr interrupt_callback;
    aux_ctrl_t *aux_ctrl;
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

void ioports_init(pin_group_pins_t *aux_inputs, pin_group_pins_t *aux_outputs);
void ioports_event (input_signal_t *input);

#endif // __DRIVER_H__
