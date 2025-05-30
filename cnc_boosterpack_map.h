/*
  cnc_boosterpack_map.h - driver code for Texas Instruments Tiva C (TM4C123GH6PM) ARM processor

  - on Texas Instruments MSP432P401R LaunchPad

  Part of grblHAL

  Copyright (c) 2020-2024 Terje Io

  grblHAL is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  grblHAL is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with grblHAL. If not, see <http://www.gnu.org/licenses/>.
*/

#define BOARD_NAME "CNC BoosterPack"

#if TRINAMIC_ENABLE
#ifdef TRINAMIC_MIXED_DRIVERS
#undef TRINAMIC_MIXED_DRIVERS
#endif
#define TRINAMIC_MIXED_DRIVERS 0
#ifdef TRINAMIC_I2C
#undef TRINAMIC_I2C
#endif
#define TRINAMIC_I2C 1
#endif

#ifdef CNC_BOOSTERPACK
#undef CNC_BOOSTERPACK
#endif
#define CNC_BOOSTERPACK 1

// Define step pulse output pins.
#define STEP_PORT               GPIO_PORTD_BASE
#define X_STEP_PIN              1
#define Y_STEP_PIN              2
#define Z_STEP_PIN              3
#define STEP_OUTMODE            GPIO_SHIFT1

// Define step direction output pins. NOTE: All direction pins must be on the same port.
#define DIRECTION_PORT          GPIO_PORTB_BASE
#define X_DIRECTION_PIN         7
#define Y_DIRECTION_PIN         6
#define Z_DIRECTION_PIN         4
#define DIRECTION_OUTMODE       GPIO_MAP

#if TRINAMIC_ENABLE
#define TRINAMIC_DIAG_IRQ_PORT  GPIO_PORTE_BASE
#define TRINAMIC_DIAG_IRQ_PIN   1
#define TRINAMIC_WARN_IRQ_PORT  GPIO_PORTF_BASE
#define TRINAMIC_WARN_IRQ_PIN   0
// Define stepper driver enable/disable output pin(s).
#else
#define XY_ENABLE_PORT          GPIO_PORTE_BASE
#define XY_ENABLE_PIN           1
#define XY_ENABLE_BIT           (1<<XY_ENABLE_PIN)
#define Z_ENABLE_PORT           GPIO_PORTF_BASE
#define Z_ENABLE_PIN            0
#define Z_ENABLE_BIT            (1<<Z_ENABLE_PIN)
#define STEPPERS_ENABLE_MASK    (XY_ENABLE_BIT|Z_ENABLE_BIT)
/*
#define STEPPERS_ENABLE_PORT    GPIO_PORTE_BASE
#define STEPPERS_ENABLE_PIN     1
#define STEPPERS_ENABLE_BIT     (1<<STEPPERS_ENABLE_BIT)
*/
#endif

#if CNC_BOOSTERPACK_A4998
// Stepper driver VDD supply
#define STEPPERS_VDD_PORT       GPIO_PORTE_BASE
#define STEPPERS_VDD_PIN        5
#endif

// Define homing/hard limit switch input pins and limit interrupt vectors.

#if CNC_BOOSTERPACK_SHORTS
#define LIMIT_PORT              GPIO_PORTF_BASE
#define X_LIMIT_PIN             1
#define Y_LIMIT_PIN             3
#define Z_LIMIT_PIN             2
#else
#define LIMIT_PORT              GPIO_PORTA_BASE
#define X_LIMIT_PIN             1
#define Y_LIMIT_PIN             3
#define Z_LIMIT_PIN             2
#endif

/*
 * CNC Boosterpack GPIO assignments
 */

#define AUXIO0_PORT             GPIO_PORTF_BASE
#define AUXIO0_PIN              4
#define AUXIO1_PORT             GPIO_PORTA_BASE
#define AUXIO1_PIN              4
#define AUXIO2_PORT             GPIO_PORTA_BASE
#define AUXIO2_PIN              3
#define AUXIO3_PORT             GPIO_PORTA_BASE
#define AUXIO3_PIN              2
#define AUXIO4_PORT             GPIO_PORTB_BASE
#define AUXIO4_PIN              0
#define AUXIO5_PORT             GPIO_PORTB_BASE
#define AUXIO5_PIN              1
#define AUXIO6_PORT             GPIO_PORTE_BASE
#define AUXIO6_PIN              4

// Output definitions (comment out the port definition if used as input)

#define AUXOUTPUT0_PORT         AUXIO0_PORT
#define AUXOUTPUT0_PIN          AUXIO0_PIN
#define AUXOUTPUT1_PORT         AUXIO1_PORT
#define AUXOUTPUT1_PIN          AUXIO1_PIN
#define AUXOUTPUT2_PORT         AUXIO3_PORT
#define AUXOUTPUT2_PIN          AUXIO3_PIN
#define AUXOUTPUT3_PORT         GPIO_PORTB_BASE // Spindle PWM
#define AUXOUTPUT3_PIN          2
#define AUXOUTPUT4_PORT         GPIO_PORTE_BASE // Spindle direction
#define AUXOUTPUT4_PIN          3
#define AUXOUTPUT5_PORT         GPIO_PORTE_BASE // Spindle enable
#define AUXOUTPUT5_PIN          2
#define AUXOUTPUT6_PORT         GPIO_PORTD_BASE // Coolant flood
#define AUXOUTPUT6_PIN          6
#define AUXOUTPUT7_PORT         GPIO_PORTD_BASE // Coolant mist
#define AUXOUTPUT7_PIN          7

// Define driver spindle pins
#if DRIVER_SPINDLE_ENABLE & SPINDLE_ENA
#define SPINDLE_ENABLE_PORT     AUXOUTPUT4_PORT
#define SPINDLE_ENABLE_PIN      AUXOUTPUT4_PIN
#endif
#if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM
#define SPINDLE_PWM_PORT        AUXOUTPUT3_PORT
#define SPINDLE_PWM_PIN         AUXOUTPUT3_PIN
#define SPINDLE_PWM_MAP         GPIO_PB2_T3CCP0
#endif
#if DRIVER_SPINDLE_ENABLE & SPINDLE_DIR
#define SPINDLE_DIRECTION_PORT  AUXOUTPUT3_PORT
#define SPINDLE_DIRECTION_PIN   AUXOUTPUT3_PIN
#endif

// Define flood and mist coolant enable output pins.
#if COOLANT_ENABLE & COOLANT_FLOOD
#define COOLANT_FLOOD_PORT      AUXOUTPUT5_PORT
#define COOLANT_FLOOD_PIN       AUXOUTPUT5_PIN
#endif
#if COOLANT_ENABLE & COOLANT_MIST
#define COOLANT_MIST_PORT       AUXOUTPUT6_PORT
#define COOLANT_MIST_PIN        AUXOUTPUT6_PIN
#endif

// Input definitions

#ifndef AUXOUTPUT0_PORT
#define AUXINPUT0_PORT          AUXIO0_PORT
#define AUXINPUT0_PIN           AUXIO0_PIN
#endif

#ifndef AUXOUTPUT1_PORT
#define AUXINPUT1_PORT          AUXIO1_PORT
#define AUXINPUT1_PIN           AUXIO1_PIN
#endif

#define AUXINPUT2_PORT          AUXIO2_PORT
#define AUXINPUT2_PIN           AUXIO2_PIN

#ifndef AUXOUTPUT2_PORT
#define AUXINPUT3_PORT          AUXIO3_PORT
#define AUXINPUT3_PIN           AUXIO3_PIN
#endif

#ifndef SERIAL2_MOD
#define AUXINPUT4_PORT          AUXIO4_PORT
#define AUXINPUT4_PIN           AUXIO4_PIN
//#define AUXINPUT5_PORT          AUXIO5_PORT
//#define AUXINPUT5_PIN           AUXIO5_PIN
#endif

#define AUXINPUT5_PORT          GPIO_PORTA_BASE
#define AUXINPUT5_PIN           5

#define AUXINPUT6_PORT          AUXIO6_PORT
#define AUXINPUT6_PIN           AUXIO6_PIN
#define AUXINPUT7_PORT          GPIO_PORTC_BASE
#define AUXINPUT7_PIN           4
#define AUXINPUT8_PORT          GPIO_PORTC_BASE
#define AUXINPUT8_PIN           7

#if CNC_BOOSTERPACK_SHORTS
#define AUXINPUT9_PORT          GPIO_PORTC_BASE
#define AUXINPUT9_PIN           5
#define AUXINPUT10_PORT         GPIO_PORTC_BASE
#define AUXINPUT10_PIN          6
#else
#define AUXINPUT9_PORT          GPIO_PORTC_BASE
#define AUXINPUT9_PIN           6
#define AUXINPUT10_PORT         GPIO_PORTC_BASE
#define AUXINPUT10_PIN          5
#endif

// Define user-control controls (cycle start, reset, feed hold) input pins.
#if CONTROL_ENABLE & CONTROL_HALT
#define RESET_PORT              AUXINPUT8_PORT
#define RESET_PIN               AUXINPUT8_PIN
#endif
#if CONTROL_ENABLE & CONTROL_FEED_HOLD
#define FEED_HOLD_PORT          AUXINPUT9_PORT
#define FEED_HOLD_PIN           AUXINPUT9_PIN
#endif
#if CONTROL_ENABLE & CONTROL_CYCLE_START
#define CYCLE_START_PORT        AUXINPUT10_PORT
#define CYCLE_START_PIN         AUXINPUT10_PIN
#endif

// Define probe switch input pin.
#if PROBE_ENABLE
#define PROBE_PORT              AUXINPUT5_PORT
#define PROBE_PIN               AUXINPUT5_PIN
#endif

#if MPG_MODE == 1
#define MPG_MODE_PORT           AUXINPUT2_PORT
#define MPG_MODE_PIN            AUXINPUT2_PIN
#endif

#if I2C_STROBE_ENABLE
#define I2C_STROBE_PORT         AUXINPUT6_PORT
#define I2C_STROBE_PIN          AUXINPUT6_PIN
#endif

#if SAFETY_DOOR_ENABLE
#define SAFETY_DOOR_PORT        AUXINPUT7_PORT
#define SAFETY_DOOR_PIN         AUXINPUT7_PIN
#endif

#if MOTOR_FAULT_ENABLE && defined(AUXINPUT2_PORT)
#define MOTOR_FAULT_PORT        AUXINPUT2_PORT
#define MOTOR_FAULT_PIN         AUXINPUT2_PIN
#endif

/*EOF*/
