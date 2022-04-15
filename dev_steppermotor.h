/*
 * dev_steppermotor.h
 *
 *  Created on: 9 Oct 2021
 *      Author: chime
 *
 *  Stepper motor type : 28BYJ-48 Uni-polar brushless Motor
 *  It has 4 gear sets
 *      1st gear (rotor shaft) 32/9 = 3.55 gear ratio
 *      2nd gear 22/11 =  2.00 GR
 *      3rd gear 26/9  =  2.88 GR
 *      4th gear 31/10 =  3.10 GR
 *      Total gear ratio (GR) = 3.55 * 2.0 * 2.88 * 3.1 = 63.388 ~ 64
 *
 *  This means that the rotor shaft completes 64 revolution for the
 *  output shaft to make one revolution
 *
 *
 * The sequence of control signals for 4 control wires are as follows:
 *
 * +--- Wire Colours ---+
 * B - Blue
 * P - Pink
 * Y - Yellow
 * O - Orange
 *
 *
 * +---- 4 Step Sequence (Full step Low Torque) ----+
 * GR = 64 (rotor shaft 64: output shaft 1)
 * Rotor makes 64 steps to make 1 output step
 * Output gear has 32 teeth (32 steps = 1 Revolution)
 * Full step sequence require 32 * 64 = 2048 steps
 * rotor shaft makes 2048 steps
 *
 * _____ ___ ___ ___ ___
 * Step | B | P | Y | O
 * -----+---+---+---+---
 *    1   1   0   0   0
 *    2   0   1   0   0
 *    3   0   0   1   0
 *    4   0   0   0   1
 *
 *
 *
 * +---- 8 Step Sequence (Half step Low Torque) ----+
 *
 * GR = 64 (rotor shaft 64: output shaft 1)
 * Rotor makes 64 steps to make 1 output step
 * Output gear has 32 teeth (32 steps = 1 Revolution)
 * Half step sequence require (32/0.5) * 64 = 4096 steps
 * rotor shaft makes 4096 steps
 * _____ ___ ___ ___ ___
 * Step | B | P | Y | O
 * -----+---+---+---+---
 *    1   1   0   0   0
 *    2   1   1   0   0
 *    3   0   1   0   0
 *    4   0   1   1   0
 *    5   0   0   1   0
 *    6   0   0   1   1
 *    7   0   0   0   1
 *    8   1   0   0   1
 *
 *
 *    The steppermotor has configured for fullstep mode 4 step sequence
 */

#ifndef DEV_STEPPERMOTOR_H_
#define DEV_STEPPERMOTOR_H_



#include <stdint.h>
#include "TM4C123GH6PM.h"
#include "php.h"
#include "misc.h"

//
// Define pins
#define STEPPERMOTOR_PIN_BLUE   (1U << 1) //(1U << 1)
#define STEPPERMOTOR_PIN_PINK   (1U << 2) //(1U << 2)
#define STEPPERMOTOR_PIN_YELLOW (1U << 4) //(1U << 4)
#define STEPPERMOTOR_PIN_ORANGE (1U << 5) //(1U << 5)
#define STEPPERMOTOR_PIN_ALL    (STEPPERMOTOR_PIN_BLUE | STEPPERMOTOR_PIN_PINK | STEPPERMOTOR_PIN_YELLOW | STEPPERMOTOR_PIN_ORANGE)
#define STEPPERMOTOR_FULL_STEP_MODE   (0)
#define STEPPERMOTOR_HALF_STEP_MODE   (1)
#define STEPPERMOTOR_DIR_CW           (1)
#define STEPPERMOTOR_DIR_CCW          (0)
#define STEPPERMOTOR_PORT GPIOE
#define STEPPERMOTOR_PORT_GPIO_PORT_CLK_EN   (1U<<4)   // PORT E


#define MOTOR_SEQ_NO   (8)


typedef struct STEPPERMOTOR_Mode{
    int rotor_steps;                     // full mode (2048), half mode (4096)
    char rotor_seq[MOTOR_SEQ_NO];        // rotor sequence for half or full step mode

}steppermotor_mode_t;

/*
static steppermotor_mode_t * STEPPERMOTOR_Mode;
static int STEPPERMOTOR_Dir = 1;     // start rotation in CW - clockwise direction
static int STEPPERMOTOR_Step_Counter = 1;
static int STEPPERMOTOR_SeqIdx=0;
//static int STEPPERMOTOR_Mode_Sel = 0;      // start rotation in full step mode, (1: half step mode )
*/


// 8 Step Sequence (Half step Low Torque)



void StepperMotor_Init(void);
void STEPPERMOTOR_Rotate_CW(void);
void STEPPERMOTOR_Rotate_CCW(void);
void STEPPERMOTOR_Rotate(void);



#endif /* DEV_STEPPERMOTOR_H_ */
