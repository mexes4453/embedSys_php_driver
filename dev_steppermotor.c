/*
 * dev_steppermotor.c
 *
 *  Created on: 9 Oct 2021
 *      Author: chime
 */


#include "dev_steppermotor.h"

//static steppermotor_mode_t * STEPPERMOTOR_Mode;
int STEPPERMOTOR_Dir = 1;     // start rotation in CW - clockwise direction
int STEPPERMOTOR_Step_Counter = 1;
int STEPPERMOTOR_SeqIdx=0;
//static int STEPPERMOTOR_Mode_Sel = 0;      // start rotation in full step mode, (1: half step mode )


// 4 Step Sequence (Full step Low Torque)
// for data structure consistency the 4 sequence is repeated to the the buffer size consistent (8)
static char STEPPERMOTOR_FullModeSeq[MOTOR_SEQ_NO] = { (char)STEPPERMOTOR_PIN_BLUE,
                                                       (char)STEPPERMOTOR_PIN_PINK,
                                                       (char)STEPPERMOTOR_PIN_YELLOW,
                                                       (char)STEPPERMOTOR_PIN_ORANGE,
                                                       (char)STEPPERMOTOR_PIN_BLUE,
                                                       (char)STEPPERMOTOR_PIN_PINK,
                                                       (char)STEPPERMOTOR_PIN_YELLOW,
                                                       (char)STEPPERMOTOR_PIN_ORANGE
                                                     };


/*
static char STEPPERMOTOR_HalfModeSeq[MOTOR_SEQ_NO] = {  (STEPPERMOTOR_PIN_BLUE),
                                                        ((STEPPERMOTOR_PIN_BLUE)   | (STEPPERMOTOR_PIN_PINK)),
                                                        (STEPPERMOTOR_PIN_PINK),
                                                        ((STEPPERMOTOR_PIN_PINK)   | (STEPPERMOTOR_PIN_YELLOW)),
                                                        (STEPPERMOTOR_PIN_YELLOW),
                                                        ((STEPPERMOTOR_PIN_YELLOW) | (STEPPERMOTOR_PIN_ORANGE)),
                                                        (STEPPERMOTOR_PIN_ORANGE),
                                                        ((STEPPERMOTOR_PIN_ORANGE) | (STEPPERMOTOR_PIN_BLUE))
                                                     };

*/

void StepperMotor_Init(void){
    SYSCTL->RCGC2  |= STEPPERMOTOR_PORT_GPIO_PORT_CLK_EN;   // 01. Pg.340 - enable Run mode for GPIO */
    STEPPERMOTOR_PORT->DIR   |= STEPPERMOTOR_PIN_ALL;               // 02. Configure all pin on port B as output
    STEPPERMOTOR_PORT->DEN   |= STEPPERMOTOR_PIN_ALL;               // 03. Configure all pin on port B as digital
    STEPPERMOTOR_PORT->AFSEL &= ~STEPPERMOTOR_PIN_ALL;
    STEPPERMOTOR_PORT->AMSEL &= ~STEPPERMOTOR_PIN_ALL;

    // Initialise the steppermotor rotation sequence mode
/*
    switch (mode_sel){

        case STEPPERMOTOR_HALF_STEP_MODE:
            mode->rotor_steps = 4096;
            for (int i=0; i<MOTOR_SEQ_NO; i++){
                mode->rotor_seq[i] = STEPPERMOTOR_HalfModeSeq[i];
            }
            //mode->rotor_seq   = STEPPERMOTOR_HalfModeSeq;
            break;

        case STEPPERMOTOR_FULL_STEP_MODE:
            mode->rotor_steps = 2048;
            for (int i=0; i<MOTOR_SEQ_NO; i++){
                (mode->rotor_seq)[i] = STEPPERMOTOR_FullModeSeq[i];
            }
            //mode->rotor_seq  = STEPPERMOTOR_FullModeSeq;
            break;


    }
*/

}


void STEPPERMOTOR_Rotate_CW(void){
    STEPPERMOTOR_SeqIdx = (STEPPERMOTOR_Step_Counter++)&(MOTOR_SEQ_NO-1);
    //STEPPERMOTOR_PORT->DATA = STEPPERMOTOR_Mode->rotor_seq[STEPPERMOTOR_SeqIdx];
    STEPPERMOTOR_PORT->DATA = STEPPERMOTOR_FullModeSeq[STEPPERMOTOR_SeqIdx];
    STEPPERMOTOR_Dir = STEPPERMOTOR_DIR_CW;
    PHP_DelayMs(5);
}


void STEPPERMOTOR_Rotate_CCW(void){
    STEPPERMOTOR_SeqIdx = (STEPPERMOTOR_Step_Counter--)&(MOTOR_SEQ_NO-1);
    //STEPPERMOTOR_PORT->DATA = STEPPERMOTOR_Mode->rotor_seq[STEPPERMOTOR_SeqIdx];
    STEPPERMOTOR_PORT->DATA = STEPPERMOTOR_FullModeSeq[STEPPERMOTOR_SeqIdx];
    STEPPERMOTOR_Dir = STEPPERMOTOR_DIR_CCW;
    PHP_DelayMs(5);
}


void STEPPERMOTOR_Rotate(void){

    if (STEPPERMOTOR_Dir){

        STEPPERMOTOR_Rotate_CW();
    }else{
        STEPPERMOTOR_Rotate_CCW();

    }
}


