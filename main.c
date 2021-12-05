/*
 * File:   main.c
 * Author: mateus
 *
 * Created on November 28, 2021, 6:42 PM
 */


#include "xc8.h"

/********************************************************************************************************************************************/

#define RED_LED_STRIP_TRIS                      TRISAbits.TRISA0
#define RED_LED_TRIS                            TRISAbits.TRISA4
#define RED_BUTTON_TRIS                         TRISBbits.TRISB3

#define RED_LED_STRIP_PORT                      PORTAbits.RA0
#define RED_LED_PORT                            PORTAbits.RA4
#define RED_BUTTON_PORT                         PORTBbits.RB3

#define GREEN_LED_STRIP_TRIS                    TRISAbits.TRISA1
#define GREEN_LED_TRIS                          TRISAbits.TRISA6
#define GREEN_BUTTON_TRIS                       TRISBbits.TRISB4

#define GREEN_LED_STRIP_PORT                    PORTAbits.RA1
#define GREEN_LED_PORT                          PORTAbits.RA6
#define GREEN_BUTTON_PORT                       PORTBbits.RB4

#define BLUE_LED_STRIP_TRIS                     TRISAbits.TRISA2
#define BLUE_LED_TRIS                           TRISAbits.TRISA7
#define BLUE_BUTTON_TRIS                        TRISBbits.TRISB5

#define BLUE_LED_STRIP_PORT                     PORTAbits.RA2
#define BLUE_LED_PORT                           PORTAbits.RA7
#define BLUE_BUTTON_PORT                        PORTBbits.RB5

/*------------------------------------------------------------------------------------------------------------------------------------------*/

#define TMR0_TOP_VALUE                          250

/********************************************************************************************************************************************/

uint8_t RED_LED_PWM_value = 0;
uint8_t RED_PWM_ON;

uint8_t GREEN_LED_PWM_value = 0;
uint8_t GREEN_PWM_ON;

uint8_t BLUE_LED_PWM_value = 0;
uint8_t BLUE_PWM_ON;

/********************************************************************************************************************************************/

void MCU_configuration(void);                                                   // Microcontroller configuration
void GPIO_configuration(void);                                                  // GPIO configuration
void TIMER0_configuration(void);                                                // 
void TIMER1_configuration(void);                                                // 
void TIMER2_configuration(void);                                                // 


void RED_PWM_Handler(void);
void GREEN_PWM_Handler(void);

/********************************************************************************************************************************************/

void __interrupt () my_isr_routine(void) {
    if(INTCONbits.T0IE && INTCONbits.T0IF){
        RED_PWM_Handler();
        INTCONbits.T0IF = 0;
    }

    if(PIE1bits.TMR1IE == 1 && PIR1bits.TMR1IF){
        GREEN_PWM_Handler();
        PIR1bits.TMR1IF = 0;
    }
}

/********************************************************************************************************************************************/

void main(void) {
    
    
    MCU_configuration();                                                        // Configures microcontroller features
 

    while(1){

        RED_LED_PWM_value = RED_LED_PWM_value + 20;
        GREEN_LED_PWM_value = GREEN_LED_PWM_value + 20;
        __delay_ms(500);
        
    }

    return;
}

/********************************************************************************************************************************************/

/*------------------------------------------------------------------------------------------------------------------------------------------*/
/*
    @brief Microcontroller settings
*/
void MCU_configuration(void){
    GPIO_configuration();
    TIMER0_configuration();
    TIMER1_configuration();
    INTCONbits.PEIE = 1;                                                        // Enable Peripheral interrupts
    INTCONbits.GIE = 1;                                                         // Enable Global Interrupts
}

/*------------------------------------------------------------------------------------------------------------------------------------------*/
/*
    @brief Configure GPIO as outpur/input
*/
void GPIO_configuration(void){
    
    CMCON = 0x07;                                                               // Disabling comparators
    PORTA = 0;
    RED_LED_PORT = 1;
    
    RED_LED_STRIP_TRIS = 0;                                                     // RED LED STRIP is a output
    RED_LED_TRIS = 0;                                                           // RED LED is a output
    RED_BUTTON_TRIS = 1;                                                        // RED BUTTON is a input
    
    GREEN_LED_STRIP_TRIS = 0;                                                   // GREEN LED STRIP is a output
    GREEN_LED_TRIS = 0;                                                         // GREEN LED is a output
    GREEN_BUTTON_TRIS = 1;                                                      // GREEN BUTTON is a input

    BLUE_LED_STRIP_TRIS = 0;                                                    // BLUE LED STRIP is a output
    BLUE_LED_TRIS = 0;                                                          // BLUE LED is a output
    BLUE_BUTTON_TRIS = 1;                                                       // BLUE BUTTON is a input
    
    
}

/*------------------------------------------------------------------------------------------------------------------------------------------*/
/*
    @brief Configure Timer 0 with 2 prescaler
*/
void TIMER0_configuration(void){
    OPTION_REGbits.T0CS = 0;                                                    // Select TMR0 clock source to internal instruction cycle
    OPTION_REGbits.PSA = 0;                                                     // Assign Prescaler to Timer 0
    OPTION_REGbits.PS = 0x02;                                                   // Set Prescaler to 1:2

    INTCONbits.T0IE = 1;                                                        // Enable Timer 0 interrupt
    // prescaler 16 top value 250

}

/*------------------------------------------------------------------------------------------------------------------------------------------*/
/*
    @brief Configure Timer 1 with 2 prescaler
*/
void TIMER1_configuration(void){
    T1CONbits.TMR1CS = 0;                                                       // Clock Source is Fosc/4
    T1CONbits.T1CKPS = 0x03;                                                    // Set Prescaler to 1:2

    T1CONbits.TMR1ON = 1;                                                       // Enable Timer 1
    PIE1bits.TMR1IE = 1;                                                        // Enable Timer 1 interrupt
    PIR1bits.TMR1IF = 0;                                                        // Reset Interrupt Flag

}

/*------------------------------------------------------------------------------------------------------------------------------------------*/
/*
    @brief RED PWM routine
*/
void RED_PWM_Handler(void){
    if(RED_PWM_ON){
        TMR0 = 255 - RED_LED_PWM_value;

        RED_LED_STRIP_PORT = 1;
        RED_PWM_ON = 0;
    }else{
        TMR0 = RED_LED_PWM_value;   

        RED_LED_STRIP_PORT = 0;
        RED_PWM_ON = 1;
    }
}

/*------------------------------------------------------------------------------------------------------------------------------------------*/
/*
    @brief GREEN PWM routine
*/
void GREEN_PWM_Handler(void){
    if(GREEN_PWM_ON){

        TMR1 = 65280 + (255 - GREEN_LED_PWM_value);

        GREEN_LED_STRIP_PORT = 1;
        GREEN_PWM_ON = 0;
    }else{
        TMR1 = 65280 + GREEN_LED_PWM_value;   

        GREEN_LED_STRIP_PORT = 0;
        GREEN_PWM_ON = 1;
    }
}