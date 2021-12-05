/*
 * File:   main.c
 * Author: mateus
 * Microcontroller: PIC16F628A
 * IDE and Compiler: MPLABX and XC8
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
void BLUE_PWM_Handler(void);
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

    if(PIE1bits.TMR2IE == 1 && PIR1bits.TMR2IF){
        BLUE_PWM_Handler();
        PIR1bits.TMR2IF = 0;
    }
}

/********************************************************************************************************************************************/

void main(void) {
    
    
    MCU_configuration();                                                        // Configures microcontroller features
 

    while(1){

        RED_LED_PWM_value = RED_LED_PWM_value + 20;
        GREEN_LED_PWM_value = GREEN_LED_PWM_value + 20;
        BLUE_LED_PWM_value = BLUE_LED_PWM_value + 20;
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
    TIMER2_configuration();
    
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
    OPTION_REGbits.PS = 0x03;                                                   // Set Prescaler to 1:2

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
    @brief Configure Timer 1 with 2 prescaler
*/
void TIMER2_configuration(void){
    T2CONbits.TOUTPS = 0x00;                                                    // 1:2 postscaler
    T2CONbits.T2CKPS = 0x02;                                                    // No prescaler
    T2CONbits.TMR2ON = 1;                                                       // Enable Timer 2
 
    PIE1bits.TMR2IE = 1;                                                        // Enable Timer 2 interrupt
    PIR1bits.TMR2IF = 0;                                                        // Reset Interrupt Flag
}


/*------------------------------------------------------------------------------------------------------------------------------------------*/
/*
    @brief RED PWM routine
*/
void RED_PWM_Handler(void){
    if(RED_PWM_ON){                                                             // Check if it's duty cycle HIGH
        TMR0 = 255 - RED_LED_PWM_value;                                         // Set PWM on time

        RED_LED_STRIP_PORT = 1;                                                 // Turn on LED
        RED_PWM_ON = 0;                                                         // Flag next Overflow a duty cycle LOW
    }else{
        TMR0 = RED_LED_PWM_value;                                               // Set PWM off time

        RED_LED_STRIP_PORT = 0;                                                 // Turn off LED
        RED_PWM_ON = 1;                                                         // Flag next overrflow a duty cycle HIGH
    }
}

/*------------------------------------------------------------------------------------------------------------------------------------------*/
/*
    @brief GREEN PWM routine
*/
void GREEN_PWM_Handler(void){
    if(GREEN_PWM_ON){                                                           // Check if it's duty cycle HIGH
        TMR1H = 0xFF;                                                           // Turn TMR1 in 8 bit timer
        TMR1L = 255 - GREEN_LED_PWM_value;                                      // Set PWM on time

        GREEN_LED_STRIP_PORT = 1;                                               // Turn on LED
        GREEN_PWM_ON = 0;                                                       // Flag next Overflow a duty cycle LOW
    }else{
        TMR1H = 0xFF;                                                           // Turn TMR1 in 8 bit timer
        TMR1L = GREEN_LED_PWM_value;                                            // Turn off LED

        GREEN_LED_STRIP_PORT = 0;                                               // Flag next overrflow a duty cycle HIGH
        GREEN_PWM_ON = 1;
    }
}

/*------------------------------------------------------------------------------------------------------------------------------------------*/
/*
    @brief BLUE PWM routine
*/
void BLUE_PWM_Handler(void){
    if(BLUE_PWM_ON){                                                             // Check if it's duty cycle HIGH
        TMR2 = 255 - BLUE_LED_PWM_value;                                         // Set PWM on time

        BLUE_LED_STRIP_PORT = 1;                                                 // Turn on LED
        BLUE_PWM_ON = 0;                                                         // Flag next Overflow a duty cycle LOW
    }else{
        TMR2 = BLUE_LED_PWM_value;                                               // Set PWM off time

        BLUE_LED_STRIP_PORT = 0;                                                 // Turn off LED
        BLUE_PWM_ON = 1;                                                         // Flag next overrflow a duty cycle HIGH
    }
}