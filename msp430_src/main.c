#include "main.h"

uint8_t wireless_buf[33];
uint8_t seesaw_buf[24];
bool i2c_succeed;

volatile uint8_t pulses;

int main(void) {
    WDTCTL = WDTPW | WDTHOLD;   // Stop watchdog timer

    // Disable the GPIO power-on default high-impedance mode to activate
    // previously configured port settings
    PM5CTL0 &= ~LOCKLPM5;

    initClockTo1MHz(); // set SMCLK and CLK up

    uint8_t iteration = 1; // loop variables for servo motors

    while(1) {
        // power pins for sensors
        P1DIR |= BIT0;
        P1OUT &= ~BIT0;

        P5DIR |= BIT3;
        P5DIR |= BIT2;
        P4DIR |= BIT5;
        P3DIR |= BIT4 | BIT6;

        P5OUT |= BIT3;
        P3OUT |= BIT4 | BIT6;
        P5OUT |= BIT2;
        P4OUT |= BIT5;

        // PWM for servo motor
        P3DIR |= BIT1;
        P3OUT &= ~BIT1;

        // turn on all sensors
        P5OUT &= ~BIT2;
        P3OUT &= ~BIT6;
        P4OUT &= ~BIT5;

        __delay_cycles(1000);

        // start up the SPI and I2C bus
        initGPIO_SPI();
        initSPI();
        initGPIO_I2C();
        initI2C();

        __delay_cycles(10000);


        // take moisture reading
        P1OUT |= BIT0;
        i2c_succeed = false;
        read_temp_seesaw();
        P1OUT &= ~BIT0;
        i2c_succeed = false;
        read_cap_seesaw();

        // take pressure sensor reading
        i2c_succeed = false;
        while (!i2c_succeed)
            init_pressure();
        i2c_succeed = false;
        while (!i2c_succeed)
            read_pressure();

        // take co2 reading
        i2c_succeed = false;
        while (!i2c_succeed)
            init_co2();
        i2c_succeed = false;
        while (!i2c_succeed)
            read_data_co2();

        // turn off sensors
        P5OUT |= BIT2;
        P3OUT |= BIT6;
        P4OUT |= BIT5;

        // send wireless data
        P3OUT &= ~BIT4;
        __delay_cycles(1000);
        while(!init_wireless());
        // must send in two bursts, we have too much data to send all at once
        wireless_send(wireless_buf, 33); // send pressure and CO2 data
        __delay_cycles(100000); // give it some time to send
        wireless_send(seesaw_buf, 24); // send moisture data
        P3OUT |= BIT4;

        pulses = 0;
        if (iteration == 6) // open for 1/2 hour, closed for 1/2 hour
            iteration = 0;

        if (iteration == 0 || iteration == 3) { // only activate the servo if we need to, the disk will give if we turn it on even for a tick
            P5OUT &= ~BIT3;
            if (iteration == 0) {
                init_timer_B_open();
            } else { // iteration must be equal to close value
                init_timer_B_close();
            }
            __bis_SR_register(LPM3_bits);
            P5OUT |= BIT3;
        }
        iteration++;

        // go to sleep for 10 minutes, turn off all sensors
        sleep_10_min();
    }
}

// RTC interrupt service routine
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=RTC_VECTOR
__interrupt void RTC_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(RTC_VECTOR))) RTC_ISR (void)
#else
#error Compiler not supported!
#endif
{
    switch(__even_in_range(RTCIV,RTCIV_RTCIF))
    {
        case  RTCIV_NONE:   break;          // No interrupt
        case  RTCIV_RTCIF:                  // RTC Overflow
            RTCCTL |= RTCSR; // turn off the timer
            __bic_SR_register_on_exit(LPM4_bits);
            break;
        default: break;
    }
}

#pragma vector=TIMER0_B0_VECTOR
__interrupt void ISR_TB0_CCR0(void)
{
    if (pulses == 75) {
        TB0CTL = TBCLR; // reset the peripheral
        __bic_SR_register_on_exit(LPM3_bits);
    }
    P3OUT |= BIT1;
    TB0CCTL0 &= ~CCIFG;
    pulses++;
}

#pragma vector=TIMER0_B1_VECTOR
__interrupt void ISR_TB0_CCR1(void)
{
    P3OUT &= ~BIT1;
    TB0CCTL1 &= ~CCIFG;
}
