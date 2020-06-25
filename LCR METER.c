// Hi There!!!!
// Kashish Shah
// EE 5314 PROJECT- LCR METER
// KASHISH HARESH SHAH
// UTA ID : 1001669323
// Unique Number Assigned: 40 Shah,
//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------
//
// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz


//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "tm4c123gh6pm.h"
#include "hw_nvic.h"
#include "hw_types.h"


#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))
#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))
#define PUSH_BUTTON  (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 4*4)))
#define MEAS_LR      (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 4*4)))
#define MEAS_C       (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 5*4)))
#define HIGHSIDE_R   (*((volatile uint32_t *)(0x42000000 + (0x400073FC-0x40000000)*32 + 3*4)))
#define LOWSIDE_R    (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 1*4)))
#define INTEGRATE    (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 2*4)))
#define Max_Char 80

uint8_t count, j, argument1, argument2, argument3, a, arg_len,arg, arg_no=0, pos[4], comm_len;
uint16_t  VDut1, VDut2, VCIN;
float time, VDUT, resistance, capacitance, inductance, Volt, VD1, VD2, Ri,RL,VDiff, alpha = 0.99;
int firstUpdate = true;
char ch, c, str[Max_Char+1], strv[10], statement [4][10], command[10], type[4], str2[5], string[10];
bool valid,comm = true, timeMode = false, timeUpdate = false;


//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------


// Blocking function that returns only when SW1 is pressed
void waitPbPress()
{
    while(PUSH_BUTTON);
}


void AnalogComp0ISR(void)
{
    time = WTIMER5_TAV_R;
    WTIMER5_TAV_R = 0;
    COMP_ACMIS_R |= 0x01;
}

// Initialize Hardware
void initHw()
{
    // Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    SYSCTL_GPIOHBCTL_R = 0;

    // Enable GPIO port A,D,E and F peripherals
    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOA | SYSCTL_RCGC2_GPIOF | SYSCTL_RCGC2_GPIOE | SYSCTL_RCGC2_GPIOD | SYSCTL_RCGC2_GPIOB | SYSCTL_RCGC2_GPIOC;

    // Configure LED and pushbutton pins
    GPIO_PORTF_DIR_R = 0x0A;  // bits 1 and 3 are outputs, other pins are inputs
    GPIO_PORTF_DR2R_R = 0x0A; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTF_DEN_R = 0x1A;  // enable LEDs and pushbuttons
    GPIO_PORTF_PUR_R = 0x10;  // enable internal pull-up for push button

    // Configure MEAS_LR, MEAS_C, LOWSIDE_R and INTEGRATE pins
    GPIO_PORTE_DIR_R = 0x36;  // bits 1,2 4 and 5 are outputs, other pins are inputs
    GPIO_PORTE_DR2R_R = 0x36; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTE_DEN_R = 0x36;  // Digital enable pins


    // Configure HIGHSIDE_R pin
    GPIO_PORTD_DIR_R = 0x08;  // bit 3 is output, other pins are inputs
    GPIO_PORTD_DR2R_R = 0x08; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTD_DEN_R = 0x08;  // Digital enable pins

    // Configure COMPARATOR pin
    GPIO_PORTC_DR2R_R = 0x80; // set drive strength to 2mA (not needed since default configuration -- for clarity)

    // Configure AN10 as an analog input for DUT1
    SYSCTL_RCGCADC_R |= 1;                           // turn on ADC module 0 clocking
    GPIO_PORTB_AFSEL_R |= 0x20;                      // select alternative functions for AN0 (PB5)
    GPIO_PORTB_DEN_R &= ~0x20;                       // turn off digital operation on pin PB5
    GPIO_PORTB_AMSEL_R |= 0x20;                      // turn on analog operation on pin PB5
    ADC0_CC_R = ADC_CC_CS_SYSPLL;                    // select PLL as the time base (not needed, since default value)
    ADC0_ACTSS_R &= ~ADC_ACTSS_ASEN3;                // disable sample sequencer 3 (SS3) for programming
    ADC0_EMUX_R = ADC_EMUX_EM3_PROCESSOR;            // select SS3 bit in ADCPSSI as trigger
    ADC0_SSMUX3_R = 0x0B;                               // set first sample to AIN11
    ADC0_SSCTL3_R = ADC_SSCTL3_END0;                 // mark first sample as the end
    ADC0_ACTSS_R |= ADC_ACTSS_ASEN3;                 // enable SS3 for operation

    // Configure AN11 as an analog input for DUT2
    SYSCTL_RCGCADC_R |= 2;                           // turn on ADC module 1 clocking
    GPIO_PORTB_AFSEL_R |= 0x10;                      // select alternative functions for AN1 (PB4)
    GPIO_PORTB_DEN_R &= ~0x10;                       // turn off digital operation on pin PB4
    GPIO_PORTB_AMSEL_R |= 0x10;                      // turn on analog operation on pin PB4
    ADC1_CC_R = ADC_CC_CS_SYSPLL;                    // select PLL as the time base (not needed, since default value)
    ADC1_ACTSS_R &= ~ADC_ACTSS_ASEN3;                // disable sample sequencer 3 (SS3) for programming
    ADC1_EMUX_R = ADC_EMUX_EM3_PROCESSOR;            // select SS3 bit in ADCPSSI as trigger
    ADC1_SSMUX3_R = 0x0A;                               // set first sample to AIN10
    ADC1_SSCTL3_R = ADC_SSCTL3_END0;                 // mark first sample as the end
    ADC1_ACTSS_R |= ADC_ACTSS_ASEN3;                 // enable SS3 for operation



    // Configure UART0 pins
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;         // turn-on UART0, leave other uarts in same status
    GPIO_PORTA_DEN_R |= 3;                           // default, added for clarity
    GPIO_PORTA_AFSEL_R |= 3;                         // default, added for clarity
    GPIO_PORTA_PCTL_R = GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;

    // Configure UART0 to 115200 baud, 8N1 format (must be 3 clocks from clock enable and config writes)
    UART0_CTL_R = 0;                                 // turn-off UART0 to allow safe programming
    UART0_CC_R = UART_CC_CS_SYSCLK;                  // use system clock (40 MHz)
    UART0_IBRD_R = 21;                               // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
    UART0_FBRD_R = 45;                               // round(fract(r)*64)=45
    UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN; // configure for 8N1 w/ 16-level FIFO
    UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN; // enable TX, RX, and module


    // Configure Wide Timer 5 as counter
    SYSCTL_RCGCWTIMER_R |= SYSCTL_RCGCWTIMER_R5;     // turn-on timer
    WTIMER5_CTL_R &= ~TIMER_CTL_TAEN;                // turn-off counter before reconfiguring
    WTIMER5_CFG_R = 4;                               // configure as 32-bit counter (A only)
    WTIMER5_TAMR_R = TIMER_TAMR_TACMR | TIMER_TAMR_TAMR_CAP | TIMER_TAMR_TACDIR; // configure for edge time mode, count up
    WTIMER5_CTL_R = TIMER_CTL_TAEVENT_POS;           // measure time from positive edge to positive edge
    WTIMER5_IMR_R = TIMER_IMR_CAEIM;                 // turn-on interrupts
    WTIMER5_TAV_R = 0;                               // zero counter for first period
    WTIMER5_CTL_R |= TIMER_CTL_TAEN;                 // turn-on counter

    // Configure Analog Comparator
    GPIO_PORTC_DIR_R &= ~0x80;                       // bit 7 is not an output, other pins are inputs
    GPIO_PORTC_AFSEL_R |= 0x80;                      // select alternative functions for AN1 (PC7)
    GPIO_PORTC_DEN_R &= ~0x80;                       // turn off digital operation on pin PC7
    GPIO_PORTC_AMSEL_R |= 0x80;                      // turn on analog operation on pin PC7
    SYSCTL_RCGCACMP_R |= 0x01;
    COMP_ACREFCTL_R |= COMP_ACREFCTL_VREF_M | COMP_ACREFCTL_EN;
    COMP_ACCTL0_R |= COMP_ACCTL0_ASRCP_REF | COMP_ACCTL0_ISEN_RISE;
    COMP_ACCTL0_R |= COMP_ACCTL0_CINV;
    COMP_ACRIS_R |= COMP_ACRIS_IN0;

    COMP_ACINTEN_R |= 0x01;
    NVIC_EN0_R &= ~(1 << INT_COMP0 - 16);

}

//Function that returns upto 12 Bit Value from ADC0
uint16_t readAdc0Ss3()
{
    ADC0_PSSI_R |= ADC_PSSI_SS3;                     // set start bit
    while (ADC0_ACTSS_R & ADC_ACTSS_BUSY);           // wait until SS3 is not busy
    return ADC0_SSFIFO3_R;                           // get single result from the FIFO
}

//Function that returns upto 12 Bit Value from ADC1
uint16_t readAdc1Ss3()
{
    ADC1_PSSI_R |= ADC_PSSI_SS3;                     // set start bit
    while (ADC1_ACTSS_R & ADC_ACTSS_BUSY);           // wait until SS3 is not busy
    return ADC1_SSFIFO3_R;                           // get single result from the FIFO
}


// Approximate busy waiting (in units of microseconds), given a 40 MHz system clock
void waitMicrosecond(uint32_t us)
{
    __asm("WMS_LOOP0:   MOV  R1, #3");          // 1
    __asm("WMS_LOOP1:   SUB  R1, #1");          // 6
    __asm("             CBZ  R1, WMS_DONE1");   // 5+1*3
    __asm("             NOP");                  // 5
    __asm("             NOP");                  // 5
    __asm("             B    WMS_LOOP1");       // 5*2 (speculative, so P=1)
    __asm("WMS_DONE1:   SUB  R0, #1");          // 1
    __asm("             CBZ  R0, WMS_DONE0");   // 1
    __asm("             NOP");                  // 1
    __asm("             B    WMS_LOOP0");       // 1*2 (speculative, so P=1)
    __asm("WMS_DONE0:");                        // ---
    // 40 clocks/us + error

}
// Blocking function that writes a serial character when the UART buffer is not full
void putcUart0(char c)
{
    while (UART0_FR_R & UART_FR_TXFF);
    UART0_DR_R = c;
}

// Blocking function that writes a string when the UART buffer is not full
void putsUart0(char* str)
{
    uint8_t i;
    for (i = 0; i < strlen(str); i++)
        putcUart0(str[i]);
}


// Blocking function that returns with serial data once the buffer is not empty
char getcUart0()
{
    while (UART0_FR_R & UART_FR_RXFE);
    return UART0_DR_R & 0xFF;
}

// Function that checks whether the character is an Alphabet
bool isalpha(char c)
{
    uint8_t ch = (uint8_t) c;
    return (ch<=90&&ch>=65)||(ch<=122&&ch>=97);
}

// Function that checks whether the character is a Number
bool isnumber(char c)
{
    uint8_t ch = (uint8_t) c;
    return (ch<=57&&ch>=48);
}

// Function that checks whether the character is neither a Number nor an Alphabet
bool isdelim(char c)
{
    return (!isalpha(c) && !isnumber(c));
}

// Function that converts Upper-case Alphabets to Lower-case Alphabets
char lower(char c)
{
    a = (uint8_t) c;
    return (char)a+32;
}

// Blocking function that reads a string
char getsUart0()
{
    count=0;
    while(count<Max_Char)
    {
        ch = getcUart0();
        putcUart0(ch);
        j= (uint8_t) ch;
        if(j==8)
        {
            if(count!=0)
                count--;
            continue;
        }
        else
        {
            if(j==13)
            {
                break;
            }
            else
            {
                if(j<32)
                    continue;
                else
                {
                    if(isalpha(ch) && j<96)
                        str[count++]=lower(ch);
                    else
                        str[count++]=ch;
                }
            }
        }
    }

    str[count] = '\0';
    return(0);
}

//Function to check whether the command entered is correct or incorrect
bool isCommand(char* verb, uint8_t arg)
{
    bool ret;
    if(strcmp(verb,"end") == 0)
        return true;
    if(arg==2)
    {
        if(strcmp(verb,"set") == 0)
            ret = true;
        else
        {
            putsUart0("\n\rWrong command\n\r");
            ret = false;
        }
    }
    else if(arg==0)
    {
        if(strcmp(verb,"voltage") == 0 || strcmp(statement[0],"reset")==0 || strcmp(statement[0],"resistor")==0 || strcmp(statement[0],"capacitor")==0 || strcmp(statement[0],"inductor")==0 || strcmp(statement[0],"esr")==0)
            ret = true;
        else
        {
            putsUart0("\n\rWrong command\n\r");
            ret = false;
        }
    }
    else
    {
        putsUart0("\n\rWrong number of arguments\n\r");
        ret = false;
    }
    return ret;
}

//Function that resets all values of 2D Array to Null
uint8_t resetCount()
{
    uint8_t i,j;
    for(i=0;i<4;i++)
        for(j=0;j<10;j++)
            statement[i][j]='\0';
    return 0;
}

//Function that Parses the Entered String to distinguish between Command and Arguments
uint8_t parse_string()
{
    putsUart0("\r\nEnter your input\r\n");
    getsUart0();

    resetCount();

    for(a=0;a<strlen(str);a++)
    {
        putcUart0(str[a]);
    }
    putsUart0("\r\n");
    a=0;
    comm = true;                    // command flag
    arg_no=0;                       // number of arguments
    comm_len = 0;                   // length of command(SET)
    arg_len = 0;                    //length of one argument
    while(a<strlen(str))
    {
        ch = str[a];
        if(isdelim(ch))
        {
            arg_len = 0;
            if(comm_len!=0)

                a++;
            continue;

        }
        if(a==0 & !isdelim(ch))
        {
            pos[arg_no] = a;
            if(isalpha(ch))

            {
                command[comm_len++] = ch;
                type[arg_no] = 'a';
            }
            else
                type[arg_no] = 'n';

            arg_no++;
            putcUart0(ch);
        }
        else if(!isdelim(ch))
        {
            if(comm)
            {
                command[comm_len] = ch;
                comm_len++;
            }

            if(isdelim(str[a-1]))
            {
                putsUart0("\r\n");
                pos[arg_no] = a;
                if(isalpha(ch))
                    type[arg_no] = 'a';
                else
                    type[arg_no] = 'n';
                arg_no++;
            }
            if(!comm)
            {
                statement[arg_no-1][arg_len++] = ch;
            }

            putcUart0(ch);
        }
        a++;
    }
    command[comm_len] = '\0';
    for(a=0;a<comm_len;a++)
        statement[0][a]=command[a];

    putsUart0("\r\n\r\nThe command is: ");
    putsUart0(command);

    if(isCommand(command,arg_no-1))
    {
        valid = true;
        putsUart0("\r\nIt is a valid command\r\n");
    }
    else
    {
        valid = false;
        putsUart0("\r\nIt is not a valid command\r\n");
    }

    putsUart0("\r\n\r\nNumber of Args =");
    sprintf(string,"%d",arg_no);
    putsUart0(string);

    for(a = 0;a<arg_no;a++)
    {
        putsUart0("\r\npos[");
        sprintf(string,"%d",a);
        putsUart0(string);
        putsUart0("] = ");
        sprintf(string,"%d",pos[a]);
        putsUart0(string);
        putsUart0("\r\ntype[");
        sprintf(string,"%d",a);
        putsUart0(string);
        putsUart0("] = ");
        putcUart0(type[a]);
    }
    return(0);
}

// Function to Toggle LEDs and GPIO Pins
uint8_t toggle()
{
    if(strcmp(statement[0],"set")==0)
    {
        if(strcmp(statement[1],"green")==0)
        {
            if(strcmp(statement[2],"on")==0)
                GREEN_LED = 1;
            else if(strcmp(statement[2],"off")==0)
                GREEN_LED = 0;
        }
        else if(strcmp(statement[1],"red")==0)
        {
            if(strcmp(statement[2],"on")==0)
                RED_LED = 1;
            else if(strcmp(statement[2],"off")==0)
                RED_LED = 0;
        }
        else if(strcmp(statement[1],"measlr")==0)
        {
            if(strcmp(statement[2],"on")==0)
                MEAS_LR = 1;
            else if(strcmp(statement[2],"off")==0)
                MEAS_LR = 0;
        }
        else if(strcmp(statement[1],"measc")==0)
        {
            if(strcmp(statement[2],"on")==0)
                MEAS_C = 1;
            else if(strcmp(statement[2],"off")==0)
                MEAS_C = 0;
        }
        else if(strcmp(statement[1],"highsider")==0)
        {
            if(strcmp(statement[2],"on")==0)
                HIGHSIDE_R = 1;
            else if(strcmp(statement[2],"off")==0)
                HIGHSIDE_R = 0;
        }
        else if(strcmp(statement[1],"lowsider")==0)
        {
            if(strcmp(statement[2],"on")==0)
                LOWSIDE_R = 1;
            else if(strcmp(statement[2],"off")==0)
                LOWSIDE_R = 0;
        }
        else if(strcmp(statement[1],"integrate")==0)
        {
            if(strcmp(statement[2],"on")==0)
                INTEGRATE = 1;
            else if(strcmp(statement[2],"off")==0)
                INTEGRATE = 0;
        }

    }
    else if(strcmp(statement[0],"voltage")==0)
    {

        VDut1 = readAdc0Ss3();
        VDut2 = readAdc1Ss3();


        sprintf(string,"%d",VDut1);
        putsUart0("\r\nVDUT1 = ");
        putsUart0(string);
        sprintf(string,"%d",VDut2);
        putsUart0("\r\nVDUT2 = ");
        putsUart0(string);

        if(VDut2>VDut1)
            VDiff = VDut2-VDut1;
        else if(VDut1>VDut2)
            VDiff = VDut1-VDut2;

        sprintf(string,"%lf",VDiff);
        putsUart0("\r\nVDiff = ");
        putsUart0(string);


        VDUT = (VDiff*3.3)/4095;
        sprintf(string,"%lf",VDUT);
        putsUart0("\r\nVDUT = ");
        putsUart0(string);

        if (firstUpdate)
        {
            Volt = VDUT;
            firstUpdate = false;
        }
        else
            Volt = Volt * alpha + VDUT * (1-alpha);
        sprintf(string,"%f",Volt);
        putsUart0("\r\nFiltered Voltage = ");
        putsUart0(string);
    }

    else if(strcmp(statement[0],"reset")==0)
    {
        HWREG(NVIC_APINT)= NVIC_APINT_VECTKEY | NVIC_APINT_SYSRESETREQ;
    }


    else if(strcmp(statement[0],"resistor")==0)
    {
        COMP_ACREFCTL_R &= ~(COMP_ACREFCTL_VREF_M | COMP_ACREFCTL_EN);
        __asm(" NOP");                                   // wait 3 clocks
        __asm(" NOP");
        __asm(" NOP");

        COMP_ACREFCTL_R |= 0x204;

        INTEGRATE= 1;
        LOWSIDE_R = 1;
        MEAS_LR = 0;


        waitMicrosecond(10000000);

        LOWSIDE_R = 0;
        MEAS_LR = 1;
        COMP_ACINTEN_R |= 0x01;
        WTIMER5_TAV_R = 0;


        NVIC_EN0_R |= (1 << INT_COMP0 - 16);
        waitMicrosecond(10000000);
        NVIC_EN0_R &= ~(1 << INT_COMP0 - 16);
        time /=40;
        VDut1 = readAdc0Ss3();

        putsUart0("\r\nTime =" );
        putsUart0("\r\n");
        sprintf(string,"%f",time);
        putsUart0(string);

        resistance =  (time)/0.4;

        putsUart0("\r\nResistance =" );
        putsUart0("\r\n");
        sprintf(string,"%lf",resistance);
        putsUart0(string);

        MEAS_LR = 0;
        INTEGRATE= 1;
        LOWSIDE_R = 1;

    }
    else if(strcmp(statement[0],"capacitor")==0)
    {
        COMP_ACREFCTL_R &= ~(COMP_ACREFCTL_VREF_M | COMP_ACREFCTL_EN);
        __asm(" NOP");                                   // wait 3 clocks
        __asm(" NOP");
        __asm(" NOP");

        COMP_ACREFCTL_R |= 0x207;

        INTEGRATE= 0;
        LOWSIDE_R = 1;
        MEAS_C = 1;
        HIGHSIDE_R = 0;
        MEAS_LR = 0;

        waitMicrosecond(10000000);
        WTIMER5_TAV_R = 0;
        COMP_ACINTEN_R |= 0x01;
        LOWSIDE_R = 0;
        MEAS_C = 1;
        HIGHSIDE_R = 1;


        NVIC_EN0_R |= (1 << INT_COMP0 - 16);
        waitMicrosecond(75000000);
        NVIC_EN0_R &= ~(1 << INT_COMP0 - 16);


        putsUart0("\r\nTime =" );

        putsUart0("\r\n");
        sprintf(string,"%f",time);
        putsUart0(string);
        time /= 40;


        capacitance = (time)/67878.47;
        putsUart0("\r\nCapacitance =" );
        putsUart0("\r\n");
        sprintf(string,"%lf",capacitance);
        putsUart0(string);
        INTEGRATE= 0;
        HIGHSIDE_R = 0;
        LOWSIDE_R = 1;
        MEAS_C = 1;
        MEAS_LR = 0;

    }


    else if(strcmp(statement[0],"inductor")==0)
    {

        INTEGRATE= 1;
        LOWSIDE_R = 1;
        MEAS_C = 0;
        HIGHSIDE_R = 0;
        MEAS_LR = 0;

        waitMicrosecond(15000000);

        INTEGRATE= 0;
        LOWSIDE_R = 1;
        MEAS_C = 0;
        HIGHSIDE_R = 0;
        MEAS_LR = 1;

        WTIMER5_TAV_R = 0;
        COMP_ACINTEN_R |= 0x01;

        NVIC_EN0_R |= (1 << INT_COMP0 - 16);
        waitMicrosecond(55000000);
        NVIC_EN0_R &= ~(1 << INT_COMP0 - 16);
        time /= 40;
        putsUart0("\r\nTime =" );
        putsUart0("\r\n");
        sprintf(string,"%f",time);
        putsUart0(string);

        inductance = (18.7 * time) + 13;
        putsUart0("\r\nInductor = ");
        putsUart0("\r\n");
        sprintf(string,"%lf",inductance);
        putsUart0(string);
    }
    else if(strcmp(statement[0],"esr")==0)
    {


        INTEGRATE= 0;
        LOWSIDE_R = 1;
        MEAS_C = 0;
        HIGHSIDE_R = 0;
        MEAS_LR = 1;

        waitMicrosecond(10000000);
        VDut1 = readAdc0Ss3();
        VDut2 = readAdc1Ss3();

        VD1 = (VDut1*3.3)/4095;
        VD2 = (VDut2*3.3)/4095;

        sprintf(string,"%f",VD1);
        putsUart0("\r\nVD1 = ");
        putsUart0(string);

        sprintf(string,"%f",VD2);
        putsUart0("\r\nVD2 = ");
        putsUart0(string);

        Ri = (33*(VD1-VD2))/VD2;
        sprintf(string,"%f",Ri);
        putsUart0("\r\nRi = ");
        putsUart0(string);

        RL = 1.14 *Ri;
        sprintf(string,"%f",RL);
        putsUart0("\r\nRL = ");
        putsUart0(string);

    }

    return 0;
}



//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

uint8_t main(void)
{
    // Initialize hardware

    initHw();
    putsUart0("\r\nLCR METER\r\n");

    MEAS_LR = 0;
    MEAS_C = 0;
    LOWSIDE_R = 0;
    HIGHSIDE_R = 0;
    INTEGRATE= 0;

    RED_LED = 1;
    waitMicrosecond(500000);
    RED_LED = 0;
    waitMicrosecond(500000);

    while(strcmp(statement[0],"end")!=0)
    {
        parse_string();
        if(valid)
            toggle();
    }
    return 0;
}
