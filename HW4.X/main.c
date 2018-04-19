#include<xc.h>
#include<sys/attribs.h> 
#include <math.h>
// DEVCFG0
#pragma config DEBUG = OFF // no debugging
#pragma config JTAGEN = OFF // no jtag
#pragma config ICESEL = ICS_PGx1 // use PGED1 and PGEC1
#pragma config PWP = OFF // no write protect
#pragma config BWP = OFF // no boot write protect
#pragma config CP = OFF // no code protect

// DEVCFG1
#pragma config FNOSC = PRIPLL // use primary oscillator with pll
#pragma config FSOSCEN = OFF // turn off secondary oscillator
#pragma config IESO = OFF // no switching clocks
#pragma config POSCMOD = HS // high speed crystal mode
#pragma config OSCIOFNC = OFF // disable secondary osc
#pragma config FPBDIV = DIV_1 // divide sysclk freq by 1 for peripheral bus clock
#pragma config FCKSM = CSDCMD // do not enable clock switch
#pragma config WDTPS = PS1048576 // use slowest wdt
#pragma config WINDIS = OFF // wdt no window mode
#pragma config FWDTEN = OFF // wdt disabled
#pragma config FWDTWINSZ = WINSZ_25 // wdt window at 25%

// DEVCFG2 - get the sysclk clock to 48MHz from the 8MHz crystal
#pragma config FPLLIDIV = DIV_2 // divide input clock to be in range 4-5MHz
#pragma config FPLLMUL = MUL_24 // multiply clock after FPLLIDIV
#pragma config FPLLODIV = DIV_2 // divide clock after FPLLMUL to get 48MHz
#pragma config UPLLIDIV = DIV_2 // divider for the 8MHz input clock, then multiplied by 12 to get 48MHz for USB
#pragma config UPLLEN = ON // USB clock on

// DEVCFG3
#pragma config USERID = 0 // some 16bit userid, doesn't matter what
#pragma config PMDL1WAY = OFF // allow multiple reconfigurations
#pragma config IOL1WAY = OFF // allow multiple reconfigurations
#pragma config FUSBIDIO = ON // USB pins controlled by USB module
#pragma config FVBUSONIO = ON // USB BUSON controlled by USB module

#define CS LATBbits.LATB8


// B8 is CS//SS digital pin
// A1 is SDO pin 3
//SCK is B14 -- already set

void initSPI1(){
    RPA1Rbits.RPA1R = 0b0011; // a1 pin for SDO1
    TRISBbits.TRISB8 = 0; // output pin for cs
    SPI1CON = 0;              // turn off the spi module and reset it
    SPI1BUF;  
    SPI1BRG = 1; // frequency 1 is high 1000 is low
    SPI1STATbits.SPIROV = 0; 
    SPI1CONbits.CKE = 1;      // data changes when clock goes from hi to lo (since CKP is 0)
    SPI1CONbits.MSTEN = 1;    // master operation
    SPI1CONbits.ON = 1;
    
    CS = 1; // initialize CS to high, active low though
    
}
char SPI1_IO( char write) {
  SPI1BUF = write ;
  while(!SPI1STATbits.SPIRBF) { // wait to receive the byte
    ;
  }
  return SPI1BUF;
  
}
void setVoltage(char channel, int voltage){
    short temp = ((voltage & 1023) << 2) | (channel << 15) | (0b1 << 14) | (0b1 << 13)| (0b1 << 12);
    CS = 0;
    SPI1_IO((temp & 0xFF00) >> 8);
    SPI1_IO(temp & 0x00FF);
//    SPI1_IO((char)0b10101010);
    CS = 1;
}

int main() {

    __builtin_disable_interrupts();

    // set the CP0 CONFIG register to indicate that kseg0 is cacheable (0x3)
    __builtin_mtc0(_CP0_CONFIG, _CP0_CONFIG_SELECT, 0xa4210583);

    // 0 data RAM access wait states
    BMXCONbits.BMXWSDRM = 0x0;

    // enable multi vector interrupts
    INTCONbits.MVEC = 0x1;

    // disable JTAG to get pins back
    DDPCONbits.JTAGEN = 0;

    // do your TRIS and LAT commands here
    initSPI1();
    TRISAbits.TRISA4 = 0; // output pin
    TRISBbits.TRISB4 = 1; // input pin button
    LATAbits.LATA4 = 1; // sets A4 to high initially for testing
    int alternater = 1; // keeps track if on or off
    
    

    __builtin_enable_interrupts();

    _CP0_SET_COUNT(0);
    int i = 0;
    while(1) {
        _CP0_SET_COUNT(0);
        setVoltage(0, 512 + 512.0*sin(i*2.0*3.14/100));
 
        if (i%200 < 100) {
            setVoltage(1,(int)((float) (i%200) / 100 * 1023));
        }
        else {
            setVoltage(1, (int) ((float)(200- i%200) / 100 * 1023));         
        }
        i++;
        
        
        
        //setVoltage(1,512);
        // use _CP0_SET_COUNT(0) and _CP0_GET_COUNT() to test the PIC timing
        // remember the core timer runs at half the sysclk
        // sysclk is at 48MHz
//        while(PORTBbits.RB4 == 0){
//            //do nothing and pause 
//        }
        // core timer half of sysclk -- core timer is 24KHZ, want full cycle 2HZ, divide by 12000
        while(_CP0_GET_COUNT() < 24000){
        ; //do nada
        }
    }
     
}