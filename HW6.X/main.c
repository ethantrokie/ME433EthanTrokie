#include<xc.h>
#include<sys/attribs.h> 
#include<stdio.h>
#include "ST7735.h"

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


void drawchar(short x, short y, char mess, short c1, short c2){
    char row = mess - 0x20;
    int col;
    for(col = 0; col < 5; col++){
        char pixels = ASCII[row][col];
        int j;
        for(j = 0; j < 8; j++){
        if((pixels >> j & 1) == 1 ){
            LCD_drawPixel(x+col,y+j,c1);
        }else{
            LCD_drawPixel(x+col,y+j,c2);
        }
       }
    }  
}
void drawString(short x, short y, char* mess, short c1, short c2){
    int i = 0;
    while(mess[i] && i < 26){
        drawchar(x+i*5,y,mess[i],c1,c2);
        i++;
    }
}

void drawLine(short x, short y, short height,short color){
    int i;
    for(i = 0; i < height; i++){
        LCD_drawPixel(x,y+i,color);
    }
}
void drawBox(short x, short y, short width,short color1){
    int i;
    for(i = 0; i < width; i++){
        drawLine(x+i,y,8,color1);
    }
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
    
    TRISAbits.TRISA4 = 0; // output pin
    TRISBbits.TRISB4 = 1; // input pin button
    LATAbits.LATA4 = 1; // sets A4 to high initially for testing
    int alternater = 1; // keeps track if on or off
    LCD_init();
    LCD_clearScreen(0x0000);
    char message[30];
    sprintf(message,"Hello World");
    drawString(30,10,message,0xFFFF,0x0000);
    drawBox(10,60,100,WHITE);

    __builtin_enable_interrupts();

    _CP0_SET_COUNT(0);
    
    short count = 0;
    while(1) {
        // use _CP0_SET_COUNT(0) and _CP0_GET_COUNT() to test the PIC timing
        // remember the core timer runs at half the sysclk
        // sysclk is at 48MHz
        while(PORTBbits.RB4 == 0){
            //do nothing and pause 
        }
        // core timer half of sysclk -- core timer is 24KHZ, want full cycle 2HZ, divide by 12000
//        if(_CP0_GET_COUNT() > 12000){
//            if(alternater == 1){
//                alternater = 0;
//                LATAbits.LATA4 = 0;
//            }else {
//                alternater = 1;
//                LATAbits.LATA4 = 1;
//            }
//            if(count == 100){
//                count == 0;
//                drawBox(10,60,110,WHITE);
//            }
//            drawLine(10+count,60,8,BLUE);
//            
//            
//            
//            _CP0_SET_COUNT(0);
//        }
         if(_CP0_GET_COUNT() > 1200000){
            if(count == 100){
                count = 0;
                drawBox(10,60,100,WHITE);
            }
            drawLine(10+count,60,8,BLUE);
            count++;
            
            
            _CP0_SET_COUNT(0);
        }
        
        
    }
}