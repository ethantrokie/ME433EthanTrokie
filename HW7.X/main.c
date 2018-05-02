#include<xc.h>
#include<sys/attribs.h> 
#include "i2c/i2c_master_noint.h"
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
        if((pixels >> j & 1) == 1){
            if((x+col < 128) && (y+j < 160)){
            LCD_drawPixel(x+col,y+j,c1);
            }
        }else{
            if((x+col < 128) && (y+j < 160)){
            LCD_drawPixel(x+col,y+j,c2);
            }
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

void drawVLine(short x, short y, short height,short color){
    int i;
    for(i = 0; i < height; i++){
        LCD_drawPixel(x,y+i,color);
    }
}
void drawHBox(short x, short y, short width, short total, short color1,int pos){
    int i;
    for(i = 0; i < width; i++){
        if(pos == 1){
            drawVLine(x+i,y,4,color1);
        }else{
            drawVLine(x-i,y,4,color1);
        }
    }
    for(i = width; i < total; i++){
        if(pos == 1){
            drawVLine(x+i,y,4,WHITE);
        }else{
            drawVLine(x-i,y,4,WHITE);
        }
    }
}


void drawHLine(short x, short y, short width, short color){
    int i;
    for(i = 0; i < width; i++){
        LCD_drawPixel(x+i,y,color);
    }
}
void drawVBox(short x, short y, short height,short total,short color1,int pos){
    int i;
    for(i = 0; i < height; i++){
        if(pos == 1){
            drawHLine(x,y+i,4,color1);
        }else{
            drawHLine(x,y-i,4,color1);
        }
    }
    for(i = height; i < total; i++){
        if(pos == 1){
            drawHLine(x,y+i,4,WHITE);
        }else{
            drawHLine(x,y-i,4,WHITE);
        }
    }
}


void initExpander(){
    
    ANSELBbits.ANSB2 = 0;
    ANSELBbits.ANSB3 = 0;
    i2c_master_setup();
}

void setExpander(char reg, char level){
    i2c_master_start();
    i2c_master_send(0b1101011<<1|0);
    i2c_master_send(reg); // the register to write to
    i2c_master_send(level); // the value to put in the register
    i2c_master_stop();
}
unsigned char getExpander(){
    i2c_master_start();
    i2c_master_send(0b1101011<<1|0);
    i2c_master_send(0x0F);
    i2c_master_restart(); // make the restart bit
    i2c_master_send(0b1101011<<1|1);
    unsigned char r = i2c_master_recv(); // save the value returned
    i2c_master_ack(1);
    i2c_master_stop(); // make the stop bit
    return r;
}
void I2C_read_multiple(unsigned char address, unsigned char reg, unsigned char * data, int length){
    i2c_master_start();
    i2c_master_send(0b1101011<<1|0);
    i2c_master_send(reg);
    i2c_master_restart(); // make the restart bit
    i2c_master_send(0b1101011<<1|1);
    int i = 0;
    for(i = 0; i < length; i++){
        data[i] = i2c_master_recv(); // save the value returned
        if (i < (length - 1)){
        i2c_master_ack(0);
        }else{
        i2c_master_ack(1);
        }
    }  
    i2c_master_stop(); // make the stop bit
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
    
    initExpander();
    
    setExpander(0x10,0b10000010); //set up accelarometer
    setExpander(0x11,0b10001000);  //set all outputs as high
    setExpander(0x12,0b00000100);

    LCD_init();
    LCD_clearScreen(0x0000);
    //drawBox(15,60,100,WHITE);

    __builtin_enable_interrupts();

    drawHBox(80,80,50,50,WHITE, 1);
    drawHBox(80,80,50,50,WHITE, 0);
    drawVBox(80,80,50,50,WHITE, 1);
    drawVBox(80,80,50,50,WHITE, 0);
    
    _CP0_SET_COUNT(0);
    unsigned char data[14];
    short dataReal[7];
    while(1) {
        char message1[30];
        if(getExpander() != 0x69){
            sprintf(message1,"PROBLEM");
            drawString(28,62,message1,0xFFFF,0x0000);
            while(1){;}
        }
//        if(getExpander()>>7 == 1){
//            setExpander(0xA,1);
//        }else{
//            setExpander(0xA,0);
//        }
        // use _CP0_SET_COUNT(0) and _CP0_GET_COUNT() to test the PIC timing
        // remember the core timer runs at half the sysclk
        // sysclk is at 48MHz
        while(PORTBbits.RB4 == 0){
            //do nothing and pause 
        }
        // core timer half of sysclk -- core timer is 24MHZ, want full cycle 2HZ, divide by 12000
        if(_CP0_GET_COUNT() > 1200000){
            if(alternater == 1){
                alternater = 0;
                LATAbits.LATA4 = 0;
                
            }else {
                alternater = 1;
                LATAbits.LATA4 = 1;
                
            }
            I2C_read_multiple(0b1101011, 0x20, data, 14);
            dataReal[0] = (data[1]<<8) | data[0];
            dataReal[1] = (data[3]<<8) | data[2];
            dataReal[2] = (data[5]<<8) | data[4];
            dataReal[3] = (data[7]<<8) | data[6];
            dataReal[4] = (data[9]<<8) | data[8];
            dataReal[5] = (data[11]<<8) | data[10];
            dataReal[6] = (data[13]<<8) | data[12];
//            char message2[30];
//            sprintf(message2,"x %d y: %d    ",dataReal[4],dataReal[5]);
//            drawString(28,32,message2,0xFFFF,0x0000);  
            
            if(dataReal[4] > 0){
            drawHBox(80,80,abs(dataReal[4])*50.0/16000.0,50,BLUE,0);
            }else{
            drawHBox(80,80,abs(dataReal[4])*50.0/16000.0,50,BLUE,1);    
            }
            if(dataReal[5] > 0){
            drawVBox(80,80,abs(dataReal[5])*50.0/16000.0,50,BLUE,0 );
            }else{
            drawVBox(80,80,abs(dataReal[5])*50.0/16000.0,50,BLUE,1 );   
            }
            
            
            _CP0_SET_COUNT(0);
        }
        
        
    }
}