/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include "app.h"
#include "i2c_master_noint.h"
#include<stdio.h>
#include "ST7735.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
*/

APP_DATA appData;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************


/* TODO:  Add any necessary local functions.
*/


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */


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





int alternater = 1;
unsigned char data[14];
short dataReal[7];
void APP_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;

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
     // keeps track if on or off
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
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
}


/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks ( void )
{

    /* Check the application's current state. */
    switch ( appData.state )
    {
        /* Application's initial state. */
        case APP_STATE_INIT:
        {
            bool appInitialized = true;
       
        
            if (appInitialized)
            {
            
                appData.state = APP_STATE_SERVICE_TASKS;
            }
            break;
        }

        case APP_STATE_SERVICE_TASKS:
        {
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
            break;
        }

        /* TODO: implement your application state machine.*/
        

        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}

 

/*******************************************************************************
 End of File
 */
