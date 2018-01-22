/******************************************************************************
* Project Name		: CE95277 ADC and UART
* File Name			: main.c
* Version 			: **
* Device Used		: CY8C5888LTI-LP097
* Software Used		: PSoC Creator 3.1 SP2
* Compiler    		: ARM GCC 4.8.4, ARM RVDS Generic, ARM MDK Generic
* Related Hardware	: CY8CKIT059 PSoC 5 LP Prototyping Kit 
* Owner				: KLMZ
*
********************************************************************************
* Copyright (2015), Cypress Semiconductor Corporation. All Rights Reserved.
********************************************************************************
* This software is owned by Cypress Semiconductor Corporation (Cypress)
* and is protected by and subject to worldwide patent protection (United
* States and foreign), United States copyright laws and international treaty
* provisions. Cypress hereby grants to licensee a personal, non-exclusive,
* non-transferable license to copy, use, modify, create derivative works of,
* and compile the Cypress Source Code and derivative works for the sole
* purpose of creating custom software in support of licensee product to be
* used only in conjunction with a Cypress integrated circuit as specified in
* the applicable agreement. Any reproduction, modification, translation,
* compilation, or representation of this software except as specified above 
* is prohibited without the express written permission of Cypress.
*
* Disclaimer: CYPRESS MAKES NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, WITH 
* REGARD TO THIS MATERIAL, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
* Cypress reserves the right to make changes without further notice to the 
* materials described herein. Cypress does not assume any liability arising out 
* of the application or use of any product or circuit described herein. Cypress 
* does not authorize its products for use as critical components in life-support 
* systems where a malfunction or failure may reasonably be expected to result in 
* significant injury to the user. The inclusion of Cypress' product in a life-
* support systems application implies that the manufacturer assumes all risk of 
* such use and in doing so indemnifies Cypress against all charges. 
*
* Use of this Software may be limited by and subject to the applicable Cypress
* software license agreement. 
*******************************************************************************/

#include <device.h>
#include "stdio.h"

/* Project Defines */
#define FALSE  0
#define TRUE   1
#define TRANSMIT_BUFFER_SIZE  50
#define ArraySize 8
#define cArraySize 91
#define sRate 0.001f

 /* Defines for DMA */
#define DMA_BYTES_PER_BURST 3
#define DMA_REQUEST_PER_BURST 1
#define DMA_SRC_BASE (CYDEV_PERIPH_BASE)
#define DMA_DST_BASE (CYDEV_PERIPH_BASE)



uint8 ContinuouslySendData;
uint8 SendSingleByte;
uint8 SendEmulatedData;
uint8 WalkingSpeedTest;
uint8 WalkingSpeedTest2;
uint8 NextCalibrate;
uint8 Calibrate;
uint8 Help;
uint8 Quit;
int16 distance,d;
uint8 CaliDisplay;
uint8 PrintEEPROM;
    
int n=0,b=0;
float32 t=0;
int16 Output,OutputD;
int16 outputA[ArraySize],outputDif[ArraySize-1],outputD[ArraySize];
float outputSpeed[ArraySize-1];
char TransmitBuffer[TRANSMIT_BUFFER_SIZE];
uint8 FlagADC; 
uint8 Ch;
int16 rDistance[cArraySize];
double fit[5]={588.3384,-1.5302,0.0020826,-0.0000013075,0.0000000002883334};
union
{
    int16 cD16[cArraySize];
    uint8 cD8[cArraySize*2];
}cDistance;
/*******************************************************************************
* Function Name: main
********************************************************************************
*
* Summary:
*  main() performs following functions:
*  1: Starts the ADC and UART components.
*  2: Checks for ADC end of conversion.  Stores latest result in output
*     if conversion complete.
*  3: Checks for UART input.
*     On 'C' or 'c' received: transmits the last sample via the UART.
*     On 'S' or 's' received: continuously transmits samples as they are completed.
*     On 'X' or 'x' received: stops continuouslsy transmitting samples.
*     On 'E' or 'e' received: transmits a dummy byte of data.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
*******************************************************************************/
void DMA_config(void);

CY_ISR(USB_ISR)
{
    Ch = UART_1_GetChar();
        switch(Ch)
        {
            case 0:
                /* No new data was recieved */
                break;
            case 'C':
            case 'c':
                Calibrate = TRUE;
                CaliDisplay = TRUE;
                break;
            case 'N':
            case 'n':
                NextCalibrate = TRUE;
                break;
            case 'Q':
            case 'q':
                Quit = TRUE;
                break;
            case 'S':
            case 's':
                SendSingleByte = TRUE;
                break;
            case 'M':
            case 'm':
                ContinuouslySendData = TRUE;
                break;
            case 'X':
            case 'x':
                ContinuouslySendData = FALSE;
                break;
            case 'E':
            case 'e':
                SendEmulatedData = TRUE;
                break;
//            case 'R':
//            case 'r':
//                WalkingSpeedTest = TRUE;
//                break;
            case 'W':
            case 'w':
                WalkingSpeedTest2 = TRUE;
                break;
            case 'P':
            case 'p':
                PrintEEPROM = TRUE;
                break;
            case 'H':
            case 'h':
                Help = TRUE;
                break;
            default:
                /* Place error handling code here */
                break;    
        }
        Ch=0;
}


CY_ISR(MyISR)
{
    outputA[n]=Output;
    Timer_ReadStatusRegister();
    n++;
}

CY_ISR(buttonISR)
{
    WalkingSpeedTest2 = TRUE;
}



void Swap(int16* array, int16 x, int16 y)
{
    int16 temp;
    temp= array[x];
    array[x] = array[y];
    array[y] = temp;
}
void BubbleSort(int16* array, int size)
{
    int16 i,j;
    for(i = 0; i < size; i++)
    {
        for(j = 1; j < size - i; j++)
        {
            if(array[j] < array[j - 1])
            {
                Swap(array, j, j - 1);
            }
        }
    }
}
  

int16 Interpolate(int16* xs,int16* ys, double x)
{
    /* number of elements in the array */
    static const int count = sizeof(xs)/sizeof(xs[0]);

    int i;
    double dx, dy;

    if (x < xs[0]) {
        /* x is less than the minimum element
         * handle error here if you want */
        return ys[0]; /* return minimum element */
    }

    if (x > xs[count-1]) {
        return ys[count-1]; /* return maximum */
    }

    /* find i, such that xs[i] <= x < xs[i+1] */
    for (i = 0; i < count-1; i++) {
        if (xs[i+1] > x) {
            break;
        }
    }

    /* interpolate */
    dx = xs[i+1] - xs[i];
    dy = ys[i+1] - ys[i];
    return (int16)ys[i] + (x - xs[i]) * dy / dx;
}
int16 CurveFitting(int16 A)
{
    int16 x;
    A=A-1400;
    x=(int16)(fit[0]+fit[1]*A+fit[2]*A*A+fit[3]*A*A*A+fit[4]*A*A*A*A);
    return x;
}
CY_ISR(ADCisr)
{
  //  Filter_ClearInterruptSource();
    FlagADC=1; 
    t++;
    Output = ADC_DelSig_1_CountsTo_mVolts(Filter_Read24(Filter_CHANNEL_A));
    if(SendSingleByte || ContinuouslySendData)
    {
        /* Format ADC result for transmition */
        OutputD= CurveFitting(Output);
        sprintf(TransmitBuffer, "%5dcm %5dmV \r\n", OutputD, Output);
        
        /* Send out the data */
        UART_1_PutString(TransmitBuffer);
        LCD_Position(1,0);
        LCD_PrintString(TransmitBuffer);
        /* Reset the send once flag */
        SendSingleByte = FALSE;
    }
}

void main()
{



    uint8 k=0,i;
    int e;
    /* Variable used to send emulated data */
    uint8 EmulatedData;
    
    /* Flags used to store transmit data commands */

    float32 sumY=0,sumX=0,sumX2=0,sumXY=0,wSpeed;
    /* Transmit Buffer */
    
    LCD_Start();
 //   LCD_DisplayOn();
    LCD_Position(0,1);
//    LCD_PrintInt16(7);
    LCD_PrintString("Walking Speed");
    /* Start the components */
    ADC_DelSig_1_Start();
    UART_1_Start();
    isr_1_StartEx(MyISR);
    isr_2_StartEx(ADCisr);
    isr_3_StartEx(buttonISR);
    isr_UART_StartEx(USB_ISR);
    CyGlobalIntEnable;
    Filter_Start();
    Filter_SetCoherency(Filter_CHANNEL_A, Filter_KEY_HIGH);
 //   ADC_DelSig_1_SetCoherency(ADC_DelSig_1_COHER_HIGH);
    DMA_config();
    /* Initialize Variables */
    ContinuouslySendData = FALSE;
    PrintEEPROM = FALSE;
    SendSingleByte = FALSE;
    SendEmulatedData = FALSE;
    WalkingSpeedTest = FALSE;
    WalkingSpeedTest2 = FALSE;
    Calibrate = FALSE;
    NextCalibrate = FALSE;
    CaliDisplay = FALSE;
    Help= FALSE;
    EmulatedData = 0;
    FlagADC=0;
    n=0;
    d=0;
    sumX=0;
    sumY=0;
    sumX2=0;
    sumXY=0;
    t=0;
    
    /* Start the ADC conversion */
    ADC_DelSig_1_StartConvert();
    
    
    /* Send message to verify COM port is connected properly */
    UART_1_PutString("COM Port Open");
    
    for(i=0;i<cArraySize;i++)
    {
        rDistance[i]=i*5+100;   
    }
    
    EEPROM_Start();
    EEPROM_UpdateTemperature();
    for(e=0;e<cArraySize*2;e++)
    {
        cDistance.cD8[e]=EEPROM_ReadByte(e);
    }
    EEPROM_Stop();

//ContinuouslySendData = TRUE;
    
    for(;;)
    {        
        /* Non-blocking call to get the latest data recieved  */
        
        
        /* Set flags based on UART command */

        if(Help)
        {
                //sprintf(TransmitBuffer, "cData:%d cm %d mV \r\n",rDistance[i], cDistance.cD16[i]);
                UART_1_PutString("\r\nW: Start walking speed test\r\n");
                UART_1_PutString("C: Calibration\r\n");
                UART_1_PutString("N: next calibration\r\n");
                UART_1_PutString("Q: quit calibration\r\n");
                UART_1_PutString("P: print EEEPROM\r\n");
                UART_1_PutString("S: Send Single Byte\r\n");
                UART_1_PutString("M: Send Data Continuously\r\n");
                UART_1_PutString("X: Stop Send Data Continuously\r\n");
                Help = FALSE;
                
            
        }
        if(PrintEEPROM)
        {
            for(i=0;i<cArraySize;i++)
            {
                sprintf(TransmitBuffer, "%5dcm %5dmV\r\n",rDistance[i], cDistance.cD16[i]);
                UART_1_PutString(TransmitBuffer);
            }
            PrintEEPROM = FALSE;
        }
        /* Check to see if an ADC conversion has completed */
        if(FlagADC)
        {
////           Output = ADC_DelSig_1_CountsTo_mVolts(Filter_Read24(Filter_CHANNEL_A));
//           Output = Filter_Read16(Filter_CHANNEL_A);
//            /* Send data based on last UART command */
//            if(SendSingleByte || ContinuouslySendData)
//            {
//                /* Format ADC result for transmition */
//                OutputD= CurveFitting(Output);
//                sprintf(TransmitBuffer, "%5dcm %5dmV \r\n", OutputD, Output);
//                
//                /* Send out the data */
//                UART_1_PutString(TransmitBuffer);
//                LCD_Position(1,0);
//                LCD_PrintString(TransmitBuffer);
//                /* Reset the send once flag */
//                SendSingleByte = FALSE;
//            }
            
            
            if(Calibrate)
            {
                if (CaliDisplay)
                {
                    sprintf(TransmitBuffer, "move to distance %d cm,and press n \r\n", rDistance[d]);
                    CaliDisplay = FALSE;
                    UART_1_PutString(TransmitBuffer);
                    LCD_Position(0,0);
                    LCD_PrintString(TransmitBuffer);
                }
                /* Send out the data */
                
                if(NextCalibrate == TRUE)
                {
                    NextCalibrate = FALSE;
                    CaliDisplay = TRUE;
                    cDistance.cD16[d]=Output;
                    
                    sprintf(TransmitBuffer, "distance %d cm, Voltage: %d mV \r\n", rDistance[d],cDistance.cD16[d] );
                    UART_1_PutString(TransmitBuffer);
                    LCD_Position(0,0);
                    LCD_PrintString(TransmitBuffer);
                    d++;  
                }
                if(d>=cArraySize) 
                {    
                    Calibrate = FALSE;
                    EEPROM_Start();
                    EEPROM_UpdateTemperature();
                    for(e=0;e<cArraySize*2;e++)
                    {                        
                        EEPROM_WriteByte(cDistance.cD8[e],e);
                        CyDelay(20);
                    }
                    EEPROM_Stop();
                    UART_1_PutString("calibration done \r\n");
                }
                if(Quit) 
                {
                    Calibrate = FALSE;
                    UART_1_PutString("quit calibration  \r\n");
                    Quit = FALSE;
                }                      
            }
            
//            if(SendEmulatedData)
//            {
//                /* Format ADC result for transmition */
//                sprintf(TransmitBuffer, "Emulated Data: %x \r\n", EmulatedData);
//                /* Send out the data */
//                UART_1_PutString(TransmitBuffer);
//                EmulatedData++;
//                /* Reset the send once flag */
//                SendEmulatedData = FALSE;   
//            }
//            
//            if(WalkingSpeedTest)
//            {
//                WalkingSpeedTest = FALSE;
//                Timer_Start();  
//            }
            
            if(WalkingSpeedTest2)
            {
                LCD_ClearDisplay();
                LCD_Position(0,0);
                LCD_PrintString(" Walking Speed");
                LCD_Position(1,2);
                LCD_PrintString("Start Testing");
                
                Output= CurveFitting(Output);
                if(Output>=100&& Output<=145 )
                {
                    t=0;
                    b=0;
                    sumY=0;
                    sumX=0;
                    sumX2=0;
                    sumXY=0;
                    
                }
                else if( Output>150 && Output <500)
                {
                    sumY+=Output;
            		sumX+=sRate*t;
            		sumX2+=sRate*t*sRate*t;
            		sumXY+=sRate*t*Output;
                    b++;
//                    sprintf(TransmitBuffer, "%d %f", b,t);
//                    UART_1_PutString(TransmitBuffer);
                }
                else if(Output >= 500 && Output <550)
                {
                    WalkingSpeedTest2= FALSE;
                    wSpeed=(( b * sumXY)- (sumX * sumY))/((b * sumX2)-( sumX * sumX));
                    sprintf(TransmitBuffer, "Walking Speed: %d cm/sec \r\n", (int16)wSpeed);
                    UART_1_PutString(TransmitBuffer);
                    LCD_ClearDisplay();
                    LCD_Position(0,0);
                    LCD_PrintString("Walking Speed:");
                    LCD_Position(1,0);
                    sprintf(TransmitBuffer, " %5d cm/sec", (int16)wSpeed);
                    LCD_PrintString(TransmitBuffer);
                }
                else
                {

                }
                
                //WalkingSpeedTest2 = FALSE;
 
            }
            
         
            if (n>=ArraySize)
            {
                n=0;
                Timer_Stop();
                for(i=0;i<ArraySize;i++)
                {
//                    outputD[i]=Interpolate(cDistance.cD16,rDistance,outputA[i]);
                    outputD[i]= CurveFitting(outputA[i]);
                }
                for(k=0; k<ArraySize-1;k++) outputDif[k]=outputD[k+1]-outputD[k];
                
                sprintf(TransmitBuffer, "Voltage:%d mV,%d mV,%d mV,%d mV,%d mV,%d mV,%d mV,%d mV \r\n", outputA[0],outputA[1],outputA[2],outputA[3],outputA[4],outputA[5],outputA[6],outputA[7]);
                UART_1_PutString(TransmitBuffer);
                sprintf(TransmitBuffer, "Distance:%d cm,%d cm,%d cm,%d cm,%d cm,%d cm,%d cm,%d cm \r\n", outputD[0],outputD[1],outputD[2],outputD[3],outputD[4],outputD[5],outputD[6],outputD[7]);
                UART_1_PutString(TransmitBuffer);
                sprintf(TransmitBuffer, "Displace:%d cm,%d cm,%d cm,%d cm,%d cm,%d cm,%d cm \r\n", outputDif[0],outputDif[1],outputDif[2],outputDif[3],outputDif[4],outputDif[5],outputDif[6]);
                UART_1_PutString(TransmitBuffer);
                for(k=0; k<ArraySize-1;k++)outputSpeed[k]=(float)outputDif[k]*2/100;
                //BubbleSort(outputDif,ArraySize-1);
                sprintf(TransmitBuffer, "speed:%.3fm/s,%.3fm/s,%.3fm/s,%.3f m/s,%.3fm/s,%.3fm/s,%.3fm/s \r\n", outputSpeed[0],outputSpeed[1],outputSpeed[2],outputSpeed[3],outputSpeed[4],outputSpeed[5],outputSpeed[6]);
                UART_1_PutString(TransmitBuffer);
            }
            FlagADC=0;
//            Filter_ClearInterruptSource();
        }

    }
}

void DMA_config(void)
{
    /* Variable declarations for DMA */
    /* Move these variable declarations to the top of the function */
    uint8 DMA_Chan;
    uint8 DMA_TD[1];

    /* DMA Configuration for DMA */
    DMA_Chan = DMA_DmaInitialize(DMA_BYTES_PER_BURST, DMA_REQUEST_PER_BURST, 
        HI16(DMA_SRC_BASE), HI16(DMA_DST_BASE));
    DMA_TD[0] = CyDmaTdAllocate();
    CyDmaTdSetConfiguration(DMA_TD[0], 3, DMA_TD[0],TD_INC_SRC_ADR | TD_INC_DST_ADR);
    CyDmaTdSetAddress(DMA_TD[0], LO16((uint32)ADC_DelSig_1_DEC_SAMP_PTR), LO16((uint32)Filter_STAGEA_PTR));
    CyDmaChSetInitialTd(DMA_Chan, DMA_TD[0]);
    CyDmaChEnable(DMA_Chan, 1);
}

/* [] END OF FILE */
