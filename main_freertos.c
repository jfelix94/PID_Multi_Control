/*
 * Copyright (c) 2016-2017, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== main_freertos.c ========
 */
#include <stdint.h>
#include <stddef.h>

#ifdef __ICCARM__
#include <DLib_Threads.h>
#endif

/* POSIX Header files */
#include <pthread.h>

/* RTOS header files */
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <ti/drivers/GPIO.h>
#include <ti/drivers/UART.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/ADC.h>
#include <ti/display/Display.h>

/* Example/Board Header files */
#include "Board.h"
#include "value.h"

extern void *mainThread(void *arg0);

/* Stack size in bytes */
#define THREADSTACKSIZE   1024

//Variables
static Display_Handle display;
uint32_t adcValue0, adcValue1;
uint32_t adcValue0MicroVolt, adcValue1MicroVolt;
uint8_t txBuffer[2];
uint8_t rxBuffer[2];
SemaphoreHandle_t xMutex;
SemaphoreHandle_t xMutex2;
uint8_t mirrorRegister[26] = "x Hello this is master";
int_fast16_t res;
ADC_Handle   adc, adc1;
ADC_Params   params;
I2C_Params   i2cParams;
I2C_Handle   i2c;
I2C_Transaction i2cTransaction;
bool retVal = false;

TaskHandle_t xPID1;
TaskHandle_t xPID2;
TaskHandle_t xPIDMulti;
TickType_t t1,t2;

/*
 *  ======== main ========
 */

uint32_t vADC(char channel);
void vI2C_DAC(char channel, float valueFloat);
void vSerial(void *pvParameters);
void vInitPerif(void);
void vPIDMulti (void *pvParameters);
void vPID1(void *pvParameters);
void vPID2(void *pvParameters);
void vTeste1(void *pvParameters);


int main(void)
{
    /* Call driver init functions */
    Board_initGeneral();
    /* Call driver init functions */
    vInitPerif();
    //display = Display_open(Display_Type_UART, NULL);
    xMutex = xSemaphoreCreateMutex();
    xMutex2 = xSemaphoreCreateMutex();
    /*Creat Tasks*/
    xTaskCreate(vSerial, "Serial", 1000, NULL, 3, NULL);
    xTaskCreate(vTeste1, "Teste1", 1000, NULL, 1, NULL);
    xTaskCreate(vPIDMulti, "PIDMulti", 1000, NULL, 2, &xPIDMulti);
    xTaskCreate(vPID1, "PID1", 1000, NULL, 1, &xPID1);
    xTaskCreate(vPID2, "PID2", 1000, NULL, 1, &xPID2);
    //xTaskCreate(vADC, "ADC", 1000, NULL, 2, NULL);
    //xTaskCreate(vI2C_DAC, "DAC", 1000, NULL, 2, NULL);

    /* Start the FreeRTOS scheduler */
    vTaskStartScheduler();

    for(;;);

    return (0);
}

void vInitPerif(void){

    GPIO_init();
    ADC_init();

    Init_valores();
    Display_init();

    /* Call driver init functions */
    I2C_init();

    ADC_Params_init(&params);
    adc = ADC_open(Board_ADC0, &params);
    adc1 = ADC_open(Board_ADC1, &params);

    if (adc == NULL) {
           //Display_printf(display, 0, 0, "Error initializing ADC channel 0\n");
           while (1);
    }

    if (adc1 == NULL) {
           //Display_printf(display, 0, 0, "Error initializing ADC channel 1\n");
           while (1);
    }

    /* Create I2C for usage */
       I2C_Params_init(&i2cParams);
       i2cParams.transferMode = I2C_MODE_BLOCKING;
       i2cParams.bitRate = I2C_1000kHz;
       i2c = I2C_open(Board_I2C_TMP, &i2cParams);

       if (i2c == NULL) {
          //Display_printf(display, 0, 0, "Error Initializing I2C!\n");
       }
       else {
          //Display_printf(display, 0, 0, "I2C Initialized!\n");
       }

       uint8_t i = 0;
       /* Initialize all buffers */
       for (i = 0; i < 2; i++) {
           rxBuffer[i] = 0x00;
           txBuffer[i] = 0x00;
       }

    //Display_printf(display, 0, 0, "Master: %s\n", mirrorRegister);
}

void vTeste1 (void *pvParameters){
    char ctrl, ctrl1, ctrl2;
    TickType_t xLastWakeTime;
    ctrl=0;
    ctrl1=0;
    ctrl2=0;
    for(;;){
        xLastWakeTime = xTaskGetTickCount();

        if(ctrl == 0){
            if(ctrl1==0){
                Altera_ref1(1861);
            }

            if(ctrl1==1){
                Altera_ref1(1365);
            }
            ctrl1 = (ctrl1+1)%2;
            ctrl = 1;
        }else if(ctrl == 1){
            if(ctrl2==0){
                Altera_ref2(2233);
            }

            if(ctrl2==1){
                Altera_ref2(1737);
            }
            ctrl2 = (ctrl2+1)%2;
            ctrl = 0;
        }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(8000));
    }
}

void vPIDMulti (void *pvParameters){
    float ie1[2], ie2[2];
    float e1[2], e2[2];
    float de1, de2;
    float r[2];
    float u[4];
    float Ki[4],Kp[4], Kd[4];
    float y;
    float Tsz;
    int   Ts;

    TickType_t xLastWakeTime;

    vTaskSuspend(xPID1);
    vTaskSuspend(xPID2);
    for(;;){
    xLastWakeTime = xTaskGetTickCount();

    Ts = Retorna_timerMulti();
    Tsz = Ts;
    Tsz = Tsz/1000;
    //++++++++++++++++++++++++ Calculo dos erros +++++++++++++++++++++++++++
    //+++++++++++++++++++++++++++++++ 1 ++++++++++++++++++++++++++++++++++++
        //Retorna valores para calculo de erros
        e1 [0] = Retorna_e1();
        ie1 [0] = Retorna_ie1();
        r [0] = Retorna_ref1();

        y = vADC(0);
        //erro = referência - valor
        e1[1] = r[0]-y;

        //erro integrador
        ie1[1] = (Tsz*(e1[1]+e1[0])/2) + ie1[0];

        //erro derivador
        de1 = (e1[1]-e1[0])/Tsz;

        //Substitui valores de erro antigos pelos novos
        Altera_e1(e1[1]);
        Altera_ie1(ie1[1]);

    //+++++++++++++++++++++++++++++++ 2 ++++++++++++++++++++++++++++++++++++
        //Retorna valores para calculo de erroos
        e2 [0] = Retorna_e2();
        ie2 [0] = Retorna_ie2();
        r [1] = Retorna_ref2();
        y = vADC(1);

        //erro = referência - valor
        e2[1] = r[1]-y;

        //erro integrador
        ie2[1] = (Tsz*(e2[1]+e2[0])/2) + ie2[0];

        //erro derivador
        de2 = (e2[1]-e2[0])/Tsz;

        //Substitui valores de erro antigos pelos novos
        Altera_e2(e2[1]);
        Altera_ie2(ie2[1]);

    //++++++++++++++++++++++++++++++ PID +++++++++++++++++++++++++++++++++++
        Ki [0] = Retorna_Ki(0);
        Kd [0] = Retorna_Kd(0);
        Kp [0] = Retorna_Kp(0);
        Ki [1] = Retorna_Ki(1);
        Kd [1] = Retorna_Kd(1);
        Kp [1] = Retorna_Kp(1);
        Ki [2] = Retorna_Ki(2);
        Kd [2] = Retorna_Kd(2);
        Kp [2] = Retorna_Kp(2);
        Ki [3] = Retorna_Ki(3);
        Kd [3] = Retorna_Kd(3);
        Kp [3] = Retorna_Kp(3);

    //+++++++++++++++++++++++++++++++ 1 ++++++++++++++++++++++++++++++++++++
        u[0] = (Kp[0]*e1[1]) + (Ki[0]*ie1[1]) + (Kd[0]*de1);
        u[1] = (Kp[2]*e2[1]) + (Ki[2]*ie2[1]) + (Kd[2]*de2);

        u[0] = u[1] + u[0];

        if(u[0] > 4095){
            u[0] = 4095;
        }
        if(u[0] < 0){
            u[0] = 0;
        }

    //+++++++++++++++++++++++++++++++ 2 ++++++++++++++++++++++++++++++++++++
        u[2] = (Kp[1]*e2[1]) + (Ki[1]*ie2[1]) + (Kd[1]*de2);
        u[3] = (Kp[3]*e1[1]) + (Ki[3]*ie1[1]) + (Kd[3]*de1);

        u[2] = u[2] + u[3];

        if(u[2] > 4095){
            u[2] = 4095;
        }
        if(u[2] < 0){
            u[2] = 0;
        }

    //++++++++++++++++++++++++++++ Retorna ++++++++++++++++++++++++++++++++++
        vI2C_DAC(0, u[0]);
        vI2C_DAC(1, u[2]);

        t2 = xTaskGetTickCount() - xLastWakeTime;
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(Ts));
    }
}

void vPID1 (void *pvParameters){
    float ie1[2], ie2[2];
    float e1[2], e2[2];
    float de1, de2;
    float r[2];
    float u[4];
    float Ki[2],Kp[2], Kd[2];
    float y;
    float Tsz;
    int   Ts;

    TickType_t xLastWakeTime;

    for(;;){
    xLastWakeTime = xTaskGetTickCount();

    Ts = Retorna_timer1();
    Tsz = Ts;
    Tsz = Tsz/1000;
    //++++++++++++++++++++++++ Calculo dos erros +++++++++++++++++++++++++++
    //+++++++++++++++++++++++++++++++ 1 ++++++++++++++++++++++++++++++++++++
        //Retorna valores para calculo de erros
        e1 [0] = Retorna_e1();
        ie1 [0] = Retorna_ie1();
        r [0] = Retorna_ref1();


        xSemaphoreTake(xMutex2, portMAX_DELAY);
            y = vADC(0);
        xSemaphoreGive(xMutex2);
        //erro = referência - valor
        e1[1] = r[0]-y;

        //erro integrador
        ie1[1] = (Tsz*(e1[1]+e1[0])/2) + ie1[0];

        //erro derivador
        de1 = (e1[1]-e1[0])/Tsz;

        //Substitui valores de erro antigos pelos novos
        Altera_e1(e1[1]);
        Altera_ie1(ie1[1]);

    //++++++++++++++++++++++++++++++ PID +++++++++++++++++++++++++++++++++++
        Ki [0] = Retorna_Ki(0);
        Kd [0] = Retorna_Kd(0);
        Kp [0] = Retorna_Kp(0);

    //+++++++++++++++++++++++++++++++ 1 ++++++++++++++++++++++++++++++++++++
        u[0] = (Kp[0]*e1[1]) + (Ki[0]*ie1[1]) + (Kd[0]*de1);

        if(u[0] > 16383){
            u[0] = 16383;
        }
        if(u[0] < 0){
            u[0] = 0;
        }
    //++++++++++++++++++++++++++++ Retorna ++++++++++++++++++++++++++++++++++
        xSemaphoreTake(xMutex, portMAX_DELAY);
            vI2C_DAC(0, u[0]);
        xSemaphoreGive(xMutex);

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(Ts));
    }
}

void vPID2(void *pvParameters){
    float ie1[2], ie2[2];
    float e1[2], e2[2];
    float de1, de2;
    float r[2];
    float u[4];
    float Ki[2],Kp[2], Kd[2];
    float y;
    float Tsz;
    int   Ts;

    TickType_t xLastWakeTime;

    for(;;){
    xLastWakeTime = xTaskGetTickCount();

    Ts = Retorna_timer2();
    Tsz = Ts;
    Tsz = Tsz/1000;
    //++++++++++++++++++++++++ Calculo dos erros +++++++++++++++++++++++++++
    //+++++++++++++++++++++++++++++++ 2 ++++++++++++++++++++++++++++++++++++
        //Retorna valores para calculo de erroos
        e2 [0] = Retorna_e2();
        ie2 [0] = Retorna_ie2();
        r [1] = Retorna_ref2();
        xSemaphoreTake(xMutex2, portMAX_DELAY);
            y = vADC(1);
        xSemaphoreGive(xMutex2);

        //erro = referência - valor
        e2[1] = r[1]-y;

        //erro integrador
        ie2[1] = (Tsz*(e2[1]+e2[0])/2) + ie2[0];

        //erro derivador
        de2 = (e2[1]-e2[0])/Tsz;

        //Substitui valores de erro antigos pelos novos
        Altera_e2(e2[1]);
        Altera_ie2(ie2[1]);

    //++++++++++++++++++++++++++++++ PID +++++++++++++++++++++++++++++++++++
        Ki [0] = Retorna_Ki(1);
        Kd [0] = Retorna_Kd(1);
        Kp [0] = Retorna_Kp(1);


    //+++++++++++++++++++++++++++++++ 2 ++++++++++++++++++++++++++++++++++++
        u[2] = (Kp[0]*e2[1]) + (Ki[0]*ie2[1]) + (Kd[0]*de2);

        if(u[2] > 16383){
            u[2] = 16383;
        }
        if(u[2] < 0){
            u[2] = 0;
        }

    //++++++++++++++++++++++++++++ Retorna ++++++++++++++++++++++++++++++++++
        xSemaphoreTake(xMutex, portMAX_DELAY);
            vI2C_DAC(1, u[2]);
        xSemaphoreGive(xMutex);

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(Ts));
    }
}

void vSerial(void *pvParameters){
        char i = 0;
        char j = 0;
        char k = 0;
        int l = 1;
        char stop=0;
        char input;
        char buff[50];
        float aux = 0;
        int auxt = 0;
        const char  echoPrompt[] = "oi\r\n";
        UART_Handle uart;
        UART_Params uartParams;
        TickType_t xLastWakeTime;

        /* Call driver init functions */
        GPIO_init();
        UART_init();

        // Configure the LED pin
        GPIO_setConfig(Board_GPIO_LED0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);

        // Turn on user LED
        GPIO_write(Board_GPIO_LED0, Board_GPIO_LED_ON);

        /* Create a UART with data processing off. */
        UART_Params_init(&uartParams);
        uartParams.writeDataMode = UART_DATA_BINARY;
        uartParams.readDataMode = UART_DATA_BINARY;
        uartParams.readReturnMode = UART_RETURN_FULL;
        uartParams.readEcho = UART_ECHO_OFF;
        uartParams.baudRate = 115200;

        uart = UART_open(Board_UART0, &uartParams);

        if (uart == NULL) {
            // UART_open() failed
            while (1);
        }

        while(1){
           xLastWakeTime = xTaskGetTickCount();

            //UART_write(uart, echoPrompt, sizeof(echoPrompt)-1);
            while(stop==0){
                UART_read(uart, &input, 1);
                buff[i] = input;
                if(buff[i] == ';'){
                   stop = 1;
                }
                i++;
            }
            l = 1;
            aux = 0;
            auxt = 0;

            if(buff[0] == '*'){
                if(buff[1] == 'P'){
                    if(buff[2] == '1'){
                        for(j=i-2; j>3;j--){
                            aux = aux + ((buff[j]-48)*l);
                            l = l*10;
                        }

                        Altera_Kp(aux, 0);
                    }
                    if(buff[2] == '2'){
                        for(j=i-2; j>3;j--){
                            aux = aux + ((buff[j]-48)*l);
                            l = l*10;
                        }

                        Altera_Kp(aux, 1);
                    }

                    if(buff[2] == '3'){
                        for(j=i-2; j>3;j--){
                            aux = aux + ((buff[j]-48)*l);
                            l = l*10;
                        }

                        Altera_Kp(aux, 2);
                    }

                    if(buff[2] == '4'){
                        for(j=i-2; j>3;j--){
                            aux = aux + ((buff[j]-48)*l);
                            l = l*10;
                        }

                        Altera_Kp(aux, 3);
                    }
                }

                if(buff[1] == 'I'){
                    if(buff[2] == '1'){
                        for(j=i-2; j>3;j--){
                            aux = aux + ((buff[j]-48)*l);
                            l = l*10;
                        }
                        Altera_Ki(aux, 0);
                    }
                    if(buff[2] == '2'){
                        for(j=i-2; j>3;j--){
                            aux = aux + ((buff[j]-48)*l);
                            l = l*10;
                        }
                        Altera_Ki(aux, 1);
                    }

                    if(buff[2] == '3'){
                        for(j=i-2; j>3;j--){
                            aux = aux + ((buff[j]-48)*l);
                            l = l*10;
                        }
                        Altera_Ki(aux, 2);
                    }

                    if(buff[2] == '4'){
                        for(j=i-2; j>3;j--){
                            aux = aux + ((buff[j]-48)*l);
                            l = l*10;
                        }
                        Altera_Ki(aux, 3);
                    }
                }

                if(buff[1] == 'D'){
                    if(buff[2] == '1'){
                        for(j=i-2; j>3;j--){
                            aux = aux + ((buff[j]-48)*l);
                            l = l*10;
                        }
                        Altera_Kd(aux, 0);
                    }
                    if(buff[2] == '2'){
                        for(j=i-2; j>3;j--){
                            aux = aux + ((buff[j]-48)*l);
                            l = l*10;
                        }
                        Altera_Kd(aux, 1);
                    }

                    if(buff[2] == '3'){
                        for(j=i-2; j>3;j--){
                            aux = aux + ((buff[j]-48)*l);
                            l = l*10;
                        }
                        Altera_Kd(aux, 2);
                    }

                    if(buff[2] == '4'){
                        for(j=i-2; j>3;j--){
                            aux = aux + ((buff[j]-48)*l);
                            l = l*10;
                        }
                        Altera_Kd(aux, 3);
                    }
                }

                if(buff[1] == 'R'){
                    if(buff[2] == '1'){
                        for(j=i-2; j>3;j--){
                            aux = aux + ((buff[j]-48)*l);
                            l = l*10;
                        }
                        Altera_ref1(aux);
                    }
                    if(buff[2] == '2'){
                        for(j=i-2; j>3;j--){
                            aux = aux + ((buff[j]-48)*l);
                            l = l*10;
                        }
                        Altera_ref2(aux);
                    }
                }

                if(buff[1] == 'T'){
                    if(buff[2] == '0'){
                        for(j=i-2; j>3;j--){
                            auxt = auxt + ((buff[j]-48)*l);
                            l = l*10;
                        }
                        Altera_timerMulti(auxt);
                    }
                    if(buff[2] == '1'){
                        for(j=i-2; j>3;j--){
                            auxt = auxt + ((buff[j]-48)*l);
                            l = l*10;
                        }
                        Altera_timer1(auxt);
                    }
                    if(buff[2] == '2'){
                        for(j=i-2; j>3;j--){
                            auxt = auxt + ((buff[j]-48)*l);
                            l = l*10;
                        }
                        Altera_timer2(auxt);
                    }
                }

                if(buff[1] == 'M'){
                    if(buff[2] == '0'){
                        if(buff[4] == '1'){
                            Init_valores();
                            vTaskSuspend(xPID1);
                            vTaskSuspend(xPID2);
                            vTaskResume(xPIDMulti);

                        }

                        if(buff[4] == '0'){
                            vTaskSuspend(xPIDMulti);
                        }
                    }
                    if(buff[2] == '1'){
                        if(buff[4] == '1'){
                            Init_valores1();
                            vI2C_DAC(1, 0);
                            vTaskSuspend(xPIDMulti);
                            vTaskResume(xPID1);

                        }

                        if(buff[4] == '0'){
                            vTaskSuspend(xPID1);
                        }
                    }
                    if(buff[2] == '2'){
                        if(buff[4] == '1'){
                            vI2C_DAC(0, 0);
                            Init_valores2();
                            vTaskSuspend(xPIDMulti);
                            vTaskResume(xPID2);
                        }

                        if(buff[4] == '0'){
                            vTaskSuspend(xPID2);
                        }
                    }
                }
            }
            stop = 0;
            UART_write(uart, buff, i-1);
            i = 0;
            vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100));
        }
}


uint32_t vADC(char channel){

    uint32_t adcValue;

    /* Blocking mode conversion */
    adcValue0 = 0;
    adcValue1 = 0;

    if(channel == 0){
        res = ADC_convert(adc, &adcValue0);
        adcValue = adcValue0;
    }

    if(channel == 1){
        res = ADC_convert(adc1, &adcValue1);
        adcValue = adcValue1;
    }
    //ADC_close(adc);
    return adcValue;
}

void vI2C_DAC(char channel,float valueFloat){
    uint16_t value;

    if(channel == 0){

        value = (uint16_t) valueFloat;

        txBuffer[0] = (value & 0xF00)>>8;
        txBuffer[1] = (value & 0xFF);

        i2cTransaction.slaveAddress = 0x60;
        i2cTransaction.writeBuf = txBuffer;
        i2cTransaction.writeCount = 2;
        i2cTransaction.readBuf = rxBuffer;
        i2cTransaction.readCount = 0;

        retVal = I2C_transfer(i2c, &i2cTransaction);

    }

    if(channel == 1){
        value = (uint16_t) valueFloat;

        txBuffer[0] = (value & 0xF00)>>8;
        txBuffer[1] = (value & 0xFF);

        i2cTransaction.slaveAddress = 0x61;
        i2cTransaction.writeBuf = txBuffer;
        i2cTransaction.writeCount = 2;
        i2cTransaction.readBuf = rxBuffer;
        i2cTransaction.readCount = 0;

        retVal = I2C_transfer(i2c, &i2cTransaction);
    }
}

//*****************************************************************************
//
//! \brief Application defined malloc failed hook
//!
//! \param  none
//!
//! \return none
//!
//*****************************************************************************
void vApplicationMallocFailedHook()
{
    /* Handle Memory Allocation Errors */
    while(1)
    {
    }
}

//*****************************************************************************
//
//! \brief Application defined stack overflow hook
//!
//! \param  none
//!
//! \return none
//!
//*****************************************************************************
void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName)
{
    //Handle FreeRTOS Stack Overflow
    while(1)
    {
    }
}
