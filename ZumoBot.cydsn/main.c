#include <project.h>
#include <stdio.h>
#include "Motor.h"
#include "Ultra.h"
#include "Nunchuk.h"
#include "Reflectance.h"
#include "I2C_made.h"
#include "Gyro.h"
#include "Accel_magnet.h"
#include "IR.h"
#include "Ambient.h"
#include "Beep.h"
#include "math.h"

int rread(void);

int main()
{
    struct sensors_ ref;
    struct sensors_ dig;
    CyGlobalIntEnable; 
    UART_1_Start();
  
    int vase;
    int oikee;
    int speedmax = 254;
    double speedOtupla;
    double speedVtupla;
    int mustaviiva = 1;

    //starters
    
    reflectance_start();
    ADC_Battery_Start();
    motor_start();
    IR_led_Write(1);
    sensor_isr_StartEx(sensor_isr_handler);
    
    int16 adcresult =0;
    float volts = 0.0;
    
    printf("\nBoot\n");

    //BatteryLed_Write(1); // Switch led on 
    BatteryLed_Write(0); // Switch led off 
    //uint8 button;
    //button = SW1_Read(); // read SW1 on pSoC board
    
    
    
    /*for (;;){
        
            ADC_Battery_StartConvert();
            if(ADC_Battery_IsEndConversion(ADC_Battery_WAIT_FOR_RESULT)) {   // wait for get ADC converted value
                adcresult = ADC_Battery_GetResult16();
                volts = ADC_Battery_CountsTo_Volts(adcresult);                  // convert value to Volts
                float realvolts = volts * 1.5;
                // If you want to print value
                 //printf("%d %f\r\n",adcresult, realvolts);
            
                // jos volttimäärä pattereissa putoaa alle 4 niin pysäyttää moottorit

                

                if (realvolts< 4){
                    printf("AKKULOPPU\n");
                    motor_stop();
                }
            }
    }*/
    
    
    for(;;)
    {
        reflectance_read(&ref);
       // printf("%d %d %d %d \r\n", ref.l3-5735, ref.l1-4874, ref.r1-4816, ref.r3-7000);       //print out each period of reflectance sensors
        reflectance_digital(&dig);      //print out 0 or 1 according to results of reflectance period
       // printf("%d %d %d %d \r\n", dig.l3, dig.l1, dig.r1, dig.r3);       //print out 0 or 1 according to results of reflectance period
        
        CyDelay(5);
        
        // matikkaa
        oikee = ref.r1 + 500;
        vase = ref.l1 + 500;
        

        if (oikee<=11000){
            oikee=ref.r1/2;
        }
        if (vase<=11000){
            vase=ref.l1/2;
        }
        
        //toisen asteen yhtälöä käytetty jotta zumo osaisi kiihdyttää ja jarruttaa todella nätisti
        double speedO = ((0.391571*oikee*oikee)-(3.84672*oikee)-227.19)/1000000;
        double speedV = ((0.391571*vase*vase)-(3.84672*vase)-227.19)/1000000;
        
        int speedOtupla = (int)speedO;
        int speedVtupla = (int)speedV;

        
        // asettaa huippunopeuden päälle kun tietyt kriteerit täyttyvät
        if (speedO >=210){
            speedO=speedmax;
            printf("1");
        }
        if (speedV >=210){
            speedV=speedmax;
            printf("2");
        }
        if (speedV >=200 && speedO >=200){
            speedO=speedmax;
            speedV=speedmax;
        }
        
        
        //jos toisen moottorin nopeus laskee tietyn nopeuden alle niin lisää toiseen moottoriin nopeutta.
        if (oikee < 30) {
            speedV = 220+speedO;
            speedO = 0;
        }
        if (vase < 30) {
            speedO = 220+speedV;
            speedV = 0;
        }
        if (oikee < 80) {
            speedV = speedmax;
                    }
        if (vase < 80) {
            speedO = speedmax;
              
        }
        
        // hätätilanteisiin joissa robotti eksyy radalta, käytämme reunimmaisia antureita
        if (ref.r3 > 6000 && dig.l1 == 1 && dig.l3 == 1){
            speedO = speedmax;
            speedV = 15;
        }
        if (dig.r3 == 1 && dig.r1 == 1 && ref.l3 > 6000){
            speedV = speedmax;
            speedO = 15;
        }
        if (ref.l3 > 13000 && ref.l3 < 8000){
            motor_turboturnLeft(255,255);
        }
        if (ref.r3 > 13000 && ref.r3 < 8000){
            motor_turboturnRight(255,255);
        }
        
        // pääasiallinen ajaminen
             motor_turn(speedO,speedV,0);
            
        // viivoille pysähtyminen ja ohjaimella lähettäminen
        if(dig.l3 == 0 && dig.l1== 0 && dig.r1== 0 && dig.r3== 0){
        
            if(mustaviiva==1){
                motor_stop();
                wait_going_down();
                motor_start();
                motor_turn(speedO,speedV,0);
            }else if(mustaviiva == 3){
                motor_stop();
        }
        mustaviiva++;
        CyDelay(92);
    }

    }
    
}



#if 0
int rread(void)
{
    SC0_SetDriveMode(PIN_DM_STRONG);
    SC0_Write(1);
    CyDelayUs(10);
    SC0_SetDriveMode(PIN_DM_DIG_HIZ);
    Timer_1_Start();
    uint16_t start = Timer_1_ReadCounter();
    uint16_t end = 0;
    while(!(Timer_1_ReadStatusRegister() & Timer_1_STATUS_TC)) {
        if(SC0_Read() == 0 && end == 0) {
            end = Timer_1_ReadCounter();
        }
    }
    Timer_1_Stop();
    
    return (start - end);
}
#endif

/* Don't remove the functions below */
int _write(int file, char *ptr, int len)
{
    (void)file; /* Parameter is not used, suppress unused argument warning */
	int n;
	for(n = 0; n < len; n++) {
        if(*ptr == '\n') UART_1_PutChar('\r');
		UART_1_PutChar(*ptr++);
	}
	return len;
}

int _read (int file, char *ptr, int count)
{
    int chs = 0;
    char ch;
 
    (void)file; /* Parameter is not used, suppress unused argument warning */
    while(count > 0) {
        ch = UART_1_GetChar();
        if(ch != 0) {
            UART_1_PutChar(ch);
            chs++;
            if(ch == '\r') {
                ch = '\n';
                UART_1_PutChar(ch);
            }
            *ptr++ = ch;
            count--;
            if(ch == '\n') break;
        }
    }
    return chs;
}
/* [] END OF FILE */
