#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
//Basic Information
#define BAUD_RATE 9600
//LCD
#define LCD_WIDTH 20
#define LCD_HEIGHT 4
#define RS 16
#define E 17
#define D4 18
#define D5 19
#define D6 20
#define D7 21
#define V0 22
//EXINT
#define EXINT 15
//ADC
#define ADC_0 26
#define ADC_1 27
#define ADC_BUFFER_SIZE 10
typedef enum {
    STATUS_WAITING = 0,
    STATUS_MEASURING,
    STATUS_DISPLAY
}DisplayState_t;
typedef struct{
    
    unsigned int rawEC;
    unsigned int rawTemp;
    unsigned int adcEC[ADC_BUFFER_SIZE+1];
    unsigned int adcTemp[ADC_BUFFER_SIZE+1]; 
    double Celsius;
    double ecVal;
    char buffer[ADC_BUFFER_SIZE];
    
    double last_ecVal;
    double last_Celsius;
}adcData_t;
volatile DisplayState_t DisplayState;
volatile bool StateChanged;
volatile bool TimeUp;
volatile unsigned short BlinkStatus;
volatile unsigned short MeaCount;
volatile bool adc_ready;
//ADC
adcData_t adcData;
const double TempStep = 0.04;
const double ECStep = 0.0056;
const double TempCompens = 13;
/*NOTE*/
/*
The Pico has a 12-bit ADC (ENOB of 8.7-bit, see https://datasheets.raspberrypi.com/rp2040/rp2040-datasheet.pdf[RP2040 datasheet section 4.9.3 for more details]),
meaning that a read operation will return a number ranging from 0 to 4095 (2^12 - 1) for a total of 4096 possible values.
Therefore, the resolution of the ADC is 3.3/4096, so roughly steps of 0.8 millivolts.
According to the ec-meter manual, the temperature could get from a voltage value that is temp=50*v-13.
Similiarly, the ec value could be obtained that is ec=7*v.  
*/

//Delay
const int DELAY_1US = 1;
const int DELAY_100US = 100;
const int DELAY_500US = 500;
const int DELAY_2MS = 2;
const int DELAY_50MS = 50;
const int DELAY_100MS = 100;
const int DELAY_500MS = 500;
const int DELAY_1000MS = 1000;
const int DELAY_2000MS = 2000;
//Basic LCD Command
const int LCD_CLEARDISPLAY = 0x01;
const int LCD_RETURNHOME = 0x02;
const int LCD_ENTRYMODESET = 0x04;
const int LCD_DISPLAYCONTROL = 0x08;
const int LCD_CURSORSHIFT = 0x10;
const int LCD_FUNCTIONSET = 0x20;
const int LCD_SETCGRAMADDR = 0x40;
const int LCD_SETDDRAMADDR = 0x80;
// flags for display entry mode
const int LCD_ENTRYSHIFTINCREMENT = 0x01;
const int LCD_ENTRYLEFT = 0x02;
// flags for display and cursor control
const int LCD_BLINKON = 0x01;
const int LCD_CURSORON = 0x02;
const int LCD_DISPLAYON = 0x04;
// flags for display and cursor shift
const int LCD_MOVERIGHT = 0x04;
const int LCD_DISPLAYMOVE = 0x08;
// flags for function set
const int LCD_5x10DOTS = 0x04;
const int LCD_2LINE = 0x08;
const int LCD_8BITMODE = 0x10;
// flag for backlight control
const int LCD_BACKLIGHT = 0x08;
const int LCD_ENABLE_BIT = 0x04;
//DDRAM Address
const uint8_t Lcd_DDRAM[4] = {0x00, 0x40, 0x14, 0x54};
/*-----------------Funtion declearation-----------------*/
//LCD Config
void Lcd_Send4Bit(uint8_t data);
void Lcd_Send8Bit(uint8_t data);
void Lcd_Command(uint8_t Command);
void Lcd_Data(uint8_t data);
void Lcd_Init(void);
void Lcd_Init(void);
void Lcd_SetCursor(uint8_t row, uint8_t column);
void Lcd_Print(const char *str);
void Lcd_Disp(const char *str, uint8_t row, uint8_t column);
//LCD Display
void Display_Status0(void);
void Display_Status1(void);
void TransAnime(void);
void Display_Status2(void);
//External Interrupt
void GPIO_Callback0(uint gpio, uint32_t event);
void EXINT_Init(void);
//Timer
int64_t alarm_callback(alarm_id_t id, __unused void *user_data);
int64_t adc_timer_callback(alarm_id_t id, __unused void *user_data);
bool repeating_timer_callback(__unused struct repeating_timer *t);
//ADC
void ADC_Init(void);
void ADC_Get(void);
void Measure(void);
void Data_Get(void);
void Data_Display(void);
//Timer
repeating_timer_t timer;
bool cancelled;
//Initialization
void Pico_Init(void);
void Data_Init(void);
//Menu
void Menu(void);
void (*Display_Function[3])(void) = {Display_Status0, Display_Status1, Display_Status2};
/*-----------------Funtion implementation-----------------*/
/*-----------------LCD Config-----------------*/
void Lcd_Send4Bit(uint8_t data){
    gpio_put(E, 1);
    data = data & 0x0F;
    gpio_put(D4, data & 0x01);
    gpio_put(D5, (data >> 1) & 0x01);
    gpio_put(D6, (data >> 2) & 0x01);
    gpio_put(D7, (data >> 3) & 0x01);
    sleep_us(DELAY_100US);
    gpio_put(E, 0);
}
void Lcd_Send8Bit(uint8_t data){
    sleep_us(DELAY_1US);
    Lcd_Send4Bit(data >> 4);
    sleep_us(DELAY_1US);
    Lcd_Send4Bit(data & 0x0F);
}
void Lcd_Command(uint8_t Command){
    gpio_put(RS, 0);
    Lcd_Send8Bit(Command);
}
void Lcd_Data(uint8_t data){
    gpio_put(RS, 1);
    Lcd_Send8Bit(data);
}
void Lcd_Init(void){
    gpio_set_dir(RS, GPIO_OUT);
    gpio_set_dir(E, GPIO_OUT);
    gpio_set_dir(D4, GPIO_OUT);
    gpio_set_dir(D5, GPIO_OUT);
    gpio_set_dir(D6, GPIO_OUT);
    gpio_set_dir(D7, GPIO_OUT);
    gpio_put(RS, 0);
    gpio_put(E, 0);
    sleep_ms(DELAY_50MS);
    Lcd_Send4Bit(0x03);  //Function set: 0x03 8BitMode
    sleep_us(DELAY_500US);
    Lcd_Send4Bit(0x03);  //Function set: 0x03 8BitMode
    sleep_us(DELAY_500US);
    Lcd_Send4Bit(0x03);  //Function set: 0x03 8BitMode
    sleep_us(DELAY_500US);
    Lcd_Send4Bit(0x02);  //4Bit mode: 0x02 4BitMode
    sleep_us(DELAY_500US);
    Lcd_Command(LCD_ENTRYMODESET | LCD_ENTRYLEFT);  //Entry Mode set: 0x06
    sleep_us(DELAY_500US);
    Lcd_Command(LCD_FUNCTIONSET | LCD_2LINE);  //Function set: 0x28
    sleep_us(DELAY_500US);
    Lcd_Command(LCD_DISPLAYCONTROL | LCD_DISPLAYON);    //Display set: 0x0C
    sleep_us(DELAY_500US);
}
void Lcd_SetCursor(uint8_t row, uint8_t column){
    uint8_t adrs = Lcd_DDRAM[row] + column;
    Lcd_Command(0x80 | adrs);
}
void Lcd_Print(const char *str){
    while(*str){
        Lcd_Data(*str++);
    }
}
void Lcd_Disp(const char *str, uint8_t row, uint8_t column){
    int Str_Length = 0;
    while(str[Str_Length] != '\0'){
        Str_Length++;
    }
    if ((LCD_WIDTH-column-1) < Str_Length)
        return;
    Lcd_SetCursor(row, column);
    while(*str){
        Lcd_Data(*str++);
    }
}
/*-----------------LCD Display-----------------*/
void Display_Status0(void){
//Wait for pressing button to start
    if(StateChanged == true){
        Lcd_Command(LCD_CLEARDISPLAY);
        sleep_ms(DELAY_2MS);
        Data_Init();
        // cancel_repeating_timer(&timer);
    }
    switch(BlinkStatus){
        case 0:
            Lcd_SetCursor(0, 0);//Line 1
            for(int i=0; i < LCD_WIDTH; i++)
                Lcd_Data(0xFF);
            Lcd_SetCursor(1, 0);//Line 2
            Lcd_Print("Press Button to");
            Lcd_SetCursor(3, 0);// Line 4
            for(int i=0; i < LCD_WIDTH; i++)
                Lcd_Data(0xFF);
            Lcd_SetCursor(2, 6);//Line 3
            Lcd_Data(0x7E);
            if(true){
                BlinkStatus++;
                TimeUp = false;
                add_alarm_in_ms(1000, alarm_callback, NULL, false);
            }
            break;
        case 1:
            Lcd_SetCursor(2, 7);
            Lcd_Print("START!");
            if(TimeUp == true){
                BlinkStatus++;
                TimeUp = false;
                add_alarm_in_ms(1000, alarm_callback, NULL, false);
            }
            break;
        case 2:
            Lcd_SetCursor(2, 7);
            Lcd_Print("      ");
            if(TimeUp == true){
                BlinkStatus = 1;
                TimeUp = false;
                add_alarm_in_ms(1000, alarm_callback, NULL, false);
            }
        default:
            break;
    }
}
void Display_Status1(void){
//performing measurement
    if(StateChanged == true){
        TransAnime();
        Lcd_Command(LCD_CLEARDISPLAY);
        sleep_ms(DELAY_2MS);
        StateChanged = 0;
    }
    for(int i = 0; i<3; i++){
        if(i == 0){
            Lcd_SetCursor(0, 0);//Line 1
            for(int i=0; i < LCD_WIDTH; i++)
                Lcd_Data(0xFF);
            Lcd_SetCursor(3, 0);// Line 4
            for(int i=0; i < LCD_WIDTH; i++)
                Lcd_Data(0xFF);
            Lcd_SetCursor(2, 3);//Line 2
            Lcd_Print("Measuring");
            //animation and measurement
            Lcd_SetCursor(2, 12);
            Measure();
            Lcd_Print(".");
            Measure();
            Lcd_Print(".");
            Measure();
            Lcd_Print(".");
            Lcd_SetCursor(0, 0);
            Lcd_Print("  ");
            Lcd_SetCursor(3, 0);
            Lcd_Print("  ");
            Measure();
            Lcd_SetCursor(2, 12);
            Lcd_Print("   ");
            Lcd_SetCursor(0, 2);
            Lcd_Print("  ");
            Lcd_SetCursor(3, 2);
            Lcd_Print("  ");
        }
        else if(i == 1){
            //animation and measurement
            Lcd_SetCursor(2, 12);
            Measure();
            Lcd_Print(".");
            Lcd_SetCursor(0, 4);
            Lcd_Print("  ");
            Lcd_SetCursor(3, 4);
            Lcd_Print("  ");
            Measure();
            Lcd_SetCursor(2, 13);
            Lcd_Print(".");
            Lcd_SetCursor(0, 6);
            Lcd_Print("  ");
            Lcd_SetCursor(3, 6);
            Lcd_Print("  ");
            Measure();
            Lcd_SetCursor(2, 14);
            Lcd_Print(".");
            Lcd_SetCursor(0, 8);
            Lcd_Print("  ");
            Lcd_SetCursor(3, 8);
            Lcd_Print("  ");
            Measure();
            Lcd_SetCursor(2, 12);
            Lcd_Print("   ");
            Lcd_SetCursor(0, 10);
            Lcd_Print("  ");
            Lcd_SetCursor(3, 10);
            Lcd_Print("  ");
        }
        else{
            DisplayState = STATUS_DISPLAY;
            StateChanged = true;
            //animation and measurement
            Lcd_SetCursor(2, 12);
            Measure();
            Lcd_Print(".");
            Lcd_SetCursor(0, 12);
            Lcd_Print("  ");
            Lcd_SetCursor(3, 12);
            Lcd_Print("  ");
            Measure();
            Lcd_SetCursor(2, 13);
            Lcd_Print(".");
            Lcd_SetCursor(0, 14);
            Lcd_Print("  ");
            Lcd_SetCursor(3, 14);
            Lcd_Print("  ");
            sleep_ms(DELAY_500MS);
            Lcd_SetCursor(2, 14);
            Lcd_Print(".");
            Lcd_SetCursor(0, 16);
            Lcd_Print("  ");
            Lcd_SetCursor(3, 16);
            Lcd_Print("  ");
            sleep_ms(DELAY_500MS);
            Lcd_SetCursor(2, 12);
            Lcd_Print("   ");
            Lcd_SetCursor(0, 16);
            Lcd_Print("  ");
            Lcd_SetCursor(3, 16);
            Lcd_Print("  ");
            sleep_ms(DELAY_100MS);
            Lcd_SetCursor(0, 18);
            Lcd_Print("  ");
            Lcd_SetCursor(3, 18);
            Lcd_Print("  ");
            sleep_ms(DELAY_500MS);
        }
    }
    
}
void Display_Status2(void){
    if(StateChanged == true){
            Lcd_Command(LCD_CLEARDISPLAY);
            sleep_ms(DELAY_2MS);
            Lcd_SetCursor(0, 0);//Line 1
            Lcd_Print("EC: ");
            Lcd_SetCursor(1, 0);// Line 2
            Lcd_Print("Temp: ");
            BlinkStatus = 0;
            StateChanged = 0;
            TimeUp = false;
            add_alarm_in_ms(2000, alarm_callback, NULL, false);
        }

    switch(BlinkStatus){
        case 0:
            Lcd_SetCursor(3, 9);// Line 4
            Lcd_Print("Soil Meter.");
            if(TimeUp == true){
                BlinkStatus++;
                TimeUp = false;
                add_alarm_in_ms(500, alarm_callback, NULL, false);
            }
            break;
        case 1:
            Lcd_SetCursor(3, 9);
            Lcd_Print("           ");
            if(TimeUp == true){
                BlinkStatus++;
                TimeUp = false;
                add_alarm_in_ms(2000, alarm_callback, NULL, false);
            }
            break;
        case 2:
            Lcd_SetCursor(3, 5);
            Lcd_Print("BlackJack Prj.");
            if(TimeUp == true){
                BlinkStatus++;
                TimeUp = false;
                add_alarm_in_ms(500, alarm_callback, NULL, false);
            }
            break;
        case 3:
            Lcd_SetCursor(3, 5);
            Lcd_Print("              ");
            if(TimeUp == true){
                BlinkStatus = 0;
                TimeUp = false;
                add_alarm_in_ms(2000, alarm_callback, NULL, false);
            }
            break;
        default:
            break;
    }
    Data_Get();
    Data_Display();
}
void TransAnime(void){
    //An animation of status 0 transform to status 1
    for(int i = 0; i < 5; i++){
        Lcd_SetCursor(2, 7);
        Lcd_Print("START!");
        sleep_ms(250-(DELAY_50MS*i));
        Lcd_SetCursor(2, 7);
        Lcd_Print("      ");
        sleep_ms(250-(DELAY_50MS*i));
    }
}
/*-----------------External Interrupt-----------------*/
void GPIO_Callback0(uint gpio, uint32_t event){
    static uint64_t last_interrupt_time = 0;
    uint64_t current_time = time_us_64();//button debouncing
    if(gpio == EXINT && (current_time - last_interrupt_time)>200000){
        if(DisplayState == STATUS_WAITING && StateChanged == 0){
            DisplayState = STATUS_MEASURING;
            StateChanged = 1;
        }
        else if(DisplayState == STATUS_DISPLAY && StateChanged == 0){
            DisplayState = STATUS_WAITING;
            StateChanged = 1;
        }               
        last_interrupt_time = current_time;
        TimeUp = false;
    }
}
void EXINT_Init(void){
    gpio_set_dir(EXINT, GPIO_IN);
    gpio_set_irq_enabled_with_callback(EXINT, GPIO_IRQ_EDGE_FALL, true, &GPIO_Callback0);
}
/*-----------------Timer-----------------*/
int64_t alarm_callback(alarm_id_t id, __unused void *user_data) {
    TimeUp = true;
    return 0;
}
int64_t adc_timer_callback(alarm_id_t id, void *user_data){
    adc_ready = true;
    return 0;
}
bool repeating_timer_callback(__unused struct repeating_timer *t){
    static unsigned short count = 0;
    if(DisplayState == STATUS_DISPLAY){
        ADC_Get();
        adcData.adcEC[10] += adcData.rawEC;
        adcData.adcTemp[10] += adcData.rawTemp;
        count++;  
        if(count>4){
            count = 0;
            adcData.adcEC[10] = adcData.adcEC[10]/5;
            adcData.adcTemp[10] = adcData.adcTemp[10]/5;
            for(int i=0; i<10; i++){
                adcData.adcEC[i] = adcData.adcEC[i+1];
                adcData.adcTemp[i] = adcData.adcTemp[i+1];
            }
            adcData.adcEC[10] = 0;
            adcData.adcTemp[10] = 0;
        }
    }
}
/*-----------------ADC-----------------*/
void ADC_Init(void){
    adc_init();
    adc_gpio_init(ADC_0);
    adc_gpio_init(ADC_1);
    //real time getting data
    if(adcData.last_ecVal == -1 && adcData.last_Celsius == -1){
    add_repeating_timer_ms(500, repeating_timer_callback, NULL, &timer);     
        
    }
}
//ADC collecting
void ADC_Get(void){
    adc_select_input(0);
    adcData.rawEC = adc_read();
    adc_select_input(1);
    adcData.rawTemp = adc_read();
}
//ADC get per 100ms
void Measure(void){
    
    for(int i = 0; i < 5; i++){
        adc_ready = false;
        add_alarm_in_ms(100, adc_timer_callback, NULL, false);
        while(!adc_ready){
            tight_loop_contents();
        }
        ADC_Get();
        adcData.adcEC[MeaCount] += adcData.rawEC;
        adcData.adcTemp[MeaCount] += adcData.rawTemp;
        
    }
    
    adcData.adcEC[MeaCount] = adcData.adcEC[MeaCount] / 5;
    adcData.adcTemp[MeaCount] = adcData.adcTemp[MeaCount] / 5;
    //ensure array index doesn't overflow
    MeaCount = (MeaCount + 1) % ADC_BUFFER_SIZE;
}
//ADC to decimal
//EC and Temperature display
void Data_Get(void){
    adcData.rawEC = 0;
    adcData.rawTemp = 0;
    for(int i = 0; i < ADC_BUFFER_SIZE; i++){
        adcData.rawEC += adcData.adcEC[i];
        adcData.rawTemp += adcData.adcTemp[i];
    }
    adcData.ecVal = (double)adcData.rawEC / ADC_BUFFER_SIZE;
    adcData.Celsius = (double)adcData.rawTemp / ADC_BUFFER_SIZE;
}
//Display measurement result
void Data_Display(void){
    adcData.ecVal = ECStep*adcData.ecVal;
    adcData.Celsius = (TempStep*adcData.Celsius)-TempCompens;
    //ec value limitation
    //0.00~7.00
    if(adcData.ecVal > 7.00)
        adcData.ecVal = 7.00;
    else if(adcData.ecVal < 0.00)
        adcData.ecVal = 0.00;
    //temperature value limitation
    //-10.00~50.00
    if(adcData.Celsius > 50.00)
        adcData.Celsius = 50.00;
    else if(adcData.Celsius < -10.00)
        adcData.Celsius = -10.00;
    //EC value display
    if(fabs(adcData.ecVal-adcData.last_ecVal) > 0.1){
        //update if value changed
        sprintf(adcData.buffer, "%.2lf mS/cm", adcData.ecVal);
        Lcd_SetCursor(0, 5);//Line 1
        Lcd_Print(adcData.buffer);
        adcData.last_ecVal = adcData.ecVal;
    }

    //Temp display
    if(fabs(adcData.Celsius-adcData.last_Celsius) > 0.01){
        sprintf(adcData.buffer, "%.2lf ", adcData.Celsius);
        Lcd_SetCursor(1, 6);//Line 2
        Lcd_Print(adcData.buffer);
        Lcd_Data(0xDF); //degree symbol
        Lcd_Print("C ");
        adcData.last_Celsius = adcData.Celsius;
    }

}
/*-----------------Initialization-----------------*/
void Pico_Init(void){
    stdio_init_all(); 
    //LCD
    gpio_init(RS);
    gpio_init(E);
    gpio_init(D4);
    gpio_init(D5);
    gpio_init(D6);
    gpio_init(D7);
    gpio_init(V0);
    gpio_set_dir(V0, GPIO_OUT);
    gpio_put(V0, 1);
    //Interrupt
    gpio_init(EXINT);
}
void Data_Init(void){
    DisplayState = STATUS_WAITING;
    StateChanged = 0;
    MeaCount = 0;
    TimeUp = false;
    BlinkStatus = 0;
    adc_ready = false;
    memset(adcData.adcEC, 0, sizeof(adcData.adcEC));
    memset(adcData.adcTemp, 0, sizeof(adcData.adcTemp));
    adcData.rawEC = 0;
    adcData.rawTemp = 0;
    adcData.Celsius = 0;
    adcData.ecVal = 0;
    adcData.last_ecVal = -1;
    adcData.last_Celsius = -1;
}
/*-----------------Menu-----------------*/
void Menu(void){
    if(DisplayState < 3){
        Display_Function[DisplayState]();
    }
 
}
//-------------main-------------
int main(void)
{
    Pico_Init();
    Data_Init(); 
    Lcd_Init();
    EXINT_Init();
    ADC_Init();

    while(1){
        Menu();        
    }
    return 0;
}
