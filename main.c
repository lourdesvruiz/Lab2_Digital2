/*
 * File:   main.c
 * Author: Lourdes Ruiz
 *
 * Created on Jan 30, 2022, 18:50 PM
 * 
 * Descripcion: 
 */


// PIC16F887 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT// Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF       // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is enabled)
#pragma config LVP = OFF         // Low Voltage Programming Enable bit (RB3/PGM pin has PGM function, low voltage programming enabled)

// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <stdint.h>
#include <stdio.h>
#include <pic16f887.h>
#include "LCD.h"
#include "ADC.h"
#include "USART.h"


// Define la frecuencia para poder usar los delays
#define _XTAL_FREQ 8000000

/*
 * Constantes
 */

/*
 * Variables 
 */

uint8_t centena, decena, unidad;
uint8_t centena2, decena2, unidad2;
uint8_t voldy = 0;
uint8_t voldy2 = 0;
uint8_t POT1 = 0;
uint8_t POT2 = 0;
uint8_t contador = 0;
float voltaje1, voltaje2;    //se utiliza float para poder obtener decimales
char values;
unsigned char sensores[5];
char flag;

/*
 * Prototipos de funciones
 */
void setup(void);
void conversion_volt(void);
void division(uint8_t counter);
void string2(char *str);
void give(char bit_cadena);

/*
 * Interrupcion 
 */

void __interrupt() isr (void)
{
    if(PIR1bits.ADIF) {
        if(ADCON0bits.CHS == 6)  //si esta en este canal, "poner" ls respuesta en la variable 
            POT1 = ADRESH;
        else
            POT2 = ADRESH;     //si no, que la despliegue el Puerto A
        PIR1bits.ADIF = 0;
    }
    
    // Contador del sensor 3
    if(RCIF){
        if (RCREG == '+'){
            contador++;
        }
        else if (RCREG == '-'){
            contador--;
        }
        else if (RCREG == 'a'){
            flag = 1;               //se prende la bandera para visualizar los voltajes en los primeros dos sensores
        }
        RCIF = 0;
        
    }
    
}

/*
 * Codigo Principal
 */

void main (void){
    
    setup();
    //unsigned int a;
    //Se inicializa el LCD
    TRISD = 0x00;
    Lcd_Clear();
    Lcd_Set_Cursor(1,1);
    Lcd_Write_String("S1:   S2:    S3:");
    
    while(1){
       //Cambio de canales de ADC
       ADC_Change();
      
       //5/255 = 0.01961
       voltaje1 = (POT1*0.01961);
       voltaje2 = (POT2*0.01961);
       division(contador); 
       
       //Se usa sprintf para pasar a ASCII los valores de los sensores
       //En la variable "sensores" (un array) se guarda el dato, tipo de dato, variable donde se encuentra el dato a traducir
       Lcd_Set_Cursor(2,1);
       sprintf(sensores, "%.2fV ", voltaje1); //se utiliza %.2f para que me dé los dos primeros dígitos despúes del punto
       Lcd_Write_String(sensores);
       sprintf(sensores, "%.2fV", voltaje2);
       Lcd_Write_String(sensores);
       sprintf(sensores, "  %d", centena);
       Lcd_Write_String(sensores);
       sprintf(sensores, "%d", decena);
       Lcd_Write_String(sensores);
       sprintf(sensores, "%d", unidad);
       Lcd_Write_String(sensores);
       
       if (flag == 1){
           flag = 0;
           //Muestra el valor de los dos sensores de voltaje en el monitor serial
           sprintf(sensores, "%.2f  ", voltaje1);
           string2("\r S1: ");
           string2(sensores);
           sprintf(sensores, "%.2f  ", voltaje2);
           string2("\r S2: ");
           string2(sensores);
       }
       
       
  }
    
}


/*
 * Funciones
 */

void setup(void){
    // Configuraciones de entradas y salidas 
    ANSEL = 0b0110000;  //RE0 y RE1 como entradas analógicas para ADC 
    ANSELH = 0;
    
    TRISA = 0;
    
    TRISD = 0;
    TRISB = 0;
    TRISE = 0b111;
    
    //valores iniciales
    PORTB = 0;
    PORTA = 0;
    
    PORTD = 0;
    PORTE = 0;
    
     //Configuracion de oscilador
    OSCCONbits.IRCF = 0b0111; //8MHz
    OSCCONbits.SCS = 1; //ocsilador interno
    
    ADC_Init(8);
    
    USART_Init(8);
    
    Lcd_Init();
    
    //Configuracion de las interrupciones
    PIR1bits.RCIF = 0; //apagar la bandera de UART
    PIE1bits.RCIE = 1; //habilitar interrupcion 
    PIR1bits.ADIF = 0; //apagar la bandera de ADC
    PIE1bits.ADIE = 1; //habilitar interrupcion analogica
    INTCONbits.PEIE = 1; //interrupciones perifericas
    INTCONbits.GIE  = 1;
   
    return;
}

void conversion_volt(void)
//% es el operador modulo, el cual produce el residuo de una division
{    // conversion a voltaje 
     voldy = (POT1<<2)+4;  //se rota a la izquierda 2 veces para multiplicar por 4 y se suma 4 para llegar a 1024
     voldy2 = voldy>>1; // se rota 1 vez a la derecha para dividir dentro de 2 y tener voltaje de 0.02 a 5.12
     centena2 = ((voldy2/100));
     decena2 = ((voldy2/10)%10);
     unidad2 = (voldy2%10);
     
    
}

void give(char bit_cadena)
{
    //while(TXSTAbits.TRMT ==0);
    while (PIR1bits.TXIF ==0); //se espera a que el registro del transmisor este vacio
        TXREG = bit_cadena;    //le envia el dato a cada "bit" de la cadena. 
           
}

void string2(char *str)  //regresa un pointer hacia un caracter
{
    while (*str != '\0')  //siempre y cuando el pointer sea distinto a caracter nulo (Que no se haya terminado la cadena)
    {
        give(*str);       //la funcion de give ira tomando caracter por caracter
        str++;            //aumentando para ir mandando bit por bit 
                
    }
}

void division(uint8_t counter)
//% es el operador modulo, el cual produce el residuo de una division
{
     centena = ((counter/100));
     decena = ((counter/10)%10);
     unidad = (counter%10);
     
     return;
}
