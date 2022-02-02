#include <xc.h>
#include <stdint.h>
#include <stdio.h>
#include "ADC.h"

#define _XTAL_FREQ 8000000

void ADC_Init(int frequency){
    //Configuracion del ADC
    // 8MHz --> Fosc/32 se recomienda con 2us
    ADCON1bits.ADFM = 0; //justificado a la izquierda
    ADCON1bits.VCFG0 = 0; //VDD 
    ADCON1bits.VCFG1 = 0; //tierra
    
    ADCON0bits.CHS = 5; //Canal 9 (AN9)
    ADCON0bits.ADON = 1; //change selection on 
    __delay_us(50);   //delay para que se cargue el capacitor en e modulo
    
    switch(frequency){     //Dependiendo de la frecuencia elegida se coloca un Fosc / x 
        case 1:
            ADCON0bits.ADCS = 0b00;
            break;
        case 4:
            ADCON0bits.ADCS = 0b01;
            break;
        case 8:
            ADCON0bits.ADCS = 0b10;
            break;
           
    }
}

void ADC_Change(){
    if (ADCON0bits.GO == 0) { // si esta en 0, revisa en qué canal está convirtiendo
        if (ADCON0bits.CHS == 6)  //si está en el 6, se cambia al 5
            ADCON0bits.CHS = 5;
        else 
            ADCON0bits.CHS = 6;
        __delay_us(50);
        ADCON0bits.GO =1;    //convertir
        }
}
