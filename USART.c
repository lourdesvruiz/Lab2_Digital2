#include <xc.h>
#include <stdint.h>
#include <stdio.h>
#include "USART.h"

#define _XTAL_FREQ 8000000

void USART_Init(int frequency){
    
    TXSTAbits.SYNC = 0;
    RCSTAbits.SPEN = 1; //enable
    RCSTAbits.RX9 = 0;  //no comunicarme en 9 bits (modo de 8 bits)
    RCSTAbits.CREN = 1; //habilitamos la recepcion
    TXSTAbits.TXEN = 1; //habilitamos la transmision
    TXIF = 0;
    TXIE = 0;
    TXSTAbits.TX9 = 0; //no comunicarme en 9 bits
    RCSTAbits.FERR = 0;
    RCSTAbits.OERR = 0;
    
    //Configuraciones para un baudrate de 9600 dependiendo de la frecuencia
    switch(frequency){
        case 4: 
            TXSTAbits.BRGH = 1;
            BAUDCTLbits.BRG16 = 0;
            SPBRG = 25;
            SPBRGH = 0;
            break;
        case 8:
            TXSTAbits.BRGH = 0;
            BAUDCTLbits.BRG16 = 0;
            SPBRG = 12;
            SPBRGH = 0;
            break;
    }
}
