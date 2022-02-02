#include <xc.h>
#include <stdint.h>
#include <stdio.h>
#include "LCD.h"

#define _XTAL_FREQ 8000000

void Lcd_Port(char a){
    PORTD = a;
}

void Lcd_Cmd(char a){
    RS = 0;             // => RS = 0 - se va a hacer un comando
	Lcd_Port(a);
	EN  = 1;             // => E = 1 - para recibir el comando
    __delay_ms(4);
    EN  = 0;  
}

//Basicamente limpia la pantalla
void Lcd_Clear() {
	Lcd_Cmd(0);
	Lcd_Cmd(1);
}

//Basicamente manda los comandos dependiendo en qué línea está
void Lcd_Set_Cursor(char a, char b){
    char c;
    if(a == 1){
        c = 0x80 + b - 1;
        Lcd_Cmd(c);
       
    }
    else if (a == 2){
        c = 0xC0 + b - 1;
        Lcd_Cmd(c);
    }
}

void Lcd_Init(){
    //se inicializa el LCD / modo de 8 bits
    Lcd_Port(0x00);
    __delay_ms(20);
    Lcd_Cmd(0x30);
    __delay_ms(5);
    Lcd_Cmd(0x30);
    __delay_ms(11);
    Lcd_Cmd(0x30);
    ////////////////////////////////////////////////////////////////////////////
    Lcd_Cmd(0x38);       //Function set - modo de 8 bits, display en 2 líneas
    Lcd_Cmd(0x0C);       //Display on, Cursor off y Cursor blink off
    //Lcd_Cmd(0x08);
    Lcd_Cmd(0x01);       //se le hace clear al display
    //__delay_ms(11);
    Lcd_Cmd(0x06);       //Entry mode set - increment = 1 y Display shift off (= 0)
    //Lcd_Cmd(0x0C);
    
    __delay_ms(100);
}

//Función para mandar caractéres
void Lcd_Write_Char(char a){
   //char temp,y;
   //temp = a&0x0F;
   //y = a&0xF0;
   RS = 1;             // => RS = 1
   Lcd_Port(a);             //Data transfer
   EN = 1;
   __delay_ms(40);
   //EN = 0;
   //Lcd_Port(temp);
   //EN = 1;
   //__delay_us(40);
   EN = 0;
}

//Función para mostrar cadenas
void Lcd_Write_String(char *a)
{
	int i;
	for(i=0;a[i]!='\0';i++)
	   Lcd_Write_Char(a[i]);   //va mandando un dato seguido del otro
}

void Lcd_Shift_Right()
{
	Lcd_Cmd(0x01);
	Lcd_Cmd(0x0C);
}

void Lcd_Shift_Left()
{
	Lcd_Cmd(0x01);
	Lcd_Cmd(0x08);
}