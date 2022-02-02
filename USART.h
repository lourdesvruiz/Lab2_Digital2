/* 
 * File:   USART.h
 * Author: Lourdes Ruiz
 * Comments:
 * Revision history: 
 */

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef USART_H
#define	USART_H

#include <xc.h> // include processor files - each processor file is guarded.  

#ifndef _XTAL_FREQ
#define _XTAL_FREQ 8000000
#endif

/*
 * Prototipos de funciones
 */

void USART_Init(int frequency);

#endif


