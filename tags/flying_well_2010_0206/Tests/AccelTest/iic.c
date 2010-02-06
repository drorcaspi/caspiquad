//********************************************************************************************
//  QuadroControl II / XS - Universelle Mulitcoptersteuerung
//
//  (c) 2007-2009 by Tido Tebben. Alle Rechte vorbehalten.
//********************************************************************************************
//  http://www.tt-tronix.de
//********************************************************************************************
//  Datei:			iic.c - Steuert Brushless Controller per IIC-Schnittstelle an
//********************************************************************************************
//
// Nutzungsbedingungen:
//
// Das Projekt QuadroControl II / XS  (inklusive Hardware, software, Quellcodes, Dokumentation, 
// HEX-Dateien) ist ausschlie�lich f�r den nicht - kommerziellen Einsatz zul�ssig. 
// Der kommerzielle Nachbau und/oder die kommerzielle Vervalueung der hier bereitgestellten 
// Informationen sind untersagt. 
//
// Eine kommerzielle Nutzung ist z.B. der Verkauf von QuadroControl II/XS Hardware oder 
// Baus�tzen, Verkauf von Quadrocoptern auf Basis der QuadroControl II/XS, Anfertigung und 
// Verkauf von Luftaufnahmen, etc.
//
// Die Portierung des Quellcodes auf andere Hardware als die von http://www.tt-tronix.de 
// bereitgestellte ist nur mit meiner ausdr�cklichen Erlaubnis gestattet.
//
// Die Verwendung und Ver�ffentlichung des Quellcodes oder der Dokumentation (auch auszugsweise) 
// in anderen Projekten und/oder auf anderen Webseiten oder anderen Medien unterliegt ebenfalls 
// diesen Nutzungsbedingungen. Die Seite "http://www.tt-tronix.de" muss dabei als Ursprung der 
// Information eindeutig genannt werden.
//
// Da bei der Entwicklung von Software und Hardware Fehler nie ganz ausgeschlossen werden k�nnen, 
// weise ich hiermit darauf hin, dass ich keinerlei Garantie f�r Sch�den, die durch den Nachbau 
// und den Gebrauch der QuadroControl II/XS und / oder der Dokumentation entstehen, 
// �bernehme. Weiterhin �bernehme ich keine Garantie f�r Folgesch�den, wie Personensch�den, 
// entgangene Gewinne, Verm�gensverluste oder anderer mittelbarer und unmittelbarer Sch�den, 
// die durch den Gebrauch oder die Nichtverwendbarkeit der QuadroControl II / XS , der 
// Quelltexte und/oder der Dokumentation entstehen. Dies gilt auch dann, wenn ich �ber die 
// M�glichkeit solcher Sch�den unterrichtet war oder bin. 
//
//********************************************************************************************
#include "quadrocontrol.h"

volatile	uint8_t 	iic_status = 0;
		 	uint8_t 	motor = 0;
volatile 	uint16_t	iic_timeout;

//********************************************************************************************
// Schreibt ein Byte auf die IIC-Schnittstelle
//********************************************************************************************
void iic_write_byte(char byte)
{ 
    TWSR = (0 << TWPS1) | (0 << TWPS0);  // Clear the status bit, prescaler = 1
    TWDR = byte;
    TWCR = (1 << TWINT) |   // Clear TWI Interrrup flag
           (1 << TWEN)  |   // Enable TWI
           (1 << TWIE);     // Enable TWI interrupt
}

//********************************************************************************************
// Initialisiert die IIC-Schnittstelle
//********************************************************************************************
void iic_init(void)
{
	IIC_STOP;
	TWCR  = (1 << TWINT);  // Clear TWI Interrrup flag
	TWAR  = 0;             // Clear TWI Slave Address Register (no slave operation required)
	TWAMR = 0;             // Clear TWI Slave Address Mask Register (no slave operation required)
	TWDR  = 0;             // Clear TWI Data Register

	TWSR = (0 << TWPS1) | (0 << TWPS0);  // Clear the status bit, prescaler = 1
  TWBR = ((F_CPU/I2C_CLOCK)-16)/2; 
	
	IIC_START;
	iic_write_byte(0);
	
	motor = 0;
	iic_timeout = 3;
}


//********************************************************************************************
// Interruptroutine f�r die Motorsteuerung
//********************************************************************************************
SIGNAL (TWI_vect)
{
  switch (iic_state)
	{
	  case 
		case 0:
			iic_write_byte(0x52+(2*motor));
			break;
		
		case 1:
			iic_write_byte(motor_iic[motor++]);
			break;
		
		case 2:
			IIC_STOP;
			
			if (motor < 4) 
				IIC_START;	
			else 
				motor = 0;

			iic_status = 0;
			iic_timeout = 3;
	}
}
