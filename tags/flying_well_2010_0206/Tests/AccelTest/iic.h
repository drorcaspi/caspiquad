//********************************************************************************************
//  QuadroControl II / XS - Universelle Mulitcoptersteuerung
//
//  (c) 2007-2009 by Tido Tebben. Alle Rechte vorbehalten.
//********************************************************************************************
//  http://www.tt-tronix.de
//********************************************************************************************
//  Datei:			iic.h - Header-Datei
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

#ifndef IIC_H
	#define IIC_H
	
	// Definitionen **
	#define I2C_CLOCK  			200000L

	//  Variablendeklaration **
	volatile extern	uint8_t 	iic_status;
			 extern	uint8_t 	motor;
	volatile extern	uint16_t	iic_timeout;

	//  Prototypen **
	void iic_init(void);
	void iic_write_byte (char byte); 

	//  Makros **

// I2C Start and Stop

#define IIC_START	TWCR = (1 << TWSTA) | (1 << TWEN)  | (1 << TWINT) | (1 << TWIE)
#define IIC_STOP 	TWCR = (1 << TWEN)  | (1 << TWSTO) | (1 << WINT)


#endif
