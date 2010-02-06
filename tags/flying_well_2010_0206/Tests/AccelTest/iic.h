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
// HEX-Dateien) ist ausschließlich für den nicht - kommerziellen Einsatz zulässig. 
// Der kommerzielle Nachbau und/oder die kommerzielle Vervalueung der hier bereitgestellten 
// Informationen sind untersagt. 
//
// Eine kommerzielle Nutzung ist z.B. der Verkauf von QuadroControl II/XS Hardware oder 
// Bausätzen, Verkauf von Quadrocoptern auf Basis der QuadroControl II/XS, Anfertigung und 
// Verkauf von Luftaufnahmen, etc.
//
// Die Portierung des Quellcodes auf andere Hardware als die von http://www.tt-tronix.de 
// bereitgestellte ist nur mit meiner ausdrücklichen Erlaubnis gestattet.
//
// Die Verwendung und Veröffentlichung des Quellcodes oder der Dokumentation (auch auszugsweise) 
// in anderen Projekten und/oder auf anderen Webseiten oder anderen Medien unterliegt ebenfalls 
// diesen Nutzungsbedingungen. Die Seite "http://www.tt-tronix.de" muss dabei als Ursprung der 
// Information eindeutig genannt werden.
//
// Da bei der Entwicklung von Software und Hardware Fehler nie ganz ausgeschlossen werden können, 
// weise ich hiermit darauf hin, dass ich keinerlei Garantie für Schäden, die durch den Nachbau 
// und den Gebrauch der QuadroControl II/XS und / oder der Dokumentation entstehen, 
// übernehme. Weiterhin übernehme ich keine Garantie für Folgeschäden, wie Personenschäden, 
// entgangene Gewinne, Vermögensverluste oder anderer mittelbarer und unmittelbarer Schäden, 
// die durch den Gebrauch oder die Nichtverwendbarkeit der QuadroControl II / XS , der 
// Quelltexte und/oder der Dokumentation entstehen. Dies gilt auch dann, wenn ich über die 
// Möglichkeit solcher Schäden unterrichtet war oder bin. 
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
