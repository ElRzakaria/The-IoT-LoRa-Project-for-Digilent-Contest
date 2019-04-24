/**
 * ALS library
 * author: Paul Leloup
 * update: 08-01-2019
 */

#define SS2 GPIOB.ODRbits.P8

//Initialise en lisant 16 octets par SPI
void initALS();

//Lire 16 octets par SPI
void readALS();

//Calcule en envoie dans la trame la lumiere
void processMessageALS(); //  unité : %Luminosité

