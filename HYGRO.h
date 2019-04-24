/**
 * HYGRO library
 * author: Paul Leloup
 * update: 11-01-2019
 */

//Initialise le capteur
void initHYGRO(void);

//Lit 4 octets par I2C (2 premiers pour la température et 2 derniers pour l'hygrométrie)
void readHYGRO();

//Calcule en envoie dans la trame la temperature
void processMessageTemperatureHYGRO(); //  unité : °C

//Calcule en envoie dans la trame l'hygrometrie
void processMessageHumidityHYGRO(); //  unité : %RH

