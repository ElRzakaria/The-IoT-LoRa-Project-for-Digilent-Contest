/**
 * HYGRO library
 * author: Paul Leloup
 * update: 11-01-2019
 */

//Initialise le capteur
void initHYGRO(void);

//Lit 4 octets par I2C (2 premiers pour la temp�rature et 2 derniers pour l'hygrom�trie)
void readHYGRO();

//Calcule en envoie dans la trame la temperature
void processMessageTemperatureHYGRO(); //  unit� : �C

//Calcule en envoie dans la trame l'hygrometrie
void processMessageHumidityHYGRO(); //  unit� : %RH

