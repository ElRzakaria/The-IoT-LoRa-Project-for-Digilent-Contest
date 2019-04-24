/**
 * LoRa library
 * author: Paul Leloup
 * update: 12-11-2018
 */

//Constantes


//PMODHAUTLORA
#define SS1 GPIOB.ODRbits.P2
#define RESET1 GPIOB.ODRbits.P0
#define TXLORA1 GPIOD.ODRbits.P14
#define RXLORA1 GPIOD.ODRbits.P13
#define DIO0LORA1 GPIOD.IDRbits.P15
#define TIMEOUT 2000

//LORA1

void testLoRa(unsigned char reg);

//Initialise le module Lora
void initLoRa();

//Reset le module LoRa
void resetLoRa();

//Met le mode LoRa
void modeLoRa();

//DIO Mapping Lora Mode
void DIOMappingLoRa(int DIOMAP);

//Met le CRC à ON
void CRCLoRa(int CRC);

//Met à ON frequency hopping
void frequencyHoppingLoRa(int FH);

//Definit le spreading factor
void SpreadingFactorLoRa(int SF);

//Definit les paramètres de BOOST
void PowerAmplifierSettingLoRa();

//Renvoie la longueur de la chaine
int lengthLora(unsigned char *Package);

//Envoie la donnée Package de longueur packageLength
void SendPackageLoRa(unsigned char *Package, int packageLength);

//Recoit la donnée Package de longueur packagelength
void ReceivePackageLoRa(char *Package);

//Définit l'interruption des registres de reception
void SetInterruptLora1();

//Envoie un unsigned char tableau
void sendTabLora1(unsigned char *messageTab);

