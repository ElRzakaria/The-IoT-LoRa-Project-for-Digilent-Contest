/**
 * GPS library
 * author: Paul Leloup
 * update: 15-10-2018
 */

#include "system.h"
#include "GPS.h"

#pragma message("Compiling functions for GPS")



int initGPS(void)
{
	step=0;
	return 0;
}
void readGPS(void){

	char c;
	x=0;
	while(UART2_IsRxNotEmpty())
	{
	c = UART2_Read();
	buffer[x]=c;
	x++;
	}
	buffer[x]='\0';
}
int tramTabRecupGPS(void)
{
	char c;
	int j=0;
	while(j!=x)
	{
	c = buffer[j];
	j++;
	switch (step)
			{
				case 0:
					if ((c == '$') && (end == 0)){
						bufferTram[i++] = c;

						step = 4;
					}
					break;
				case 4:
					bufferTram[i++] = c;
					step = 5;
					break;
				case 5:
					bufferTram[i++] = c;
					step = 6;
					break;
				case 6:
					bufferTram[i++] = c;
					step = 7;
					break;
				case 7:
					bufferTram[i++] = c;
					step = 1;
					break;
				case 1:
					if (c != 'C')
						{
							step = 0;
							i = 0;
							break;
						}
					bufferTram[i++] = c;
					step = 2;
					break;

				case 2:
					if (c == '*')
						{
							bufferTram[i++] = c;
							step = 3;
							break;
						}

					bufferTram[i++] = c;
					break;

				case 3:

					bufferTram[i] = '\0';
					end = 1;
					step = 0;
					break;
			}
	}
	return 0;
}

int printTramGPS(void)
{
	int j;
	//affichage tableau
	if(end==1)
	{
		printf("Tram printed : \n");
		for(j =0; j <i; j++){
			UART1_Send(bufferTram[j]);
		}
		printf("\n\n");
	}
	return 0;
}

int processMessageGPS(void)
{
	int j=0,n=0, virgule =0;
	char tmp;

	tramTabRecupGPS();

	if(bufferMessage[0]=='#'){
			while(bufferMessage[n] != '*'){
				n++;
			}
			if(end==1)
				{
					while(bufferTram[j]!='\0')
					{
						if(bufferTram[j]==',')
						{
							virgule++;
						}
						//heure
						if(virgule==1)
						{
							j++;
							bufferMessage[n++]=',';
							bufferMessage[n++]='H';
							for(int m = 0 ; m < 6 ; m++){
								if(n==2){
									tmp=bufferTram[j++];
									//tmp=tmp+1; //décalage horaire de 1
									bufferMessage[n++]=tmp;
								}else{
									bufferMessage[n++]=bufferTram[j++];
								}
							}
							bufferMessage[n++]=',';
							j += 3;
						}
						//latitude
						if(virgule==3)
						{
							if(bufferTram[j+1]!=',')
							{
								j++;
								bufferMessage[n++]='L';
								bufferMessage[n++]='A';
								bufferMessage[n++]='T';
								for(int m = 0 ; m < 9 ; m++){
									bufferMessage[n++]=bufferTram[j++];
								}
								j--;
							}
						}
						//N/S indicator
						if(virgule==4)
						{
							if(bufferTram[j+1]!=',')
							{
								j++;
								bufferMessage[n++]=bufferTram[j];
							}
							bufferMessage[n++]=',';
						}
						//longitude
						if(virgule==5)
						{
							if(bufferTram[j+1]!=',')
							{
								j++;
								bufferMessage[n++]='L';
								bufferMessage[n++]='O';
								bufferMessage[n++]='N';
								for(int m = 0 ; m < 10 ; m++){
									bufferMessage[n++]=bufferTram[j++];
								}
								j--;
							}
						}
						//E/W indicator
						if(virgule==6)
						{
							if(bufferTram[j+1]!=',')
							{
								j++;
								bufferMessage[n++]=bufferTram[j];
							}
							bufferMessage[n++]=',';
						}
						if(virgule==9)
						{
							j++;
							bufferMessage[n++]='D';
							for(int m = 0 ; m < 6 ; m++){
								bufferMessage[n++]=bufferTram[j++];
							}
							bufferMessage[n++]='*';
							bufferMessage[n++]='\0';
						}
						j++;
					}
					cleanTramGPS();
				}
	}else
	{
		bufferMessage[n++]='#';
		//Trier la trame
		if(end==1)
		{
			while(bufferTram[j]!='\0')
			{
				if(bufferTram[j]==',')
				{
					virgule++;
				}
				//heure
				if(virgule==1)
				{
					j++;
					bufferMessage[n++]='H';
					for(int m = 0 ; m < 6 ; m++){
						if(n==2){
							tmp=bufferTram[j++];
							//tmp=tmp+1; //décalage horaire de 1
							bufferMessage[n++]=tmp;
						}else{
							bufferMessage[n++]=bufferTram[j++];
						}
					}
					bufferMessage[n++]=',';
					j += 3;
				}
				//latitude
				if(virgule==3)
				{
					if(bufferTram[j+1]!=',')
					{
						j++;
						bufferMessage[n++]='L';
						bufferMessage[n++]='A';
						bufferMessage[n++]='T';
						for(int m = 0 ; m < 9 ; m++){
							bufferMessage[n++]=bufferTram[j++];
						}
						j--;
					}
				}
				//N/S indicator
				if(virgule==4)
				{
					if(bufferTram[j+1]!=',')
					{
						j++;
						bufferMessage[n++]=bufferTram[j];
					}
					bufferMessage[n++]=',';
				}
				//longitude
				if(virgule==5)
				{
					if(bufferTram[j+1]!=',')
					{
						j++;
						bufferMessage[n++]='L';
						bufferMessage[n++]='O';
						bufferMessage[n++]='N';
						for(int m = 0 ; m < 10 ; m++){
							bufferMessage[n++]=bufferTram[j++];
						}
						j--;
					}
				}
				//E/W indicator
				if(virgule==6)
				{
					if(bufferTram[j+1]!=',')
					{
						j++;
						bufferMessage[n++]=bufferTram[j];
					}
					bufferMessage[n++]=',';
				}
				if(virgule==9)
				{
					j++;
					bufferMessage[n++]='D';
					for(int m = 0 ; m < 6 ; m++){
						bufferMessage[n++]=bufferTram[j++];
					}
					bufferMessage[n++]='*';
					bufferMessage[n++]='\0';
				}
				j++;
			}
			cleanTramGPS();
		}
	}
	return 0;
}

int lengthMessageGPS(){
	int length = 0;
	while(bufferMessage[length] != '\0'){
		length++;
	}
	return length;
}

int printMessageGPS(void){
	int messageLength = 0;
	messageLength = lengthMessageGPS();
	printf("Longueur : %d \r\n",messageLength);
	printf("Valeur: ");
	for(int m = 0; m <messageLength; m++){

		printf("%c",bufferMessage[m]);
	}
	return 0;
}

int cleanTramGPS(void)
{
	if(end==1)
		end=0;

	return 0;
}

int bufferInit(void)
{
	int j;
	for(j =0; j <i; j++){
		bufferTram[j]='0';
	}
	return 0;
}

int printIfNotNull(unsigned char c, int j)
{
	if(c!='\0')
	{
		UART1_Send(c);
		j++;
		return j;
	}
	else
	{
		j=0;
		bufferTram[1]='\0';
		return -1;
	}
}

