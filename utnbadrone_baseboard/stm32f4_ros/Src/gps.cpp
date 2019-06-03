#include "gps.h"
#include <stddef.h>
#include <string.h>
#include <stdlib.h>
#include "mainpp.h"
#include "stm32f4xx_hal.h"

extern uint8_t databuf[PINGPONG_SIZE];
//extern UART_HandleTypeDef huart2;

bool gps_getfromRMC(gps_rmcdata *gps_rmc)
{
	const char fin='\n';
	int coma = ',';
	int i;
	int ind, offset = 0;
//	uint8_t auxbuf[PINGPONG_SIZE];
	float latitude_aux = 0,longitude_aux = 0, latitude_aux_min = 0, longitude_aux_min = 0;
	float latitude_aux_int = 0,longitude_aux_int = 0;
//	char *test = "$GPRMC,,V,1,,,,2,,,4,,N*53\r\n";

	/* GPRMC DATA:
	 *
	 * 	0	  1		    2 3		  4 5		 6 7   8   9    10 ,11
	 * 	|	  |			| |		  |	|		 | |   |   |	|	| 12
	 * 	|	  |         | |       | |        | |   |   |    |   | |
	 * $--RMC,hhmmss.ss,A,llll.ll,a,yyyyy.yy,a,x.x,x.x,xxxx,x.x,a*hh
	 *
	 * 0  ->  IDENTIFIER (GPRMC)
	 * 1  ->  TIME (UTC)
	 * 2  ->  STATUS, V = Navigation receiver warning
	 * 3  ->  Latitude
	 * 4  ->  N or S
	 * 5  ->  Longitude
	 * 6  ->  E or W
	 * 7  ->  Speed over ground, knots
	 * 8  ->  Track made good, degrees true
	 * 9  ->  Date, ddmmyy
	 * 10 ->  Magnetic Variation, degrees
	 * 11 ->  E or W
	 * 12 ->  Checksum
	 */

//	memcpy((void*)auxbuf,(const void*) databuf, strlen((const char*) databuf));   /* Copy first part */

	//Verifico que el checksum sea correcto
	if(gps_checksum(databuf) == false)
		return false;

	ind = (strcspn((const char*)&databuf[offset],(const char*)&fin));
	databuf[ind+1] = '\0';

//	HAL_UART_Transmit(&huart2,auxbuf,strlen((const char*)auxbuf),200);
	ind = (strcspn((const char*)&databuf[offset],(const char*)&coma));
	databuf[offset + ind] = '\0';

	//Me fijo si el mensaje corresponde a GPRMC
	if(strcmp((const char*)&databuf[offset],(const char*)"$GPRMC") == 0)
	{
		offset = offset + ind + 1;

		for(i=1 ; i<=12 ; i++)
		{
			ind = (strcspn((const char*)&databuf[offset],(const char*)&coma));
			databuf[offset + ind] = '\0';

			switch(i)
			{
				case GPRMC__TIME:
					if(ind > 0)
						gps_rmc->time = atof((const char*)&databuf[offset]);
//					else
//						gps_rmc->time = 0.0;
					break;

				case GPRMC__STATUS:
					if(ind > 0)
						gps_rmc->status = databuf[offset];
//						if(databuf[offset] != 'A')
//							return true;
//					else
//						gps_rmc->status = 0;
					break;

				case GPRMC__LATITUDE:
					if(ind > 0)
						latitude_aux = atof((const char*)&databuf[offset]);
//						gps_rmc->latitude  = atof((const char*)&databuf[offset]);
//					else
//						gps_rmc->latitude = 0.0;
					break;

				case GPRMC__LATHEMISPH:
					if(ind > 0)
					{
						gps_rmc->latitudeHemisphere = databuf[offset];
						latitude_aux_int = int(latitude_aux/100);
						latitude_aux_min = float((latitude_aux - (latitude_aux_int*100))/60);
						if(gps_rmc->latitudeHemisphere == 'N')
							gps_rmc->latitude = (float(latitude_aux_int) + latitude_aux_min);
						else
							gps_rmc->latitude = -(float(latitude_aux_int) + latitude_aux_min);
					}
//					else
//						gps_rmc->latitudeHemisphere = 0;
					break;

				case GPRMC__LONGITUDE:
					if(ind > 0)
						longitude_aux = atof((const char*)&databuf[offset]);
//						gps_rmc->longitude = atof((const char*)&databuf[offset]);
//					else
//						gps_rmc->longitude = 0.0;
					break;

				case GPRMC__LONGMERID:
					if(ind > 0)
					{
						gps_rmc->longitudeMeridian = databuf[offset];
						longitude_aux_int = int(longitude_aux/100);
						longitude_aux_min = float((longitude_aux - longitude_aux_int*100)/60);
						if(gps_rmc->longitudeMeridian == 'E')
							gps_rmc->longitude = (float(longitude_aux_int) + longitude_aux_min);
						else
							gps_rmc->longitude = -(float(longitude_aux_int) + longitude_aux_min);
					}
//					else
//						gps_rmc->longitudeMeridian = 0;
					break;

				case GPRMC__SPEEDKNOTS:
					if(ind > 0)
						gps_rmc->speedKnots = atof((const char*)&databuf[offset]);
//					else
//						gps_rmc->speedKnots = 0.0;
					break;

				case GPRMC__TRACKANGLE:
					if(ind > 0)
						gps_rmc->trackAngle = atof((const char*)&databuf[offset]);
//					else
//						gps_rmc->trackAngle = 0.0;
					break;

				case GPRMC__DATE:
					if(ind > 0)
						gps_rmc->date = atoi((const char*)&databuf[offset]);
//					else
//						gps_rmc->date = 0;
					break;

				case GPRMC__MAGNETICVAR:
					if(ind > 0)
						gps_rmc->magneticVariation = atof((const char*)&databuf[offset]);
//					else
//						gps_rmc->magneticVariation = 0.0;
					break;

				case GPRMC__MAGVARORIENT:
					if(ind > 0)
						gps_rmc->magneticVariationOrientation = databuf[offset];
//					else
//						gps_rmc->magneticVariationOrientation = 0;
					break;

				default:
					break;
			}

			offset = offset + ind + 1;

			if(i == 12)
				i = 13;
		}
	}
	databuf[0] = '\0';
	return true;
}



bool gps_checksum(uint8_t* vec)
{
	const char asterisco = '*';
	int ind, i;
	uint8_t check[4];
	int checksum = 0x00;
	int num = 0;

	//Primero, verifico que el checksum este bien
	ind = (strcspn((const char*)&vec[0],(const char*)&asterisco));

	if(ind <= 0)
		return false;

	for(i = 1; i< ind; i++)
		checksum = checksum ^ vec[i];

	ind++;

	i = 0;
	for (; ind < int(strlen((const char*)vec)-1) ; ++ind)
	{

		if(vec[ind] == '\r')
		{
			i++;
			break;
		}
		else
		{
			check[i]=vec[ind];
			i++;
		}
		if(i > 4)
			return false;
	}

	check[i]='\0';
	num = strtoul((const char*)check,NULL,16);

	if(num != checksum)
		return false;

	return true;
}
