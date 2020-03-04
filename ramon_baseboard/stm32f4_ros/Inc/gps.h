#include <stdint.h>

#define	GPRMC__TIME			1
#define	GPRMC__STATUS		2
#define	GPRMC__LATITUDE		3
#define	GPRMC__LATHEMISPH	4
#define	GPRMC__LONGITUDE	5
#define	GPRMC__LONGMERID	6
#define	GPRMC__SPEEDKNOTS	7
#define	GPRMC__TRACKANGLE	8
#define	GPRMC__DATE			9
#define	GPRMC__MAGNETICVAR	10
#define	GPRMC__MAGVARORIENT	11


struct gps_rmcdata
{
	float time;
	uint8_t status;
	float latitude;
	uint8_t latitudeHemisphere;
	float longitude;
	uint8_t longitudeMeridian;
	float speedKnots;
	float trackAngle;
	uint32_t date;
	float magneticVariation;
	uint8_t magneticVariationOrientation;
};


bool gps_getfromRMC(struct gps_rmcdata*);
bool gps_checksum(uint8_t* vec);
