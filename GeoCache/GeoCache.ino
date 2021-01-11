//Names of Team Members: Evelyn Lima Rocha, Mei Salud, Gabriel Koga
//Team Number: 7

#define NEO_ON 1		// NeoPixelShield
#define TRM_ON 1		// SerialTerminal
#define SDC_ON 1		// SecuREDigital
#define GPS_ON 1		// Live GPS Message (off = simulated)

// define pin usage
#define NEO_TX	6		// NEO transmit
#define GPS_TX	7		// GPS transmit
#define GPS_RX	8		// GPS receive
#define BUTTON  2
#define POTENTIOMETER A0

// GPS message buffer
#define GPS_RX_BUFSIZ	96
char cstr[GPS_RX_BUFSIZ];

// global variables
uint8_t target = 0;		// target number
float heading = 0.0;	// target heading
float distance = 0.0;	// target distance
char* info[15];

float currentLatitude = 0.0;
float currentLongitude = 0.0;

#if GPS_ON
#include <SoftwareSerial.h>
SoftwareSerial gps(GPS_RX, GPS_TX);
#endif

#if NEO_ON
#include <Adafruit_NeoPixel.h>
Adafruit_NeoPixel strip = Adafruit_NeoPixel(40, NEO_TX, NEO_GRB + NEO_KHZ800);
#endif

#if SDC_ON
#include <SD.h>
#endif

File curFile;

#define GEOLAT0 28.594532f
#define GEOLON0 -81.304437f

#if GPS_ON

#define PMTK_AWAKE "$PMTK010,002*2D"
#define PMTK_STANDBY "$PMTK161,0*28"
#define PMTK_Q_RELEASE "$PMTK605*31"
#define PMTK_ENABLE_WAAS "$PMTK301,2*2E"
#define PMTK_ENABLE_SBAS "$PMTK313,1*2E"
#define PMTK_CMD_HOT_START "$PMTK101*32"
#define PMTK_CMD_WARM_START "$PMTK102*31"
#define PMTK_CMD_COLD_START "$PMTK103*30"
#define PMTK_CMD_FULL_COLD_START "$PMTK104*37"
#define PMTK_SET_BAUD_9600 "$PMTK251,9600*17"
#define PMTK_SET_BAUD_57600 "$PMTK251,57600*2C"
#define PMTK_SET_NMEA_UPDATE_1HZ  "$PMTK220,1000*1F"
#define PMTK_SET_NMEA_UPDATE_5HZ  "$PMTK220,200*2C"
#define PMTK_API_SET_FIX_CTL_1HZ  "$PMTK300,1000,0,0,0,0*1C"
#define PMTK_API_SET_FIX_CTL_5HZ  "$PMTK300,200,0,0,0,0*2F"
#define PMTK_SET_NMEA_OUTPUT_RMC "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29"
#define PMTK_SET_NMEA_OUTPUT_GGA "$PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29"
#define PMTK_SET_NMEA_OUTPUT_RMCGGA "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"
#define PMTK_SET_NMEA_OUTPUT_OFF "$PMTK314,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"
#endif

float degMin2DecDeg(char *cind, char *ccor)
{
	double degrees = 0.0;
	double degMin = strtod(ccor, 0);
	double decDeg = (int)degMin / 100;
	degMin -= decDeg * 100;
	decDeg += degMin / 60;

	if (*cind == 'S' || *cind == 'W') {
		decDeg *= -1.0;
	}

	degrees = decDeg;
	return(degrees);
}

double calcDistance(float flat1, float flon1, float flat2, float flon2)
{
	double distance = 0.0;

	double earthRadiusMilesFeet = 3959.0 * 5280.00;

	double lat1rad = (flat1 * M_PI / 180);
	double lon1rad = (flon1 * M_PI / 180);
	double lat2rad = (flat2 * M_PI / 180);
	double lon2rad = (flon2 * M_PI / 180);

	double deltaLat = lat2rad - lat1rad;
	double deltaLon = lon2rad - lon1rad;

	double cos1 = pow(sin(deltaLat / 2.0), 2.0) + cos(lat1rad)*cos(lat2rad)*pow(sin(deltaLon / 2), 2);
	double cos2 = 2 * atan2(sqrt(cos1), sqrt(1.0 - cos1));

	double mDistance = earthRadiusMilesFeet * cos2;
	distance = mDistance;
	
	return distance;
	
}

float calcBearing(float flat1, float flon1, float flat2, float flon2)
{
	float bearing = 0.0;

	float lat1rad = (flat1 * M_PI / 180);
	float lon1rad = (flon1 * M_PI / 180);
	float lat2rad = (flat2 * M_PI / 180);
	float lon2rad = (flon2 * M_PI / 180);

	float bear1 = cos(lat2rad) * sin(lon2rad - lon1rad);
	float bear2 = (cos(lat1rad) * sin(lat2rad)) - (sin(lat1rad) * cos(lat2rad) * (cos(lon2rad - lon1rad)));

	bearing = atan2(bear1, bear2);
	bearing = (bearing * 180 / M_PI);

	bearing = fmod((bearing + 360), 360);

	return(bearing);
}

#if NEO_ON

int WestArrowPixels[] = { 4,11,12,13 };
int EastArrowPixels[] = { 36,27,28,29 };
int SouthArrowPixels[] = { 18,27,19,11 };
int NorthArrowPixels[] = { 13,21,22,29 };
int NortheastArrowPixels[] = { 30,38,37 };
int SoutheastArrowPixels[] = { 26,34,35 };
int SouthwestArrowPixels[] = { 2,3,10 };
int NorthwestArrowPixels[] = { 5,6,14 };
int DistanceRange[] = { 0, 8, 16, 24, 32,
						1, 9, 17, 25, 33, };
// 2500, 2000, 1600, 1200, 800, 400, 200, 100, 50, 10
int objectives[] = { 7,15,23,31 };
enum Ranges {
	One = 2500,
	Two = 2000,
	Three = 1600,
	Four = 1200,
	Five = 800,
	Six = 400,
	Seven = 200,
	Eigth = 100,
	Nine = 50,
	Ten = 10
};

void bright()
{
	strip.setBrightness(map(analogRead(POTENTIOMETER), 0, 1023, 0, 255));
	strip.show();
}

void clear() {
	strip.clear();
}

void setNeoPixel(void)
{
	uint8_t objective = target;

	// target number

	clear();
#define BLUE strip.Color(0, 0, 255)
#define GREEN strip.Color(0, 255, 0)
#define RED strip.Color(255, 0, 0)
#define YELLOW strip.Color(255, 250, 0)
	//Objective
	if (objective == 0)
		toggle(objectives, 1, RED);
	else if (objective == 1)
		toggle(objectives, 2, RED);
	else if (objective == 2)
		toggle(objectives, 3, RED);
	else if (objective == 3)
		toggle(objectives, 4, RED);


	//Distance
	if (distance < Ranges::One && distance > Ranges::Two)
		toggle(DistanceRange, 1, RED);
	else if (distance < Ranges::Two && distance > Ranges::Three)
		toggle(DistanceRange, 2, RED);
	else if (distance < Ranges::Three && distance > Ranges::Four)
		toggle(DistanceRange, 3, RED);
	else if (distance < Ranges::Four && distance > Ranges::Five)
		toggle(DistanceRange, 4, YELLOW);
	else if (distance < Ranges::Five && distance > Ranges::Six)
		toggle(DistanceRange, 5, YELLOW);
	else if (distance < Ranges::Six && distance > Ranges::Seven)
		toggle(DistanceRange, 6, YELLOW);
	else if (distance < Ranges::Seven && distance > Ranges::Eigth)
		toggle(DistanceRange, 7, GREEN);
	else if (distance < Ranges::Eigth && distance > Ranges::Nine)
		toggle(DistanceRange, 8, GREEN);
	else if (distance < Ranges::Nine && distance > Ranges::Ten)
		toggle(DistanceRange, 9, GREEN);
	else if (distance < Ranges::Ten)
		toggle(DistanceRange, 10, GREEN);

	if (heading < 22.5 || heading > 337.5)
		toggle(NorthArrowPixels, 4, BLUE);
	else if (heading > 22.5 && heading < 67.5)
		toggle(NortheastArrowPixels, 3, BLUE);
	else if (heading > 67.5  && heading < 112.5)
		toggle(EastArrowPixels, 4, BLUE);
	else if (heading > 112.5  && heading < 157.5)
		toggle(SoutheastArrowPixels, 3, BLUE);
	else if (heading > 157.5 && heading < 202.5)
		toggle(SouthArrowPixels, 4, BLUE);
	else if (heading > 202.5 && heading < 247.5)
		toggle(SouthwestArrowPixels, 3, BLUE);
	else if (heading > 247.5 && heading < 292.5)
		toggle(WestArrowPixels, 4, BLUE);
	else if (heading > 292.5 && heading < 337.5)
		toggle(NorthwestArrowPixels, 3, BLUE);
}

void toggle(int pixels[], int size, uint32_t color)
{
	for (int i = 0; i < size; i++)
	{
		strip.setPixelColor(pixels[i], color);
		bright();
	}
}



#endif	// NEO_ON

#if GPS_ON

void getGPSMessage(void)
{
	uint8_t x = 0, y = 0, isum = 0;

	memset(cstr, 0, sizeof(cstr));

	// get nmea string
	while (true)
	{
		if (gps.peek() != -1)
		{
			cstr[x] = gps.read();

			// if multiple inline messages, then restart
			if ((x != 0) && (cstr[x] == '$'))
			{
				x = 0;
				cstr[x] = '$';
			}

			// if complete message
			if ((cstr[0] == '$') && (cstr[x++] == '\n'))
			{
				// nul terminate string before /r/n
				cstr[x - 2] = 0;

				// if checksum not found
				if (cstr[x - 5] != '*')
				{
					x = 0;
					continue;
				}

				// convert hex checksum to binary
				isum = strtol(&cstr[x - 4], NULL, 16);

				// reverse checksum
				for (y = 1; y < (x - 5); y++) isum ^= cstr[y];

				// if invalid checksum
				if (isum != 0)
				{
					x = 0;
					continue;
				}

				// else valid message
				break;
			}
		}
	}
}
#endif	// GPS_ON

#if SDC_ON

bool CheckFiles() {
	for (uint32_t i = 0; i < 100; ++i) {
		char *fileName = new char[13];
		sprintf(fileName, "Mymap%d.txt", i);
		if (!SD.exists(fileName)) {
			curFile = SD.open(fileName, FILE_WRITE);
			return true;
		}
	}
	return false;
}

#endif

void setup(void)
{
#if TRM_ON
	// init serial interface
	Serial.begin(115200);
#endif	

#if NEO_ON
	// init NeoPixel Shield
	pinMode(POTENTIOMETER, INPUT);
	strip.begin();

	clear();
#endif	

#if SDC_ON
	SD.begin();
	if (!CheckFiles()) {
		Serial.println("Can't open file");
	}
#endif

#if GPS_ON
	// enable GPS sending GPRMC message
	gps.begin(9600);
	gps.println(PMTK_SET_NMEA_UPDATE_1HZ);
	gps.println(PMTK_API_SET_FIX_CTL_1HZ);
	gps.println(PMTK_SET_NMEA_OUTPUT_RMC);
#endif		

	currentLatitude = GEOLAT0;
	currentLongitude = GEOLON0;
	// init target button here
	pinMode(BUTTON, INPUT_PULLUP);
}

void loop(void)
{
	// max 1 second blocking call till GPS message received
	getGPSMessage();

#if TRM_ON

#endif	

	// if button pressed, set new target
	if (digitalRead(BUTTON) == LOW)
	{
		Serial.println("Button Pressed");
		target++;

		if (target >= 4)
		{
			target = 0;
		}

		if (target == 0)
		{
			currentLatitude = 28.59326;
			currentLongitude = -81.30323;
		}
		else if (target == 1)
		{
			currentLatitude = 28.59082;
			currentLongitude = -81.30391;
		}
		else if (target == 2)
		{
			currentLatitude = 28.59641;
			currentLongitude = -81.30081;
		}
		else if (target == 3)
		{
			currentLatitude = 28.59574;
			currentLongitude = -81.3052;
		}
	}

	//created here for later use on SD
	float givenLatitude;
	float givenLongitude;
	float givenBearing;

	// if GPRMC message (index 3 = R)
	if (cstr[3] == 'R')
	{
		// parse message parameters
		info[0] = strtok(cstr, ","); // separate by commas
		for (int i = 1; i < 15; i++)
		{
			info[i] = strtok(nullptr, ",");
		}

		givenLatitude = degMin2DecDeg(info[4], info[3]);
		givenLongitude = degMin2DecDeg(info[6], info[5]);
		
		// calculated destination heading
		char sKnots[3];
		for (int j = 0; j < 3; j++)
		{
			sKnots[j] = info[8][j];
		}

		givenBearing = calcBearing(givenLatitude, givenLongitude, currentLatitude, currentLongitude);
		float convertKnots = atof(sKnots);
		heading = fmod(givenBearing - (float)convertKnots + 360, 360);

		// calculated destination distance
		char cdog[12];
		distance = calcDistance(givenLatitude, givenLongitude, currentLatitude, currentLongitude);
		dtostrf(distance, 2, 2, cdog);

	}

#if SDC_ON
			// write current position to SecuREDigital then flush
	curFile.print(givenLongitude, 6);
	curFile.print(',');
	curFile.print(givenLatitude, 6);
	curFile.print(',');
	curFile.print(givenBearing);
	curFile.print('.');
	curFile.println(distance);
	curFile.flush();
#endif

#if NEO_ON
		// set NeoPixel target display
		setNeoPixel();

#endif			
}