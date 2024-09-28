/**
 * @file NMEAParser.cpp
 * @brief Implementation of the NMEAParser class for parsing NMEA sentences.
 * 
 * This file contains the implementation of the NMEAParser class, which is responsible
 * for parsing various types of NMEA sentences and extracting relevant GNSS data.
 *
 * @author bvernoux@gmail.com
 */

#include "NMEAParser.hpp"
#include "GNSSUtility.hpp"
#include <sstream>
#include <iomanip>
#include <algorithm>

NMEAParser::NMEAParser()
{
	// Initialize any necessary members here
}

bool NMEAParser::parseNMEASentence(const std::string& sentence, GNSSData& data)
{
	if (!GNSSUtility::validateNMEAChecksum(sentence))
	{
		return false;
	}

	std::istringstream iss(sentence);
	std::string sentenceType;
	std::getline(iss, sentenceType, ',');

	if (sentenceType == "$GPRMC" || sentenceType == "$GNRMC")
	{
		return parseRMC(iss, data);
	}
	else if (sentenceType == "$GPGGA" || sentenceType == "$GNGGA")
	{
		// New NMEA cycle, clear the PRNs list
		data.satellitePRNs.clear();
		return parseGGA(iss, data);
	}
	else if (sentenceType == "$GPGSA" || sentenceType == "$GNGSA")
	{
		return parseGSA(iss, data);
	}
	// Add more sentence types as needed

	return false; // Unrecognized sentence type
}

/*
	https://receiverhelp.trimble.com/alloy-gnss/en-us/NMEA-0183messages_RMC.html
	$GNRMC - Recommended Minimum Specific GNSS Data
	Format:  $GNRMC,hhmmss.ss,A,llll.ll,a,yyyyy.yy,a,x.x,x.x,ddmmyy,x.x,a*hh
	Example: $GNRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A
	Fields: Time, Status, Latitude, N/S, Longitude, E/W, Speed, Track angle, Date, Magnetic variation, E/W, Checksum

	https://openrtk.readthedocs.io/en/latest/communication_port/nmea.html?highlight=gnrmc
	Format: $GNRMC,<1>,<2>,<3>,<4>,<5>,<6>,<7>,<8>,<9>,<10>,<11>,< 12>*xx<CR><LF>
	E.g:    $GNRMC,072446.00,A,3130.5226316,N,12024.0937010,E,0.01,0.00,040620,0.0,E,D*3D
	Field explanation:
	<0> $GNRMC
	<1> UTC time, the format is hhmmss.sss
	<2> Positioning status, A=effective positioning, V=invalid positioning
	<3> Latitude, the format is ddmm.mmmmmmm
	<4> Latitude hemisphere, N or S (north latitude or south latitude)
	<5> Longitude, the format is dddmm.mmmmmmm
	<6> Longitude hemisphere, E or W (east longitude or west longitude)
	<7> Ground speed
	<8> Ground heading (take true north as the reference datum)
	<9> UTC date, the format is ddmmyy (day, month, year)
	<10> Magnetic declination (000.0~180.0 degrees)
	<11> Magnetic declination direction, E (east) or W (west)
	<12> Mode indication (A=autonomous positioning, D=differential, E=estimation, N=invalid data)
	* Statement end marker
	XX XOR check value of all bytes starting from $ to *
	<CR> Carriage return, end tag
	<LF> line feed, end tag

	$GNRMC,135406.00,A,4453.86521,N,00501.04969,E,0.029,,180924,,,D,V*11
	$GNRMC,0        ,1,2         ,3,4          ,5,6    ,7,8    ,9,10,11*xx<CR><LF>
*/
bool NMEAParser::parseRMC(std::istringstream& iss, GNSSData& data)
{
	std::string field;

	// Time
	std::getline(iss, field, ',');
	if (!field.empty())
	{
		data.time = field.substr(0, 6);
	}

	// Status
	std::getline(iss, field, ',');
	data.isValid = (field == "A");

	// Latitude
	std::getline(iss, field, ',');
	std::string latDirection;
	std::getline(iss, latDirection, ',');
	if (!field.empty() && !latDirection.empty())
	{
		data.latitude = GNSSUtility::nmeaToDecimalDegrees(field, latDirection[0]);
	}

	// Longitude
	std::getline(iss, field, ',');
	std::string lonDirection;
	std::getline(iss, lonDirection, ',');
	if (!field.empty() && !lonDirection.empty())
	{
		data.longitude = GNSSUtility::nmeaToDecimalDegrees(field, lonDirection[0]);
	}

	// Speed over ground
	std::getline(iss, field, ',');
	if (!field.empty())
	{
		try
		{
			data.speed = std::stod(field);
			std::stringstream ss;
			ss << std::fixed << std::setprecision(1) << (data.speed * 1.852); // 1 knot = 1.852 km/h
			data.speedKmh = ss.str();
		}
		catch (...)
		{
			/* Exception ignore keep previous data and continue => probably std::invalid_argument / std::out_of_range */
		}
	}

	// Course over ground
	std::getline(iss, field, ',');
	if (!field.empty())
	{
		try
		{
			data.heading = std::stoi(field);
		}
		catch (...)
		{
			/* Exception ignore keep previous data and continue => probably std::invalid_argument / std::out_of_range */
		}
	}

	// Date
	std::getline(iss, field, ',');
	if (!field.empty())
	{
		data.date = field;
	}

	return true;
}

/*
    https://receiverhelp.trimble.com/alloy-gnss/en-us/NMEA-0183messages_GGA.html
	$GNGGA - Global Positioning System Fix Data
	Format: $GNGGA,hhmmss.ss,llll.ll,a,yyyyy.yy,a,x,xx,x.x,x.x,M,x.x,M,x.x,xxxx*hh
	Example: $GNGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47
	Fields: Time, Latitude, N/S, Longitude, E/W, Fix quality, Satellites used, HDOP, Altitude, M, Geoidal separation, M, Age of diff. corr., Diff. ref. station ID

	https://openrtk.readthedocs.io/en/latest/communication_port/nmea.html?highlight=gngga
	$GNGGA
	Format: $GNGGA,<1>,<2>,<3>,<4>,<5>,<6>,<7>,<8>,<9>,M,<10>,M,< 11>,<12>*xx<CR><LF>
	E.g:    $GNGGA,072446.00,3130.5226316,N,12024.0937010,E,4,27,0.5,31.924,M,0.000,M,2.0,*44
	Field explanation:
	<0> $GNGGA
	<1> UTC time, the format is hhmmss.sss
	<2> Latitude, the format is ddmm.mmmmmmm
	<3> Latitude hemisphere, N or S (north latitude or south latitude)
	<4> Longitude, the format is dddmm.mmmmmmm
	<5> Longitude hemisphere, E or W (east longitude or west longitude)
	<6> GNSS positioning status: 0 not positioned, 1 single point positioning, 2 differential GPS fixed solution, 4 fixed solution, 5 floating point solution
	<7> Number of satellites used
	<8> HDOP level precision factor
	<9> Altitude
	<10> The height of the earth ellipsoid relative to the geoid
	<11> Differential time
	<12> Differential reference base station label
	* Statement end marker
	xx XOR check value of all bytes starting from $ to *
	<CR> Carriage return, end tag
	<LF> line feed, end tag

	$GNGGA,140920.00,4453.86803,N,00501.04813,E,2,12,0.70,205.9,M,47.5,M,,*4A
	$GNGGA,0        ,1         ,2,3          ,4,5,6 ,7   ,8    ,9,10  ,11,12*xx<CR><LF>
*/
bool NMEAParser::parseGGA(std::istringstream& iss, GNSSData& data)
{
	std::string field;

	// Time
	std::getline(iss, field, ',');
	// Do not use time of that field as we prefer to use Time from $GNGSA
	/*
	if (!field.empty())
	{
		data.dateTime = field;
	}
	*/

	// Latitude
	std::getline(iss, field, ',');
	std::string latDirection;
	std::getline(iss, latDirection, ',');
	if (!field.empty() && !latDirection.empty())
	{
		data.latitude = GNSSUtility::nmeaToDecimalDegrees(field, latDirection[0]);
	}

	// Longitude
	std::getline(iss, field, ',');
	std::string lonDirection;
	std::getline(iss, lonDirection, ',');
	if (!field.empty() && !lonDirection.empty())
	{
		data.longitude = GNSSUtility::nmeaToDecimalDegrees(field, lonDirection[0]);
	}

	// Fix quality
	std::getline(iss, field, ',');
	if (!field.empty())
	{
		try
		{
			data.fixQuality = std::stoi(field);
		}
		catch (...)
		{
			/* Exception ignore keep previous data and continue => probably std::invalid_argument / std::out_of_range */
		}
	}

	// Number of satellites Used
	std::getline(iss, field, ',');
	if (!field.empty())
	{
		try
		{
			data.satellitesUsed = std::stoi(field);
		}
		catch (...)
		{
			/* Exception ignore keep previous data and continue => probably std::invalid_argument / std::out_of_range */
		}
	}

	// HDOP
	std::getline(iss, field, ',');
	// Do not use HDOP of that field as we prefer to use Time from $GNGSA
	/*
	if (!field.empty())
	{
		try
		{
			data.hdop = std::stod(field);
		}
		catch (...)
		{
			// Exception ignore keep previous data and continue => probably std::invalid_argument / std::out_of_range
		}
	}
	*/

	// Altitude
	std::getline(iss, field, ',');
	if (!field.empty())
	{
		try
		{
			data.altitude = std::stod(field);
		}
		catch (...)
		{
			/* Exception ignore keep previous data and continue => probably std::invalid_argument / std::out_of_range */
		}
	}
	return true;
}

/*
https://receiverhelp.trimble.com/alloy-gnss/en-us/NMEA-0183messages_GSA.html
$GNGSA - GNSS DOP and Active Satellites
Format: $GNGSA,a,x,xx,xx,xx,xx,xx,xx,xx,xx,xx,xx,xx,xx,x.x,x.x,x.x*hh
Example: $GNGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30
Fields: Mode (M=Manual, A=Automatic), Fix type (1=No fix, 2=2D, 3=3D), PRNs of satellites used for fix (12 fields),
		 PDOP (Position Dilution of Precision), HDOP (Horizontal DOP), VDOP (Vertical DOP)

https://openrtk.readthedocs.io/en/latest/communication_port/nmea.html?highlight=gngsa
$GNGSA

format: $GNGSA,<1>,<2>,<3>,<3>,,,,,<3>,<3>,<3>,<4>,<5>,<6>,<7> *xx<CR><LF>
   E.g: $GNGSA,A,3,03,06,09,17,19,23,28,,,,,,3.0,1.5,2.6,1*25
		$GNGSA,A,3,65,66,67,81,82,88,,,,,,,2.4,1.3,2.1,2*36
		$GNGSA,A,3,02,05,09,15,27,,,,,,,,,10.8,2.7,10.4,3*3A
		$GNGSA,A,3,01,02,07,08,10,13,27,28,32,33,37,,2.1,1.0,1.9,5*33
Field explanation:
		<1> Mode: M=Manual, A=Auto
		<2> Positioning type: 1=not positioned, 2=two-dimensional positioning, 3=three-dimensional positioning
		<3> PRN code (Pseudo Random Noise Code), channels 1 to 12, up to 12
		<4> PDOP position precision factor
		<5> HDOP level precision factor
		<6> VDOP vertical precision factor
		<7> GNSS system ID: 1(GPS), 2(GLONASS), 3(GALILEO), 5(BEIDOU)
		* Statement end marker
		xx XOR check value of all bytes starting from $ to *
		<CR> Carriage return, end tag
		<LF> line feed, end tag

Example
$GNGSA,A,3,05,07,11,13,14,15,18,20,22,30,,,1.13,0.61,0.95,1*00
$GNGSA,A,3,04,10,11,12,19,21,27,29,,,,,1.13,0.61,0.95,3*08
$GNGSA,A,3,21,22,36,42,45,,,,,,,,1.13,0.61,0.95,4*0D
$GNGSA,A,3,,,,,,,,,,,,,1.13,0.61,0.95,5*0D
*/
bool NMEAParser::parseGSA(std::istringstream& iss, GNSSData& data)
{
	std::string field;

	// Auto selection of 2D or 3D fix
	std::getline(iss, field, ',');

	// 3D fix
	std::getline(iss, field, ',');
	if (!field.empty())
	{
		try
		{
			data.fixType = std::stoi(field);
		}
		catch (...)
		{
			/* Exception ignore keep previous data and continue => probably std::invalid_argument / std::out_of_range */
		}
	}

	// PRNs of satellites used for fix
	for (int i = 0; i < 12; ++i)
	{
		std::getline(iss, field, ',');
		if (!field.empty())
		{
			data.satellitePRNs.push_back(field);
		}
	}

	// PDOP
	std::getline(iss, field, ',');
	if (!field.empty())
	{
		try
		{
			data.pdop = std::stod(field);
		}
		catch (...)
		{
			/* Exception ignore keep previous data and continue => probably std::invalid_argument / std::out_of_range */
		}
	}

	// HDOP
	std::getline(iss, field, ',');
	if (!field.empty())
	{
		try
		{
			data.hdop = std::stod(field);
		}
		catch (...)
		{
			/* Exception ignore keep previous data and continue => probably std::invalid_argument / std::out_of_range */
		}
	}

	// VDOP
	std::getline(iss, field, ',');
	if (!field.empty())
	{
		try
		{
			data.vdop = std::stod(field);
		}
		catch (...)
		{
			/* Exception ignore keep previous data and continue => probably std::invalid_argument / std::out_of_range */
		}
	}
	return true;
}
