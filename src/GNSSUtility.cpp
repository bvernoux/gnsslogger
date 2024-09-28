/**
 * @file GNSSUtility.cpp
 * @brief Implementation of utility class for GNSS data processing.
 * 
 * This file contains the implementation of various utility class used in GNSS data
 * processing, including coordinate conversions, distance calculations, and data structure
 * management.
 *
 * @author bvernoux@gmail.com
 */

#include "GNSSUtility.hpp"
#include <cmath>
#include <sstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <cstdlib>  // for std::stod
#include <stdexcept>

// Define M_PI if not already defined (for non-POSIX systems)
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// GNSSData::clear() implementation should be outside of any namespace
void GNSSData::clear()
{
	date.clear();
	time.clear();
	latitude = 0.0;
	longitude = 0.0;
	fixQuality = 0;
	fixType = 0;
	satellitesUsed = 0;
	altitude = 0.0;
	speed = 0.0;
	heading = 0.0;
	hdop = 0.0;
	pdop = 0.0;
	vdop = 0.0;
	satellitePRNs.clear();
	isValid = false;
	speedKmh.clear();
	nb_frames_received = 0;
}

/**
 * @brief Converts degrees to radians.
 * 
 * @param deg Value in degrees.
 * @return Value in radians.
 */
constexpr double deg2rad(double deg) {
	return deg * (M_PI / 180.0);
}

/**
 * @brief Converts radians to degrees.
 * 
 * @param rad Value in radians.
 * @return Value in degrees.
 */
constexpr double rad2deg(double rad) {
	return rad * (180.0 / M_PI);
}

namespace GNSSUtility {

/**
 * @brief Converts NMEA coordinate format to decimal degrees.
 *
 * This function takes an NMEA position string in the format `DDMM.MMMM` for latitude
 * or `DDDMM.MMMM` for longitude and converts it to decimal degrees. The function also
 * considers the direction ('N', 'S', 'E', 'W') to correctly adjust the sign of the result.
 *
 * @param nmeaPos   The NMEA coordinate as a string, in the format `DDMM.MMMM` (latitude) or `DDDMM.MMMM` (longitude).
 * @param direction The hemisphere or meridian character, must be one of:
 *                  - 'N' for northern latitude
 *                  - 'S' for southern latitude
 *                  - 'E' for eastern longitude
 *                  - 'W' for western longitude
 *
 * @return The corresponding decimal degrees as a `double`.
 *
 * @throws std::invalid_argument If the input format is incorrect or if the direction is not valid.
 */
double nmeaToDecimalDegrees(const std::string& nmeaPos, char direction) {
	// Validate the direction
	if (direction != 'N' && direction != 'S' && direction != 'E' && direction != 'W') {
		throw std::invalid_argument("Invalid direction. Expected 'N', 'S', 'E', or 'W'.");
	}

	// Validate input length (for latitude: at least 4 characters for DDMM; for longitude: at least 5 for DDDMM)
	if ((direction == 'N' || direction == 'S') && nmeaPos.length() < 4) {
		throw std::invalid_argument("Invalid NMEA coordinate format for latitude. Expected at least 4 characters (DDMM.MMMM).");
	}
	if ((direction == 'E' || direction == 'W') && nmeaPos.length() < 5) {
		throw std::invalid_argument("Invalid NMEA coordinate format for longitude. Expected at least 5 characters (DDDMM.MMMM).");
	}

	double decimalDegrees;
	std::string degreesStr, minutesStr;

	// Extract degrees and minutes depending on latitude or longitude
	try {
		if (direction == 'N' || direction == 'S') {
			// Latitude has 2 degrees digits (DDMM.MMMM)
			degreesStr = nmeaPos.substr(0, 2);
			minutesStr = nmeaPos.substr(2);
		} else if (direction == 'E' || direction == 'W') {
			// Longitude has 3 degrees digits (DDDMM.MMMM)
			degreesStr = nmeaPos.substr(0, 3);
			minutesStr = nmeaPos.substr(3);
		}

		// Convert degrees and minutes to double
		double degrees = std::stod(degreesStr); // May throw exception if conversion fails
		double minutes = std::stod(minutesStr); // May throw exception if conversion fails

		// Calculate decimal degrees
		decimalDegrees = degrees + (minutes / 60.0);

		// Adjust sign based on direction
		if (direction == 'S' || direction == 'W') {
			decimalDegrees = -decimalDegrees;
		}
	} catch (const std::exception& e) {
		(void)e;
		// Catch any conversion error from std::stod or substring access
		throw std::invalid_argument("Failed to parse NMEA coordinate format. Check if input contains valid numbers.");
	}

	return decimalDegrees;
}

/**
 * @brief Converts decimal degrees to NMEA coordinate format.
 *
 * This function takes a decimal degree value and converts it to the NMEA format (`DDMM.MMMM` for latitude
 * or `DDDMM.MMMM` for longitude). It also returns the direction based on the hemisphere or meridian.
 *
 * @param decimalDegrees The decimal degree value (positive for 'N' or 'E', negative for 'S' or 'W').
 * @param isLatitude     A boolean value indicating whether the decimal degree is for latitude (true) or longitude (false).
 *
 * @return A `std::pair<std::string, char>`, where:
 *         - The first part is the NMEA-formatted string (`DDMM.MMMM` for latitude or `DDDMM.MMMM` for longitude).
 *         - The second part is the direction ('N', 'S', 'E', 'W').
 *
 * @throws std::invalid_argument If the decimal degree is out of range for latitude or longitude.
 */
std::pair<std::string, char> decimalDegreesToNMEA(double decimalDegrees, bool isLatitude) {
	// Validate the range of latitude and longitude
	if (isLatitude) {
		if (decimalDegrees < -90.0 || decimalDegrees > 90.0) {
			throw std::invalid_argument("Invalid latitude value. Must be in the range [-90, 90].");
		}
	} else {
		if (decimalDegrees < -180.0 || decimalDegrees > 180.0) {
			throw std::invalid_argument("Invalid longitude value. Must be in the range [-180, 180].");
		}
	}

	// Determine the direction based on the sign of the decimal degree
	char direction;
	if (isLatitude) {
		direction = (decimalDegrees >= 0.0) ? 'N' : 'S';
	} else {
		direction = (decimalDegrees >= 0.0) ? 'E' : 'W';
	}

	// Convert decimal degrees to absolute value (NMEA uses positive numbers for format)
	double absDegrees = std::fabs(decimalDegrees);

	// Extract the degree and minute components
	int degrees = static_cast<int>(absDegrees);
	double minutes = (absDegrees - degrees) * 60.0;

	// Create the NMEA string with format `DDMM.MMMM` or `DDDMM.MMMM`
	std::ostringstream nmeaStream;
	if (isLatitude) {
		nmeaStream << std::setw(2) << std::setfill('0') << degrees
				   << std::fixed << std::setprecision(4) << std::setw(7) << minutes;
	} else {
		nmeaStream << std::setw(3) << std::setfill('0') << degrees
				   << std::fixed << std::setprecision(4) << std::setw(7) << minutes;
	}

	// Return the NMEA string and direction as a pair
	return std::make_pair(nmeaStream.str(), direction);
}

/**
 * @brief Calculates the distance between two points on Earth using the Haversine formula.
 *
 * @param lat1 Latitude of the first point in degrees.
 * @param lon1 Longitude of the first point in degrees.
 * @param lat2 Latitude of the second point in degrees.
 * @param lon2 Longitude of the second point in degrees.
 * @return The distance between the two points in kilometers.
 */
double calculateDistance(double lat1, double lon1, double lat2, double lon2) {
	// Radius of the Earth in kilometers
	constexpr double R = 6371.0;

	// Convert latitude and longitude from degrees to radians
	double lat1_rad = deg2rad(lat1);
	double lon1_rad = deg2rad(lon1);
	double lat2_rad = deg2rad(lat2);
	double lon2_rad = deg2rad(lon2);

	// Haversine formula
	double dlat = lat2_rad - lat1_rad;
	double dlon = lon2_rad - lon1_rad;
	
	double a = std::sin(dlat / 2) * std::sin(dlat / 2) +
			   std::cos(lat1_rad) * std::cos(lat2_rad) *
			   std::sin(dlon / 2) * std::sin(dlon / 2);
	
	double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1 - a));
	
	// Calculate the distance
	double distance = R * c;
	
	return distance;  // distance in kilometers
}

/**
 * @brief Calculates the initial bearing (forward azimuth) between two points on Earth.
 * 
 * This function takes the latitude and longitude of two points (in degrees) on the Earth and calculates 
 * the initial bearing (direction) from the first point to the second point.
 * 
 * @param lat1 Latitude of the first point in degrees.
 * @param lon1 Longitude of the first point in degrees.
 * @param lat2 Latitude of the second point in degrees.
 * @param lon2 Longitude of the second point in degrees.
 * @return The initial bearing from the first point to the second point in degrees.
 */
double calculateBearing(double lat1, double lon1, double lat2, double lon2) {
	// Convert latitude and longitude from degrees to radians
	double lat1_rad = deg2rad(lat1);
	double lon1_rad = deg2rad(lon1);
	double lat2_rad = deg2rad(lat2);
	double lon2_rad = deg2rad(lon2);

	// Difference in longitude
	double dLon = lon2_rad - lon1_rad;

	// Haversine formula for initial bearing
	double y = std::sin(dLon) * std::cos(lat2_rad);
	double x = std::cos(lat1_rad) * std::sin(lat2_rad) - 
			   std::sin(lat1_rad) * std::cos(lat2_rad) * std::cos(dLon);

	double bearing_rad = std::atan2(y, x);

	// Convert the bearing from radians to degrees
	double bearing_deg = rad2deg(bearing_rad);

	// Normalize the bearing to a value between 0 and 360 degrees
	bearing_deg = fmod((bearing_deg + 360.0), 360.0);

	return bearing_deg;
}

/**
 * @brief Validates an NMEA checksum.
 * 
 * @param sentence The NMEA sentence to validate.
 * @return true if the checksum is valid, false otherwise.
 */
bool validateNMEAChecksum(const std::string& sentence)
{
	// Find the position of the last asterisk ('*'), which marks the start of the checksum.
	size_t asteriskPos = sentence.find_last_of('*');

	// If no asterisk is found or if there are not exactly 2 characters for the checksum, return false.
	if (asteriskPos == std::string::npos || asteriskPos + 3 > sentence.length()) {
		return false;
	}

	// Extract the checksum part (2 hex digits) after the asterisk.
	std::string checksumStr = sentence.substr(asteriskPos + 1, 2);

	// Convert the checksum string to an integer (hexadecimal), and cast it to uint8_t.
	uint8_t providedChecksum = 0;
	try
	{
		providedChecksum = static_cast<uint8_t>(std::stoi(checksumStr, nullptr, 16));
	}
	catch (...)
	{
		/* Exception ignore keep previous data and continue => probably std::invalid_argument / std::out_of_range */
		return false;
	}

	// Calculate the checksum by XORing all characters between the '$' (excluded) and '*' (excluded).
	uint8_t calculatedChecksum = 0;
	for (size_t i = 1; i < asteriskPos; ++i) {
		calculatedChecksum ^= sentence[i];
	}

	// Return true if the provided checksum matches the calculated checksum.
	return providedChecksum == calculatedChecksum;
}

// Conversion factor
constexpr double KNOTS_TO_KMH_FACTOR = 1.852;

/**
 * @brief Converts knots to kilometers per hour.
 *
 * @param knots Speed in knots.
 * @return Speed in kilometers per hour.
 */
double knotsToKmh(double knots) {
	return knots * KNOTS_TO_KMH_FACTOR;
}

/**
 * @brief Converts kilometers per hour to knots.
 *
 * @param kmh Speed in kilometers per hour.
 * @return Speed in knots.
 */
double kmhToKnots(double kmh) {
	return kmh / KNOTS_TO_KMH_FACTOR;
}

/**
 * @brief Converts an integer representing the fix type into a string.
 *
 * @param type An integer representing the type of fix:
 *             - 1: No Fix Available ("NA")
 *             - 2: 2D Fix ("2D")
 *             - 3: 3D Fix ("3D")
 * 
 * @return A string representation of the fix type. Returns "Unknown" for unrecognized types.
 */
std::string fixTypeToString(int type){
	switch (type){
	case 1:
		return "NA";
	case 2:
		return "2D";
	case 3:
		return "3D";
	default:
		return "Unknown";
	}
}

/**
 * @brief Converts an integer representing the quality of the fix into a string.
 *
 * @param quality An integer representing the quality of the fix:
 *                - 0: Invalid Fix
 *                - 1: Standard GPS Fix
 *                - 2: DGPS (Differential GPS) Fix
 *                - 3: PPS (Precise Positioning Service) Fix
 *                - 4: Real Time Kinematic (RTK) Fix
 *                - 5: Real Time Kinematic (float) Fix
 *                - 6: Estimated Fix
 * 
 * @return A string representation of the fix quality. Returns "Unknown" for unrecognized qualities.
 */
std::string fixQualityToString(int quality){
	switch (quality){
	case 0:
		return "Invalid";
	case 1:
		return "Standard";
	case 2:
		return "DGPS";
	case 3:
		return "PPS fix";
	case 4:
		return "Real Time Kinetic";
	case 5:
		return "Real Time Kinetic(float)";
	case 6:
		return "Estimate";
	default:
		return "Unknown";
	}
}

} // namespace GNSSUtility

