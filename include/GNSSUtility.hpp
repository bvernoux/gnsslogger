/**
 * @file GNSSUtility.cpp
 * @brief Declaration of utility class for GNSS data processing.
 * 
 * This file contains the declaration of various utility class used in GNSS data
 * processing, including coordinate conversions, distance calculations, and data structure
 * management.
 *
 * @author bvernoux@gmail.com
 */
#ifndef GNSS_UTILITY_HPP
#define GNSS_UTILITY_HPP

#include <string>
#include <vector>
#include <cstdint>

// Define M_PI if not already defined
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/**
 * @struct GNSSData
 * @brief Stores parsed GNSS data from NMEA sentences.
 */
struct GNSSData {
	std::string date;
	std::string time;
	double latitude = 0.0;
	double longitude = 0.0;
	int fixQuality = 0;
	int fixType = 0;
	int satellitesUsed = 0;
	double altitude = 0.0;
	double speed = 0.0;
	double heading = 0.0;
	double hdop = 0.0;
	double pdop = 0.0;
	double vdop = 0.0;
	std::vector<std::string> satellitePRNs;
	bool isValid = false;
	std::string speedKmh;
	unsigned int nb_frames_received;

	/**
	 * @brief Clears all fields of the GNSSData structure.
	 */
	void clear();
};

namespace GNSSUtility {

	/**
	 * @brief Converts NMEA coordinate format to decimal degrees.
	 *
	 * @param nmeaPos The position in NMEA format (ddmm.mmmm or dddmm.mmmm).
	 * @param direction The direction character ('N', 'S', 'E', or 'W').
	 * @return The position in decimal degrees.
	 */
	double nmeaToDecimalDegrees(const std::string& nmeaPos, char direction);

	/**
	 * @brief Converts decimal degrees to NMEA coordinate format.
	 *
	 * @param decimalDegrees The position in decimal degrees.
	 * @param isLatitude True if the coordinate is a latitude, false for longitude.
	 * @return A pair containing the NMEA formatted string and direction character.
	 */
	std::pair<std::string, char> decimalDegreesToNMEA(double decimalDegrees, bool isLatitude);

	/**
	 * @brief Calculates the distance between two points on Earth using the Haversine formula.
	 *
	 * @param lat1 Latitude of the first point in decimal degrees.
	 * @param lon1 Longitude of the first point in decimal degrees.
	 * @param lat2 Latitude of the second point in decimal degrees.
	 * @param lon2 Longitude of the second point in decimal degrees.
	 * @return The distance between the two points in kilometers.
	 */
	double calculateDistance(double lat1, double lon1, double lat2, double lon2);

	/**
	 * @brief Calculates the bearing between two points on Earth.
	 *
	 * @param lat1 Latitude of the first point in decimal degrees.
	 * @param lon1 Longitude of the first point in decimal degrees.
	 * @param lat2 Latitude of the second point in decimal degrees.
	 * @param lon2 Longitude of the second point in decimal degrees.
	 * @return The bearing in degrees (0-360).
	 */
	double calculateBearing(double lat1, double lon1, double lat2, double lon2);

	/**
	 * @brief Validates an NMEA checksum.
	 *
	 * @param sentence The NMEA sentence to validate.
	 * @return true if the checksum is valid, false otherwise.
	 */
	bool validateNMEAChecksum(const std::string& sentence);

	/**
	 * @brief Converts knots to kilometers per hour.
	 *
	 * @param knots Speed in knots.
	 * @return Speed in kilometers per hour.
	 */
	double knotsToKmh(double knots);

	/**
	 * @brief Converts kilometers per hour to knots.
	 *
	 * @param kmh Speed in kilometers per hour.
	 * @return Speed in knots.
	 */
	double kmhToKnots(double kmh);

	std::string fixTypeToString(int type);
	
	std::string fixQualityToString(int quality);

} // namespace GNSSUtility

#endif // GNSS_UTILITY_HPP