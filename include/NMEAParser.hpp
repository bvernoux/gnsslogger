/**
 * @file NMEAParser.hpp
 * @brief Declaration of the NMEAParser class for parsing NMEA sentences.
 * 
 * This file contains the declaration of the NMEAParser class, which is responsible
 * for parsing various types of NMEA sentences and extracting relevant GNSS data.
 *
 * @author bvernoux@gmail.com
 */

#ifndef NMEA_PARSER_HPP
#define NMEA_PARSER_HPP

#include <string>
#include <sstream>
#include "GNSSUtility.hpp"

/**
 * @class NMEAParser
 * @brief A class for parsing NMEA sentences and extracting GNSS data.
 */
class NMEAParser {
public:
	/**
	 * @brief Construct a new NMEAParser object.
	 */
	NMEAParser();

	/**
	 * @brief Parse an NMEA sentence and extract GNSS data.
	 * 
	 * This method determines the type of NMEA sentence and calls the appropriate
	 * parsing method. It also validates the checksum of the sentence.
	 * 
	 * @param sentence The NMEA sentence to parse.
	 * @param data The GNSSData object to populate with parsed information.
	 * @return true if parsing was successful, false otherwise.
	 */
	bool parseNMEASentence(const std::string& sentence, GNSSData& data);

private:
	/**
	 * @brief Parse an RMC (Recommended Minimum Navigation Information) sentence.
	 * 
	 * @param iss An input string stream containing the RMC sentence fields.
	 * @param data The GNSSData object to populate with parsed information.
	 * @return true if parsing was successful, false otherwise.
	 */
	bool parseRMC(std::istringstream& iss, GNSSData& data);

	/**
	 * @brief Parse a GGA (Global Positioning System Fix Data) sentence.
	 * 
	 * @param iss An input string stream containing the GGA sentence fields.
	 * @param data The GNSSData object to populate with parsed information.
	 * @return true if parsing was successful, false otherwise.
	 */
	bool parseGGA(std::istringstream& iss, GNSSData& data);

	/**
	 * @brief Parse a GSA (GNSS DOP and Active Satellites) sentence.
	 * 
	 * @param iss An input string stream containing the GSA sentence fields.
	 * @param data The GNSSData object to populate with parsed information.
	 * @return true if parsing was successful, false otherwise.
	 */
	bool parseGSA(std::istringstream& iss, GNSSData& data);

	// Add any additional private helper methods or member variables here
};

#endif // NMEA_PARSER_HPP