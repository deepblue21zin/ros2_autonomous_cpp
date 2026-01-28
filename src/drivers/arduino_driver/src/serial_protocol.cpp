#include "arduino_driver/serial_protocol.hpp"
#include <sstream>

namespace arduino_driver {

std::map<std::string, double> parseUltrasonicData(const std::string& line) {
    std::map<std::string, double> result;

    // Parse format: "F:25,FL:30,FR:28,R:0,RL:0,RR:0"
    std::istringstream stream(line);
    std::string token;

    while (std::getline(stream, token, ',')) {
        // Find colon separator
        size_t colon_pos = token.find(':');
        if (colon_pos == std::string::npos) {
            continue;
        }

        std::string key = token.substr(0, colon_pos);
        std::string value_str = token.substr(colon_pos + 1);

        // Trim whitespace
        while (!key.empty() && std::isspace(key.front())) key.erase(key.begin());
        while (!key.empty() && std::isspace(key.back())) key.pop_back();
        while (!value_str.empty() && std::isspace(value_str.front())) value_str.erase(value_str.begin());
        while (!value_str.empty() && std::isspace(value_str.back())) value_str.pop_back();

        try {
            // Convert cm to meters
            double value_cm = std::stod(value_str);
            double value_m = value_cm / 100.0;
            result[key] = value_m;
        } catch (const std::exception&) {
            // Ignore invalid values
        }
    }

    return result;
}

}  // namespace arduino_driver
