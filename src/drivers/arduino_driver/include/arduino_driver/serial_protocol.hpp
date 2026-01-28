#ifndef ARDUINO_DRIVER_SERIAL_PROTOCOL_HPP
#define ARDUINO_DRIVER_SERIAL_PROTOCOL_HPP

#include <string>
#include <map>
#include <cmath>

namespace arduino_driver {

// Convert radians to degrees
inline double radToDeg(double rad) {
    return rad * 180.0 / M_PI;
}

// Convert degrees to radians
inline double degToRad(double deg) {
    return deg * M_PI / 180.0;
}

// Clamp value to range
template<typename T>
T clamp(T value, T min_val, T max_val) {
    return std::max(min_val, std::min(value, max_val));
}

// Parse ultrasonic line format: "F:25,FL:30,FR:28,R:0,RL:0,RR:0"
std::map<std::string, double> parseUltrasonicData(const std::string& line);

}  // namespace arduino_driver

#endif  // ARDUINO_DRIVER_SERIAL_PROTOCOL_HPP
