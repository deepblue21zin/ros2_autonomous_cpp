#ifndef PERCEPTION_PKG_DETECTION_TYPES_HPP
#define PERCEPTION_PKG_DETECTION_TYPES_HPP

#include <tuple>
#include <utility>

namespace perception_pkg {

// Line style enumeration
enum class LineStyle : int {
    UNKNOWN = 0,
    SOLID = 1,
    DASHED = 2
};

// Line type enumeration
enum class LineType : int {
    LEFT_WHITE = 0,
    RIGHT_WHITE = 1,
    YELLOW_CENTER = 2
};

// Corresponds to Python LineSegment dataclass
struct LineSegment {
    LineType type_id;
    LineStyle style_id;
    std::pair<float, float> p1;  // (x, y) bottom point
    std::pair<float, float> p2;  // (x, y) top point
    float score;

    LineSegment(LineType type, LineStyle style,
                float x1, float y1, float x2, float y2, float sc)
        : type_id(type), style_id(style),
          p1{x1, y1}, p2{x2, y2}, score(sc) {}

    LineSegment() : type_id(LineType::LEFT_WHITE), style_id(LineStyle::UNKNOWN),
                    p1{0.0f, 0.0f}, p2{0.0f, 0.0f}, score(0.0f) {}
};

// Corresponds to Python Detection dataclass
struct Detection {
    float x1, y1, x2, y2;
    float score;

    Detection() : x1(0), y1(0), x2(0), y2(0), score(0) {}
    Detection(float x1_, float y1_, float x2_, float y2_, float sc)
        : x1(x1_), y1(y1_), x2(x2_), y2(y2_), score(sc) {}

    float width() const { return std::max(0.0f, x2 - x1); }
    float height() const { return std::max(0.0f, y2 - y1); }
    float area() const { return width() * height(); }
    float centerX() const { return (x1 + x2) * 0.5f; }
    float centerY() const { return (y1 + y2) * 0.5f; }
};

// Stop line result
struct StopLine {
    float x1, y1, x2, y2;
    float distance_m;
    bool valid;

    StopLine() : x1(0), y1(0), x2(0), y2(0), distance_m(0), valid(false) {}
    StopLine(float x1_, float y1_, float x2_, float y2_, float dist)
        : x1(x1_), y1(y1_), x2(x2_), y2(y2_), distance_m(dist), valid(true) {}
};

}  // namespace perception_pkg

#endif  // PERCEPTION_PKG_DETECTION_TYPES_HPP
