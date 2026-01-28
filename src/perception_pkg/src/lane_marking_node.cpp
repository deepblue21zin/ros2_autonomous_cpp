#include "perception_pkg/lane_marking_node.hpp"
#include "perception_pkg/common/image_utils.hpp"
#include "perception_pkg/common/lane_geometry.hpp"
#include <opencv2/imgcodecs.hpp>
#include <algorithm>
#include <numeric>
#include <sstream>

namespace perception_pkg {

LaneMarkingNode::LaneMarkingNode()
    : Node("lane_marking_node"), current_light_state_("unknown") {
    // Declare and load parameters
    this->declare_parameter("camera_topic", "/camera/image_raw");
    this->declare_parameter("use_compressed", false);
    this->declare_parameter("roi_y_ratio", 0.55);
    this->declare_parameter("canny_low", 50);
    this->declare_parameter("canny_high", 150);
    this->declare_parameter("stop_roi_ratio", 0.25);
    this->declare_parameter("nwindows", 9);
    this->declare_parameter("window_margin", 50);
    this->declare_parameter("minpix", 60);
    this->declare_parameter("stop_line_pixels_per_meter", 60.0);
    this->declare_parameter("stop_line_min_aspect", 2.5);
    this->declare_parameter("stop_line_min_width", 80.0);
    this->declare_parameter("use_traffic_light_gate", true);
    this->declare_parameter("allowed_stop_states", "red,yellow");

    camera_topic_ = this->get_parameter("camera_topic").as_string();
    use_compressed_ = this->get_parameter("use_compressed").as_bool();
    roi_y_ratio_ = this->get_parameter("roi_y_ratio").as_double();
    canny_low_ = this->get_parameter("canny_low").as_int();
    canny_high_ = this->get_parameter("canny_high").as_int();
    stop_roi_ratio_ = this->get_parameter("stop_roi_ratio").as_double();
    nwindows_ = this->get_parameter("nwindows").as_int();
    window_margin_ = this->get_parameter("window_margin").as_int();
    minpix_ = this->get_parameter("minpix").as_int();
    stop_line_pixels_per_meter_ = this->get_parameter("stop_line_pixels_per_meter").as_double();
    stop_line_min_aspect_ = this->get_parameter("stop_line_min_aspect").as_double();
    stop_line_min_width_ = this->get_parameter("stop_line_min_width").as_double();
    use_traffic_light_gate_ = this->get_parameter("use_traffic_light_gate").as_bool();

    // Parse allowed stop states
    std::string states_str = this->get_parameter("allowed_stop_states").as_string();
    std::istringstream ss(states_str);
    std::string state;
    while (std::getline(ss, state, ',')) {
        // Trim whitespace
        state.erase(0, state.find_first_not_of(" \t\n\r\f\v"));
        state.erase(state.find_last_not_of(" \t\n\r\f\v") + 1);
        if (!state.empty()) {
            allowed_stop_states_.push_back(state);
        }
    }

    // QoS for low latency
    auto qos_sensor = rclcpp::QoS(1).best_effort();

    // Setup subscribers
    if (use_compressed_) {
        compressed_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
            camera_topic_ + "/compressed", qos_sensor,
            std::bind(&LaneMarkingNode::compressedCallback, this, std::placeholders::_1));
    } else {
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            camera_topic_, qos_sensor,
            std::bind(&LaneMarkingNode::imageCallback, this, std::placeholders::_1));
    }

    traffic_light_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/perception/traffic_light_state", 10,
        std::bind(&LaneMarkingNode::trafficLightCallback, this, std::placeholders::_1));

    // Setup publishers
    lane_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/perception/lane_markings", 10);
    stop_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/perception/stop_line", 10);

    RCLCPP_INFO(this->get_logger(), "LaneMarkingNode initialized");
    RCLCPP_INFO(this->get_logger(), "  camera_topic: %s", camera_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  roi_y_ratio: %.2f", roi_y_ratio_);
    RCLCPP_INFO(this->get_logger(), "  nwindows: %d, margin: %d, minpix: %d", nwindows_, window_margin_, minpix_);
}

void LaneMarkingNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    try {
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
        handleFrame(cv_ptr->image);
    } catch (const cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
}

void LaneMarkingNode::compressedCallback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
    try {
        cv::Mat frame = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
        if (!frame.empty()) {
            handleFrame(frame);
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Image decode error: %s", e.what());
    }
}

void LaneMarkingNode::trafficLightCallback(const std_msgs::msg::String::SharedPtr msg) {
    current_light_state_ = msg->data;
}

void LaneMarkingNode::handleFrame(const cv::Mat& frame) {
    // Extract ROI
    auto [roi_color, roi_y] = extractROI(frame, roi_y_ratio_);

    // Convert to HSV
    cv::Mat hsv;
    cv::cvtColor(roi_color, hsv, cv::COLOR_BGR2HSV);

    // Threshold white and yellow
    cv::Mat white_mask = thresholdWhite(hsv);
    cv::Mat yellow_mask = thresholdYellow(hsv);

    // Apply morphological closing
    white_mask = morphClose(white_mask, 3, 2);
    yellow_mask = morphClose(yellow_mask, 3, 2);

    // Detect lane segments
    std::vector<LineSegment> segments = detectLaneSegments(white_mask, yellow_mask, roi_y);

    // Publish lane segments
    publishLaneSegments(segments);

    // Detect stop line (only if traffic light allows)
    if (trafficLightAllowsStop() || !use_traffic_light_gate_) {
        StopLine stop_line = detectStopLine(frame, roi_y);
        publishStopLine(stop_line);
    }
}

std::vector<LineSegment> LaneMarkingNode::detectLaneSegments(
    const cv::Mat& white_mask, const cv::Mat& yellow_mask, int roi_y) {

    std::vector<LineSegment> all_segments;

    // Detect white lanes (left and right)
    auto white_segments = fitLaneFromMask(white_mask, roi_y, 2,
                                          {LineType::LEFT_WHITE, LineType::RIGHT_WHITE});
    all_segments.insert(all_segments.end(), white_segments.begin(), white_segments.end());

    // Detect yellow center line
    auto yellow_segments = fitLaneFromMask(yellow_mask, roi_y, 1,
                                           {LineType::YELLOW_CENTER});
    all_segments.insert(all_segments.end(), yellow_segments.begin(), yellow_segments.end());

    return all_segments;
}

std::vector<LineSegment> LaneMarkingNode::fitLaneFromMask(
    const cv::Mat& mask, int roi_y,
    int expected_lanes, const std::vector<LineType>& type_ids) {

    std::vector<LineSegment> segments;

    if (cv::countNonZero(mask) < 50) {
        return segments;
    }

    int h = mask.rows;
    int w = mask.cols;

    // Compute histogram on lower half
    std::vector<int> histogram = computeHistogram(mask, h / 2);

    // Find peaks
    std::vector<int> peaks;
    if (expected_lanes == 2) {
        int left_peak = findHistogramPeak(histogram, 0, w / 2);
        int right_peak = findHistogramPeak(histogram, w / 2, w);
        peaks.push_back(left_peak);
        peaks.push_back(right_peak);
    } else {
        int peak = findHistogramPeak(histogram, 0, w);
        peaks.push_back(peak);
    }

    // Get nonzero points
    std::vector<cv::Point> nonzero;
    cv::findNonZero(mask, nonzero);

    if (nonzero.empty()) {
        return segments;
    }

    int window_height = h / std::max(nwindows_, 1);

    for (size_t idx = 0; idx < peaks.size() && idx < type_ids.size(); ++idx) {
        int x_current = peaks[idx];
        std::vector<int> lane_x, lane_y;

        for (int window = 0; window < nwindows_; ++window) {
            int win_y_low = h - (window + 1) * window_height;
            int win_y_high = h - window * window_height;
            int win_x_low = std::max(0, x_current - window_margin_);
            int win_x_high = std::min(w, x_current + window_margin_);

            std::vector<int> good_x;

            for (const auto& pt : nonzero) {
                if (pt.y >= win_y_low && pt.y < win_y_high &&
                    pt.x >= win_x_low && pt.x < win_x_high) {
                    lane_x.push_back(pt.x);
                    lane_y.push_back(pt.y);
                    good_x.push_back(pt.x);
                }
            }

            if (static_cast<int>(good_x.size()) > minpix_) {
                double sum = std::accumulate(good_x.begin(), good_x.end(), 0.0);
                x_current = static_cast<int>(sum / good_x.size());
            }
        }

        if (lane_x.size() < 50) {
            continue;
        }

        // Fit polynomial
        std::vector<double> x_d(lane_x.begin(), lane_x.end());
        std::vector<double> y_d(lane_y.begin(), lane_y.end());
        Eigen::Vector3d poly = fitPolynomial2D(y_d, x_d);

        // Evaluate at top and bottom
        double y_bottom = static_cast<double>(h - 1);
        double y_top = std::max(static_cast<double>(h - window_height * nwindows_), 0.0);
        double x_bottom = evaluatePolynomial2D(poly, y_bottom);
        double x_top = evaluatePolynomial2D(poly, y_top);

        // Estimate style
        LineStyle style = estimateStyle(mask, lane_x, lane_y);

        segments.emplace_back(
            type_ids[idx], style,
            static_cast<float>(x_bottom), static_cast<float>(y_bottom + roi_y),
            static_cast<float>(x_top), static_cast<float>(y_top + roi_y),
            static_cast<float>(lane_x.size())
        );
    }

    return segments;
}

LineStyle LaneMarkingNode::estimateStyle(const cv::Mat& mask,
                                         const std::vector<int>& xs,
                                         const std::vector<int>& ys) {
    if (xs.empty()) {
        return LineStyle::UNKNOWN;
    }

    // Calculate effective area
    int effective_area = mask.rows * std::max(window_margin_ * 2, 1);
    double coverage = static_cast<double>(xs.size()) / effective_area;

    if (coverage > 0.35) {
        return LineStyle::SOLID;
    } else if (coverage > 0.12) {
        return LineStyle::DASHED;
    }
    return LineStyle::UNKNOWN;
}

StopLine LaneMarkingNode::detectStopLine(const cv::Mat& frame, int roi_y) {
    int h = frame.rows;
    int w = frame.cols;

    // Get stop line ROI (bottom portion)
    int stop_y_start = static_cast<int>(h * (1.0 - stop_roi_ratio_));
    cv::Mat stop_roi = frame(cv::Range(stop_y_start, h), cv::Range::all());

    // Convert to HSV and threshold white
    cv::Mat hsv;
    cv::cvtColor(stop_roi, hsv, cv::COLOR_BGR2HSV);
    cv::Mat white_mask = thresholdWhite(hsv);

    // Apply blur and threshold
    cv::Mat blurred;
    cv::GaussianBlur(white_mask, blurred, cv::Size(5, 5), 0);
    cv::Mat thresh;
    cv::threshold(blurred, thresh, 200, 255, cv::THRESH_BINARY);

    // Find contours
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(thresh, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    for (const auto& contour : contours) {
        cv::RotatedRect rect = cv::minAreaRect(contour);

        float width = rect.size.width;
        float height = rect.size.height;
        float angle = rect.angle;

        // Normalize so width > height
        if (width < height) {
            std::swap(width, height);
            angle += 90.0f;
        }

        // Filter criteria
        if (height < 5) continue;
        if (std::abs(angle) > 20) continue;

        float aspect = width / std::max(height, 1.0f);
        if (aspect < stop_line_min_aspect_) continue;
        if (width < stop_line_min_width_) continue;

        // Get bounding box
        cv::Rect bbox = cv::boundingRect(contour);

        // Calculate distance
        float stop_y = stop_y_start + bbox.y + bbox.height / 2.0f;
        float distance_pixels = h - stop_y;
        float distance_m = distance_pixels / static_cast<float>(stop_line_pixels_per_meter_);

        return StopLine(
            static_cast<float>(bbox.x),
            stop_y,
            static_cast<float>(bbox.x + bbox.width),
            stop_y,
            distance_m
        );
    }

    return StopLine();  // Invalid
}

bool LaneMarkingNode::trafficLightAllowsStop() const {
    for (const auto& state : allowed_stop_states_) {
        if (current_light_state_ == state) {
            return true;
        }
    }
    return false;
}

void LaneMarkingNode::publishLaneSegments(const std::vector<LineSegment>& segments) {
    std_msgs::msg::Float32MultiArray msg;

    // Layout: [type_id, style_id, x1, y1, x2, y2, score] per segment
    msg.layout.dim.resize(2);
    msg.layout.dim[0].label = "segments";
    msg.layout.dim[0].size = segments.size();
    msg.layout.dim[0].stride = 7;
    msg.layout.dim[1].label = "fields";
    msg.layout.dim[1].size = 7;
    msg.layout.dim[1].stride = 1;

    for (const auto& seg : segments) {
        msg.data.push_back(static_cast<float>(seg.type_id));
        msg.data.push_back(static_cast<float>(seg.style_id));
        msg.data.push_back(seg.p1.first);
        msg.data.push_back(seg.p1.second);
        msg.data.push_back(seg.p2.first);
        msg.data.push_back(seg.p2.second);
        msg.data.push_back(seg.score);
    }

    lane_pub_->publish(msg);
}

void LaneMarkingNode::publishStopLine(const StopLine& stop_line) {
    std_msgs::msg::Float32MultiArray msg;

    if (stop_line.valid) {
        msg.layout.dim.resize(2);
        msg.layout.dim[0].label = "stop_line";
        msg.layout.dim[0].size = 1;
        msg.layout.dim[0].stride = 5;
        msg.layout.dim[1].label = "fields";
        msg.layout.dim[1].size = 5;
        msg.layout.dim[1].stride = 1;

        msg.data.push_back(stop_line.x1);
        msg.data.push_back(stop_line.y1);
        msg.data.push_back(stop_line.x2);
        msg.data.push_back(stop_line.y2);
        msg.data.push_back(stop_line.distance_m);
    }

    stop_pub_->publish(msg);
}

}  // namespace perception_pkg

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    try {
        auto node = std::make_shared<perception_pkg::LaneMarkingNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("lane_marking_node"),
                     "Exception in lane_marking_node: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
