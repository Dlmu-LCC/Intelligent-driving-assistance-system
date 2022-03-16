#ifndef LANE_DETECTOR_H
#define LANE_DETECTOR_H

#include <getopt.h>

#include <iostream>
#include <string>
#include <vector>

#include "perception/lane_detection/lane_line.h"
#include "perception/common/uff_models/unet/unet.h"
#include "utils/timer.h"

class LaneDetector {
   private:
    std::shared_ptr<Unet> model;

    std::vector<Timer::time_point_t> dual_line_checking_time;
    std::vector<bool> is_dual_line;

   public:
    bool ready = false;

    LaneDetector();

    cv::Mat getLaneMask(const cv::Mat& input_img);

    // Lane detect function
    // For debug purpose
    std::vector<LaneLine> detectLaneLines(const cv::Mat& input_img,
                                                cv::Mat& line_mask,
                                                cv::Mat& detected_lines_img,
                                                cv::Mat& reduced_lines_img,
                                                bool &lane_departure);
    // For general usage
    std::vector<LaneLine> detectLaneLines(const cv::Mat& img, bool &lane_departure);

   private:
    // Utils functions
    std::vector<cv::Vec4i> detectAndReduceLines(const cv::Mat& img,
                                                cv::Mat& detected_lines_img,
                                                cv::Mat& reduced_linesImg);
    static cv::Vec2d linearParameters(cv::Vec4i line);
    static cv::Vec4i extendedLine(cv::Vec4i line, double d);
    static std::vector<cv::Point2i> boundingRectangleContour(cv::Vec4i line,
                                                             float d);
    static bool extendedBoundingRectangleLineEquivalence(
        const cv::Vec4i& _l1, const cv::Vec4i& _l2,
        float extensionLengthFraction, float maxAngleDiff,
        float boundingRectangleThickness);
    void getLinePointinImageBorder(const cv::Point &p1_in,
        const cv::Point &p2_in,
        cv::Point &p1_out,
        cv::Point &p2_out, int rows,
        int cols);
};

#endif