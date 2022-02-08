#pragma once

#include <map>
#include <opencv2/core.hpp>

#include "track.h"
#include "munkres.h"
#include "utils.h"

class Tracker {
public:
    Tracker();
    ~Tracker() = default;

    static double distanceCalculate(double x1, double y1, double x2, double y2);
    static float CalculateDistance(const My_RotatedRect& det, const Track& track);
    static float CalculateIou(const My_RotatedRect& det, const Track& track);
    static float ExpCost(const My_RotatedRect& det, const Track& track);
    static float LinCost(const My_RotatedRect& det, const Track& track);
    static float LandCost(const My_RotatedRect& det, const Track& track);

    static void HungarianMatching(const std::vector<std::vector<float>>& iou_matrix,
                           size_t nrows, size_t ncols,
                           std::vector<std::vector<float>>& association);

/**
 * Assigns detections to tracked object (both represented as bounding boxes)
 * Returns 2 lists of matches, unmatched_detections
 * @param detection
 * @param tracks
 * @param matched
 * @param unmatched_det
 * @param iou_threshold
 */
    static void AssociateDetectionsToTrackers(const std::vector<My_RotatedRect>& detection,
                                       std::map<int, Track>& tracks,
                                       std::map<int, My_RotatedRect>& matched,
                                       std::vector<My_RotatedRect>& unmatched_det,
                                       float iou_threshold = 0.1); // 0.2

    void Run(const std::vector<My_RotatedRect>& detections);

    std::map<int, Track> GetTracks();

private:
    // Hash-map between ID and corresponding tracker
    std::map<int, Track> tracks_;

    // Assigned ID for each bounding box
    int id_;
};