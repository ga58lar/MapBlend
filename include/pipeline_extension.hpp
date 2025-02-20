// include/pipeline_extension.hpp
#pragma once

#include "pipeline/openlidarmap.hpp"
#include "io/loader_factory.hpp" // Add this include
#include "utils/file_utils.hpp"
#include "core/prediction.hpp"  // For ConstantDistancePredictor
#include "utils/pose_utils.hpp"
#include <memory>

namespace openlidarmap::pipeline {

class PipelineAdapter {
public:
    PipelineAdapter(const config::Config& config) 
        : pipeline_(config) {}

    bool initialize(const std::string &map_path,
                   const std::string &scans_dir, 
                   const std::string &output_path,
                   const Vector7d &initial_pose) {
        return pipeline_.initialize(map_path, scans_dir, output_path, initial_pose);
    }

    bool process(small_gicp::PointCloud::Ptr &frame) {
        return pipeline_.processFrame(frame);
    }

    void writeResults() const {
        pipeline_.writeResults();
    }

private:
    Pipeline pipeline_;
};

}