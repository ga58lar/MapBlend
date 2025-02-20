#pragma once

#include <iostream>
#include <filesystem>
#include "config/config.hpp"
#include "pipeline_extension.hpp"
#include "lidar_centerpoint/centerpoint_trt.hpp"
#include "lidar_centerpoint/centerpoint_config.hpp" 
#include "io/loader_factory.hpp"
#include "utils/file_utils.hpp"

#include <guik/viewer/async_light_viewer.hpp>

class MapBlendPipeline;

namespace openlidarmap::pipeline {
    class Pipeline; 

class MapBlendPipeline {
public:
    MapBlendPipeline(const openlidarmap::config::Config& olm_config,
                     const centerpoint::CenterPointConfig& cp_config,
                     const centerpoint::NetworkParam& encoder_param,
                     const centerpoint::NetworkParam& head_param,
                     const centerpoint::DensificationParam& dense_param)
        : config_(olm_config),
          olm_pipeline_(olm_config),
          detector_(std::make_unique<centerpoint::CenterPointTRT>(
              encoder_param, head_param, dense_param, cp_config)) {
                async_viewer_ = guik::async_viewer();
              }

    bool initialize(const std::string& map_path, 
                   const std::string& scans_dir,
                   const std::string& output_path,
                   const openlidarmap::Vector7d& initial_pose) {
        scans_dir_ = scans_dir;
        return olm_pipeline_.initialize(map_path, scans_dir, output_path, initial_pose);
    }

    bool run() {
        std::vector<std::string> scan_files = openlidarmap::utils::FileUtils::getFiles(scans_dir_);
        
        for (const auto& scan_file : scan_files) {
            // 1. Load PCD file
            auto cloud = openlidarmap::io::LoaderFactory::loadPointCloud(config_, scan_file);
            
            // 2. Convert to CenterPoint format and detect objects
            std::vector<Eigen::Vector4f> points;
            points.reserve(cloud->points.size());
            for (const auto& pt : cloud->points) {
                points.emplace_back(pt[0], pt[1], pt[2], 1.0f);
            }

            std::vector<centerpoint::Box3D> objects;
            Eigen::Isometry3f tf = Eigen::Isometry3f::Identity();
            if (!detector_->detect(points, tf, 0.0, objects)) {
                std::cerr << "Detection failed for " << scan_file << std::endl;
                continue;
            }

            // 3. Filter points inside boxes
            std::vector<Eigen::Vector3f> inside_points, outside_points;
            filterPointsByBoxes(points, objects, inside_points, outside_points);

            // 4. Create filtered point cloud
            auto filtered_cloud = std::make_shared<small_gicp::PointCloud>();
            filtered_cloud->points.reserve(outside_points.size());
            for (const auto& pt : outside_points) {
                filtered_cloud->points.emplace_back(pt[0], pt[1], pt[2], 1.0);
            }

            // 5. Process filtered frame through registration pipeline
            if (!processFrame(filtered_cloud)) {
                std::cerr << "Scan matching failed for " << scan_file << std::endl;
                continue;
            }

            // 6. Visualize results
            visualizeFrame(points, objects, inside_points, outside_points);
        }

        // Write final results
        olm_pipeline_.writeResults();
        return true;
    }

private:
    bool processFrame(std::shared_ptr<small_gicp::PointCloud> cloud);
    void filterPointsByBoxes(const std::vector<Eigen::Vector4f>& points,
                            const std::vector<centerpoint::Box3D>& boxes,
                            std::vector<Eigen::Vector3f>& inside_points,
                            std::vector<Eigen::Vector3f>& outside_points);
    bool isPointInBox(const Eigen::Vector3f& point,
                     const Eigen::Vector3f& center,
                     const Eigen::Vector3f& dims,
                     const Eigen::Quaternionf& quat);
    void visualizeFrame(const std::vector<Eigen::Vector4f>& points,
                       const std::vector<centerpoint::Box3D>& boxes,
                       const std::vector<Eigen::Vector3f>& inside_points,
                       const std::vector<Eigen::Vector3f>& outside_points);

    std::string scans_dir_;
    openlidarmap::config::Config config_;
    openlidarmap::pipeline::PipelineAdapter olm_pipeline_;
    std::unique_ptr<centerpoint::CenterPointTRT> detector_;

    guik::AsyncLightViewer* async_viewer_;
};


bool MapBlendPipeline::processFrame(std::shared_ptr<small_gicp::PointCloud> cloud) {
    return olm_pipeline_.process(cloud);
}

void MapBlendPipeline::filterPointsByBoxes(const std::vector<Eigen::Vector4f>& points,
                        const std::vector<centerpoint::Box3D>& boxes,
                        std::vector<Eigen::Vector3f>& inside_points,
                        std::vector<Eigen::Vector3f>& outside_points) {
    inside_points.clear();
    outside_points.clear();
    inside_points.reserve(points.size());
    outside_points.reserve(points.size());

    for (const auto& pt : points) {
        Eigen::Vector3f point = pt.head<3>(); 
        bool inside = false;
        for (const auto& box : boxes) {
            Eigen::Vector3f center(box.x, box.y, box.z);
            Eigen::Vector3f dims(box.length, box.width, box.height);
            Eigen::Quaternionf quat(Eigen::AngleAxisf(-box.yaw - M_PI_2, Eigen::Vector3f::UnitZ()));
            
            if (isPointInBox(point, center, dims, quat)) {
                inside = true;
                break;
            }
        }
        (inside ? inside_points : outside_points).push_back(point);
    }
}

bool MapBlendPipeline::isPointInBox(const Eigen::Vector3f& point,
                    const Eigen::Vector3f& center,
                    const Eigen::Vector3f& dims,
                    const Eigen::Quaternionf& quat) {
    Eigen::Vector3f local_point = quat.inverse() * (point - center);
    return (std::abs(local_point[0]) <= dims[0]/2.0f &&
            std::abs(local_point[1]) <= dims[1]/2.0f &&
            std::abs(local_point[2]) <= dims[2]/2.0f);
}

void MapBlendPipeline::visualizeFrame(const std::vector<Eigen::Vector4f>& points,
                    const std::vector<centerpoint::Box3D>& boxes,
                    const std::vector<Eigen::Vector3f>& inside_points,
                    const std::vector<Eigen::Vector3f>& outside_points) {
    if (!async_viewer_) return;

    async_viewer_->update_points("dynamic", inside_points,
                                guik::FlatRed(Eigen::Isometry3d::Identity()));
    async_viewer_->update_points("static", outside_points,
                                guik::FlatWhite(Eigen::Isometry3d::Identity()));
    
    std::cout << "Frame stats:\n"
                << "Total points: " << points.size() << "\n"
                << "Detected boxes: " << boxes.size() << "\n"
                << "Points inside boxes: " << inside_points.size() << "\n"
                << "Points outside boxes: " << outside_points.size() << std::endl;
}
