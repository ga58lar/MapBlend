// main.cpp
#include <iostream>
#include <filesystem>
#include <yaml-cpp/yaml.h>
#include "include/mapblend.hpp"
#include "lidar_centerpoint/centerpoint_trt.hpp"
#include "lidar_centerpoint/utils.hpp"
#include "lidar_centerpoint/centerpoint_config.hpp"
#include "lidar_centerpoint/preprocess/pointcloud_densification.hpp"

using namespace centerpoint;

// Configuration structs and loading functions
struct ModelConfig {
    std::vector<std::string> class_names;
    int point_feature_size;
    int max_voxel_size;
    std::vector<double> point_cloud_range;
    std::vector<double> voxel_size;
    int downsample_factor;
    int encoder_in_feature_size;
    bool has_variance;
    bool has_twist;
};

struct NetworkConfig {
    std::string encoder_onnx_path;
    std::string encoder_engine_path;
    std::string head_onnx_path;
    std::string head_engine_path;
    std::string trt_precision;
    double score_threshold;
    double circle_nms_dist_threshold;
    std::vector<double> yaw_norm_thresholds;
};

struct DensificationConfig {
    int num_past_frames;
};

ModelConfig loadModelConfig(const YAML::Node& config) {
    ModelConfig model_cfg;
    const auto& ros_params = config["/**"]["ros__parameters"];
    const auto& model_params = ros_params["model_params"];
    
    model_cfg.class_names = model_params["class_names"].as<std::vector<std::string>>();
    model_cfg.point_feature_size = model_params["point_feature_size"].as<int>();
    model_cfg.max_voxel_size = model_params["max_voxel_size"].as<int>();
    model_cfg.point_cloud_range = model_params["point_cloud_range"].as<std::vector<double>>();
    model_cfg.voxel_size = model_params["voxel_size"].as<std::vector<double>>();
    model_cfg.downsample_factor = model_params["downsample_factor"].as<int>();
    model_cfg.encoder_in_feature_size = model_params["encoder_in_feature_size"].as<int>();
    model_cfg.has_variance = model_params["has_variance"].as<bool>();
    model_cfg.has_twist = model_params["has_twist"].as<bool>();
    return model_cfg;
}

NetworkConfig loadNetworkConfig(const YAML::Node& config) {
    NetworkConfig net_cfg;
    const auto& ros_params = config["/**"]["ros__parameters"];
    
    net_cfg.encoder_onnx_path = ros_params["encoder_onnx_path"].as<std::string>();
    net_cfg.encoder_engine_path = ros_params["encoder_engine_path"].as<std::string>();
    net_cfg.head_onnx_path = ros_params["head_onnx_path"].as<std::string>();
    net_cfg.head_engine_path = ros_params["head_engine_path"].as<std::string>();
    net_cfg.trt_precision = ros_params["trt_precision"].as<std::string>();
    
    const auto& post_process = ros_params["post_process_params"];
    net_cfg.score_threshold = post_process["score_threshold"].as<double>();
    net_cfg.circle_nms_dist_threshold = post_process["circle_nms_dist_threshold"].as<double>();
    net_cfg.yaw_norm_thresholds = post_process["yaw_norm_thresholds"].as<std::vector<double>>();
    return net_cfg;
}

DensificationConfig loadDensificationConfig(const YAML::Node& config) {
    DensificationConfig dense_cfg;
    const auto& ros_params = config["/**"]["ros__parameters"];
    const auto& dense_params = ros_params["densification_params"];
    dense_cfg.num_past_frames = dense_params["num_past_frames"].as<int>();
    return dense_cfg;
}

CenterPointConfig createCenterPointConfig(const ModelConfig& model_cfg, const NetworkConfig& net_cfg) {
    return CenterPointConfig(
        model_cfg.class_names.size(),
        model_cfg.point_feature_size,
        model_cfg.max_voxel_size,
        model_cfg.point_cloud_range,
        model_cfg.voxel_size,
        model_cfg.downsample_factor,
        model_cfg.encoder_in_feature_size,
        net_cfg.score_threshold,
        net_cfg.circle_nms_dist_threshold,
        net_cfg.yaw_norm_thresholds,
        model_cfg.has_variance
    );
}

int main(int argc, char** argv) {
    if (argc != 5 && argc != 12) {
        std::cout << "Usage: " << argv[0] 
                  << " <config_dir> <map_path> <scans_dir> <output_path> "
                  << "[x y z qx qy qz qw]" << std::endl;
        return 1;
    }

    try {
        // Load configurations
        const fs::path config_dir(argv[1]);
        auto model_config = YAML::LoadFile((config_dir / "centerpoint_ml_package.param.yaml").string());
        auto network_config = YAML::LoadFile((config_dir / "centerpoint.param.yaml").string());
        auto dense_config = YAML::LoadFile((config_dir / "centerpoint.param.yaml").string());

        // Initialize configs
        auto model_cfg = loadModelConfig(model_config);
        auto network_cfg = loadNetworkConfig(network_config);
        auto dense_cfg = loadDensificationConfig(dense_config);
        auto centerpoint_cfg = createCenterPointConfig(model_cfg, network_cfg);

        // Setup network parameters
        NetworkParam encoder_param(network_cfg.encoder_onnx_path,
                                network_cfg.encoder_engine_path,
                                network_cfg.trt_precision);
        NetworkParam head_param(network_cfg.head_onnx_path,
                              network_cfg.head_engine_path,
                              network_cfg.trt_precision);
        DensificationParam dense_param(dense_cfg.num_past_frames);

        // Initialize OpenLidarMap config with safe defaults
        openlidarmap::config::Config olm_config{};
        olm_config.preprocess_.min_range = 7.5;
        olm_config.preprocess_.max_range = 50.0;

        // Parse initial pose if provided
        openlidarmap::Vector7d initial_pose;
        if (argc == 12) {
            for (int i = 0; i < 7; ++i) {
                initial_pose[i] = std::stod(argv[i + 5]);
            }
        } else {
            initial_pose << 0, 0, 0, 0, 0, 0, 1;  // Default pose
        }
        
        // Verify map file exists
        if (!fs::exists(argv[2])) {
            std::cerr << "Map file not found: " << argv[2] << std::endl;
            return 1;
        }

        // Create and run pipeline
        MapBlendPipeline pipeline(olm_config, centerpoint_cfg, 
                                encoder_param, head_param, dense_param);

       if (!pipeline.initialize(argv[2], argv[3], argv[4], initial_pose)) {
            std::cerr << "Failed to initialize pipeline" << std::endl;
            return 1;
        }

        if (!pipeline.run()) {
            std::cerr << "Pipeline execution failed" << std::endl;
            return 1;
        }

        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
}