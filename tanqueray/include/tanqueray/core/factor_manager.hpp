/*
* April 2025
* Manage the Factor Graph
*/

#pragma once

#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/ExpressionFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/slam/InitializePose3.h>

#include "imu_buffer.hpp"

#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include <map>
#include <cmath>
#include <numeric>
#include <tuple>

// Symbol shorthand
using gtsam::symbol_shorthand::B; // Bias
using gtsam::symbol_shorthand::V; // Velocity
using gtsam::symbol_shorthand::X; // Pose

// Helper function declarations
Eigen::Vector3d vector3(double x, double y, double z);
double nanosecInt2Float(int64_t timestamp);

namespace Tanqueray 
{

class FactorManager
{
    public:
        FactorManager() = default;
        FactorManager(const std::map<std::string, double>& config);
        
        static boost::shared_ptr<gtsam::PreintegrationCombinedParams> defaultParams(double g);
        
        void initializeGraph();
        void imuInitialize(const Eigen::Vector3d& accel_meas, const Eigen::Vector3d& gyro_meas, const Eigen::Vector4d& orient);

        std::tuple<Eigen::Vector3d, Eigen::Vector4d> predict(int64_t timestamp); 

        void addGpsFactor(int64_t timestamp, const Eigen::Vector3d& gps);
        void addOdometryFactor(int64_t timestamp, const Eigen::Vector3d& pose, const Eigen::Vector4d& quat);
        void addImuFactor(int64_t timestamp, const Eigen::Vector3d& accel, const Eigen::Vector3d& gyro, const Eigen::Vector4d& orient);
                         
        gtsam::Values optimize();  
        std::tuple<Eigen::Vector3d, Eigen::Vector4d, Eigen::Matrix3d> runner();

        gtsam::ExpressionFactorGraph getGraph();
        bool isInitialized();

        template <typename T>
        T getKeyIndex();

    private:
        std::map<std::string, double> config;
        std::map<std::string, Eigen::MatrixXd> matrix_config;
        
        Eigen::Vector3d gravity_vec;
        Eigen::MatrixXd bias_estimate_vec;
        
        int init_counter;
        Eigen::Matrix3d imu2body;
        
        bool _initialized;
        
        gtsam::Key _key_index;
    
        double _lastOptimizeTime;
        double _lastImuTime;
        double _last_gps_time;

        ImuBuffer _imu_buffer;

        boost::shared_ptr<gtsam::PreintegrationCombinedParams> params;
        
        gtsam::noiseModel::Isotropic::shared_ptr _prior_noise;
        gtsam::noiseModel::Isotropic::shared_ptr _odom_noise;
        gtsam::noiseModel::Isotropic::shared_ptr _gps_noise;
        gtsam::ExpressionFactorGraph _graph;
        gtsam::GaussNewtonParams _params;
        gtsam::Values _initials;
        gtsam::ISAM2Params _parameters;
        gtsam::ISAM2 _isam;
        
        Eigen::Vector4d _orientation;
        
        gtsam::Point3 _translation;
        gtsam::Pose3 optimized_pose;
        gtsam::Point3 last_velocity;
	    gtsam::Matrix last_marginal_covariance;
        gtsam::imuBias::ConstantBias bias;
        
        boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements> pim;
        boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements> pim_copy;
        
        gtsam::Rot3 initial_orientation;
        gtsam::Pose3 navstate_pose;
        gtsam::NavState init_navstate;
        gtsam::NavState lastNavState;
        gtsam::GaussNewtonOptimizer* _optimizer;
        
        Eigen::Vector3d _last_accel_meas;
        Eigen::Vector3d _last_gyro_meas;
    	Eigen::Vector4d _orient;
};
}
