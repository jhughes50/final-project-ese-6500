/*
* Unit tests for add*Factor
* functions in the factor manager
*/
#include <iostream>
#include <gtest/gtest.h>
#include <Eigen/Dense>

#include "tanqueray/core/factor_manager.hpp"
#include "tanqueray/utils/params.hpp"

Params params;




TEST(FactorTestSuite, ImuInitialization)
{

    std::map<std::string, double> config;
    config = params.load<double>("/home/jason/ws/src/tanqueray/config/graph_params.yaml");

    Tanqueray::FactorManager manager(config);

    Eigen::Vector3d accel(0.0, 0.0, 0.0);
    Eigen::Vector3d gyro(0.0, 0.0, 0.0);
    Eigen::Vector4d orient(0.0, 0.0, 0.0, 1.0);

    manager.imuInitialize(accel, gyro, orient);
    
    EXPECT_EQ(manager.isInitialized(), false);
 
    for (int i=0; i <= config["bias_num_measurments"]+1; ++i)
    {
        manager.imuInitialize(accel, gyro, orient);
    }

    EXPECT_EQ(manager.isInitialized(), true);
}


TEST(FactorTestSuite, GPSFactor)
{
    // Test setup
    std::map<std::string, double> config;
    Tanqueray::FactorManager manager(config);

    int64_t time = 0;
    Eigen::Vector3d gps(39.941570, -75.199093, 0.0);

    size_t initial_factor_count = manager.getGraph().size();

    // pass to manager
    manager.addGpsFactor(time, gps);

    // TODO test graph
    gtsam::ExpressionFactorGraph graph = manager.getGraph();

    std::cout << "inital size: " << initial_factor_count;
    std::cout << " graph size: " << manager.getGraph().size() << std::endl;

    EXPECT_EQ(manager.getGraph().size(), initial_factor_count);

    std::cout << "key index: " << manager.getKeyIndex<int>() << std::endl;
    EXPECT_EQ(manager.getKeyIndex<int>(), 0);
}


TEST(FactorTestSuite, IMUFactor)
{

}


//TEST(FactorTestSuite, OdomFactor)
//{
//    
//}
