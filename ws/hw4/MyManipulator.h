#pragma once // includes are included only once
#include "AMPCore.h"
#include <cmath>
#include <iostream>
using std::vector, Eigen::Vector2d, std::cout;
 
namespace amp
{
    class MyManipulator : public LinkManipulator2D
    {
    public:
        MyManipulator();
        MyManipulator(const std::vector<double> &link_lengths);
        MyManipulator(const Eigen::Vector2d &base_location, const std::vector<double> &link_lengths);
        Eigen::Vector2d getJointLocation(const ManipulatorState &state, uint32_t joint_index) const override;
        ManipulatorState getConfigurationFromIK(const Eigen::Vector2d &end_effector_location) const override;
 
    };
}