#pragma once 
#include "AMPCore.h"
#include "hw/HW5.h"




class MyGDAlgorithm : public amp::GDAlgorithm { 
    public:
        MyGDAlgorithm(double eta_repmag, double Q_star, double NoiseScalar, double d_star, double zeta_attract, double alpha); 

        virtual amp::Path2D plan(const amp::Problem2D& problem) override;
        virtual std::pair<double, Eigen::Vector2d> findMinimumPerpendicularDistance(const Eigen::Vector2d& Point, const amp::Obstacle2D& polygon ) ;
        virtual std::pair<Eigen::Vector2d, double> ClosePointPolygon_NearestPointFrom(const Eigen::Vector2d& p, const amp::Obstacle2D& polygon) ;
        virtual Eigen::Vector2d AddNoise(const double& scalar);
        Eigen::Vector2d calculateGradient(Eigen::Vector2d& currentPoint, const amp::Problem2D& problem);
        // Calculate the gradient based problem



        bool isConverged(Eigen::Vector2d& currentPoint, const Eigen::Vector2d& goalPoint, const double& epsilon);
            // Check for convergence based on  criteria
            // check if the distance between currentPoint and goalPoint is less than tolerance.
    


        double findDistance(const Eigen::Vector2d& point1,const Eigen::Vector2d& point2);
            // find distance between two points defined as vectors
    private:
        double eta_repmag;
        double Q_star;
        double NoiseScalar;
        double d_star;
        double zeta_attract;
        double alpha;

        

};










// first determine, your initial configuration (q_init)
// second, given a desired point in workspace (q_goal)
// create the attractive potential field for the Bug Algorithm 
//// maybe have a move forward method 
// call in thw problem from HW 2 and HW to locate the obstacles 
// create a repulsive potential field method 
// Use gradient descent algorithm to reach your target:
// 1. int i = 0; q_start = q0 
// 2. if ||q_ith - q_final|| > epsilon 
//// 3. q_i+1  = q_i + alpha * tao(q_i)/||tao(q_i)||
//// 4. i++
// 4. else 
//// 5. return <q0, q1, q2, ..., q_th>



