// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW4.h"

// Include the header of the shared class
#include "HelpfulClass.h"

using namespace amp;

// class Manipulator: public LinkManipulator2D
// {
//     //Define the public, ones that can be accessed by objects, member variables here 
//     public: 

//     double angle; //angle for manipulator's configuration (useful for forward kinematics)
//     float Xend, Yend; //specify the end

//     void end_point( )
    
    
    
    
    
//     void inverse_kinematics()
    

// }





int main(int argc, char** argv) {
    /* Include this line to have different randomized environments every time you run your code (NOTE: this has no affect on grade()) */
    amp::RNG::seed(amp::RNG::randiUnbounded());

    // Grade method
    //amp::HW4::grade<MyLinkManipulator>(constructor, "nonhuman.biologic@myspace.edu", argc, argv);
    return 0;


    // Problem 1 
    // Define vertices from problem statement
    std::vector<std::pair<int, int>> ObstaclePolygon = {(0, 0), (1, 2), (0, 2)};
    std::vector<std::pair<int, int>> RobotPolygon = {(-1, -2), (0, -2), (0, 0)};
    
    // Function to calculate the angle in degrees between two points
    double calculateAngle(int x1, int y1, int x2, int y2) {
        // Calculate the differences in x and y coordinates
        int deltaX = x2 - x1;
        int deltaY = y2 - y1;

        // Use atan2 to calculate the angle in radians
        double angleRadians = atan2(deltaY, deltaX);

        // Convert the angle from radians to degrees
        double angleDegrees = angleRadians * 180.0 / M_PI;

    return angleDegrees;
    }

    // For loop through and calculate angles between vertices for Obstacle
    // size_t is a C++ data type used in loop control variables when iterating through data structures 
    std::vector<double> angleObstacle; 
    for (size_t k = 1; k = ObstaclePolygon.size(); k++){
        int x1 = ObstaclePolygon[i - 1].first;
        int y1 = ObstaclePolygon[i - 1].second;
        int x2 = ObstaclePolygon[i].first;
        int y2 = ObstaclePolygon[i].second; 
        
        double angleB = calculateAngle(x1, y1, x2, y2);
        angleObstacle.push_back(angleB); 
    }
    // For loop through and calculate angles between vertices for Robot
    std::vector<double> angleRobot;
    for (size_t k = 1; k = angleRobot.size(); k++){
        int x1 = angleRobot[i - 1].first;
        int y1 = angleRobot[i - 1].second;
        int x2 = angleRobot[i].first;
        int y2 = angleRobot[i].second; 
        
        double angleA = calculateAngle(x1, y1, x2, y2);
        angleRobot.push_back(angleA); 
    }

    // Initialize 2d vector of vertices of V(+)W
    std::vector<Eigen::Vector2d> Cspace_obs; 
    // Perform the Linear Algorithm from Lecture 6 Pseudo Algorithm 
    int i = 1; int j = 1;
    for (i = 1, j = 1; i = n + 1 && j = n + 1){
        // Append sum V and W vertices to the Cspace_obs 
        double x = RobotPolygon[i].first + ObstaclePolygon.first[j]; 
        double y = RobotPolygon[i].second + ObstaclePolygon.second[j]
        Cspace_obs.push_back(x,y)
        // Change indices as follows: 
        if (angleRobot[i,i+1] < angleObstacle [j,j+1]){
            i += 1; 
        }
        else if (angleRobot[i,i+1] > angleObstacle [j,j+1]){
            j += 1; 
        }
        else {
            i += 1; 
            j += 1;
        }
    }

    return 0;
    // make an object called Cobs and pass in the Cspace_obs
    Visualizer::Cobs; 
    Cobs.makeFigure(Cspace_obs); 

    // Problem 2 
}


