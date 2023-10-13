// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW4.h"
#include "MyManipulator.h"

// Include the header of the shared class
#include "HelpfulClass.h"
using namespace amp;
#include <cmath>
#include <iostream>
using std::vector, Eigen::Vector2d, std::cout;

//////////////////////////////////////////////////////////////////////////
//                          problem 1 functions 

// Custom atan2 function that returns a positive angle from horizontal
double atan2_positive(double y, double x) {
    double angle = std::atan2(y, x); // Calculate the angle using atan2

    // Ensure the angle is positive
    if (angle < 0) {
        angle += 2 * M_PI; // Add 360 degrees in radians to make it positive
    }

    return angle;
}
// Calculating angles between link function
double calculateAngle(int x1, int y1, int x2, int y2) {
    // Calculate the differences in x and y coordinates
    int deltaX = x2 - x1;
    int deltaY = y2 - y1;
    
    // Use atan2 to calculate the angle in radians
    double angleRadians = atan2_positive(deltaY, deltaX);
    
    // Convert the angle from radians to degrees
    double angleDegrees = angleRadians * 180.0 / M_PI;
    
    return angleDegrees;
}

void problem1a(){
    //////////////////////////////// Problem 1 ////////////////////////////////////////////////////////////////////
    //Define vertices from problem statement
    std::vector<std::pair<int, int>> ObstaclePolygon = {{0, 0}, {1, 2}, {0, 2},{0, 0}};
    std::vector<std::pair<int, int>> RobotPolygon = {{-1, -2}, {0, -2}, {0, 0},{-1, -2}};
    //std::cout << "Hello, Peter (frown face)1" << std::endl;


    // For loop through and calculate angles between vertices for Obstacle
    // size_t is a C++ data type used in loop control variables when iterating through data structures
    std::vector<double> angleObstacle;
    for (int k = 0; k < ObstaclePolygon.size(); k++){
        int x1 = ObstaclePolygon[k].first;
        int y1 = ObstaclePolygon[k].second;
        int x2 = ObstaclePolygon[k + 1].first;
        int y2 = ObstaclePolygon[k + 1].second;
        //std::cout << "Obs point: (" << x1 << ", " << y1 << ")\n"; 

        double angleB = calculateAngle(x1, y1, x2, y2);
        //std::cout << "Obs angle: (" << angleB << ")\n"; 
        angleObstacle.push_back(angleB);
    }
     
    //std::cout << "Hello, Peter (frown face)2" << std::endl;

    // For loop through and calculate angles between vertices for Robot
    std::vector<double> angleRobot;
    for (int k = 0; k < ObstaclePolygon.size(); k++){
        int x1 = RobotPolygon[k].first;
        int y1 = RobotPolygon[k].second;
        int x2 = RobotPolygon[k + 1].first;
        int y2 = RobotPolygon[k + 1].second;
        //std::cout << "Robot point: (" << x1 << ", " << y1 << ")\n"; 

        double angleA = calculateAngle(x1, y1, x2, y2);
        //std::cout << "Robot angle: (" << angleA << ")\n"; 
        angleRobot.push_back(angleA);
    }
    //std::cout << "Hello, Peter (frown face)3" << std::endl;

   // Perform the Linear Algorithm from Lecture 6 Pseudo Algorithm 
    std::vector<Eigen::Vector2d> Cspace_obs; // Initialize 2d vector of vertices of V(+)W
    int n = RobotPolygon.size() ;
    int m = ObstaclePolygon.size() ;
    int i = 0; int j = 0;
    while (i < n && j < m) {
        double x = RobotPolygon[i].first + ObstaclePolygon[j].first;
        double y = RobotPolygon[i].second + ObstaclePolygon[j].second;
        Cspace_obs.push_back(Eigen::Vector2d(x, y));
        std::cout << "Cspace_obs point: (" << x << ", " << y << ")\n"; 
        // Change indices as follows:
        if (angleRobot[i] < angleObstacle[j]) {
            i++;
        }
        else if (angleRobot[i] > angleObstacle[j]) {
            j++;
        }
        else {
            i++;
            j++;
        }
    }
    std::cout << "Hello, Peter (frown face)4" << std::endl;

    std::vector<double> height;
    height.push_back(0);
    std::vector<amp::Polygon> Cspace_Polygons; // Vector to store rotated polygons
    // Store the original Cspace_obs as the first element of Cspace_Polygons
    Cspace_Polygons.push_back(amp::Polygon(Cspace_obs));
    double rotation_angle = M_PI / 6.0;
    //int vertices = Cspace_obs.size(); 
    int num_rotations = 12;
    for (int i = 1; i < num_rotations; ++i) {
        std::vector<Eigen::Vector2d> rotated_vertices;

        for (const Eigen::Vector2d& vertex : Cspace_obs) {
            double x = vertex.x();
            double y = vertex.y();

            // Apply rotation
            //std::cout << "rotation angle is: " << rotation_angle << " times " << i << "radians\n";
            double new_x = x * cos(rotation_angle*i) - y * sin(rotation_angle*i);
            double new_y = x * sin(rotation_angle*i) + y * cos(rotation_angle*i);

            rotated_vertices.push_back(Eigen::Vector2d(new_x, new_y));
        }

        // Construct an amp::Polygon object and store it
        amp::Polygon Cspace_poly(rotated_vertices);
        Cspace_Polygons.push_back(Cspace_poly);
        height.push_back(i);
    }
    std::cout << "Hello, Peter (frown face) Problem 1 almost done" << std::endl;

    
    // make an object called Cobs and pass in the Cspace_obs
    Visualizer::makeFigure(Cspace_Polygons, height);
    Visualizer::showFigures(); 
    std::cout << "Hello, Peter (frown face) Problem 1 done" << std::endl;
    
    //return 0;

}

///////////////////////////////////////////////////////////////////////////
// Function to find the lower-left vertex and rearrange vertices counter-clockwise
std::vector<std::pair<double, double>> rearrangeVertices(const std::vector<std::pair<double, double>>& vertices) {
    std::vector<std::pair<double, double>> rearrangedVertices = vertices;
    // Check if the vertices vector is empty
    if (vertices.empty()) {
        std::cerr << "Vertices vector is empty." << std::endl;
        return {}; // Return an empty vector when vertices are empty
    }

    // Find the lower-left vertex based on (x, y) coordinates
    std::pair<double, double> lowerLeftVertex = vertices[0];
    int lowerLeftIndex = 0;

    for (int i = 1; i < vertices.size(); ++i) {
        const auto& vertex = vertices[i];
        if (vertex.first < lowerLeftVertex.first ||
            (vertex.first == lowerLeftVertex.first && vertex.second < lowerLeftVertex.second)) {
            lowerLeftVertex = vertex;
            lowerLeftIndex = i;
        }
    }

    // Rearrange vertices starting from the lower-left index counter-clockwise
    std::vector<std::pair<double, double>> rearrangedVerticesResult; // Choose a different name for the result vector
    for (int i = lowerLeftIndex; i < vertices.size(); ++i) {
        rearrangedVerticesResult.push_back(vertices[i]);
    }
    for (int i = 0; i < lowerLeftIndex; ++i) {
        rearrangedVerticesResult.push_back(vertices[i]);
    }

    // Update the original vertices vector with the rearranged vertices
    return rearrangedVerticesResult; // Return the result vector
}

std::vector<std::pair<double, double>> applyRotations(const std::vector<std::pair<double, double>>& points, double rotation_angle, int num_rotations) {
    std::vector<std::pair<double, double>> rotatedPoints = points;

    for (int i = 0; i < num_rotations; ++i) {
        std::vector<std::pair<double, double>> newRotatedPoints;

        for (const std::pair<double, double>& point : rotatedPoints) {
            double x = point.first;
            double y = point.second;

            // Apply rotation
            double new_x = x * cos(rotation_angle) - y * sin(rotation_angle);
            double new_y = x * sin(rotation_angle) + y * cos(rotation_angle);

            newRotatedPoints.push_back(std::make_pair(new_x, new_y));
        }

        rotatedPoints = newRotatedPoints;
    }

    return rotatedPoints;
}

void problem1b() {
    
//////////////////////////////// Problem 1b ////////////////////////////////////////////////////////////////////
    //Define vertices from problem statement
    std::vector<std::pair<double, double>> ObstaclePolygon = {{0.0, 0.0}, {1.0, 2.0}, {0.0, 2.0},{0.0, 0.0}};
    std::vector<std::pair<double, double>> RobotPolygon = {{-1.0, -2.0}, {0.0, -2.0}, {0.0, 0.0},{-1.0, -2.0}};
    // Specify the number of rotations (change this as needed)
    int numRotations = 12;
    std::vector<double> height; 
    //height.push_back(0);
    double theta_height; // initialize this to then append to height
    std::vector<amp::Polygon> Cspace_Polygons; // Vector to store rotated polygons
    
    for (int i = 0; i < numRotations; ++i) {
        // Apply incremental rotations
        RobotPolygon = applyRotations(RobotPolygon, M_PI/12, 1);

        // Rearrange the vertices based on the lower-left vertex and counter-clockwise order
        RobotPolygon = rearrangeVertices(RobotPolygon);

        // For loop through and calculate angles between vertices for Obstacle
        std::vector<double> angleObstacle;
        for (int k = 0; k < ObstaclePolygon.size(); k++){
            double x1 = ObstaclePolygon[k].first;
            double y1 = ObstaclePolygon[k].second;
            double x2 = ObstaclePolygon[k + 1].first;
            double y2 = ObstaclePolygon[k + 1].second;
            //std::cout << "Obs point: (" << x1 << ", " << y1 << ")\n"; 

            double angleB = calculateAngle(x1, y1, x2, y2);
            //std::cout << "Obs angle: (" << angleB << ")\n"; 
            angleObstacle.push_back(angleB);
        }
        
        // For loop through and calculate angles between vertices for Robot
        std::vector<double> angleRobot;
        for (int k = 0; k < ObstaclePolygon.size(); k++){
            double x1 = RobotPolygon[k].first;
            double y1 = RobotPolygon[k].second;
            double x2 = RobotPolygon[k + 1].first;
            double y2 = RobotPolygon[k + 1].second;
            //std::cout << "Robot point: (" << x1 << ", " << y1 << ")\n"; 

            double angleA = calculateAngle(x1, y1, x2, y2);
            //std::cout << "Robot angle: (" << angleA << ")\n"; 
            angleRobot.push_back(angleA);
        }

        // Perform the Linear Algorithm from Lecture 6 Pseudo Algorithm 
        std::vector<Eigen::Vector2d> Cspace_obs; // Initialize 2d vector of vertices of V(+)W
        int n = RobotPolygon.size() ;
        int m = ObstaclePolygon.size() ;
        int a = 0; int j = 0;
        while (a < n && j < m) {
            double x = RobotPolygon[a].first + ObstaclePolygon[j].first;
            double y = RobotPolygon[a].second + ObstaclePolygon[j].second;
            Cspace_obs.push_back(Eigen::Vector2d(x, y));
            std::cout << "Cspace_obs point: (" << x << ", " << y << ")\n"; 
            // Change indices as follows:
            if (angleRobot[a] < angleObstacle[j]) {
                a++;
            }
            else if (angleRobot[a] > angleObstacle[j]) {
                j++;
            }
            else {
                a++;
                j++;
            }
        }


    // Store the original Cspace_obs as the first element of Cspace_Polygons
    Cspace_Polygons.push_back(amp::Polygon(Cspace_obs));
    theta_height = M_PI / 6.0 * i; 
    height.push_back(theta_height);
    
    }
        
    Visualizer::makeFigure(Cspace_Polygons, height);
    Visualizer::showFigures();     

}


//////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////
//                        problem 2 content 


MyManipulator::MyManipulator():LinkManipulator2D(){}
MyManipulator::MyManipulator(const std::vector<double> &link_lengths):LinkManipulator2D(link_lengths) {}
MyManipulator::MyManipulator(const Eigen::Vector2d &base_location, const std::vector<double> &link_lengths):LinkManipulator2D(base_location, link_lengths) {}

//Define  a function called createTransformationMatrix
Eigen::Matrix3d createTransformationMatrix(double theta, double x_translation) {
    
    // Convert the angle from degrees to radians
    double theta_rad = theta;

    // Calculate sine and cosine of the angle
    double cos_theta = std::cos(theta_rad);
    double sin_theta = std::sin(theta_rad);

    // Create the 3x3 transformation matrix
    Eigen::Matrix3d transformation_matrix;
    transformation_matrix << cos_theta, -sin_theta, x_translation,
                            sin_theta, cos_theta, 0.0,
                            0.0, 0.0, 1.0;

    return transformation_matrix;
}



// Now, define the function for getJointLocation using Forward Kinematics 
Eigen::Vector2d MyManipulator::getJointLocation(const ManipulatorState &state, uint32_t joint_index) const{
//Eigen::Vector2d jointLocation(0.0, 0.0); // Initialize joint location to (0, 0) as an Eigen::Vector2d
std::vector<double> link_lengths = getLinkLengths(); 
Eigen::Vector2d position;
// Check if the provided joint_index is within bounds
if (joint_index == 0){
    position = getBaseLocation();

} else{
    
    Eigen::Matrix<double, 3, 1> location;
        Eigen::Matrix<double, 3, 3> translationT; 
        translationT << 0, 0, link_lengths[joint_index-1],
                        0, 1, 0,
                        0, 0, 1; 
        location << 0,
                    0,
                    1; 
        location = translationT * location;
    double length; 
    for (int i = joint_index; i > 0; --i) {
        std::cout << "joint index: " << i << std::endl;
        //double x_translation = link_lengths[i]; // Same as the variable above, but defined differently for the next function 
        //std::cout << "x-direction translation: " << x_translation << std::endl;
        double theta = state[i-1]; // Same as the variable above, but defined differently for the next function 
        //std::cout << "Rotation Angles: " << theta_degrees << std::endl;

        if (i == 1){
            length = 0.0; 
        }else{
            length = link_lengths[i-2]; 
        }
        Eigen::Matrix3d transformationMatrix = createTransformationMatrix(theta, length);
        location =  transformationMatrix * location;
        std::cout << "Matrix location" << std::endl << location << "\n" << std::endl; 
    
    }
    position = {location.coeff(0,0), location.coeff(1, 0)}; // x direction is R13 of cumulativeTransformationMatrix
    std::cout << position(0) << " " << position(1) << std::endl; 
    return position;

}}

// // Now, define the function for getConfigurationFromIK using Inverse Kinematics 
amp::ManipulatorState amp::MyManipulator::getConfigurationFromIK(const Eigen::Vector2d &end_effector_location) const{
    Eigen::Vector2d jointLocation(0.0, 0.0); 
    }



// //////////////////////////////////////////////////////////////////////////


int main(int argc, char** argv) {
    /* Include this line to have different randomized environments every time you run your code (NOTE: this has no affect on grade()) */
    amp::RNG::seed(amp::RNG::randiUnbounded());

    // Grade method
    //amp::HW4::grade<MyLinkManipulator>(constructor, "nonhuman.biologic@myspace.edu", argc, argv);
    //return 0;

    //////////////////////////////// Problem 1 ////////////////////////////////////////////////////////////////////
    problem1a();
    problem1b(); 

    //////////////////////////////// Problem 2 ////////////////////////////////////////////////////////////////////
    // Create a vector of link lengths (given in the HW4)
    std::vector<double> linkLengths = {0.5, 1.0, 0.5};
    //std::vector<double> linkLengths = {0.5};

    // Create a MyManipulator object with the specified link lengths
    MyManipulator manipulator(linkLengths);
    
    //ManipulatorState currentState;
    std::vector<double> state_angles = {M_PI/6, M_PI/3, 7*M_PI/4};
    //std::vector<double> state_angles = {M_PI/2};

    ManipulatorState currentState(state_angles);

    std::cout << "Problem 2 manipulator and currentState objects created" << std::endl;

    // uint32_t jointIndex = 3; // Specify the joint index
    // // This specifies returns the Joint Location...BUT only if it has been defined before...so define it above 
    // Eigen::Vector2d jointLocation = manipulator.getJointLocation(currentState, jointIndex);

    //makeFigure(const LinkManipulator2D& link_manipulator, const ManipulatorState& state)
    Visualizer::makeFigure(manipulator, currentState);
    Visualizer::showFigures(); 

   
    return 0; 

}


