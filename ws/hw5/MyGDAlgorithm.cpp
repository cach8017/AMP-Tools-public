#include "MyGDAlgorithm.h"

//Global Variables


// tolerance "epsilon": distance between currentPoint and goalPoint  
//double epsilon = 5;

// Define zeta attraction constant for the Uatt gradient 
//double zeta_attract = 1.0; 

// Define threshold distance from the goal where Uatt planner
// switches between conic and quadratic potentials 
//double d_star = 4.0; 

// Define the eta replusive gain/magintude for the Urep planner 
//double eta_repmag = 1.0; 

// Define the threshold value for the distance between robot and 
// the closest point on obstacle where Urep planner switches between 0 and 
// a defined function Urep 
//double Q_star = 1.0; 

//double NoiseScalar = .1; 



//ζ = 0.5; dgoal* = 15; ŋ = 1; Q* = 1

// Constructor implementation
MyGDAlgorithm::MyGDAlgorithm(double eta_repmag, double Q_star, double NoiseScalar, double d_star, double zeta_attract, double alpha)
    : eta_repmag(eta_repmag), Q_star(Q_star), NoiseScalar(NoiseScalar), d_star(d_star), zeta_attract(zeta_attract), alpha(alpha) {
    // Initialize other member variables if needed
}

amp::Path2D MyGDAlgorithm::plan(const amp::Problem2D& problem) {
        // Implement the Gradient Descent Logic here per the pseudo-algorithm on Pg 85
        amp::Path2D path; // Create an empty path

        // Initialize starting position, problem.q_init
        Eigen::Vector2d currentPoint = problem.q_init;
        path.waypoints.push_back(currentPoint);
        PRINT_VEC2("currentPoint before while loop: ", currentPoint);

        // Define End Point
        Eigen::Vector2d goalPoint = problem.q_goal; 
        //std::cout << "Goal position: \n"<< goalpoint << "\n"<<std::endl;

        // Initialize the gradient
        //Eigen::Vector2d grad = calculateGradient(currentPoint, problem);        

        // scalar rate for gradient descent
        double alpha = .1;

        //double maxiteration = 1000;
        double maxiteration =4000;
        double epsilon = .25; // Convergence threshold
        while (!isConverged(currentPoint, goalPoint, epsilon)) {

            // Calculate the attractive gradient based on the d_star threshold
            // Calculate the distance to the goal (Euclidean distance)
            //Eigen::Vector2d numerator = (currentPoint - goalPoint);
            //PRINT_VEC2("Checking (currentPoint - goalPoint) = ", numerator);
            //double dgoal = findDistance(currentPoint,goalPoint);
            double dgoal = (currentPoint-goalPoint).norm();
            //LOG("dgoal: "<<dgoal);
            // Initialize the attractive gradient with its data type
            Eigen::Vector2d att_gradient;
            //double d_star = 15; 
            //double zeta_attract = 1.0; 
            // Calculate the attractive gradient based on the d_star threshold
            if (dgoal <= d_star) {
                // If within d_star, use a linear attractive force
                att_gradient =  zeta_attract * (currentPoint - goalPoint);
                // You have a negative sign because the gradient should point towards the goal.
            } else {
                // If beyond d_star, use a quadratic attractive force
                att_gradient = d_star * zeta_attract * (currentPoint - goalPoint) / dgoal;
            }

            // Calculate total repulsive gradient for all obstacles by looping
            // Loop through the polygons and calculate the minimum perpendicular distance for each
            // Initialize the repulsive gradient with its data type
/////////First Draft
            // Eigen::Vector2d rep_gradient = Eigen::Vector2d::Zero();
            // double eta_repmag = 1.0; 
            // double Q_star = .5; 
            // // Loop through the polygons and calculate the minimum perpendicular distance for each
            // for (const amp::Obstacle2D& Obstacle_i : problem.obstacles) {
            //     std::pair<double, Eigen::Vector2d> result = findMinimumPerpendicularDistance(currentPoint, Obstacle_i);
            //     double minDistance = result.first;
            //     Eigen::Vector2d closestPoint = result.second;

            //     if (minDistance <= Q_star) {
            //         rep_gradient += eta_repmag * (1.0 / Q_star - 1.0 / minDistance) * (1.0 / (minDistance * minDistance)) * (currentPoint - closestPoint) / minDistance;
            //     } else {
            //         rep_gradient = Eigen::Vector2d::Zero();
            //     }
            // }

            // // If no obstacles are within the repulsive range, set rep_gradient to zero
            // if (rep_gradient.norm() == 0) {
            //     rep_gradient = Eigen::Vector2d::Zero();
            // }
////////Second Draft
            Eigen::Vector2d rep_gradient = Eigen::Vector2d::Zero();
            //double eta_repmag = 1.0; 
            //double Q_star = .75; 
            // Loop through the polygons and calculate the minimum perpendicular distance for each
            for (const amp::Obstacle2D& Obstacle_i : problem.obstacles) {
                std::pair<Eigen::Vector2d,double> result = ClosePointPolygon_NearestPointFrom(currentPoint, Obstacle_i);
                //double minDistance = result.second;
                Eigen::Vector2d closestPoint = result.first;
                PRINT_VEC2("closest point ",closestPoint);
                double minDistance = (currentPoint-closestPoint).norm();

                if (minDistance <= Q_star) {
                    rep_gradient += eta_repmag * (1.0 / Q_star - 1.0 / minDistance) * (1.0 / (minDistance * minDistance)) * (currentPoint - closestPoint) / minDistance;
                } else {
                    rep_gradient += Eigen::Vector2d::Zero();
                }
            }

            // If no obstacles are within the repulsive range, set rep_gradient to zero
            if (rep_gradient.norm() == 0) {
                rep_gradient = Eigen::Vector2d::Zero();
            }

            Eigen::Vector2d gradient; 
            //double NoiseScalar = 0; 
            gradient = att_gradient + rep_gradient + AddNoise(NoiseScalar);
            //gradient = att_gradient + rep_gradient;

            //gradient = att_gradient;

            // Calculate gradient at the current point
            // Eigen::Vector2d gradient = calculateGradient(currentPoint, problem);
            PRINT_VEC2("Gradient after one while loop iteration: ", gradient);
            // Update the current point using the gradient descent rule
            currentPoint -= alpha * gradient;
            PRINT_VEC2("currentPoint after one while loop iteration: ", currentPoint);
            // Add the updated point to the path
            path.waypoints.push_back(currentPoint);
            maxiteration--; 
            if (maxiteration == 0) break;
            
            
        //
    }
    path.waypoints.push_back(problem.q_goal);
    LOG("maxiteration "<<1000-maxiteration);
        return path;
}



bool MyGDAlgorithm::isConverged(Eigen::Vector2d& currentPoint, const Eigen::Vector2d& goalPoint, const double& epsilon) {
    // Check if the distance between currentPoint and goalPoint is less than tolerance
    // Implement convergence criteria here
    // Calculate the magnitude (distance) between the two points manually
    double distance = 0.0;
    for (int i = 0; i < currentPoint.size(); ++i) {
        double diff = currentPoint[i] - goalPoint[i];
        distance += diff * diff;
    }
    distance = std::sqrt(distance);
    // Return true when the criteria are met.
    if (distance < epsilon) {
        return true; 
    }
}

Eigen::Vector2d MyGDAlgorithm::calculateGradient(Eigen::Vector2d& currentPoint, const amp::Problem2D& problem) {
//     Calculate the gradient based problem
//     Implement  gradient calculation logic here.
//     Return the calculated gradient.

//     Define End Point
//     Eigen::Vector2d goalpoint = problem.q_goal; 
//     Eigen::Vector2d goalpoint = problem.q_goal; 
//     LOG("goalpoint: "<<goalpoint);
//     // Find the distance to the goal
//     // Calculate the 2-norm (Euclidean distance)
//     double dgoal = (currentPoint - goalpoint).norm();
//     LOG("dgoal: "<<dgoal);
//     //std::cout << dgoal <<std::endl; 
//     // Initialize attractive gradient
//     Eigen::Vector2d position; 

//     // Find the attractive gradient: Uatt using the d_star threshold
//     if (dgoal <= d_star) {
//         position = 2*zeta_attract * (currentPoint - goalpoint);
//         std::cout<<"\n Less than dstar Uatt\n"<<std::endl; 
//     } else {
//         position = 2*d_star * zeta_attract * (currentPoint - goalpoint)/dgoal;
//         //std::cout<<"\n Greater than dstar Uatt\n"<<std::endl; 
//     }
//     PRINT_VEC2("Position after Gradient: ", position);
    

//     Define the goal point
//         Eigen::Vector2d goalPoint = problem.q_goal;

//         // Calculate the distance to the goal (Euclidean distance)
//         double dgoal = (currentPoint - goalPoint).norm();
//         LOG("dgoal: "<<dgoal);

//         // Initialize the attractive gradient
//         Eigen::Vector2d gradient;

//         // Calculate the attractive gradient based on the d_star threshold
//         if (dgoal <= d_star) {
//             // If within d_star, use a linear attractive force
//             gradient =  zeta_attract * (currentPoint - goalPoint);
//             // You have a negative sign because the gradient should point towards the goal.
//         } else {
//             // If beyond d_star, use a quadratic attractive force
//             gradient = d_star * zeta_attract * (currentPoint - goalPoint) / dgoal;
//         }
//         double gradientMagnitude = gradient.norm();
//         LOG("Gradient Norm"<<gradientMagnitude);
//         // Return the calculated gradient
//         return gradient;
//     }

//         amp::Obstacle2D Oi;
        
//         //Find the repulsive gradient
//         for (const std::vector<Obstacle2D>& obstacles : #####OBSTACLE) {

//         define di = q_current - closest point on obstacle Oi / distance(q,c)  Eq 4.7
//         Find the closest point on the obstacle Oi
    

//            if (di <= Q_star) {
//                xTot += eta_repmag * (1.0 / Q_star - 1.0 / di) * (1.0 / (di * di)) * ((cx - ObsxClosest) / di);
//                yTot += eta_repmag * (1.0 / Q_star - 1.0 / di) * (1.0 / (di * di)) * ((cy - ObsyClosest) / di);
//            }
//         }

//         Return the gradient of the potential
//         std::cout << "current position: "<< position << "\n"<<std::endl; 
        
        
//         return position;
//     }

}

double MyGDAlgorithm::findDistance(const Eigen::Vector2d& point1,const Eigen::Vector2d& point2) {
    double dx = point1[0] - point2[0];
    double dy = point1[1] - point2[1];
    
    return std::sqrt(dx * dx + dy * dy);
    //return dx * dx + dy * dy;
}


std::pair<double, Eigen::Vector2d> MyGDAlgorithm::findMinimumPerpendicularDistance(const Eigen::Vector2d& Point, const amp::Obstacle2D& polygon) {


    // The purpose of setting minDistance to this maximum value is
    // to ensure that the first calculated distance within the loop will always be
    // smaller than the initial minDistance. As you iterate through the edges and update
    // minDistance with the minimum distance, it will eventually hold the minimum perpendicular
    // distance from the point to the polygon. This is a common technique to initialize a variable
    // when you're searching for a minimum value.

    double minDistance = std::numeric_limits<double>::max();
    Eigen::Vector2d closestPoint;

    // Iterate through the edges of the polygon
    for (size_t i = 0; i < polygon.verticesCCW().size(); ++i) {
        const Eigen::Vector2d& p1 = polygon.verticesCCW()[i];
        const Eigen::Vector2d& p2 = polygon.verticesCCW()[(i + 1) % polygon.verticesCCW().size()];

        // Calculate the vector from p1 to p2
        double edgeX = p2.x() - p1.x();
        double edgeY = p2.y() - p1.y();

        // Calculate the vector from p1 to the point
        double pointX = Point.x() - p1.x();
        double pointY = Point.y() - p1.y();

        // Calculate the dot product of the edge and point vectors
        double dotProduct = edgeX * pointX + edgeY * pointY;

        // Calculate the squared length of the edge
        double edgeLengthSquared = edgeX * edgeX + edgeY * edgeY;

        // Calculate the parameter t for the projection of the point onto the edge
        double t = dotProduct / edgeLengthSquared;

        // Clamp t to the range [0, 1] to ensure it falls on the edge segment
        t = std::max(0.0, std::min(1.0, t));

        // Calculate the closest point on the edge to the given point
        double closestX = p1.x() + t * edgeX;
        double closestY = p1.y() + t * edgeY;

        // Calculate the distance between the closest point and the given point
        Eigen::Vector2d currentClosestPoint(closestX, closestY);
        double dist = (currentClosestPoint - Point).norm();

        // Update the minimum distance and closest point if necessary
        if (dist < minDistance) {
            minDistance = dist;
            closestPoint = currentClosestPoint;
        }
    }

    // Return the minimum distance and the closest point as a pair
    return std::make_pair(minDistance, closestPoint);
}


std::pair<Eigen::Vector2d, double> MyGDAlgorithm::ClosePointPolygon_NearestPointFrom(const Eigen::Vector2d& p, const amp::Obstacle2D& polygon) {
    std::vector<Eigen::Vector2d> points = polygon.verticesCCW(); //extract the vertices from the polygon in amp::Obstacle2D&
    std::vector<Eigen::Vector2d> iEdges;
    double nearestNormSqr = std::numeric_limits<double>::max(); //keep track of the squared distance between the nearest point found on the polygon and the original point p
                                                                //It's initialized with the maximum possible value so that any real squared distance calculated during the subsequent loop iterations is guaranteed to be smaller than this initial value


    //loop through the list of points, calculates the direction vector between each consecutive pair of points, and normalize it
    // auto Precalculate = [&]() { //find the number of edges in Points
    //     if (points.empty()) {
    //         iEdges.clear();
    //     }
    //     else if (points.size() == 1) {
    //         iEdges = {Eigen::Vector2d(0, 0)};
    //     }
    //     else {
    //         std::vector<Eigen::Vector2d> list;
    //         Eigen::Vector2d pB = points.back();
    //         for (const Eigen::Vector2d& point : points) {
    //             Eigen::Vector2d pA = pB;
    //             pB = point;
    //             Eigen::Vector2d pAB = pB - pA;
    //             double DD = pAB.squaredNorm();
    //             if (DD > 0.0) {
    //                 list.push_back(pAB / DD);
    //             }
    //             else {
    //                 list.push_back(Eigen::Vector2d(0, 0));
    //             }
    //         }
    //         iEdges = list;
    //     }
    // };

    std::vector<Eigen::Vector2d> list;
    Eigen::Vector2d pB = points.back();
    for (const Eigen::Vector2d& point : points) {
        Eigen::Vector2d pA = pB;
        pB = point;
        Eigen::Vector2d pAB = pB - pA;
        double DD = pAB.squaredNorm();
        if (DD > 0.0) {
            list.push_back(pAB / DD);
        }
        else {
            list.push_back(Eigen::Vector2d(0, 0));
        }
    }
    iEdges = list;

    int length = static_cast<int>(points.size()); //check to see the polygon points is not empty
    Eigen::Vector2d nearest = Eigen::Vector2d::Constant(std::numeric_limits<float>::infinity());
    double minDistance = std::numeric_limits<double>::max();

    if (length > 1) {
        nearestNormSqr = std::numeric_limits<float>::max();
        nearest = Eigen::Vector2d::Constant(std::numeric_limits<float>::infinity());

        // if (iEdges.empty()) {
        //     Precalculate();
        // }

        Eigen::Vector2d pB = points.back();
        for (int i = 0; i < length; i++) {
            Eigen::Vector2d pA = pB;
            pB = points[i];

            Eigen::Vector2d q = Eigen::Vector2d::Constant(std::numeric_limits<float>::infinity());
            double t = (p - pA).dot(iEdges[i]);

            if (t <= 0.0) {
                q = pA;
            } else if (t >= 1.0) {
                q = pB;
            } else {
                q = (1.0 - t) * pA + t * pB;
            }

            double qq = (q - p).squaredNorm();
            if (qq < nearestNormSqr) {
                nearest = q;
                nearestNormSqr = qq;
                minDistance = std::sqrt(qq); // Calculate distance between nearest and p
            }
        }
    }
    else if (length == 1) {
        nearest = points[0];
        nearestNormSqr = (nearest - p).squaredNorm();
        minDistance = std::sqrt(nearestNormSqr); // Calculate distance between nearest and p
    } else {
        std::cerr << "ClosePointPolygon_NearestPointFrom points is empty" << std::endl;
        nearestNormSqr = std::numeric_limits<float>::max();
    }

    return std::make_pair(nearest, minDistance);
}

Eigen::Vector2d MyGDAlgorithm::AddNoise(const double& scalar) {
    // Seed the random number generator with a time-based value
    std::random_device rd;
    std::mt19937 genx(rd());
    std::mt19937 geny(rd());


    // Create a distribution for random noise
    std::normal_distribution<double> distribution(0.0, scalar); // Mean: 0.0, Standard Deviation: scalar

    // Generate small (x, y) perturbations as noise
    double noiseX = distribution(genx);
    double noiseY = distribution(geny);

    Eigen::Vector2d noisyPoint(noiseX, noiseY);
    PRINT_VEC2("perturbation: ", noisyPoint);
    return noisyPoint;
    
}
