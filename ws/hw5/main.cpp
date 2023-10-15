#include "AMPCore.h"
#include "hw/HW5.h"
#include "hw/HW2.h"
#include "MyGDAlgorithm.h"




void problem1(){

    double eta_repmag = 0.05; 
    double Q_star = .75; 
    double NoiseScalar = 0; 
    double d_star = 15; 
    double zeta_attract = 0.1; 
    double alpha = .1;

    // create problem object assign it to HW5::getWorkspace1()
    amp::Problem2D Problem = amp::HW5::getWorkspace1(); 
    // create robot object 
    MyGDAlgorithm robot(eta_repmag,Q_star,NoiseScalar,d_star,zeta_attract,alpha); 
    // pass in the problem into the robot object to create a path
    amp::Path2D Path = robot.plan(Problem); 
    // Make the figure
    amp::Visualizer::makeFigure(Problem, Path);
    amp::Visualizer::showFigures(); 
    //return path 
}

void problem2(){
    double eta_repmag = 0.5; 
    double Q_star = .5; 
    double NoiseScalar = 1; 
    double d_star = 3; 
    double zeta_attract = 0.1; 
    double alpha = .1;

    // create problem object assign it to HW5::getWorkspace1()
    amp::Problem2D Problem = amp::HW2::getWorkspace1(); 
    // create robot object 
    MyGDAlgorithm robot(eta_repmag,Q_star,NoiseScalar,d_star,zeta_attract,alpha); 
    // pass in the problem into the robot object to create a path
    amp::Path2D Path = robot.plan(Problem); 
    // Make the figure
    amp::Visualizer::makeFigure(Problem, Path);
    amp::Visualizer::showFigures(); 
    //return path
}

void problem3(){

    double eta_repmag = 0.1; 
    double Q_star = .75; 
    double NoiseScalar = .9; 
    double d_star = 15; 
    double zeta_attract = 0.1; 
    double alpha = .1;

    // create problem object assign it to HW5::getWorkspace1()
    amp::Problem2D Problem = amp::HW2::getWorkspace2(); 
    // create robot object 
    MyGDAlgorithm robot(eta_repmag,Q_star,NoiseScalar,d_star,zeta_attract,alpha); 
    // pass in the problem into the robot object to create a path
    amp::Path2D Path = robot.plan(Problem); 
    // Make the figure
    amp::Visualizer::makeFigure(Problem, Path);
    amp::Visualizer::showFigures(); 
    //return path
}

int main(int arg, char** argv){


    //problem1();
    //problem2();
    problem3();
    //MyGDAlgorithm robot(eta_repmag,Q_star,NoiseScalar,d_star,zeta_attract,alpha); 
    //amp::HW5::grade(robot,"cach8017@colorado.edu",arg,argv);
    return 0;
}