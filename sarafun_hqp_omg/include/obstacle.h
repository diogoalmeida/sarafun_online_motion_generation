#ifndef OBSTACLE_H
#define OBSTACLE_H


#include <eigen3/Eigen/Dense>
#include <iostream>

// Define the accepted overshoot bound for the obstacle avoidance
#define MU 0.1



class obstacle
{
public:
  
    obstacle();
    obstacle(const Eigen::Vector3d pos, double radius, double repulsive_gain, const Eigen::Vector3d p0, const Eigen::Vector3d pT);
    void update(const Eigen::Vector3d pos, double radius, double repulsive_gain, const Eigen::Vector3d p0, const Eigen::Vector3d pT);
    
    void print(void);
    double getRadius(void); 
    Eigen::Vector3d getPos(void); 
    double getGain(void); 
    double getDm(void); 
    double getRho(void); 
 
    

private:
    
    double rad_; 
    Eigen::Vector3d pos_; 
    double rep_; 
    double dcap_;
    double dm_;
    double rho_;
    

};

#endif // OBSTACLE_H
