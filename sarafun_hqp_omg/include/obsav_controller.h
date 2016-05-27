#ifndef OBSAV_CONTROLLER_H
#define OBSAV_CONTROLLER_H


#include <eigen3/Eigen/Dense>
#include "obstacle.h"
#include <iostream>


#define D0 0.07


class obsav_controller
{
public:
    obsav_controller();
    obsav_controller( const Eigen::Vector3d p0, const Eigen::Vector3d pT);
    void init( const Eigen::Vector3d p0, const Eigen::Vector3d pT);
     
    int addObstacle(const Eigen::Vector3d obs_pos, double obs_r, double repulsive_gain);

    void updObstacle(int obs_id, const Eigen::Vector3d obs_pos, double obs_r, double repulsive_gain);
    
    void print(void);
    Eigen::Vector3d getControlSignal(Eigen::Vector3d tipPos_); 

private:
  
  
    void updateObsP0PT(void);
      
    int Nobs = 0;
    obstacle *obs_;
    Eigen::Vector3d sig_;
    Eigen::Vector3d p0_;
    Eigen::Vector3d pT_;


};

#endif // OBSAV_CONTROLLER_H
