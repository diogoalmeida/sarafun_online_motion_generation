
#include "obstacle.h"

/**
 * @brief obstacle::obstacle
 */
obstacle::obstacle(){}

/**
 * @brief obstacle::obstacle
 * @param pos, the center of mass of the obstacle
 * @param radius, the radius of the obstacle
 * @param repulsive_gain, the intensity of the repulsive field of the obstacle 
 */
obstacle::obstacle(const Eigen::Vector3d pos, double radius, double repulsive_gain, const Eigen::Vector3d p0, const Eigen::Vector3d pT){
  this->pos_ = pos;
  this->rad_ = radius;
  this->rep_ = repulsive_gain;
  
  // compute the outter bound
  this->dcap_ = std::max( (p0-pos).norm() , (pT-pos).norm() ) + MU ; 
  
  // compute the mean 
  this->dm_ = (radius + dcap_) / 2;
  
  // compute rho for the controller
  this->rho_ = this->dcap_ - this->dm_;
  
}


/**
 * @brief obstacle::update
 * @param pos, the center of mass of the obstacle
 * @param radius, the radius of the obstacle
 * @param repulsive_gain, the intensity of the repulsive field of the obstacle 
 */
void obstacle::update(const Eigen::Vector3d pos, double radius, double repulsive_gain, const Eigen::Vector3d p0, const Eigen::Vector3d pT){
  this->pos_ = pos;
  this->rad_ = radius;
  this->rep_ = repulsive_gain;
  
  // compute the outter bound
  this->dcap_ = std::max( (p0-pos).norm() , (pT-pos).norm() ) + MU ; 
  
  // compute the mean 
  this->dm_ = (radius + dcap_) / 2;
  
  // compute rho for the controller
  this->rho_ = this->dcap_ - this->dm_;
  
}

/**
 * @brief obstacle::print
 */
void obstacle::print(void){
  
    printf("-------------\n");
    printf("[Obstacle] Center of mass [m]:\n");
    for(int i = 0; i<this->pos_.size();i++){ printf("%lf",this->pos_[i]); }
    printf("\n");
    printf("[Obstacle] Radius: %lf m \n",this->rad_);
    printf("[Obstacle] Repulsive Gain: %lf\n",this->rep_);  
    printf("[Obstacle] Outer bound: %lf\n",this->dcap_);  
    printf("[Obstacle] Mean value: %lf\n",this->dm_);  
}

/**
 * @brief obstacle::getRadius
 * @return radius of the obstacle
 */
double obstacle::getRadius(void){return  this->rad_;} 

/**
 * @brief obstacle::getPos
 * @return center of mass
 */
Eigen::Vector3d obstacle::getPos(void){return  this->pos_;} 

/**
 * @brief obstacle::getGain
 * @return the repulsive gain of the obstacle
 */
double obstacle::getGain(void){return  this->rep_;}  

/**
 * @brief obstacle::getDm
 * @return the dm parameter for the controller
 */
double obstacle::getDm(void){return  this->dm_;} 

/**
 * @brief obstacle::getRho
 * @return the rho parameter for the controller
 */
double obstacle::getRho(void){return  this->rho_;} 