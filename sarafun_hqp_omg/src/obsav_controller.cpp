#include "obsav_controller.h"


/**
 * @brief obsav_controller::obsav_controller
 */
obsav_controller::obsav_controller( ){}

/**
 * @brief obsav_controller::obsav_controller
 * @param p0, initial position of the endeffector
 * @param pT, target position for the end-effector
 */
obsav_controller::obsav_controller( const Eigen::Vector3d p0, const Eigen::Vector3d pT){
  Nobs = 0;
  this->p0_=p0;
  this->pT_=pT;
  
}

/**
 * @brief obsav_controller::init
 * @param p0, initial position of the endeffector
 * @param pT, target position for the end-effector
 */
void obsav_controller::init( const Eigen::Vector3d p0, const Eigen::Vector3d pT){
  this->p0_=p0;
  this->pT_=pT;
  this->updateObsP0PT();
}
  
/**
* @brief obsav_controller::updateObsP0PT
*/
void obsav_controller::updateObsP0PT(void){

  for(int i = 0; i<this->Nobs;i++){ 
    printf("-------------\n");
    printf("[Obstacle Avoidance Controller] Updating p0 and pT of obstacle: %i \n", i);
    this->obs_[i].update( this->obs_[i].getPos(), this->obs_[i].getRadius(), this->obs_[i].getGain(), this->p0_, this->pT_);
  }
  
  
}
  
  
/**
* @brief obsav_controller::addObstacle
* @param obs_pos, center of mass of the additional obstacle
* @param obs_r, the radius of the obstacle
* @param repulsive_gain, the intensity of the repulsive field of the obstacle 
* @return obstacle id 
*/
int obsav_controller::addObstacle(const Eigen::Vector3d obs_pos, double obs_r, double repulsive_gain){
  
  this->Nobs += 1;
  
  //std::cout<<"this->Nobs = "<<this->Nobs<<std::endl;
  obstacle *temp;
  temp = new obstacle[this->Nobs-1]();
  for(int i = 0; i<this->Nobs-1;i++){
    temp[i] = obs_[i];
  }
  
  obs_ = new obstacle[this->Nobs]();
  
  for(int i = 0; i<this->Nobs-1;i++){
    obs_[i] = temp[i] ;
  }
  
  //printf("[addObstacle] Obstacles ob%d, orig: %f,%f,%f, radius:%f \n",Nobs-1,obs_pos[0],obs_pos[1],obs_pos[2],obs_r);
  
  this->obs_[Nobs-1].update(obs_pos, obs_r, repulsive_gain, this->p0_, this->pT_);
  
  return this->Nobs-1;
  
  this->print();
  
}

/**
* @brief obsav_controller::updObstacle
* @param obs_id, id of the obstacle (index of the obs_ array)
* @param obs_pos, center of mass of the additional obstacle
* @param obs_r, the radius of the obstacle
* @param repulsive_gain, the intensity of the repulsive field of the obstacle 
*/
void obsav_controller::updObstacle(int obs_id, const Eigen::Vector3d obs_pos, double obs_r, double repulsive_gain){
  
  this->obs_[obs_id].update(obs_pos, obs_r, repulsive_gain, this->p0_, this->pT_);
  
  
}

/**
* @brief obsav_controller::print
*/
void obsav_controller::print(void){
    printf("===========\n");
    printf("[Obstacle Avoidance Controller] Number of obstacles: %i \n", this->Nobs);
    for(int i = 0; i<this->Nobs;i++){ 
      printf("-------------\n");
      printf("[Obstacle Avoidance Controller] Obstacle: %i \n", i);
      obs_[i].print(); 
    }
}

/**
* @brief obsav_controller::getControlSignal
* @return the control signal, which is a velocity vector of dimension of 3 on the task space
* @param tipPos_, position of the arm's end-effector
*/
Eigen::Vector3d obsav_controller::getControlSignal(Eigen::Vector3d tipPos_){
  
  
  // the normal vector
  Eigen::Vector3d nd;
  
  double d = 0;
  double rho = 0;
  double e = 0;
  double c = 0;
  double epsilon = 0;
  double dT = 0;
  
  // initialization of the signal
  this->sig_  << 0,0,0;
  
  
  
  for(int i = 0; i<this->Nobs;i++){
    
    
    // compute the error of the point from the obstacle
    d = ( this->obs_[i].getPos() - tipPos_).norm();
    
    // compu te normal vector
    nd = ( this->obs_[i].getPos() - tipPos_ ) / d;
    
    // compute the error form the mean
    e = d - this->obs_[i].getDm() - D0;
    
    // compute the intermediate parameter c of controller
    c = e / this->obs_[i].getRho();
    
    // compute espilon of PPC methodology
    epsilon = std::log( ( 1 + c) / ( 1 - c ) ); 
    
    // compute jacobian of PPC method
    dT 	=  1 / ( e + this->obs_[i].getRho() ) - 1 / ( e - this->obs_[i].getRho());
    
       
    // accumulative signal for each obstacle
    this->sig_ += this->obs_[i].getGain() * dT * epsilon * nd ;
  }
  
  return this->sig_;
  
}
