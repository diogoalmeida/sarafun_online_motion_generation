#include "hqp_wrapper.h"

#include <ros/ros.h>
#include <ros/service_server.h>
#include <actionlib/server/simple_action_server.h>
#include <sarafun_hqp_omg/OnlineMotionAction.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <initHQP.h>
#include <addObs.h>
#include <obsav_controller.h>

#include <kdl/kdl.hpp>
#include <kdl/frames.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <urdf/model.h>
#include <visualization_msgs/Marker.h>
#include <kdl_conversions/kdl_msg.h>
#include <math.h>

// No actual need for mtxs currently..
#include <mutex>

#define OBS_REP_GAIN  0.002
#define LOOP_FREQ  100.0

#define ENABLE_HQP_FOR_OBS  true

int Nj;              /**< Number of joints, will be read from URDF */
const int Ntask = 6; /**< Task DOF */

Eigen::VectorXd g_q;                        /**< Global variable, storing robot q's from topic */
Eigen::Vector3d g_x_ref;                    /**< Cartesian reference/desired position (x,y,z) */
Eigen::Vector3d g_x_msr;                    /**< Cartesian measured position (x,y,z) */
Eigen::Matrix3d g_R_ref;                    /**< Reference rotation matrix (that is end effector to base) */
Eigen::Matrix3d g_R_msr;                    /**< Robot's measured rotation matrix */
Eigen::Quaternion<double> g_Q_ref,g_Q_msr;  /**< Quaternions used for messaging only */
int g_new_ref = 0;                          /**< Flag set if  new reference pose */
int g_new_q = 0;                            /**< Flag set if new robot's q obtained */

/**< We use rr_mtx to pass variables between main controller and topics callbacks. CURRENTLY spinonce is executed in same thread with controller so is not necessary.. */
std::mutex rr_mtx;

static double dt = 1.0/LOOP_FREQ; // pretty obvious
static int task_start = 0;                  /**< Flag indicating if commanded q's are updated, waits for a ref pose to be set */
static double safeDst = 0;                  /**< see launch file */
static int start = 0;                       /**< Flag set to if HQP is initiliased, is set to launch file or call init service */
static int useFK = 0;                       /**< Flag set if FK (from urdf is used) or get robot's pose from topic. see launch file */
static bool useSim = 0;                     /**< if set, sends obstacles to rViz */


/** @struct ob_
 *  @brief Structure storing single obstacle info
 *  @var ob_::origin stores obstacle's position
 *  @var ob_::radius Currently only spheres....
 *  @var ob_::name name of the obstacle used as ID
 */
typedef struct {
    Eigen::Vector3d  origin;
    double radius;
    std::string name;
}ob_;


// ROS related variables
static ros::NodeHandle *nh;
static actionlib::SimpleActionServer<sarafun_hqp_omg::OnlineMotionAction> *as;
static ros::Rate *loop_rate;
static ros::Publisher joint_publisher;
static ros::Subscriber msr_q_subscr;
static ros::Subscriber ref_pos_subscr;
static ros::Subscriber msr_pos_subscr;
static ros::ServiceServer init_Service;
static ros::ServiceServer addObs_Service;
static sensor_msgs::JointState msg;             /**< msg used for publishing joint's states */

std::string msr_q_topic;
std::string cmd_q_topic;
std::string msr_pose_topic;
std::string ref_pose_topic;
std::string actionlib_server_name;


// Controller related variables
static hqp_wrapper *hqpw = NULL;                /**< HQP wrapper class */
static obsav_controller *oaController = NULL;                /**< obstacle avoidance controller class */

std::vector<double> ob_tmp;
std::vector<ob_> obs;
ob_ ob;
std::vector<double> ref_pose;                   /**< used to pass reference pose from launch file */

urdf::Model model;
Eigen::VectorXd qmin,qmax,q,solution,sol_prev;
Eigen::VectorXd e;
Eigen::Vector3d ep,ew,x_ref,x_m;
Eigen::Vector3d n;
Eigen::Matrix3d R_ref,R_msr,dR;
Eigen::Quaternion<double> Q_ref,Q_msr;


std::vector<double> joint_lim_min,joint_lim_max;
KDL::ChainJntToJacSolver *JacSolver_;
KDL::ChainFkSolverPos_recursive *fkSolver_;
KDL::ChainIkSolverPos_LMA *ikSolver_;
KDL::Chain chain_;
KDL::Tree tree_;
KDL::Jacobian J_;
KDL::JntArray q_;
KDL::Frame Fr_;

ros::Publisher marker_pub;
visualization_msgs::Marker marker;
static struct timespec tv;
static volatile double tic;

double th;
Eigen::Matrix3d tmp_M3d;


static double alpha;
static double beta;

Eigen::Vector3d pdddot,pddot,pd, kd;
static double theta_dddot=0, theta_ddot=0, theta_d;
static double wd,wd_max;
double t0, t1;
static double g_error_thres = 0.05;


inline void toEigen(Eigen::Vector3d &V, const KDL::Vector& V_ ) {
    V(0)=V_.x(); V(1)=V_.y(); V(2)=V_.z();
}
inline void toEigen(Eigen::Matrix3d &M, const KDL::Rotation& M_ ) {
    //  std::cout << "Fr_\n";
    for(int i=0;i<3;i++){
        for(int j=0;j<3;j++){
            M(i,j) = M_(i,j);
            //        std::cout << M_(i,j)<<" ";
        }
        //  std::cout <<"\n";
    }
}

void msr_q_callback(const sensor_msgs::JointState::ConstPtr& msg){
    static int i;
    rr_mtx.lock();
    for(i=0;i<Nj;i++){
        g_q[i] = msg->position[i];
    }
    g_new_q = 1;
    rr_mtx.unlock();
}

void msr_pose_callback(const geometry_msgs::PoseStamped::ConstPtr &msg){
    rr_mtx.lock();
    g_x_msr(0) = msg->pose.position.x;
    g_x_msr(1) = msg->pose.position.y;
    g_x_msr(2) = msg->pose.position.z;

    g_Q_msr.w() = msg->pose.orientation.w;
    g_Q_msr.x() = msg->pose.orientation.x;
    g_Q_msr.y() = msg->pose.orientation.y;
    g_Q_msr.z() = msg->pose.orientation.z;
    g_R_msr = g_Q_msr.toRotationMatrix();

    rr_mtx.unlock();
}

void ref_pose_callback(const geometry_msgs::PoseStamped::ConstPtr &msg){
    rr_mtx.lock();
    g_x_ref(0) = msg->pose.position.x;
    g_x_ref(1) = msg->pose.position.y;
    g_x_ref(2) = msg->pose.position.z;

    g_Q_ref.w() = msg->pose.orientation.w;
    g_Q_ref.x() = msg->pose.orientation.x;
    g_Q_ref.y() = msg->pose.orientation.y;
    g_Q_ref.z() = msg->pose.orientation.z;




    double tmp = g_Q_ref.norm();
    if( tmp < 0.001){
        g_R_ref = Eigen::Matrix3d::Zero(3,3);
    }
    else{
        g_Q_ref.normalize();
        g_R_ref = g_Q_ref.toRotationMatrix();
    }
    task_start = 1;
    g_new_ref = 1;
    std::cout <<"g_R_ref:\n" << g_R_ref<<"\n";
    rr_mtx.unlock();

}

/**
 * @brief init_obsav  Initilises the obstacle avoidance controller solver
 * @param p0       initial position
 * @param pT       target position
 * @return          Always works!
 */
bool init_obsav(void){

    Eigen::Vector3d p0 = g_x_msr;

    Eigen::Vector3d pT = g_x_ref;

    ROS_INFO("[init_obsav] Initiliazing obstacle avoidance controller\n");
    int i;
    if (oaController != NULL){
        ROS_INFO("[init_obsav] Destroying previous instance\n");
        delete oaController;
    }

    oaController = new obsav_controller( p0 , pT );

    // Add obstacle after Joint limits and before task
    for (i = 0; i < obs.size(); i++){
      oaController->addObstacle(obs[i].origin, obs[i].radius , OBS_REP_GAIN);
    }

    oaController->print();
    return true;

}

/**
 * @brief init_hqp  Initilises HQP solver, adding joint limits, obstacles and main task
 * @param req       Not used
 * @param res       Not used
 * @return          Always works!
 */
bool init_hqp(PROJECT_NAME::initHQP::Request &req, PROJECT_NAME::initHQP::Response &res){
    ROS_INFO("Initiliazing hqp");
    int i;
    if (hqpw != NULL){
        ROS_INFO("Destroying previous instance");
        delete hqpw;
    }
    hqpw = new hqp_wrapper(Nj);

    hqpw->addStage("JointLimMin",Eigen::MatrixXd::Identity(Nj,Nj),0.1*(qmin-q)/dt,soth::Bound::BOUND_INF);
    hqpw->addStage("JointLimMax",Eigen::MatrixXd::Identity(Nj,Nj),0.1*(qmax-q)/dt,soth::Bound::BOUND_SUP);
    // Add obstacle after Joint limits and before task
    if(ENABLE_HQP_FOR_OBS){
      for (i = 0; i < obs.size(); i++){
	hqpw->addObstacle(obs[i].name, Eigen::MatrixXd::Zero(Ntask,Nj) ,Eigen::Vector3d::Zero(3,1),-1e6);
      }
    }
    hqpw->addStage("Task",Eigen::MatrixXd::Zero(Ntask,Nj),Eigen::VectorXd::Zero(Ntask,1),soth::Bound::BOUND_TWIN);
    hqpw->init();
    hqpw->print();

    if(!ENABLE_HQP_FOR_OBS){
      init_obsav();
    }

    start = 1;
    return true;

}


/** addObs
 *  @brief Service called to add objects if ID is not already stored in obs. If found it updates existing ones
 */
bool addObs(PROJECT_NAME::addObs::Request &req,PROJECT_NAME::addObs::Response &res){

    int indx;
    int newObs;
    newObs = 0;
    ROS_INFO("Updating  obstacles");
    for(int i = 0 ; i < req.obs.size(); i++){
        std::cout <<"name:" << req.obs[i].name <<"\n";
        std::cout <<"pose:" << req.obs[i].pose.position.x <<", "<< req.obs[i].pose.position.y <<", "<< req.obs[i].pose.position.z<<"\n";
        std::cout <<"axes:" << req.obs[i].axes.x <<"\n";

    }
    for(int i = 0 ; i < req.obs.size(); i++){
        indx = -1;
        //Check if exists
        for(int j = 0 ; j < obs.size(); j++){
            if(obs[j].name == req.obs[i].name)
                indx = j;
        }
        if(indx == -1){ // Object does not exists
            newObs = 1;
            obs.push_back(ob); //insert whatever,
            indx = obs.size()-1;
        }
        std::cout <<"indx: "<< indx << " i:" << i <<"\n";
        obs[indx].origin << req.obs[i].pose.position.x, req.obs[i].pose.position.y, req.obs[i].pose.position.z;
        obs[indx].radius = req.obs[i].axes.x;
        obs[indx].name = req.obs[i].name;
    }

    for(int i = 0; i < obs.size(); i++){
        ROS_INFO("Obstacles ob%d, orig: %f,%f,%f, radius:%f",i+1,obs[i].origin[0],obs[i].origin[1],obs[i].origin[2],obs[i].radius);
    }
    if(newObs){
        //Call init
        ROS_INFO("New obstacle found, calling init_hqp");
        PROJECT_NAME::initHQP::Request req;
        PROJECT_NAME::initHQP::Response res;
        init_hqp(req,res);
    }

    return true;
}


/** addObs_sarafun
 *  @brief Service called to add objects if ID is not already stored in obs. If found it updates existing ones
 */
bool addObs_sarafun(PROJECT_NAME::addObs::Request &req,PROJECT_NAME::addObs::Response &res){

    int indx;
    int newObs;
    newObs = 0;
    ROS_INFO("[addObs_sarafun] Updating  obstacles");
    for(int i = 0 ; i < req.obs.size(); i++){
        std::cout <<"name:" << req.obs[i].name <<"\n";
        std::cout <<"pose:" << req.obs[i].pose.position.x <<", "<< req.obs[i].pose.position.y <<", "<< req.obs[i].pose.position.z<<"\n";
        std::cout <<"axes:" << req.obs[i].axes.x <<"\n";

    }
    for(int i = 0 ; i < req.obs.size(); i++){
        indx = -1;
        //Check if exists
        for(int j = 0 ; j < obs.size(); j++){
            if(obs[j].name == req.obs[i].name)
                indx = j;
        }
        if(indx == -1){ // Object does not exists
            newObs = 1;
            obs.push_back(ob); //insert whatever,
            indx = obs.size()-1;
        }
        std::cout <<"indx: "<< indx << " i:" << i <<"\n";
        obs[indx].origin << req.obs[i].pose.position.x, req.obs[i].pose.position.y, req.obs[i].pose.position.z;
        obs[indx].radius = req.obs[i].axes.x;
        obs[indx].name = req.obs[i].name;
    }
    ROS_INFO("[addObs_sarafun] Initialization and update of obsav_controller ...");
    init_obsav();
    for(int i = 0; i < obs.size(); i++){
        oaController->updObstacle( i , obs[i].origin, obs[indx].radius , OBS_REP_GAIN);
        ROS_INFO("[addObs_sarafun] Register obstacle ob%d, orig: %f,%f,%f, radius:%f",i+1,obs[i].origin[0],obs[i].origin[1],obs[i].origin[2],obs[i].radius);
    }

    oaController->print();

    return true;
}

/** getReferencePose
 */
void getReferencePose(geometry_msgs::Pose ref)
{
    
	x_ref << ref.position.x, ref.position.y, ref.position.z;
	Q_ref.w() = ref.orientation.x;
	Q_ref.x() = ref.orientation.y;
	Q_ref.y() = ref.orientation.z;
	Q_ref.z() = ref.orientation.w;
	R_ref = Q_ref.toRotationMatrix();
	ROS_INFO("Ref pose set to, x:%f, y:%f, z:%f",x_ref[0],x_ref[1],x_ref[2]);
	g_new_ref = 1;
    
	// Initialize globals,msgs before subscribing
	
	if (task_start != 1)
	{
		g_q = q;
	
	}
	g_x_ref = x_ref;
	g_x_msr = x_ref;
	g_Q_ref = Q_ref;
	g_R_ref = R_ref;
	task_start = 1;
}

/** getObstacles
 */
void getObstacles()
{
    int i = 1;
	obs.clear();
    while(nh->getParam("ob"+std::to_string(i),ob_tmp)){
        ROS_INFO("Found obstacle ob%d, orig: %f,%f,%f, radius:%f",i,ob_tmp[0],ob_tmp[1],ob_tmp[2],ob_tmp[3]);
        ob.origin << ob_tmp[0],ob_tmp[1],ob_tmp[2];
        ob.radius = ob_tmp[3];
        ob.name = "ob"+std::to_string(i);
        obs.push_back(ob);
        i++;
    }
}

/** drawObstacles
 */
void drawObstacles()
{
    int i;
    g_new_q = 1;
    g_q += (dt*solution);
    for(i=0; i<obs.size();i++){
        marker.id = i;
        marker.pose.position.x = obs[i].origin[0];
        marker.pose.position.y = obs[i].origin[1];
        marker.pose.position.z = obs[i].origin[2];
        marker.scale.x = 2*obs[i].radius;
        marker.scale.y = 2*obs[i].radius;
        marker.scale.z = 2*obs[i].radius;
        marker_pub.publish(marker);
    }
}

/** controlIteration
 */
void controlIteration(int new_ref)
{
    int i;
    static double b;
    // Execute only when g_new_q is set, that is we have updated q
    if( (g_new_q == 1)){
        clock_gettime(CLOCK_MONOTONIC_RAW,&tv);
        tic = ((double)tv.tv_sec + 1.0e-9*tv.tv_nsec);

        rr_mtx.lock();          //< No need for mtxs, callbacks are executed now in same thread
        q_.data = g_q;          //< Pass global variables to controllers local ones, as before no needed currently
        x_ref = g_x_ref;
        R_ref = g_R_ref;
        new_ref = g_new_ref;
        g_new_ref = 0;
        rr_mtx.unlock();


        if(useFK == 1 ){        //< Calculate pose from urdf tree
            fkSolver_->JntToCart(q_,Fr_);
            toEigen(x_m,Fr_.p);
            toEigen(R_msr,Fr_.M);
        }
        else{                   //< Read x_m  from topic (g_x_msr, g_R_msr)
            rr_mtx.lock();
            x_m = g_x_msr;
            R_msr = g_R_msr;
            rr_mtx.unlock();
        }

        JacSolver_->JntToJac(q_,J_);

        //  Main task controller
        /**
        The dynamical system that we used in this implementation is the bio-inspired dynamical model VITE  [1],

        [1] S. Bullock, Daniel Grossberg "Neural dynamics of planned arm movements: Emergent invariants and
        speed-accuracy properties during trajectory formation", Psychol. Rev., 95 (1) (1988), pp. 4990
        */

        ep = x_m - x_ref;
        dR = R_msr.transpose() *R_ref  ; // If R_ref equals 0, dR and th =0, keeps orientation the same

        th = acos((dR.trace()-1)/2);
        if(fabs(th)>1e-8){
            kd(0) = (dR(2,1) - dR(1,2))/(2*sin(th)) ;
            kd(1) = (dR(0,2) - dR(2,0))/(2*sin(th)) ;
            kd(2) = (dR(1,0) - dR(0,1))/(2*sin(th)) ;
        }else{
            kd << 1,0,0;
            th = 0;
        }
        kd = R_msr*kd;


        if(new_ref == 1){       //< Executed only once when new refence pose is obtained
            pd = x_m;
            std::cout <<"x_ref:\n" << x_ref << "\n";
            std::cout <<"pd:\n" << pd << "\n";

            theta_d = th;
            pddot = Eigen::Vector3d::Zero(3,1);
            pdddot = Eigen::Vector3d::Zero(3,1);
            sol_prev = Eigen::VectorXd::Zero(Nj,1);

            theta_ddot = 0;
            theta_dddot = 0;
            new_ref = 0;
        }

        // If Q are 0, g_R_ref is set to 0, and dR->0
        // Second order VITE DS
        if ( (fabs(th) > 1e-8) && (th < (M_PI-1e-8)) )
            tmp_M3d = th/(2*dt*sin(th))*(dR - dR.transpose());
        else
            tmp_M3d = Eigen::Matrix3d::Zero(3,3);

        pdddot = alpha*(-pddot-beta*ep);
        pddot += pdddot*dt;

        if((x_m - pd).norm()<0.2)
            pd += pddot*dt;

        ep = pddot - 4.0*(x_m - pd);

        theta_dddot = alpha*(-theta_ddot + beta*th);
        theta_ddot += theta_dddot*dt;
        theta_d += theta_ddot*dt;

        if(fabs(th)<1e-8){
            theta_d = 0;
            theta_ddot = 0;
        }

        ew = kd * ( (theta_ddot)  - 0.01 * (th - theta_d) );
        // Proportional only
        // ew = -0.4*kd*th;
        // ep = -0.4* (x_m - x_ref);

    if(ENABLE_HQP_FOR_OBS){
      e << ep,ew;
    }else{
      e << ep + oaController->getControlSignal(x_m),ew;
    }

   // ROS_INFO("[main] control signal: [%f, %f, %f]",oaController->getControlSignal(x_m)[0],oaController->getControlSignal(x_m)[1],oaController->getControlSignal(x_m)[2]);
   //  ROS_INFO("[main] e: [%f, %f, %f, %f, %f, %f]",e[0],e[1],e[2],e[3],e[4],e[5]);

        // Update HQP solver joint limit
        hqpw->updBounds("JointLimMin",0.01*(qmin-q_.data)/dt);
        hqpw->updBounds("JointLimMax",0.01*(qmax-q_.data)/dt);
    //hqpw->print();

        // Update Obstacles
    if(ENABLE_HQP_FOR_OBS){
      for(i=0; i<obs.size();i++){
      n = x_m - obs[i].origin ;
      n.normalize();
      b =  0.01*(safeDst + obs[i].radius + n.dot(obs[i].origin - x_m) )/dt;
      //    printf("i:%u, 200.00Hzb:%f\n",i,b);
      //     std::cout << "n:\n" <<n <<"\n";
      hqpw->updObstacle(obs[i].name,J_.data,n,b);
      }
    }
        //  Update Task
        hqpw->updTask("Task",J_.data,e);

        // Solve...
        hqpw->solve(solution);
    //hqpw->print();

        // Apply safety velocity limits, could use limits form urdf in future version
        for(i = 0; i < Nj; i++){
            // Velocity Limit
            solution(i) = SATUR(solution(i),80.0*M_PI/180.0); //10dps
            // Acceleration Limit
            //                wd = (solution(i) - sol_prev(i)) / dt;
            //                wd = SATUR(wd,wd_max); //
            //                solution(i) = wd*dt + sol_prev(i);
            //                sol_prev(i) = solution(i);
        }

        // Send velocity solution to topic, position is propagated in order to inform low level controller about the expected end position of the current step.
        // For example if something goes wrong and communication is lost the low level controller should start reducing reference velocity if position is exceeded..!!!
        q_.data += 2.0*dt*solution; //Allow for a few lost samples.. (2x dt)
        // First Nj joints, are for the first arm!!! will probably change
        if( task_start == 0){
            for(i=0; i<Nj; i++){
                msg.position[i] = q_.data[i];
                msg.velocity[i] = 0;
                msg.effort[i] = 0;
            }
        }
        else{
            for(i=0; i<Nj; i++){
                msg.position[i] = q_.data[i];
                msg.velocity[i] = solution[i];
                msg.effort[i] = 0;
            }
        }
        msg.header.stamp = ros::Time::now();
        joint_publisher.publish(msg);

        //std::cout <<"ep:\n" << ep <<"\n\n";
        //std::cout <<"x_m:\n" << x_m <<"\n\n";
        //std::cout <<"x_ref:\n" << x_ref <<"\n\n";
        //std::cout <<"q:\n"<< q*180.0/M_PI <<"\n\n";

        clock_gettime(CLOCK_MONOTONIC_RAW,&tv);
        tic = ((double)tv.tv_sec + 1.0e-9*tv.tv_nsec) - tic;
        //printf("time:%f\n",tic);
   }
}

/** computeError
 * @ goal: desired end effector Pose
 *
 * Computes the error between the current
 * arm position and the desired position for the goal
 */
double computeError(geometry_msgs::Pose goal)
{
	KDL::Frame goal_frame, current_frame;
	KDL::Vector error;
	double d_error = 0.0;
	
	tf::poseMsgToKDL(goal, goal_frame);
	fkSolver_->JntToCart(q_, current_frame);

	error = current_frame.p - goal_frame.p;
	
	for (int i = 0; i < 3; i ++)
	{
		d_error += pow(error(i), 2);
	}

	d_error = sqrt(d_error);

	return d_error;
}


/** controlCallback
* @goal: actionlib goal received when server is called
 */
void controlCallback(const sarafun_hqp_omg::OnlineMotionGoalConstPtr &goal)
{
    int new_ref = g_new_ref;
    bool success = false;
	double error = 100000.0;
	sarafun_hqp_omg::OnlineMotionFeedback feedback;

    ROS_INFO("%s server called! Will execute", actionlib_server_name.c_str());

    getReferencePose(goal->ref);
    getObstacles();

    while( ros::ok() && (start == 1)){ //We need ROS, start (that is, initiliazied HQP) and a g_new_q
        //t1 = ros::Time::now().toSec();
        //std::cout <<t1 - t0<<" , " ;

         if (as->isPreemptRequested() || !ros::ok())
         {
             ROS_INFO("%s: Preempted", actionlib_server_name.c_str());
             // set the action state to preempted
             as->setPreempted();
             success = false;
             break;
         }

         if(useSim)
         {
             drawObstacles();
         }

         controlIteration(new_ref);
		 error = computeError(goal->ref);

		 feedback.error = error;
		 as->publishFeedback(feedback);

		 if (error < g_error_thres)
		 {
			 success = true;
			 break;
		 }

         //std::cout <<( ros::Time::now().toSec() - t1 )<<"\n" ;

         ros::spinOnce(); // Keep spin out of control loop to keep q,pos etc. up to date
         loop_rate->sleep();
     }

     if(success)
     {
         ROS_INFO("%s: Succeeded", actionlib_server_name.c_str());
         // set the action state to succeeded
         as->setSucceeded();
     }
}

int main(int argc, char *argv[])
{
    int i;

    ros::init(argc, argv, "hqp_node");
    if(ros::master::check()){
        ROS_INFO_STREAM("HQP client saluts you...");
        actionlib_server_name = "sarafun/motion/online";

        nh = new ros::NodeHandle("~"); //http://answers.ros.org/question/135181/roscpp-parameters-with-roslaunch-remapping/
        as = new actionlib::SimpleActionServer<sarafun_hqp_omg::OnlineMotionAction> (actionlib_server_name, boost::bind(&controlCallback, _1), false);

        loop_rate = new ros::Rate(LOOP_FREQ);

        if(!model.initParam(ros::this_node::getName()+ "/robot_description")){
            ROS_ERROR("ERROR getting robot description");
        }

        kdl_parser::treeFromUrdfModel(model, tree_);

        // Read first chain from urdf, starts from base link until end effector
        std::map<std::string,KDL::TreeElement>::const_iterator root;
        std::map<std::string,KDL::TreeElement> leafs;
        root = tree_.getRootSegment();
        leafs = tree_.getSegments();
        std::string chain_root = root->first;
        KDL::SegmentMap::const_iterator s;
        s=tree_.getSegment(chain_root);
        while( (s != leafs.end()) && (!s->second.children.empty()) ){
            s=s->second.children[0]; // Reads only first chain currently..
            std::cout << "Joint Names: " << s->second.segment.getJoint().getName() << ", Type:"<<s->second.segment.getJoint().getTypeName()<< " \n";
            if ( s->second.segment.getJoint().getType() == KDL::Joint::RotAxis){
                msg.position.push_back(0);
                msg.velocity.push_back(0);
                msg.effort.push_back(0);
                msg.name.push_back(s->second.segment.getJoint().getName());
            }
        }
        tree_.getChain(chain_root, s->first, chain_);
        Nj = chain_.getNrOfJoints();
        fkSolver_ = new KDL::ChainFkSolverPos_recursive(chain_);
        JacSolver_ = new KDL::ChainJntToJacSolver(chain_);
		ikSolver_ = new KDL::ChainIkSolverPos_LMA(chain_);
        std::cout << "Get chain from:"<< chain_root <<", to:"<< s->first <<", Nj:" << Nj <<"\n";

        // !!! Currently used only for Yumi Initialisation !!! (if used with KUKA same joints are repeated.. not a problem but will be fixed)
        //Get second chain, and add them to msg
        s=tree_.getSegment(chain_root);
        while( (s != leafs.end()) && (!s->second.children.empty()) ){
            s=s->second.children[s->second.children.size()-1]; // Reads only second chain currently..
            std::cout << "Second Chain Joint Names: " << s->second.segment.getJoint().getName() <<"\n";
            msg.position.push_back(0);
            msg.velocity.push_back(0);
            msg.effort.push_back(0);
            msg.name.push_back(s->second.segment.getJoint().getName());
        }


        if(!nh->getParam("autoStart",start)){
            ROS_ERROR("ERROR getting autoStart");
        }
        ROS_INFO("autoStart set to:%d", start);

        if(!nh->getParam("useSim",useSim)){
            ROS_ERROR("ERROR getting useSim");
        }
        ROS_INFO("useSim set to:%d", useSim);

        if(!nh->getParam("safeDst",safeDst)){
            ROS_ERROR("ERROR getting safeDst");
        }
        ROS_INFO("safeDst set to:%f", safeDst);

        if(!nh->getParam("joint_lim_min",joint_lim_min)){
            // If not provided read from urdf
            std::cout <<"Reading:" << msg.name[0] <<"\n";
            joint_lim_min.resize(Nj);
            for(i = 0; i<Nj; i++){
                joint_lim_min[i] = model.getJoint(msg.name[i])->limits->lower;
            }
            ROS_INFO("Getting joint_lim_min from urdf");
        }

        if(!nh->getParam("joint_lim_max",joint_lim_max)){
            // If not provided read from urdf, limits in urdf in degress..
            joint_lim_max.resize(Nj);
            for(i = 0; i<Nj; i++){
                joint_lim_max[i] = model.getJoint(msg.name[i])->limits->upper;
            }
            ROS_INFO("Getting joint_lim_max from urdf");
        }

        x_ref = Eigen::Vector3d::Zero(3,1);
        R_ref = Eigen::Matrix3d::Zero(3,3);
        q = Eigen::VectorXd::Zero(Nj,1);
        qmin = Eigen::VectorXd::Zero(Nj,1);
        qmax = Eigen::VectorXd::Zero(Nj,1);
        solution = Eigen::VectorXd::Zero(Nj,1);
        ep = Eigen::Vector3d::Zero(3,1);
        ew = Eigen::Vector3d::Zero(3,1);
        e = Eigen::VectorXd::Zero(Ntask,1);
        x_m = Eigen::Vector3d::Zero(3,1);
        n = Eigen::Vector3d::Zero(3,1);
        J_.resize(Nj);
        q_.resize(Nj);
        Fr_ = KDL::Frame::Identity();
		geometry_msgs::Pose init_pose;

		init_pose.position.x = 0;
		init_pose.position.y = 0;
		init_pose.position.z = 0;
		init_pose.orientation.w = 1;
		init_pose.orientation.x = 0;
		init_pose.orientation.y = 0;
		init_pose.orientation.z = 0;


        getReferencePose(init_pose);
        getObstacles();

        // Need a topic to read robot's q
        if(!nh->getParam("msr_q_topic",msr_q_topic)){
            ROS_ERROR("ERROR getting msr_q_topic \n");
        }
        msr_q_subscr = nh->subscribe(msr_q_topic,1,msr_q_callback,ros::TransportHints().tcpNoDelay(true));
        ROS_INFO("Using q from topic:%s ",msr_q_topic.c_str());

        // Need a topic to read robot's pose
        if(!nh->getParam("msr_pose_topic",msr_pose_topic)){
            ROS_ERROR("ERROR getting msr_pos_topic \n");
        }
        if(msr_pose_topic == "NULL"){
            ROS_INFO("Using pose from FK solver");
            useFK = 1;
        }
        else{
            ROS_INFO("Using pose from topic:%s",msr_pose_topic.c_str());
            useFK = 0;
            msr_pos_subscr = nh->subscribe(msr_pose_topic,1,msr_pose_callback,ros::TransportHints().tcpNoDelay(true));
        }

        // Need a topic to read ref pos
        if(!nh->getParam("ref_pose_topic",ref_pose_topic)){
            ROS_ERROR("ERROR getting ref_pos_topic");
        }
        ref_pos_subscr = nh->subscribe(ref_pose_topic,1,ref_pose_callback,ros::TransportHints().tcpNoDelay(true));
        ROS_INFO("Using ref pose from topic:%s",ref_pose_topic.c_str());

        // And to publish, q,dq
        if(!nh->getParam("cmd_q_topic",cmd_q_topic)){
            ROS_ERROR("ERROR getting robot_cmd_q_topic");
        }
        joint_publisher = nh->advertise<sensor_msgs::JointState>(cmd_q_topic, 1);
        ROS_INFO("Publishing cmd'ed q in topic:%s \n",cmd_q_topic.c_str());


        init_Service = nh->advertiseService("init_HQP", init_hqp);
		if(ENABLE_HQP_FOR_OBS){
		  addObs_Service = nh->advertiseService("addObs_HQP", addObs);
		}else{
		  addObs_Service = nh->advertiseService("addObs_HQP", addObs_sarafun);
		}

        for(i = 0; i<Nj; i++){
            qmin(i) = joint_lim_min[i];
            qmax(i) = joint_lim_max[i];
            q_(i) = 0 ;
        }
        std::cout <<"qmin:\n" <<qmin.transpose() <<"\n"<<"---------------\n";
        std::cout <<"qmax:\n" <<qmax.transpose() <<"\n"<<"---------------\n";

        if(start == 1){
            PROJECT_NAME::initHQP::Request req;
            PROJECT_NAME::initHQP::Response res;
            init_hqp(req,res);
        }

        // Used only when useSim is set
        marker_pub = nh->advertise<visualization_msgs::Marker>("/visualization_marker", 1);
        marker.header.frame_id = chain_root;
        marker.header.stamp = ros::Time::now();
        marker.ns = "basic_shapes";
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;
        marker.lifetime = ros::Duration();
    }
    else{
        return -1;  //
    }

    std::cout <<"Rref:\n" << R_ref <<"\n";
    tmp_M3d = Eigen::Matrix3d::Zero(3,3);
    ROS_INFO("Starting hqp loop");


    int new_ref = g_new_ref;
    double t1 = 0;
    double t0 = ros::Time::now().toSec();
    wd_max = 20;
    alpha = 1.0;
    beta = alpha/4.0;
    kd << 1,0,0;

    as->start();
    ros::spin();

    ROS_INFO_STREAM("HQP is going down!");
    ros::shutdown();
    return 0;
}
