
#include "hqp_wrapper.h"

/**
 * @brief hqp_wrapper::hqp_wrapper
 * @param NC, in this context, the number of joints (q size)
 */
hqp_wrapper::hqp_wrapper(int NC): NC(NC){
    hsolver = new soth::HCOD(NC,0);
    solution = new soth::VectorXd(NC);
}

/**
 * @brief hqp_wrapper::addStage Adds stage (Jq <=> B)
 * @param levelName     String to be used as ID
 * @param J
 * @param B
 * @param bound         Type of bound (<,>,=)
 * @return
 */
int hqp_wrapper::addStage(std::string levelName,Eigen::MatrixXd J,  Eigen::VectorXd B, soth::Bound::bound_t bound){
    soth::VectorBound vB;
    std::cout <<"B.rows:"<<B.rows()<<"\n";
    vB.resize(B.rows());
    for(int i=0;i<B.rows();i++){
        vB[i] = soth::Bound(B[i],bound);
    }
    this->J.push_back(J);
    this->B.push_back(vB);
    this->Btyp.push_back(bound);
    leveNameMap.insert(std::make_pair(levelName,this->B.size()-1));
    return 0;
}

/**
 * @brief hqp_wrapper::addObstacle
 * @param levelName         String to be used as ID
 * @param Jac               Jacobian (6xNc), only the first 3 rows (position) is used currently
 * @param n                 Normalised Vector between end effector and obstacles' position
 * @param b                 scalar b, n*J < b
 */
void hqp_wrapper::addObstacle(std::string levelName, const Eigen::MatrixXd Jac, const Eigen::Vector3d n, double b){
    Eigen::MatrixXd Jtmp;
    Jtmp = n.transpose()*Jac.block(0,0,3,this->NC);
    Eigen::Matrix<double,1,1>  B;
    B(0) = b;
    addStage(levelName,Jtmp,B,soth::Bound::BOUND_INF);
    std::cout <<"Adding new element:" << levelName <<", at:" << leveNameMap[levelName]<<"\n";
}

/**
 * @brief hqp_wrapper::updObstacle Updates obstacles' matrices
 * @param levelName                 Level's ID
 * @param Jac                       Jacobian (6xNc)
 * @param n                         Normalised Vector between end effector and obstacles' position
 * @param b                         scalar b, n*J < b
 */
void hqp_wrapper::updObstacle(std::string levelName, const Eigen::MatrixXd Jac, const Eigen::Vector3d n, const double b ){
    int indx = leveNameMap[levelName];
    this->B[indx][0] = soth::Bound(b, soth::Bound::BOUND_INF);
    this->J[indx] = n.transpose()*Jac.block(0,0,3,this->NC);
    //  std::cout << "J[indx]:\n" <<J[indx] <<"\n";

}

/**
 * @brief hqp_wrapper::updStage Updates matrix J, and vector B of the current level's in/equaltity (Jq <=> B). Type (<,>,=) is kept the same as init
 * @param indx          Level's index
 * @param J
 * @param B
 */
void hqp_wrapper::updStage(int indx, const Eigen::MatrixXd J, const Eigen::VectorXd B){
    this->J[indx] = J;
    for(int i = 0; i < B.rows(); i++){
        this->B[indx][i] = soth::Bound(B[i],this->Btyp[indx]);
    }
}

/**
 * @brief hqp_wrapper::updStage same as above uses string as ID instead of index
 * @param levelName String used as level ID
 */
void hqp_wrapper::updStage(std::string levelName, const Eigen::MatrixXd J, const Eigen::VectorXd B){
    this->updStage(leveNameMap[levelName], J, B);
}

/**
 * @brief hqp_wrapper::updBounds
 * @param levelName
 * @param B
 */
void hqp_wrapper::updBounds(std::string levelName, const Eigen::VectorXd B){
    int indx = leveNameMap[levelName];
    for(int i=0;i<B.rows();i++){
        this->B[indx][i] = soth::Bound(B[i],this->Btyp[indx]);
    }
}


/**
 * @brief hqp_wrapper::updTask
 * @param levelName     A String used as an ID of the level
 * @param Jac           Jacobian of the current task
 * @param e             Task error vector (3x pose + 3x orientation)
 */
void hqp_wrapper::updTask(std::string levelName, const Eigen::MatrixXd Jac, const Eigen::VectorXd e){
    int indx = leveNameMap[levelName];
    //for(int k = 0; k < e.rows(); k++){
    for(int k = 0; k < 6; k++){
        this->B[indx][k] = e(k);
        for(int j=0;j<7;j++){
            J[indx](k,j) = Jac(k,j);
        }
    }
}

/**
 * @brief hqp_wrapper::print
 * Just printing inequalities of all levels set
 */
void hqp_wrapper::print(void){
    soth::VectorBound vB;
    soth::Bound B;
    soth::Bound::bound_t typ;
    double val;
    for(int i = 0; i<this->B.size();i++){
        vB = this->B[i];
        for(int j=0;j<vB.rows();j++){
            B = vB[j];
            typ = B.getType();
            val = B.getBound(typ);
            printf("Val: %f, type:%d\n",val,typ);
        }
        printf("-------------\n");
    }
}

/**
 * @brief hqp_wrapper::init
 * Initialises HQP solver, matrix J, vector B, are passed by reference so keep them intact in the main app...
 * Keep damping of main Task increased to smooth solution during singularities (tested with 0.2 with KUKA LWR)
 */
void hqp_wrapper::init(void){
    for(int i=0;i<this->B.size();i++){
        hsolver->pushBackStage(this->J[i],this->B[i]);
    }
    hsolver->setNameByOrder("level");
    hsolver->setDamping(0.00001);
    if(leveNameMap.count("Task") > 0){ // Check  if "Task" exists
        hsolver->stage(leveNameMap["Task"]).damping(0.2);
    }
    hsolver->setInitialActiveSet();
    hsolver->initialize();
    //hsolver->show(std::cout,true);
}



