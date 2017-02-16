#ifndef HQP_WRAPPER_H
#define HQP_WRAPPER_H


#include <soth/HCOD.hpp>
#include <eigen3/Eigen/Dense>


#define SATUR(x,Lim) ((x)>(Lim))?(Lim):(((x)<-(Lim))?(-(Lim)):(x))
#define SATUR2(x,Low,High) ((x)>(High))?(High):(((x)<(Low))?(Low):(x))

class hqp_wrapper
{
public:
    hqp_wrapper(int NC);
    int addStage(std::string levelName, Eigen::MatrixXd J,  Eigen::VectorXd B , soth::Bound::bound_t bound);
    void addObstacle(std::string levelName, const Eigen::MatrixXd Jac, const Eigen::Vector3d n, double b);

    void updStage(std::string levelName, const Eigen::MatrixXd J, const Eigen::VectorXd B);
    void updStage(int indx, const Eigen::MatrixXd J, const Eigen::VectorXd B);
    void updBounds(std::string levelName, const Eigen::VectorXd B);
    void updTask(std::string levelName, const Eigen::MatrixXd J, const Eigen::VectorXd e);
    void updObstacle(std::string levelName, const Eigen::MatrixXd J, const Eigen::Vector3d n, const double b);

    void init(void);
    void print(void);
    void solve(Eigen::VectorXd &solution){ this->hsolver->activeSearch(solution); }

private:
    hqp_wrapper():NC(0){}

    soth::HCOD *hsolver;                        /**< HQP solver class */
    soth::VectorXd *solution;                   /**< Solution vector */
    std::vector<Eigen::MatrixXd> J;             /**< Vector of Jacobian matrices */
    std::vector<soth::VectorBound> B;           /**< Vector of level's B vectors ( Jq <=> B) */
    std::vector<soth::Bound::bound_t> Btyp;     /**< Vector of level's bound type */
    std::map<std::string,int> leveNameMap;      /**< Map between level's string ID and index */
    const int NC;                               /**< Number of joints, DOF */

};

#endif // HQP_WRAPPER_H
