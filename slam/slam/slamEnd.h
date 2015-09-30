#ifndef SLAMEND_H_
#define SLAMEND_H_

#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/factory.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_factory.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>

#include "../registration/registration.h"
#include "../feature/feature_matcher.h"
#include "../tool/pcl_utils.h"
#include "../tool/eigen_extensions.h"

//g2o define
typedef g2o::BlockSolver_6_3 SlamBlockSolver;
typedef g2o::LinearSolverCSparse< SlamBlockSolver::PoseMatrixType >  SlamLinearSolver;

// 检测两个帧，结果定义
enum CHECK_RESULT {NOT_MATCHED=0, TOO_FAR_AWAY, TOO_CLOSE, KEYFRAME};

class  SlamEnd
{
public:
    SlamEnd ();
    ~SlamEnd ();
    int  addVertex( int index, bool setFixed = false);
    int  addEdge(int index1, int index2, Eigen::Isometry3d measure, Eigen::Matrix<double, 6, 6> information);
    void checkNearbyLoops(std::vector<FRAME>& frames, FRAME& frame );
    void checkRandomLoops(std::vector<FRAME>& frames, FRAME& frame );
    CHECK_RESULT checkKeyframes(FRAME& f1, FRAME& f2, bool is_loops = false);
    static void  transformToFars(Eigen::Matrix4d& transform, double& angle , double& dist ){
        angle = Eigen::AngleAxisd( transform.block<3,3>(0,0) ).angle();
        dist = transform.block<3,1>(0,3).norm();
    }
    Eigen::Vector7d estimateMotion(FRAME& lastFrame, FRAME& currentFrame, int& inlierNum);
    void save (std::string saveStr);
    void optimize(int step);
    Eigen::Isometry3d getPos(int frameID);
    void clear(void);

private:
     g2o::SparseOptimizer _globalOptimizer;
     g2o::RobustKernel* _robustKernel;


};

#endif // SLAMEND_H_
