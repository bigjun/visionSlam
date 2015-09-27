#ifndef SLAMEND_H_
#define SLAMEND_H_

#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/factory.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_factory.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>

//g2o define
typedef g2o::BlockSolver_6_3 SlamBlockSolver;
typedef g2o::LinearSolverCSparse< SlamBlockSolver::PoseMatrixType >  SlamLinearSolver;


class  SlamEnd
{
public:
    SlamEnd ();
    int  addVertex( int index, bool setFixed = false);
    int  addEdge(int index1, int index2, Eigen::Isometry3d measure, Eigen::Matrix<double, 6, 6> information);
private:
     g2o::SparseOptimizer _globalOptimizer;
     g2o::RobustKernel* _robustKernel;

};

#endif // SLAMEND_H_
