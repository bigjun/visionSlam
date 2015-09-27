#include "slamEnd.h"

SlamEnd::SlamEnd() {

    SlamLinearSolver* linearSolver = new SlamLinearSolver();
    linearSolver->setBlockOrdering(false);
    SlamBlockSolver* blockSolver = new SlamBlockSolver(linearSolver);
    g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg( blockSolver );

    _globalOptimizer.setAlgorithm( solver );
    _globalOptimizer.setVerbose( false );

    _robustKernel = g2o::RobustKernelFactory::instance()->construct("Cauchy");
}


int SlamEnd::addVertex( int index, bool setFixed) {

     g2o::VertexSE3* v = new g2o::VertexSE3();
     v->setId( index );
     v->setEstimate( Eigen::Isometry3d::Identity() );
     v->setFixed(setFixed);
     _globalOptimizer.addVertex(v);
     return _globalOptimizer.vertices().size();
 }


 int  SlamEnd::addEdge(int index1, int index2, Eigen::Isometry3d measure, Eigen::Matrix<double, 6, 6> information) {

     g2o::EdgeSE3* edge = new g2o::EdgeSE3();
     edge->vertices()[0] = _globalOptimizer.vertex(index1);
     edge->vertices()[1] = _globalOptimizer.vertex(index2);
     edge->setInformation(information);
     edge->setMeasurement(measure);
     edge->setRobustKernel(_robustKernel);
     _globalOptimizer.addEdge(edge);

 }
