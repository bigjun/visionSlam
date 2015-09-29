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
    //边
     g2o::EdgeSE3* edge = new g2o::EdgeSE3();
     // 连接此边的两个定点
     edge->vertices()[0] = _globalOptimizer.vertex(index1);
     edge->vertices()[1] = _globalOptimizer.vertex(index2);
     //信息矩阵
     edge->setInformation(information);
     //观测值
     edge->setMeasurement(measure);
     edge->setRobustKernel(_robustKernel);
     //把此边加入地图
     _globalOptimizer.addEdge(edge);

 }


 void SlamEnd::checkNearbyLoops(std::vector<FRAME>& frames, FRAME frame){

 }

 void SlamEnd::checkRandomLoops(std::vector<FRAME>& frames, FRAME frame) {

 }

 CHECK_RESULT SlamEnd::checkKeyframes(FRAME& f1, FRAME& f2) {

 }
