#include <QCoreApplication>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <vector>
#include <fstream>
#include <list>
#include <vector>
#include <chrono>
#include <ctime>
#include <climits>

// for opencv
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <boost/concept_check.hpp>
// for g2o
#include "g2o_types.h"
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/types/slam3d/se3quat.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <sys/time.h>

using namespace std;
using namespace Eigen;
#define random(x) (rand()%x)


int main ( int argc, char** argv )
{
    srand( (unsigned int) time(0) );

    Vector3d temp;
    vector<Vector3d> posL,posR;
    for(int i1 = 0;i1<100;i1++)
    {
        temp(0) = random(50);
        temp(1) = random(50);
        temp(2) = random(50);
        posL.push_back(temp);
    }

    for(int i2 = 0;i2<100;i2++)
    {
        temp(0) = random(10);
        temp(1) = random(10);
        temp(2) = random(10);
        posR.push_back(temp);
    }

    Eigen::Isometry3d Tcw = Eigen::Isometry3d::Identity();
        // 使用直接法计算相机运动及投影点
        chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
        //poseEstimationDirect( measurements, &gray, K, Tcw );

        // 初始化g2o
        typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,1>> DirectBlock;  // 求解的向量是6＊1的
        DirectBlock::LinearSolverType* linearSolver = new g2o::LinearSolverDense< DirectBlock::PoseMatrixType > ();
        DirectBlock* solver_ptr = new DirectBlock( linearSolver );
        g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton( solver_ptr );
        g2o::SparseOptimizer optimizer;
        optimizer.setAlgorithm( solver );
        optimizer.setVerbose( true );       // 打开调试输出

        g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap();
        pose->setEstimate( g2o::SE3Quat(Tcw.rotation(), Tcw.translation()) );
        pose->setId(0);
        optimizer.addVertex( pose );

        // 添加边
        int id=1;
        for(int i=0;i<100;i++)
        {
            g2o::EdgeSE3ProjectDirect* edge = new g2o::EdgeSE3ProjectDirect(
                posL[i],
               &posL,&posR
            );
            edge->setVertex( 0, pose );
            edge->setMeasurement(posR[i]);
            edge->setInformation( Eigen::Matrix<double,1,1>::Identity() );
            edge->setId( id++ );
            optimizer.addEdge(edge);
        }
        cout<<"edges in graph: "<<optimizer.edges().size()<<endl;
        optimizer.initializeOptimization();
        optimizer.optimize(10);
        Tcw = pose->estimate();

        chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
        chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>( t2-t1 );
        cout<<"direct method costs time: "<<time_used.count()<<" seconds."<<endl;
        cout<<"Tcw="<<Tcw.matrix()<<endl;

    return 0;
}

