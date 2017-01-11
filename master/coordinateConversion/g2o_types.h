#ifndef G2O_TYPES_DIRECT_H_
#define G2O_TYPES_DIRECT_H_

// 定义应用于直接法的g2o边
#include <Eigen/Geometry>
#include <g2o/core/base_unary_edge.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <opencv2/core/core.hpp>
#include <vector>
using namespace std;
namespace g2o
{
// project a 3d point into an image plane, the error is photometric error
// an unary edge with one vertex SE3Expmap (the pose of camera)
class EdgeSE3ProjectDirect: public BaseUnaryEdge< 1, Vector3D, VertexSE3Expmap>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeSE3ProjectDirect() {}

    EdgeSE3ProjectDirect( Eigen::Vector3d point, vector<Vector3D> * posL,vector<Vector3D> * posR )
    : x_world_(point), posL_(posL),posR_(posR)
    {}

    virtual void computeError();

    // plus in manifold
   // virtual void linearizeOplus( );

    // dummy read and write functions because we don't care...
    virtual bool read( std::istream& in );
    virtual bool write( std::ostream& out ) const ;

protected:
    int findIdofPoint(Vector3D pos);
    //float getPixelValue( float x, float y );
public:
    // 3d point in world frame
    Eigen::Vector3d x_world_;
    // Pinhole camera intrinsics
    //float cx_=0, cy_=0, fx_=0, fy_=0;
    // the reference image
    vector<Vector3D> *posL_=nullptr;
    vector<Vector3D> *posR_=nullptr;
};
} // namespace g2o
#endif // G@O_TYPES_DIRECT_H_

