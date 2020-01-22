#ifndef G2OTYPES_H_
#define G2OTYPES_H_



#include "g2o/core/base_unary_edge.h"
#include "g2o/core/base_multi_edge.h"
#include "g2o/types/slam3d/vertex_se3.h"
#include "g2o/types/slam3d/edge_se3.h"
#include "g2o/types/sba/types_six_dof_expmap.h"

#include "g2o/types/slam3d/se3quat.h"
#include "g2o/types/slam3d/types_slam3d.h"

#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"
// #include "g2o/types/sba/types_six_dof_expmap.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/solvers/dense/linear_solver_dense.h"
#include "g2o/types/sim3/types_seven_dof_expmap.h"
#include "g2o/types/slam3d/edge_se3_pointxyz.h"

#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_with_hessian.h"
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"


#include "thirdparty/sophus/sophus/se3.hpp"

// using namespace vill;

using namespace Eigen;

using namespace Sophus;
#define TEST_STEREO

namespace g2o {
	typedef Matrix<int, 3, 1> Vector3i;
	
	class  EdgeSE3ProjectXYZ: public  BaseBinaryEdge<2, Vector2d, VertexSBAPointXYZ, VertexSE3Expmap>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		EdgeSE3ProjectXYZ() {}

		bool read ( std::istream& is );

		bool write ( std::ostream& os ) const;

		void computeError()  {
			const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*> ( _vertices[1] );
			const VertexSBAPointXYZ* v2 = static_cast<const VertexSBAPointXYZ*> ( _vertices[0] );
			Vector2d obs ( _measurement );
			_error = obs-cam_project ( v1->estimate() * ( v2->estimate() ) );
		}

		bool isDepthPositive() {
			const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*> ( _vertices[1] );
			const VertexSBAPointXYZ* v2 = static_cast<const VertexSBAPointXYZ*> ( _vertices[0] );
			return ( v1->estimate() * ( v2->estimate() ) ) ( 2 ) >0.0;
		}


		virtual void linearizeOplus();

		Vector2d cam_project ( const Vector3d & trans_xyz ) const;

		double fx, fy, cx, cy;
	}; // EO  EdgeSE3ProjectXYZ
	
	class  EdgeStereoSE3ProjectXYZ: public  BaseBinaryEdge<3, Vector3d, VertexSBAPointXYZ, VertexSE3Expmap>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		EdgeStereoSE3ProjectXYZ();

		bool read ( std::istream& is );

		bool write ( std::ostream& os ) const;

		void computeError()  {
			const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*> ( _vertices[1] );
			const VertexSBAPointXYZ* v2 = static_cast<const VertexSBAPointXYZ*> ( _vertices[0] );
			Vector3d obs ( _measurement );
			_error = obs - cam_project ( v1->estimate() * ( v2->estimate() ),bf );
		}

		bool isDepthPositive() {
			const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*> ( _vertices[1] );
			const VertexSBAPointXYZ* v2 = static_cast<const VertexSBAPointXYZ*> ( _vertices[0] );
			return ( v1->estimate() * ( v2->estimate() ) ) ( 2 ) >0.0;
		}


		virtual void linearizeOplus();

		Vector3d cam_project ( const Vector3d & trans_xyz, const float &bf ) const;

		double fx, fy, cx, cy, bf;
	};
	
	class  EdgeSE3ProjectXYZOnlyPose: public  BaseUnaryEdge<2, Vector2d, VertexSE3Expmap>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		EdgeSE3ProjectXYZOnlyPose() {}

		bool read ( std::istream& is );

		bool write ( std::ostream& os ) const;

		void computeError()  {
			const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*> ( _vertices[0] );
			Vector2d obs ( _measurement );
			_error = obs-cam_project ( v1->estimate() * ( Xw ) );
		}

		bool isDepthPositive() {
			const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*> ( _vertices[0] );
			return ( v1->estimate() * ( Xw ) ) ( 2 ) >0.0;
		}


		virtual void linearizeOplus();

		Vector2d cam_project ( const Vector3d & trans_xyz ) const;

		Vector3d Xw;
		double fx, fy, cx, cy;
	};
	
	class  EdgeStereoSE3ProjectXYZOnlyPose: public  BaseUnaryEdge<3, Vector3d, VertexSE3Expmap>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		EdgeStereoSE3ProjectXYZOnlyPose() {}

		bool read ( std::istream& is );

		bool write ( std::ostream& os ) const;

		void computeError()  {
			const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*> ( _vertices[0] );
			Vector3d obs ( _measurement );
			_error = obs - cam_project ( v1->estimate() * ( Xw ) );
		}

		bool isDepthPositive() {
			const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*> ( _vertices[0] );
			return ( v1->estimate() * ( Xw ) ) ( 2 ) >0.0;
		}


		virtual void linearizeOplus();

		Vector3d cam_project ( const Vector3d & trans_xyz ) const;

		Vector3d Xw;
		double fx, fy, cx, cy, bf;
	};	

}

#endif // G2OTYPES_H
