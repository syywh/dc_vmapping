#include "g2otypes.h"
#include "dc_optimizer.h"



using namespace vill;
using namespace std;
double dc_optimizer::optimize_BA_with_dc_error()
{
	vector<KeyFrame*> vKeyFrames = pMap->GetAllKeyFrames();
	vector<MapPoint*> vMapPoints = pMap->GetAllMapPoints();
	
	// Setup optimizer
	g2o::SparseOptimizer optimizer;
	g2o::BlockSolver_6_3::LinearSolverType *linearSolver;
	linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

	g2o::BlockSolver_6_3 *solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

	g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
	optimizer.setAlgorithm(solver);

	unsigned long maxKFid = 0;

	map<int, int> m_kfid_seqid;
	int kfid = 0;
	for(size_t i = 0; i < vKeyFrames.size(); i++){

		KeyFrame* pKFi = vKeyFrames[i];
		
		g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();
		vSE3->setEstimate( Converter_g2o::toSE3Quat(pKFi->GetPose().cast<double>()));
		vSE3->setId(pKFi->mnId);
		vSE3->setFixed(pKFi->mnId == 0);
		optimizer.addVertex(vSE3);
		if (pKFi->mnId > maxKFid)
			maxKFid = pKFi->mnId;
		
		m_kfid_seqid[pKFi->mnId] = kfid++;
	}
	
	int keyframe_size = vKeyFrames.size();
	const int mappoint_size = vMapPoints.size();
	
	const int nExpectedSize = keyframe_size * mappoint_size;
	
	vector<g2o::EdgeSE3ProjectXYZ *> vpEdgesMono;
	vpEdgesMono.reserve(nExpectedSize);

	vector<KeyFrame *> vpEdgeKFMono;
	vpEdgeKFMono.reserve(nExpectedSize);

	vector<MapPoint *> vpMapPointEdgeMono;
	vpMapPointEdgeMono.reserve(nExpectedSize);

	vector<g2o::EdgeStereoSE3ProjectXYZ *> vpEdgesStereo;
	vpEdgesStereo.reserve(nExpectedSize);

	vector<KeyFrame *> vpEdgeKFStereo;
	vpEdgeKFStereo.reserve(nExpectedSize);

	vector<MapPoint *> vpMapPointEdgeStereo;
	vpMapPointEdgeStereo.reserve(nExpectedSize);

	const float thHuberMono = sqrt(5.991);
	const float thHuberStereo = sqrt(7.815);
	
	
	map<int, int> m_mpid_seqid;
	int mpid = 0;
	for ( size_t i = 0; i < vMapPoints.size(); i++){
		
		MapPoint* pMP = vMapPoints[i];
		
		g2o::VertexSBAPointXYZ *vPoint = new g2o::VertexSBAPointXYZ();
		vPoint->setEstimate(pMP->GetWorldPos().cast<double>());
		int id = pMP->mnId + maxKFid + 1;
		vPoint->setId(id);
		vPoint->setMarginalized(true);
		optimizer.addVertex(vPoint);
		
		m_mpid_seqid[pMP->mnId] = mpid++;
		
		const map<KeyFrame *, size_t> observations = pMP->GetObservations();
		
		//Set edges
		for (map<KeyFrame *, size_t>::const_iterator mit = observations.begin(), 
			mend = observations.end();	mit != mend; mit++) 
		{
			KeyFrame *pKFi = mit->first;	
			if (pKFi->isBad()) continue;
			
			const cv::KeyPoint& kpUn = pKFi->mvKeys[mit->second];
			
			//Monocular observations
			if (pKFi->mvuRight[mit->second] < 0) {
				Eigen::Matrix<double, 2, 1> obs;
				obs << kpUn.pt.x , kpUn.pt.y;
				
				g2o::EdgeSE3ProjectXYZ *e = new g2o::EdgeSE3ProjectXYZ();
				
				e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
				e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId)));
				e->setMeasurement(obs);
				const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
				e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);
				
				g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
				e->setRobustKernel(rk);
				rk->setDelta(thHuberMono);
				
				e->fx = pKFi->fx;
				e->fy = pKFi->fy;
				e->cx = pKFi->cx;
				e->cy = pKFi->cy;

				optimizer.addEdge(e);
				vpEdgesMono.push_back(e);
				vpEdgeKFMono.push_back(pKFi);
				vpMapPointEdgeMono.push_back(pMP);
				
			} else {
				Eigen::Matrix<double, 3, 1> obs;
				const float kp_ur = pKFi->mvuRight[mit->second];
				obs << kpUn.pt.x, kpUn.pt.y, kp_ur;
				
				g2o::EdgeStereoSE3ProjectXYZ *e = new g2o::EdgeStereoSE3ProjectXYZ();
				e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
				e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKFi->mnId)));
				e->setMeasurement(obs);        
				
				const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
				Eigen::Matrix3d Info = Eigen::Matrix3d::Identity() * invSigma2;
				e->setInformation(Info);
				
				g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
				e->setRobustKernel(rk);
				rk->setDelta(thHuberStereo);
				
				e->fx = pKFi->fx;
				e->fy = pKFi->fy;
				e->cx = pKFi->cx;
				e->cy = pKFi->cy;
				e->bf = pKFi->mbf;

				optimizer.addEdge(e);
				vpEdgesStereo.push_back(e);
				vpEdgeKFStereo.push_back(pKFi);
				vpMapPointEdgeStereo.push_back(pMP);
			}
			
		}
	}
	
	double res = 0;
	
	for(size_t i = 0; i < vpEdgesStereo.size(); i++){
		
		g2o::EdgeStereoSE3ProjectXYZ *e = vpEdgesStereo[i];
		e->computeError();
		res += e->error().norm();
	}
	
	//update from depth completion
	
	
	for(size_t i = 0; i < vKeyFrames.size(); i++){
		
		KeyFrame* pKFi = vKeyFrames[i];
		if(pKFi->loss==0) continue;
		
		g2o::VertexSE3Expmap *vKFi = static_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(pKFi->mnId));
		g2o::SE3Quat init_kf_pose = vKFi->estimate(); //Tcw
		
		Eigen::MatrixXd Jacob(1, mappoint_size);
		Eigen::MatrixXd mappoints(4,mappoint_size);
		mappoints.setZero();
	
		
		vector<g2o::VertexSBAPointXYZ *> vpMapPoints;
		std::map<pair<int,int>, int>::iterator pt_mpid_iter = pKFi->m_pt_mpid.begin();
		for(; pt_mpid_iter!=pKFi->m_pt_mpid.end(); pt_mpid_iter++){
			//may not be necessary
			
			if(m_mpid_seqid.find(pt_mpid_iter->second) == m_mpid_seqid.end()) continue;
			int seq_id = m_mpid_seqid[pt_mpid_iter->second];
			Jacob(0, seq_id) = 
					pKFi->gradient_for_mp.at<float>(pt_mpid_iter->first.first,pt_mpid_iter->first.second);
					
			g2o::VertexSBAPointXYZ *vPoint = 
						static_cast<g2o::VertexSBAPointXYZ *>(optimizer.vertex(pt_mpid_iter->second+maxKFid+1));
			vpMapPoints.push_back(vPoint);		
			
			g2o::Vector3D init_p = vPoint->estimate();
			
			g2o::Vector3D p_in_C = init_kf_pose * init_p;
			for(int j = 0; j < 3; j++)
				mappoints(j,seq_id) = p_in_C(j);
			mappoints(3,seq_id) = 1;
			
			double b = pKFi->loss * Jacob(0, seq_id);
			double hessian = Jacob(0, seq_id) * Jacob(0, seq_id);
			double delta_z = -hessian * b;
			
			mappoints(2,seq_id) += delta_z;
			p_in_C(2) += delta_z;
			g2o::Vector3D update_p = init_kf_pose.inverse() * p_in_C;
			vPoint->setEstimate(update_p);
			
		}
		
// 		Eigen::MatrixXd Hessian(mappoint_size, mappoint_size);
// 		Eigen::MatrixXd b(1, mappoint_size);
// 		
// 		double cov = 1;
// 		b = pKFi->loss * cov * Jacob;
// 		Hessian = Jacob.transpose() * cov * Jacob;
// 		
// 		Eigen::MatrixXd delta_z(1,mappoint_size);
// 		delta_z = - Hessian.inverse() * b;
// 		
// 		for(int j = 0; j < vpMapPoints.size(); j++){
// 			
// 			g2o::VertexSBAPointXYZ *vPoint = vpMapPoints[j];
// 			int seq_id = m_mpid_seqid[vPoint->id()-maxKFid-1];
// 			
// 			mappoints(2,seq_id) += delta_z(0, seq_id);
// 			
// 			g2o::Vector3D update_p;
// 			update_p(0) = mappoints(0,seq_id);
// 			update_p(1) = mappoints(1,seq_id);
// 			update_p(2) = mappoints(2,seq_id);
// 			
// 			update_p = init_kf_pose.inverse() * update_p;
// 			
// 			vPoint->setEstimate(update_p);
// 			
// 		}
		
		res = 0;
	
		for(size_t i = 0; i < vpEdgesStereo.size(); i++){
			
			g2o::EdgeStereoSE3ProjectXYZ *e = vpEdgesStereo[i];
			e->computeError();
			res += e->error().norm();
		}
		
		cout<<"in kf " << pKFi->mnId <<" res = " << res <<endl;

		
	}
	
	
	
	return res;
	
}
