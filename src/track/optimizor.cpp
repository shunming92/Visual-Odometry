#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

#include "track/optimizor.h"

using namespace std;

namespace StereoVO
{
    Optimizor::Optimizor()
    {
    }
    
    // code below is obtained from book - 视觉SLAM十四讲(Gao Xiang), theory behind these code can also be found in that book
    // The theory is similar to the pos optimization in ORB-SLAM2.
    void Optimizor::poseOptimze(const vector<cv::Point3f> & point_3d, const vector<cv::Point2f>& point_2d, const cv::Mat& k, cv::Mat& R, cv::Mat& t)
    {
        typedef g2o::BlockSolver< g2o::BlockSolverTraits<6,3> > Block;  
        Block::LinearSolverType* linearSolver = new g2o::LinearSolverCSparse<Block::PoseMatrixType>(); 
        Block* solver_ptr = new Block ( linearSolver );   
        g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg ( solver_ptr );
        g2o::SparseOptimizer optimizer;
        optimizer.setAlgorithm ( solver );
        
        // vertex
        g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap(); // camera pose
        Eigen::Matrix3d R_mat;
        R_mat <<
        R.at<double> ( 0,0 ), R.at<double> ( 0,1 ), R.at<double> ( 0,2 ),
        R.at<double> ( 1,0 ), R.at<double> ( 1,1 ), R.at<double> ( 1,2 ),
        R.at<double> ( 2,0 ), R.at<double> ( 2,1 ), R.at<double> ( 2,2 );
        pose->setId ( 0 );
//         std::cout << "R: "<<std::endl<<R<<std::endl;
//         std::cout << "R_mat: "<<std::endl<<R_mat<<std::endl;
        // rotation first then translation - construct function
        pose->setEstimate ( g2o::SE3Quat (
            R_mat,
            Eigen::Vector3d ( t.at<double> ( 0,0 ), t.at<double> ( 1,0 ), t.at<double> ( 2,0 ) )
        ) );
        optimizer.addVertex ( pose );
        
        int index = 1;
        for ( const cv::Point3f p:point_3d ) 
        {
            g2o::VertexSBAPointXYZ* point = new g2o::VertexSBAPointXYZ();
            point->setId ( index++ );
            point->setEstimate ( Eigen::Vector3d ( p.x, p.y, p.z ) );
            point->setMarginalized ( true ); 
            optimizer.addVertex ( point );
        }
        
        // parameter: camera intrinsics
        g2o::CameraParameters* camera = new g2o::CameraParameters (
            k.at<double> ( 0,0 ), Eigen::Vector2d ( k.at<double> ( 0,2 ), k.at<double> ( 1,2 ) ), 0
        );
        camera->setId ( 0 );
        optimizer.addParameter ( camera );
        
        // edges
        index = 1;
        for ( const cv::Point2f p:point_2d )
        {
            g2o::EdgeProjectXYZ2UV* edge = new g2o::EdgeProjectXYZ2UV();
            edge->setId ( index );
            edge->setVertex ( 0, dynamic_cast<g2o::VertexSBAPointXYZ*> ( optimizer.vertex ( index ) ) );
            edge->setVertex ( 1, pose );
            edge->setMeasurement ( Eigen::Vector2d ( p.x, p.y ) );
            edge->setParameterId ( 0,0 );
            edge->setInformation ( Eigen::Matrix2d::Identity() );
            optimizer.addEdge ( edge );
            index++;
        }
        
        optimizer.setVerbose ( true );
        optimizer.initializeOptimization();
        optimizer.optimize ( 100 );
        
//         cout<<endl<<"after optimization:"<<endl;
//         cout<<"T="<<endl<<Eigen::Isometry3d ( pose->estimate() ).matrix() <<endl;
    }
}
