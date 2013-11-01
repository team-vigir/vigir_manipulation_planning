#include <flor_visualization_utils/marker_utils.h>

#include <ros/ros.h>


namespace flor_visualization_utils {

  void drawEllipsoid(const geometry_msgs::PoseStamped& pose,
                     const Eigen::MatrixXcd &eigen_values,
                     const Eigen::MatrixXcd &eigen_vectors,
                     visualization_msgs::MarkerArray& markerArray)
  {

    visualization_msgs::Marker tempMarker;
    tempMarker.type = visualization_msgs::Marker::SPHERE;
    tempMarker.header = pose.header;

    tempMarker.color.r = 1.0;
    tempMarker.color.a = 0.5;

    if ((eigen_vectors.rows() != 3) || (eigen_vectors.cols() != 3)){
      ROS_WARN("Can´t draw ellipsoid, Matrix of Eigenvectors has %d rows and %d columns", (int)eigen_vectors.rows(), (int)eigen_vectors.cols());
    }


     tempMarker.pose.position.x = pose.pose.position.x;
     tempMarker.pose.position.y = pose.pose.position.y;
     tempMarker.pose.position.z = pose.pose.position.z;


    /*
     Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eig(covMatrix);

     const Eigen::Vector3f& eigValues (eig.eigenvalues());
     const Eigen::Matrix3f& eigVectors (eig.eigenvectors());
     */

    Eigen::Matrix3d eigVectorsReal(eigen_vectors.real());

    Eigen::Matrix3d eigVectorsFlipped;
    eigVectorsFlipped.col(0) = eigVectorsReal.col(2);
    eigVectorsFlipped.col(1) = eigVectorsReal.col(1);
    eigVectorsFlipped.col(2) = eigVectorsReal.col(0);

    if (eigVectorsFlipped.determinant() < 0){
      eigVectorsFlipped.col(2) = -eigVectorsFlipped.col(2);
    }

    Eigen::Quaterniond quaternion (eigVectorsFlipped);

    //std::cout << "\neigVec:\n" << eigVectors << "\n";
    //std::cout << "\neigVecFlipped:\n" << eigVectorsFlipped << "\n";

    //std::cout << "\now:" << quaternion.w() << " x:" << quaternion.x() << " y:" << quaternion.y() << " z:" << quaternion.z() << "\n";


    tempMarker.pose.orientation.w = quaternion.w();
    tempMarker.pose.orientation.x = quaternion.x();
    tempMarker.pose.orientation.y = quaternion.y();
    tempMarker.pose.orientation.z = quaternion.z();

    tempMarker.scale.x = sqrt(eigen_values(2).real());
    tempMarker.scale.y = sqrt(eigen_values(1).real());
    tempMarker.scale.z = sqrt(eigen_values(0).real());

    tempMarker.id = markerArray.markers.size();
    markerArray.markers.push_back(tempMarker);
  }

  void drawPoses(const std::vector<geometry_msgs::Pose> poses,
                     visualization_msgs::MarkerArray& markerArray,
                     std::string frame_id,
                     ros::Time stamp)
  {
    size_t size = poses.size();

    if (size == 0){
      return;
    }

    visualization_msgs::Marker tempMarker;
    tempMarker.type = visualization_msgs::Marker::LINE_LIST;
    tempMarker.header.frame_id = frame_id;
    tempMarker.header.stamp = stamp;

    tempMarker.color.r = 0.0;
    tempMarker.color.a = 0.15;

    tempMarker.scale.x = 0.01;
    tempMarker.scale.y = 0.1;
    tempMarker.scale.z = 0.1;

    std_msgs::ColorRGBA col;
    col.a=0.25;
    col.r=0.0;
    col.g=0.0;
    col.b=0.0;

    geometry_msgs::Point zero_point;

    for (size_t i = 0; i < size; ++i){

      const geometry_msgs::Pose& pose = poses[i];

      tempMarker.pose.position.x = pose.position.x;
      tempMarker.pose.position.y = pose.position.y;
      tempMarker.pose.position.z = pose.position.z;

      tempMarker.pose.orientation.w = pose.orientation.w;
      tempMarker.pose.orientation.x = pose.orientation.x;
      tempMarker.pose.orientation.y = pose.orientation.y;
      tempMarker.pose.orientation.z = pose.orientation.z;

      tempMarker.points.push_back(zero_point);
      tempMarker.points.push_back(zero_point);
      tempMarker.points[0].x = 0.05;

      tempMarker.points.push_back(zero_point);
      tempMarker.points.push_back(zero_point);
      tempMarker.points[2].y = 0.05;

      tempMarker.points.push_back(zero_point);
      tempMarker.points.push_back(zero_point);
      tempMarker.points[4].z = 0.05;

      tempMarker.colors.push_back(col);
      tempMarker.colors.push_back(col);
      tempMarker.colors[0].r=1.0;
      tempMarker.colors[1].r=1.0;
      tempMarker.colors.push_back(col);
      tempMarker.colors.push_back(col);
      tempMarker.colors[2].g=1.0;
      tempMarker.colors[3].g=1.0;
      tempMarker.colors.push_back(col);
      tempMarker.colors.push_back(col);
      tempMarker.colors[4].b=1.0;
      tempMarker.colors[5].b=1.0;



      tempMarker.id = markerArray.markers.size();

      markerArray.markers.push_back(tempMarker);

      tempMarker.points.clear();
      tempMarker.colors.clear();

    }
  }



}

