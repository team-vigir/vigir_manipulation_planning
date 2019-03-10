/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Jon Binney, Ioan Sucan */

#ifndef VIGIR_LIDAR_OCTOMAP_UPDATER_
#define VIGIR_LIDAR_OCTOMAP_UPDATER_

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <moveit/occupancy_map_monitor/occupancy_map_updater.h>
#include <moveit/point_containment_filter/shape_mask.h>
#include <laser_geometry/laser_geometry.h>
#include <filters/filter_chain.h>
#include <vigir_perception_msgs/FilteredLocalizedLaserScan.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>

#include <hector_nav_msgs/GetDistanceToObstacle.h>

#include <dynamic_reconfigure/server.h>
#include <vigir_lidar_octomap_updater/LidarOctomapUpdaterConfig.h>


namespace occupancy_map_monitor
{

class LidarOctomapUpdater : public OccupancyMapUpdater
{
public:

  LidarOctomapUpdater();
  virtual ~LidarOctomapUpdater();

  virtual bool setParams(XmlRpc::XmlRpcValue &params);

  virtual bool initialize();
  virtual void start();
  virtual void stop();
  virtual ShapeHandle excludeShape(const shapes::ShapeConstPtr &shape);
  virtual void forgetShape(ShapeHandle handle);

  void LidarQueueThread();

  void dynRecParamCallback(vigir_lidar_octomap_updater::LidarOctomapUpdaterConfig &config, uint32_t level);

protected:

  virtual void updateMask(const sensor_msgs::PointCloud2 &cloud, const Eigen::Vector3d &sensor_origin, std::vector<int> &mask);

private:

  bool getShapeTransform(ShapeHandle h, Eigen::Isometry3d &transform) const;
  void laserMsgCallback(const sensor_msgs::LaserScan::ConstPtr &cloud_msg);
  void cloudMsgCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg);
  void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped pose);
  bool clearOctomap(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
  bool clearRobotVicinityOctomap(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
  bool disableOctomapUpdates(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
  void stopHelper();

  void resetOctomap(bool load_prior = true);


  void serviceThread();
  bool lookupServiceCallback(hector_nav_msgs::GetDistanceToObstacle::Request  &req,
                             hector_nav_msgs::GetDistanceToObstacle::Response &res );

  void cast_ray_mod_direction(
                                              const octomap::point3d& origin,
                                              const octomap::OcTree& octree,
                                              const tf::Vector3& direction,
                                              double pitch,
                                              double yaw,
                                              octomap::point3d& end_point);

  void get_endpoints(const octomap::point3d& origin,
                     const octomap::OcTree& octree,
                     const float reference_distance,
                     const tf::Vector3& direction,
                     std::vector<octomap::point3d>& directions,
                     std::vector<octomap::point3d>& endPoints,
                     int n);

  boost::shared_ptr< dynamic_reconfigure::Server<vigir_lidar_octomap_updater::LidarOctomapUpdaterConfig> >dyn_rec_server_;



  ros::NodeHandle root_nh_;
  ros::NodeHandle private_nh_;
  boost::shared_ptr<tf::Transformer> tf_;

  /* params */
  std::string laser_scan_topic_;
  std::string point_cloud_topic_;
  double scale_;
  double padding_;
  double max_range_;
  unsigned int point_subsample_;
  std::string filtered_cloud_topic_;
  ros::Publisher filtered_cloud_publisher_;

  std::string p_prior_map_file_;

  message_filters::Subscriber<sensor_msgs::LaserScan> *laser_scan_subscriber_;
  tf::MessageFilter<sensor_msgs::LaserScan> *laser_scan_filter_;

  message_filters::Subscriber<sensor_msgs::PointCloud2> *point_cloud_subscriber_;
  tf::MessageFilter<sensor_msgs::PointCloud2> *point_cloud_filter_;

  /* used to store all cells in the map which a given ray passes through during raycasting.
     we cache this here because it dynamically pre-allocates a lot of memory in its constructor */
  octomap::KeyRay key_ray_;

  boost::scoped_ptr<point_containment_filter::ShapeMask> shape_mask_;
  std::vector<int> mask_;

  ros::Duration wait_duration_;
  boost::shared_ptr<sensor_msgs::PointCloud2> cloud_msg;
  laser_geometry::LaserProjection projector_;
  filters::FilterChain<sensor_msgs::LaserScan> filter_chain_;
  sensor_msgs::LaserScan scan_filtered_;
  sensor_msgs::LaserScan scan_self_filtered_;

  vigir_perception_msgs::FilteredLocalizedLaserScan filtered_localized_scan_;

  ros::Publisher scan_filtered_publisher_;
  ros::Publisher scan_self_filtered_publisher_;
  ros::Publisher filtered_localized_scan_publisher_;

  ros::Subscriber initial_pose_sub_;
  ros::ServiceServer clear_service_;
  ros::ServiceServer clear_robot_vicinity_service_;
  ros::ServiceServer disable_octomap_updates_service_;

  ros::CallbackQueue lidar_queue_;
  boost::thread lidar_callback_queue_thread_;

  bool disable_octomap_updates_;




  ros::ServiceServer dist_lookup_srv_server_;

  ros::Publisher m_markerPub;

  ros::CallbackQueue service_queue_;
  boost::thread service_thread_;

  //boost::shared_ptr<tf::Transformer> tf_;

  double octo_min_distance_;
  double octo_max_distance_;
  double secondary_rays_max_dist_;
  double secondary_rays_opening_angle_;

  std::string target_frame_;
protected:
  boost::mutex shape_lock_;

  //octomap::KeySet free_cells, occupied_cells, model_cells, clip_cells;

};

}

#endif
