//=================================================================================================
// Copyright (c) 2014, Stefan Kohlbrecher, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

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

/* Author: Jon Binney, Ioan Sucan, Stefan Kohlbrecher */

#include <cmath>
#include <vigir_lidar_octomap_updater/lidar_octomap_updater.h>
#include <moveit/occupancy_map_monitor/occupancy_map_monitor.h>
#include <message_filters/subscriber.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <XmlRpcException.h>
#include <std_srvs/SetBool.h>

#include <tf/transform_datatypes.h>

#include <visualization_msgs/MarkerArray.h>

#include <octomap/octomap.h>
#include <octomap_ros/conversions.h>

namespace occupancy_map_monitor
{

LidarOctomapUpdater::LidarOctomapUpdater() : OccupancyMapUpdater("LidarCloudUpdater"),
                                                       private_nh_("~"),
                                                       scale_(1.0),
                                                       padding_(0.0),
                                                       max_range_(std::numeric_limits<double>::infinity()),
                                                       point_subsample_(1),
                                                       laser_scan_subscriber_(NULL),
                                                       laser_scan_filter_(NULL),
                                                       point_cloud_subscriber_(NULL),
                                                       filter_chain_("sensor_msgs::LaserScan"),
                                                       disable_octomap_updates_(false)
{
}

LidarOctomapUpdater::~LidarOctomapUpdater()
{
  stopHelper();
}

bool LidarOctomapUpdater::setParams(XmlRpc::XmlRpcValue &params)
{
  try
  {
    //if (!params.hasMember("scan_topic"))
    //  return false;
    laser_scan_topic_ = static_cast<const std::string&>(params["scan_topic"]);

    point_cloud_topic_ = static_cast<const std::string&>(params["cloud_topic"]);

    readXmlParam(params, "max_range", &max_range_);
    readXmlParam(params, "padding_offset", &padding_);
    readXmlParam(params, "padding_scale", &scale_);
    readXmlParam(params, "point_subsample", &point_subsample_);
    if (params.hasMember("filtered_cloud_topic"))
      filtered_cloud_topic_ = static_cast<const std::string&>(params["filtered_cloud_topic"]);
  }
  catch (XmlRpc::XmlRpcException &ex)
  {
    ROS_ERROR("XmlRpc Exception: %s", ex.getMessage().c_str());
    return false;
  }

  return true;
}

bool LidarOctomapUpdater::initialize()
{
  wait_duration_ = ros::Duration(0.5);

  filtered_localized_scan_.processed_scan.ranges.resize(3);
  filtered_localized_scan_.processed_scan.intensities.resize(1);

  if (!filter_chain_.configure("scan_filter_chain", private_nh_))
    ROS_ERROR("Error while configuring filter chain, proceeding without filtering!");

  cloud_msg.reset(new sensor_msgs::PointCloud2());

  tf_ = monitor_->getTFClient();
  shape_mask_.reset(new point_containment_filter::ShapeMask());
  shape_mask_->setTransformCallback(boost::bind(&LidarOctomapUpdater::getShapeTransform, this, _1, _2));
  if (!filtered_cloud_topic_.empty())
    filtered_cloud_publisher_ = private_nh_.advertise<sensor_msgs::PointCloud2>(filtered_cloud_topic_, 10, false);

  scan_filtered_publisher_ = private_nh_.advertise<sensor_msgs::LaserScan>("scan_filtered", 10, false);
  scan_self_filtered_publisher_ = private_nh_.advertise<sensor_msgs::LaserScan>("scan_self_filtered", 10, false);
  filtered_localized_scan_publisher_ = private_nh_.advertise<vigir_perception_msgs::FilteredLocalizedLaserScan>("scan_filtered_localized", 10, false);


  private_nh_.param("prior_octomap_to_load", p_prior_map_file_, std::string(""));

  if (!p_prior_map_file_.empty()){
    ROS_INFO("Using prior octomap file: %s", p_prior_map_file_.c_str());
    tree_->lockWrite();
    tree_->readBinary(p_prior_map_file_);
    tree_->unlockWrite();
  }else{
    ROS_INFO("No prior octomap to use given, not loading any.");
  }

  initial_pose_sub_ = private_nh_.subscribe("/initialpose", 1, &LidarOctomapUpdater::initialPoseCallback, this);
  clear_service_ = private_nh_.advertiseService("/clear_octomap", &LidarOctomapUpdater::clearOctomap, this);
  clear_robot_vicinity_service_ = private_nh_.advertiseService("/clear_robot_vicinity_octomap", &LidarOctomapUpdater::clearRobotVicinityOctomap, this);

  disable_octomap_updates_service_ = private_nh_.advertiseService("/disable_octomap_updates", &LidarOctomapUpdater::disableOctomapUpdates, this);


  this->lidar_callback_queue_thread_ =
      boost::thread(boost::bind(&LidarOctomapUpdater::LidarQueueThread, this));


  ros::AdvertiseServiceOptions ops=ros::AdvertiseServiceOptions::create<hector_nav_msgs::GetDistanceToObstacle>("get_distance_to_obstacle", boost::bind(&LidarOctomapUpdater::lookupServiceCallback, this,_1,_2),ros::VoidConstPtr(),&service_queue_);
  dist_lookup_srv_server_ = private_nh_.advertiseService(ops);

  service_thread_=boost::thread(boost::bind(&LidarOctomapUpdater::serviceThread,this));

  m_markerPub = private_nh_.advertise<visualization_msgs::MarkerArray>("distance_to_obstacle_debug_markers", 1, false);


  //tf_ = this->context_->planning_scene_monitor_->getTFClient();

  return true;
}

void LidarOctomapUpdater::start()
{
  //if (laser_scan_subscriber_)
  //  return;
  /* subscribe to point cloud topic using tf filter*/
  laser_scan_subscriber_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(root_nh_, laser_scan_topic_, 40, ros::TransportHints(), &lidar_queue_);
  if (tf_ && !monitor_->getMapFrame().empty())
  {
    laser_scan_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(*laser_scan_subscriber_, *tf_, monitor_->getMapFrame(), 40);
    laser_scan_filter_->registerCallback(boost::bind(&LidarOctomapUpdater::laserMsgCallback, this, _1));
    ROS_INFO("Listening to '%s' using message filter with target frame '%s'", laser_scan_topic_.c_str(), laser_scan_filter_->getTargetFramesString().c_str());
  }
  else
  {
    laser_scan_subscriber_->registerCallback(boost::bind(&LidarOctomapUpdater::laserMsgCallback, this, _1));
    ROS_INFO("Listening to laser topic'%s'", laser_scan_topic_.c_str());
  }

  //if (point_cloud_subscriber_)
  //  return;
  /* subscribe to point cloud topic using tf filter*/
  point_cloud_subscriber_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>(root_nh_, point_cloud_topic_, 40, ros::TransportHints(), &lidar_queue_);
  if (tf_ && !monitor_->getMapFrame().empty())
  {
    point_cloud_filter_ = new tf::MessageFilter<sensor_msgs::PointCloud2>(*point_cloud_subscriber_, *tf_, monitor_->getMapFrame(), 40);
    point_cloud_filter_->registerCallback(boost::bind(&LidarOctomapUpdater::cloudMsgCallback, this, _1));
    ROS_INFO("Listening to '%s' using message filter with target frame '%s'", point_cloud_topic_.c_str(), point_cloud_filter_->getTargetFramesString().c_str());
  }
  else
  {
    point_cloud_subscriber_->registerCallback(boost::bind(&LidarOctomapUpdater::cloudMsgCallback, this, _1));
    ROS_INFO("Listening to point cloud topic '%s'", point_cloud_topic_.c_str());
  }
}

void LidarOctomapUpdater::LidarQueueThread() {
    static const double timeout = 0.05;

    while (ros::ok()) {
      lidar_queue_.callAvailable(ros::WallDuration(timeout));
    }
  }

void LidarOctomapUpdater::stopHelper()
{
  delete laser_scan_filter_;
  delete laser_scan_subscriber_;
}

void LidarOctomapUpdater::stop()
{
  stopHelper();
  laser_scan_filter_ = NULL;
  laser_scan_subscriber_ = NULL;
}

ShapeHandle LidarOctomapUpdater::excludeShape(const shapes::ShapeConstPtr &shape)
{
  boost::mutex::scoped_lock scoped_lock(shape_lock_);

  ShapeHandle h = 0;
  if (shape_mask_)
    h = shape_mask_->addShape(shape, scale_, padding_);
  else
    ROS_ERROR("Shape filter not yet initialized!");
  return h;
}

void LidarOctomapUpdater::forgetShape(ShapeHandle handle)
{
  boost::mutex::scoped_lock scoped_lock(shape_lock_);

  if (shape_mask_)
    shape_mask_->removeShape(handle);
}

bool LidarOctomapUpdater::getShapeTransform(ShapeHandle h, Eigen::Affine3d &transform) const
{
  ShapeTransformCache::const_iterator it = transform_cache_.find(h);
  if (it == transform_cache_.end())
  {
    ROS_ERROR("Internal error. Shape filter handle %u not found", h);
    return false;
  }
  transform = it->second;
  return true;
}

void LidarOctomapUpdater::updateMask(const sensor_msgs::PointCloud2 &cloud, const Eigen::Vector3d &sensor_origin, std::vector<int> &mask)
{
}

void LidarOctomapUpdater::laserMsgCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
{
  if (!filter_chain_.update(*scan_msg, scan_filtered_)){
    ROS_ERROR("Error in filter chain update, aborting self filtering/octomap update!");
    return;
  }

  ROS_DEBUG("Received a new point cloud message");
  ros::WallTime start = ros::WallTime::now();
  ros::WallTime self_filter_finished_time;

  if (monitor_->getMapFrame().empty())
    monitor_->setMapFrame(scan_msg->header.frame_id);

  /* get transform for cloud into map frame */
  tf::StampedTransform map_H_sensor;
  tf::StampedTransform map_H_sensor_last_ray;
  if (monitor_->getMapFrame() == scan_filtered_.header.frame_id)
    map_H_sensor.setIdentity();
  else
  {
    if (tf_)
    {
      try
      {
        ros::Time end_time   = scan_filtered_.header.stamp + ros::Duration().fromSec((scan_filtered_.ranges.size() -1)*scan_filtered_.time_increment) ;

        if(tf_->waitForTransform(monitor_->getMapFrame(), scan_filtered_.header.frame_id, scan_filtered_.header.stamp, wait_duration_) &&
           tf_->waitForTransform(monitor_->getMapFrame(), scan_filtered_.header.frame_id, end_time, wait_duration_)){
          tf_->lookupTransform(monitor_->getMapFrame(), scan_filtered_.header.frame_id, scan_filtered_.header.stamp, map_H_sensor);
          tf_->lookupTransform(monitor_->getMapFrame(), scan_filtered_.header.frame_id, end_time, map_H_sensor_last_ray);
        }
      }
      catch (tf::TransformException& ex)
      {
        ROS_ERROR_STREAM("Transform error of sensor data: " << ex.what() << "; quitting callback in LidarOctomapUpdater");
        return;
      }
      catch(...)
      {
        ROS_ERROR_STREAM("Exception while retrieving scan transform in LidarOctomapUpdater");
        return;
      }
    }
    else
    {
      ROS_ERROR("No tf listener, cannot run LidarOctomapUpdater");
      return;
    }
  }

  /* compute sensor origin in map frame */
  const tf::Vector3 &sensor_origin_tf = map_H_sensor.getOrigin();
  octomap::point3d sensor_origin(sensor_origin_tf.getX(), sensor_origin_tf.getY(), sensor_origin_tf.getZ());
  Eigen::Vector3d sensor_origin_eigen(sensor_origin_tf.getX(), sensor_origin_tf.getY(), sensor_origin_tf.getZ());

  try
  {
    projector_.transformLaserScanToPointCloud(monitor_->getMapFrame(), scan_filtered_, *cloud_msg, *tf_, max_range_, laser_geometry::channel_option::Intensity|laser_geometry::channel_option::Index);
  }
  catch(...)
  {
    ROS_ERROR_STREAM("Exception while transforming scan; quitting callback in LidarOctomapUpdater");
    return;
  }

  if (!updateTransformCache(cloud_msg->header.frame_id, cloud_msg->header.stamp))
  {
    ROS_ERROR_THROTTLE(1, "Transform cache was not updated. Self-filtering may fail.");
    return;
  }

  if (cloud_msg->height > 1){
    ROS_ERROR_THROTTLE(1,"Cloud height > 1 not supported by lidar octomap updater!");
    return;
  }

  uint32_t index_offset = -1;
  bool has_intensity = false;

  //Check if field "index" exists, exit otherwise
  for(unsigned int i = 0; i < cloud_msg->fields.size(); ++i)
  {
    if(cloud_msg->fields[i].name == "index")
    {
      index_offset = cloud_msg->fields[i].offset;
    }

    else if(cloud_msg->fields[i].name == "intensity")
    {
      has_intensity = true;
    }
    //ROS_INFO("Field name: %s ", cloud_msg->fields[i].name.c_str());
  }

  //ROS_INFO("Index offset: %d", index_offset);

  if (index_offset < 0){
    ROS_ERROR_THROTTLE(1, "Index offset for cloud < 0, aborting.");
    return;
  }


  /* mask out points on the robot */
  
  //We are in world frame, so using the scan max range cuts off things in a circle around world origin.
  //@TODO: Do this in a nicer way.
  double max_mask_range = 20000.0;

  {
    boost::mutex::scoped_lock scoped_lock(shape_lock_);
    shape_mask_->maskContainment(*cloud_msg, sensor_origin_eigen, 0.0, max_mask_range, mask_);
    updateMask(*cloud_msg, sensor_origin_eigen, mask_);
  }

  self_filter_finished_time = ros::WallTime::now();

  octomap::KeySet free_cells, occupied_cells, model_cells, clip_cells;
  //free_cells.clear();
  //occupied_cells.clear();
  //model_cells.clear();
  //clip_cells.clear();
  //std::cout << free_cells.bucket_count() <<  "\n";
  boost::scoped_ptr<sensor_msgs::PointCloud2> filtered_cloud;

  //We only use these iterators if we are creating a filtered_cloud for
  //publishing. We cannot default construct these, so we use scoped_ptr's
  //to defer construction
  boost::scoped_ptr<sensor_msgs::PointCloud2Iterator<float> > iter_filtered_x;
  boost::scoped_ptr<sensor_msgs::PointCloud2Iterator<float> > iter_filtered_y;
  boost::scoped_ptr<sensor_msgs::PointCloud2Iterator<float> > iter_filtered_z;
  boost::scoped_ptr<sensor_msgs::PointCloud2Iterator<float> > iter_filtered_i;

  bool publish_filtered_cloud = (!filtered_cloud_topic_.empty()) && (filtered_cloud_publisher_.getNumSubscribers() > 0);

  if (publish_filtered_cloud) {
    filtered_cloud.reset(new sensor_msgs::PointCloud2());
    filtered_cloud->header = cloud_msg->header;
    sensor_msgs::PointCloud2Modifier pcd_modifier(*filtered_cloud);
    //pcd_modifier.setPointCloud2FieldsByString(2, "xyz", "i");
    pcd_modifier.setPointCloud2Fields(4,
      "x", 1, sensor_msgs::PointField::FLOAT32,
      "y", 1, sensor_msgs::PointField::FLOAT32,
      "z", 1, sensor_msgs::PointField::FLOAT32,
      "intensity", 1, sensor_msgs::PointField::FLOAT32);
    pcd_modifier.resize(cloud_msg->width * cloud_msg->height);

    //we have created a filtered_out, so we can create the iterators now
    iter_filtered_x.reset(new sensor_msgs::PointCloud2Iterator<float>(*filtered_cloud, "x"));
    iter_filtered_y.reset(new sensor_msgs::PointCloud2Iterator<float>(*filtered_cloud, "y"));
    iter_filtered_z.reset(new sensor_msgs::PointCloud2Iterator<float>(*filtered_cloud, "z"));
    iter_filtered_i.reset(new sensor_msgs::PointCloud2Iterator<float>(*filtered_cloud, "intensity"));
  }

  size_t filtered_cloud_size = 0;

  tree_->lockRead();

  bool octomap_updates_disabled_for_this_cloud = disable_octomap_updates_;

  //Make copy of filtered scan, modify below with self filter information
  scan_self_filtered_ = scan_filtered_;

  try
  {
    /* do ray tracing to find which cells this point cloud indicates should be free, and which it indicates
     * should be occupied */
    //for (
    unsigned int row = 0;//; row < cloud_msg->height; row += point_subsample_)
    {
      //unsigned int row_c = row * cloud_msg->width;
      sensor_msgs::PointCloud2ConstIterator<float> pt_iter(*cloud_msg, "x");
      sensor_msgs::PointCloud2ConstIterator<unsigned int> index_iter(*cloud_msg, "index");
      //set iterator to point at start of the current row
      //pt_iter += row_c;

      for (unsigned int col = 0; col < cloud_msg->width; col += point_subsample_,
        pt_iter += point_subsample_, index_iter += point_subsample_)
      {
        //if (mask_[row_c + col] == point_containment_filter::ShapeMask::CLIP)
        //  continue;

        if (has_intensity && std::isnan(pt_iter[3]))
          continue;

        /* check for NaN */
        if (!std::isnan(pt_iter[0]) && !std::isnan(pt_iter[1]) && !std::isnan(pt_iter[2]))
        {
          /* transform to map frame */
          //tf::Vector3 point_tf = map_H_sensor * tf::Vector3(pt_iter[0], pt_iter[1],
          //  pt_iter[2]);
          tf::Vector3 point_tf (pt_iter[0], pt_iter[1], pt_iter[2]);

          /* occupied cell at ray endpoint if ray is shorter than max range and this point
             isn't on a part of the robot*/
          if (mask_[col] == point_containment_filter::ShapeMask::INSIDE){
            model_cells.insert(tree_->coordToKey(point_tf.getX(), point_tf.getY(), point_tf.getZ()));
            scan_self_filtered_.ranges[index_iter[0]] = std::numeric_limits<float>::quiet_NaN();
          }
          else if (mask_[col] == point_containment_filter::ShapeMask::CLIP)
          {
            // CLIP cells are only those outside min to max range. Given we operate in world frame for
            // containment check and scan is prefiltered for min/max, there are no CLIP cells.
            //clip_cells.insert(tree_->coordToKey(point_tf.getX(), point_tf.getY(), point_tf.getZ()));
          }
          else
          {
            occupied_cells.insert(tree_->coordToKey(point_tf.getX(), point_tf.getY(), point_tf.getZ()));
            //build list of valid points if we want to publish them
            if (publish_filtered_cloud)
            {
              **iter_filtered_x = pt_iter[0];
              **iter_filtered_y = pt_iter[1];
              **iter_filtered_z = pt_iter[2];
              **iter_filtered_i = has_intensity ? pt_iter[3] : 1.0f;
              ++filtered_cloud_size;
              ++*iter_filtered_x;
              ++*iter_filtered_y;
              ++*iter_filtered_z;
              ++*iter_filtered_i;
            }
          }
        }
      }
    }

    if (!octomap_updates_disabled_for_this_cloud){
        /* compute the free cells along each ray that ends at an occupied cell */
        for (octomap::KeySet::iterator it = occupied_cells.begin(), end = occupied_cells.end(); it != end; ++it)
          if (tree_->computeRayKeys(sensor_origin, tree_->keyToCoord(*it), key_ray_))
            free_cells.insert(key_ray_.begin(), key_ray_.end());

        /* compute the free cells along each ray that ends at a model cell */
        for (octomap::KeySet::iterator it = model_cells.begin(), end = model_cells.end(); it != end; ++it)
          if (tree_->computeRayKeys(sensor_origin, tree_->keyToCoord(*it), key_ray_))
            free_cells.insert(key_ray_.begin(), key_ray_.end());

        /* compute the free cells along each ray that ends at a clipped cell */
        for (octomap::KeySet::iterator it = clip_cells.begin(), end = clip_cells.end(); it != end; ++it)
          if (tree_->computeRayKeys(sensor_origin, tree_->keyToCoord(*it), key_ray_))
            free_cells.insert(key_ray_.begin(), key_ray_.end());
    }

    //std::cout << free_cells.bucket_count() <<  "\n";
  }
  catch (...)
  {
    tree_->unlockRead();
    return;
  }

  tree_->unlockRead();


  // ----------------- Finished with self filtering, can publish filtered data now


  if (filtered_localized_scan_publisher_.getNumSubscribers() > 0){

    filtered_localized_scan_.header.stamp = scan_msg->header.stamp;
    filtered_localized_scan_.header.frame_id = monitor_->getMapFrame();

    filtered_localized_scan_.processed_scan.header = scan_msg->header;

    filtered_localized_scan_.processed_scan.angle_min = scan_msg->angle_min;
    filtered_localized_scan_.processed_scan.angle_max = scan_msg->angle_max;
    filtered_localized_scan_.processed_scan.angle_increment = scan_msg->angle_increment;
    filtered_localized_scan_.processed_scan.time_increment = scan_msg->time_increment;
    filtered_localized_scan_.processed_scan.scan_time = scan_msg->scan_time;
    filtered_localized_scan_.processed_scan.range_min = scan_msg->range_min;
    filtered_localized_scan_.processed_scan.range_max = scan_msg->range_max;

    filtered_localized_scan_.processed_scan.ranges[vigir_perception_msgs::FilteredLocalizedLaserScan::SCAN_PREPROCESSED].echoes = scan_filtered_.ranges;
    filtered_localized_scan_.processed_scan.ranges[vigir_perception_msgs::FilteredLocalizedLaserScan::SCAN_SELF_FILTERED].echoes = scan_self_filtered_.ranges;
    filtered_localized_scan_.processed_scan.ranges[vigir_perception_msgs::FilteredLocalizedLaserScan::SCAN_RAW].echoes = scan_msg->ranges;
    filtered_localized_scan_.processed_scan.intensities[0].echoes = scan_msg->intensities;

    tf::transformTFToMsg(map_H_sensor, filtered_localized_scan_.transform_first_ray);
    tf::transformTFToMsg(map_H_sensor_last_ray, filtered_localized_scan_.transform_last_ray);

    filtered_localized_scan_publisher_.publish(filtered_localized_scan_);
  }

  if (scan_filtered_publisher_.getNumSubscribers() > 0){
    scan_filtered_publisher_.publish(scan_filtered_);
  }

  if (scan_self_filtered_publisher_.getNumSubscribers() > 0){
    scan_self_filtered_publisher_.publish(scan_self_filtered_);
  }

  if (publish_filtered_cloud)
  {
    sensor_msgs::PointCloud2Modifier pcd_modifier(*filtered_cloud);
    pcd_modifier.resize(filtered_cloud_size);
    filtered_cloud_publisher_.publish(*filtered_cloud);
  }



  // ----------------- Update octomap --------------------

  if (!octomap_updates_disabled_for_this_cloud){

      /* cells that overlap with the model are not occupied */
      for (octomap::KeySet::iterator it = model_cells.begin(), end = model_cells.end(); it != end; ++it)
        occupied_cells.erase(*it);

      /* occupied cells are not free */
      for (octomap::KeySet::iterator it = occupied_cells.begin(), end = occupied_cells.end(); it != end; ++it)
        free_cells.erase(*it);

      tree_->lockWrite();

      try
      {
        /* mark free cells only if not seen occupied in this cloud */
        for (octomap::KeySet::iterator it = free_cells.begin(), end = free_cells.end(); it != end; ++it)
          tree_->updateNode(*it, false);

        /* now mark all occupied cells */
        for (octomap::KeySet::iterator it = occupied_cells.begin(), end = occupied_cells.end(); it != end; ++it)
          tree_->updateNode(*it, true);

        // set the logodds to the minimum for the cells that are part of the model
        const float lg = tree_->getClampingThresMinLog() - tree_->getClampingThresMaxLog();
        for (octomap::KeySet::iterator it = model_cells.begin(), end = model_cells.end(); it != end; ++it)
          tree_->updateNode(*it, lg);
      }
      catch (...)
      {
        ROS_ERROR("Internal error while updating octree");
      }
      tree_->unlockWrite();
      tree_->triggerUpdateCallback();
  }

  ROS_DEBUG("Processed laser scan in %lf ms. Self filtering took %lf ms", (ros::WallTime::now() - start).toSec() * 1000.0, (self_filter_finished_time - start).toSec() * 1000.0 );
}


void LidarOctomapUpdater::cloudMsgCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg_in)
{

  ROS_DEBUG("Received a new point cloud message");
  ros::WallTime start = ros::WallTime::now();
  ros::WallTime self_filter_finished_time;

  if (monitor_->getMapFrame().empty())
    monitor_->setMapFrame(cloud_msg_in->header.frame_id);

  /* get transform for cloud into map frame */
  tf::StampedTransform map_H_sensor;
  if (monitor_->getMapFrame() == cloud_msg_in->header.frame_id)
    map_H_sensor.setIdentity();
  else
  {
    if (tf_)
    {
      try
      {
        tf_->lookupTransform(monitor_->getMapFrame(), cloud_msg_in->header.frame_id, cloud_msg_in->header.stamp,
                             map_H_sensor);
      }
      catch (tf::TransformException& ex)
      {
        ROS_ERROR_STREAM("Transform error of sensor data: " << ex.what() << "; quitting callback");
        return;
      }
    }
    else
      return;
  }

  /* compute sensor origin in map frame */
  const tf::Vector3& sensor_origin_tf = map_H_sensor.getOrigin();
  octomap::point3d sensor_origin(sensor_origin_tf.getX(), sensor_origin_tf.getY(), sensor_origin_tf.getZ());
  Eigen::Vector3d sensor_origin_eigen(sensor_origin_tf.getX(), sensor_origin_tf.getY(), sensor_origin_tf.getZ());


  if (!updateTransformCache(cloud_msg_in->header.frame_id, cloud_msg_in->header.stamp))
  {
    ROS_ERROR_THROTTLE(1, "Transform cache was not updated. Self-filtering may fail.");
    return;
  }

  //if (cloud_msg_in->height > 1){
  //  ROS_ERROR_THROTTLE(1,"Cloud height > 1 not supported by lidar octomap updater!");
  //  return;
  //}

  uint32_t index_offset = -1;
  bool has_intensity = false;

  //Check if field "index" exists, exit otherwise
  for(unsigned int i = 0; i < cloud_msg_in->fields.size(); ++i)
  {
    if(cloud_msg_in->fields[i].name == "index")
    {
      index_offset = cloud_msg_in->fields[i].offset;
    }

    else if(cloud_msg_in->fields[i].name == "intensity")
    {
      has_intensity = true;
    }
    //ROS_INFO("Field name: %s ", cloud_msg_in->fields[i].name.c_str());
  }

  //ROS_INFO("Index offset: %d", index_offset);

  if (index_offset < 0){
    ROS_ERROR_THROTTLE(1, "Index offset for cloud < 0, aborting.");
    return;
  }


  /* mask out points on the robot */

  //We are in world frame, so using the scan max range cuts off things in a circle around world origin.
  //@TODO: Do this in a nicer way.
  double max_mask_range = 20000.0;

  {
    boost::mutex::scoped_lock scoped_lock(shape_lock_);
    shape_mask_->maskContainment(*cloud_msg_in, sensor_origin_eigen, 0.0, max_mask_range, mask_);
    updateMask(*cloud_msg_in, sensor_origin_eigen, mask_);
  }

  self_filter_finished_time = ros::WallTime::now();

  octomap::KeySet free_cells, occupied_cells, model_cells, clip_cells;
  //free_cells.clear();
  //occupied_cells.clear();
  //model_cells.clear();
  //clip_cells.clear();
  //std::cout << free_cells.bucket_count() <<  "\n";
  boost::scoped_ptr<sensor_msgs::PointCloud2> filtered_cloud;

  //We only use these iterators if we are creating a filtered_cloud for
  //publishing. We cannot default construct these, so we use scoped_ptr's
  //to defer construction
  boost::scoped_ptr<sensor_msgs::PointCloud2Iterator<float> > iter_filtered_x;
  boost::scoped_ptr<sensor_msgs::PointCloud2Iterator<float> > iter_filtered_y;
  boost::scoped_ptr<sensor_msgs::PointCloud2Iterator<float> > iter_filtered_z;
  boost::scoped_ptr<sensor_msgs::PointCloud2Iterator<float> > iter_filtered_i;

  bool publish_filtered_cloud = (!filtered_cloud_topic_.empty()) && (filtered_cloud_publisher_.getNumSubscribers() > 0);

  if (publish_filtered_cloud) {
    filtered_cloud.reset(new sensor_msgs::PointCloud2());
    filtered_cloud->header = cloud_msg_in->header;
    sensor_msgs::PointCloud2Modifier pcd_modifier(*filtered_cloud);
    //pcd_modifier.setPointCloud2FieldsByString(2, "xyz", "i");
    pcd_modifier.setPointCloud2Fields(4,
      "x", 1, sensor_msgs::PointField::FLOAT32,
      "y", 1, sensor_msgs::PointField::FLOAT32,
      "z", 1, sensor_msgs::PointField::FLOAT32,
      "intensity", 1, sensor_msgs::PointField::FLOAT32);
    pcd_modifier.resize(cloud_msg_in->width * cloud_msg_in->height);

    //we have created a filtered_out, so we can create the iterators now
    iter_filtered_x.reset(new sensor_msgs::PointCloud2Iterator<float>(*filtered_cloud, "x"));
    iter_filtered_y.reset(new sensor_msgs::PointCloud2Iterator<float>(*filtered_cloud, "y"));
    iter_filtered_z.reset(new sensor_msgs::PointCloud2Iterator<float>(*filtered_cloud, "z"));
    iter_filtered_i.reset(new sensor_msgs::PointCloud2Iterator<float>(*filtered_cloud, "intensity"));
  }

  size_t filtered_cloud_size = 0;

  tree_->lockRead();

  bool octomap_updates_disabled_for_this_cloud = disable_octomap_updates_;

  //Make copy of filtered scan, modify below with self filter information
  //scan_self_filtered_ = scan_filtered_;

  double max_range_squared = max_range_ * max_range_;

  try
  {
    /* do ray tracing to find which cells this point cloud indicates should be free, and which it indicates
     * should be occupied */
    for (unsigned int row = 0; row < cloud_msg_in->height; row += point_subsample_)
    {
      unsigned int row_c = row * cloud_msg_in->width;
      sensor_msgs::PointCloud2ConstIterator<float> pt_iter(*cloud_msg_in, "x");
      //sensor_msgs::PointCloud2ConstIterator<unsigned int> index_iter(*cloud_msg_in, "index");
      //set iterator to point at start of the current rowindex_iter
      pt_iter += row_c;

      for (unsigned int col = 0; col < cloud_msg_in->width; col += point_subsample_,
        pt_iter += point_subsample_) //index_iter += point_subsample_)
      {
        //if (mask_[row_c + col] == point_containment_filter::ShapeMask::CLIP)
        //  continue;

        if (has_intensity && std::isnan(pt_iter[3]))
          continue;

        //if (sensor_origin_tf.distance(tf::Vector3(pt_iter[0], pt_iter[1], pt_iter[2])) > this->max_range_)
        if (tf::Vector3(pt_iter[0], pt_iter[1], pt_iter[2]).length2() > max_range_squared)
          continue;

        /* check for NaN */
        if (!std::isnan(pt_iter[0]) && !std::isnan(pt_iter[1]) && !std::isnan(pt_iter[2]))
        {
          /* transform to map frame */
          //tf::Vector3 point_tf = map_H_sensor * tf::Vector3(pt_iter[0], pt_iter[1],
          //  pt_iter[2]);





          tf::Vector3 point_tf (map_H_sensor * tf::Vector3(pt_iter[0], pt_iter[1], pt_iter[2]));

          /* occupied cell at ray endpoint if ray is shorter than max range and this point
             isn't on a part of the robot*/
          if (mask_[row_c + col] == point_containment_filter::ShapeMask::INSIDE){
            model_cells.insert(tree_->coordToKey(point_tf.getX(), point_tf.getY(), point_tf.getZ()));
            //scan_self_filtered_.ranges[index_iter[0]] = std::numeric_limits<float>::quiet_NaN();
          }
          else if (mask_[row_c + col] == point_containment_filter::ShapeMask::CLIP)
          {
            // CLIP cells are only those outside min to max range. Given we operate in world frame for
            // containment check and scan is prefiltered for min/max, there are no CLIP cells.
            //clip_cells.insert(tree_->coordToKey(point_tf.getX(), point_tf.getY(), point_tf.getZ()));
          }
          else
          {
            occupied_cells.insert(tree_->coordToKey(point_tf.getX(), point_tf.getY(), point_tf.getZ()));
            //build list of valid points if we want to publish them
            if (publish_filtered_cloud)
            {
              **iter_filtered_x = pt_iter[0];
              **iter_filtered_y = pt_iter[1];
              **iter_filtered_z = pt_iter[2];
              **iter_filtered_i = has_intensity ? pt_iter[3] : 1.0f;
              ++filtered_cloud_size;
              ++*iter_filtered_x;
              ++*iter_filtered_y;
              ++*iter_filtered_z;
              ++*iter_filtered_i;
            }
          }
        }
      }
    }

    if (!octomap_updates_disabled_for_this_cloud){
        /* compute the free cells along each ray that ends at an occupied cell */
        for (octomap::KeySet::iterator it = occupied_cells.begin(), end = occupied_cells.end(); it != end; ++it)
          if (tree_->computeRayKeys(sensor_origin, tree_->keyToCoord(*it), key_ray_))
            free_cells.insert(key_ray_.begin(), key_ray_.end());

        /* compute the free cells along each ray that ends at a model cell */
        for (octomap::KeySet::iterator it = model_cells.begin(), end = model_cells.end(); it != end; ++it)
          if (tree_->computeRayKeys(sensor_origin, tree_->keyToCoord(*it), key_ray_))
            free_cells.insert(key_ray_.begin(), key_ray_.end());

        /* compute the free cells along each ray that ends at a clipped cell */
        for (octomap::KeySet::iterator it = clip_cells.begin(), end = clip_cells.end(); it != end; ++it)
          if (tree_->computeRayKeys(sensor_origin, tree_->keyToCoord(*it), key_ray_))
            free_cells.insert(key_ray_.begin(), key_ray_.end());
    }

    //std::cout << free_cells.bucket_count() <<  "\n";
  }
  catch (...)
  {
    ROS_ERROR("Caught expection during point cloud update!");
    tree_->unlockRead();
    return;
  }

  tree_->unlockRead();


  // ----------------- Finished with self filtering, can publish filtered data now

  /*
  if (filtered_localized_scan_publisher_.getNumSubscribers() > 0){

    filtered_localized_scan_.header.stamp = scan_msg->header.stamp;
    filtered_localized_scan_.header.frame_id = monitor_->getMapFrame();

    filtered_localized_scan_.processed_scan.header = scan_msg->header;

    filtered_localized_scan_.processed_scan.angle_min = scan_msg->angle_min;
    filtered_localized_scan_.processed_scan.angle_max = scan_msg->angle_max;
    filtered_localized_scan_.processed_scan.angle_increment = scan_msg->angle_increment;
    filtered_localized_scan_.processed_scan.time_increment = scan_msg->time_increment;
    filtered_localized_scan_.processed_scan.scan_time = scan_msg->scan_time;
    filtered_localized_scan_.processed_scan.range_min = scan_msg->range_min;
    filtered_localized_scan_.processed_scan.range_max = scan_msg->range_max;

    filtered_localized_scan_.processed_scan.ranges[vigir_perception_msgs::FilteredLocalizedLaserScan::SCAN_PREPROCESSED].echoes = scan_filtered_.ranges;
    filtered_localized_scan_.processed_scan.ranges[vigir_perception_msgs::FilteredLocalizedLaserScan::SCAN_SELF_FILTERED].echoes = scan_self_filtered_.ranges;
    filtered_localized_scan_.processed_scan.ranges[vigir_perception_msgs::FilteredLocalizedLaserScan::SCAN_RAW].echoes = scan_msg->ranges;
    filtered_localized_scan_.processed_scan.intensities[0].echoes = scan_msg->intensities;

    tf::transformTFToMsg(map_H_sensor, filtered_localized_scan_.transform_first_ray);
    tf::transformTFToMsg(map_H_sensor_last_ray, filtered_localized_scan_.transform_last_ray);

    filtered_localized_scan_publisher_.publish(filtered_localized_scan_);
  }

  if (scan_filtered_publisher_.getNumSubscribers() > 0){
    scan_filtered_publisher_.publish(scan_filtered_);
  }

  if (scan_self_filtered_publisher_.getNumSubscribers() > 0){
    scan_self_filtered_publisher_.publish(scan_self_filtered_);
  }
  */

  if (publish_filtered_cloud)
  {
    sensor_msgs::PointCloud2Modifier pcd_modifier(*filtered_cloud);
    pcd_modifier.resize(filtered_cloud_size);
    filtered_cloud_publisher_.publish(*filtered_cloud);
  }



  // ----------------- Update octomap --------------------

  if (!octomap_updates_disabled_for_this_cloud){

      /* cells that overlap with the model are not occupied */
      for (octomap::KeySet::iterator it = model_cells.begin(), end = model_cells.end(); it != end; ++it)
        occupied_cells.erase(*it);

      /* occupied cells are not free */
      for (octomap::KeySet::iterator it = occupied_cells.begin(), end = occupied_cells.end(); it != end; ++it)
        free_cells.erase(*it);

      tree_->lockWrite();

      try
      {
        /* mark free cells only if not seen occupied in this cloud */
        for (octomap::KeySet::iterator it = free_cells.begin(), end = free_cells.end(); it != end; ++it)
          tree_->updateNode(*it, false);

        /* now mark all occupied cells */
        for (octomap::KeySet::iterator it = occupied_cells.begin(), end = occupied_cells.end(); it != end; ++it)
          tree_->updateNode(*it, true);

        // set the logodds to the minimum for the cells that are part of the model
        const float lg = tree_->getClampingThresMinLog() - tree_->getClampingThresMaxLog();
        for (octomap::KeySet::iterator it = model_cells.begin(), end = model_cells.end(); it != end; ++it)
          tree_->updateNode(*it, lg);
      }
      catch (...)
      {
        ROS_ERROR("Internal error while updating octree");
      }
      tree_->unlockWrite();
      tree_->triggerUpdateCallback();
  }

  ROS_DEBUG("Processed laser scan in %lf ms. Self filtering took %lf ms", (ros::WallTime::now() - start).toSec() * 1000.0, (self_filter_finished_time - start).toSec() * 1000.0 );
}

void LidarOctomapUpdater::initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped pose)
{
  {
    ROS_INFO("Received intialpose, resetting planning scene octomap");

    this->resetOctomap();
  }
}

bool LidarOctomapUpdater::clearOctomap(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  this->resetOctomap();

  return true;
}

bool LidarOctomapUpdater::disableOctomapUpdates(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
  if (req.data){
    ROS_INFO("Disable Octomap updates");
  }else{
    ROS_INFO("Enable Octomap updates");
  }

  tree_->lockWrite();
  disable_octomap_updates_ = req.data;
  tree_->unlockWrite();


  res.success = true;
  return true;
}


bool LidarOctomapUpdater::clearRobotVicinityOctomap(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  if (tf_)
  {
    tf::StampedTransform robot_pose;
    try
    {
      ros::Time end_time   = scan_filtered_.header.stamp + ros::Duration().fromSec((scan_filtered_.ranges.size() -1)*scan_filtered_.time_increment) ;


      if(tf_->waitForTransform(monitor_->getMapFrame(), "base_link", ros::Time(0), wait_duration_)){
        tf_->lookupTransform(monitor_->getMapFrame(), "base_link", ros::Time(0), robot_pose);
      }
    }
    catch (tf::TransformException& ex)
    {
      ROS_ERROR_STREAM("Transform error of sensor data: " << ex.what() << "; quitting selective clearing callback in LidarOctomapUpdater");
      return false;
    }
    catch(...)
    {
      ROS_ERROR_STREAM("Exception while retrieving robot pose for selective clearing in LidarOctomapUpdater");
      return false;
    }



    octomap::point3d min, max;

    min.x() = robot_pose.getOrigin().x() - 0.3;
    min.y() = robot_pose.getOrigin().y() - 0.3;
    min.z() = robot_pose.getOrigin().z();

    max.x() = robot_pose.getOrigin().x() + 0.3;
    max.y() = robot_pose.getOrigin().y() + 0.3;
    max.z() = robot_pose.getOrigin().z() + 1.1;

    ROS_INFO("Clearing octomap around robot, coords min x: %f y: %f z: %f max x: %f y: %f z: %f", min.x(), min.y(), min.z(), max.x(), max.y(), max.z());

    tree_->lockWrite();

    for(octomap::OcTree::leaf_bbx_iterator it = tree_->begin_leafs_bbx(min,max),
        end=tree_->end_leafs_bbx(); it!= end; ++it)
    {
      tree_->deleteNode(it.getKey(), it.getDepth());
    }

    tree_->unlockWrite();


  }
  else
  {
    ROS_ERROR("No tf listener, cannot run selective clearing in LidarOctomapUpdater");
    return false;
  }



  return true;
}

void LidarOctomapUpdater::resetOctomap(bool use_prior)
{
  if (use_prior && !p_prior_map_file_.empty()){
    ROS_INFO("Resetting planning scene octomap to prior map");

    tree_->lockWrite();

    if (!tree_->readBinary(p_prior_map_file_)){
      ROS_WARN("Failed reading prior octomap, instead completely clearing octomap!");
      tree_->clear();
    }

    tree_->unlockWrite();

  }else{
    ROS_INFO("Resetting planning scene octomap to cleared map");
    tree_->lockWrite();
    tree_->clear();
    tree_->unlockWrite();
  }
}

void LidarOctomapUpdater::serviceThread(){
    ros::Rate rate(100.0);
    while (ros::ok()){
        service_queue_.callAvailable(ros::WallDuration(1.0));
        rate.sleep();
    }
}


/*
void move_group::OctomapRaycastCapability::dynRecParamCallback(hector_move_group_capabilities::OctomapRaycastCapabilityConfig &config, uint32_t level)
{
  octo_min_distance_ = config.min_distance_to_obstacle;
  octo_max_distance_ = config.max_distance_to_obstacle;
  secondary_rays_max_dist_ = config.secondary_rays_max_dist;
  secondary_rays_opening_angle_ = angles::from_degrees(config.secondary_rays_opening_angle_deg);
}
*/

bool LidarOctomapUpdater::lookupServiceCallback(hector_nav_msgs::GetDistanceToObstacle::Request  &req,
                                                                 hector_nav_msgs::GetDistanceToObstacle::Response &res )
{

  ROS_DEBUG("Octomap distance lookup service called");
  tf::StampedTransform camera_transform;

  //const std::string target_frame = context_->planning_scene_monitor_->getPlanningScene()->getPlanningFrame();
  const std::string target_frame = monitor_->getMapFrame();

  try{
    tf_->waitForTransform(target_frame ,req.point.header.frame_id, req.point.header.stamp, ros::Duration(1.0));
    tf_->lookupTransform(target_frame, req.point.header.frame_id, req.point.header.stamp, camera_transform);
  }catch(tf::TransformException e){
    ROS_ERROR("Transform failed in lookup distance service call: %s",e.what());
    return false;
  }

  bool useOutliers = true;


  const octomap::point3d origin = octomap::pointTfToOctomap(camera_transform.getOrigin());

  tf::Point end_point = camera_transform * tf::Point(req.point.point.x, req.point.point.y, req.point.point.z);
  tf::Vector3 direction = end_point - camera_transform.getOrigin();

  const octomap::point3d directionOc = octomap::pointTfToOctomap(direction);
  std::vector<octomap::point3d> endPoints;
  std::vector<float> distances;
  int n=2;
  endPoints.resize(1);
  std::vector<octomap::point3d> directions;
  directions.push_back(directionOc);

  //planning_scene_monitor::LockedPlanningSceneRO ls (context_->planning_scene_monitor_);

  // Below is inspired by
  // https://github.com/ros-planning/moveit_core/blob/jade-devel/planning_scene/src/planning_scene.cpp#L850
  // and quite hacky. There should be a better way to access the octomap (at least read-only)?
  //collision_detection::CollisionWorld::ObjectConstPtr map = ls.getPlanningSceneMonitor()->getPlanningScene()->getWorld()->getObject("<octomap>");
  //const shapes::OcTree* octree_shape = static_cast<const shapes::OcTree*>(map->shapes_[0].get());
  const std::shared_ptr<const octomap::OcTree> octree_ = tree_;

  tree_->lockRead();

  if(octree_->castRay(origin,directions[0],endPoints[0],true,octo_max_distance_)) {
    distances.push_back(origin.distance(endPoints[0]));
  }

  if (distances.size()!=0) {
    int count_outliers;
    endPoints.resize(1+(n*2));
    get_endpoints(origin, *octree_, distances[0], direction, directions, endPoints, n);
    if (useOutliers) {
      double distance_threshold=0.7;
      count_outliers=0;
      for (size_t i =1;i<endPoints.size();i++){
        ROS_DEBUG("distance to checkpoints %f",endPoints[0].distance(endPoints[i]));
        if(endPoints[0].distance(endPoints[i])>distance_threshold) {
          count_outliers++;
        }
      }
      ROS_DEBUG("corner case: number of outliers: %d ",count_outliers);
    }

    res.distance = distances[0];

    res.end_point.header.frame_id = target_frame;
    res.end_point.header.stamp = req.point.header.stamp;
    res.end_point.point.x = endPoints[0].x();
    res.end_point.point.y = endPoints[0].y();
    res.end_point.point.z = endPoints[0].z();

    if (res.distance < octo_min_distance_) {
      res.distance = -1.0;
      ROS_WARN("Octomap GetDistanceToObstacle got distance under min_Distance -> returning -1.0");
    }

    if (useOutliers) {
      if (count_outliers>=n-1) {
        res.distance=-1.0;
        ROS_DEBUG("Octomap GetDistanceToObstacle Corner Case");
      }
    }
  } else {
    res.distance=-1.0;
  }

  tree_->unlockRead();

  ROS_DEBUG("Result: getDistanceObstacle_Octo: distance: %f",res.distance);
  if (m_markerPub.getNumSubscribers() > 0){
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker marker;
    marker.header.stamp = req.point.header.stamp;
    marker.header.frame_id = target_frame;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.color.r= 1.0;
    marker.color.a = 1.0;
    marker.scale.x = 0.02;
    marker.ns ="";
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;

    std::vector<geometry_msgs::Point> point_vector;
    for (size_t i = 0; i < endPoints.size(); ++i){
      geometry_msgs::Point tmp = octomap::pointOctomapToMsg(origin);
      point_vector.push_back(tmp);

      tmp = octomap::pointOctomapToMsg(endPoints[i]);
      point_vector.push_back(tmp);
    }
    marker.points=point_vector;
    marker_array.markers.push_back(marker);
    m_markerPub.publish(marker_array);
  }

  return true;
}

void LidarOctomapUpdater::cast_ray_mod_direction(
                                            const octomap::point3d& origin,
                                            const octomap::OcTree& octree,
                                            const tf::Vector3& direction,
                                            double pitch,
                                            double yaw,
                                            octomap::point3d& end_point)
{
  tf::Vector3 dir_mod = direction;
  dir_mod = dir_mod.rotate(tf::Vector3(0.0 ,0.0, 1.0), yaw);
  dir_mod = dir_mod.rotate(tf::Vector3(0.0, 1.0 ,0.0), pitch);

  const octomap::point3d dir_mod_oc = octomap::pointTfToOctomap(dir_mod);

  octree.castRay(origin, dir_mod_oc, end_point, true, secondary_rays_max_dist_);
}

void LidarOctomapUpdater::get_endpoints(const octomap::point3d& origin,
                                                         const octomap::OcTree& octree,
                                                         const float reference_distance,
                                                         const tf::Vector3& direction,
                                                         std::vector<octomap::point3d>& directions,
                                                         std::vector<octomap::point3d>& endPoints,
                                                         int n){

  cast_ray_mod_direction(origin, octree, direction,  0.0 , secondary_rays_opening_angle_, endPoints[1]);
  cast_ray_mod_direction(origin, octree, direction,  0.0 ,-secondary_rays_opening_angle_, endPoints[2]);
  cast_ray_mod_direction(origin, octree, direction,  secondary_rays_opening_angle_ , 0.0, endPoints[3]);
  cast_ray_mod_direction(origin, octree, direction, -secondary_rays_opening_angle_ , 0.0, endPoints[4]);
  /*
    tf::Vector3 z_axis(0,0,1);

    double tolerance=octree.getResolution()*0.05;

    for (int i=1;i<=n;i++){
        double angle=std::atan2((octree.getResolution()*i)+tolerance,reference_distance);

        tf::Vector3 direction_z_plus = direction.rotate(z_axis, +angle);
        tf::Vector3 direction_z_minus = direction.rotate(z_axis, -angle);

        const octomap::point3d direction_z_plus_Oc = octomap::pointTfToOctomap(direction_z_plus);
        const octomap::point3d direction_z_minus_Oc = octomap::pointTfToOctomap(direction_z_minus);

        directions.push_back(direction_z_plus_Oc);
        directions.push_back(direction_z_minus_Oc);

        octree.castRay(origin,directions[2*i-1],endPoints[2*i-1],true,5.0);
        octree.castRay(origin,directions[2*i],endPoints[2*i],true,5.0);

    }
    */
}


}
