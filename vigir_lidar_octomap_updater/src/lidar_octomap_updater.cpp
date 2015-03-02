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

#include <tf/transform_datatypes.h>


namespace occupancy_map_monitor
{

LidarOctomapUpdater::LidarOctomapUpdater() : OccupancyMapUpdater("LidarCloudUpdater"),
                                                       private_nh_("~"),
                                                       scale_(1.0),
                                                       padding_(0.0),
                                                       max_range_(std::numeric_limits<double>::infinity()),
                                                       point_subsample_(1),
                                                       point_cloud_subscriber_(NULL),
                                                       point_cloud_filter_(NULL),
                                                       filter_chain_("sensor_msgs::LaserScan")
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
    if (!params.hasMember("scan_topic"))
      return false;
    point_cloud_topic_ = static_cast<const std::string&>(params["scan_topic"]);

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
  return true;
}

void LidarOctomapUpdater::start()
{
  if (point_cloud_subscriber_)
    return;
  /* subscribe to point cloud topic using tf filter*/
  point_cloud_subscriber_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(root_nh_, point_cloud_topic_, 40);
  if (tf_ && !monitor_->getMapFrame().empty())
  {
    point_cloud_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(*point_cloud_subscriber_, *tf_, monitor_->getMapFrame(), 40);
    point_cloud_filter_->registerCallback(boost::bind(&LidarOctomapUpdater::cloudMsgCallback, this, _1));
    ROS_INFO("Listening to '%s' using message filter with target frame '%s'", point_cloud_topic_.c_str(), point_cloud_filter_->getTargetFramesString().c_str());
  }
  else
  {
    point_cloud_subscriber_->registerCallback(boost::bind(&LidarOctomapUpdater::cloudMsgCallback, this, _1));
    ROS_INFO("Listening to '%s'", point_cloud_topic_.c_str());
  }
}

void LidarOctomapUpdater::stopHelper()
{
  delete point_cloud_filter_;
  delete point_cloud_subscriber_;
}

void LidarOctomapUpdater::stop()
{
  stopHelper();
  point_cloud_filter_ = NULL;
  point_cloud_subscriber_ = NULL;
}

ShapeHandle LidarOctomapUpdater::excludeShape(const shapes::ShapeConstPtr &shape)
{
  ShapeHandle h = 0;
  if (shape_mask_)
    h = shape_mask_->addShape(shape, scale_, padding_);
  else
    ROS_ERROR("Shape filter not yet initialized!");
  return h;
}

void LidarOctomapUpdater::forgetShape(ShapeHandle handle)
{
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

void LidarOctomapUpdater::cloudMsgCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
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
        ROS_ERROR_STREAM("Transform error of sensor data: " << ex.what() << "; quitting callback");
        return;
      }
    }
    else
      return;
  }

  /* compute sensor origin in map frame */
  const tf::Vector3 &sensor_origin_tf = map_H_sensor.getOrigin();
  octomap::point3d sensor_origin(sensor_origin_tf.getX(), sensor_origin_tf.getY(), sensor_origin_tf.getZ());
  Eigen::Vector3d sensor_origin_eigen(sensor_origin_tf.getX(), sensor_origin_tf.getY(), sensor_origin_tf.getZ());

  projector_.transformLaserScanToPointCloud(monitor_->getMapFrame(), scan_filtered_, *cloud_msg, *tf_, max_range_, laser_geometry::channel_option::Intensity|laser_geometry::channel_option::Index);

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

  //Check if field "index" exists, exit otherwise
  for(unsigned int i = 0; i < cloud_msg->fields.size(); ++i)
  {
    if(cloud_msg->fields[i].name == "index")
    {
      index_offset = cloud_msg->fields[i].offset;
    }
    //ROS_INFO("Field name: %s ", cloud_msg->fields[i].name.c_str());
  }

  //ROS_INFO("Index offset: %d", index_offset);

  if (index_offset < 0){
    ROS_ERROR_THROTTLE(1, "Index offset for cloud < 0, aborting.");
    return;
  }


  /* mask out points on the robot */
  shape_mask_->maskContainment(*cloud_msg, sensor_origin_eigen, 0.0, max_range_, mask_);
  updateMask(*cloud_msg, sensor_origin_eigen, mask_);

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

  bool publish_filtered_cloud = (!filtered_cloud_topic_.empty()) && (filtered_cloud_publisher_.getNumSubscribers() > 0);

  if (publish_filtered_cloud) {
    filtered_cloud.reset(new sensor_msgs::PointCloud2());
    filtered_cloud->header = cloud_msg->header;
    sensor_msgs::PointCloud2Modifier pcd_modifier(*filtered_cloud);
    pcd_modifier.setPointCloud2FieldsByString(1, "xyz");
    pcd_modifier.resize(cloud_msg->width * cloud_msg->height);

    //we have created a filtered_out, so we can create the iterators now
    iter_filtered_x.reset(new sensor_msgs::PointCloud2Iterator<float>(*filtered_cloud, "x"));
    iter_filtered_y.reset(new sensor_msgs::PointCloud2Iterator<float>(*filtered_cloud, "y"));
    iter_filtered_z.reset(new sensor_msgs::PointCloud2Iterator<float>(*filtered_cloud, "z"));
  }

  size_t filtered_cloud_size = 0;

  tree_->lockRead();

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

        /* check for NaN */
        if (!isnan(pt_iter[0]) && !isnan(pt_iter[1]) && !isnan(pt_iter[2]))
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
              ++filtered_cloud_size;
              ++*iter_filtered_x;
              ++*iter_filtered_y;
              ++*iter_filtered_z;
            }
          }
        }
      }
    }

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

  ROS_DEBUG("Processed laser scan in %lf ms. Self filtering took %lf ms", (ros::WallTime::now() - start).toSec() * 1000.0, (self_filter_finished_time - start).toSec() * 1000.0 );
}

}
