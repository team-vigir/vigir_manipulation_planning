/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013-2015, Team ViGIR ( TORC Robotics LLC, TU Darmstadt, Virginia Tech, Oregon State University, Cornell University, and Leibniz University Hanover )
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
 *   * Neither the name of Team ViGIR, TORC Robotics, nor the names of its
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
//@TODO_ADD_AUTHOR_INFO
#include <moveit/vigir_motion_planning_rviz_plugin/motion_planning_frame.h>
#include <moveit/vigir_motion_planning_rviz_plugin/motion_planning_display.h>

#include <vigir_planning_msgs/ExtendedPlanningOptions.h>

#include <QTreeWidgetItem>
#include <QDoubleSpinBox>
#include <QSignalMapper>
#include <QFileDialog>
#include <QDomDocument>
#include <QMessageBox>

#include "ui_motion_planning_rviz_plugin_frame.h"


namespace vigir_moveit_rviz_plugin
{

void MotionPlanningFrame::initCartesianTrajectoryTab()
{
    int tab_page_index = ui_->tabWidget->indexOf(ui_->cartesian_trajectory);
    ui_->tabWidget->removeTab(tab_page_index);

    cartesian_trajectory_orientation_group_.addButton(ui_->keep_full_orientation_button, vigir_planning_msgs::ExtendedPlanningOptions::ORIENTATION_FULL);
    cartesian_trajectory_orientation_group_.addButton(ui_->use_axis_only_button, vigir_planning_msgs::ExtendedPlanningOptions::ORIENTATION_AXIS_ONLY);
    cartesian_trajectory_orientation_group_.addButton(ui_->ignore_orientation_button, vigir_planning_msgs::ExtendedPlanningOptions::ORIENTATION_IGNORE);

    connect(&cartesian_trajectory_orientation_group_, SIGNAL(buttonClicked(int)), this, SLOT(cartesianTrajectoryOrientationChanged()));

    clearWaypoints();
}

void MotionPlanningFrame::addWaypointButtonClicked()
{
    QList<QTreeWidgetItem*> selected_items = ui_->waypoints_treewidget->selectedItems();
    if ( selected_items.size() == 1 ) { // exactly one item => copy it
      // check if item is toplevel-item
      if ( ui_->waypoints_treewidget->indexOfTopLevelItem((selected_items[0])) != -1)
      {
            addWaypoint(selected_items[0]);
      }
      else {
          addWaypoint();
      }
    }
    else {
        addWaypoint();
    }


}

void MotionPlanningFrame::removeWaypointButtonClicked()
{
  QTreeWidgetItem *current_item = ui_->waypoints_treewidget->currentItem();
  
  if ( current_item == NULL)
      return;

  // find top-level item
  while( current_item->parent() ) {
    current_item = current_item->parent();
  }
  
  removeWaypoint(current_item);
}

void MotionPlanningFrame::clearWaypointsButtonClicked()
{
  clearWaypoints();
}

void MotionPlanningFrame::loadCartesianTrajectoryButtonClicked()
{
  QString filename = QFileDialog::getOpenFileName(this, QString(), QString(), "XML (*.xml)");
  
  if ( filename.isEmpty() )
    return;

  vigir_planning_msgs::ExtendedPlanningOptionsPtr extended_planning_options = loadExtendedPlanningOptions(filename);
  clearWaypoints();
  
  QDoubleSpinBox *current_spin_box = NULL;
  QLineEdit *current_line_edit = NULL;
  for ( int i = 0; i < extended_planning_options->target_poses.size(); i++ ) {
    QTreeWidgetItem *waypoint_item = addWaypoint();
    waypoint_item->setText(0, QString::fromStdString(extended_planning_options->target_pose_names[i]));

    // set position values
    current_spin_box = (QDoubleSpinBox*)(ui_->waypoints_treewidget->itemWidget(waypoint_item->child(0)->child(0), 1));
    current_spin_box->setValue( extended_planning_options->target_poses[i].position.x );
    current_spin_box = (QDoubleSpinBox*)(ui_->waypoints_treewidget->itemWidget(waypoint_item->child(0)->child(1), 1));
    current_spin_box->setValue( extended_planning_options->target_poses[i].position.y );
    current_spin_box = (QDoubleSpinBox*)(ui_->waypoints_treewidget->itemWidget(waypoint_item->child(0)->child(2), 1));
    current_spin_box->setValue( extended_planning_options->target_poses[i].position.z );

    // set orientation values
    current_spin_box = (QDoubleSpinBox*)(ui_->waypoints_treewidget->itemWidget(waypoint_item->child(1)->child(0), 1));
    current_spin_box->setValue( extended_planning_options->target_poses[i].orientation.w );
    current_spin_box = (QDoubleSpinBox*)(ui_->waypoints_treewidget->itemWidget(waypoint_item->child(1)->child(1), 1));
    current_spin_box->setValue( extended_planning_options->target_poses[i].orientation.x );
    current_spin_box = (QDoubleSpinBox*)(ui_->waypoints_treewidget->itemWidget(waypoint_item->child(1)->child(2), 1));
    current_spin_box->setValue( extended_planning_options->target_poses[i].orientation.y );
    current_spin_box = (QDoubleSpinBox*)(ui_->waypoints_treewidget->itemWidget(waypoint_item->child(1)->child(3), 1));
    current_spin_box->setValue( extended_planning_options->target_poses[i].orientation.z );

    // set target link names
    current_line_edit = (QLineEdit*)(ui_->waypoints_treewidget->itemWidget(waypoint_item->child(2), 1));
    current_line_edit->setText( QString::fromStdString(extended_planning_options->target_link_names[i]));

    // set target link axis values
    current_spin_box = (QDoubleSpinBox*)(ui_->waypoints_treewidget->itemWidget(waypoint_item->child(3)->child(0), 1));
    current_spin_box->setValue( extended_planning_options->target_link_axis[i].x );
    current_spin_box = (QDoubleSpinBox*)(ui_->waypoints_treewidget->itemWidget(waypoint_item->child(3)->child(1), 1));
    current_spin_box->setValue( extended_planning_options->target_link_axis[i].y );
    current_spin_box = (QDoubleSpinBox*)(ui_->waypoints_treewidget->itemWidget(waypoint_item->child(3)->child(2), 1));
    current_spin_box->setValue( extended_planning_options->target_link_axis[i].z );


    // set target pose times
    current_spin_box = (QDoubleSpinBox*)(ui_->waypoints_treewidget->itemWidget(waypoint_item->child(4), 1));
    current_spin_box->setValue( extended_planning_options->target_pose_times[i]);
  }

  // read orientation mode
  int orientation_mode = extended_planning_options->target_orientation_type;
  cartesian_trajectory_orientation_group_.button(orientation_mode)->setChecked(true);

  // read sample rate
  ui_->sample_rate_spin->setValue( extended_planning_options->trajectory_sample_rate );

  // read check self-collisions flag
  bool check_self_collisions = extended_planning_options->check_self_collisions;
  ui_->check_self_collisions_checkbox->setChecked(check_self_collisions);
  
}

vigir_planning_msgs::ExtendedPlanningOptionsPtr MotionPlanningFrame::loadExtendedPlanningOptions(QString filename)
{
    QDomDocument doc("CartesianTrajectoryOptions");
    QFile file(filename);
    if (!file.open(QIODevice::ReadOnly)) {
      qWarning("Could not open file for reading");
      return vigir_planning_msgs::ExtendedPlanningOptionsPtr();
    }

    if (!doc.setContent(&file)) {
        qWarning("Could not read xml content from file");
       file.close();
       return vigir_planning_msgs::ExtendedPlanningOptionsPtr();
    }

    vigir_planning_msgs::ExtendedPlanningOptionsPtr extended_planning_options(new vigir_planning_msgs::ExtendedPlanningOptions());

    // read waypoints
    QDomNodeList waypointsList = doc.elementsByTagName ( "Waypoint" );
    for ( int i = 0; i < waypointsList.size(); i++ ) {
      QDomNode current_node = waypointsList.at(i);

      QString target_pose_name = current_node.toElement().attribute("name");
      extended_planning_options->target_pose_names.push_back(target_pose_name.toStdString() );

      geometry_msgs::Pose target_pose;
      QDomElement position_element = current_node.namedItem("Position").toElement();
      target_pose.position.x = position_element.attribute("x").toDouble();
      target_pose.position.y = position_element.attribute("y").toDouble();
      target_pose.position.z = position_element.attribute("z").toDouble();

      QDomElement orientation_element = current_node.namedItem("Orientation").toElement();
      target_pose.orientation.w = orientation_element.attribute("w").toDouble();
      target_pose.orientation.x = orientation_element.attribute("x").toDouble();
      target_pose.orientation.y = orientation_element.attribute("y").toDouble();
      target_pose.orientation.z = orientation_element.attribute("z").toDouble();
      extended_planning_options->target_poses.push_back(target_pose);

      QDomElement target_link_name_element = current_node.namedItem("TargetLinkName").toElement();
      QString target_link_name = target_link_name_element.attribute("value");
      extended_planning_options->target_link_names.push_back(target_link_name.toStdString());

      geometry_msgs::Point target_link_axis;
      QDomElement axis_element = current_node.namedItem("TargetLinkAxis").toElement();
      target_link_axis.x = axis_element.attribute("x").toDouble();
      target_link_axis.y = axis_element.attribute("y").toDouble();
      target_link_axis.z = axis_element.attribute("z").toDouble();
      extended_planning_options->target_link_axis.push_back(target_link_axis);

      QDomElement time_element = current_node.namedItem("Time").toElement();
      double target_pose_time = time_element.attribute("value").toDouble();
      extended_planning_options->target_pose_times.push_back(target_pose_time);
    }

    // read orientation mode
    QDomElement orientation_element = doc.elementsByTagName("OrientationMode").at(0).toElement();
    int orientation_mode = orientation_element.attribute("value").toInt();
    extended_planning_options->target_orientation_type = orientation_mode;

    // read sample rate
    QDomElement sample_rate_element = doc.elementsByTagName("SampleRate").at(0).toElement();
    double sample_rate = sample_rate_element.attribute("value").toDouble();
    extended_planning_options->trajectory_sample_rate = sample_rate;

    // read check self-collisions flag
    QDomElement check_self_collisions_element = doc.elementsByTagName("CheckSelfCollisions").at(0).toElement();
    bool check_self_collisions = (check_self_collisions_element.attribute("value").toInt() == 1);
    extended_planning_options->check_self_collisions = check_self_collisions;

    return extended_planning_options;
}

void MotionPlanningFrame::saveCartesianTrajectoryButtonClicked()
{
  QString filename = QFileDialog::getSaveFileName(this, QString(), QString(), "XML (*.xml)");
  
  if ( filename.isEmpty() )
    return;

  vigir_planning_msgs::ExtendedPlanningOptionsPtr extended_planning_options = buildExtendedPlanningOptions();
  saveExtendedPlanningOptions(extended_planning_options, filename);
}

bool MotionPlanningFrame::saveExtendedPlanningOptions(vigir_planning_msgs::ExtendedPlanningOptionsPtr extended_planning_options, QString filename)
{
  QDomDocument doc("CartesianTrajectoryOptions");
  QDomElement root = doc.createElement("CartesianTrajectoryOptions");
  doc.appendChild(root);
  
  QDomElement waypoints = doc.createElement("Waypoints");
  root.appendChild(waypoints);

  for ( int i = 0; i < extended_planning_options->target_poses.size(); i++ ) {
    QTreeWidgetItem *rootItem = ui_->waypoints_treewidget->topLevelItem(i);
    geometry_msgs::Pose currentPose = extended_planning_options->target_poses[i];
    std::string currentLinkName = extended_planning_options->target_link_names[i];
    geometry_msgs::Point currentLinkAxis = extended_planning_options->target_link_axis[i];
    double currentTime = extended_planning_options->target_pose_times[i];

    QDomElement currentWaypoint = doc.createElement("Waypoint");
    currentWaypoint.setAttribute("name", rootItem->text(0) );
    waypoints.appendChild(currentWaypoint);    

    // set position
    QDomElement currentPosition = doc.createElement("Position");    
    currentPosition.setAttribute("x", currentPose.position.x);
    currentPosition.setAttribute("y", currentPose.position.y);
    currentPosition.setAttribute("z", currentPose.position.z);
    currentWaypoint.appendChild(currentPosition);
    
    // set orientation
    QDomElement currentOrientation = doc.createElement("Orientation");
    currentOrientation.setAttribute("w", currentPose.orientation.w);
    currentOrientation.setAttribute("x", currentPose.orientation.x);
    currentOrientation.setAttribute("y", currentPose.orientation.y);
    currentOrientation.setAttribute("z", currentPose.orientation.z);
    currentWaypoint.appendChild(currentOrientation);
    
    // set target link name
    QDomElement currentTargetLinkName = doc.createElement("TargetLinkName");
    currentTargetLinkName.setAttribute("value", QString::fromStdString(currentLinkName));
    currentWaypoint.appendChild(currentTargetLinkName);

    // set target link axis
    QDomElement currentAxis = doc.createElement("TargetLinkAxis");
    currentAxis.setAttribute("x", currentLinkAxis.x);
    currentAxis.setAttribute("y", currentLinkAxis.y);
    currentAxis.setAttribute("z", currentLinkAxis.z);
    currentWaypoint.appendChild(currentAxis);

    // set time
    QDomElement currentPoseTime = doc.createElement("Time");
    currentPoseTime.setAttribute("value", currentTime);
    currentWaypoint.appendChild(currentPoseTime);
  }
  
  // set other options
  QDomElement options = doc.createElement("Options");
  root.appendChild(options);
  
  // set orientation mode
  QDomElement orientationMode = doc.createElement("OrientationMode");
  orientationMode.setAttribute("value", extended_planning_options->target_orientation_type);
  options.appendChild(orientationMode);
  
  // set sample rate
  QDomElement sampleRate = doc.createElement("SampleRate");
  sampleRate.setAttribute("value", extended_planning_options->trajectory_sample_rate );
  options.appendChild(sampleRate);
  
  // set self-collision check
  QDomElement checkSelfCollisions = doc.createElement("CheckSelfCollisions");
  checkSelfCollisions.setAttribute("value", extended_planning_options->check_self_collisions );
  options.appendChild(checkSelfCollisions);  
  
  // save to file
  QFile file(filename);
  if (!file.open(QIODevice::Truncate | QIODevice::WriteOnly)) {
    qWarning("Could not open save file...");
    return false;
  }
  QByteArray xml = doc.toByteArray();
  file.write(xml);
  file.close();

  return true;
  
}

void MotionPlanningFrame::planCartesianTrajectoryButtonClicked()
{
    if ( ui_->planning_algorithm_combo_box->currentText() != "drake") {
        QMessageBox::warning(this, tr("Warning"), tr("Selected planning algorithm is not 'drake'. Cartesian planning is only possible with Drake planner selected. Aborting."), QMessageBox::Ok);
        return;
    }

    planning_display_->addBackgroundJob(boost::bind(&MotionPlanningFrame::computeCartesianPlanButtonClicked, this), "compute cartesian plan");
}

void MotionPlanningFrame::cartesianTrajectoryOrientationChanged() {
    bool hide = true;
    if ( cartesian_trajectory_orientation_group_.checkedButton() == ui_->use_axis_only_button ) {
        hide = false;
    }

    for ( int i = 0; i < ui_->waypoints_treewidget->topLevelItemCount(); i++ ) {
        QTreeWidgetItem *axisItem = ui_->waypoints_treewidget->topLevelItem(i)->child(3);
        axisItem->setHidden(hide);
    }
}


  
 
void MotionPlanningFrame::updateWaypointData(QWidget *widget)
{
  QDoubleSpinBox *spinBox = dynamic_cast<QDoubleSpinBox*>(widget);
  if ( spinBox == NULL ) {
    qWarning("MotionPlanningFrame::updateWaypointData(): widget is not q QDoubleSpinBox - aborting");
    return;
  }
    
  
  QTreeWidgetItem *item = (QTreeWidgetItem*)spinBox->property("TreeWidgetItem").value<void*>();
  if ( item == NULL ) {
    qWarning("MotionPlanningFrame::updateWaypointData(): no valid item received - aborting");
    return;
  }
  
  QTreeWidgetItem *parent = item->parent();
  if ( parent == NULL ) {
    qWarning("MotionPlanningFrame::updateWaypointData(): item has no parent - aborting");
    return;
  }
  
  QStringList itemText;
  itemText << "[ ";
  for ( int i = 0; i < parent->childCount(); i++ ) {
    QTreeWidgetItem *child = parent->child(i);
    QWidget *child_widget = ui_->waypoints_treewidget->itemWidget(child, 1);
    QDoubleSpinBox *child_spin = dynamic_cast<QDoubleSpinBox*>(child_widget);
    if ( child_spin == NULL )
      continue;
    
    itemText << QString("%1").arg(child_spin->value(), 0, 'f', 3) << "; ";
  }
  
  itemText.removeLast();
  itemText << " ]";

  parent->setText(1, itemText.join(""));
}

QDoubleSpinBox *MotionPlanningFrame::createItemSpinBox(QTreeWidgetItem *item) {
  QDoubleSpinBox *spinbox = new QDoubleSpinBox();
  spinbox->setMinimum(-100.0);
  spinbox->setMaximum(100.0);
  spinbox->setValue(0.0);
  spinbox->setDecimals(3);
  spinbox->setSingleStep(0.1);
  
  spinbox->setProperty( "TreeWidgetItem", QVariant::fromValue<void*>(item) ); 
  connect(spinbox, SIGNAL(valueChanged(double)), &cartesian_trajectory_waypoint_change_mapper_, SLOT(map()));
  cartesian_trajectory_waypoint_change_mapper_.setMapping(spinbox, spinbox);
  return spinbox;
}

QTreeWidgetItem *MotionPlanningFrame::addWaypoint(QTreeWidgetItem *copy) {
  QTreeWidgetItem *waypoint_item = new QTreeWidgetItem( QStringList() << tr("New Waypoint") << tr("Time: 1.0") );
  waypoint_item->setFlags(waypoint_item->flags() | Qt::ItemIsEditable);

  QTreeWidgetItem *position_item = new QTreeWidgetItem(waypoint_item, QStringList() << tr("Position") << tr("[0.00; 0.00; 0.00]"));
  QTreeWidgetItem *orientation_item = new QTreeWidgetItem(waypoint_item, QStringList() << tr("Orientation") << tr("[1.00; 0.00; 0.00; 0.00]"));
  QTreeWidgetItem *target_link_name_item = new QTreeWidgetItem(waypoint_item, QStringList() << tr("Target Link Name"));
  QTreeWidgetItem *axis_item = new QTreeWidgetItem(waypoint_item, QStringList() << tr("Link Axis") << tr("[0.00; 1.00; 0.00]"));
  QTreeWidgetItem *time_item = new QTreeWidgetItem(waypoint_item, QStringList() << tr("Time"));

  QTreeWidgetItem *orientation_value_item[4];
  orientation_value_item[0] = new QTreeWidgetItem(orientation_item, QStringList() << tr("W:"));
  orientation_value_item[1] = new QTreeWidgetItem(orientation_item, QStringList() << tr("X:"));
  orientation_value_item[2] = new QTreeWidgetItem(orientation_item, QStringList() << tr("Y:"));
  orientation_value_item[3] = new QTreeWidgetItem(orientation_item, QStringList() << tr("Z:"));

  QTreeWidgetItem *position_value_item[3];
  position_value_item[0] = new QTreeWidgetItem(position_item, QStringList() << tr("X:"));
  position_value_item[1] = new QTreeWidgetItem(position_item, QStringList() << tr("Y:"));
  position_value_item[2] = new QTreeWidgetItem(position_item, QStringList() << tr("Z:"));

  QTreeWidgetItem *axis_value_item[3];
  axis_value_item[0] = new QTreeWidgetItem(axis_item, QStringList() << tr("X:"));
  axis_value_item[1] = new QTreeWidgetItem(axis_item, QStringList() << tr("Y:"));
  axis_value_item[2] = new QTreeWidgetItem(axis_item, QStringList() << tr("Z:"));

  ui_->waypoints_treewidget->addTopLevelItem(waypoint_item);
  QDoubleSpinBox *orientation_spinbox[4];
  for ( int i = 0; i < 4; i++ ) {
      orientation_spinbox[i] = createItemSpinBox(orientation_value_item[i]);
      ui_->waypoints_treewidget->setItemWidget(orientation_value_item[i], 1, orientation_spinbox[i]);
  }

  QDoubleSpinBox *position_spinbox[3];  
  for ( int i = 0; i < 3; i++ ) {
      position_spinbox[i] = createItemSpinBox(position_value_item[i]);
      ui_->waypoints_treewidget->setItemWidget(position_value_item[i], 1, position_spinbox[i]);
  }

  QLineEdit *target_link_name_edit = new QLineEdit("");
  ui_->waypoints_treewidget->setItemWidget(target_link_name_item, 1, target_link_name_edit);

  QDoubleSpinBox *axis_spinbox[3];
  for ( int i = 0; i < 3; i++ ) {
      axis_spinbox[i] = createItemSpinBox(axis_value_item[i]);
      ui_->waypoints_treewidget->setItemWidget(axis_value_item[i], 1, axis_spinbox[i]);
  }

  QDoubleSpinBox *time_spinbox = createItemSpinBox(time_item);
  ui_->waypoints_treewidget->setItemWidget(time_item, 1, time_spinbox);
  
  if ( copy ) {
      QString target_pose_name;
      QString target_link_name;
      geometry_msgs::Pose target_pose;
      geometry_msgs::Point target_link_axis;
      double target_pose_time;

      getContentFromItem(copy, target_pose_name, target_pose, target_link_name, target_link_axis, target_pose_time);
      waypoint_item->setText(0, target_pose_name);

      position_spinbox[0]->setValue( target_pose.position.x);
      position_spinbox[1]->setValue( target_pose.position.y);
      position_spinbox[2]->setValue( target_pose.position.z);

      orientation_spinbox[0]->setValue( target_pose.orientation.w);
      orientation_spinbox[1]->setValue( target_pose.orientation.x);
      orientation_spinbox[2]->setValue( target_pose.orientation.y);
      orientation_spinbox[3]->setValue( target_pose.orientation.z);

      target_link_name_edit->setText(target_link_name);

      axis_spinbox[0]->setValue( target_link_axis.x);
      axis_spinbox[1]->setValue( target_link_axis.y);
      axis_spinbox[2]->setValue( target_link_axis.z);

      time_spinbox->setValue(target_pose_time);


  }

  updateWaypointData( ui_->waypoints_treewidget->itemWidget(orientation_value_item[0], 1) );
  updateWaypointData( ui_->waypoints_treewidget->itemWidget(position_value_item[0], 1) );
  updateWaypointData( ui_->waypoints_treewidget->itemWidget(time_item, 1) );

  return waypoint_item;
}

bool MotionPlanningFrame::getContentFromItem(QTreeWidgetItem *waypoint_item, QString &name, geometry_msgs::Pose &target_pose, QString &target_link_name, geometry_msgs::Point &target_link_axis, double &target_pose_time) {
    if ( ui_->waypoints_treewidget->indexOfTopLevelItem(waypoint_item) == -1)
        return false;

    name = waypoint_item->text(0);

    QTreeWidgetItem *current_item = NULL;
    QLineEdit *current_lineedit = NULL;
    QDoubleSpinBox *current_spinbox = NULL;

    // set position
    current_item = waypoint_item->child(0)->child(0);
    current_spinbox = (QDoubleSpinBox*)(ui_->waypoints_treewidget->itemWidget(current_item, 1));
    target_pose.position.x = current_spinbox->value();

    current_item = waypoint_item->child(0)->child(1);
    current_spinbox = (QDoubleSpinBox*)(ui_->waypoints_treewidget->itemWidget(current_item, 1));
    target_pose.position.y = current_spinbox->value();

    current_item = waypoint_item->child(0)->child(2);
    current_spinbox = (QDoubleSpinBox*)(ui_->waypoints_treewidget->itemWidget(current_item, 1));
    target_pose.position.z = current_spinbox->value();

    // set orientation
    current_item = waypoint_item->child(1)->child(0);
    current_spinbox = (QDoubleSpinBox*)(ui_->waypoints_treewidget->itemWidget(current_item, 1));
    target_pose.orientation.w = current_spinbox->value();

    current_item = waypoint_item->child(1)->child(1);
    current_spinbox = (QDoubleSpinBox*)(ui_->waypoints_treewidget->itemWidget(current_item, 1));
    target_pose.orientation.x = current_spinbox->value();

    current_item = waypoint_item->child(1)->child(2);
    current_spinbox = (QDoubleSpinBox*)(ui_->waypoints_treewidget->itemWidget(current_item, 1));
    target_pose.orientation.y = current_spinbox->value();

    current_item = waypoint_item->child(1)->child(3);
    current_spinbox = (QDoubleSpinBox*)(ui_->waypoints_treewidget->itemWidget(current_item, 1));
    target_pose.orientation.z = current_spinbox->value();

    // set target link name
    current_item = waypoint_item->child(2);
    current_lineedit = (QLineEdit*)(ui_->waypoints_treewidget->itemWidget(current_item, 1));
    target_link_name = current_lineedit->text();

    // set axis
    current_item = waypoint_item->child(3)->child(0);
    current_spinbox = (QDoubleSpinBox*)(ui_->waypoints_treewidget->itemWidget(current_item, 1));
    target_link_axis.x = current_spinbox->value();

    current_item = waypoint_item->child(3)->child(1);
    current_spinbox = (QDoubleSpinBox*)(ui_->waypoints_treewidget->itemWidget(current_item, 1));
    target_link_axis.y = current_spinbox->value();

    current_item = waypoint_item->child(3)->child(2);
    current_spinbox = (QDoubleSpinBox*)(ui_->waypoints_treewidget->itemWidget(current_item, 1));
    target_link_axis.z = current_spinbox->value();

    // set time
    current_item = waypoint_item->child(4);
    current_spinbox = (QDoubleSpinBox*)(ui_->waypoints_treewidget->itemWidget(current_item, 1));
    target_pose_time = current_spinbox->value();

    return true;
}

void MotionPlanningFrame::removeWaypoint(QTreeWidgetItem *item)
{
    // only allow removal of toplevel items
    if ( ui_->waypoints_treewidget->indexOfTopLevelItem(item) == -1)
        return;

    // remove item and children itself
    delete item;
    item = NULL;
}

void MotionPlanningFrame::clearWaypoints()
{
  while ( QTreeWidgetItem *currentItem = ui_->waypoints_treewidget->takeTopLevelItem(0) ) 
  {
    removeWaypoint( currentItem );
  }
}

vigir_planning_msgs::ExtendedPlanningOptionsPtr MotionPlanningFrame::buildExtendedPlanningOptions() {
    vigir_planning_msgs::ExtendedPlanningOptionsPtr extended_options(new vigir_planning_msgs::ExtendedPlanningOptions());
    extended_options->target_motion_type = vigir_planning_msgs::ExtendedPlanningOptions::TYPE_CARTESIAN_WAYPOINTS;
    extended_options->target_frame = "world";
    extended_options->continuous_replanning = false;

    extended_options->check_self_collisions = ui_->check_self_collisions_checkbox->isChecked();
    extended_options->target_orientation_type = cartesian_trajectory_orientation_group_.checkedId();
    extended_options->trajectory_sample_rate = ui_->sample_rate_spin->value();

    for ( int i = 0; i < ui_->waypoints_treewidget->topLevelItemCount(); i++ ) {
      geometry_msgs::Pose current_pose;
      QString target_link_name;
      double waypoint_time;
      QString target_pose_name;
      geometry_msgs::Point target_link_axis;

      getContentFromItem(ui_->waypoints_treewidget->topLevelItem(i), target_pose_name, current_pose, target_link_name, target_link_axis, waypoint_time);

      // add to vector
      extended_options->target_pose_names.push_back(target_pose_name.toStdString());
      extended_options->target_poses.push_back(current_pose);
      extended_options->target_pose_times.push_back(waypoint_time);
      extended_options->target_link_names.push_back(target_link_name.toStdString());
      extended_options->target_link_axis.push_back(target_link_axis);
    }

    return extended_options;
}

void MotionPlanningFrame::computeCartesianPlanButtonClicked()
{
  if (!move_group_)
    return;

  // Clear status
  ui_->cartesian_trajectory_status_label->setText("Planning...");

  configureForPlanning();
  current_plan_.reset(new moveit::planning_interface::VigirMoveGroup::Plan());

  vigir_planning_msgs::ExtendedPlanningOptionsPtr extendedOptions = buildExtendedPlanningOptions();

  if (move_group_->plan(*current_plan_, extendedOptions))
  {
    ui_->execute_button->setEnabled(true);
    ui_->execute_cartesian_trajectory_button->setEnabled(true);

    // Success
    ui_->cartesian_trajectory_status_label->setText(QString("Time: ").append(
        QString::number(current_plan_->planning_time_,'f',3)));
  }
  else
  {
    ui_->execute_cartesian_trajectory_button->setEnabled(true);
    current_plan_.reset();

    // Failure
    ui_->cartesian_trajectory_status_label->setText("Failed");
  }
}

}
