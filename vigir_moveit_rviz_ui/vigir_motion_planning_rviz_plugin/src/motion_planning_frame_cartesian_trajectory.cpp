#include <moveit/vigir_motion_planning_rviz_plugin/motion_planning_frame.h>
#include <moveit/vigir_motion_planning_rviz_plugin/motion_planning_display.h>

#include <vigir_planning_msgs/ExtendedPlanningOptions.h>

#include <QTreeWidgetItem>
#include <QDoubleSpinBox>
#include <QSignalMapper>
#include <QFileDialog>
#include <QDomDocument>

#include "ui_motion_planning_rviz_plugin_frame.h"


namespace vigir_moveit_rviz_plugin
{

void MotionPlanningFrame::initCartesianTrajectoryTab()
{
    cartesian_trajectory_orientation_group_.addButton(ui_->keep_full_orientation_button, vigir_planning_msgs::ExtendedPlanningOptions::ORIENTATION_FULL);
    cartesian_trajectory_orientation_group_.addButton(ui_->use_axis_only_button, vigir_planning_msgs::ExtendedPlanningOptions::ORIENTATION_AXIS_ONLY);
    cartesian_trajectory_orientation_group_.addButton(ui_->ignore_orientation_button, vigir_planning_msgs::ExtendedPlanningOptions::ORIENTATION_IGNORE);

    clearWaypoints();
}

void MotionPlanningFrame::addWaypointButtonClicked()
{
    QList<QTreeWidgetItem*> selectedItems = ui_->waypoints_treewidget->selectedItems();
    if ( selectedItems.size() == 1 ) { // exactly one item => copy it
      // check if item is toplevel-item
      if ( ui_->waypoints_treewidget->indexOfTopLevelItem((selectedItems[0])) != -1)
      {
            addWaypoint(selectedItems[0]);
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
  QTreeWidgetItem *currentItem = ui_->waypoints_treewidget->currentItem();
  
  if ( currentItem == NULL)
      return;

  // find top-level item
  while( currentItem->parent() ) {
    currentItem = currentItem->parent();
  }
  
  removeWaypoint(currentItem); 
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

  vigir_planning_msgs::ExtendedPlanningOptionsPtr extendedPlanningOptions = loadExtendedPlanningOptions(filename);
  clearWaypoints();
  
  QDoubleSpinBox *currentSpinBox = NULL;
  QLineEdit *currentLineEdit = NULL;
  for ( int i = 0; i < extendedPlanningOptions->target_poses.size(); i++ ) {
    QTreeWidgetItem *waypointItem = addWaypoint();
    waypointItem->setText(0, QString::fromStdString(extendedPlanningOptions->target_pose_names[i]));

    // set position values
    currentSpinBox = (QDoubleSpinBox*)(ui_->waypoints_treewidget->itemWidget(waypointItem->child(0)->child(0), 1));
    currentSpinBox->setValue( extendedPlanningOptions->target_poses[i].position.x );
    currentSpinBox = (QDoubleSpinBox*)(ui_->waypoints_treewidget->itemWidget(waypointItem->child(0)->child(1), 1));
    currentSpinBox->setValue( extendedPlanningOptions->target_poses[i].position.y );
    currentSpinBox = (QDoubleSpinBox*)(ui_->waypoints_treewidget->itemWidget(waypointItem->child(0)->child(2), 1));
    currentSpinBox->setValue( extendedPlanningOptions->target_poses[i].position.z );

    // set orientation values
    currentSpinBox = (QDoubleSpinBox*)(ui_->waypoints_treewidget->itemWidget(waypointItem->child(1)->child(0), 1));
    currentSpinBox->setValue( extendedPlanningOptions->target_poses[i].orientation.w );
    currentSpinBox = (QDoubleSpinBox*)(ui_->waypoints_treewidget->itemWidget(waypointItem->child(1)->child(1), 1));
    currentSpinBox->setValue( extendedPlanningOptions->target_poses[i].orientation.x );
    currentSpinBox = (QDoubleSpinBox*)(ui_->waypoints_treewidget->itemWidget(waypointItem->child(1)->child(2), 1));
    currentSpinBox->setValue( extendedPlanningOptions->target_poses[i].orientation.y );
    currentSpinBox = (QDoubleSpinBox*)(ui_->waypoints_treewidget->itemWidget(waypointItem->child(1)->child(3), 1));
    currentSpinBox->setValue( extendedPlanningOptions->target_poses[i].orientation.z );

    // set target link names
    currentLineEdit = (QLineEdit*)(ui_->waypoints_treewidget->itemWidget(waypointItem->child(2), 1));
    currentLineEdit->setText( QString::fromStdString(extendedPlanningOptions->target_link_names[i]));

    // set target pose times
    currentSpinBox = (QDoubleSpinBox*)(ui_->waypoints_treewidget->itemWidget(waypointItem->child(3), 1));
    currentSpinBox->setValue( extendedPlanningOptions->target_pose_times[i]);
  }

  // read orientation mode
  int orientationMode = extendedPlanningOptions->target_orientation_type;
  cartesian_trajectory_orientation_group_.button(orientationMode)->setChecked(true);

  // read sample rate
  ui_->sample_rate_spin->setValue( extendedPlanningOptions->trajectory_sample_rate );

  // read check self-collisions flag
  bool checkSelfCollisions = extendedPlanningOptions->check_self_collisions;
  ui_->check_self_collisions_checkbox->setChecked(checkSelfCollisions);
  
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

    vigir_planning_msgs::ExtendedPlanningOptionsPtr extendedPlanningOptions(new vigir_planning_msgs::ExtendedPlanningOptions());

    // read waypoints
    QDoubleSpinBox *currentSpinBox = NULL;
    QLineEdit *currentLineEdit = NULL;
    QDomNodeList waypointsList = doc.elementsByTagName ( "Waypoint" );
    for ( int i = 0; i < waypointsList.size(); i++ ) {
      QDomNode currentNode = waypointsList.at(i);

      QString targetPoseName = currentNode.toElement().attribute("name");
      extendedPlanningOptions->target_pose_names.push_back(targetPoseName.toStdString() );

      geometry_msgs::Pose targetPose;
      QDomNodeList waypointChildren = currentNode.childNodes();
      QDomElement positionElement = currentNode.namedItem("Position").toElement();
      targetPose.position.x = positionElement.attribute("x").toDouble();
      targetPose.position.y = positionElement.attribute("y").toDouble();
      targetPose.position.z = positionElement.attribute("z").toDouble();

      QDomElement orientationElement = currentNode.namedItem("Orientation").toElement();
      targetPose.orientation.w = orientationElement.attribute("w").toDouble();
      targetPose.orientation.x = orientationElement.attribute("x").toDouble();
      targetPose.orientation.y = orientationElement.attribute("y").toDouble();
      targetPose.orientation.z = orientationElement.attribute("z").toDouble();
      extendedPlanningOptions->target_poses.push_back(targetPose);

      QDomElement targetLinkNameElement = currentNode.namedItem("TargetLinkName").toElement();
      QString targetLinkName = targetLinkNameElement.attribute("value");
      extendedPlanningOptions->target_link_names.push_back(targetLinkName.toStdString());

      QDomElement timeElement = currentNode.namedItem("Time").toElement();
      double targetPoseTime = timeElement.attribute("value").toDouble();
      extendedPlanningOptions->target_pose_times.push_back(targetPoseTime);
    }

    // read orientation mode
    QDomElement orientationElement = doc.elementsByTagName("OrientationMode").at(0).toElement();
    int orientationMode = orientationElement.attribute("value").toInt();
    extendedPlanningOptions->target_orientation_type = orientationMode;

    // read sample rate
    QDomElement sampleRateElement = doc.elementsByTagName("SampleRate").at(0).toElement();
    double sampleRate = sampleRateElement.attribute("value").toDouble();
    extendedPlanningOptions->trajectory_sample_rate = sampleRate;

    // read check self-collisions flag
    QDomElement checkSelfCollisionsElement = doc.elementsByTagName("CheckSelfCollisions").at(0).toElement();
    bool checkSelfCollisions = (checkSelfCollisionsElement.attribute("value").toInt() == 1);
    extendedPlanningOptions->check_self_collisions = checkSelfCollisions;

    return extendedPlanningOptions;
}

void MotionPlanningFrame::saveCartesianTrajectoryButtonClicked()
{
  QString filename = QFileDialog::getSaveFileName(this, QString(), QString(), "XML (*.xml)");
  
  if ( filename.isEmpty() )
    return;

  vigir_planning_msgs::ExtendedPlanningOptionsPtr extendedPlanningOptions = buildExtendedPlanningOptions();
  saveExtendedPlanningOptions(extendedPlanningOptions, filename);
}

bool MotionPlanningFrame::saveExtendedPlanningOptions(vigir_planning_msgs::ExtendedPlanningOptionsPtr extendedPlanningOptions, QString filename)
{
  QDomDocument doc("CartesianTrajectoryOptions");
  QDomElement root = doc.createElement("CartesianTrajectoryOptions");
  doc.appendChild(root);
  
  QDomElement waypoints = doc.createElement("Waypoints");
  root.appendChild(waypoints);

  for ( int i = 0; i < extendedPlanningOptions->target_poses.size(); i++ ) {
    QTreeWidgetItem *rootItem = ui_->waypoints_treewidget->topLevelItem(i);
    geometry_msgs::Pose currentPose = extendedPlanningOptions->target_poses[i];
    std::string currentLinkName = extendedPlanningOptions->target_link_names[i];
    double currentTime = extendedPlanningOptions->target_pose_times[i];

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
  orientationMode.setAttribute("value", extendedPlanningOptions->target_orientation_type);
  options.appendChild(orientationMode);
  
  // set sample rate
  QDomElement sampleRate = doc.createElement("SampleRate");
  sampleRate.setAttribute("value", extendedPlanningOptions->trajectory_sample_rate );
  options.appendChild(sampleRate);
  
  // set self-collision check
  QDomElement checkSelfCollisions = doc.createElement("CheckSelfCollisions");
  checkSelfCollisions.setAttribute("value", extendedPlanningOptions->check_self_collisions );
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
    planning_display_->addBackgroundJob(boost::bind(&MotionPlanningFrame::computeCartesianPlanButtonClicked, this), "compute cartesian plan");
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

  ui_->waypoints_treewidget->addTopLevelItem(waypoint_item);
  QDoubleSpinBox *orientationSpinBox[4];
  for ( int i = 0; i < 4; i++ ) {
      orientationSpinBox[i] = createItemSpinBox(orientation_value_item[i]);
      ui_->waypoints_treewidget->setItemWidget(orientation_value_item[i], 1, orientationSpinBox[i]);
  }

  QDoubleSpinBox *positionSpinBox[3];
  for ( int i = 0; i < 3; i++ ) {
      positionSpinBox[i] = createItemSpinBox(position_value_item[i]);
      ui_->waypoints_treewidget->setItemWidget(position_value_item[i], 1, positionSpinBox[i]);
  }

  QLineEdit *target_link_name_edit = new QLineEdit("");
  ui_->waypoints_treewidget->setItemWidget(target_link_name_item, 1, target_link_name_edit);

  QDoubleSpinBox *timeSpinBox = createItemSpinBox(time_item);
  ui_->waypoints_treewidget->setItemWidget(time_item, 1, timeSpinBox);
  
  if ( copy ) {
      QString targetPoseName;
      QString targetLinkName;
      geometry_msgs::Pose targetPose;
      double targetPoseTime;

      getContentFromItem(copy, targetPoseName, targetPose, targetLinkName, targetPoseTime);
      waypoint_item->setText(0, targetPoseName);

      positionSpinBox[0]->setValue( targetPose.position.x);
      positionSpinBox[1]->setValue( targetPose.position.y);
      positionSpinBox[2]->setValue( targetPose.position.z);

      orientationSpinBox[0]->setValue( targetPose.orientation.w);
      orientationSpinBox[1]->setValue( targetPose.orientation.x);
      orientationSpinBox[2]->setValue( targetPose.orientation.y);
      orientationSpinBox[3]->setValue( targetPose.orientation.z);

      target_link_name_edit->setText(targetLinkName);
      timeSpinBox->setValue(targetPoseTime);


  }

  updateWaypointData( ui_->waypoints_treewidget->itemWidget(orientation_value_item[0], 1) );
  updateWaypointData( ui_->waypoints_treewidget->itemWidget(position_value_item[0], 1) );
  updateWaypointData( ui_->waypoints_treewidget->itemWidget(time_item, 1) );

  return waypoint_item;
}

bool MotionPlanningFrame::getContentFromItem(QTreeWidgetItem *waypointItem, QString &name, geometry_msgs::Pose &targetPose, QString &targetLinkName, double &targetPoseTime) {
    if ( ui_->waypoints_treewidget->indexOfTopLevelItem(waypointItem) == -1)
        return false;

    name = waypointItem->text(0);

    QTreeWidgetItem *currentItem = NULL;
    QLineEdit *currentLineEdit = NULL;
    QDoubleSpinBox *currentSpinBox = NULL;

    // set position
    currentItem = waypointItem->child(0)->child(0);
    currentSpinBox = (QDoubleSpinBox*)(ui_->waypoints_treewidget->itemWidget(currentItem, 1));
    targetPose.position.x = currentSpinBox->value();

    currentItem = waypointItem->child(0)->child(1);
    currentSpinBox = (QDoubleSpinBox*)(ui_->waypoints_treewidget->itemWidget(currentItem, 1));
    targetPose.position.y = currentSpinBox->value();

    currentItem = waypointItem->child(0)->child(2);
    currentSpinBox = (QDoubleSpinBox*)(ui_->waypoints_treewidget->itemWidget(currentItem, 1));
    targetPose.position.z = currentSpinBox->value();

    // set orientation
    currentItem = waypointItem->child(1)->child(0);
    currentSpinBox = (QDoubleSpinBox*)(ui_->waypoints_treewidget->itemWidget(currentItem, 1));
    targetPose.orientation.w = currentSpinBox->value();

    currentItem = waypointItem->child(1)->child(1);
    currentSpinBox = (QDoubleSpinBox*)(ui_->waypoints_treewidget->itemWidget(currentItem, 1));
    targetPose.orientation.x = currentSpinBox->value();

    currentItem = waypointItem->child(1)->child(2);
    currentSpinBox = (QDoubleSpinBox*)(ui_->waypoints_treewidget->itemWidget(currentItem, 1));
    targetPose.orientation.y = currentSpinBox->value();

    currentItem = waypointItem->child(1)->child(3);
    currentSpinBox = (QDoubleSpinBox*)(ui_->waypoints_treewidget->itemWidget(currentItem, 1));
    targetPose.orientation.z = currentSpinBox->value();

    // set target link name
    currentItem = waypointItem->child(2);
    currentLineEdit = (QLineEdit*)(ui_->waypoints_treewidget->itemWidget(currentItem, 1));
    targetLinkName = currentLineEdit->text();

    // set time
    currentItem = waypointItem->child(3);
    currentSpinBox = (QDoubleSpinBox*)(ui_->waypoints_treewidget->itemWidget(currentItem, 1));
    targetPoseTime = currentSpinBox->value();

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
    vigir_planning_msgs::ExtendedPlanningOptionsPtr extendedOptions(new vigir_planning_msgs::ExtendedPlanningOptions());
    extendedOptions->target_motion_type = vigir_planning_msgs::ExtendedPlanningOptions::TYPE_CARTESIAN_MOTION;
    extendedOptions->target_frame = "world";
    extendedOptions->continuous_replanning = false;

    extendedOptions->check_self_collisions = ui_->check_self_collisions_checkbox->isChecked();
    extendedOptions->target_orientation_type = cartesian_trajectory_orientation_group_.checkedId();
    extendedOptions->trajectory_sample_rate = ui_->sample_rate_spin->value();

    for ( int i = 0; i < ui_->waypoints_treewidget->topLevelItemCount(); i++ ) {
      geometry_msgs::Pose currentPose;
      QString targetLinkName;
      double waypointTime;
      QString targetPoseName;

      getContentFromItem(ui_->waypoints_treewidget->topLevelItem(i), targetPoseName, currentPose, targetLinkName, waypointTime);

      // add to vector
      extendedOptions->target_pose_names.push_back(targetPoseName.toStdString());
      extendedOptions->target_poses.push_back(currentPose);
      extendedOptions->target_pose_times.push_back(waypointTime);
      extendedOptions->target_link_names.push_back(targetLinkName.toStdString());
    }

    return extendedOptions;
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

    // Success
    ui_->cartesian_trajectory_status_label->setText(QString("Time: ").append(
        QString::number(current_plan_->planning_time_,'f',3)));
  }
  else
  {
    current_plan_.reset();

    // Failure
    ui_->cartesian_trajectory_status_label->setText("Failed");
  }
}

}
