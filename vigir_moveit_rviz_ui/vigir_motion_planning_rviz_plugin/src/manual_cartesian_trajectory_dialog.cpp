#ifndef VIGIR_MOVEIT_MOTION_PLANNING_RVIZ_PLUGIN_MANUAL_CARTESIAN_TRAJECTORY_DIALOG_
#define VIGIR_MOVEIT_MOTION_PLANNING_RVIZ_PLUGIN_MANUAL_CARTESIAN_TRAJECTORY_DIALOG_

#include <moveit/vigir_motion_planning_rviz_plugin/manual_cartesian_trajectory_dialog.h>

#include <QTreeWidgetItem>
#include <QDoubleSpinBox>
#include <QSignalMapper>
#include <QFileDialog>
#include <QDomDocument>

#include "ui_manual_cartesian_trajectory_dialog.h"

namespace vigir_moveit_rviz_plugin
{
  
ManualCartesianTrajectoryDialog::ManualCartesianTrajectoryDialog(QWidget *parent, Qt::WindowFlags flags) : 
  QDialog(parent, flags),
  ui_(new Ui::CartesianTrajectoryUI())

{
  // set up the GUI
  ui_->setupUi(this);
    
  value_change_mapper_ = new QSignalMapper(this);
  
  clearWaypoints();
  
  
  // setup connections 
  connect(ui_->add_waypoint_button, SIGNAL(clicked()), this, SLOT(addWaypointButtonClicked()));
  connect(ui_->remove_waypoint_button, SIGNAL(clicked()), this, SLOT(removeWaypointButtonClicked()));
  connect(ui_->clear_waypoints_button, SIGNAL(clicked()), this, SLOT(clearWaypointsButtonClicked()));
  connect(ui_->load_button, SIGNAL(clicked()), this, SLOT(loadButtonClicked()));
  connect(ui_->save_button, SIGNAL(clicked()), this, SLOT(saveButtonClicked()));
  connect(ui_->plan_button, SIGNAL(clicked()), this, SLOT(planButtonClicked()));
  connect(ui_->exit_button, SIGNAL(clicked()), this, SLOT(close()));
  
  connect(value_change_mapper_, SIGNAL(mapped(QWidget*)), this, SLOT(updateWaypointData(QWidget*)));
  
}

ManualCartesianTrajectoryDialog::~ManualCartesianTrajectoryDialog() 
{
}

void ManualCartesianTrajectoryDialog::addWaypointButtonClicked() 
{
  addWaypoint();
}

void ManualCartesianTrajectoryDialog::removeWaypointButtonClicked() 
{
  QTreeWidgetItem *currentItem = ui_->waypoints_treewidget->currentItem();
  
  // find top-level item
  while( currentItem->parent() ) {
    currentItem = currentItem->parent();
  }
  
  removeWaypoint(currentItem); 
}

void ManualCartesianTrajectoryDialog::clearWaypointsButtonClicked()
{
  clearWaypoints();
}

void ManualCartesianTrajectoryDialog::loadButtonClicked()
{
  QString filename = QFileDialog::getOpenFileName(this, QString(), QString(), "XML (*.xml)");
  
  if ( filename.isEmpty() )
    return;
  
  clearWaypoints();
  
  QDomDocument doc("CartesianTrajectoryOptions");
  QFile file(filename);
  if (!file.open(QIODevice::ReadOnly)) {
    qWarning("Could not open file for reading");
    return;
  }
  
  if (!doc.setContent(&file)) {
      qWarning("Could not read xml content from file");
     file.close();
     return;
  }
  
  // read waypoints
  double currentValue = 0.0;
  QDoubleSpinBox *currentSpinBox = NULL;  
  QDomNodeList waypointsList = doc.elementsByTagName ( "Waypoint" );
  for ( int i = 0; i < waypointsList.size(); i++ ) {
    QDomNode currentNode = waypointsList.at(i);
    
    QTreeWidgetItem *waypointItem = addWaypoint();    
    waypointItem->setText(0, currentNode.toElement().attribute("name"));
    
    QDomNodeList waypointChildren = currentNode.childNodes();
    
    QDomElement positionElement = currentNode.namedItem("Position").toElement();    
    currentSpinBox = (QDoubleSpinBox*)(ui_->waypoints_treewidget->itemWidget(waypointItem->child(0)->child(0), 1));
    currentSpinBox->setValue( positionElement.attribute("x").toDouble() );
    currentSpinBox = (QDoubleSpinBox*)(ui_->waypoints_treewidget->itemWidget(waypointItem->child(0)->child(1), 1));
    currentSpinBox->setValue( positionElement.attribute("y").toDouble() );
    currentSpinBox = (QDoubleSpinBox*)(ui_->waypoints_treewidget->itemWidget(waypointItem->child(0)->child(2), 1));
    currentSpinBox->setValue( positionElement.attribute("z").toDouble() );
    
    QDomElement orientationElement = currentNode.namedItem("Orientation").toElement();
    currentSpinBox = (QDoubleSpinBox*)(ui_->waypoints_treewidget->itemWidget(waypointItem->child(1)->child(0), 1));
    currentSpinBox->setValue( orientationElement.attribute("w").toDouble() );
    currentSpinBox = (QDoubleSpinBox*)(ui_->waypoints_treewidget->itemWidget(waypointItem->child(1)->child(1), 1));
    currentSpinBox->setValue( orientationElement.attribute("x").toDouble() );
    currentSpinBox = (QDoubleSpinBox*)(ui_->waypoints_treewidget->itemWidget(waypointItem->child(1)->child(2), 1));
    currentSpinBox->setValue( orientationElement.attribute("y").toDouble() );
    currentSpinBox = (QDoubleSpinBox*)(ui_->waypoints_treewidget->itemWidget(waypointItem->child(1)->child(3), 1));
    currentSpinBox->setValue( orientationElement.attribute("z").toDouble() );
    
    QDomElement timeElement = currentNode.namedItem("Time").toElement();
    currentSpinBox = (QDoubleSpinBox*)(ui_->waypoints_treewidget->itemWidget(waypointItem->child(2), 1));
    currentSpinBox->setValue( timeElement.attribute("value").toDouble() );   
  }
  
  // read orientation mode
  QDomElement orientationElement = doc.elementsByTagName("OrientationMode").at(0).toElement();
  int orientationMode = orientationElement.attribute("value").toInt();
  switch (orientationMode) {
    case 0:
      ui_->keep_full_orientation_button->setChecked(true);
      break;
      
    case 1:
      ui_->use_axis_only_button->setChecked(true);
      break;
      
    default:
      ui_->ignore_orientation_button->setChecked(true);
      break;
  }
  
  // read sample rate
  QDomElement sampleRateElement = doc.elementsByTagName("SampleRate").at(0).toElement();
  ui_->sample_rate_spin->setValue( sampleRateElement.attribute("value").toDouble() );
  
  // read target link name
  QDomElement targetLinkNameElement = doc.elementsByTagName("TargetLinkName").at(0).toElement();
  ui_->target_link_name_edit->setText(targetLinkNameElement.attribute("value"));
    
  // read check self-collisions flag
  QDomElement checkSelfCollisionsElement = doc.elementsByTagName("CheckSelfCollisions").at(0).toElement();
  bool checkSelfCollisions = (checkSelfCollisionsElement.attribute("value").toInt() == 1);
  ui_->check_self_collisions_checkbox->setChecked(checkSelfCollisions);  
  
}

void ManualCartesianTrajectoryDialog::saveButtonClicked()
{
  QString filename = QFileDialog::getSaveFileName(this, QString(), QString(), "XML (*.xml)");
  
  if ( filename.isEmpty() )
    return;
  
  QDomDocument doc("CartesianTrajectoryOptions");
  QDomElement root = doc.createElement("CartesianTrajectoryOptions");
  doc.appendChild(root);
  
  QDomElement waypoints = doc.createElement("Waypoints");
  root.appendChild(waypoints);
  
  QTreeWidgetItem *currentItem = NULL;
  QDoubleSpinBox *currentSpinBox = NULL;
  for ( int i = 0; i < ui_->waypoints_treewidget->topLevelItemCount(); i++ ) {
    QTreeWidgetItem *rootItem = ui_->waypoints_treewidget->topLevelItem(i);
    QDomElement currentWaypoint = doc.createElement("Waypoint");
    currentWaypoint.setAttribute("name", rootItem->text(0) );
    waypoints.appendChild(currentWaypoint);    

    // set position
    QDomElement currentPosition = doc.createElement("Position");
    currentItem = rootItem->child(0)->child(0);
    currentSpinBox = (QDoubleSpinBox*)(ui_->waypoints_treewidget->itemWidget(currentItem, 1));
    currentPosition.setAttribute("x", currentSpinBox->value());
    
    currentItem = rootItem->child(0)->child(1);
    currentSpinBox = (QDoubleSpinBox*)(ui_->waypoints_treewidget->itemWidget(currentItem, 1));
    currentPosition.setAttribute("y", currentSpinBox->value());
    
    currentItem = rootItem->child(0)->child(2);
    currentSpinBox = (QDoubleSpinBox*)(ui_->waypoints_treewidget->itemWidget(currentItem, 1));
    currentPosition.setAttribute("z", currentSpinBox->value());
    currentWaypoint.appendChild(currentPosition);
    
    // set orientation
    QDomElement currentOrientation = doc.createElement("Orientation");
    currentItem = rootItem->child(1)->child(0);
    currentSpinBox = (QDoubleSpinBox*)(ui_->waypoints_treewidget->itemWidget(currentItem, 1));
    currentOrientation.setAttribute("w", currentSpinBox->value());
    
    currentItem = rootItem->child(1)->child(1);
    currentSpinBox = (QDoubleSpinBox*)(ui_->waypoints_treewidget->itemWidget(currentItem, 1));
    currentOrientation.setAttribute("x", currentSpinBox->value());
    
    currentItem = rootItem->child(1)->child(2);
    currentSpinBox = (QDoubleSpinBox*)(ui_->waypoints_treewidget->itemWidget(currentItem, 1));
    currentOrientation.setAttribute("y", currentSpinBox->value());    
    
    currentItem = rootItem->child(1)->child(3);
    currentSpinBox = (QDoubleSpinBox*)(ui_->waypoints_treewidget->itemWidget(currentItem, 1));
    currentOrientation.setAttribute("z", currentSpinBox->value());
    currentWaypoint.appendChild(currentOrientation);
    
    // set time
    QDomElement currentTime = doc.createElement("Time");
    currentItem = rootItem->child(2);
    currentSpinBox = (QDoubleSpinBox*)(ui_->waypoints_treewidget->itemWidget(currentItem, 1));
    currentTime.setAttribute("value", currentSpinBox->value());   
    currentWaypoint.appendChild(currentTime);
  }
  
  // set other options
  QDomElement options = doc.createElement("Options");
  root.appendChild(options);
  
  // set orientation mode
  QDomElement orientationMode = doc.createElement("OrientationMode");
  if ( ui_->keep_full_orientation_button->isChecked() )
    orientationMode.setAttribute("value", 0);
  else if ( ui_->use_axis_only_button->isChecked() )
    orientationMode.setAttribute("value", 1);
  else
    orientationMode.setAttribute("value", 2);
  options.appendChild(orientationMode);
  
  // set sample rate
  QDomElement sampleRate = doc.createElement("SampleRate");
  sampleRate.setAttribute("value", ui_->sample_rate_spin->value() );
  options.appendChild(sampleRate);
  
  // set target link name
  QDomElement targetLinkName = doc.createElement("TargetLinkName");
  targetLinkName.setAttribute("value", ui_->target_link_name_edit->text() );
  options.appendChild(targetLinkName);
  
  // set self-collision check
  QDomElement checkSelfCollisions = doc.createElement("CheckSelfCollisions");
  checkSelfCollisions.setAttribute("value", ui_->check_self_collisions_checkbox->isChecked() );
  options.appendChild(checkSelfCollisions);  
  
  // save to file
  QFile file(filename);
  if (!file.open(QIODevice::Truncate | QIODevice::WriteOnly)) {
    qWarning("Could not open save file...");
    return;
  }
  QByteArray xml = doc.toByteArray();
  file.write(xml);
  file.close();
  
}

void ManualCartesianTrajectoryDialog::planButtonClicked()
{
}
  
 
void ManualCartesianTrajectoryDialog::updateWaypointData(QWidget *widget)
{
  QDoubleSpinBox *spinBox = dynamic_cast<QDoubleSpinBox*>(widget);
  if ( spinBox == NULL ) {
    qWarning("ManualCartesianTrajectoryDialog::updateWaypointData(): widget is not q QDoubleSpinBox - aborting");
    return;
  }
    
  
  QTreeWidgetItem *item = (QTreeWidgetItem*)spinBox->property("TreeWidgetItem").value<void*>();
  if ( item == NULL ) {
    qWarning("ManualCartesianTrajectoryDialog::updateWaypointData(): no valid item received - aborting");
    return;
  }
  
  QTreeWidgetItem *parent = item->parent();
  if ( parent == NULL ) {
    qWarning("ManualCartesianTrajectoryDialog::updateWaypointData(): item has no parent - aborting");
    return;
  }
  
  QStringList itemText;
  itemText << "[ ";
  for ( int i = 0; i < parent->childCount(); i++ ) {
    QTreeWidgetItem *child = parent->child(i);
    QDoubleSpinBox *child_spin = (QDoubleSpinBox*)ui_->waypoints_treewidget->itemWidget(child, 1);
    if ( child_spin == NULL )
      continue;
    
    itemText << QString("%1").arg(child_spin->value(), 0, 'f', 3) << "; ";
  }
  
  itemText.removeLast();
  itemText << " ]";

  parent->setText(1, itemText.join(""));
}

QDoubleSpinBox *ManualCartesianTrajectoryDialog::createItemSpinBox(QTreeWidgetItem *item) {
  QDoubleSpinBox *spinbox = new QDoubleSpinBox();
  spinbox->setMinimum(-100.0);
  spinbox->setMaximum(100.0);
  spinbox->setValue(0.0);
  spinbox->setDecimals(3);
  spinbox->setSingleStep(0.1);
  
  spinbox->setProperty( "TreeWidgetItem", QVariant::fromValue<void*>(item) ); 
  connect(spinbox, SIGNAL(valueChanged(double)), value_change_mapper_, SLOT(map()));
  value_change_mapper_->setMapping(spinbox, spinbox);
  return spinbox;
}

QTreeWidgetItem *ManualCartesianTrajectoryDialog::addWaypoint() {
  QTreeWidgetItem *waypoint_item = new QTreeWidgetItem( QStringList() << tr("New Waypoint") << tr("Time: 1.0") );
  waypoint_item->setFlags(waypoint_item->flags() | Qt::ItemIsEditable);
  
  QTreeWidgetItem *position_item = new QTreeWidgetItem(waypoint_item, QStringList() << tr("Position") << tr("[0.00; 0.00; 0.00]"));
  QTreeWidgetItem *orientation_item = new QTreeWidgetItem(waypoint_item, QStringList() << tr("Orientation") << tr("[1.00; 0.00; 0.00; 0.00]"));  
  QTreeWidgetItem *time_item = new QTreeWidgetItem(waypoint_item, QStringList() << tr("Time"));
  
  QTreeWidgetItem *orientation_w_item = new QTreeWidgetItem(orientation_item, QStringList() << tr("W:"));
  QTreeWidgetItem *orientation_x_item = new QTreeWidgetItem(orientation_item, QStringList() << tr("X:"));
  QTreeWidgetItem *orientation_y_item = new QTreeWidgetItem(orientation_item, QStringList() << tr("Y:"));
  QTreeWidgetItem *orientation_z_item = new QTreeWidgetItem(orientation_item, QStringList() << tr("Z:"));
  
  QTreeWidgetItem *position_x_item = new QTreeWidgetItem(position_item, QStringList() << tr("X:"));
  QTreeWidgetItem *position_y_item = new QTreeWidgetItem(position_item, QStringList() << tr("Y:"));
  QTreeWidgetItem *position_z_item = new QTreeWidgetItem(position_item, QStringList() << tr("Z:"));  
  
  ui_->waypoints_treewidget->addTopLevelItem(waypoint_item);
  ui_->waypoints_treewidget->setItemWidget(orientation_w_item, 1, createItemSpinBox(orientation_w_item));
  ui_->waypoints_treewidget->setItemWidget(orientation_x_item, 1, createItemSpinBox(orientation_x_item));
  ui_->waypoints_treewidget->setItemWidget(orientation_y_item, 1, createItemSpinBox(orientation_y_item));
  ui_->waypoints_treewidget->setItemWidget(orientation_z_item, 1, createItemSpinBox(orientation_z_item));
  
  ui_->waypoints_treewidget->setItemWidget(position_x_item, 1, createItemSpinBox(position_x_item));
  ui_->waypoints_treewidget->setItemWidget(position_y_item, 1, createItemSpinBox(position_y_item));
  ui_->waypoints_treewidget->setItemWidget(position_z_item, 1, createItemSpinBox(position_z_item));
  
  ui_->waypoints_treewidget->setItemWidget(time_item, 1, createItemSpinBox(time_item));
  
  updateWaypointData( ui_->waypoints_treewidget->itemWidget(orientation_w_item, 1) );
  updateWaypointData( ui_->waypoints_treewidget->itemWidget(position_x_item, 1) );
  updateWaypointData( ui_->waypoints_treewidget->itemWidget(time_item, 1) );
  
  return waypoint_item;
}

void ManualCartesianTrajectoryDialog::removeWaypoint(QTreeWidgetItem *item) 
{
  // TODO: go through children and remove children and widgets
  
  // remove item itself
  int itemIndex = ui_->waypoints_treewidget->indexOfTopLevelItem(item);
  ui_->waypoints_treewidget->takeTopLevelItem(itemIndex);
  delete item;
  item = NULL;
}

void ManualCartesianTrajectoryDialog::clearWaypoints() 
{
  while ( QTreeWidgetItem *currentItem = ui_->waypoints_treewidget->takeTopLevelItem(0) ) 
  {
    removeWaypoint( currentItem );
  }
}

}

#endif