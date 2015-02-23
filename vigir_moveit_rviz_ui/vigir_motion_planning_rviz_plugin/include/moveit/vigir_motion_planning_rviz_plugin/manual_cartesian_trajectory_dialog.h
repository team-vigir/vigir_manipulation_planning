#include <QDialog>

class QTreeWidgetItem;
class QDoubleSpinBox;
class QSignalMapper;

namespace Ui
{
class CartesianTrajectoryUI;
}

namespace vigir_moveit_rviz_plugin
{
  
class ManualCartesianTrajectoryDialog : public QDialog
{
  friend class CartesianTrajectoryUI;
  Q_OBJECT
  
public:
  
  ManualCartesianTrajectoryDialog(QWidget *parent = NULL, Qt::WindowFlags flags = 0);
  ~ManualCartesianTrajectoryDialog();
  
private Q_SLOTS:
  void addWaypointButtonClicked();
  void removeWaypointButtonClicked();
  void clearWaypointsButtonClicked();
  void loadButtonClicked();
  void saveButtonClicked();
  void planButtonClicked();
  
  void updateWaypointData(QWidget *spinbox);    
    
private:
  QTreeWidgetItem *addWaypoint();
  void removeWaypoint(QTreeWidgetItem *item);
  void clearWaypoints();
  
  QDoubleSpinBox *createItemSpinBox(QTreeWidgetItem *item);
  
  QSignalMapper *value_change_mapper_;
  Ui::CartesianTrajectoryUI *ui_;
};

}