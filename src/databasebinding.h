#ifndef DATABASEBINDING_H
#define DATABASEBINDING_H
//#include <database_interface/db_class.h>
#include <database_interface/postgresql_database.h>
#include <ros/ros.h>
#include <ros/wall_timer.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <database_binding/formPlaces.h>
#include <database_binding/returnPlaces.h>

struct pos {
  double x;
  double y;
  double z;
  ros::Time rosTime;
};

class databaseBinding
{
private:
  ros::NodeHandle n_;
  ros::Subscriber position_;
  ros::Publisher explorationGoalPub_;
  std::vector<database_interface::Notification> notifyQueue_;
  pos latPos_;
  int robot_id_;

  void positionCallback(const geometry_msgs::PoseWithCovarianceStamped pos);
  void NotifyCallback();

  bool getPlaces();
  bool NotifyThread();
  bool PlacesThread();
public:
  database_interface::PostgresqlDatabase* database_;
  database_interface::PostgresqlDatabase* databaseListen_;
  databaseBinding();
  ~databaseBinding();
  int run();
  bool getConnection();
};

#endif // DATABASEBINDING_H
