#ifndef DATABASEBINDING_H
#define DATABASEBINDING_H
//#include <database_interface/db_class.h>
#include <database_interface/postgresql_database.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <database_binding/formPlaces.h>

struct pos {
  double x;
  double y;
  double z;
  ros::Time latest;
};

//was class databaseBinding : public database_interface::DBClass
class databaseBinding
{
private:
  ros::NodeHandle n_;
  ros::Subscriber position_;
  void positionCallback(const geometry_msgs::PoseWithCovarianceStamped pos);
  pos latPos_;
  database_interface::notification no_;
public:
  databaseBinding();
  ~databaseBinding();
  int run();
  bool getConnection();
  database_interface::PostgresqlDatabase* database_;
};

#endif // DATABASEBINDING_H
