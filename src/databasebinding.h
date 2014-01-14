#ifndef DATABASEBINDING_H
#define DATABASEBINDING_H
//#include <database_interface/db_class.h>
#include <database_interface/postgresql_database.h>
#include <ros/ros.h>
#include <ros/wall_timer.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <database_binding/formPlaces.h>
#include <database_binding/returnPlaces.h>

/*! This struct stores a position and a timestamp. 
 *  Here, the information is provided by AMCL
 */
struct pos {
  double x;
  double y;
  double z;
  ros::Time rosTime;
};

//was class databaseBinding : public database_interface::DBClass
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
  //! A database object. It is public to avoid get and set methods. It will be made private in the later process //TODO Make PostgresqlDatabase private
  database_interface::PostgresqlDatabase* database_;
  
  //! Initializes the setup
  databaseBinding();
  
  //! Closes the connection
  ~databaseBinding();
  
  //! It is a loop function which is called after the initialization.
  int run();
  
  //! Returns, if we have a connection.
  bool getConnection();
};

#endif // DATABASEBINDING_H
