#ifndef DATABASEBINDING_H
#define DATABASEBINDING_H
//#include <database_interface/db_class.h>
#include <database_interface/postgresql_database.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

/*! This struct stores a position and a timestamp. 
 *  Here, the information is provided by AMCL
 */

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
  database_interface::Notification no_;
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
