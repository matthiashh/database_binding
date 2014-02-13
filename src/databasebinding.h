#ifndef DATABASEBINDING_H
#define DATABASEBINDING_H
#include <database_interface/postgresql_database.h>
#include <ros/ros.h>
#include <ros/wall_timer.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <database_binding/formPlaces.h>
#include <database_binding/returnPlaces.h>
#include <database_binding/returnChannels.h>
#include <database_binding/returnTasks.h>
#include <database_binding/returnConfiguration.h>
#include <database_binding/DatabaseConnection.h>

class databaseBinding
{
private:
  ros::NodeHandle n_;
  //! Subscriber to position updates of AMCL
  ros::Subscriber position_;
  //! Publisher for received exploration goals
  ros::Publisher explorationGoalPub_;
  //! Publisher for the database connection quality
  ros::Publisher connection_pub_;
  std::vector<database_interface::Notification> notifyQueue_;
  geometry_msgs::PoseWithCovarianceStamped latPos_;
  database_binding::DatabaseConnection connection_state;
  //! This ID is used to identify the robot to the database
  int robot_id_;

  void positionCallback(const geometry_msgs::PoseWithCovarianceStamped pos);

  bool getConfiguration();
  bool getPlaces();
  bool getUpdate();
  //! This is the function for the thread which receives the notifies.
  bool NotifyThread();
  //! This is the function which is called, if a notify is received.
  void NotifyCallback();
  //! The is the function for the places thread. This thread periodically forwards the position to the database.
  void PlacesThread(const ros::TimerEvent &event);
public:
  //! A database object. It is public to avoid get and set methods. It will be made private in the later process //TODO Make PostgresqlDatabase private
  database_interface::PostgresqlDatabase* database_;

  //! An extra database connection to perform the listen task
  //database_interface::PostgresqlDatabase* databaseListen_;
  
  //! Initializes the setup
  databaseBinding();
  
  //! Closes the connection
  ~databaseBinding();
  
  //! It is a loop function which is called after the initialization.
  int run();
  
  //! Returns, if we have a connection.
  bool getConnection();

  //! Publishes information about our connection quality
  void ConnectionPublish_(const ros::TimerEvent &event);
};

#endif // DATABASEBINDING_H
