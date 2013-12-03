#include "databasebinding.h"
#include <ros/ros.h>
#include <database_interface/postgresql_database.h>
#include <string>
#include <database_interface/db_class.h>


int main(int argc, char **argv)
{
  ROS_INFO("Database_binding starting");
  ROS_DEBUG("This is a debug-message");
  ros::init(argc, argv, "database_binding");
  ros::NodeHandle n;
  std::string host = "192.168.10.100";
  std::string port = "5432";
  std::string user = "turtlebot";
  std::string passwd = "";
  std::string db = "";

 // database_interface::PostgresqlDatabase database(host);
  database_interface::PostgresqlDatabase database (host,port,user,passwd, db);
  ROS_INFO("Connection established");
  //database_interface::PostgresqlDatabase database("192.168.10.100", "5432", "turtlebot", "rosdb");
  if (!database.isConnected())

  {
      ROS_INFO("Database failed to connect \n");
    return -1;
  }
  ROS_INFO("Database connected successfully \n");
  return 0;

}


databaseBinding::databaseBinding()
{
}
