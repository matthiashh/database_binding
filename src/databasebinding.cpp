#include "databasebinding.h"
#include <ros/ros.h>
#include <database_interface/db_class.h>
//#include <database_interface/postgresql_database.h>
//#include <database_interface/postgresql_database_interface.h>
#include <string>

//#include <postgresql/postgres.h>

void test (std::string teststring, std::string teststring2)
{
  ROS_INFO("Teststring received");
}

int main(int argc, char **argv)
{
  ros::NodeHandle n;
  ros::init(argc, argv, "database_binding");
  std::string host = "192.168.10.100";
  std::string port = "5432";
  std::string user = "turtlebot";
  std::string passwd = "";
  std::string db = "rosdb";


 // database_interface::PostgresqlDatabase database(host);
  //database_interface::PostgresqlDatabase database (host,port,user,passwd,db);
  //database_interface::PostgresqlDatabaseInterface database("192.168.10.100", "5432", "turtlebot", "test", "rosdb");
//  if (!database.isConnected())
//  {
//      ROS_INFO("Database failed to connect \n");
//    return -1;
//  }
//  ROS_INFO("Database connected successfully \n");
  return 0;
}


databaseBinding::databaseBinding()
{
}
