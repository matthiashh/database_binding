#include "databasebinding.h"
#include <ros/ros.h>
#include <database_interface/postgresql_database.h>   //needed for database object
#include <string>
#include <database_interface/db_class.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>  //msg needed to retrieve the latest position
#include <boost/shared_ptr.hpp>                       //needed for database handling


int main(int argc, char **argv)
{
  ROS_INFO("Database_binding-node starting");
  ROS_DEBUG("This is a debug-message");
  ros::init(argc, argv, "database_binding");

  //creating one database element
  databaseBinding sql_connection;

  if (sql_connection.getConnection())
    {
      ROS_INFO("Connection established");
    } else {
      ROS_INFO("Connection NOT established");
    }
  sql_connection.database_->listenToChannel("bla");

   //run endless
  sql_connection.run();
  return 0;
}

void databaseBinding::positionCallback(const geometry_msgs::PoseWithCovarianceStamped pos)
{
  //transform data to the position struct
  latPos_.latest.sec = pos.header.stamp.toSec();
  latPos_.x = pos.pose.pose.position.x;
  latPos_.y = pos.pose.pose.position.y;
  latPos_.z = pos.pose.pose.position.z;
}

databaseBinding::databaseBinding()
{
  ROS_INFO("DatabaseBinding built");
  //TO-DO: read configuration from yaml-file
  //Initializing the database
  std::string host = "192.168.10.100";
  std::string port = "5432";
  std::string user = "turtlebot";
  std::string passwd = "";
  std::string db = "rosdb";
  ROS_INFO("Trying to connect with host %s, port %s, user %s, passwd %s, db %s",host.c_str(), port.c_str(),user.c_str(),passwd.c_str(),db.c_str());
  this->database_ = new database_interface::PostgresqlDatabase (host,port,user,passwd,db);

  //Initialize subscribers
  position_ = n_.subscribe("/amcl_pose", 10, &databaseBinding::positionCallback, this);
}

databaseBinding::~databaseBinding()
{
  ROS_INFO("DatabaseBinding destroyed");
}

int databaseBinding::run()
{
  ros::Rate r(5);
  while (ros::ok())
    {
      //Do crazy stuff
      if (database_->checkNotify(no_))
        {
          getPlaces();
        }
      ros::spinOnce();
      r.sleep();
    }
  return 0;
}

bool databaseBinding::getConnection()
{
  if (database_->isConnected())
    {
      return 1;
    }
  else {
      return 0;
    }
}

bool databaseBinding::getPlaces()
{
  std::vector< boost::shared_ptr<returnPlaces> > places;

//    if (!sql_connection.database_->getList(places))
//    {
//      ROS_INFO("Failed to get list of places\n");
//      return -1;
//    }


    std::vector<std::string> parameter;
    parameter.push_back("return_id_x_y");
    database_->callFunction(places,parameter);
    std::cerr << "Retrieved " << places.size() << " places(s) \n";
    if (places.size() > 0) {
        std::cerr << "These are:\n";
        std::cerr << "key\t" << "pos_x\t" << "pos_y\n";
      }
    for (size_t i=0; i<places.size(); i++)
      {
        std::cerr << places[i]->id_.data() << "\t";
        std::cerr << places[i]->pos_x_.data() << "\t";
        std::cerr << places[i]->pos_y_.data() << "\n";
      }
    return true;
}
