#include "databasebinding.h"
#include <ros/ros.h>
#include <database_interface/postgresql_database.h>   //needed for database object
#include <string>
#include <sstream>
#include <database_interface/db_class.h>
#include <database_binding/explorationGoal.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>  //msg needed to retrieve the latest position
#include <boost/shared_ptr.hpp>                       //needed for database handling
#include <boost/thread.hpp>                           //used for creating threads
#include <boost/lexical_cast.hpp>                     //type conversions
#include <stdlib.h>                                   //for itoa


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
  sql_connection.databaseListen_->listenToChannel("start");

  sql_connection.database_->listenToChannel("bla2");

  //run endless
  sql_connection.run();
  return 0;
}

void databaseBinding::positionCallback(const geometry_msgs::PoseWithCovarianceStamped pos)
{
  //transform data to the position struct
  latPos_.rosTime.sec = pos.header.stamp.toSec();
  latPos_.x = pos.pose.pose.position.x;
  latPos_.y = pos.pose.pose.position.y;
  latPos_.z = pos.pose.pose.position.z;
}

databaseBinding::databaseBinding()
{
  ROS_INFO("DatabaseBinding built");
  //TO-DO: read configuration from yaml-file
  //Initializing the database
  robot_id_ = -1;
  std::string host = "192.168.10.100";
  std::string port = "5432";
  std::string user = "";
  std::string passwd = "testpassword";
  std::string db = "rosdb";
  ROS_INFO("Trying to connect with host %s, port %s, user %s, passwd %s, db %s",host.c_str(), port.c_str(),user.c_str(),passwd.c_str(),db.c_str());
  this->database_ = new database_interface::PostgresqlDatabase (host,port,user,passwd,db);
  this->databaseListen_ = new database_interface::PostgresqlDatabase (host,port,user,passwd,db);

  //Initialize subscribers and publishers
  position_ = n_.subscribe("/amcl_pose", 10, &databaseBinding::positionCallback, this);
  explorationGoalPub_ = n_.advertise<database_binding::explorationGoal>("/database_binding/exploration_goals",1000);
}

databaseBinding::~databaseBinding()
{
  ROS_INFO("DatabaseBinding destroyed");
}

int databaseBinding::run()
{
  boost::thread notifyThreadObject (&databaseBinding::NotifyThread, this);
  //boost::thread placesThreadObject (&databaseBinding::PlacesThread, this);

  ros::Rate r(10);
  while (ros::ok())
      {
      //Do crazy stuff

      ros::spinOnce();
      r.sleep();
    }
  return 0;
}

bool databaseBinding::getConnection()
{
  if (database_->isConnected())
  {
    return true;
  }
  else
  {
    return false;
  }
}

bool databaseBinding::getPlaces()
{
  std::vector< boost::shared_ptr<returnPlaces> > places;
  database_interface::FunctionCallObj parameter;
  database_binding::explorationGoal newGoal;
  parameter.name = "return_id_x_y_prob";
  database_->callFunction(places,parameter);
  std::cerr << "Retrieved " << places.size() << " places(s) \n";
  if (places.size() > 0) {
      std::cerr << "These are:\n";
      std::cerr << "key\t" << "pos_x\t" << "pos_y\t" << "probability\n";
    }
  for (size_t i=0; i<places.size(); i++)
    {
      std::cerr << places[i]->id_.data() << "\t";
      std::cerr << places[i]->pos_x_.data() << "\t";
      std::cerr << places[i]->pos_y_.data() << "\t";
      std::cerr << places[i]->prob_.data() << "\t";
      newGoal.exploration_x = places[i]->pos_x_.data();
      newGoal.exploration_y = places[i]->pos_y_.data();
      newGoal.exploration_z = 0;
      newGoal.exploration_prob = places[i]->prob_.data();
      explorationGoalPub_.publish (newGoal);
      std::cerr << "PUBLISHED\n";
    }
  return true;
}

bool databaseBinding::NotifyThread()
{
  ros::Rate r(1);
  database_interface::Notification no_;
  while (ros::ok())
    {
      if (getConnection())
        {
          ROS_INFO("The connection should be OK");
        }
      else
        {
          ROS_INFO("I checked and it's not ok");
        }
      if (getConnection() && databaseListen_->checkNotifyIdle(no_))
          {
            database_interface::Notification* tempNotification = new database_interface::Notification;
            *tempNotification = no_;
            ROS_INFO("Got sth from %c with payload %s on channel %s",no_.sending_pid,no_.payload.c_str(),no_.channel.c_str());
            notifyQueue_.push_back(*tempNotification);
            NotifyCallback();
          }

      r.sleep();
    }
  return false;
}

bool databaseBinding::PlacesThread()
{
  ros::Rate r(0.2);
  //Variable declaration
  database_interface::FunctionCallObj functionObj;
  //std::vector<database_interface::FunctionCallObj> placesQueue;
  std::vector< boost::shared_ptr<database_interface::DBClass> > fakePtr;
  std::string fakeStr = "";
  functionObj.params.push_back(fakeStr);
  functionObj.params.push_back(fakeStr);
  functionObj.params.push_back(fakeStr);
  functionObj.params.push_back(fakeStr);
  functionObj.name = "turtlebot_insert_position";


  while (ros::ok())
  {
    if (getConnection())
      {
        functionObj.params[0] = boost::lexical_cast<std::string>(robot_id_);
        functionObj.params[1] = boost::lexical_cast<std::string>(latPos_.x);
        functionObj.params[2] = boost::lexical_cast<std::string>(latPos_.y);
        functionObj.params[3] = boost::lexical_cast<std::string>(latPos_.z);
        ROS_INFO("Right before calling the function");
        database_->callFunction(fakePtr,functionObj);
        ROS_INFO("Right after calling the function");
      }
    r.sleep();
  }
}

void databaseBinding::NotifyCallback()
{
  ROS_INFO("I'm proud to announce, that the Callback has been called");
  getPlaces();
}
