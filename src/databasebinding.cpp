#include "databasebinding.h"
#include <ros/ros.h>                                  //general header for ros
#include <ros/console.h>                              //for the debugging
#include <database_interface/postgresql_database.h>   //needed for database object
#include <string>                                     //strings are handy
#include <sstream>
#include <exploration_hh/ExplorationGoal.h>         //if we receive new exloration goals
#include <geometry_msgs/PoseWithCovarianceStamped.h>  //msg needed to retrieve the latest position
#include <boost/shared_ptr.hpp>                       //needed for database handling
#include <boost/thread.hpp>                           //used for creating threads
#include <boost/lexical_cast.hpp>                     //type conversions
#include <stdlib.h>                                   //for itoa
#include <database_interface/db_class.h>              //to have a return object



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
 //run endless
  sql_connection.run();
  return 0;
}

void databaseBinding::positionCallback(const geometry_msgs::PoseWithCovarianceStamped pos)
{
  //transform data to the position struct
  //latPos_ = pos;
}

databaseBinding::databaseBinding()
{
  ROS_INFO("DatabaseBinding built");
  //Initializing the database
  robot_id_ = -1;
  connection_state.header.seq = 0;
  std::string host = "192.168.10.100";
  n_.getParam("hostname",host);
  std::string port = "5432";
  n_.getParam("port",port);
  std::string user = "turtlebot";
  n_.getParam("db_user",user);
  std::string passwd = "";
  n_.getParam("db_passwd",passwd);
  std::string db = "rosdb";
  n_.getParam("db_name",db);
  ROS_INFO("Trying to connect with host %s, port %s, user %s, passwd %s, db %s",host.c_str(), port.c_str(),user.c_str(),passwd.c_str(),db.c_str());
  this->database_ = new database_interface::PostgresqlDatabase (host,port,user,passwd,db);
  database_->listenToChannel("start");

  //Initialize subscribers and publishers
  position_ = n_.subscribe("/amcl_pose", 10, &databaseBinding::positionCallback, this);
  explorationGoalPub_ = n_.advertise<exploration_hh::ExplorationGoal>("/database_binding/exploration_goals",1000);
  connection_pub_ = n_.advertise<database_binding::DatabaseConnection>("/database_binding/connection_status",10);
}

databaseBinding::~databaseBinding()
{
  connection_state.connection = 2;
  connection_state.header.seq++;
  connection_state.header.stamp = ros::Time::now();
  connection_pub_.publish(connection_state);
  ROS_INFO("DatabaseBinding destroyed");
}

int databaseBinding::run()
{
  boost::thread notifyThreadObject (&databaseBinding::NotifyThread, this);
  //ros::Timer notify_timer = n_.createTimer(ros::Duration(1), &databaseBinding::PlacesThread, this);
  ros::Timer connection_pub_timer = n_.createTimer(ros::Duration(1), &databaseBinding::ConnectionPublish_,this);
//  boost::thread placesThreadObject (&databaseBinding::PlacesThread, this);

  //returnToolkit test;

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
    database_->reconnect();
    return false;
  }
}

bool databaseBinding::getConfiguration()
{
  //first get a configuration
  std::vector< boost::shared_ptr<returnConfiguration> > configuration;
  database_interface::FunctionCallObj parameter;
  parameter.name = "get_configuration";
  parameter.params.push_back("uuid");
  database_->callFunction(configuration,parameter);
  //assigns results to our configuration
  robot_id_ =  configuration.front()->robot_id_.data();

  //then get the channels to listen to
  std::vector< boost::shared_ptr<returnPlaces> > places;
  database_interface::FunctionCallObj parameter_places;
  parameter_places.name = "get_channels";
  parameter_places.params.push_back(boost::lexical_cast<std::string>(robot_id_));
  database_->callFunction(places,parameter_places);

}

bool databaseBinding::getPlaces()
{
  std::vector< boost::shared_ptr<returnPlaces> > places;
  //std::vector< boost::shared_ptr<database_interface::DBClass> > places;


  //std::vector< boost::shared_ptr<returnToolkit> > toolkit_places;

  database_interface::FunctionCallObj parameter;
  exploration_hh::ExplorationGoal newGoal;
  newGoal.header.stamp = ros::Time::now();
  newGoal.header.frame_id = "/map";
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
      newGoal.pose.pose.position.x = places[i]->pos_x_.data();
      newGoal.pose.pose.position.y = places[i]->pos_y_.data();
      newGoal.pose.pose.position.z = 0;
      newGoal.probability = places[i]->prob_.data();
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
    if (getConnection() && database_->checkNotifyIdle(no_))
        {
          ROS_INFO("Got sth from %c with payload %s on channel %s",no_.sending_pid,no_.payload.c_str(),no_.channel.c_str());
          NotifyCallback();
        }
      r.sleep();
    }
  return false;
}

void databaseBinding::PlacesThread(const ros::TimerEvent& event)
{
  //Variable declaration
  database_interface::FunctionCallObj functionObj;
  std::vector< boost::shared_ptr<database_interface::DBClass> > fakePtr (4);
  std::string empty_string = "";
  functionObj.params.push_back(empty_string);
  functionObj.params.push_back(empty_string);
  functionObj.params.push_back(empty_string);
  functionObj.params.push_back(empty_string);

  functionObj.name = "turtlebot_insert_position";

  if (getConnection())
    {
      functionObj.params[0] = boost::lexical_cast<std::string>(robot_id_);
      functionObj.params[1] = boost::lexical_cast<std::string>(latPos_.pose.pose.position.x);
      functionObj.params[2] = boost::lexical_cast<std::string>(latPos_.pose.pose.position.y);
      functionObj.params[3] = boost::lexical_cast<std::string>(latPos_.pose.pose.position.z);
      database_->callFunction(fakePtr,functionObj);
    }
}

void databaseBinding::NotifyCallback()
{
  ROS_INFO("I'm proud to announce, that the Callback has been called");
  getPlaces();
}


void databaseBinding::ConnectionPublish_(const ros::TimerEvent &event)
{
  if (getConnection())
  {
    connection_state.connection = 0;
  }
  else
  {
    connection_state.connection = 2;
  }
  connection_state.header.seq++;
  connection_state.header.stamp = ros::Time::now();
  connection_pub_.publish(connection_state);
}
