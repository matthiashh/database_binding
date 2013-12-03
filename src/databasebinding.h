#ifndef DATABASEBINDING_H
#define DATABASEBINDING_H
//#include <database_interface/db_class.h>
#include <database_interface/postgresql_database.h>
#include <ros/ros.h>

//was class databaseBinding : public database_interface::DBClass
class databaseBinding
{
private:
  database_interface::PostgresqlDatabase* database_;
  ros::NodeHandle n_;
public:
  databaseBinding();
  ~databaseBinding();
  int run();
  bool getConnection();
};

#endif // DATABASEBINDING_H
