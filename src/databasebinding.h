#ifndef DATABASEBINDING_H
#define DATABASEBINDING_H
#include <database_interface/db_class.h>
#include <ros/ros.h>

class databaseBinding : public database_interface::DBClass
{
public:
  databaseBinding();
};

#endif // DATABASEBINDING_H
