database_binding
================

The database connection node of our ROS project. It communicates with a PostgreSQL-Server using the SQL-Database ROS-Package [1] and hands over information to the robotControl [2] and to other nodes, which need information from the database.

It also collects information about the robots state and hands them over to the database.

The code written is licenced under GPL v2. 
If you have any question, don't hestitate to ask.

[1] https://github.com/matthiashh/sql_database
[2] https://github.com/matthiashh/exploration_hh