#include <string>
#include <vector>

#include <database_interface/db_class.h>

class Places : public database_interface::DBClass
{
public:
  database_interface::DBField<int> id_;
  database_interface::DBField<double> pos_x_;
  database_interface::DBField<double> pos_y_;
  database_interface::DBField<std::string> stamp_;

  Places () :
    id_(database_interface::DBFieldBase::TEXT,this,"key_column","places",true),
    pos_x_(database_interface::DBFieldBase::TEXT,this,"pos_x","places",true),
    pos_y_(database_interface::DBFieldBase::TEXT,this,"pos_y","places",true),
    stamp_(database_interface::DBFieldBase::TEXT,this,"stamp","places",true)
  {
    primary_key_field_ = &id_;
    fields_.push_back(&pos_x_);
    fields_.push_back(&pos_y_);
    fields_.push_back(&stamp_);
  }
};

class Student : public database_interface::DBClass
{
public:
  database_interface::DBField<int> student_id_;
  database_interface::DBField<std::string> student_first_name_;
  database_interface::DBField<std::string> student_last_name_;

  Student() :
    student_id_(database_interface::DBFieldBase::TEXT,
                this, "student_id", "students", true),
    student_first_name_(database_interface::DBFieldBase::TEXT,
                        this, "student_first_name", "students", true),
    student_last_name_(database_interface::DBFieldBase::TEXT,
                       this, "student_last_name", "students", true)
  {
    primary_key_field_ = &student_id_;

    fields_.push_back(&student_first_name_);
    fields_.push_back(&student_last_name_);

    setAllFieldsReadFromDatabase(true);
    setAllFieldsWriteToDatabase(true);
  }
};
