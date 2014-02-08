#include <string>
#include <vector>

#include <database_interface/db_class.h>

class returnChannels : public database_interface::DBClass
{
public:
  database_interface::DBField<int> id_;
  database_interface::DBField<std::string> channels_;

  returnChannels () :
    id_(database_interface::DBFieldBase::TEXT,this,"key_column","places2",true),
    channels_(database_interface::DBFieldBase::TEXT,this,"channels","places2",true)
  {
    primary_key_field_ = &id_;
    fields_.push_back(&channels_);
  }
};


