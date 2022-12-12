// Copyright 2022 Eugen Kaltenegger


#include "tuw_ros_control_generic/generic_connection.h"

using tuw_ros_control_generic::GenericConnection;
using tuw_ros_control_generic::GenericConnectionDescription;

GenericConnection::GenericConnection(GenericConnectionDescription connection_description)
{
  // call connect in implementation
}

GenericConnection::~GenericConnection()
{
  // call disconnect in implementation
}

std::shared_ptr<GenericConnection>GenericConnection::getConnection(GenericConnectionDescription connection_description)
{
  if (GenericConnection::connection_table_ == nullptr)
  {
    GenericConnection::mutex_.lock();

    if (GenericConnection::connection_table_ == nullptr)

      GenericConnection::connection_table_ = std::make_unique<std::map<std::string, std::shared_ptr<GenericConnection>>>();

    GenericConnection::mutex_.unlock();
  }

  if (GenericConnection::connection_table_->find(connection_description.getHash()) == GenericConnection::connection_table_->end())
  {
    std::string key = connection_description.getHash();
    std::shared_ptr<GenericConnection> connection = nullptr;
    GenericConnection::connection_table_->insert({key, connection}) ;
  }

  return GenericConnection::connection_table_->at(connection_description.getHash());
}
