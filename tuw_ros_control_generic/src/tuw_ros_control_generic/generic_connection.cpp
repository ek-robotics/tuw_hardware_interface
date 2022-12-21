// Copyright 2022 Eugen Kaltenegger

#include "tuw_ros_control_generic/generic_connection.h"
#include "tuw_ros_control_generic/description/generic_connection_description.h"

using tuw_ros_control_generic::GenericConnection;
using tuw_ros_control_generic::GenericConnectionDescription;

std::mutex GenericConnection::mutex_;
std::unique_ptr<std::map<std::string, std::shared_ptr<GenericConnection>>> GenericConnection::connection_table_;

std::shared_ptr<GenericConnection>GenericConnection::getConnection(GenericConnectionDescription connection_description)
{
  std::string connection_hash = connection_description.getHash();

  if (GenericConnection::connection_table_ == nullptr)
  {
    GenericConnection::mutex_.lock();

    if (GenericConnection::connection_table_ == nullptr)

      GenericConnection::connection_table_ = std::make_unique<std::map<std::string, std::shared_ptr<GenericConnection>>>();

    GenericConnection::mutex_.unlock();
  }

  if (GenericConnection::connection_table_->find(connection_hash) == GenericConnection::connection_table_->end())
  {
    std::shared_ptr<GenericConnection> connection = nullptr;
    GenericConnection::connection_table_->insert({connection_hash, connection}) ;
  }

  return GenericConnection::connection_table_->at(connection_description.getHash());
}

GenericConnection::GenericConnection(GenericConnectionDescription connection_description)
{
  // call connect in implementation
}

GenericConnection::GenericConnection()
{
  // constructor for mocking
}

GenericConnection::~GenericConnection()
{
  // call disconnect in implementation
}
