// Copyright 2022 Eugen Kaltenegger

#ifndef DIP_WS_GENERIC_CONFIG_H
#define DIP_WS_GENERIC_CONFIG_H

#include <memory>
#include "tuw_ros_control_generic/description/generic_config_description.h"
#include "generic_connection.h"

namespace tuw_ros_control_generic
{
class GenericConfig {
public:
  GenericConfig(GenericConfigDescription config_description,
                std::shared_ptr<GenericConnection> connection,
                std::shared_ptr<GenericHardware> hardware);
private:
};
}

#endif //DIP_WS_GENERIC_CONFIG_H
