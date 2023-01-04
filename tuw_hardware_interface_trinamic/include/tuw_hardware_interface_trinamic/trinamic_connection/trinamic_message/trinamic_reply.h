// Copyright 2022 Eugen Kaltenegger

#ifndef DIP_WS_TRINAMIC_REPLY_H
#define DIP_WS_TRINAMIC_REPLY_H

#include <tuw_hardware_interface_trinamic/trinamic_connection/trinamic_message/trinamic_message.h>

namespace tuw_hardware_interface
{
class TrinamicReply : public TrinamicMessage
{
public:
  unsigned char getReplyAddress();
  unsigned char getModuleAddress();
  unsigned char getStatus();
  unsigned char getCommand();
  int getValue() override;
  unsigned char getChecksum() override;
};
}  // namespace tuw_hardware_interface


#endif //DIP_WS_TRINAMIC_REPLY_H
