// Copyright 2022 Eugen Kaltenegger

#include <tuw_hardware_interface_trinamic/trinamic_connection/trinamic_message/trinamic_reply.h>

using tuw_hardware_interface::TrinamicReply;

unsigned char TrinamicReply::getReplyAddress()
{
  return this->getByte1();
}

unsigned char TrinamicReply::getModuleAddress()
{
  return this->getByte2();
}

unsigned char TrinamicReply::getStatus()
{
  return this->getByte3();
}

unsigned char TrinamicReply::getCommand()
{
  return this->getByte4();
}

int TrinamicReply::getValue()
{
  return TrinamicMessage::getValue();
}

unsigned char TrinamicReply::getChecksum()
{
  return TrinamicMessage::getChecksum();
}
