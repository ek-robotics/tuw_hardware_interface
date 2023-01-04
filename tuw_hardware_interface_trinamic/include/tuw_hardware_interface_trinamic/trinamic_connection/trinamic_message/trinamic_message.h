// Copyright 2022 Eugen Kaltenegger

#ifndef DIP_WS_TRINAMIC_MESSAGE_H
#define DIP_WS_TRINAMIC_MESSAGE_H

#include <string>

namespace tuw_hardware_interface
{
class TrinamicMessage
{
public:
  unsigned char* getBufferPointer();
  size_t getBufferSize();
  unsigned char calculateChecksum();

  std::string toString();

protected:
  TrinamicMessage() = default;
  ~TrinamicMessage() = default;

  void setByte1(unsigned char byte);
  unsigned char getByte1();
  void setByte2(unsigned char byte);
  unsigned char getByte2();
  void setByte3(unsigned char byte);
  unsigned char getByte3();
  void setByte4(unsigned char byte);
  unsigned char getByte4();
  virtual void setValue(int value);
  virtual int getValue();
  void setChecksum(unsigned char checksum);
  virtual unsigned char getChecksum();

  unsigned char buffer_[9]{};
  unsigned char* module_address_ = &buffer_[0];
  unsigned char* command_number_ = &buffer_[1];
  unsigned char* command_type_ = &buffer_[2];
  unsigned char* motor_number_ = &buffer_[3];
  unsigned char* checksum_ = &buffer_[8];
};
}  // namespace tuw_hardware_interface

#endif  // DIP_WS_TRINAMIC_MESSAGE_H
