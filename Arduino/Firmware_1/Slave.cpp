#include "Arduino.h"
#include "Slave.h"
#include "FastTWISlave.h"

// delay to accomodate the Broadcom I2C bug.
void Slave::piDelay()
{
  delayMicroseconds(10);
}

void Slave::receive(unsigned char b)
{
  piDelay();
  if(!index_set)
  {
    index = b;
    index_set = 1;
  }
  else
  {
    // Write the data to the buffer
    ((char *)&data)[index] = b;
    index ++;

    // Write the lock value to status if we wrote to the command area.
    if(index < sizeof(CommandData))
      data.slaveCommand.status = CMD_STATUS_LOCK;

    // Wrap at the end of the buffer
    if(index > sizeof(Data))
      index = 0;
  }
}

void Slave::start()
{
  piDelay();
  index = 0; // go to the beginning of the array
  index_set = 0;
}

void Slave::stop()
{
  // Sets the status to "call" if it is in "lock".
  if(CMD_STATUS_LOCK == data.slaveCommand.status)
    data.slaveCommand.status = CMD_STATUS_CALL;
}

unsigned char Slave::transmit()
{
  piDelay();
  return ((char *)&data)[index++];
}
  
void Slave::init(unsigned char address)
{
  FastTWISlave::init(address, *this);
}

bool Slave::commandReady()
{
  return (CMD_STATUS_CALL == data.slaveCommand.status);
}

void Slave::commandDone()
{
  data.slaveCommand.status = CMD_STATUS_DONE;
}
