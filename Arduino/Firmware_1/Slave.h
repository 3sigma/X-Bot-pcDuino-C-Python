#pragma once
#include "FastTWISlave.h"

const uint8_t ARGS_LENGTH = 32;

static unsigned char CMD_STATUS=0;
static unsigned char CMD_NUMBER=1;
static unsigned char CMD_STATUS_LOCK=2;
static unsigned char CMD_STATUS_CALL=1;
static unsigned char CMD_STATUS_DONE=0;

class Slave: public FastTWISlave
{
private:
  struct CommandData
  {
    uint8_t status, command;
    char args[ARGS_LENGTH];
  };

  struct Data
  {
    CommandData slaveCommand;
    CommandData masterCommand;
  } data;
  unsigned char index;
  unsigned char index_set = 0;

  void piDelay();

public:

  template<typename T>
  void runMasterCommand(int cmd, T &t)
  {
    data.masterCommand.command = cmd;
    memcpy(data.masterCommand.args, t, 19);
    data.masterCommand.status = CMD_STATUS_CALL;
  };

  template <typename R>
  void checkCommand(int x, R (*slaveCommand)())
  {
    if(x == data.slaveCommand.command)
    {
      *(R *)data.slaveCommand.args = slaveCommand();
    }
  };

  template <typename R, typename T>
  void checkCommand(int x, R (*slaveCommand)(T))
  {
    if(x == data.slaveCommand.command)
    {
      *(R *)data.slaveCommand.args = slaveCommand(*(T *)(data.slaveCommand.args));
    }
  };
  
  template <typename R, typename T, typename U>
  void checkCommand(int x, R (*slaveCommand)(T, U))
  {
    struct __attribute__ ((__packed__)) args { T t; U u; };
    if(x == data.slaveCommand.command)
    {
      args * a = (args *)(data.slaveCommand.args);
      *(R *)data.slaveCommand.args = slaveCommand(a->t, a->u);
    }
  };
  
  template <typename R, typename T, typename U, typename V>
  void checkCommand(int x, R (*slaveCommand)(T, U, V))
  {
    struct __attribute__ ((__packed__)) args { T t; U u; V v; };
    if(x == data.slaveCommand.command)
    {
      args * a = (args *)(data.slaveCommand.args);
      *(R *)data.slaveCommand.args = slaveCommand(a->t, a->u, a->v);
    }
  };

  void checkCommand(int x, void (*slaveCommand)(char *))
  {
    if(x == data.slaveCommand.command)
    {
      slaveCommand((char *)data.slaveCommand.args);
    }
  };

  template <typename T>
  void checkCommand(int x, void (*slaveCommand)(T&))
  {
    if(x == data.slaveCommand.command)
    {
      slaveCommand(*(T *)data.slaveCommand.args);
    }
  };

  template <typename T>
  void checkCommand(int x, void (*slaveCommand)(T))
  {
    if(x == data.slaveCommand.command)
    {
      slaveCommand(*(T *)data.slaveCommand.args);
    }
  };
  
  template <typename T, typename U>
  void checkCommand(int x, void (*slaveCommand)(T, U))
  {
    struct __attribute__ ((__packed__)) args { T t; U u; };
    if(x == data.slaveCommand.command)
    {
      args * a = (args *)(data.slaveCommand.args);
      slaveCommand(a->t, a->u);
    }
  };
  
  template <typename T, typename U, typename V>
  void checkCommand(int x, void (*slaveCommand)(T, U, V))
  {
    struct __attribute__ ((__packed__)) args { T t; U u; V v; };
    if(x == data.slaveCommand.command)
    {
      args * a = (args *)(data.slaveCommand.args);
      slaveCommand(a->t, a->u, a->v);
    }
  };
  
  bool commandReady();
  void commandDone();

  virtual void receive(uint8_t b);
  virtual uint8_t transmit();
  virtual void start();
  virtual void stop();

  /* Initialize the slave on a given address. */
  void init(unsigned char address);
};
