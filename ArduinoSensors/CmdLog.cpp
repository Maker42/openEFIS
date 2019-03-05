#include "CmdLog.h"

void cmdLog(CmdMessenger &cmdMessenger, unsigned level, const char *s)
{
  int16_t checksum = 0;
  cmdMessenger.sendCmdStart(CMD_LOG);
  cmdMessenger.sendCmdArg (level);
  cmdMessenger.sendCmdEscArg (s);
  CheckSumStart(checksum);
  CheckSumDigest(checksum, sizeof(level), &level);
  CheckSumDigest(checksum, strlen(s), s);
  cmdMessenger.sendCmdArg (checksum);
  cmdMessenger.sendCmdEnd();
}


