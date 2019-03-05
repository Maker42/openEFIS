#include <Arduino.h>

void CheckSumStart(int16_t &c) {}
void CheckSumDigest(int16_t &cs, unsigned nbytes, const void *b)
{
  const unsigned char *bytes = (const unsigned char*)b;
  int     i;
  for (i = nbytes; i > 0; i--)
  {
    cs ^= *bytes++;
  }
}

bool CheckSumIsValid(int16_t &cs) {return (cs == 0);}


