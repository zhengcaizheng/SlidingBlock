#ifndef _ULONG_H
#define _ULONG_H
typedef unsigned char byte;
typedef unsigned long long uLong;

extern void setMask(int mask_len);
extern uLong getValue(uLong status, int index);
extern uLong getBitRep(byte status[], int n);
extern uLong resetZero(uLong status, int index);
extern uLong setValue(uLong status, int index, byte value);
extern byte* getValues(uLong status);
extern int getZero(uLong status);
#endif