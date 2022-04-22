#ifndef _TWO_BUFFER_H
#define _TWO_BUFFER_H
#include <vector>
#include <string>
#include <iostream>
using namespace std;
extern void showBuffer(const char* buffer, int len, char* caption = (char*)"Sibling Block");
extern void initDoubleBuffer();
#endif
