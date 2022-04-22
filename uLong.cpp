#include "ulong.h"
#include <stdlib.h>
#include <bitset>

using namespace std;

static int MASK_LEN = 2;   //掩码长度;
//64位二进制数的64/MASK_LEN个块所对应的二进制掩码的数组
static uLong* masks = NULL;
static int LEFT_MOST = 0;  //最左边MASK_LEN位二进制数的位置

/// <summary>
/// 设置64位二进制数的掩码长度，并设置各个掩码
/// </summary>
/// <param name="mask_len"></param>
extern void setMask(int mask_len)
{
    MASK_LEN = mask_len;
    LEFT_MOST = (64 - MASK_LEN) / MASK_LEN;

    masks = (uLong*)malloc(sizeof(uLong) * 64 / MASK_LEN);

    //核心掩码 如11或1111，分别对应2位二进制块和4位二进制块
    int mask = 1;
    for (int i = 1; i < MASK_LEN; i++) {
        mask = mask << 1 | 1;
    }

    //不同二进制块的掩码不同
    for (int i = 0; i < 64 / MASK_LEN; i++) {
        masks[i] = (uLong)mask << (LEFT_MOST - i) * MASK_LEN;
    }
}

//获取64位二进制数中，从左往右第index个“MASK_LEN位二进制数”（index从0开始）
uLong getValue(uLong status, int index) {
    int a = masks[index];
    return (status & masks[index]) >> (LEFT_MOST - index) * MASK_LEN;
}


/// <summary>
///  将byte状态数组转换为64位无符号整数   byte[] -> ulong
///     例如MASK_LEN=4时，可将0~15转换位4位二进制数，总共16个，所以1个uLong
///     可将0~8转换为4位二进制数，总共9个，占uLOng的前36个二进制位
///  数组的第一个元素（index = 0）对应该整数的最高四位
/// </summary>
/// <param name="status"></param>
/// <param name="n">数组元素的个数</param>
/// <returns></returns>
uLong getBitRep(byte status[], int n) {
    uLong rep = 0;
    for (int i = 0; i < n; i++) {
        auto b1 = bitset<64>(rep);
        auto b = bitset<64>(((uLong)status[i] << (LEFT_MOST - i) * MASK_LEN));
        rep = rep | ((uLong)status[i] << (LEFT_MOST - i) * MASK_LEN);
        auto b2 = bitset<64>(rep);
        int a = 0;
    }
    return rep;
}

/// <summary>
/// 将64位二进制数，转换为byte数组。数组的个数=LEFT_MOST+1      ulong -> byte[]
/// </summary>
/// <param name="status"></param>
/// <returns></returns>
byte* getValues(uLong status)
{
    byte* values = (byte*)malloc(sizeof(byte) * (LEFT_MOST + 1));
    if (!values) return NULL;

    for (int i = 0; i <= LEFT_MOST; i++) {
        values[i] = (byte)getValue(status, i);
    }
    return values;
}

//将status的第index个“长为MASK_LEN的二进制块”重置为0并返回
uLong resetZero(uLong status, int index) {
    return status & (~masks[index]);
}

//将status的第index个“长为MASK_LEN的二进制块”置为value并返回
uLong setValue(uLong status, int index, byte value) {
    status = resetZero(status, index);
    return status | ((uLong)value << (LEFT_MOST - index) * MASK_LEN);
}

// 返回0的索引
int getZero(uLong status)
{
    byte* bytes = getValues(status);
    for (int i = 0; i < 64 / MASK_LEN; i++)
    {
        if (!bytes[i])
        {
            free(bytes);
            return i;
        }
    }
    free(bytes);
    return -1;
}
