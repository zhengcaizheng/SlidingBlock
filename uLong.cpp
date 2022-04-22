#include "ulong.h"
#include <stdlib.h>
#include <bitset>

using namespace std;

static int MASK_LEN = 2;   //���볤��;
//64λ����������64/MASK_LEN��������Ӧ�Ķ��������������
static uLong* masks = NULL;
static int LEFT_MOST = 0;  //�����MASK_LENλ����������λ��

/// <summary>
/// ����64λ�������������볤�ȣ������ø�������
/// </summary>
/// <param name="mask_len"></param>
extern void setMask(int mask_len)
{
    MASK_LEN = mask_len;
    LEFT_MOST = (64 - MASK_LEN) / MASK_LEN;

    masks = (uLong*)malloc(sizeof(uLong) * 64 / MASK_LEN);

    //�������� ��11��1111���ֱ��Ӧ2λ�����ƿ��4λ�����ƿ�
    int mask = 1;
    for (int i = 1; i < MASK_LEN; i++) {
        mask = mask << 1 | 1;
    }

    //��ͬ�����ƿ�����벻ͬ
    for (int i = 0; i < 64 / MASK_LEN; i++) {
        masks[i] = (uLong)mask << (LEFT_MOST - i) * MASK_LEN;
    }
}

//��ȡ64λ���������У��������ҵ�index����MASK_LENλ������������index��0��ʼ��
uLong getValue(uLong status, int index) {
    int a = masks[index];
    return (status & masks[index]) >> (LEFT_MOST - index) * MASK_LEN;
}


/// <summary>
///  ��byte״̬����ת��Ϊ64λ�޷�������   byte[] -> ulong
///     ����MASK_LEN=4ʱ���ɽ�0~15ת��λ4λ�����������ܹ�16��������1��uLong
///     �ɽ�0~8ת��Ϊ4λ�����������ܹ�9����ռuLOng��ǰ36��������λ
///  ����ĵ�һ��Ԫ�أ�index = 0����Ӧ�������������λ
/// </summary>
/// <param name="status"></param>
/// <param name="n">����Ԫ�صĸ���</param>
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
/// ��64λ����������ת��Ϊbyte���顣����ĸ���=LEFT_MOST+1      ulong -> byte[]
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

//��status�ĵ�index������ΪMASK_LEN�Ķ����ƿ顱����Ϊ0������
uLong resetZero(uLong status, int index) {
    return status & (~masks[index]);
}

//��status�ĵ�index������ΪMASK_LEN�Ķ����ƿ顱��Ϊvalue������
uLong setValue(uLong status, int index, byte value) {
    status = resetZero(status, index);
    return status | ((uLong)value << (LEFT_MOST - index) * MASK_LEN);
}

// ����0������
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
