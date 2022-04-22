#include <Windows.h>
#include "twobuff.h"

static HANDLE consoleBuffers[2]; //����������

/// <summary>
/// </summary>
/// <param name="buffer">�ַ�������</param>
/// <param name="len">����</param>
/// <param name="caption">����ı���</param>
void showBuffer(const char* buffer, int len, char* caption) {
	static int flag = 0;	//����ʹ������buffer
	DWORD bytes = 0;
	COORD coord = { 1, 0 };

	string head = "";
	for (int i = 0; i < len; i++) head += "+---";
	head += "+";
	const char* hc = head.data();
	WriteConsoleOutputCharacterA(consoleBuffers[flag], caption, strlen(caption), coord, &bytes); 
	++coord.Y; WriteConsoleOutputCharacterA(consoleBuffers[flag], hc, strlen(hc), coord, &bytes);
	++coord.Y; WriteConsoleOutputCharacterA(consoleBuffers[flag], buffer, strlen(buffer), coord, &bytes);
	++coord.Y; WriteConsoleOutputCharacterA(consoleBuffers[flag], hc, strlen(hc), coord, &bytes);
	
	SetConsoleActiveScreenBuffer(consoleBuffers[flag]);
	flag = flag ? 0 : 1;
	Sleep(1000);
}

/// <summary>
/// ��ʼ������������
/// </summary>
void initDoubleBuffer() {
	//���������������Ĺ��
	CONSOLE_CURSOR_INFO cci;
	cci.bVisible = 0;
	cci.dwSize = 1;
	//�����µĿ���̨������
	for (int i = 0; i < 2; i++) {
		consoleBuffers[i] = CreateConsoleScreenBuffer(
			GENERIC_WRITE,//������̿�����������д����
			FILE_SHARE_WRITE,//���建�����ɹ���дȨ��
			NULL,
			CONSOLE_TEXTMODE_BUFFER,
			NULL
		);
		SetConsoleCursorInfo(consoleBuffers[i], &cci);
	}
}
