#include <Windows.h>
#include "twobuff.h"

static HANDLE consoleBuffers[2]; //两个缓冲区

/// <summary>
/// </summary>
/// <param name="buffer">字符缓冲区</param>
/// <param name="len">长度</param>
/// <param name="caption">输出的标题</param>
void showBuffer(const char* buffer, int len, char* caption) {
	static int flag = 0;	//交替使用两个buffer
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
/// 初始化两个缓冲区
/// </summary>
void initDoubleBuffer() {
	//隐藏两个缓冲区的光标
	CONSOLE_CURSOR_INFO cci;
	cci.bVisible = 0;
	cci.dwSize = 1;
	//创建新的控制台缓冲区
	for (int i = 0; i < 2; i++) {
		consoleBuffers[i] = CreateConsoleScreenBuffer(
			GENERIC_WRITE,//定义进程可以往缓冲区写数据
			FILE_SHARE_WRITE,//定义缓冲区可共享写权限
			NULL,
			CONSOLE_TEXTMODE_BUFFER,
			NULL
		);
		SetConsoleCursorInfo(consoleBuffers[i], &cci);
	}
}
