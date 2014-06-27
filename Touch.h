#pragma once 
#include <Windows.h>
class Touch
{
private:
	
public:
	Touch();
	~Touch();
	void Move(float x, float y);
	void Click();
	void DoubleClick();
	void Reset();
private:
	bool  start= false;
	///
	float Ax = 1;
	float Hx = 1;
	float Qx = 0.01;
	float Rx = 0.01;
	float Px;
	float X;
	float Kalmanx(float);
	//////
	float Ay = 1;
	float Hy = 1;
	float Qy = 0.01;
	float Ry = 0.01;
	float Py;
	float Y;
	float Kalmany(float);
	//////////
	int INCx = 0.2;
	int INCy = 0.1;
	float LastX;
	float LastY;
};

Touch::Touch()
{

}
Touch::~Touch()
{

}
void Touch::Move(float x,float y)
{
	if (!start)
	{
		LastX = x;
		Px = 10;
		LastY = x;
		Py = 10;
		start = true;
		return;
	}
	float tempx, tempy;
	float movx, movy;
	tempx = x - LastX;
	tempy = Y - LastY;
	LastX = x;
	LastY = y;
	
	movx = Kalmanx(tempx);
	movy = Kalmany(tempy);
    POINT pt;	
	GetCursorPos(&pt);
	pt.x = pt.x + (int)movx*INCx;
	pt.y = pt.y + (int)movy*INCy;
	SetCursorPos(pt.x, pt.y);
}
void Touch::Click()
{

}
void Touch::DoubleClick()
{

}
void Touch::Reset()
{

}
float Touch::Kalmanx(float zx)
{
	float tempx, tempp,kg;
	//tempx 下一次预测 X上一次最优化
	tempx = Ax*X;
	tempp = Ax*Px + Qx;
	kg = tempp*Hx / (Hx*tempp*Hx + Rx);
	X = tempx + kg*(zx - Hx*tempx);
	Px = (1 - kg*Hx)*tempp;
	return X;
}
float Touch::Kalmany(float zy)
{
	float tempy, tempp, kg;
	//tempx 下一次预测 X上一次最优化
	tempy = Ay*Y;
	tempp = Ay*Py + Qy;
	kg = tempp*Hy / (Hy*tempp*Hy + Ry);
	Y = tempy + kg*(zy - Hy*tempy);
	Py = (1 - kg*Hy)*tempp;
	return Y;
}