#pragma once
#include "GRes.h"
#include <time.h>
#include <math.h>
class Position
{
private:
	PosPoint point1;
	PosPoint point2;
	PositiondirectionEnum direction;
	int      mm;
public:
	Position(PositiondirectionEnum direct, int distance)
	{
		
		direction = direct;
		mm = distance * 10;
	}
	bool Check(PosPoint p1/*参考点*/, PosPoint p2/*目标检测点*/)   //p2 在 p1 的哪个方向
	{
		point1 = p1;
		point2 = p2;
		switch (direction)
		{
		case T0_THE_LEFT_OF:
			if ((point2.X - point1.X) < -mm)
				return true;
			else
			    break;
		case TO_THE_RIGHT_OF:
			if ((point2.X - point1.X)>mm)
				return true;
			else
			    break;
		case IN_FRONT_OF:
			if ((point2.Z - point1.Z) < -mm)
				return true;
			else
			    break;
		case BEHIND:
			if ((point2.Z - point1.Z)>mm)
				return true;
			else
			    break;
		case ABOVE:
			if ((point2.Y - point1.Y) >mm)
				return true;
			else
			    break;
		case BELOW:
			if ((point2.Y - point1.Y)<-mm)
				return true;
			else
			    break;
		case APART_FROM:
			break;
		default:
			break;
		}
		return false;
	}
};

class Body
{
private:
	BodyEnum     bodyform;            //检测的内容
	//LEAN检测
	double    Lean_angle;
	//TURN 检测
	double    Turn_angle;
	//JUMP  检测
	JumpPosition jumpposition[2];     //起始与终点
	bool         positionlegal[4];    //对应4个关键点
	double       CheckStart = 0;      //检测开始
	double       CheckEnd = 0;        //检测结束
	int          jumpheight;          //mm
	bool         jumpinit = false;    //是否初始化
public:
	Body( int jumpnum /*厘米为单位*/,BodyEnum body = JUMP/*JUMP*/)  //jump
	{
		bodyform = body;
		jumpheight = jumpnum * 10;
	}
	Body(BodyEnum body,double angle)  //偏转、倾斜角度
	{
		bodyform = body;
		if (body == LEAN_LEFT||body==LEAN_RIGHT)
		{		
			Lean_angle = angle;
		}
		else
		{
			Turn_angle = angle;
		}
	}
	//pos 0-head 1-neck 2-left_shoulder  3-right_shoulder 4-Tosor  
	bool CheckJump(JumpPosition pos)
	{
		if (jumpinit == false)
		{
			if (!Check(pos))
				return false;
			jumpposition[0] = pos;
			//jumpposition[1] = pos;
			CheckStart = clock();
			jumpinit = true;
			return false;
		}
		return IncreaseY(pos);
	}
	bool CheckTurn(PosPoint Left_Shoulder, PosPoint Right_Shoulder)
	{
		if (Left_Shoulder.X == -1.0&& Left_Shoulder.Y == -1.0&&Left_Shoulder.Z == -1.0)
			return false;
		if (Right_Shoulder.X == -1.0&&Right_Shoulder.Y == -1.0&& Right_Shoulder.Z == -1.0)
			return false;
		if (bodyform == TURN_LEFT)
		{
			double disz = Left_Shoulder.Z - Right_Shoulder.Z;
			double disx = Left_Shoulder.X - Right_Shoulder.X;
			if (disz<=0)
				return false;
			if (disx>=0)
				return false;
			disx = abs(disx);
			double tempangle = atan(disz / disx);
			tempangle = tempangle*180/M_PI;
			//cout << tempangle << endl;
			if (tempangle > Turn_angle)
				return true;
			else
				return false;
		}
		if (bodyform == TURN_RIGHT)
		{
			double disz = Right_Shoulder.Z - Left_Shoulder.Z;
			double disx = Right_Shoulder.X - Left_Shoulder.X;
			if (disz <= 0)
				return false;
			if (disx <= 0)
				return false;
			double tempangle = atan(disz / disx);
			tempangle = tempangle*180/M_PI;
			if (tempangle > Turn_angle)
				return true;
			else
				return false;
		}
	}
	bool CheckLean(LeanPosition pos)
	{
		bool headable=true;
		bool neckable=true;
		if (pos.Tosor.X == -1.0&& pos.Tosor.Y == -1.0&&pos.Tosor.Z == -1.0)
			return false;
		if (pos.Head.X == -1.0&&pos.Head.Y == -1.0&&pos.Head.Z == -1.0)
		{
			headable = false;
			return false;
		}
		if (pos.Neck.X == -1.0&& pos.Neck.Y == -1.0&& pos.Neck.Z == -1.0)
		{
			neckable =false;
			return false;
		}
		double tempx1 = pos.Neck.X - pos.Tosor.X;
		double tempy1 = pos.Neck.Y - pos.Tosor.Y;
		double tempx2 = pos.Head.X - pos.Tosor.X;
		double tempy2 = pos.Head.Y - pos.Tosor.Y;
		if (tempy1 <= 0 || tempy2 <= 0)
				return false;
		if (bodyform == LEAN_LEFT)
		{
			if (tempx1 >= 0 || tempx2 >= 0)
				return false;
		}
		if (bodyform == LEAN_RIGHT)
		{
			if (tempx1 <= 0 || tempx2 <= 0)
				return false;
		}
		tempx1 = abs(tempx1);
		tempx2 = abs(tempx2);
		double angle1 = atan(tempx1 / tempy1) * 180 / M_PI;
		double angle2 = atan(tempx2 / tempy2) * 180 / M_PI;
		if ((angle1 + angle2) / 2 > Lean_angle)
			return true;
		else
			return false;
	}
private:
	//符合要求返回TRUE  不等于-1
	bool Check(JumpPosition pos)
	{
		int num = 0;   //无效的点数

		if (pos.Neck.X == -1.0&& pos.Neck.Y == -1.0&& pos.Neck.Z == -1.0)
		{
			positionlegal[0] = false;
			num++;
		}
		else
			positionlegal[0] = true;

		if (pos.Left_Shoulder.X == -1.0&& pos.Left_Shoulder.Y == -1.0&& pos.Left_Shoulder.Z == -1.0)
		{
			positionlegal[1] = false;
			num++;
		}
		else
			positionlegal[1] = true;

		if (pos.Right_Shoulder.X == -1.0&&pos.Right_Shoulder.Y == -1.0&&pos.Right_Shoulder.Z == -1.0)
		{
			positionlegal[2] = false;
			num++;
		}
		else
			positionlegal[2] = true;


		if (pos.Tosor.X == -1.0&&pos.Tosor.Y == -1.0&&pos.Tosor.Z == -1.0)
		{
			positionlegal[3] = false;
			num++;
		}
		else
			positionlegal[3] = true;

		if (num == 0)
			return true;
		else
			return false;
	}
	bool CheckJumpResult(JumpPosition pos)
	{
		CheckEnd = clock();
		if (CheckEnd - CheckStart > 1000)
			return false;

		float sub[4];
		sub[0] = jumpposition[1].Neck.Y - jumpposition[0].Neck.Y;
		sub[1] = jumpposition[1].Left_Shoulder.Y - jumpposition[0].Left_Shoulder.Y;
		sub[2] = jumpposition[1].Right_Shoulder.Y - jumpposition[0].Right_Shoulder.Y;
		sub[3] = jumpposition[1].Tosor.Y - jumpposition[0].Tosor.Y;
       // cout << sub[0];
		
		if (sub[0] > jumpheight&&sub[1] > jumpheight&&sub[2] > jumpheight&&sub[3] > jumpheight)
		{
            cout << jumpheight;
			return true;
		}
		return false;
		
	}
	//num 结构体数组长度  与0比较
	bool IncreaseY(JumpPosition pos)
	{
		float sub[4];
		bool  check = Check(pos);

		sub[0] = pos.Neck.Y - jumpposition[0].Neck.Y;
		sub[1] = pos.Left_Shoulder.Y - jumpposition[0].Left_Shoulder.Y;
		sub[2] = pos.Right_Shoulder.Y - jumpposition[0].Right_Shoulder.Y;
		sub[3] = pos.Tosor.Y - jumpposition[0].Tosor.Y;
		if (check)
		{
			if (sub[0] > 0 && sub[1] > 0 && sub[2] > 0 && sub[3] > 0)
			{
				jumpposition[1] = pos;
				if (CheckJumpResult(pos))
					return true;
			    else
				    return false;
			}
			else
			{
				jumpposition[0] = pos;
				CheckStart = clock();
			}
		}
		return false;
	}
	
};

class velocity
{
private:
	double time;  //检测时间阀值
	double start;
	double end;
	double activatetime;
	bool   activate=false;
	//三个方向的速度
	float  vx, vy, vz;	
	//速度
	int v;  //  mm/s
	velocityDirection direction;
	PosPoint          last;
public:
	velocity(velocityDirection direct, int num,int continuetime=100 /*100ms*/)
	{
		direction = direct;
		v = num * 10;
		last = { -1, -1, -1 };
		vx = vy = vz = 0;
		time = continuetime;
	}
	~velocity()
	{

	}
	float Vx()
	{
		return vx;
	}
	float Vy()
	{
		return vy;
	}
	float Vz()
	{
		return vz;
	}
	//速率
	float Count(float x, float y, float z)
	{
		return sqrt(x*x + y*y + z*z);
	}
	bool  Checkvelocity(PosPoint pos)
	{
		if (pos.X == -1.0&& pos.Y == -1.0&& pos.Z == -1.0)
			return false;
		if (last.X == -1.0&& last.Y == -1.0&& last.Z == -1.0)
		{
			start = clock();
			last = pos;
		}
		vx= pos.X - last.X;
		vy= pos.Y - last.Y;
		vz = pos.Z - last.Z;
		
		end = clock();
		float temptime = (end - start)/1000;
		vx = vx / temptime;
		vy = vy / temptime;
		vz = vz / temptime;
		switch (direction)
		{
		case T0_THE_LEFT:
			if (vx < -v)
			{
				if (!activate)
				{
					activate = true;
					activatetime = end;
				}
				else
				{
					if ((end - activatetime)>time)
						return true;
				}
			}
			else
			{
				activate = false;
				activatetime = end;
			}
			break;
		case TO_THE_RIGHT:
			if (vx >v)
			{
				if (!activate)
				{
					activate = true;
					activatetime = end;
				}
				else
				{
					if ((end - activatetime)>time)
						return true;
				}
			}
			else
			{
				activate = false;
				activatetime = end;
			}
			break;
		case FORWARD:
			if (vz < -v)
			{
				if (!activate)
				{
					activate = true;
					activatetime = end;
				}
				else
				{
					if ((end - activatetime)>time)
						return true;
				}
			}
			else
			{
				activate = false;
				activatetime = end;
			}
			break;
		case BACKWARD:
			if (vz >v)
			{
				if (!activate)
				{
					activate = true;
					activatetime = end;
				}
				else
				{
					if ((end - activatetime)>time)
						return true;
				}
			}
			else
			{
				activate = false;
				activatetime = end;
			}
			break;
		case UP:
			if (vy >v)
			{
				if (!activate)
				{
					activate = true;
					activatetime = end;
				}
				else
				{
					if ((end - activatetime)>time)
						return true;
				}
			}
			else
			{
				activate = false;
				activatetime = end;
			}
			break;
		case DOWN:
			if (vy < -v)
			{
				if (!activate)
				{
					activate = true;
					activatetime = end;
				}
				else
				{
					if ((end - activatetime)>time)
						return true;
				}
			}
			else
			{
				activate = false;
				activatetime = end;
			}
			break;
		case IN_ANY_DIRECTION:
			float any;
			any= Count(vx, vy, vz);
			if (any >v)
			{
				if (!activate)
				{
					activate = true;
					activatetime = end;
				}
				else
				{
					if ((end - activatetime)>time)
						return true;
				}
			}
			else
			{
				activate = false;
				activatetime = end;
			}
			break;
		default:
			break;
		}
		last = pos;
		start = end;
		return false;
	}
};
