#define _USE_MATH_DEFINES
#include<XnTypes.h>
#include<math.h>
class PersonMessage
{
private:
	float sqrtsquare(float a1, float a2)
	{
		return sqrt(a1*a1+a2*a2);
	}
public:
	XnPoint3D* converttoflour(int n, XnPoint3D* point)
	{
		XnPoint3D* temppoint;
		for (int i = 0; i < n; i++)
		{
			temppoint[i].X = point[i].X;
			temppoint[i].Y = point[i].Y;
			temppoint[i].Z = sqrt(point[i].Z*point[i].Z - point[i].X*point[i].X - point[i].Y*point[i].Y);
		}  
		return point;
	}
	XnPoint3D converttoflour(XnPoint3D point)
	{
		XnPoint3D temppoint;
		temppoint.X = point.X;
		temppoint.Y = point.Y;
		temppoint.Z = sqrt(point.Z*point.Z - point.X*point.X - point.Y*point.Y);
		return temppoint;
	}
public: 
	//0---水平 1---竖直
	float* Getangle(XnPoint3D start,XnPoint3D end)  
	{
		float temp[2];
		XnPoint3D temppoint;
		temppoint.X = end.X-start.X;
		temppoint.Z = end.Y - start.Y;  //Y相减
		temppoint.Y = converttoflour(start).Z - converttoflour(end).Z;//z反过来  z相减
		float xy = sqrtsquare(temppoint.X, temppoint.Y);
		temp[1] = atan(temppoint.Z / xy) * 180 / M_PI;
		temp[0]=atan2(temppoint.Y, temppoint.X) * 180 / M_PI;
		return temp;
	}

};