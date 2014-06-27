#pragma once
#include "XtionPersonTra.h"
#include <opencv2\highgui\highgui.hpp>
#include <string>
#include "PersonMessage.h"
#include "Position.h"
#include "Touch.h"
void XtionPersonTrademo()
{
	XtionPerson persontra = XtionPerson(true, true, true, true);

	if (persontra.Check())
		std::cout << "true" << std::endl;
	std::string  str = "test1";
	cvNamedWindow(str.c_str());
	std::string  str2 = "test2";
	cvNamedWindow(str2.c_str());
	std::string  str3 = "test3";
	cvNamedWindow(str3.c_str());
	//persontra.Update();
	//persontra.CalcHist();
	persontra.WriteVideoMat("1.avi");
	Mat mat1;
	Mat mat2;
	Mat mat3;
	Mat mat4;
	while (cvWaitKey(20) != 'a')
	{
		//更新数据  *****
		persontra.Update();
		//也可以只调用一次，不更新  ***
		persontra.CalcHist();
		//转换为Mat  *****
		persontra.DepthMDtoMat3();   //返回值  仅仅是深度数据转换为Mat
		//识别关节和用户状态   ****  
		persontra.GetJiont();
		//显示
		cv::imshow(str, persontra.GetFrame());
		//写入
		persontra.WriteMat();
		//获取关节坐标
		XnPoint3D point = persontra.ConvertToProject(persontra.GetJointPostionReal(1, XN_SKEL_TORSO));  //1 表示用户1 XN_SKEL_HEAD--〉头部XN_SKEL_TORSO  XN_SKEL_HEAD
		//cout << persontra.GetJointPostionReal(1, XN_SKEL_TORSO).Z << endl;
		//cout << point.X << " " << point.Y << "  " << point.Z << endl;
		if (point.Z != -1)
		{
			cout << "头部关节的深度值为 " << point.X << " " << point.Y << " " << point.Z << endl;
			if ((int)point.X > 0 && (int)point.Y > 0)
			{
				Mat temp1 = persontra.GetUserMat();
				Mat temp2 = persontra.GetDepthMat();  //深度
				cout << "头部深度数据      " << (int)point.X << " " << (int)point.Y << " " << temp2.ptr<unsigned short>((int)point.X, (int)point.Y)[0]
					<< "  " << temp1.ptr<unsigned short>((int)point.X, (int)point.Y)[0] << endl;
				//cout << "qnmlgb" << temp3.ptr<float>((int)point.X, (int)point.Y)[0] << "  " << temp3.ptr<float>((int)point.X, (int)point.Y)[1]<<endl;
			}			
		}
		mat1 = persontra.GetDepthMat();
		//cout << "原始" << mat1.ptr<float>(100, 100)[0] << endl;
		mat1.convertTo(mat2, CV_8U,255.0/4000);
		circle(mat2, Point((int)point.X, (int)point.Y), 2, Scalar(0, 0, 255), 2, 8, 0);
		//cout << "转换" << (int)mat2.data[100 * 320 + 100] << endl;
		cv::imshow(str2, mat2);
		if (persontra.GetnPerson() != 0)
		{
			mat3 = persontra.GetUsersDepthMat(1);
			mat3.convertTo(mat4, CV_8U, 255.0 / 4000);
			cv::imshow(str3, mat4);
		}
		//Mat temp4 = persontra.GetDepthMat();
		//cout << temp4.ptr<float>(10, 10)[0] << endl;
		//circle(temp4, Point(10,10), 2, Scalar(255, 255, 255), 2, 8, 0);
		//Mat temp5;	
		//temp4.convertTo(temp5, CV_8U, 255.0 / 4000);
		//cv::imshow(str3, temp5);
		
	}
}
void XtionGetPosition()
{
	XtionPerson persontra = XtionPerson(true, true, true, true);

	if (persontra.Check())
		std::cout << "true" << std::endl;
	Mat mat1;
	Mat mat2;
	Mat mat3;
	Mat mat4;
	std::string  str = "test1";
	cvNamedWindow(str.c_str());
	while (cvWaitKey(20) != 'a')
	{
		//更新数据  *****
		persontra.Update();
		//也可以只调用一次，不更新  ***
		persontra.CalcHist();
		//转换为Mat  *****
		persontra.DepthMDtoMat3();   //返回值  仅仅是深度数据转换为Mat
		//识别关节和用户状态   ****  
		persontra.GetJiont();
		//显示
		cv::imshow(str, persontra.GetFrame());

		XnPoint3D pointShouder = persontra.GetJointPostionReal(1, XN_SKEL_RIGHT_SHOULDER);  //1 表示用户1 XN_SKEL_HEAD--〉头部
		XnPoint3D pointElbow = persontra.GetJointPostionReal(1, XN_SKEL_RIGHT_ELBOW);
		XnPoint3D pointHand = persontra.GetJointPostionReal(1, XN_SKEL_RIGHT_HAND);
		
		XnPoint3D pointHead = persontra.GetJointPostionReal(1, XN_SKEL_HEAD);  //1 表示用户1 XN_SKEL_HEAD--〉头部
		XnPoint3D pointNeck = persontra.GetJointPostionReal(1, XN_SKEL_NECK);
		//if (pointShouder.Z != -1)
		//	cout << "  " << pointShouder.X << "   " << pointShouder.Y<<"  "<<pointShouder.Z << endl;
		PersonMessage message = PersonMessage();
		float* a = message.Getangle(pointNeck, pointHead);
		cout << a[0] << "  " << a[1] << endl;
		//float* Getangle(XnPoint3D start, XnPoint3D end)

	}
}
void XtionMouse()
{
	XtionPerson persontra = XtionPerson(true, true, true, true);

	if (persontra.Check())
		std::cout << "true" << std::endl;
	Mat mat;
	std::string  str = "test1";
	cvNamedWindow(str.c_str());
//	XnPoint3D  last;
//	XnPoint3D  lastpos;
	bool       Activate=false;
	Touch      touch = Touch();
	while (cvWaitKey(20) != 'a')
	{
		//更新数据  *****
		persontra.Update();
		//也可以只调用一次，不更新  ***
		persontra.CalcHist();
		//转换为Mat  *****
		persontra.DepthMDtoMat3();   //返回值  仅仅是深度数据转换为Mat
		//识别关节和用户状态   ****  
		persontra.GetJiont();
		//显示
		mat = persontra.GetFrame();
		
		cv::imshow(str, mat);
		XnPoint3D pointHand = persontra.GetJointPostionReal(1, XN_SKEL_RIGHT_HAND);
		XnPoint3D pointHead = persontra.GetJointPostionReal(1, XN_SKEL_HEAD);
		
		if (pointHand.Z != -1 && pointHead.Z!=-1)
		{
			//cout << pointHand.Z << "   " << pointHead.Z << endl;
			if (pointHead.Z - pointHand.Z > 300)
			{
				touch.Move(pointHand.X, pointHand.Y);
			}
		}
	}
}
void XtionPostion()
{
	XtionPerson persontra = XtionPerson(true, true, true, true);

	if (persontra.Check())
		std::cout << "true" << std::endl;
	Mat mat;
	std::string  str = "test1";
	cvNamedWindow(str.c_str(), CV_WINDOW_AUTOSIZE);
	Position postion = Position(BELOW, 40);
	//Body     body = Body(JUMP, 20);
	//Body       body = Body(TURN_RIGHT, 30);
	//Body       body = Body(LEAN_LEFT, 30);
	velocity v = velocity(UP, 100);
	while (cvWaitKey(20) != 'a')
	{
		//更新数据  *****
		persontra.Update();
		//也可以只调用一次，不更新  ***
		persontra.CalcHist();
		//转换为Mat  *****
		persontra.DepthMDtoMat3();   //返回值  仅仅是深度数据转换为Mat
		//识别关节和用户状态   ****  
		persontra.GetJiont();
		//显示
		cv::imshow(str, persontra.GetFrame());

		XnPoint3D Left_Hand = persontra.GetJointPostionReal(1, XN_SKEL_RIGHT_HAND);
		PosPoint left_hand = { Left_Hand.X, Left_Hand.Y, Left_Hand.Z };
		if (v.Checkvelocity(left_hand))
			cout << "激活" << endl;
		//cout << v.Vx() << endl;
		/*
		XnPoint3D Head = persontra.GetJointPostionReal(1, XN_SKEL_HEAD);
		XnPoint3D Neck = persontra.GetJointPostionReal(1, XN_SKEL_NECK);
		XnPoint3D Torso = persontra.GetJointPostionReal(1, XN_SKEL_TORSO);

		PosPoint head = { Head.X, Head.Y, Head.Z };
		PosPoint neck = { Neck.X, Neck.Y, Neck.Z };
		PosPoint troso = { Torso.X, Torso.Y, Torso.Z };
		LeanPosition pos = { head, neck, troso };
		if (body.CheckLean(pos))
			cout << "激活" << endl;
		*/

		/*
		XnPoint3D Left_Shoulder = persontra.GetJointPostionReal(1, XN_SKEL_LEFT_SHOULDER);
		XnPoint3D Right_Shoulder = persontra.GetJointPostionReal(1, XN_SKEL_RIGHT_SHOULDER);
		PosPoint left = { Left_Shoulder.X, Left_Shoulder.Y, Left_Shoulder.Z };
		PosPoint right = { Right_Shoulder.X, Right_Shoulder.Y, Right_Shoulder.Z };
		if (body.CheckTurn(left, right))
			cout << "激活" << endl;
		*/

		/*
		XnPoint3D Neck = persontra.GetJointPostionReal(1, XN_SKEL_NECK);
		XnPoint3D Left_Shoulder = persontra.GetJointPostionReal(1, XN_SKEL_LEFT_SHOULDER);
		XnPoint3D Right_Shoulder = persontra.GetJointPostionReal(1, XN_SKEL_RIGHT_SHOULDER);
		XnPoint3D Torso = persontra.GetJointPostionReal(1, XN_SKEL_TORSO);

		PosPoint neck = { Neck.X, Neck.Y, Neck.Z };
		PosPoint left = { Left_Shoulder.X, Left_Shoulder.Y, Left_Shoulder.Z };
		PosPoint right = { Right_Shoulder.X, Right_Shoulder.Y, Right_Shoulder.Z };
		PosPoint troso = { Torso.X, Torso.Y, Torso.Z };
		JumpPosition jump = { neck, left, right, troso };
		//cout << neck.X << endl;
		if (body.CheckJump(jump))
			cout << "激活" << endl;
		*/


		/*
		XnPoint3D head = persontra.GetJointPostionReal(1, XN_SKEL_HEAD);
		XnPoint3D Left_hand = persontra.GetJointPostionReal(1, XN_SKEL_LEFT_HAND);
		PosPoint  poshead = { head.X,head.Y,head.Z};
		PosPoint  posLeftHand = { Left_hand.X, Left_hand.Y, Left_hand.Z };
		if (postion.Check(poshead, posLeftHand))
		{
			cout << "激活" << endl;
		}
		*/
		


		
	}
}
void TakeOff()
{
	XtionPerson persontra = XtionPerson(true, true, true, true);

	if (persontra.Check())
		std::cout << "true" << std::endl;
	Mat mat;
	std::string  str = "test1";
	cvNamedWindow(str.c_str(), CV_WINDOW_AUTOSIZE);
	Position postion = Position(ABOVE, 10);
	//Body     body = Body(JUMP, 20);
	//Body       body = Body(TURN_RIGHT, 30);
	//Body       body = Body(LEAN_LEFT, 30);
	velocity v = velocity(UP, 20);

	while (cvWaitKey(20) != 'a')
	{
		//更新数据  *****
		persontra.Update();
		//也可以只调用一次，不更新  ***
		persontra.CalcHist();
		//转换为Mat  *****
		persontra.DepthMDtoMat3();   //返回值  仅仅是深度数据转换为Mat
		//识别关节和用户状态   ****  
		persontra.GetJiont();
		//显示
		cv::imshow(str, persontra.GetFrame());

		XnPoint3D Right_Hand = persontra.GetJointPostionReal(1, XN_SKEL_RIGHT_HAND);
		PosPoint right_hand = { Right_Hand.X, Right_Hand.Y, Right_Hand.Z };
		XnPoint3D Torso = persontra.GetJointPostionReal(1, XN_SKEL_TORSO);
		PosPoint troso = { Torso.X, Torso.Y, Torso.Z };
		if (postion.Check(troso, right_hand))
		if (v.Checkvelocity(right_hand))
			cout << "激活" << endl;
	}
}
void Show()
{
	XtionPerson persontra = XtionPerson(true, true, true, true);

	if (persontra.Check())
		std::cout << "true" << std::endl;
	Mat mat;
	std::string  str = "test1";
	cvNamedWindow(str.c_str(), CV_WINDOW_AUTOSIZE);
	while (cvWaitKey(20) != 'a')
	{
		//更新数据  *****
		persontra.Update();
		//也可以只调用一次，不更新  ***
		persontra.CalcHist();
		//转换为Mat  *****
		persontra.DepthMDtoMat3();   //返回值  仅仅是深度数据转换为Mat
		//识别关节和用户状态   ****  
		persontra.GetJiont();
		//显示
		cv::imshow(str, persontra.GetFrame());
	}
}
int main()
{
	Show();
	//XtionPersonTrademo();
	//XtionGetPosition();
	//XtionMouse();
	//XtionPostion();
	//TakeOff();
	/*
	XtionPerson persontra = XtionPerson(true, true, true, true);
	if (persontra.Check())
		std::cout << "true" << std::endl;
	PersonMessage message = PersonMessage();
	XnPoint3D point1 = { 100, 200, 1000 };
	XnPoint3D point2 = { 200, 230, 1000 };
	float *p=message.Getangle(point1, point2);
	cout << p[0] << " " << p[1] << endl;
	//message.converttoflour(1,&point1);
	//cout << point1.Z << endl;
	*/
}
