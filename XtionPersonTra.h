
#ifdef _MSC_VER
#undef _MSC_VER
#define _MSC_VER 1500
#endif 

#include <XnOpenNI.h>
#include <XnCodecIDs.h>
#include <XnCppWrapper.h>
#include <XnPropNames.h>

#include <iostream>
#include <string>
#include <map>
//#include <vector>
//#include <opencv\cv.h>
//#include <opencv\highgui.h>
//#include <opencv\cxcore.hpp>
//#include <opencv2/objdetect/objdetect.hpp>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>

#include "cv_lib.h"

using namespace std;
using namespace cv;

#pragma comment(lib,"openNI.lib")
#pragma comment(lib,"NiSampleModule.lib")
#pragma comment(lib,"NiSampleExtensionModule.lib")

#pragma comment( lib,cvLIB("core"))
#pragma comment( lib,cvLIB("highgui"))
#pragma comment( lib,cvLIB("imgproc"))
#pragma comment(lib,cvLIB("video"))


XnFloat Colors[][3];
XnUInt32 nColors;

void  XN_CALLBACK_TYPE User_NewUser(xn::UserGenerator& , XnUserID nId, void* );
void  XN_CALLBACK_TYPE User_LostUser(xn::UserGenerator& , XnUserID nId, void* );
void  XN_CALLBACK_TYPE UserPose_PoseDetected(xn::PoseDetectionCapability&,const XnChar* strPose, XnUserID nId, void* );
void  XN_CALLBACK_TYPE UserCalibration_CalibrationStart(xn::SkeletonCapability& , XnUserID nId, void*);
void  XN_CALLBACK_TYPE UserCalibration_CalibrationComplete(xn::SkeletonCapability& , XnUserID nId, XnCalibrationStatus eStatus, void*);
void  XN_CALLBACK_TYPE MyCalibrationInProgress(xn::SkeletonCapability&, XnUserID id, XnCalibrationStatus calibrationError, void*);
void  XN_CALLBACK_TYPE MyPoseInProgress(xn::PoseDetectionCapability& , const XnChar*, XnUserID id, XnPoseDetectionStatus poseError, void*);
class XtionPerson
{
private:
	xn::Context g_Context;
	xn::ScriptNode g_scriptNode;
	xn::DepthGenerator g_DepthGenerator;                           //深度
	xn::Player g_Player;
	XnStatus nRetVal = XN_STATUS_OK;

	xn::SceneMetaData sceneMD;        //区域标志
	xn::DepthMetaData depthMD;        //深度数据
	float *pDepthHist;               //分布数据
	Mat frame;                      //当前的一帧
    VideoWriter videomatwriter;
	static xn::UserGenerator g_UserGenerator;                                //用户
	static XnBool g_bNeedPose ;
	static XnChar g_strPose[20];
	static map<XnUInt32, pair<XnCalibrationStatus, XnPoseDetectionStatus> > m_Errors;

	//用户关节
	map<XnUserID, map<XnSkeletonJoint, XnPoint3D>> JointsPostion;
	//所有深度  0----标志区域  1----深度数据
	//float  depthData[240][320][2];
	Mat    UsersMat;
	Mat    DepthMat;
	XnBool b_DrawJoints;
	XnBool b_DrawBackground;
	XnBool b_DrawJionsLine;
	XnBool b_DrawState;
	//人区域内的深度
	map<XnUserID, Mat> PersonDepth;

private:
#define XN_CALIBRATION_FILE_NAME "UserCalibration.bin"
#define SAMPLE_XML_PATH "SamplesConfig.xml"

	// Save calibration to file
	void SaveCalibration()
	{
		XnUserID aUserIDs[20] = { 0 };
		XnUInt16 nUsers = 20;
		g_UserGenerator.GetUsers(aUserIDs, nUsers);
		for (int i = 0; i < nUsers; ++i)
		{
			// Find a user who is already calibrated
			if (g_UserGenerator.GetSkeletonCap().IsCalibrated(aUserIDs[i]))
			{
				// Save user's calibration to file
				g_UserGenerator.GetSkeletonCap().SaveCalibrationDataToFile(aUserIDs[i], XN_CALIBRATION_FILE_NAME);
				break;
			}
		}
	}
	// Load calibration from file
	void LoadCalibration()
	{
		XnUserID aUserIDs[20] = { 0 };
		XnUInt16 nUsers = 20;
		g_UserGenerator.GetUsers(aUserIDs, nUsers);
		for (int i = 0; i < nUsers; ++i)
		{
			// Find a user who isn't calibrated or currently in pose
			if (g_UserGenerator.GetSkeletonCap().IsCalibrated(aUserIDs[i])) continue;
			if (g_UserGenerator.GetSkeletonCap().IsCalibrating(aUserIDs[i])) continue;

			// Load user's calibration from file
			XnStatus rc = g_UserGenerator.GetSkeletonCap().LoadCalibrationDataFromFile(aUserIDs[i], XN_CALIBRATION_FILE_NAME);
			if (rc == XN_STATUS_OK)
			{
				// Make sure state is coherent
				g_UserGenerator.GetPoseDetectionCap().StopPoseDetection(aUserIDs[i]);
				g_UserGenerator.GetSkeletonCap().StartTracking(aUserIDs[i]);
			}
			break;
		}
	}
	//check statu
	bool CheckStatus(XnStatus s, const string msg) {
		if (s != XN_STATUS_OK) {
			cout << msg << "failed :" << xnGetStatusString(s) << endl;
			return false;
		}
		return true;
	}
	const XnChar* GetCalibrationErrorString(XnCalibrationStatus error)
	{
		switch (error)
		{
		case XN_CALIBRATION_STATUS_OK:
			return "OK";
		case XN_CALIBRATION_STATUS_NO_USER:
			return "NoUser";
		case XN_CALIBRATION_STATUS_ARM:
			return "Arm";
		case XN_CALIBRATION_STATUS_LEG:
			return "Leg";
		case XN_CALIBRATION_STATUS_HEAD:
			return "Head";
		case XN_CALIBRATION_STATUS_TORSO:
			return "Torso";
		case XN_CALIBRATION_STATUS_TOP_FOV:
			return "Top FOV";
		case XN_CALIBRATION_STATUS_SIDE_FOV:
			return "Side FOV";
		case XN_CALIBRATION_STATUS_POSE:
			return "Pose";
		default:
			return "Unknown";
		}
	}
	const XnChar* GetPoseErrorString(XnPoseDetectionStatus error)
	{
		switch (error)
		{
		case XN_POSE_DETECTION_STATUS_OK:
			return "OK";
		case XN_POSE_DETECTION_STATUS_NO_USER:
			return "NoUser";
		case XN_POSE_DETECTION_STATUS_TOP_FOV:
			return "Top FOV";
		case XN_POSE_DETECTION_STATUS_SIDE_FOV:
			return "Side FOV";
		case XN_POSE_DETECTION_STATUS_ERROR:
			return "General error";
		default:
			return "Unknown";
		}
	}
	//绘制一个关节
	void DrawJoint(XnUserID player, XnSkeletonJoint eJoint)
	{
		XnPoint3D pt = GetJointPostionReal(player, eJoint);
		if (pt.Z == -1)
			return;

		g_DepthGenerator.ConvertRealWorldToProjective(1, &pt, &pt);
		
		JointsPostion[player].insert(pair<XnSkeletonJoint, XnPoint3D>(eJoint, pt));
        
		if (!b_DrawJoints)//不显示关节
			return;

		CvPoint point= cvPoint((int)pt.X, (int)pt.Y);		
		//cout << "x" << point.x << " y " << point.y << endl;
		if (point.x<314&&point.y<234)
		circle(frame, point, 2, Scalar(255, 255, 255), 2, 8, 0);
		//else
		//{
		//	Rect rect = Rect(point.x - 3, point.y, 3, 3);
			//rectangle(frame, rect, Scalar(255, 255, 255), 1, 8, 0);
		//}		
	}
	//绘制每一个关节
	void DrawJoints(XnUserID ID)
	{
		DrawJoint(ID, XN_SKEL_HEAD);
		DrawJoint(ID, XN_SKEL_NECK);
		DrawJoint(ID, XN_SKEL_TORSO);
		DrawJoint(ID, XN_SKEL_WAIST);

		DrawJoint(ID, XN_SKEL_LEFT_COLLAR);
		DrawJoint(ID, XN_SKEL_LEFT_SHOULDER);
		DrawJoint(ID, XN_SKEL_LEFT_ELBOW);
		DrawJoint(ID, XN_SKEL_LEFT_WRIST);
		DrawJoint(ID, XN_SKEL_LEFT_HAND);
		DrawJoint(ID, XN_SKEL_LEFT_FINGERTIP);

		DrawJoint(ID, XN_SKEL_RIGHT_COLLAR);
		DrawJoint(ID, XN_SKEL_RIGHT_SHOULDER);
		DrawJoint(ID, XN_SKEL_RIGHT_ELBOW);
		DrawJoint(ID, XN_SKEL_RIGHT_WRIST);
		DrawJoint(ID, XN_SKEL_RIGHT_HAND);
		DrawJoint(ID, XN_SKEL_RIGHT_FINGERTIP);

		DrawJoint(ID, XN_SKEL_LEFT_HIP);
		DrawJoint(ID, XN_SKEL_LEFT_KNEE);
		DrawJoint(ID, XN_SKEL_LEFT_ANKLE);
		DrawJoint(ID, XN_SKEL_LEFT_FOOT);

		DrawJoint(ID, XN_SKEL_RIGHT_HIP);
		DrawJoint(ID, XN_SKEL_RIGHT_KNEE);
		DrawJoint(ID, XN_SKEL_RIGHT_ANKLE);
		DrawJoint(ID, XN_SKEL_RIGHT_FOOT);
	}
	//绘制关节连接
	bool DrawLimb(XnUserID player, XnSkeletonJoint eJoint1, XnSkeletonJoint eJoint2)
	{
		if (!g_UserGenerator.GetSkeletonCap().IsTracking(player))
		{
			cout<<"not tracked!"<<endl;
			return true;
		}
		if (!g_UserGenerator.GetSkeletonCap().IsJointActive(eJoint1) ||
			!g_UserGenerator.GetSkeletonCap().IsJointActive(eJoint2))
		{
			return false;
		}
		XnSkeletonJointPosition joint1, joint2;
		g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(player, eJoint1, joint1);
		g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(player, eJoint2, joint2);

		if (joint1.fConfidence < 0.5 || joint2.fConfidence < 0.5)
		{
			return true;
		}
		XnPoint3D pt[2];
		pt[0] = joint1.position;
		pt[1] = joint2.position;
		g_DepthGenerator.ConvertRealWorldToProjective(2, pt,pt);
		//g_DepthGenerator.ConvertRealWorldToProjective(1, &pt[0], &pt[0]);
		//g_DepthGenerator.ConvertRealWorldToProjective(1, &pt[1], &pt[1]);
		Point point1 = Point((int)pt[0].X, (int)pt[0].Y);
		Point point2 = Point((int)pt[1].X, (int)pt[1].Y);
		if (point1.x > 316)
			point1.x = 316;
		if (point1.y > 236)
			point1.y = 236;
		if (point2.x > 316)
			point2.x = 316;
		if (point2.y > 236)
			point2.y = 236;
		line(frame, point1,point2, Scalar(0, 0, 0));
	}
	//绘制所有的关节连接
	void DrawJointsLine(XnUserID ID)
	{
		DrawLimb(ID, XN_SKEL_HEAD, XN_SKEL_NECK);

		DrawLimb(ID, XN_SKEL_NECK, XN_SKEL_LEFT_SHOULDER);
		DrawLimb(ID, XN_SKEL_LEFT_SHOULDER, XN_SKEL_LEFT_ELBOW);
		if (!DrawLimb(ID, XN_SKEL_LEFT_ELBOW, XN_SKEL_LEFT_WRIST))
		{
			DrawLimb(ID, XN_SKEL_LEFT_ELBOW, XN_SKEL_LEFT_HAND);
		}
		else
		{
			DrawLimb(ID, XN_SKEL_LEFT_WRIST, XN_SKEL_LEFT_HAND);
			DrawLimb(ID, XN_SKEL_LEFT_HAND, XN_SKEL_LEFT_FINGERTIP);
		}
		DrawLimb(ID, XN_SKEL_NECK, XN_SKEL_RIGHT_SHOULDER);
		DrawLimb(ID, XN_SKEL_RIGHT_SHOULDER, XN_SKEL_RIGHT_ELBOW);
		if (!DrawLimb(ID, XN_SKEL_RIGHT_ELBOW, XN_SKEL_RIGHT_WRIST))
		{
			DrawLimb(ID, XN_SKEL_RIGHT_ELBOW, XN_SKEL_RIGHT_HAND);
		}
		else
		{
			DrawLimb(ID, XN_SKEL_RIGHT_WRIST, XN_SKEL_RIGHT_HAND);
			DrawLimb(ID, XN_SKEL_RIGHT_HAND, XN_SKEL_RIGHT_FINGERTIP);
		}

		DrawLimb(ID, XN_SKEL_LEFT_SHOULDER, XN_SKEL_TORSO);
		DrawLimb(ID, XN_SKEL_RIGHT_SHOULDER, XN_SKEL_TORSO);

		DrawLimb(ID, XN_SKEL_TORSO, XN_SKEL_LEFT_HIP);
		DrawLimb(ID, XN_SKEL_LEFT_HIP, XN_SKEL_LEFT_KNEE);
		DrawLimb(ID, XN_SKEL_LEFT_KNEE, XN_SKEL_LEFT_FOOT);

		DrawLimb(ID, XN_SKEL_TORSO, XN_SKEL_RIGHT_HIP);
		DrawLimb(ID, XN_SKEL_RIGHT_HIP, XN_SKEL_RIGHT_KNEE);
		DrawLimb(ID, XN_SKEL_RIGHT_KNEE, XN_SKEL_RIGHT_FOOT);

		DrawLimb(ID, XN_SKEL_LEFT_HIP, XN_SKEL_RIGHT_HIP);
	}
	bool  GetState(int user)
	{
		if (g_UserGenerator.GetSkeletonCap().IsTracking(user))
			return true;
		else
			return false;
	}
public:
	//DrawStates 显示状态 DrawJoints 绘制关节 DrawJionsLine 绘制关节连线  DrawBackground 绘制背景
	XtionPerson(XnBool DrawStates = true, XnBool DrawJoints = true,
		XnBool DrawJionsLine = true, XnBool DrawBackground = true)
	{
		b_DrawJoints = DrawJoints;
		b_DrawBackground = DrawBackground;
		b_DrawJionsLine = DrawJionsLine;
		b_DrawState = DrawStates;
	}
	~XtionPerson()
	{
		g_scriptNode.Release();
		g_DepthGenerator.Release();
		g_UserGenerator.Release();
		g_Player.Release();
		g_Context.Release();
		videomatwriter.release();
	}
	static xn::UserGenerator GetUserGenerator()
	{
		return g_UserGenerator;
	}
	//不用管
	static XnBool GetNeedPose()
	{
		return g_bNeedPose;
	}
	//不用管
	static void SetNeedPose(XnBool needpose)
	{
		g_bNeedPose = needpose;
	}
	//不用管
	static XnChar* GetstrPose()
	{
		return g_strPose;
	}
	//不用管
	static map<XnUInt32, pair<XnCalibrationStatus, XnPoseDetectionStatus> > GetErrors()
	{
		return m_Errors;
	}
	
	
	
	//获取 Mat RGB数据  条件:调用过DepthMDtoMat3
	Mat  GetFrame()
	{
		return frame;
	}
	//检测是否设备可用 注册相关的检测回调函数   条件:用于检测人  
	bool Check()
	{
		xn::EnumerationErrors errors;
		nRetVal = g_Context.InitFromXmlFile(SAMPLE_XML_PATH, g_scriptNode, &errors);
		if (nRetVal == XN_STATUS_NO_NODE_PRESENT)
		{
			XnChar strError[1024];
			errors.ToString(strError, 1024);
			//printf("%s\n", strError);
			cout << strError << endl;
			return false;
		}
		else if (nRetVal != XN_STATUS_OK)
		{
			//printf("Open failed: %s\n", xnGetStatusString(nRetVal));
			cout << "Open Failed :" << xnGetStatusString(nRetVal) << endl;
			return false;
		}

		nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_DEPTH, g_DepthGenerator);
		if (nRetVal != XN_STATUS_OK)
		{
			//printf("No depth generator found. Using a default one...");
			cout << "No depth generator found. Using a default one..." << endl;

			xn::MockDepthGenerator mockDepth;
			nRetVal = mockDepth.Create(g_Context);
			if (!CheckStatus(nRetVal, "Create mock depth"))
				return false;

			// set some defaults
			XnMapOutputMode defaultMode;
			defaultMode.nXRes = 320;
			defaultMode.nYRes = 240;
			defaultMode.nFPS = 30;
			nRetVal = mockDepth.SetMapOutputMode(defaultMode);
			if (!CheckStatus(nRetVal, "set default mode"))
				return false;

			// set FOV
			XnFieldOfView fov;
			fov.fHFOV = 1.0225999419141749;
			fov.fVFOV = 0.79661567681716894;
			nRetVal = mockDepth.SetGeneralProperty(XN_PROP_FIELD_OF_VIEW, sizeof(fov), &fov);
			if (!CheckStatus(nRetVal, "set FOV"))
				return false;

			XnUInt32 nDataSize = defaultMode.nXRes * defaultMode.nYRes * sizeof(XnDepthPixel);
			XnDepthPixel* pData = (XnDepthPixel*)xnOSCallocAligned(nDataSize, 1, XN_DEFAULT_MEM_ALIGN);

			nRetVal = mockDepth.SetData(1, 0, nDataSize, pData);
			if (!CheckStatus(nRetVal, "set empty depth map"))
				return false;

			g_DepthGenerator = mockDepth;                   //defaut  深度
		}

		nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_USER, g_UserGenerator);
		if (nRetVal != XN_STATUS_OK)
		{
			nRetVal = g_UserGenerator.Create(g_Context);
			if (!CheckStatus(nRetVal, "Find user generator"))
				return false;
		}

		XnCallbackHandle hUserCallbacks, hCalibrationStart, hCalibrationComplete, hPoseDetected, hCalibrationInProgress, hPoseInProgress;
		if (!g_UserGenerator.IsCapabilitySupported(XN_CAPABILITY_SKELETON))
		{
			//printf("Supplied user generator doesn't support skeleton\n");
			cout << "Supplied user generator doesn't support skeleton" << endl;
			return false;
		}
		nRetVal = g_UserGenerator.RegisterUserCallbacks(User_NewUser,User_LostUser,NULL, hUserCallbacks);
		if (!CheckStatus(nRetVal, "Register to user callbacks"))		
			return false;	
		nRetVal = g_UserGenerator.GetSkeletonCap().RegisterToCalibrationStart(UserCalibration_CalibrationStart, NULL, hCalibrationStart);
		if(!CheckStatus(nRetVal, "Register to calibration start"))
			return false;
		nRetVal = g_UserGenerator.GetSkeletonCap().RegisterToCalibrationComplete(UserCalibration_CalibrationComplete, NULL, hCalibrationComplete);
		if(!CheckStatus(nRetVal, "Register to calibration complete"))
			return false;
		
		if (g_UserGenerator.GetSkeletonCap().NeedPoseForCalibration())
		{
			g_bNeedPose = TRUE;
			if (!g_UserGenerator.IsCapabilitySupported(XN_CAPABILITY_POSE_DETECTION))
			{
				cout<<"Pose required, but not supported"<<endl;
				return false;
			}
			nRetVal = g_UserGenerator.GetPoseDetectionCap().RegisterToPoseDetected(UserPose_PoseDetected, NULL, hPoseDetected);
			if(!CheckStatus(nRetVal, "Register to Pose Detected"))
				return false;
			g_UserGenerator.GetSkeletonCap().GetCalibrationPose(g_strPose);
			nRetVal = g_UserGenerator.GetPoseDetectionCap().RegisterToPoseInProgress(MyPoseInProgress, NULL, hPoseInProgress);
			if(!CheckStatus(nRetVal, "Register to pose in progress"))
				return false;
		}
		
		g_UserGenerator.GetSkeletonCap().SetSkeletonProfile(XN_SKEL_PROFILE_ALL);

		nRetVal = g_UserGenerator.GetSkeletonCap().RegisterToCalibrationInProgress(MyCalibrationInProgress, NULL, hCalibrationInProgress);
		if(!CheckStatus(nRetVal, "Register to calibration in progress"))
			return false;

		nRetVal = g_Context.StartGeneratingAll();
		if(!CheckStatus(nRetVal, "StartGenerating"))
			return false;

		//cout << "end" << endl;
		return true;
	}     
	//更新深度数据
	void Update()
	{
		// Read next available data
		g_Context.WaitOneUpdateAll(g_UserGenerator);
		g_DepthGenerator.GetMetaData(depthMD);
		g_UserGenerator.GetUserPixels(0, sceneMD);
	}
	//计算分布
	void CalcHist()
	{
		unsigned int nNumberOfPoints = 0;
		const XnDepthPixel* pDepth = depthMD.Data();
		//const XnLabel* pLabels = sceneMD.Data();
		unsigned int YMax = depthMD.YRes();
		unsigned int XMax = depthMD.XRes();
		unsigned int ZMax = depthMD.ZRes();
		unsigned int nValue = 0;
	    pDepthHist = (float*)malloc(depthMD.ZRes()* sizeof(float));
		memset(pDepthHist, 0, depthMD.ZRes()*sizeof(float));
		for (unsigned int nY = 0; nY < YMax; nY++)
		{
			for (unsigned int nX = 0; nX<XMax; nX++)
			{
				nValue = *pDepth;
				if (nValue != 0)
				{
					pDepthHist[nValue]++;
					nNumberOfPoints++;
				}
				pDepth++;
			}
		}
		for (unsigned int nIndex = 1; nIndex<ZMax; nIndex++)
		{
			pDepthHist[nIndex] += pDepthHist[nIndex - 1];
		}	
		if (nNumberOfPoints)
		{
			for (unsigned int nIndex = 1; nIndex<ZMax; nIndex++)
			{
				pDepthHist[nIndex] = (unsigned int)(256 * (1.0f - (pDepthHist[nIndex] / nNumberOfPoints)));
			}
		}
	}
	//深度数据转换为RGB图像  条件:调用CalcHist 
	Mat DepthMDtoMat3()
	{
		unsigned int YMax = depthMD.YRes();
		unsigned int XMax = depthMD.XRes();
		unsigned int ZMax = depthMD.ZRes();
		unsigned short nValue;        //深度
		unsigned int nHistValue;    //对应颜色值	 

		unsigned short tempuser[240][320];  //XnLabel unsigned short 
		unsigned short   tempdepth[240][320];  //XnUInt32 unsigned short  10000  0-65535
		uchar tempdata[240][320][3];
		memset(tempuser, 0, sizeof(tempuser));
		memset(tempdepth, 0, sizeof(tempdepth));
		memset(tempdata, 0, sizeof(tempdata));


		const XnDepthPixel* pDepth = depthMD.Data();
		const XnLabel* pLabels = sceneMD.Data();
		XnLabel label;    //sign
		XnUInt32 nColorID;
		
		for (unsigned int nY = 0; nY < YMax; nY++)
		{
			for (unsigned int nX = 0; nX < XMax; nX++)
			{			
				if (!b_DrawBackground&&*pLabels == 0)
				{
					pDepth++;
					pLabels++;
					continue;
				}
				//if (b_DrawBackground||*pLabels != 0)
				{
					nValue = *pDepth;
					label = *pLabels;
					nColorID = label % nColors;
					if (label == 0)
					{
						nColorID = nColors;
					}
					if (nValue != 0)
					{
						nHistValue = pDepthHist[nValue];
						tempdata[nY][nX][0] = (uchar)nHistValue * Colors[nColorID][0];
						tempdata[nY][nX][1] = (uchar)nHistValue * Colors[nColorID][1];
						tempdata[nY][nX][2] = (uchar)nHistValue * Colors[nColorID][2];
					}
					//保存深度值和标志
					tempuser[nY][nX] = label;
					tempdepth[nY][nX] = nValue;

					pDepth++;
					pLabels++;
				}			
			}
		}
		frame = Mat(240, 320, CV_8UC3, tempdata);
		UsersMat = Mat(240, 320, CV_16UC1, tempuser);
		DepthMat = Mat(240, 320, CV_16UC1, tempdepth);
		return frame;
	}
	//获取状态和骨骼信息  条件:先调用DepthMDtoMat3  功能：获取用户数据
	void GetJiont()
	{
		char strLabel[50] = "";
		XnUInt32 nDummy = 0;
		XnUserID aUsers[15];
		XnUInt16 nUsers = 15;
		g_UserGenerator.GetUsers(aUsers, nUsers);
		//清除JointsPosition的数据
		if (!JointsPostion.empty())
			JointsPostion.clear();
		//所有用户的关节检测
		for (int i = 0; i < nUsers; ++i)
		{
			XnPoint3D com;
			g_UserGenerator.GetCoM(aUsers[i], com);
			g_DepthGenerator.ConvertRealWorldToProjective(1, &com, &com);
			xnOSMemSet(strLabel, 0, sizeof(strLabel));
			if (!b_DrawState)//!g_bPrintState)  添加变量判断
			{
				// Tracking
				xnOSStrFormat(strLabel, sizeof(strLabel), &nDummy, "%d", aUsers[i]);
			}
			else if (g_UserGenerator.GetSkeletonCap().IsTracking(aUsers[i]))
			{
				// Tracking
				xnOSStrFormat(strLabel, sizeof(strLabel), &nDummy, "%d - Tracking", aUsers[i]);
			}
			else if (g_UserGenerator.GetSkeletonCap().IsCalibrating(aUsers[i]))
			{
				// Calibrating
				xnOSStrFormat(strLabel, sizeof(strLabel), &nDummy, "%d - Calibrating [%s]", aUsers[i], GetCalibrationErrorString(m_Errors[aUsers[i]].first));
			}
			else
			{
				// Nothing
				xnOSStrFormat(strLabel, sizeof(strLabel), &nDummy, "%d - Looking for pose [%s]", aUsers[i], GetPoseErrorString(m_Errors[aUsers[i]].second));
			}
			CvPoint point;
			point.x = com.X;
			point.y = com.Y;
			cv::putText(frame, strLabel, point, 2, 0.5, CV_RGB(255 * Colors[i % 10][0], 255 * Colors[i % 10][1], 255 * Colors[i % 10][2]));
			if (g_UserGenerator.GetSkeletonCap().IsTracking(aUsers[i]))
			{ 
				DrawJoints(aUsers[i]);
				if (b_DrawJionsLine)
				DrawJointsLine(aUsers[i]);	
			}
		}
	}
	//创建VideoWriter
	void WriteVideoMat(string str)
	{	
		videomatwriter.open(str, CV_FOURCC('D', 'I', 'V', 'X'), 30, Size(320, 240), true);
	}
	//写入每一帧  条件:先调用WriteVideoMat
	void  WriteMat()
	{
		videomatwriter << frame;
	}	
	//获取某个关节图像坐标  
	XnPoint3D GetJointPostionReal(XnUserID player, XnSkeletonJoint eJoint/*关节名称*/)
	{
		XnPoint3D temp = { -1, -1, -1 };
		if (!g_UserGenerator.GetSkeletonCap().IsTracking(player))
		{
			//cout << "not tracked!" << endl;
			return temp;
		}
		if (!g_UserGenerator.GetSkeletonCap().IsJointActive(eJoint))
		{
			return temp;
		}
		XnSkeletonJointPosition joint;
		g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(player, eJoint, joint);
		if (joint.fConfidence < 0.5)
		{
			return temp;
		}
		temp = joint.position;
		return temp;		
	}
	//转换到现实世界坐标
	XnPoint3D ConvertToReal(XnPoint3D point)
	{
		XnPoint3D temp=point;
		g_DepthGenerator.ConvertProjectiveToRealWorld(1, &temp, &temp);
		return temp;
	}
	XnPoint3D* ConvertToReal(XnPoint3D point[], int num)
	{
		g_DepthGenerator.ConvertProjectiveToRealWorld(num, point, point);
		return point;
	}
	XnPoint3D ConvertToProject(XnPoint3D point)
	{
		XnPoint3D temp = point;
		g_DepthGenerator.ConvertRealWorldToProjective(1, &temp, &temp);
		return temp;
	}
	XnPoint3D* ConvertToProject(XnPoint3D point[], int num)
	{
		g_DepthGenerator.ConvertRealWorldToProjective(num, point, point);
		return point;
	}
	//获取用户标志Mat
	Mat GetUserMat()
	{
		Mat tempuse = Mat(UsersMat);
		return tempuse;
	}
	//深度Mat
	Mat GetDepthMat()
	{
		Mat tempdep = Mat(DepthMat);
		return tempdep;
	}
	/*
	map<XnUserID, Mat> ConVertToMap()
	{
		int nPerson = GetnPerson();
		PersonDepth.clear();
		if (nPerson == 0)
			return PersonDepth;
		//for (int i = 0; i < nPerson; i++)
		//{
		//	if (g_UserGenerator.GetSkeletonCap().IsTracking(i))
		//		return PersonDepth;
		//}

		int num=1;
		//先清空历史数据
		class temp{
		public:
			float data[240][320];
			temp()
			{
				memset(data, 0, sizeof(data));
			}
		};
		temp* tempdata = new temp[nPerson];
		for (int i = 0; i < nPerson; i++)
		{
			tempdata[i] = temp();
		}
		for (int i = 0; i < 240; i++)
		{
			for (int j = 0; j < 320; j++)
			{
				if (depthData[i][j][0] == 0)
					continue;
				else
				{
					num = depthData[i][j][0];
					if (num>nPerson)
						return PersonDepth;
					tempdata[num - 1].data[i][j] = depthData[i][j][1];
				}
			}
		}
		for (int i = 0; i < nPerson; i++)
		{
			Mat tempmat = Mat(240, 320, CV_32FC1, tempdata[i].data);
			PersonDepth.insert(pair<XnUserID, Mat>(i+1, tempmat));

			cout << "insert" << i << "   "<<tempmat.ptr<float>(100,100)[0] << endl;
		}
		delete[] tempdata;
		return PersonDepth;
	}
	*/
	int GetnPerson()
	{
		XnUserID aUsers[15];
		XnUInt16 nUsers = 15;
		g_UserGenerator.GetUsers(aUsers, nUsers);
		//保存用户数量
		return nUsers;
	}
	Mat GetUsersDepthMat(int user)
	{
		unsigned short temp[240][320];
		//bool  Tracking = false;
		Mat   tempmat; 
		memset(temp, 0, sizeof(temp));
		MatIterator_<unsigned short> itdep = DepthMat.begin<unsigned short>(), itdep_end = DepthMat.end<unsigned short>();
		MatIterator_<unsigned short> ituse = UsersMat.begin<unsigned short>(), ituse_end = UsersMat.end<unsigned short>();
		//MatIterator_<float> ittemp = tempmat.begin<float>(), ittemp_end = tempmat.end<float>();
		for (int i=0; ituse != ituse_end; ++itdep, ++ituse,i++)
		{
			if (unsigned short(*ituse) == (unsigned short)user)
				temp[(int)(i / 320)][(int)(i % 320)] = (unsigned short)*itdep;
		}
		tempmat = Mat(240, 320, CV_16UC1,temp );
		return tempmat;
	}
};
//颜色值
XnFloat Colors[][3] =
{
	{ 0, 1, 1 },
	{ 0, 0, 1 },
	{ 0, 1, 0 },
	{ 1, 1, 0 },
	{ 1, 0, 0 },
	{ 1, .5, 0 },
	{ .5, 1, 0 },
	{ 0, .5, 1 },
	{ .5, 0, 1 },
	{ 1, 1, .5 },
	{ 1, 1, 1 }
};


//使用颜色数量
extern XnUInt32 nColors = 10;
//做声明
XnBool XtionPerson::g_bNeedPose;
xn::UserGenerator XtionPerson::g_UserGenerator;
XnChar XtionPerson::g_strPose[20];
map<XnUInt32, pair<XnCalibrationStatus, XnPoseDetectionStatus> > XtionPerson::m_Errors;

// Callback: New user was detected
void XN_CALLBACK_TYPE User_NewUser(xn::UserGenerator& /*generator*/, XnUserID nId, void* /*pCookie*/)
{
	XnUInt32 epochTime = 0;
	xnOSGetEpochTime(&epochTime);
	//printf("%d New User %d\n", epochTime, nId);
	cout << epochTime << " New User " << nId << endl;
	// New user found
	if (XtionPerson::GetNeedPose())
	{
		XtionPerson::GetUserGenerator().GetPoseDetectionCap().StartPoseDetection(XtionPerson::GetstrPose(), nId);
	}
	else
	{
		XtionPerson::GetUserGenerator().GetSkeletonCap().RequestCalibration(nId, TRUE);
	}
}
// Callback: An existing user was lost
void XN_CALLBACK_TYPE User_LostUser(xn::UserGenerator& /*generator*/, XnUserID nId, void* /*pCookie*/)
{
	XnUInt32 epochTime = 0;
	xnOSGetEpochTime(&epochTime);
	//printf("%d Lost user %d\n", epochTime, nId);
	cout << epochTime << " Lost User " << nId << endl;
}
// Callback: Detected a pose
void XN_CALLBACK_TYPE UserPose_PoseDetected(xn::PoseDetectionCapability& /*capability*/, const XnChar* strPose, XnUserID nId, void* /*pCookie*/)
{
	XnUInt32 epochTime = 0;
	xnOSGetEpochTime(&epochTime);
	//printf("%d Pose %s detected for user %d\n", epochTime, strPose, nId);
	cout << epochTime << " Pose " << strPose << " detected for user " << nId << endl;
	XtionPerson::GetUserGenerator().GetPoseDetectionCap().StopPoseDetection(nId);
	XtionPerson::GetUserGenerator().GetSkeletonCap().RequestCalibration(nId, TRUE);
}
// Callback: Started calibration
void XN_CALLBACK_TYPE UserCalibration_CalibrationStart(xn::SkeletonCapability& /*capability*/, XnUserID nId, void* /*pCookie*/)
{
	XnUInt32 epochTime = 0;
	xnOSGetEpochTime(&epochTime);
	//printf("%d Calibration started for user %d\n", epochTime, nId);
	cout << epochTime << " Calibration started for user " << nId << endl;
}
// Callback: Finished calibration
void XN_CALLBACK_TYPE UserCalibration_CalibrationComplete(xn::SkeletonCapability& /*capability*/, XnUserID nId, XnCalibrationStatus eStatus, void* /*pCookie*/)
{
	XnUInt32 epochTime = 0;
	xnOSGetEpochTime(&epochTime);
	if (eStatus == XN_CALIBRATION_STATUS_OK)
	{
		// Calibration succeeded
		//printf("%d Calibration complete, start tracking user %d\n", epochTime, nId);
		cout << epochTime << " Calibration complete, start tracking user " << nId << endl;
		XtionPerson::GetUserGenerator().GetSkeletonCap().StartTracking(nId);
	}
	else
	{
		// Calibration failed
		//printf("%d Calibration failed for user %d\n", epochTime, nId);
		cout << epochTime << " Calibration failed for user " << nId << endl;
		if (eStatus == XN_CALIBRATION_STATUS_MANUAL_ABORT)
		{
			//printf("Manual abort occured, stop attempting to calibrate!");
			cout << "Manual abort occured, stop attempting to calibrate!" << endl;
			return;
		}
		if (XtionPerson::GetNeedPose())
		{
			XtionPerson::GetUserGenerator().GetPoseDetectionCap().StartPoseDetection(XtionPerson::GetstrPose(), nId);
		}
		else
		{
			XtionPerson::GetUserGenerator().GetSkeletonCap().RequestCalibration(nId, TRUE);
		}
	}
}
void XN_CALLBACK_TYPE MyCalibrationInProgress(xn::SkeletonCapability& /*capability*/, XnUserID id, XnCalibrationStatus calibrationError, void* /*pCookie*/)
{
	XtionPerson::GetErrors()[id].first = calibrationError;
}
void XN_CALLBACK_TYPE MyPoseInProgress(xn::PoseDetectionCapability& /*capability*/, const XnChar* /*strPose*/, XnUserID id, XnPoseDetectionStatus poseError, void* /*pCookie*/)
{
	XtionPerson::GetErrors()[id].second = poseError;
}






