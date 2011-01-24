/****************************************************************************
*                                                                           *
*   Nite 1.3 - Stick Figure Sample                                          *
*                                                                           *
*   Author:     Oz Magal, Ilan Atias, Alon Haber                            *
*   Based on PrimeSense EE Sample 3DViewer, by Ziv Hendel                   *
*                                                                           *
****************************************************************************/

/****************************************************************************
*                                                                           *
*   Nite 1.3	                                                            *
*   Copyright (C) 2006 PrimeSense Ltd. All Rights Reserved.                 *
*                                                                           *
*   This file has been provided pursuant to a License Agreement containing  *
*   restrictions on its use. This data contains valuable trade secrets      *
*   and proprietary information of PrimeSense Ltd. and is protected by law. *
*                                                                           *
****************************************************************************/

// Application Usage:
// 1 � Top right perspective camera view.
// 2 � Front camera view.
// 3 � Top camera view.
// 4 � Side camera view.
// s � Enable/Disable stick figure.
// p � Show the laser pointer and the cutoff parameters.
// m � Enable/Disable the mirror mode.
// c � Recalibrate first user (White user).
// C - Recalibrate second user (Black user).
// z - Enable/Disable 3D points view.
// o - Enable/Disable orientation drawing.
// b - Enable/Disable background.
// k - Start/Stop recording. (to Data\recording.xns)
// q � Decrease the minimum depth cutoff by 1.
// Q � Decrease the minimum depth cutoff by 50.
// w � Increase the minimum depth cutoff by 1.
// W � Increase the minimum depth cutoff by 50.
// e � Decrease the maximum depth cutoff by 1.
// E � Decrease the maximum depth cutoff by 50.
// r � Increase the maximum depth cutoff by 1.
// R � Increase the maximum depth cutoff by 50.
// + - Increase the GL point size by 1.
// - - Decrease the GL point size by 1.
// ESC � exit.

// --------------------------------
// Includes
// --------------------------------

#include <XnOpenNI.h>
#include <XnList.h>
#include <XnCppWrapper.h>
#include <XnCodecIDs.h>

#define GLH_EXT_SINGLE_FILE
#include "glh/glh_extensions.h"
#include "glh/glh_obs.h"
#include "glh/glh_glut2.h"
#include "glh/glh_glut_text.h"

using namespace glh;

// [parag]: OSC stuff
// --------------------------------

#include "ofxOsc.h"

#define HOST "localhost"
#define PORT 12345

bool bSendOSC = true;

ofxOscSender sender;
void sendOSCMarker(string prefix, XnSkeletonJointPosition &pos, XnSkeletonJointOrientation &ori);

#ifndef max(a,b)
#define max(a,b) (a) > (b) ? (a) : (b)
#endif

// --------------------------------
// Defines
// --------------------------------
#define GLSX -1.0
#define GLSY -1.0
#define GLSZ 1.0
#define GLUX 2.0 
#define GLVY 2.0
#define WIN_SIZE_X 1280
#define WIN_SIZE_Y 1024

// [parag]: I've moved the sample data files into the root of the project folder in a 
// directory called Data
#define RECORD_FILE		"../../../Data/Recording.oni"
#define SAMPLE_XML_PATH "../../../Data/Sample-User.xml"

// --------------------------------
// Global Variables
// --------------------------------
glut_callbacks cb;
glut_simple_mouse_interactor camera, light, room, object;
glut_perspective_reshaper reshaper;
display_list face;
tex_object_2D wall;

bool b[256];

vec4f light_position(1,1,1,1);
float room_ambient = .4;

bool bDoRGBFix = true;
bool bStaticFilter = false;
bool bRealWorld = true;
bool bShowPointer = false;
bool g_bDrawOrientations = false;
bool g_bPrintFrameID = false;
int nGLPointSize = 5;

bool g_bDrawPointCloud = true;
bool g_verbose = false;

bool g_bRecord = false;
xn::Recorder* g_pRecorder;

int nPointerX;
int nPointerY;
int nPointerDiffX;
int nPointerDiffY;
XnUInt64 nCutOffMin;
XnUInt64 nCutOffMax;

short* PointCloudShadowArr;

float fXRes;
float fYRes;
float fMaxDepth;

xn::Context g_Context;
xn::DepthGenerator g_DepthGenerator;
xn::UserGenerator g_UserGenerator;

double g_LastSmoothingFactor = 0.0;

XnBool	g_bCalibrate1 = false, g_bCalibrate2 = false;
XnBool g_bShowBG = false;
XnBool g_IsSkeletonEnabled = true;


// --------------------------------
// Function Declaration
// --------------------------------
void display();
void key(unsigned char k, int x, int y);
void draw_pointcloud();
void draw_mesh(float size, int tess);
void draw_cube();
void draw_room(bool do_diffuse);
void idle();
void init_opengl();
void init_model();
void glPrintString(void *font, char *str);

// --------------------------------
// Code
// --------------------------------
void glPrintString(void *font, char *str)
{
   int i,l = strlen(str);

   for(i=0; i<l; i++)
   {
      glutBitmapCharacter(font,*str++);
   }
}

void MotionCallback(int x, int y)
{
	nPointerX = x / nPointerDiffX;
	nPointerY = y / nPointerDiffY;
}


// [parag]: I completely hacked this i had no idea what the original return values were all about
// but now they are just int and void *... this does crash occasionally when there 
// are multiple skeletons being tracked!!!
/*
 DWORD WINAPI EEThreadProc(LPVOID lpThreadParameter)
 {
	{
		g_Context.WaitAndUpdateAll();
		if (g_bRecord)
		{
			g_pRecorder->Record();
		}
	}
	return 0;
 }
 */
int EEThreadProc(void *lpThreadParameter)
{
	{
		g_Context.WaitAndUpdateAll();
		if (g_bRecord)
		{
			g_pRecorder->Record();
		}
	}
	return 0;
}

void XN_CALLBACK_TYPE NewUser(xn::UserGenerator& generator, XnUserID user, void* pCookie)
{
	printf("New user identified: %d\n", user);
	g_UserGenerator.GetPoseDetectionCap().StartPoseDetection("Psi", user);
}

void XN_CALLBACK_TYPE LostUser(xn::UserGenerator& generator, XnUserID user, void* pCookie)
{
	printf("User %d lost\n", user);
}

void XN_CALLBACK_TYPE CalibrationStart(xn::SkeletonCapability& skeleton, XnUserID user, void* pCookie)
{
	printf("Calibration start for user %d\n", user);
}
void XN_CALLBACK_TYPE CalibrationEnd(xn::SkeletonCapability& skeleton, XnUserID user, XnBool bSuccess, void* pCookie)
{
	printf("Calibration complete for user %d: %s\n", user, bSuccess?"Success":"Failure");
	if (bSuccess)
	{
		skeleton.StartTracking(user);
	}
	else
	{
		g_UserGenerator.GetPoseDetectionCap().StartPoseDetection("Psi", user);
	}
}
void XN_CALLBACK_TYPE PoseDetected(xn::PoseDetectionCapability& poseDetection, const XnChar* strPose, XnUserID nId, void* pCookie)
{
	printf("Pose '%s' detected for user %d\n", strPose, nId);
	g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, FALSE);
	g_UserGenerator.GetPoseDetectionCap().StopPoseDetection(nId);
}

#define CHECK_RC(rc, what)											\
	if (rc != XN_STATUS_OK)											\
	{																\
		printf("%s failed: %s\n", what, xnGetStatusString(rc));		\
		return rc;													\
	}


#include <XnLog.h>
int main(int argc, char **argv)
{
	sender.setup(HOST, PORT);
	
	XnStatus rc = XN_STATUS_OK;

	rc = g_Context.InitFromXmlFile(SAMPLE_XML_PATH);
	CHECK_RC(rc, "InitFromXml");

	rc = g_Context.FindExistingNode(XN_NODE_TYPE_DEPTH, g_DepthGenerator);
	CHECK_RC(rc, "Find depth generator");
	rc = g_Context.FindExistingNode(XN_NODE_TYPE_USER, g_UserGenerator);
	CHECK_RC(rc, "Find user generator");

	XnCallbackHandle h;

	g_UserGenerator.RegisterUserCallbacks(NewUser, LostUser, NULL, h);
	g_UserGenerator.GetSkeletonCap().SetSkeletonProfile(XN_SKEL_PROFILE_ALL);

	XnCallbackHandle hCalib;
	XnCallbackHandle hPose;
	g_UserGenerator.GetSkeletonCap().RegisterCalibrationCallbacks(&CalibrationStart, &CalibrationEnd, NULL, hCalib);
	g_UserGenerator.GetPoseDetectionCap().RegisterToPoseCallbacks(&PoseDetected, NULL, NULL, hPose);

	rc = g_Context.StartGeneratingAll();
	CHECK_RC(rc, "StartGenerating");

	xn::DepthMetaData depthMD;
	g_DepthGenerator.GetMetaData(depthMD);

	fXRes = depthMD.XRes();
	fYRes = depthMD.YRes();
	fMaxDepth = depthMD.ZRes();
	
	nCutOffMin = 0;
	nCutOffMax = fMaxDepth;

	nPointerX = fXRes / 2;
	nPointerY = fXRes / 2;
	nPointerDiffX = (WIN_SIZE_X / fXRes / 2) + 1;
	nPointerDiffY = (WIN_SIZE_Y / fXRes / 2) + 1;

	if (argc == 2)
	{
		nCutOffMax = atol(argv[1]);
	}

	srand(XnUInt32(time(NULL)));

	glutInit(&argc, argv);
	glutInitDisplayString("stencil depth>16 double rgb samples=0");
    glutInitWindowSize(WIN_SIZE_X, WIN_SIZE_Y);
	glutCreateWindow("Prime Sense Stick Figure Sample");
	glutSetCursor(GLUT_CURSOR_NONE);

	init_opengl();

	glut_helpers_initialize();

	cb.passive_motion_function = MotionCallback;
	cb.keyboard_function = key;

	camera.configure_buttons(0);
	camera.set_camera_mode(true);
	camera.set_parent_rotation( & camera.trackball.r);
	camera.enable();

	object.configure_buttons(1);
	object.translator.t[2] = -1;
	object.translator.scale *= .1f;
	object.trackball.r = rotationf(vec3f(2.0,0.01,0.01), to_radians(180));
	object.set_parent_rotation( & camera.trackball.r);
	object.disable();

	light.configure_buttons(0);
	light.translator.t = vec3f(.5, .5, -1);
	light.set_parent_rotation( & camera.trackball.r);
	light.disable();

	reshaper.zNear = 1;
	reshaper.zFar = 100;

    // make sure all interactors get glut events
	glut_add_interactor(&cb);
	glut_add_interactor(&camera);
	glut_add_interactor(&reshaper);
	glut_add_interactor(&light);
	glut_add_interactor(&object);

	camera.translator.t = vec3f(0, 0, 0);
	camera.trackball.r = rotationf(vec3f(0, 0, 0), to_radians(0));

	light.translator.t = vec3f (0, 1.13, -2.41);
	light.trackball.r = rotationf(vec3f(0.6038, -0.1955, -0.4391), to_radians(102));

	glutIdleFunc(idle);
	glutDisplayFunc(display);
	// Per frame code is in display
	glutMainLoop();


	return (0);
}

void init_opengl()
{
	glClearStencil(128);
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LESS);
	glEnable(GL_NORMALIZE);

	glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_FALSE);
	GLfloat ambient[] = {0.5, 0.5, 0.5, 1};
	glLightfv(GL_LIGHT0, GL_AMBIENT, ambient);
	glLightf (GL_LIGHT0, GL_QUADRATIC_ATTENUATION, 0.1);
	face.new_list(GL_COMPILE);
	draw_mesh(20, 40);
	face.end_list();

	wall.bind();
	glTexParameteri(GL_TEXTURE_2D, GL_GENERATE_MIPMAP_SGIS, GL_TRUE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	GLfloat * tex = new GLfloat[32*32];
	{
		for(int i=0; i < 32; i++)
		{
			for(int j=0; j < 32; j++)
			{
				if(i>>4 ^ j>>4)
					tex[i+j*32] = 1;
				else
					tex[i+j*32] = .9;
			}
		}
	}
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, 32, 32, 0, GL_LUMINANCE, GL_FLOAT, tex);
	delete [] tex;
}



void idle()
{
	// [parag]: What is this function!?
	EEThreadProc(NULL);
	
	
	glutPostRedisplay();
}

void StopCapture()
{
	g_bRecord = false;
	if (g_pRecorder != NULL)
	{
		g_pRecorder->RemoveNodeFromRecording(g_DepthGenerator);
		g_pRecorder->Unref();
		delete g_pRecorder;
	}
	g_pRecorder = NULL;
}
#define START_CAPTURE_CHECK_RC(rc, what)												\
	if (nRetVal != XN_STATUS_OK)														\
{																					\
	printf("Failed to %s: %s\n", what, xnGetStatusString(rc));				\
	StopCapture();															\
	return ;																	\
}
void StartCapture()
{
	char recordFile[256] = {0};
	time_t rawtime;
	struct tm *timeinfo;

	time(&rawtime);
	timeinfo = localtime(&rawtime);
	XnUInt32 size;
	xnOSStrFormat(recordFile, sizeof(recordFile)-1, &size,
		"%d_%02d_%02d[%02d_%02d_%02d].oni",
		timeinfo->tm_year + 1900, timeinfo->tm_mon + 1, timeinfo->tm_mday, timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);

	if (g_pRecorder != NULL)
	{
		StopCapture();
	}

	XnStatus nRetVal = XN_STATUS_OK;
	g_pRecorder = new xn::Recorder;

	nRetVal = g_Context.CreateAnyProductionTree(XN_NODE_TYPE_RECORDER, NULL, *g_pRecorder);
	START_CAPTURE_CHECK_RC(nRetVal, "Create recorder");

	nRetVal = g_pRecorder->SetDestination(XN_RECORD_MEDIUM_FILE, recordFile);
	START_CAPTURE_CHECK_RC(nRetVal, "set destination");
	nRetVal = g_pRecorder->AddNodeToRecording(g_DepthGenerator, XN_CODEC_16Z_EMB_TABLES);
	START_CAPTURE_CHECK_RC(nRetVal, "add node");

	g_bRecord = true;
}

void Cleanup()
{
	if (g_pRecorder)
		g_pRecorder->RemoveNodeFromRecording(g_DepthGenerator);
	StopCapture();
	g_Context.Shutdown();
}

void key(unsigned char k, int x, int y)
{
	b[k] = !b[k];

	if(k==27)
	{
		Cleanup();
		exit(0);
	}
	else if('1' == k) // Camera view
	{
		camera.translator.t = vec3f(1.518, 0.916, -0.959);
		camera.trackball.r = rotationf(vec3f(-0.392,0.883,0.191), to_radians(50));
	}
	else if('2' == k) // Front view
	{
		camera.translator.t = vec3f(0, 0, 0);
		camera.trackball.r = rotationf(vec3f(0, 0, 0), to_radians(0));
	}
	else if('3' == k) // Top view
	{
		camera.translator.t = vec3f(0, 3.5, -2);
		camera.trackball.r = rotationf(vec3f(-0.5, 0, 0), to_radians(90));
	}
	else if('4' == k) // Side view
	{
		camera.translator.t = vec3f(4, 0, -2);
		camera.trackball.r = rotationf(vec3f(0, 0.5, 0), to_radians(90));
	}

	else if('p' == k) // Laser pointer
	{
		bShowPointer = !bShowPointer;
	}
	else if('m' == k) // Mirror
	{
		XnBool bMirror = g_DepthGenerator.GetMirrorCap().IsMirrored();
		bMirror = !bMirror;
		g_DepthGenerator.GetMirrorCap().SetMirror(bMirror);
	}

	else if('+' == k) // Point
	{
		nGLPointSize++;
	}

	else if('-' == k) // Point
	{
		nGLPointSize--;
	}

	else if('z' == k) // draw "zippel" point cloud
	{
		g_bDrawPointCloud = !g_bDrawPointCloud;
	}
	else if ('o' == k)
	{
		g_bDrawOrientations = !g_bDrawOrientations;
	}
	else if ('b' == k)
	{
		g_bShowBG = !g_bShowBG;
	}
	else if ('f' == k)
	{
		g_bPrintFrameID = !g_bPrintFrameID;
	}
	else if ('s' == k)
	{
		XnUserID Users[10];
		XnUInt16 nUsers = 10;
		g_UserGenerator.GetUsers(Users, nUsers);
		g_IsSkeletonEnabled = !g_IsSkeletonEnabled;

		for (int i = 0; i < nUsers; ++i)
			g_UserGenerator.GetSkeletonCap().StopTracking(Users[i]);

	}
	else if ('k' == k)
	{
		if (g_bRecord)
		{
			// Stop capture
			StopCapture();
		}
		else
		{
			StartCapture();
			// Start capture

		}
	}
	glutPostRedisplay();
}


#define DrawSinglePointRW(point, corner)								\
{																		\
	float fX = (point.X / corner.X);								\
	float fY = -(point.Y / corner.X);								\
	float fU = fX + 0.5;												\
	float fV = fY + 0.5;												\
	glVertex3f(GLSX + (fU*GLUX), GLSY  + (fV*GLVY), point.Z/1000);	\
}

#define MAX_DEPTH 10000
float g_pDepthHist[MAX_DEPTH];

XnFloat g_r = 0, g_g = 0, g_b = 0;

void drawStickPoint(XnSkeletonJointPosition p, XnPoint3D corner)
{
	float fX = 0;
	float fY = 0;
	float fU = 0;
	float fV = 0;

	if (bRealWorld)
	{
		DrawSinglePointRW(p.position, corner);
	}
	else
	{
		XnPoint3D point = p.position;

		g_DepthGenerator.ConvertRealWorldToProjective(1, &point, &point);

		fU =  point.X / fXRes;
		fV =  point.Y / fYRes;
	}

}

void drawStickLine(XnUserID user, xn::UserGenerator userGenerator, XnSkeletonJoint joint1, XnSkeletonJoint joint2, const XnPoint3D& corner)
{
	XnSkeletonJointPosition pos1, pos2;
	userGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, joint1, pos1);
	userGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, joint2, pos2);

	if (pos1.fConfidence == 1 &&
		pos2.fConfidence == 1)
	{
		glColor3f(g_r,g_g, g_b);
	}
	else
	{
		glColor3f(0.5,0.5,0.5);

		if ((pos1.position.X == 0 && pos1.position.Y == 0 && pos1.position.Z == 0) ||
			(pos2.position.X == 0 && pos2.position.Y == 0 && pos2.position.Z == 0))
		{
			return;
		}
	}

	drawStickPoint(pos1, corner);
	drawStickPoint(pos2, corner);
}

void drawOrientation(XnUserID user, xn::UserGenerator userGenerator, XnSkeletonJoint joint, const XnPoint3D& corner)
{
	XnSkeletonJointOrientation orientation;
	XnSkeletonJointPosition position;

	userGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, joint, position);
	userGenerator.GetSkeletonCap().GetSkeletonJointOrientation(user, joint, orientation);

	if (position.fConfidence != 1 &&
		orientation.fConfidence != 1)
	{
		return;
	}

	XnSkeletonJointPosition virt1, virt2;
	virt1.fConfidence = virt2.fConfidence = 1;

	glColor3f(1,0,0);

	virt1.position = position.position;
	virt2.position = xnCreatePoint3D(100*orientation.orientation.elements[0],
		100*orientation.orientation.elements[3],
		100*orientation.orientation.elements[6]);
	drawStickPoint(virt1, corner);
	drawStickPoint(virt2, corner);

	glColor3f(0,1,0);

	virt1.position = position.position;
	virt2.position = xnCreatePoint3D(100*orientation.orientation.elements[1],
		100*orientation.orientation.elements[4],
		100*orientation.orientation.elements[7]);
	drawStickPoint(virt1, corner);
	drawStickPoint(virt2, corner);

	glColor3f(0,0,1);

	virt1.position = position.position;
	virt2.position = xnCreatePoint3D(100*orientation.orientation.elements[2],
		100*orientation.orientation.elements[5],
		100*orientation.orientation.elements[8]);
	drawStickPoint(virt1, corner);
	drawStickPoint(virt2, corner);

}

XnFloat Colors[][3] =
{
	{1,0,1},
	{1,1,0},
	{1,0.5,0},
	{0,1,1},
	{0,0.5,1},
	{1,0,0.5},
	{1,1,1}
};
XnUInt32 nColors = 6;

void DrawSingleUser(XnUserID user, xn::UserGenerator g_UserGenerator, const XnPoint3D& corner)
{
	glPushMatrix();
	object.apply_transform();

	glLineWidth(3.0);

	glBegin(GL_LINES);

	g_r = 1-Colors[user][0];
	g_g = 1-Colors[user][1];
	g_b = 1-Colors[user][2];

	drawStickLine(user, g_UserGenerator, XN_SKEL_LEFT_HAND, XN_SKEL_LEFT_ELBOW, corner);
 	drawStickLine(user, g_UserGenerator, XN_SKEL_LEFT_HAND, XN_SKEL_LEFT_ELBOW, corner);
 	drawStickLine(user, g_UserGenerator, XN_SKEL_LEFT_ELBOW, XN_SKEL_LEFT_SHOULDER, corner);
 	drawStickLine(user, g_UserGenerator, XN_SKEL_LEFT_SHOULDER, XN_SKEL_TORSO, corner);
 	drawStickLine(user, g_UserGenerator, XN_SKEL_LEFT_SHOULDER, XN_SKEL_RIGHT_SHOULDER, corner);
 	drawStickLine(user, g_UserGenerator, XN_SKEL_TORSO, XN_SKEL_RIGHT_SHOULDER, corner);
 	drawStickLine(user, g_UserGenerator, XN_SKEL_RIGHT_SHOULDER, XN_SKEL_RIGHT_ELBOW, corner);
 	drawStickLine(user, g_UserGenerator, XN_SKEL_RIGHT_ELBOW, XN_SKEL_RIGHT_HAND, corner);
 	drawStickLine(user, g_UserGenerator, XN_SKEL_NECK, XN_SKEL_HEAD, corner);
 
	drawStickLine(user, g_UserGenerator, XN_SKEL_TORSO, XN_SKEL_LEFT_HIP, corner);
	drawStickLine(user, g_UserGenerator, XN_SKEL_TORSO, XN_SKEL_RIGHT_HIP, corner);

	drawStickLine(user, g_UserGenerator, XN_SKEL_LEFT_HIP, XN_SKEL_RIGHT_HIP, corner);

	drawStickLine(user, g_UserGenerator, XN_SKEL_LEFT_HIP, XN_SKEL_LEFT_KNEE, corner);
	drawStickLine(user, g_UserGenerator, XN_SKEL_LEFT_KNEE, XN_SKEL_LEFT_FOOT, corner);
	drawStickLine(user, g_UserGenerator, XN_SKEL_RIGHT_HIP, XN_SKEL_RIGHT_KNEE, corner);
	drawStickLine(user, g_UserGenerator, XN_SKEL_RIGHT_KNEE, XN_SKEL_RIGHT_FOOT, corner);
 
 	XnSkeletonJointPosition leftShoulder;
 	XnSkeletonJointPosition rightShoulder;
 	XnSkeletonJointPosition neck;
 	XnSkeletonJointPosition midShoulder;

	g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_LEFT_SHOULDER, leftShoulder);
	g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_RIGHT_SHOULDER, rightShoulder);
	g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_NECK, neck);
 
	midShoulder.position = xnCreatePoint3D((leftShoulder.position.X + rightShoulder.position.X) / 2,
											(leftShoulder.position.Y + rightShoulder.position.Y) / 2,
											(leftShoulder.position.Z + rightShoulder.position.Z) / 2);
	midShoulder.fConfidence = (leftShoulder.fConfidence + rightShoulder.fConfidence) / 2;
 
 	drawStickPoint(neck, corner);
 	drawStickPoint(midShoulder, corner);

	if (g_bDrawOrientations)
	{
		drawOrientation(user, g_UserGenerator, XN_SKEL_TORSO, corner);
		drawOrientation(user, g_UserGenerator, XN_SKEL_HEAD, corner);
		drawOrientation(user, g_UserGenerator, XN_SKEL_LEFT_SHOULDER, corner);
		drawOrientation(user, g_UserGenerator, XN_SKEL_RIGHT_SHOULDER, corner);
		drawOrientation(user, g_UserGenerator, XN_SKEL_LEFT_ELBOW, corner);
		drawOrientation(user, g_UserGenerator, XN_SKEL_RIGHT_ELBOW, corner);
		drawOrientation(user, g_UserGenerator, XN_SKEL_NECK, corner);
		drawOrientation(user, g_UserGenerator, XN_SKEL_LEFT_HAND, corner);
		drawOrientation(user, g_UserGenerator, XN_SKEL_RIGHT_HAND, corner);
		drawOrientation(user, g_UserGenerator, XN_SKEL_LEFT_HIP, corner);
		drawOrientation(user, g_UserGenerator, XN_SKEL_RIGHT_HIP, corner);
		drawOrientation(user, g_UserGenerator, XN_SKEL_LEFT_KNEE, corner);
		drawOrientation(user, g_UserGenerator, XN_SKEL_RIGHT_KNEE, corner);
		drawOrientation(user, g_UserGenerator, XN_SKEL_LEFT_FOOT, corner);
		drawOrientation(user, g_UserGenerator, XN_SKEL_RIGHT_FOOT, corner);
	}
	
	
	if(bSendOSC)
	{
		
		XnSkeletonJointPosition leftShoulder;
 		XnSkeletonJointPosition rightShoulder;
 		XnSkeletonJointPosition neck;
 		XnSkeletonJointPosition midShoulder;
 		XnSkeletonJointPosition torso;
		XnSkeletonJointPosition head;
		XnSkeletonJointPosition leftElbow;
		XnSkeletonJointPosition rightElbow;
		XnSkeletonJointPosition leftHand;
		XnSkeletonJointPosition rightHand;
		XnSkeletonJointPosition leftHip;
		XnSkeletonJointPosition rightHip;
		XnSkeletonJointPosition leftKnee;
		XnSkeletonJointPosition rightKnee;
		XnSkeletonJointPosition leftFoot;
		XnSkeletonJointPosition rightFoot;
		
		g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_LEFT_SHOULDER, leftShoulder);
		g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_RIGHT_SHOULDER, rightShoulder);
		g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_NECK, neck);
		g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_TORSO, torso);
		g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_HEAD, head);
		g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_LEFT_ELBOW, leftElbow);
		g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_RIGHT_ELBOW, rightElbow);
		g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_LEFT_HAND, leftHand);
		g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_RIGHT_HAND, rightHand);
		g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_LEFT_HIP, leftHip);
		g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_RIGHT_HIP, rightHip);
		g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_LEFT_KNEE, leftKnee);
		g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_RIGHT_KNEE, rightKnee);
		g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_LEFT_FOOT, leftFoot);
		g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_RIGHT_FOOT, rightFoot);
		
		midShoulder.position = xnCreatePoint3D((leftShoulder.position.X + rightShoulder.position.X) / 2,
											   (leftShoulder.position.Y + rightShoulder.position.Y) / 2,
											   (leftShoulder.position.Z + rightShoulder.position.Z) / 2);
		
		XnSkeletonJointOrientation leftShoulderO;
 		XnSkeletonJointOrientation rightShoulderO;
 		XnSkeletonJointOrientation neckO;
 		XnSkeletonJointOrientation midShoulderO;
 		XnSkeletonJointOrientation torsoO;
		XnSkeletonJointOrientation headO;
		XnSkeletonJointOrientation leftElbowO;
		XnSkeletonJointOrientation rightElbowO;
		XnSkeletonJointOrientation leftHandO;
		XnSkeletonJointOrientation rightHandO;
		XnSkeletonJointOrientation leftHipO;
		XnSkeletonJointOrientation rightHipO;
		XnSkeletonJointOrientation leftKneeO;
		XnSkeletonJointOrientation rightKneeO;
		XnSkeletonJointOrientation leftFootO;
		XnSkeletonJointOrientation rightFootO;
		
		g_UserGenerator.GetSkeletonCap().GetSkeletonJointOrientation(user, XN_SKEL_LEFT_SHOULDER, leftShoulderO);
		g_UserGenerator.GetSkeletonCap().GetSkeletonJointOrientation(user, XN_SKEL_RIGHT_SHOULDER, rightShoulderO);
		g_UserGenerator.GetSkeletonCap().GetSkeletonJointOrientation(user, XN_SKEL_NECK, neckO);
		g_UserGenerator.GetSkeletonCap().GetSkeletonJointOrientation(user, XN_SKEL_TORSO, torsoO);
		g_UserGenerator.GetSkeletonCap().GetSkeletonJointOrientation(user, XN_SKEL_HEAD, headO);
		g_UserGenerator.GetSkeletonCap().GetSkeletonJointOrientation(user, XN_SKEL_LEFT_ELBOW, leftElbowO);
		g_UserGenerator.GetSkeletonCap().GetSkeletonJointOrientation(user, XN_SKEL_RIGHT_ELBOW, rightElbowO);
		g_UserGenerator.GetSkeletonCap().GetSkeletonJointOrientation(user, XN_SKEL_LEFT_HAND, leftHandO);
		g_UserGenerator.GetSkeletonCap().GetSkeletonJointOrientation(user, XN_SKEL_RIGHT_HAND, rightHandO);
		g_UserGenerator.GetSkeletonCap().GetSkeletonJointOrientation(user, XN_SKEL_LEFT_HIP, leftHipO);
		g_UserGenerator.GetSkeletonCap().GetSkeletonJointOrientation(user, XN_SKEL_RIGHT_HIP, rightHipO);
		g_UserGenerator.GetSkeletonCap().GetSkeletonJointOrientation(user, XN_SKEL_LEFT_KNEE, leftKneeO);
		g_UserGenerator.GetSkeletonCap().GetSkeletonJointOrientation(user, XN_SKEL_RIGHT_KNEE, rightKneeO);
		g_UserGenerator.GetSkeletonCap().GetSkeletonJointOrientation(user, XN_SKEL_LEFT_FOOT, leftFootO);
		g_UserGenerator.GetSkeletonCap().GetSkeletonJointOrientation(user, XN_SKEL_RIGHT_FOOT, rightFootO);
		
		
		sendOSCMarker("/body/head", head, headO);
		sendOSCMarker("/body/torso", torso, torsoO);
		sendOSCMarker("/body/neck", neck, neckO);
		sendOSCMarker("/body/leftShoulder", leftShoulder, leftShoulderO);
		sendOSCMarker("/body/rightShoulder", rightShoulder, rightShoulderO);
		sendOSCMarker("/body/midShoulder", midShoulder, midShoulderO);
		sendOSCMarker("/body/leftElbow", leftElbow, leftElbowO);
		sendOSCMarker("/body/rightElbow", rightElbow, rightElbowO);
		sendOSCMarker("/body/leftHand", leftHand, leftHandO);
		sendOSCMarker("/body/rightHand", rightHand, rightHandO);
		sendOSCMarker("/body/leftHip", leftHip, leftHipO);
		sendOSCMarker("/body/rightHip", rightHip, rightHipO);
		sendOSCMarker("/body/leftKnee", leftKnee, leftKneeO);
		sendOSCMarker("/body/rightKnee", rightKnee, rightKneeO);
		sendOSCMarker("/body/leftFoot", leftFoot, leftFootO);
		sendOSCMarker("/body/rightFoot", rightFoot, rightFootO);
		
	}

	glEnd();
	glPopMatrix();
}



void sendOSCMarker(string prefix, XnSkeletonJointPosition &pos, XnSkeletonJointOrientation &ori)
{
	ofxOscMessage m;
	m.setAddress(prefix);
	m.addFloatArg(pos.position.X);		
	m.addFloatArg(pos.position.Y);		
	m.addFloatArg(pos.position.Z);	
	
	float m00 = ori.orientation.elements[0],
	m01 = ori.orientation.elements[1],
	m02 = ori.orientation.elements[2],
	m10 = ori.orientation.elements[3],
	m11 = ori.orientation.elements[4],
	m12 = ori.orientation.elements[5],
	m20 = ori.orientation.elements[6],
	m21 = ori.orientation.elements[7],
	m22 = ori.orientation.elements[8];
	
	float x, y, z, w;
	w = sqrt( max( 0, 1 + m00 + m11 + m22 ) ) / 2.0f;
	x = sqrt( max( 0, 1 + m00 - m11 - m22 ) ) / 2.0f;
	y = sqrt( max( 0, 1 - m00 + m11 - m22 ) ) / 2.0f;
	z = sqrt( max( 0, 1 - m00 - m11 + m22 ) ) / 2.0f;
	x = copysign( x, m21 - m12 );
	y = copysign( y, m02 - m20 );
	z = copysign( z, m10 - m01 );
	
	//m.addFloatArg(x);
	//m.addFloatArg(y);
	//m.addFloatArg(z);
	//m.addFloatArg(w);
	sender.sendMessage(m);
}


void draw_stickfigure()
{
	XnUInt16 nXRes;
	XnUInt16 nYRes;

	xn::DepthMetaData dm;
	g_DepthGenerator.GetMetaData(dm);
	XnPoint3D corner = xnCreatePoint3D(dm.XRes(), dm.YRes(), dm.ZRes());
	g_DepthGenerator.ConvertProjectiveToRealWorld(1, &corner, &corner);

	nXRes = dm.XRes();
	nYRes = dm.YRes();
	glDisable(GL_LIGHTING);

	XnUserID users[10];
	XnUInt16 nUsers = 10;

	g_UserGenerator.GetUsers(users, nUsers);
	for (int i = 0; i < nUsers; ++i)
	{
		if (g_UserGenerator.GetSkeletonCap().IsTracking(users[i]))
		{
			DrawSingleUser(users[i], g_UserGenerator, corner);
		}
	}

	glEnable(GL_LIGHTING);
}


void draw_pointcloud()
{
	float fValueH = 0;
	float fX = 0;
	float fY = 0;
	float fU = 0;
	float fV = 0;
	float fNewColor = 0;
	XnUInt16 nXRes;
	XnUInt16 nYRes;
	XnDepthPixel nMaxDepth;

	xn::DepthMetaData dm;
	g_DepthGenerator.GetMetaData(dm);

	nXRes = dm.XRes();
	nYRes = dm.YRes();
	nMaxDepth = dm.ZRes();

	const XnDepthPixel* pDepth = dm.Data();

	memset(g_pDepthHist, 0, MAX_DEPTH*sizeof(float));
	int nNumberOfPoints = 0;

	XnDepthPixel nValue;
	for (XnUInt16 nY=0; nY<nYRes; nY++)
	{
		for (XnUInt16 nX=0; nX<nXRes; nX++)
		{
			nValue = *pDepth;

			if (nValue != 0)
			{
				g_pDepthHist[nValue]++;
				nNumberOfPoints++;
			}

			pDepth++;
		}
	}
	XnUInt32 nIndex;
	for (nIndex=1; nIndex<MAX_DEPTH; nIndex++)
	{
		g_pDepthHist[nIndex] += g_pDepthHist[nIndex-1];
	}
	for (nIndex=1; nIndex<MAX_DEPTH; nIndex++)
	{
		if (g_pDepthHist[nIndex] != 0)
		{
			g_pDepthHist[nIndex] = (nNumberOfPoints-g_pDepthHist[nIndex]) / nNumberOfPoints;
		}
	}

	pDepth = dm.Data();


	// GL Transform
	glPushMatrix();
	object.apply_transform();

	// Point cloud
	float zero[] = {0,0,0,0};
	float dim[] = {.2,.2,.2,.2};
	glLightfv(GL_LIGHT0, GL_DIFFUSE, dim);
	glLightfv(GL_LIGHT0, GL_SPECULAR, zero);
	glPolygonOffset(0,-2);
	glPushAttrib(GL_ENABLE_BIT);
	glEnable(GL_DEPTH_TEST);
	glDisable(GL_LIGHTING);
	glDisable(GL_COLOR_MATERIAL);

	glPointSize (nGLPointSize);
	glBegin(GL_POINTS);

	XnPoint3D corner = xnCreatePoint3D(dm.XRes(), dm.YRes(), dm.ZRes());
	g_DepthGenerator.ConvertProjectiveToRealWorld(1, &corner, &corner);

	XnBool bLabelsExists = false;
	const XnLabel* pLabels = NULL;
	xn::SceneMetaData smd;
	if (g_UserGenerator.GetUserPixels(0, smd) == XN_STATUS_OK)
	{
		bLabelsExists = TRUE;
		pLabels = smd.Data();
	}

	nIndex = 0;
	for (XnUInt16 nY = 0; nY < nYRes; nY++)
	{
		for (XnUInt16 nX = 0; nX < nXRes; nX++, nIndex++, pLabels++)
		{
			fValueH = pDepth[nIndex];
			if (pDepth[nIndex] == 0)
				continue;

			XnUInt32 nColorID = nColors;
			if (bLabelsExists)
			{
				nColorID = *pLabels % nColors;
				if (*pLabels == 0)
				{
					if (!g_bShowBG)
					{
						continue;
					}
					nColorID = nColors;
				}
			}
			else if (!g_bShowBG)
			{
				continue;
			}

			glColor3f(
				g_pDepthHist[pDepth[nIndex]] * Colors[nColorID][0],
				g_pDepthHist[pDepth[nIndex]] * Colors[nColorID][1],
				g_pDepthHist[pDepth[nIndex]] * Colors[nColorID][2]);

			if (bRealWorld != TRUE)
			{
				fX = nX;
				fY = nY;

				fU =  fX / fXRes;
				fV =  fY / fXRes;
			}
			else
			{
				XnPoint3D point = xnCreatePoint3D(nX, nY, fValueH);
				g_DepthGenerator.ConvertProjectiveToRealWorld(1, &point, &point);

				DrawSinglePointRW(point, corner);
			}

		}
	}
	glEnd();

	// Pointer
	if (bShowPointer)
	{
		glBegin(GL_POINTS);

		if (nPointerY < 0)
			nPointerY = 0;
		if (nPointerY >= nYRes)
			nPointerY = nYRes - 1;
		if (nPointerX < 0)
			nPointerX = 0;
		if (nPointerX >= nXRes)
			nPointerX = nXRes - 1;

		fValueH = pDepth[(nPointerY * nXRes) + nPointerX] - 30;
		fU =  nPointerX / fXRes;
		fV =  nPointerY / fXRes;

		glColor3f(1,0,0);
		glVertex3f(GLSX + (fU*GLUX), GLSY  + (fV*GLVY), fValueH/1000);	

		glEnd();
	}

	glPopAttrib();

	// Undo GL Transform
	glPopMatrix();
}

void draw_mesh(float size, int tess)
{
	float hsize = size/2;
	float delta = size/(tess-1);

	glPushMatrix();
	glTranslatef(-hsize, -hsize, hsize);
	
	glNormal3f(0,0,-1);

	float x = 0;
	for(int i=0; i < tess-1; i++)
	{
		float y = 0;
		glBegin(GL_QUAD_STRIP);
		for(int j=0; j < tess; j++)
		{
			glTexCoord2f(    x, y);
			glVertex2f(      x, y);
			glTexCoord2f(x+delta, y);
			glVertex2f(x+delta, y);
			y += delta;
		}
		glEnd();
		x += delta;
	}
	glPopMatrix();
}


void draw_cube()
{
	wall.bind();
	wall.enable();
	glPushMatrix();
	room.apply_transform();
	face.call_list();
	glRotatef(90, 1, 0, 0);
	face.call_list();
	glRotatef(90, 1, 0, 0);
	face.call_list();
	glRotatef(90, 1, 0, 0);
	face.call_list();
	glRotatef(90, 1, 0, 0);
	glRotatef(90, 0, 1, 0);
	face.call_list();
	glRotatef(180, 0, 1, 0);
	face.call_list();
	glPopMatrix();
	wall.disable();
}

void draw_room(bool do_diffuse)
{
	float zero[] = {0,0,0,0};
	float a[4];
	a[0] = room_ambient;
	a[1] = room_ambient;
	a[2] = room_ambient;
	a[3] = 1;

	float d1[] = {.1,.1,.1,.1};
	float d2[] = {.7,.7,.7,.7};
	float s[] = {.7,.7,.7,.7};
	float emission[4];
	float ambient[4];
	float diffuse[4];
	float specular[4];

	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, a);
	if (g_bRecord)
	{
		glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, vec4f(0.8, 0.0, 0.0, 1).v);
	}
	else
	{
		glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, vec4f(0.8, 0.8, 0.8, 1).v);
	}
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, vec4f(0.4, 0.4, 0.4, 1).v);
	glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 64.f);

	if(! do_diffuse)
	{
		glGetLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse);
		glLightfv(GL_LIGHT0, GL_DIFFUSE, d1);
		glGetLightfv(GL_LIGHT0, GL_SPECULAR, specular);
		glLightfv(GL_LIGHT0, GL_SPECULAR, zero);

		glStencilFunc(GL_ALWAYS, 128, ~0);
	}
	else
	{
        glGetMaterialfv(GL_FRONT, GL_EMISSION, emission);
        glMaterialfv(GL_FRONT, GL_EMISSION, zero);
		glGetLightfv(GL_LIGHT0, GL_AMBIENT, ambient);
		glLightfv(GL_LIGHT0, GL_AMBIENT, zero);
		glLightfv(GL_LIGHT0, GL_DIFFUSE, d2);
		glLightfv(GL_LIGHT0, GL_SPECULAR, s);

		glBlendFunc(GL_ONE, GL_ONE);
		glEnable(GL_BLEND);
		glStencilFunc(GL_EQUAL, 128, ~0);
		glDepthFunc(GL_EQUAL);
	}
	glPushMatrix();
	glTranslatef(0,9,0);
	glEnable(GL_LIGHTING);
	glStencilOp(GL_KEEP, GL_KEEP, GL_KEEP);
	glEnable(GL_STENCIL_TEST);

	draw_cube();

	glStencilFunc(GL_ALWAYS, 128, ~0);
	glStencilOp(GL_KEEP, GL_KEEP, GL_KEEP);

	glDisable(GL_LIGHTING);
	glPopMatrix();
	
	if(! do_diffuse)
	{
		glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse);
		glLightfv(GL_LIGHT0, GL_SPECULAR, specular);
	}
	else
	{
        glMaterialfv(GL_FRONT, GL_EMISSION, emission);
		glLightfv(GL_LIGHT0, GL_AMBIENT, ambient);

		glDisable(GL_BLEND);
		glDepthFunc(GL_LESS);
	}
}


// The infinite frustum set-up code.
matrix4f infinite_frustum(float left, float right,
						  float bottom, float top,
						  float zNear)
{
	matrix4f m;
	m.make_identity();
	
	m(0,0) = (2*zNear) / (right - left);
	m(0,2) = (right + left) / (right - left);
	
	m(1,1) = (2*zNear) / (top - bottom);
	m(1,2) = (top + bottom) / (top - bottom);
	
    float nudge = 1.0;
    
    if(b['j'])    // nudge infinity in some in case of lsb slop
        nudge = 0.999;


	m(2,2) = -1  * nudge;
	m(2,3) = -2*zNear * nudge;
	
	m(3,2) = -1;
	m(3,3) = 0;
	
	return m;
}

inline matrix4f infinite_frustum_inverse(float left, float right,
								float bottom, float top,
								float zNear)
{
	matrix4f m;
	m.make_identity();
	
	m(0,0) = (right - left) / (2 * zNear);
	m(0,3) = (right + left) / (2 * zNear);
	
	m(1,1) = (top - bottom) / (2 * zNear);
	m(1,3) = (top + bottom) / (2 * zNear);
	
	m(2,2) = 0;
	m(2,3) = -1;
	
	m(3,2) = -1 / (2 * zNear);
	m(3,3) = 1 / (2 * zNear);
	
	return m;
}

matrix4f infinite_perspective(float fovy, float aspect, float zNear)
{
	double tangent = tan(to_radians(fovy/2.0));
	float y = tangent * zNear;
	float x = aspect * y;
	return infinite_frustum(-x, x, -y, y, zNear);
}

inline matrix4f infinite_perspective_inverse(float fovy, float aspect, float zNear)
{
	double tangent = tan(to_radians(fovy/2.0));
	float y = tangent * zNear;
	float x = aspect * y;
	return infinite_frustum_inverse(-x, x, -y, y, zNear);
}

void apply_infinite_perspective(glut_perspective_reshaper & r)
{
	r.aspect = r.aspect_factor * float(r.width)/float(r.height);
	if ( r.aspect < 1 )
	{
		float fovx = r.fovy; 
		float real_fov = to_degrees(2 * atan(tan(to_radians(fovx/2))/r.aspect));
		glMultMatrixf(infinite_perspective(real_fov, r.aspect, r.zNear).m);
	}
	else
		glMultMatrixf(infinite_perspective(r.fovy, r.aspect, r.zNear).m);
}

void display()
{
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	apply_infinite_perspective(reshaper);
	glMatrixMode(GL_MODELVIEW);

	glPushMatrix();
	light.apply_transform();
	glLightfv(GL_LIGHT0, GL_POSITION, light_position.v);
	glPopMatrix();

	glEnable(GL_LIGHT0);
	glPushMatrix();
	camera.apply_inverse_transform();
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
	glDisable(GL_DEPTH_TEST);

	draw_room(false);

	if (g_bDrawPointCloud)
	{
		draw_pointcloud();
	}


	draw_room(true);
	draw_stickfigure();
	glPopMatrix();

	// Print the Scale + "Laser Pointer"
	if (bShowPointer)
	{
		// Setup the opengl env for fixed location view
		glMatrixMode(GL_PROJECTION);
		glPushMatrix();
		glLoadIdentity();
		glOrtho(0,WIN_SIZE_X,0,WIN_SIZE_Y,-1.0,1.0);
		glDisable(GL_DEPTH_TEST); 

		// Dark back
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);		
		glBegin(GL_QUAD_STRIP);
		glColor4f(0,0,0,0.85);
		glVertex3f(0, 0, 0);
		glVertex3f(1050, 0, 0);
		glVertex3f(0, 115, 0);
		glVertex3f(1050, 115, 0);
		glEnd();
		glDisable(GL_BLEND);

		// Print the scale black background
		glPointSize(15);
		glColor3f(0, 0, 0);
		glBegin(GL_POINTS);
		for (int i=0; i<fMaxDepth; i+=1)
		{			
			glVertex3f(1024-(i*2), 23, 1);
			glVertex3f(1024-(i*2), 70, 1);
		}
		glEnd();

		const XnDepthPixel* pDepth = g_DepthGenerator.GetDepthMap();

		// Print the scale data
		glBegin(GL_POINTS);
		for (int i=0; i<fMaxDepth; i+=1)
		{
			float fNewColor = g_pDepthHist[pDepth[i]];
			if ((fNewColor > 0.004) && (fNewColor < 0.996))
			{
				glColor3f(fNewColor, fNewColor, 0);
				glVertex3f((i*2), 23, 1);
			}
		}
		glEnd();

		// Print the pointer scale data
		glBegin(GL_POINTS);
		int nPointerValue = pDepth[(nPointerY * (int)fXRes) + nPointerX];
		glColor3f(1,0,0);
		glVertex3f((nPointerValue*2), 70, 1);
		glEnd();

		// Print the scale texts
		char buf[80];
		for (int i=0; i<fMaxDepth; i+=1)
		{
			if (i % 25 == 0)
			{
				sprintf(buf, "%d", i);
				glRasterPos2i(10+(i*2),40);
				glPrintString(GLUT_BITMAP_HELVETICA_18,buf);
			}
		}

		// Print the pointer text
		if (nPointerValue != fMaxDepth)
		{
			sprintf(buf, "Pointer Value: %.1f (X:%d Y:%d) Cutoff: %d-%d", (float)nPointerValue/10, nPointerX, nPointerY, nCutOffMin, nCutOffMax);
		}
		else
		{
			sprintf(buf, "Pointer Value: - (X:%d Y:%d)) Cutoff: %d-%d", nPointerX, nPointerY, nCutOffMin, nCutOffMax);
		}
		glRasterPos2i(10,88);
		glPrintString(GLUT_BITMAP_TIMES_ROMAN_24,buf);

		// Undo the opengl env settings
		glEnable(GL_DEPTH_TEST); 
		glPopMatrix();
	}
	if (g_bPrintFrameID)
	{
		glMatrixMode(GL_PROJECTION);
		glPushMatrix();
		glLoadIdentity();
		glOrtho(0,WIN_SIZE_X,0,WIN_SIZE_Y,-1.0,1.0);
		glDisable(GL_DEPTH_TEST); 

		XnChar strLabel[20];
		XnUInt32 nFrameID = g_DepthGenerator.GetFrameID();
		sprintf(strLabel, "%d", nFrameID);
		
		glColor4f(1,0,0,1);
		glRasterPos2i(20, WIN_SIZE_Y-20);
		glPrintString(GLUT_BITMAP_HELVETICA_18, strLabel);

		// Undo the opengl env settings
		glEnable(GL_DEPTH_TEST); 
		glPopMatrix();
	}

	glutSwapBuffers();
}
