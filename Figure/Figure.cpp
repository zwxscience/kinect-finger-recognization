// Figure.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "MemTracker.h"
#include <vld.h>
/****************************************************************************
*                                                                           *
*   Nite 1.3 - Point Viewer Sample                                          *
*                                                                           *
*   Author:     Oz Magal                                                    *
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

// Headers for OpenNI
#include <XnOpenNI.h>
#include <XnCppWrapper.h>
#include <XnHash.h>
#include <XnLog.h>

// Header for NITE
#include "XnVNite.h"
// local header
#include "PointDrawer.h"



// Headers for OpenCV

#include <opencv2/legacy/blobtrack.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
//system
#include <iostream>
#include <math.h>
using namespace  std;
using namespace  cv;
#define CHECK_RC(rc, what)											\
	if (rc != XN_STATUS_OK)											\
{																\
	printf("%s failed: %s\n", what, xnGetStatusString(rc));		\
	return rc;													\
}


#include "gl/glut.h"


// OpenNI objects
xn::Context g_Context;
xn::DepthGenerator g_DepthGenerator;
xn::HandsGenerator g_HandsGenerator;
xn::ImageGenerator g_ImageGenerator;

// NITE objects
XnVSessionManager* g_pSessionManager;
XnVFlowRouter* g_pFlowRouter;

// the drawer
XnVPointDrawer* g_pDrawer;

#define GL_WIN_SIZE_X 720
#define GL_WIN_SIZE_Y 480

// Draw the depth map?
XnBool g_bDrawDepthMap = true;
XnBool g_bPrintFrameID = false;
// Use smoothing?
XnFloat g_fSmoothing = 0.0f;
XnBool g_bPause = false;
XnBool g_bQuit = false;

SessionState g_SessionState = NOT_IN_SESSION;

#define  Max(a,b) (a>b?a:b)
vector<CvPoint>figures;//���ָ������
vector<CvPoint>figures2;//���ָ������(���������е�����²���)
CvPoint3D32f pad;
CvMemStorage *storage=0;//��������ʱ��Ҫ�Ĵ洢����
CvSeq *contour =0;//���ô洢��ȡ������ָ��
void CleanupExit()
{
	g_Context.Shutdown();

	exit (1);
}

// Callback for when the focus is in progress
void XN_CALLBACK_TYPE FocusProgress(const XnChar* strFocus, const XnPoint3D& ptPosition, XnFloat fProgress, void* UserCxt)
{
	//	printf("Focus progress: %s @(%f,%f,%f): %f\n", strFocus, ptPosition.X, ptPosition.Y, ptPosition.Z, fProgress);
}
// callback for session start
void XN_CALLBACK_TYPE SessionStarting(const XnPoint3D& ptPosition, void* UserCxt)
{
	printf("Session start: (%f,%f,%f)\n", ptPosition.X, ptPosition.Y, ptPosition.Z);
	g_SessionState = IN_SESSION;
}
// Callback for session end
void XN_CALLBACK_TYPE SessionEnding(void* UserCxt)
{	

	printf("Session end\n");
	g_SessionState = NOT_IN_SESSION;
}
void XN_CALLBACK_TYPE NoHands(void* UserCxt)
{
	printf("Quick refocus\n");

	g_SessionState = QUICK_REFOCUS;
}

//ͼ��ϸ���㷨
typedef unsigned char byte;
void cvThin( IplImage* src, IplImage* dst, int iterations=1)
{
	CvSize size = cvGetSize(src);
	cvCopy(src, dst);
	int n = 0,i = 0,j = 0;
	IplImage* t_image=NULL;
	for(n=0; n<iterations; n++)
	{
		if (t_image==NULL)
		{	
			t_image = cvCreateImage(cvGetSize(dst),dst->depth,dst->nChannels);
		}
		cvCopy(dst,t_image);
		for(i=0; i<size.height;  i++)
		{
			for(j=0; j<size.width; j++)
			{
				if(CV_IMAGE_ELEM(t_image,byte,i,j)==1)
				{
					int ap=0;
					int p2 = (i==0)?0:CV_IMAGE_ELEM(t_image,byte, i-1, j);
					int p3 = (i==0 || j==size.width-1)?0:CV_IMAGE_ELEM(t_image,byte, i-1, j+1);
					if (p2==0 && p3==1)
					{
						ap++;
					}
					int p4 = (j==size.width-1)?0:CV_IMAGE_ELEM(t_image,byte,i,j+1);
					if(p3==0 && p4==1)
					{
						ap++;
					}
					int p5 = (i==size.height-1 || j==size.width-1)?0:CV_IMAGE_ELEM(t_image,byte,i+1,j+1);
					if(p4==0 && p5==1)
					{
						ap++;
					}
					int p6 = (i==size.height-1)?0:CV_IMAGE_ELEM(t_image,byte,i+1,j);
					if(p5==0 && p6==1)
					{
						ap++;
					}
					int p7 = (i==size.height-1 || j==0)?0:CV_IMAGE_ELEM(t_image,byte,i+1,j-1);
					if(p6==0 && p7==1)
					{
						ap++;
					}
					int p8 = (j==0)?0:CV_IMAGE_ELEM(t_image,byte,i,j-1);
					if(p7==0 && p8==1)
					{
						ap++;
					}
					int p9 = (i==0 || j==0)?0:CV_IMAGE_ELEM(t_image,byte,i-1,j-1);
					if(p8==0 && p9==1)
					{
						ap++;
					}
					if(p9==0 && p2==1)
					{
						ap++;
					}
					if((p2+p3+p4+p5+p6+p7+p8+p9)>1 && (p2+p3+p4+p5+p6+p7+p8+p9)<7)
					{
						if(ap==1)
						{
							if(!(p2 && p4 && p6))
							{
								if(!(p4 && p6 && p8)) 
								{
									CV_IMAGE_ELEM(dst,byte,i,j)=0;
								}
							}
						}
					}

				}
			}
		}
		//	cvReleaseImage(&t_image);
		cvCopy(dst,t_image);
		//	t_image = cvCloneImage(dst);
		for(i=0; i<size.height;  i++)
		{
			for(int j=0; j<size.width; j++)
			{
				if(CV_IMAGE_ELEM(t_image,byte,i,j)==1)
				{
					int ap=0;
					int p2 = (i==0)?0:CV_IMAGE_ELEM(t_image,byte, i-1, j);
					int p3 = (i==0 || j==size.width-1)?0:CV_IMAGE_ELEM(t_image,byte, i-1, j+1);
					if (p2==0 && p3==1)
					{
						ap++;
					}
					int p4 = (j==size.width-1)?0:CV_IMAGE_ELEM(t_image,byte,i,j+1);
					if(p3==0 && p4==1)
					{
						ap++;
					}
					int p5 = (i==size.height-1 || j==size.width-1)?0:CV_IMAGE_ELEM(t_image,byte,i+1,j+1);
					if(p4==0 && p5==1)
					{
						ap++;
					}
					int p6 = (i==size.height-1)?0:CV_IMAGE_ELEM(t_image,byte,i+1,j);
					if(p5==0 && p6==1)
					{
						ap++;
					}
					int p7 = (i==size.height-1 || j==0)?0:CV_IMAGE_ELEM(t_image,byte,i+1,j-1);
					if(p6==0 && p7==1)
					{
						ap++;
					}
					int p8 = (j==0)?0:CV_IMAGE_ELEM(t_image,byte,i,j-1);
					if(p7==0 && p8==1)
					{
						ap++;
					}
					int p9 = (i==0 || j==0)?0:CV_IMAGE_ELEM(t_image,byte,i-1,j-1);
					if(p8==0 && p9==1)
					{
						ap++;
					}
					if(p9==0 && p2==1)
					{
						ap++;
					}
					if((p2+p3+p4+p5+p6+p7+p8+p9)>1 && (p2+p3+p4+p5+p6+p7+p8+p9)<7)
					{
						if(ap==1)
						{
							if(p2*p4*p8==0)
							{
								if(p2*p6*p8==0)
								{
									CV_IMAGE_ELEM(dst, byte,i,j)=0;
								}
							}
						}
					}                    
				}

			}

		}            
		cvReleaseImage(&t_image);
	}

}
//Ѱ����������λ��
void find_center(const IplImage *img, CvPoint &center_point)
{
	double M00(0.0),M10(0.0),M11(0.0);
	for(int i=0;i<img->height;i++)
		for(int j=0;j<img->width;j++)
		{
			M00+=cvGetReal2D(img,i,j);
		}
		for(int i=0;i<img->height;i++)
			for (int j=0;j<img->width;j++)
			{
				M10+=(double)i*cvGetReal2D(img,i,j);
				M11+=(double)j*cvGetReal2D(img,i,j);
			}
			center_point=cvPoint(M11/M00,M10/M00);
}
//Ѱ����ָ
void AngleDect(const IplImage* img,const 	CvPoint center_point){
	//IplImage *src =cvCloneImage(img); 
	IplImage *src =cvCreateImage(cvGetSize(img),img->depth,img->nChannels);
	cvCopy(img,src);
	IplImage* color_dst;
	CvMemStorage* Houghstorage = cvCreateMemStorage(0);
	CvSeq* lines = 0;
	int i;
	color_dst = cvCreateImage( cvGetSize(src), 8, 3 );
	cvCanny( src, src, 50, 200, 3 );//����50��200�ĻҶȱ任
	cvSmooth(src,src);
	cvCvtColor( src, color_dst, CV_GRAY2BGR );
	lines = cvHoughLines2( src, Houghstorage, CV_HOUGH_PROBABILISTIC, 1, CV_PI/180, 25, 25, 10);
	int total= lines->total;
	if (total>10)
	{
		total=10;
	}
	for( i = 0; i < total; i++ )
	{
		CvPoint* line = (CvPoint*)cvGetSeqElem(lines,i);
		//�߶γ���
		float length=sqrt((float)(line[0].y-line[1].y)*(line[0].y-line[1].y)+(line[0].x-line[1].x)*(line[0].x-line[1].x));
		bool addtag=false;
		//�����ĵľ���
		float 	distance1=sqrt((float)(line[0].x-center_point.x)*(line[0].x-center_point.x)+(line[0].y-center_point.y)*(line[0].y-center_point.y));
		float 	distance2=sqrt((float)(line[1].x-center_point.x)*(line[1].x-center_point.x)+(line[1].y-center_point.y)*(line[1].y-center_point.y));
		CvPoint start1;//start1Ϊhough����ָ��
		float length1;//���ĵ�ָ��ľ���
		float length3;//���ĵ�ָ���ľ���
		float angle;
		if (distance2<distance1)//˵��line��0������ָ��
		{
			length1=distance1;
			length3=distance2;
			angle=atan((float)(line[0].y-line[1].y)/(line[0].x-line[1].x));
			start1.x=line[1].x;
			start1.y=line[1].y;
		}else{
			angle=atan((float)(line[1].y-line[0].y)/(line[1].x-line[0].x));
			length1=distance2;
			length3=distance1;
			start1.x=line[0].x;
			start1.y=line[0].y;
		}
		//�������������򣬱�֤0Ϊָͷ����start1Ϊָ��

		for (int j=0;j<figures2.size();j+=2)
		{
			float angle2=atan((float)(figures2[j+1].y-figures2[j].y)/(figures2[j+1].x-figures2[j].x));//�Ѵ�ָ��
			float  figuredist=sqrt((float)(figures2[j].x-start1.x)*(figures2[j].x-start1.x)+(figures2[j].y-start1.y)*(figures2[j].y-start1.y));//�µ����Ѵ�֮��ľ���
			float  figurespace=fabs((float)figures2[j].x-start1.x);
			if (figuredist<=5||fabs((float)angle-angle2)<0.05||figurespace<=3)//�����������ָ���ľ�����5�����ڣ���ָ�Ƕȷ�Χ���.05�ڣ�����ָ��x�����3�������ڣ�����Ϊ��ͬһ����ָ
			{
				float length2=sqrt((float)(figures2[j+1].x-center_point.x)*(figures2[j+1].x-center_point.x)+(figures2[j+1].y-center_point.y)*(figures2[j+1].y-center_point.y));//�Ѵ�ָ�������ĵľ���
				if (length2<length1)//����µ����ָ���������滻
				{
					if (distance2>distance1)
					{//˵��line��1������ָ��
						figures2[j].x=line[0].x;
						figures2[j].y=line[0].y;
						figures2[j+1].x=line[1].x;
						figures2[j+1].y=line[1].y;
					}else{
						figures2[j].x=line[1].x;
						figures2[j].y=line[1].y;
						figures2[j+1].x=line[0].x;
						figures2[j+1].y=line[0].y;
					}
				}
				addtag=true;
				break;
			}
		}
		if (!addtag&&length>20&&length3<100&&length1>60)//��Ҫ����һ���ĳ��Ⱥ��߼��ԣ���ӱ��δ�䣬ָ������20�����ĵ�ָ��������60��������ָ��������100������
		{
			if (distance2>distance1)//˵��line��1������ָ�⣬����ƥ��
			{
				figures2.push_back(line[0]);
				figures2.push_back(line[1]);
			}else{
				figures2.push_back(line[1]);
				figures2.push_back(line[0]);
			}
		}
		cvLine( color_dst, line[0], line[1], CV_RGB(255,0,0), 1,8 );

	}
	cout <<"Houghֱ����ӣ�"<<figures2.size()/2 << endl;

	//	cvShowImage( "Hough", color_dst );
	cvReleaseMemStorage(&Houghstorage);
	cvReleaseImage(&color_dst);
	cvReleaseImage(&src);
}
//�������ͼ��ϸ�����Եó��Ľ��ͼ
void DrawThinalgorithm( IplImage* GrayHand){
	IplImage *pDst = NULL,*pTmp = NULL;
	//method :ͼ��ϸ��+hough transform
	//pTmp = cvCloneImage(GrayHand);
	pTmp=cvCreateImage(cvGetSize(GrayHand),GrayHand->depth,GrayHand->nChannels);
	cvCopy(GrayHand,pTmp);
	pDst = cvCreateImage(cvGetSize(GrayHand),GrayHand->depth,GrayHand->nChannels);
	cvZero(pDst);
	cvThreshold(GrayHand,pTmp,128,1,CV_THRESH_BINARY_INV);//����ֵ������ͼ��ת����0��1��ʽ
	cvErode(pDst,pDst,NULL,3);
	cvDilate(pDst,pDst,NULL,3);
	cvThin(pTmp,pDst,8);//ϸ����ͨ���޸�iterations������һ��ϸ��
	cvReleaseImage(&pTmp);
	CvPoint center_point;
	//����ֵͼ��ת���ɻҶȣ��Ա���ʾ
	int i = 0,j = 0;
	CvSize size = cvGetSize(pDst);
	for(i=0; i<size.height;  i++)
	{
		for(j=0; j<size.width; j++)
		{
			if(CV_IMAGE_ELEM(pDst,uchar,i,j)==1)
			{
				CV_IMAGE_ELEM(pDst,uchar,i,j) = 0;
			}
			else
			{
				CV_IMAGE_ELEM(pDst,uchar,i,j) = 255;
			}
		}
	}

	//GrayHand=cvCloneImage(pDst);���Ƽ�
	cvCopy(pDst,GrayHand);
	//������������ϸ�������ָ���ָ����ƣ�Ȼ���ԭ���ģ�ϸ�����������򣬵õ�ϸ����ָ
	cvDilate(pDst,pDst);
	cvErode(pDst,pDst);
	//CvPoint center_point;
	find_center(pDst,center_point);//Ѱ����������
	for(i=0; i<size.height;  i++)
	{
		for(j=0; j<size.width; j++)
		{
			if(CV_IMAGE_ELEM(pDst,uchar,i,j)==CV_IMAGE_ELEM(GrayHand,uchar,i,j) )
			{
				CV_IMAGE_ELEM(pDst,uchar,i,j) =255;
			}
			else
			{
				CV_IMAGE_ELEM(pDst,uchar,i,j) = 0;
			}
		}
	}
	//
	AngleDect(pDst,center_point);//Ѱ����ָ��
	cvReleaseImage(&pDst);
	cvReleaseImage(&pTmp);
}
static Scalar randomColor(RNG& rng)
{
	int icolor = (unsigned)rng;
	return Scalar(icolor&255, (icolor>>8)&255, (icolor>>16)&255);
}
//�������Ҳ���
int Findcounter(	IplImage *GrayHand)
{
	IplImage *pTmp = cvCreateImage(cvGetSize(GrayHand),GrayHand->depth,GrayHand->nChannels);
	cvCopy(GrayHand,pTmp);
	CvSize size = cvGetSize(pTmp);
	//����߽�
	for(int i=0; i<size.height;  i++)
	{
		for(int j=0; j<size.width; j++)
		{
			if (i<=3||j>=size.width-3||j<=3||i>=size.height-3)
			{	
				CV_IMAGE_ELEM(pTmp,uchar,i,j) = 255;
			}

		}
	}
	//
	CvPoint center_point;
	find_center(pTmp,center_point);//Ѱ����������
	int i=0;
	int mode=CV_RETR_EXTERNAL;//��ȡ������ģʽ
	int contours_num=0;//ͼ������ȡ��������Ŀ
	cvCanny(pTmp,pTmp,50,150);//canny
	cvSmooth(pTmp,pTmp);//smoth
	IplImage *pContourImg = cvCreateImage(cvGetSize(pTmp),
		IPL_DEPTH_8U,3);
	/*cvFindContours������������*/
	mode=CV_RETR_EXTERNAL;//��ȡ�������������
	if (storage==NULL)
	{
		storage = cvCreateMemStorage(0);
	}else{
		cvReleaseMemStorage(&storage);
		storage = cvCreateMemStorage(0);
	}
	contours_num=cvFindContours(pTmp,storage,&contour,
		sizeof(CvContour),mode,CV_CHAIN_APPROX_NONE);
	cout<<"������������ĿΪ��"<<contours_num<<" "<<endl;
	/*��㽫��������*/
	CvSeqReader reader;//������

	if (contour==NULL){
		cvReleaseImage(&pContourImg);
		cvReleaseImage(&pTmp);
		return 0;
	}

	//count=contour->total;
	cvStartReadSeq(contour,&reader,0);
	CvPoint pt1;
	//������Ѱ�Ұ���
	CvSeq* convexHull2 =cvConvexHull2(contour,storage,CV_CLOCKWISE,0);
	CvMemStorage* dftStorage = cvCreateMemStorage(0);
	CvSeq *defects=	cvConvexityDefects(contour,convexHull2,dftStorage);
	//cout<<"ao������"<<defects->total	<<endl;
	if (defects==NULL){
		cvReleaseMemStorage(&dftStorage);
		cvReleaseImage(&pContourImg);
		cvReleaseImage(&pTmp);
		return 0;
	}
		int count=0;
		count=defects->total;
		//cout<<count<<endl;
	cvStartReadSeq(defects,&reader,0);
	for (i=0;i<count;i++)//����ͼ��
	{
		CvConvexityDefect p;
		CV_READ_SEQ_ELEM(p,reader);//����һ����
		CvScalar color=CV_RGB(128,128,128);
		CvPoint start,depth_point,end;
		start=*p.start;
		depth_point=*p.depth_point;
		end=*p.end;
		float dist1=sqrt((float)(start.x-depth_point.x)*(start.x-depth_point.x)+(start.y-depth_point.y)*(start.y-depth_point.y));
		float dist2=sqrt((float)(end.x-depth_point.x)*(end.x-depth_point.x)+(end.y-depth_point.y)*(end.y-depth_point.y));
		float dist3=sqrt((float)(center_point.x-depth_point.x)*(center_point.x-depth_point.x)+(center_point.y-depth_point.y)*(center_point.y-depth_point.y));//�������ĵ�����ľ���
		float angle=180*acos((float)((start.x-depth_point.x)*(end.x-depth_point.x)+(start.y-depth_point.y)*(end.y-depth_point.y))/(dist1*dist2))/CV_PI;

		if ((p.depth>25&&angle>0&&angle<85)||((Max(dist1,dist2))>50&&p.depth>45&&angle>0&&angle<90)&&dist3<70)
		{

			cvCircle(pContourImg,*p.start,5,color);//����һ���㣨����һ����һ�����ش�СΪ�뾶��Բ��
			cvCircle(pContourImg,*p.depth_point,5,color);//����һ���㣨����һ����һ�����ش�СΪ�뾶��Բ��
			cvCircle(pContourImg,*p.end,3,color);//����һ���㣨����һ����һ�����ش�СΪ�뾶��Բ��
			cvLine(pContourImg,*p.start,*p.end,color,2);
			figures.push_back(start);
			figures.push_back(depth_point);
			figures.push_back(end);
		}	
		cvCircle(pContourImg,center_point,3,color);//����һ���㣨����һ����3�����ش�СΪ�뾶��Բ��

	}


	if (figures.size()==0)
	{

		DrawThinalgorithm( GrayHand);//ʶ��һ����ָͷ����6

	}	
	cvReleaseMemStorage(&dftStorage);
	cvReleaseImage(&pTmp);
	cvReleaseImage(&pContourImg);
	return 0;
}

//// this function is called each frame
//void glutDisplay (void)
//{
//
//	glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
//	// Setup the OpenGL viewpoint
//	glMatrixMode(GL_PROJECTION);//��ͶӰ����Ӧ�����ľ������.
//	glPushMatrix();// ����ǰջ�����еľ�������ѹһ������ǰ�����ɺ���glMatrixMode()ָ��������ջ�����󣬲�����ѹ���ջ��������ջ�����������������������ͬ�����ѹ��ľ�����࣬�����´���
//	glLoadIdentity();//���õ�ǰָ���ľ���Ϊ��λ����
//	XnMapOutputMode mode;//x,y,fps,��ȡxy�����غ�ÿ�����
//	g_DepthGenerator.GetMapOutputMode(mode);
//	glOrtho(0, mode.nXRes, mode.nYRes, 0,-1.0, 1.0);
//	glDisable(GL_TEXTURE_2D);
//
//	// Read next available data
//	g_Context.WaitAndUpdateAll();
//	// Update NITE tree
//	g_pSessionManager->Update(&g_Context);
//	PrintSessionState(g_SessionState);//���״̬
//	// get the depth map    
//	xn::DepthMetaData m_DepthMD;  
//	xn::ImageMetaData m_ImageMD; 
//	Mat  m_depth16u( 480,640,CV_16UC1);
//	Mat  m_rgb8u( 480,640,CV_8UC3);
//	Mat  m_DepthShow( 480,640,CV_8UC1);
//	Mat  m_RGBShow( 480,640,CV_8UC1);
//	g_DepthGenerator.GetMetaData(m_DepthMD);  
//	memcpy(m_depth16u.data,m_DepthMD.Data(), 640*480*2);  //������ݿ������
//
//	// get the image map    
//	g_ImageGenerator.GetMetaData(m_ImageMD);  
//	memcpy(m_rgb8u.data,m_ImageMD.Data(),640*480*3);  
//	cvtColor(m_rgb8u,m_RGBShow,CV_RGB2BGR);
//	//��δ֪���תΪ��ɫ��������OPENCV�з���  
//	XnDepthPixel* pDepth = (XnDepthPixel*)m_depth16u.data;  
//	pad.x=g_pDrawer->HandPostion.X;
//	pad.y=g_pDrawer->HandPostion.Y;
//	pad.z=g_pDrawer->HandPostion.Z;
//	for (XnUInt y = 0; y < m_DepthMD.YRes(); ++y)  
//	{  
//		for (XnUInt x = 0; x < m_DepthMD.XRes(); ++x, ++pDepth)  
//		{  
//			if (*pDepth == 0)  
//			{  
//				*pDepth = 0xFFFF;  
//			}  
//			if(pad.x!=0&&pad.y!=0&&pad.z!=0){//�����ֺ�����
//				if (fabs(*pDepth-g_pDrawer->HandPostion.Z) >50)
//				{
//					*pDepth = 0xFFFF;  
//				}
//			}
//
//		}  
//	}  
//
//	//����OpenNI��õ����ͼƬ��16λ�޷�����������OpenCV��ʾ����8λ�ģ�����Ҫ��ת����  
//	//������ת��Ϊ�Ҷ�ֵ��0-2550mm ת���� 0-255��������1000����ת��Ϊ 1000��255/2550 = 100  
//	//m_depth16u.convertTo(m_DepthShow,CV_8U, 255/2096.0);  
//	m_depth16u.convertTo(m_DepthShow,CV_8U, 255/2550.0);  
//	switch (g_SessionState)
//	{
//	case IN_SESSION:{
//		putText(m_RGBShow,"Tracking hands",cvPoint(20,20),CV_FONT_HERSHEY_SIMPLEX | CV_FONT_ITALIC,0.5,cvScalar(0,255,255));
//		break;}
//	case NOT_IN_SESSION:
//		{
//			g_pDrawer->HandPostion.X=0;
//			g_pDrawer->HandPostion.Y=0;
//			g_pDrawer->HandPostion.Z=0;
//			putText(m_RGBShow,"Perform click or wave gestures to track hand",cvPoint(20,20),CV_FONT_HERSHEY_SIMPLEX | CV_FONT_ITALIC,0.5,cvScalar(0,255,255));
//
//			break;
//		}
//	case QUICK_REFOCUS:{
//		putText(m_RGBShow,"Raise your hand for it to be identified, or perform click or wave gestures",cvPoint(20,20),CV_FONT_HERSHEY_SIMPLEX | CV_FONT_ITALIC,0.5,cvScalar(0,255,255));
//		g_pDrawer->HandPostion.X=0;
//		g_pDrawer->HandPostion.Y=0;
//		g_pDrawer->HandPostion.Z=0;
//
//		break;
//					   }
//	}
//
//	imshow("RGB",m_RGBShow);
//	IplImage rgbshow=IplImage(m_RGBShow);
//	IplImage Depth_Img;
//	Depth_Img = IplImage(m_DepthShow); // cv::Mat -> IplImage
//	double  thd_max = 0xFFFF;  
//	double  thd_val = 100.0;  
//	cvThreshold (&Depth_Img,&Depth_Img, thd_val, thd_max, CV_THRESH_BINARY);  //��ֵ��
//	CvPoint pt1,pt2;
//	//��������ָʶ�����
//	if(pad.x!=0&&pad.y!=0&&pad.z!=0){//��������������������
//		pt1=cvPoint(pad.x-100,pad.y-100);
//		pt2=cvPoint(pad.x+100,pad.y+100);
//		//���滮����Χ
//		CvFont font;
//		double hscale = 1.0;
//		double vscale = 1.0;
//		int linewidth = 2;
//		cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX | CV_FONT_ITALIC,hscale,vscale,0,linewidth);
//
//		if (pt1.x<0)
//		{
//
//			cvPutText(&rgbshow,"The pad is out of range",cvPoint(100,100),&font,cvScalar(0,255,255));
//			cvShowImage("depth", &Depth_Img);
//			cvShowImage("RGB", &rgbshow);
//			cvReleaseMemStorage(&storage);
//				glutSwapBuffers();
//			return;
//		}
//		if (pt1.y<0)
//		{
//
//			cvPutText(&rgbshow,"The pad is out of range",cvPoint(100,100),&font,cvScalar(0,255,255));
//			cvShowImage("depth", &Depth_Img);
//			cvShowImage("RGB", &rgbshow);
//			cvReleaseMemStorage(&storage);
//				glutSwapBuffers();
//			return;
//		}
//		if (pt2.x>=Depth_Img.width)
//		{
//
//			cvPutText(&rgbshow,"The pad is out of range",cvPoint(100,100),&font,cvScalar(0,255,255));
//			cvShowImage("depth", &Depth_Img);
//			cvShowImage("RGB", &rgbshow);
//			cvReleaseMemStorage(&storage);
//				glutSwapBuffers();
//			return;
//		}
//		if (pt2.y>=Depth_Img.height)
//		{
//
//			cvPutText(&rgbshow,"The pad is out of range",cvPoint(100,100),&font,cvScalar(0,255,255));
//			cvShowImage("depth", &Depth_Img);
//			cvShowImage("RGB", &rgbshow);
//			cvReleaseMemStorage(&storage);
//				glutSwapBuffers();
//			return;
//		}
//		cvRectangle(&Depth_Img,cvPoint(pt1.x-5,pt1.y-5),cvPoint(pt2.x+5,pt2.y+5),cvScalarAll(0),2);
//		////����ͼ�������
//		cvSetImageROI(&Depth_Img,cvRect(pad.x-100,pad.y-100,200,200));
//		IplImage  *GrayHand=cvCreateImage(cvSize(200,200),Depth_Img.depth,1);
//		cvCopy(&Depth_Img,GrayHand);
//		cvResetImageROI(&Depth_Img);
//		//figures.clear();
//		//figures2.clear();
//		//RNG rng(0xFFFFFFFF);
//		//storage=cvCreateMemStorage(0);//��������ʱ��Ҫ�Ĵ洢����
//		//Findcounter(GrayHand);//ʶ���㷨
//		//CvSeqReader reader;//������
//		//CvPoint2D32f minCircleCenter;
//		//CvPoint  pt1;
//		//int count=0;
//		//if (contour!=NULL)
//		//{
//		//	count=contour->total;
//		//	cvStartReadSeq(contour,&reader,0);
//		//	CvPoint2D32f *cvData=new CvPoint2D32f[count];
//		//	for (int i=0;i<count;i++)//����ͼ��
//		//	{
//		//		CV_READ_SEQ_ELEM(pt1,reader);//����һ����
//		//		CvScalar color=CV_RGB(255,255,0);
//		//		cvCircle(&rgbshow,cvPoint(pt1.x+pad.x-100,pt1.y+pad.y-100),1,color);//����һ���㣨����һ����һ�����ش�СΪ�뾶��Բ��
//		//		cvData[i].x = pt1.x+pad.x-100;
//		//		cvData[i].y = pt1.y+pad.y-100;
//		//	}
//		//	//��Բ����hand������
//		//	CvMat pointsMat = cvMat(1,count,CV_32FC2,cvData);
//		//	float minCircleRadius;
//		//	cvMinEnclosingCircle(&pointsMat, &minCircleCenter, &minCircleRadius);
//		// cvCircle(&rgbshow,cvPoint((int)minCircleCenter.x,minCircleCenter.y),minCircleRadius,randomColor(rng),2);
//		// delete []cvData;
//		////	circle(Mat(&rgbshow), cvPoint(minCircleCenter.x,minCircleCenter.y),rng.uniform(0, 250), randomColor(rng),
//		//	//	rng.uniform(-1, 9), CV_AA);
//		//}
//
//	
//		//string str[]={"A","B","C","D","E"};
//		//count=0;
//
//		//if (figures.size()!=0)//������
//		//{
//		//
//		//	cvCircle(&rgbshow,cvPoint(figures[0].x+pad.x-100,figures[0].y+pad.y-100),3,cvScalar(255,0,0),2);
//		//	cvLine( &rgbshow,cvPoint(figures[0].x+pad.x-100,figures[0].y+pad.y-100),cvPoint(minCircleCenter.x,minCircleCenter.y), cvScalar(0,0,255), 2,8 );
//		//	cvPutText(&rgbshow,str[count].c_str(),cvPoint(figures[0].x+pad.x-100,figures[0].y+pad.y-100),&font,randomColor(rng));
//		//	for (int j=0;j<figures.size();j+=3)
//		//	{
//		//		count++;
//		//		cvCircle(&rgbshow,cvPoint(figures[j+2].x+pad.x-100,figures[j+2].y+pad.y-100),3,cvScalar(255,0,0),2);
//		//		cvLine( &rgbshow,cvPoint(figures[j+2].x+pad.x-100,figures[j+2].y+pad.y-100),cvPoint(minCircleCenter.x,minCircleCenter.y), cvScalar(0,0,255), 2,8 );
//		//		cvPutText(&rgbshow,str[count].c_str(),cvPoint(figures[j+2].x+pad.x-100,figures[j+2].y+pad.y-100),&font,randomColor(rng));
//		//	
//		//	}
//		//}
//		//if (figures2.size()!=0)//����������ʧ��ʱ��������
//		//{
//
//		//	for (int j=0;j<figures2.size();j+=2)
//		//	{
//		//		cvCircle(&rgbshow,cvPoint(figures2[j+1].x+pad.x-100,figures2[j+1].y+pad.y-100),3,cvScalar(255,0,0),2);
//		//		cvLine( &rgbshow,cvPoint(figures2[j+1].x+pad.x-100,figures2[j+1].y+pad.y-100),cvPoint(minCircleCenter.x,minCircleCenter.y), cvScalar(0,0,255), 2,8 );
//		//		cvPutText(&rgbshow,str[count].c_str(),cvPoint(figures2[j+1].x+pad.x-100,figures2[j+1].y+pad.y-100),&font,randomColor(rng));
//		//	}
//
//		//}
//		//pad.x=minCircleCenter.x;
//		//pad.y=minCircleCenter.y;
//		//cvCircle(&rgbshow,cvPoint(pad.x,pad.y),2,randomColor(rng),2);
//		//cvCircle(&rgbshow,cvPoint(pad.x,pad.y),30,randomColor(rng),2);
//		cvShowImage("RGB",&rgbshow);//������ڽڵ��򸲸���ʾ
//		cvShowImage("depth", &Depth_Img);
//		cvReleaseImage(&GrayHand);	
//
//	}
//	cvReleaseMemStorage(&storage);
//	glutSwapBuffers();
//
//}
//
//
//void glutIdle (void)
//{
//	if (g_bQuit) {
//		CleanupExit();
//	}
//
//	// Display the frame
//	glutPostRedisplay();
//}
//
//void glutKeyboard (unsigned char key, int x, int y)
//{
//	switch (key)
//	{
//	case 27:
//		// Exit
//		CleanupExit();
//	case'p':
//		// Toggle pause����ͣӦ�ó���
//		g_bPause = !g_bPause;
//		break;
//	case 'd':
//		// Toggle drawing of the depth map
//		g_bDrawDepthMap = !g_bDrawDepthMap;
//		g_pDrawer->SetDepthMap(g_bDrawDepthMap);
//		break;
//	case 'f':
//		g_bPrintFrameID = !g_bPrintFrameID;
//		g_pDrawer->SetFrameID(g_bPrintFrameID);
//		break;
//	case 's':
//		// Toggle smoothing
//		if (g_fSmoothing == 0)
//			g_fSmoothing = 0.1;
//		else 
//			g_fSmoothing = 0;
//		g_HandsGenerator.SetSmoothing(g_fSmoothing);
//		break;
//	case 'e':
//		// end current session
//		g_pSessionManager->EndSession();
//		break;
//	}
//}
//void glInit (int * pargc, char ** argv)
//{
//	glutInit(pargc, argv);
//	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
//	//glutInitWindowSize(GL_WIN_SIZE_X, GL_WIN_SIZE_Y);
//	//glutInitWindowPosition(200,200);
//	//glutCreateWindow ("����ʶ����ͼ");
//	//glutFullScreen();
//	//glutSetCursor(GLUT_CURSOR_CROSSHAIR);//�趨�����ʽ
//
//	//	glutKeyboardFunc(glutKeyboard);
//	glutDisplayFunc(glutDisplay);
//	glutIdleFunc(glutIdle);
//
//	glDisable(GL_DEPTH_TEST);
//	glEnable(GL_TEXTURE_2D);
//
//	glEnableClientState(GL_VERTEX_ARRAY);//���õ��ö����������С�
//	glDisableClientState(GL_COLOR_ARRAY);//���õ�����ɫ����
//
//}


//״̬����
void state(SessionState g_SessionState,Mat &m_RGBShow )
{
	switch(g_SessionState)
	{
	case IN_SESSION:{
		putText(m_RGBShow,"Tracking hands",cvPoint(20,20),CV_FONT_HERSHEY_SIMPLEX | CV_FONT_ITALIC,0.5,cvScalar(0,255,255));
		break;
					}
	case NOT_IN_SESSION:
		{
			g_pDrawer->HandPostion.X=0;
			g_pDrawer->HandPostion.Y=0;
			g_pDrawer->HandPostion.Z=0;
			putText(m_RGBShow,"Perform click or wave gestures to track hand",cvPoint(20,20),CV_FONT_HERSHEY_SIMPLEX | CV_FONT_ITALIC,0.5,cvScalar(0,255,255));

			break;
		}
	case QUICK_REFOCUS:{
		putText(m_RGBShow,"Raise your hand for it to be identified, or perform click or wave gestures",cvPoint(20,20),CV_FONT_HERSHEY_SIMPLEX | CV_FONT_ITALIC,0.5,cvScalar(0,255,255));
		g_pDrawer->HandPostion.X=0;
		g_pDrawer->HandPostion.Y=0;
		g_pDrawer->HandPostion.Z=0;

		break;
					   }
	}

}
int  RGBDepView(){//��ʾ����

	g_pSessionManager->Update(&g_Context);
	PrintSessionState(g_SessionState);//���״̬
	// get the depth map    
	xn::DepthMetaData m_DepthMD;  
	xn::ImageMetaData m_ImageMD; 
	Mat  m_depth16u( 480,640,CV_16UC1);
	Mat  m_rgb8u( 480,640,CV_8UC3);
	Mat  m_DepthShow( 480,640,CV_8UC1);
	Mat  m_RGBShow( 480,640,CV_8UC1);
	g_DepthGenerator.GetMetaData(m_DepthMD);  
	memcpy(m_depth16u.data,m_DepthMD.Data(), 640*480*2);  //������ݿ������
	// get the image map    
	g_ImageGenerator.GetMetaData(m_ImageMD);  
	memcpy(m_rgb8u.data,m_ImageMD.Data(),640*480*3);  
	cvtColor(m_rgb8u,m_RGBShow,CV_RGB2BGR);
	//��δ֪���תΪ��ɫ��������OPENCV�з���  
	XnDepthPixel* pDepth = (XnDepthPixel*)m_depth16u.data;  
	pad.x=g_pDrawer->HandPostion.X;
	pad.y=g_pDrawer->HandPostion.Y;
	pad.z=g_pDrawer->HandPostion.Z;
	for (XnUInt y = 0; y < m_DepthMD.YRes(); ++y)  
	{  
		for (XnUInt x = 0; x < m_DepthMD.XRes(); ++x, ++pDepth)  
		{  
			if (*pDepth == 0)  
			{  
				*pDepth = 0xFFFF;  
			}  
			if(pad.x!=0&&pad.y!=0&&pad.z!=0){//�����ֺ�����
				if (fabs(*pDepth-g_pDrawer->HandPostion.Z) >50)
				{
					*pDepth = 0xFFFF;  
				}
			}

		}  
	}  

	//����OpenNI��õ����ͼƬ��16λ�޷�����������OpenCV��ʾ����8λ�ģ�����Ҫ��ת����  
	//������ת��Ϊ�Ҷ�ֵ��0-2550mm ת���� 0-255��������1000����ת��Ϊ 1000��255/2550 = 100  
	//m_depth16u.convertTo(m_DepthShow,CV_8U, 255/2096.0);  
	m_depth16u.convertTo(m_DepthShow,CV_8U, 255/2550.0);  
	state(g_SessionState,m_RGBShow);
	imshow("RGB",m_RGBShow);
	IplImage rgbshow=IplImage(m_RGBShow);
	IplImage Depth_Img;
	Depth_Img = IplImage(m_DepthShow); // cv::Mat -> IplImage
	double  thd_max = 0xFFFF;  
	double  thd_val = 100.0;  
	cvThreshold (&Depth_Img,&Depth_Img, thd_val, thd_max, CV_THRESH_BINARY);  //��ֵ��
	CvPoint pt1,pt2;
	//��������ָʶ�����
	if(pad.x!=0&&pad.y!=0&&pad.z!=0){//��������������������
		pt1=cvPoint(pad.x-100,pad.y-100);
		pt2=cvPoint(pad.x+100,pad.y+100);
		//���滮����Χ
		CvFont font;
		double hscale = 1.0;
		double vscale = 1.0;
		int linewidth = 2;
		cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX | CV_FONT_ITALIC,hscale,vscale,0,linewidth);

		if (pt1.x<0)
		{

			cvPutText(&rgbshow,"The pad is out of range",cvPoint(100,100),&font,cvScalar(0,255,255));
			cvShowImage("depth", &Depth_Img);
			cvShowImage("RGB", &rgbshow);
			if (storage!=NULL)
			{
				cvReleaseMemStorage(&storage);
			}
			return 0;
		}
		if (pt1.y<0)
		{

			cvPutText(&rgbshow,"The pad is out of range",cvPoint(100,100),&font,cvScalar(0,255,255));
			cvShowImage("depth", &Depth_Img);
			cvShowImage("RGB", &rgbshow);
			if (storage!=NULL)
			{
				cvReleaseMemStorage(&storage);
			}
			return 0;
		}
		if (pt2.x>=Depth_Img.width)
		{

			cvPutText(&rgbshow,"The pad is out of range",cvPoint(100,100),&font,cvScalar(0,255,255));
			cvShowImage("depth", &Depth_Img);
			cvShowImage("RGB", &rgbshow);
			if (storage!=NULL)
			{
				cvReleaseMemStorage(&storage);
			}
			return 0;
		}
		if (pt2.y>=Depth_Img.height)
		{

			cvPutText(&rgbshow,"The pad is out of range",cvPoint(100,100),&font,cvScalar(0,255,255));
			cvShowImage("depth", &Depth_Img);
			cvShowImage("RGB", &rgbshow);
			if (storage!=NULL)
			{
				cvReleaseMemStorage(&storage);
			}

			return 0;
		}
		cvRectangle(&Depth_Img,cvPoint(pt1.x-5,pt1.y-5),cvPoint(pt2.x+5,pt2.y+5),cvScalarAll(0),2);
		////����ͼ�������
		cvSetImageROI(&Depth_Img,cvRect(pad.x-100,pad.y-100,200,200));
		IplImage  *GrayHand=cvCreateImage(cvSize(200,200),Depth_Img.depth,1);
		cvCopy(&Depth_Img,GrayHand);
		cvResetImageROI(&Depth_Img);
		figures.clear();
		figures2.clear();
		RNG rng(0xFFFFFFFF);

		Findcounter(GrayHand);//ʶ���㷨
		CvSeqReader reader;//������
		CvPoint2D32f minCircleCenter;
		CvPoint  pt1;
		int count=0;
		if (contour!=NULL)
		{
			count=contour->total;
			cvStartReadSeq(contour,&reader,0);
			CvPoint2D32f *cvData=new CvPoint2D32f[count];
			for (int i=0;i<count;i++)//����ͼ��
			{
				CV_READ_SEQ_ELEM(pt1,reader);//����һ����
				CvScalar color=CV_RGB(255,255,0);
				cvCircle(&rgbshow,cvPoint(pt1.x+pad.x-100,pt1.y+pad.y-100),1,color);//����һ���㣨����һ����һ�����ش�СΪ�뾶��Բ��
				cvData[i].x = pt1.x+pad.x-100;
				cvData[i].y = pt1.y+pad.y-100;
			}
			//��Բ����hand������
			CvMat pointsMat = cvMat(1,count,CV_32FC2,cvData);
			float minCircleRadius;
			cvMinEnclosingCircle(&pointsMat, &minCircleCenter, &minCircleRadius);
		 cvCircle(&rgbshow,cvPoint((int)minCircleCenter.x,minCircleCenter.y),minCircleRadius,randomColor(rng),2);
		 delete []cvData;
		//	circle(Mat(&rgbshow), cvPoint(minCircleCenter.x,minCircleCenter.y),rng.uniform(0, 250), randomColor(rng),
			//	rng.uniform(-1, 9), CV_AA);
		}


		string str[]={"A","B","C","D","E"};
		count=0;

		if (figures.size()!=0)//������
		{
		
			cvCircle(&rgbshow,cvPoint(figures[0].x+pad.x-100,figures[0].y+pad.y-100),3,cvScalar(255,0,0),2);
			cvLine( &rgbshow,cvPoint(figures[0].x+pad.x-100,figures[0].y+pad.y-100),cvPoint(minCircleCenter.x,minCircleCenter.y), cvScalar(0,0,255), 2,8 );
			cvPutText(&rgbshow,str[count].c_str(),cvPoint(figures[0].x+pad.x-100,figures[0].y+pad.y-100),&font,randomColor(rng));
			for (int j=0;j<figures.size();j+=3)
			{
				count++;
				cvCircle(&rgbshow,cvPoint(figures[j+2].x+pad.x-100,figures[j+2].y+pad.y-100),3,cvScalar(255,0,0),2);
				cvLine( &rgbshow,cvPoint(figures[j+2].x+pad.x-100,figures[j+2].y+pad.y-100),cvPoint(minCircleCenter.x,minCircleCenter.y), cvScalar(0,0,255), 2,8 );
				cvPutText(&rgbshow,str[count].c_str(),cvPoint(figures[j+2].x+pad.x-100,figures[j+2].y+pad.y-100),&font,randomColor(rng));
			
			}
		}
		if (figures2.size()!=0)//����������ʧ��ʱ��������
		{

			for (int j=0;j<figures2.size();j+=2)
			{
				cvCircle(&rgbshow,cvPoint(figures2[j+1].x+pad.x-100,figures2[j+1].y+pad.y-100),3,cvScalar(255,0,0),2);
				cvLine( &rgbshow,cvPoint(figures2[j+1].x+pad.x-100,figures2[j+1].y+pad.y-100),cvPoint(minCircleCenter.x,minCircleCenter.y), cvScalar(0,0,255), 2,8 );
				cvPutText(&rgbshow,str[count].c_str(),cvPoint(figures2[j+1].x+pad.x-100,figures2[j+1].y+pad.y-100),&font,randomColor(rng));
			}

		}
		pad.x=minCircleCenter.x;
		pad.y=minCircleCenter.y;
		cvCircle(&rgbshow,cvPoint(pad.x,pad.y),2,randomColor(rng),2);
		cvCircle(&rgbshow,cvPoint(pad.x,pad.y),30,randomColor(rng),2);
		g_pDrawer->Draw();
for (int i=0;i<sizeof(g_pDrawer->m_pfPositionBuffer)/sizeof(XnFloat);i+=3)
{
		cvCircle(&rgbshow,cvPoint(g_pDrawer->m_pfPositionBuffer[i] ,g_pDrawer->m_pfPositionBuffer[i+1] ),4,randomColor(rng),2);
}
		


		cvShowImage("RGB",&rgbshow);//������ڽڵ��򸲸���ʾ
		cvShowImage("depth", &Depth_Img);
		cvReleaseImage(&GrayHand);	
		cvReleaseMemStorage(&storage);
	}

}
// xml to initialize OpenNI
#define SAMPLE_XML_PATH "Sample-Tracking.xml"

int main(int argc, char ** argv)
{
	XnStatus rc = XN_STATUS_OK;


	// Initialize OpenNI
	rc = g_Context.InitFromXmlFile(SAMPLE_XML_PATH);
	CHECK_RC(rc, "InitFromXmlFile");
	rc = g_Context.FindExistingNode(XN_NODE_TYPE_IMAGE, g_ImageGenerator);//��ͼ��
	CHECK_RC(rc, "Find image generator");
	rc = g_Context.FindExistingNode(XN_NODE_TYPE_DEPTH, g_DepthGenerator);//�����
	CHECK_RC(rc, "Find depth generator");
	rc = g_Context.FindExistingNode(XN_NODE_TYPE_HANDS, g_HandsGenerator);//����
	CHECK_RC(rc, "Find hands generator");
	// 5. set map mode    
	XnMapOutputMode mapMode;   
	mapMode.nXRes = 640;    
	mapMode.nYRes = 480;   
	mapMode.nFPS = 30;   
	rc = g_DepthGenerator.SetMapOutputMode( mapMode );    
	rc = g_ImageGenerator.SetMapOutputMode( mapMode );    

	//���� Kinect �����������Ͳ�ɫ��������ڲ�ͬ��λ�ã����Ҿ�ͷ����Ĳ���Ҳ����ȫ��ͬ�����������������ȡ�õĻ������Щ΢�Ĳ���  
	//�������������ӽǵ�����RGB�����λ��  
	// 6. correct view port    
	g_DepthGenerator.GetAlternativeViewPointCap().SetViewPoint( g_ImageGenerator );   
	/******************������openNI�Ĳ���*************************/
	// Create NITE objects

	g_pSessionManager = new XnVSessionManager;
	rc = g_pSessionManager->Initialize(&g_Context, "Click,Wave", "RaiseHand");
	CHECK_RC(rc, "SessionManager::Initialize");
	g_pSessionManager->RegisterSession(NULL, SessionStarting, SessionEnding, FocusProgress);


	//�����ʰ����
	g_pDrawer = new XnVPointDrawer(1, g_DepthGenerator); 
	g_pFlowRouter = new XnVFlowRouter;//listen
	g_pFlowRouter->SetActive(g_pDrawer);
	g_pSessionManager->AddListener(g_pFlowRouter);
	g_pDrawer->RegisterNoPoints(NULL, NoHands);//Register a callback for when there are no longer any points

	g_pDrawer->SetDepthMap(g_bDrawDepthMap);//�ر�

	// Initialization done. Start generating
	rc = g_Context.StartGeneratingAll();
	CHECK_RC(rc, "StartGenerating");
	//����OPenGL�Ĵ���
	// Mainloop
	//glInit(&argc, argv);
	//glutMainLoop();
	XnMapOutputMode mode;//x,y,fps,��ȡxy�����غ�ÿ�����
	g_DepthGenerator.GetMapOutputMode(mode);
	// Read next available data
	rc = g_Context.WaitNoneUpdateAll();  
	char key=0;
	while( (key!=27) && !(rc = g_Context.WaitNoneUpdateAll( ))  ) 
	{  
		//g_Context.WaitAndUpdateAll();
		// Update NITE tree
		RGBDepView();
		key=cvWaitKey(20);
	}


	// 10. stop  
	g_Context.StopGeneratingAll();
	g_Context.Shutdown();  

	return 0;
}
