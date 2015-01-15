#include <stdio.h>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include "opencv2/video/tracking.hpp"
#include "opencv2/calib3d/calib3d.hpp"

//#include <iostream>
#include <math.h>

#include <fstream>
#include <sstream>
#include <dirent.h>
#include <fnmatch.h>
#include <iostream>
//#include <math.h>
#include <cstdlib>
#include <time.h>
//#include "cvplot.h"	
//#include "epnp.h"
//##include <iostream>
#define EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET
#include <vector>
#include <unsupported/Eigen/MatrixFunctions>
//#include "SO3.h"
//#include "SE3.h"
#include <Eigen/Sparse>
#include "planarSFM.hpp"


#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

/*#include <Eigen/Core>
#include <Eigen/StdVector>
#include <Eigen/Geometry>

#include "basicConstants.h"*/

 

//#define NUM_MATCHES 500

using namespace cv;
using namespace std;
//using namespace newSLAM;
using namespace Eigen;
using namespace pe;

//template<typename T>

const double uc = 607.192;
const double vc = 185.215;
const double fu = 718.856;
const double fv = 718.856;
//Mat F = (Mat_<double>(3,3) << 0, 0, 0, 0, 0, -1, 0, 1, 0);
//Mat C1 = (Mat_<float>(3,4) << 718.856,0.000,607.192,0.000,0.000,718.856,185.215,0.000,0.000, 0.000,1.000,0.000);//389.956, 0, 254.903, 0, 0, 389.956, 201.899, 0, 0, 0, 1,0);
//Mat C2 = (Mat_<float>(3,4) << 718.856,0.000,607.192,-386.144,0.000,718.856,185.215,0.000,0.000, 0.000,1.000,0.000);//389.956, 0, 254.903, 0, 0, 389.956, 201.899, 0, 0, 0, 1,0);
Mat Cam1 = (Mat_<float>(3,3) << 1856.9,0,0,0,1856.9,0,0,0,1);//913.1033,0,354.726,0,907.36,257.24,0,0,1);//525.0, 0.0, 319.5,0, 525.0, 239.5, 0, 0 ,1.0);//389.956, 0, 254.903, 0, 0, 389.956, 201.899, 0, 0, 0, 1,0);
//Mat Cam1=(Mat_<double>(3,3) << 389.956, 0, 254.903, 0, 389.956, 201.899, 0, 0, 1);//, 1,0)
//Mat C2 = (Mat_<float>(3,4) << 389.956, 0, 254.903, 46.194,0, 389.956, 201.899, 0, 0, 0, 1,0);
Mat Ipose=(Mat_<float>(3,4) << 1, 0, 0, 0, 0, 1, 0, 0, 0,0,1,0);//,0,0,0,1);
const char *lreg, *rreg;
int count1=0;
//typedef Eigen::SparseMatrix<double> SpMat; // declares a column-major sparse matrix type of double
//typedef Eigen::Triplet<double> Tm;

//Mat points3d;
vector<Point2f> Corners,Corners1;


int isLeft(struct dirent const *entry)
{
	//printf("hello %s \n",entry->d_name);
  return !fnmatch(lreg, entry->d_name,0);
}

int isRight(struct dirent const *entry)
{
 // printf("hello 12%s \n",entry->d_name);
  return !fnmatch(rreg, entry->d_name,0);
}

vector<Point2f> Tracking(Mat img1,Mat img2,vector<Point2f>corners)
{
		vector<Point2f>corners1;
		Size winSize1 = Size( 101,101 );
		int maxLevel=3;	
		Mat status,err;
		TermCriteria criteria1 = TermCriteria( CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.001 );
		cout<<"hello"<<corners.size()<<"hello"<<endl;		
		calcOpticalFlowPyrLK(img1,img2,corners,corners1,status,err,winSize1,maxLevel,criteria1,0,1e-4);
		return corners1;
}	

void Detection(Mat img1,Mat img2,vector<Point2f>& corners,vector<Point2f>& corners1)
{
		
		int minHessian = 1;
		SurfFeatureDetector detector( minHessian );
		vector<KeyPoint> keypoints_1, keypoints_2,keypoints1,keypoints2;
		detector.detect( img1, keypoints_1 );
		detector.detect( img2, keypoints_2 );
		
		cout<<keypoints_1.size()<<endl;
		//-- Step 2: Calculate descriptors (feature vectors)
		SurfDescriptorExtractor extractor;
		Mat descriptors_1, descriptors_2;


		extractor.compute( img1, keypoints_1, descriptors_1 );
		extractor.compute( img2, keypoints_2, descriptors_2 );
		
		//-- Step 3: Matching descriptor vectors with a brute force matcher
		//BruteForceMatcher< L2<float> > matcher;
		BFMatcher matcher(NORM_L2);
		vector< DMatch > matches1,matches2;
		matcher.match( descriptors_1, descriptors_2, matches1 );
		matcher.match( descriptors_2, descriptors_1, matches2 );
	//	cout<<matches.size()<<keypoints_1.size()<<keypoints_2.size()<<endl;
	//	cout<<matches[0].queryIdx<<matches[0].trainIdx<<matches[0].imgIdx<<matches[0].distance<<endl;
		int x=0;
		float thresh=0.2;
		if(matches1.size()<=matches2.size())
		{
			if(matches1.size()==matches2.size())
			{
				
				for(int i=0;i<matches1.size();i++)
				{
					if(matches1[i].distance<thresh && i == matches2[matches1[i].trainIdx].trainIdx)
					{
						corners.push_back(keypoints_1[matches1[i].queryIdx].pt);
						corners1.push_back(keypoints_2[matches1[i].trainIdx].pt);
					}
				
				}
			
			}
			
			else
			{
				for(int i=0;i<matches1.size();i++)
				{
					if(matches1[i].distance<thresh && i == matches2[matches1[i].trainIdx].trainIdx)
					{
						corners.push_back(keypoints_1[matches1[i].queryIdx].pt);
						corners1.push_back(keypoints_2[matches1[i].trainIdx].pt);
					}
				
				}
			}
		}	
		else
		{
			for(int i=0;i<matches2.size();i++)
			{
				if(matches2[i].distance<thresh && i == matches1[matches2[i].trainIdx].trainIdx)
				{
					corners.push_back(keypoints_1[matches2[i].trainIdx].pt);
					corners1.push_back(keypoints_2[matches2[i].queryIdx].pt);
				}
			}
		}
		
		float winsize=10;
		for(int i =0;i<corners.size();i++)
		{ 
	//	  keypoints2[i].pt=Tracking(img1,img2,keypoints1[i].pt);
		  if( corners1[i].y > (corners[i].y+winsize) || (corners1[i].y) < (corners[i].y-winsize) )
		  {
			  corners.erase(corners.begin()+i);
			  corners1.erase(corners1.begin()+i);
			  i=i-1;
			  
		//  mask1.push_back(descriptors1.row(num[i]));
		  }
		}//*/
		
//	 	Corners=corners;  //here COMMENT 
//		Corners1=corners1;
		cout<<corners.size()<<corners1.size()<<endl;//*/
	/*	for(int i=0; i<corners1.size();i++)
		{
			line(img1,corners[i],corners1[i], Scalar(0,0,255), 1);
		}
		
		imshow("matches", img1);
		waitKey(0);*/
	
	
	//	cout<<keypoints1.size()<<keypoints2.size()<<endl;
			
			
	/*	//-- Draw matches
		Mat img_matches;
		drawMatches( img1, keypoints1, img2, keypoints2, matches1, img_matches );
		
		//-- Show detected matches
		imshow("Matches", img_matches );
		waitKey(0);*/ 

	/*	float scoreType=ORB::HARRIS_SCORE;
		int num_matches=5000;			
		ORB orb(num_matches,1.2f,4,31,0,2,scoreType,31);			
		//vector<Point2f> corners;
		vector<KeyPoint> keypoints1, keypoints2;
					
		
		Mat descriptors1, descriptors2,mask;			
		orb.operator() (img1,mask,keypoints1,descriptors1,false);
		vector<int>num (keypoints1.size());
		for(int i=0;i<keypoints1.size();i++)
		{
		  num[i]=i;
		}
			
		//cout<<scoreType<<"oiyuu"<<endl;
						
 	for(int i=0;i<keypoints1.size();i++)
		{
			//cout<<keypoints1.size()<<endl;
			for(int j=i+1;j<(keypoints1.size());j++)
				{
									
				   // if(i!=j)
				//	{	
														
						float dist=sqrt(pow((keypoints1[i].pt.x-keypoints1[j].pt.x),2)+pow((keypoints1[i].pt.y-keypoints1[j].pt.y),2));
				  	     
						if(dist<5 )
						{
							num.erase(num.begin()+j);
							keypoints1.erase(keypoints1.begin()+j);
			//				keypoints2.erase(keypoints2.begin()+j);
							j=j-1; 
						}
					  //mat1.push_back(descriptor) 
				//	}
				}
		}//*/
	
	//	cout<<keypoints1.size()<<endl;  //COMMENTED HERE
		//Mat mask1;//(0,32,CV_32FC1);
	//	Mat mask2;//(0,32,CV_32FC1);
//	cout<<corners.size()<<endl;
	/*		for(int i =0;i<keypoints1.size();i++)// COMMENT HERE
			{
				corners.push_back(keypoints1[i].pt);
			} 
//	cout<<corners.size()<<endl;			
			Size winSize = Size( 20, 20 );
			Size zeroZone = Size( -1, -1 );
			TermCriteria criteria = TermCriteria( CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 40, 0.001 );
			cornerSubPix( img1, corners, winSize, zeroZone, criteria );
		//	Corners=corners;
			corners1=Tracking(img1,img2,corners);
	//		cout<<corners.size()<<corners1.size()<<endl;

		float winsize=5;
		for(int i =0;i<corners.size();i++)
		{ 
	//	  keypoints2[i].pt=Tracking(img1,img2,keypoints1[i].pt);
		  if( corners1[i].y > (corners[i].y+winsize) || (corners1[i].y) < (corners[i].y-winsize) )
		  {
			  corners.erase(corners.begin()+i);
			  corners1.erase(corners1.begin()+i);
			  i=i-1;
			  
		//  mask1.push_back(descriptors1.row(num[i]));
		  }
		}
			//cout<<mask1.row(0)<<descriptors1.row(num[0])<<endl;
//	cout<<corners.size()<<corners1.size()<<endl;

			Corners=corners;
			Corners1=corners1;
//			disparity.push_back(corners-corners1);
		
	//	return 0;*/
		}	
		
vector<Point2f> Epipruning(vector<Point2f>& corners,vector<Point2f> corners1,Mat F)
{
	//	cout<<F<<endl;
	//	cout<<corners<<endl<<corners1<<endl;
		Mat coord= Mat(corners.size(),3,CV_64FC1);
		Mat coord1= Mat(corners.size(),3,CV_64FC1);
		for(int i=0;i<corners.size();i++)
		{
		  coord.at<double>(i,0)=corners[i].x;
		  coord.at<double>(i,1)=corners[i].y;
		  coord.at<double>(i,2)=1;
		  coord1.at<double>(i,0)=corners1[i].x;
		  coord1.at<double>(i,1)=corners1[i].y;
		  coord1.at<double>(i,2)=1;

		}	
				
		Mat m = (coord1*F)*coord.t();
		
		for( int i = 0; i < corners.size(); i++ )
		{ 
			float dist =m.at<float>(i,i) ;
			//cout<<m<<endl;
		//cout<<dist<<endl;
			if( dist > 0.05 ) 
			  {
				corners.erase(corners.begin()+i);
				corners1.erase(corners1.begin()+i);
				Corners.erase(Corners.begin()+i);
				Corners1.erase(Corners1.begin()+i);
				
			   }	
		}
		return corners1;
	}

void help()
{
	
	printf("\nThis program demonstrates using features2d detector, descriptor extractor and simple matcher\n"
			"Using the SURF desriptor:\n"
			"\n"
			"Usage:\n matcher_simple <image1> <image2>\n"
			);
			
}

void Mousehandler(int event, int x, int y,int , void * param){
		
		if(event == CV_EVENT_LBUTTONDOWN){
			std::stringstream ss;
			count1++;
			ss<<count1;
			string buffer=ss.str();
			Corners.push_back(Point(x,y));
	//		circle(image_global,Point(x,y),1,Scalar(0,255,0),2);
	//		putText(image_global,buffer,Point(x,y),FONT_HERSHEY_PLAIN,1.0,Scalar(0,255,0),1);
		}
}

vector<Point3f> triangulate(vector<Point2f> corners,vector<Point2f> corners1, Mat C1 , Mat C2)
{
		Mat points2d= Mat(2,corners.size(),CV_32FC1);
		Mat points2d1= Mat(2,corners.size(),CV_32FC1);
		for(int i =0;i<corners.size();i++)
		{
			points2d1.at<float>(0,i)=corners[i].x;
			points2d1.at<float>(1,i)=corners[i].y;
		}
		
		vector<Point3f> points3d;// = Mat(3,corners.size(),CV_64FC1);
//		triangulatePoints(C11,C12,points2d,points2d1,points3d);
		cout<<corners.size()<<corners1.size()<<endl;
		for(int i=0;i<corners.size();i++)
		{
			Mat w;
			//float wt,wt1=0;
			float tmp2=1;
			float tmp3=1;
			float weight =1;
			float weight1=1;
			int q=0;
			Point3f point3d;
			//cout<<q<<endl;
			//while(tmp2>=0.01 && tmp3>=0.01)
			//{
				//cout
				Mat A, u ,vt;
				//cout<<q<<endl;
				q=q+1;
			//	cout<<tmp2<<endl;
				float wt=weight;
				float wt1=weight1;
			//	cout<<wt<<endl;
				A.push_back<float>((corners[i].x*C1.row(2)-C1.row(0))/weight1);
				A.push_back<float>((corners[i].y*C1.row(2)-C1.row(1))/weight1);
				A.push_back<float>((corners1[i].x*C2.row(2)-C2.row(0))/weight);
				A.push_back<float>((corners1[i].y*C2.row(2)-C2.row(1))/weight);
				
		//		cout<<A.row(0)<<endl;
				Mat B(1,4,CV_32FC1);
				B =- A.col(3);
				A=A.colRange(0,3).clone();		
				//cout<<A.rows<<A.cols<<B.rows<<endl;
				
				solve(A,B,w,DECOMP_SVD);
				//hconcat(w,1,w);
			//	cout<<w<<endl;
				
				vt.push_back<float>(1);
				vt.push_back<float>(w.at<float>(0,2));
				vt.push_back<float>(w.at<float>(0,1));
				vt.push_back<float>(w.at<float>(0,0));
			//	cout<<vt<<endl;
				Mat tmp=C1.row(2);
				Mat tmp1=C2.row(2);
				//cout<<tmp<<vt.rows<<endl;
				tmp=tmp*vt;
				tmp1=tmp1*vt;
				//cout<<(C1.row(2)*w.t());
				weight=tmp.at<float>(0,0);
				weight1=tmp1.at<float>(0,0);
			//	cout<<weight<<endl;
				tmp2=abs(weight-wt);
				tmp3=abs(weight1-wt1);					
		//	}
		//	cout<<w<<endl;
		//points3d.push_back(w);
	//	cout<<points3d<<endl;
		
			
				point3d.x=w.at<float>(0,0);
				point3d.y=w.at<float>(1,0);
				point3d.z=w.at<float>(2,0);
				points3d.push_back(point3d);
			//cout<<A.size()<<endl;
		}	
	
	return points3d;
	}
	

int compareFn ( const void *pa, const void *pb ) {
	const float *a = *(const float **)pa;
	const float *b = *(const float **)pb;
	if(a[2] > b[2])
		return 1;
	return 0;
}

int main(int argc, char** argv)
{
	IplImage *limg, *rimg, *img_col;
	struct dirent **limgs, **rimgs;
	int nlimgs,nrimgs=0;	
	RNG rng( 0xFFFFFFFF);
	Mat D = (Mat_<double>(3,1) << 0,0,0);
	Mat fundamental_matrix;
	

	//Matrix3d R1=exp();
//	cout<<R1<<endl;
  /**
   * Preparing a list of left and right images.
   * If left and right images are in the same folder then there will be three args: <path to images folder> <reg expr for left> <reg ecpr for right>
   * If lett and right images are in seperate folder then there will be two args: <path to left images folder> <path to right images folder>
   */
	if(argc < 3)
	{
		help();
		return -1;
	}
	
	
	if(argc==3)
	{
		lreg = "*";
		rreg = "*";
	}
	else
	{
		cout<<"argv = "<<argv[2]<<", "<<argv[3]<<endl;
		lreg = argv[2];
		rreg = argv[3];
	}
	//Mat lm,rm;
	nlimgs = scandir(argv[1], &limgs, isLeft, versionsort);
	if(argc==3)
		nrimgs = scandir(argv[2], &rimgs, isRight, versionsort);
	else
		nrimgs = scandir(argv[1], &rimgs, isRight, versionsort);

	printf("left=%d, right=%d\n", nlimgs, nrimgs);
//	printf("l = %s, r = %s\n", limgs[10]->d_name, rimgs[10]->d_name);
//return 0;
	if(nlimgs <0 || nrimgs <0)
	  {
		if(argc==3 && nrimgs<0)
		  perror(argv[2]);
		else
		  perror(argv[1]);
		return 0;
	  }
	if(nlimgs != nrimgs)
	  {
		printf("Number of left images and right images is not equal: left=%d, right%d.\n", nlimgs, nrimgs);
		return 0;
	  }
	
    int count = argc==4?2:2;
    std::stringstream fn,fl;
 	Mat img1;
	
	
	int z=20;//(nlimgs)/16 ;
//	cout<<z<<endl;
//	return 0;
	vector<Point2f> Fcorners,Fcorners1;
	vector< vector <Point3f> > Points3D;
	Mat CPose;
	CPose.push_back(Ipose);
	
/*	Vector6d vrot;
	VectorXd vrot1(12*(z-1));
	VectorXd vrot2(6*(z+1));
	VectorXd vrot3(6*(z+1));
	vrot1.setZero(12*(z-1));
//	vrot2.setIdentity(6*(z+1));
/*	MatrixXd mrot;
	MatrixXd mrot1;
	MatrixXd mrot2;
//	mrot2.setIdentity(248,248);
	mrot.setZero(12*z,6*(z+2));*/
//	mrot1.setZero(12*z,4*(z+2));
	
	
	Mat T3d;
	
	
//	fstream //myfile;
//	ofstream //myfile1;
	ofstream myfile;
//	ofstream //myfile3;
//
//	//myfile.open("/home/prateek/Desktop/example.txt",ios::in);
	//myfile1.open("/home/itachi/Desktop/example6_o.txt",ios_base::app);
	myfile.open("/home/itachi/Desktop/matches.txt",ios_base::app);
	//myfile3.open("/home/itachi/Desktop/example8_o.txt",ios_base::app);
	//myfile4.open("/home/itachi/Desktop/example9.txt",ios_base::app);
	
	//myfile1<<"--------------------------------------"<<endl;
	myfile<<"--------------------------------------"<<endl;
	//myfile3<<"--------------------------------------"<<endl;
	//myfile4<<"--------------------------------------"<<endl;
	vector <Vector3d> mvec;
	//vector <Vector6d> vvec;
	vector <Mat> DPoses;
	vector <Mat> FPoses;
	//vector <Mat> Points3D;
	DPoses.push_back(CPose);
	cout<<DPoses[0]<<endl;
//	//myfile3<<Ipose;
	int tempx=0;		
//	Eigen::Triplet SparseTriplet;
//	vector<Tm> tripletList;
//	vector<Tm> tripletList;
//	triplets.reserve(12*(z+1));
//	triplets.reserve(12*(z+1));
		
	
/*	while(!//myfile.eof())//t i=0;i<z;i++)
	{
		double b;
		//myfile>>b;
//		cout<<b<<endl;
		vrot(tempx)=b;
		if(tempx==5)
		{
			cout<<vrot<<endl;
			tempx=-1;
			vvec.push_back(vrot);
		}
		
		tempx+=1;		
	}
	
	
	for(int i=0;i<32;i++)
	{
		Matrix4d Tre=exp(vvec[i]);
		//myfile4<<Tre;
		cout<<Tre<<endl;
	}
	
	
	//myfile4.close();*/
/*	Mat mrot3 = Mat(3,3,CV_64FC1);
	
	for(int x=0;x<120;x++)
	{
		for(	int i=0;i<3;i++)
		{
			for(int j=0;j<3;j++)
			{
				double a;
				//myfile>>a;
			//    cout<<a<<endl;
				mrot3.at<double>(i,j)=a;
			}
		}
//	cout<<mrot3<<endl;	
	mvec.push_back(logn(quattomat(mrot3)));
//	cout<<mvec[x]<<endl;
	}
	*/
//	cout<<mvec<<endl;
//	return 0;
/*	MatrixXd brot;
	brot.setZero(480,248);
	for(int i=0;i<480;i++)
	{
		for(int j=0;j<4548;j++)
		{
			double a;
			//myfile>>a;
			if( i<480 && j <248)
			{
		//		cout<<a<<endl;
				brot(i,j)=a;
			}
		}
	}*/
	
//	int cols=6*(30+1);
//	SpMat mrot(12*(z-1),6*(z+1));
//	mrot.reserve(VectorXi::Constant(cols,2));	
	Mat ZPose;
	int p=0;
	int num=0;//100;
	cout<<Ipose.rowRange(0,2)<<endl;
	Mat C1=Cam1*Ipose.rowRange(0,3);
	Mat C2;
	cout<<C1<<endl;
	vector<Mat> TPoses;
	vector<Mat> RPoses;
	vector<Mat> NPoses;
	
	
	for(int y=0;y<z;y++)
	{ 		
		
		  vector<Point2f> corners,corners1;
	//	int x=0;
	//	char c=0;
		vector<Point2f> inliers1,inliers2,outliers1,outliers2;
		vector< vector<Point2f> > totcorners,totcorners1;
		vector<Mat> homographies;
		
		int	c=0;
		Mat img3,img6;
		while(c < 1 )
		{
		 	cout<<c<<endl;
			Mat img2,opoints3d,img4,img5;
			int x=0;
	//		putchar(c);
		//	c=getchar();
			int p=0;
//			p+=1;
//			
		//		Corners.clear()
		while(x<2)
		{		
			cout<<(((y)+count)+x)<<endl;
		//	cout<<limgs[11]->d_name<<endl;
		//	cout<<argv[1]<<endl;
		//	cout<<argv[1]<<"/"<<limgs[11]->d_name;
			fn<<argv[1]<<"/"<<limgs[((y)+count)+x+num/*+c*/]->d_name;
			fl<<argv[1]<<"/"<<limgs[((y)+count)+num/*+c*/]->d_name;
			printf("fn=%s\n", fn.str().c_str());
			limg = cvLoadImage(fn.str().c_str());
			img_col=cvLoadImage(fn.str().c_str());
			//("left", img_col);
			//cvWaitKey(1000);
			if(!limg)
			{
			  printf("image %s was not loaded, hence exiting.\n", limgs[count]->d_name);
			  break;
			}
			//printf("Number of left images and right images is not equal:\n");
			fn.str("");
			if(argc==4)
			  fn<<argv[1]<<"/"<<rimgs[((y)+count)+(x)+num]->d_name;
			else
			  fn<<argv[2]<<"/"<<rimgs[((y)+count)+(x)+num]->d_name;
			printf("fn=%s\n", fn.str().c_str());
			rimg =cvLoadImage(fn.str().c_str(), CV_LOAD_IMAGE_GRAYSCALE);
			if(!rimg)
			{
			  printf("image %s was not loaded, hence exiting.\n", rimgs[count]->d_name);
			  break;
			}
			fn.str("");


			img1=limg;
			img2=rimg;
			img5=img_col;
	//		Mat img6= img1+img3;
			
					
			if(x==0 )//&& c==0 )
			{
			//	cout<<x<<endl;
				img3=img1;
				img4=img2;
				img6=img5;
				Detection(img1,img2,corners,corners1);
			//	imshow("a",img3);
			//	waitKey(0);
				//cout<<img3.at<Vec3b>(100,100)[1]<<endl;
		/*		namedWindow("Image");
				setMouseCallback("Image", Mousehandler, 0 );

				cout<<"\n\n\t\t\t\t***Select four corners to be projected :::\n\n";
				cout<<"\t\t\t\t ***Press Esc to see output:::\n\n";
				imshow("Image",img3);

					for(; ;){
							imshow("Image",img3);
							char d = (char)waitKey(10);
								if(d==27){
								break;
								}
							}	
			    cout<<Corners<<endl;
				corners1.clear();
			    for(int i=0;i<corners.size();i++)
			    {
					
					if ( corners[i].x > Corners[0].x && corners[i].x < Corners[1].x && corners[i].y > Corners[0].y && corners[i].y < Corners[2].y )  
					{
						corners1.push_back(corners[i]);
					}
				}			
			  //  Detection(img3,img1,corners,corners1);
				Corners.clear();
				corners.clear();
				corners=corners1; //*/
															
			}
			else
			{
								
	
			 	corners1=Tracking(img3,img1,corners);	
	//		  	//myfile1<<corners<<endl<<corners1<<endl;
		//		fundamental_matrix =findFundamentalMat(corners, corners1, FM_RANSAC, 3, 0.99);
		//		//myfile3<<fundamental_matrix<<endl;
		//		corners1=Epipruning(corners,corners1,fundamental_matrix);						
	//			//myfile4<<corners<<endl<<corners1<<endl;
		//		corners1=Tracking(img1,img2,corners);						
			
								
	//		corners1=Epipruning(corners,corners1,F);
	/*			for(int i=0;i<corners1.size();i++)
				{
					line(img5,corners[i],corners1[i], Scalar(0,0,255), 1);
				}		
				imshow("matches3", img5);
				waitKey(100);	//	//*/	
			    cout<<corners.size()<<corners1.size()<<endl;
		//	    Corners.clear();
			}
			
			
	//		if (x==1 && c==0)  break;
	//		img6=img
			img3=img1;
			x=x+1;
		
		}	
		
		c=c+1;
		Mat H,R,T,R1,T1, N,N1;
		double reprojectionError=4;
	
		Mat H1=findHomography(Mat(corners),Mat(corners1),CV_RANSAC,4);
	
		homographies.push_back(H1);
		for(int i=0;i<corners.size();i++)
		{
			myfile<<corners[i].x<<" "<<corners[i].y<<" "<<corners1[i].x<<" "<<corners1[i].y<<endl;
		}//*/
		
		FILE *fp = fopen("/home/itachi/read.txt","r");
	
		int numHomographies = 0;

		fscanf(fp, "%d", &numHomographies);
		
	/*	for(int kk = 0; kk < numHomographies; kk++)
		{ 

			cout<<"Starting "<<endl;
				double Hmat[9];
			for(int j = 0; j < 9; j++) { 
				fscanf(fp, "%lf", &Hmat[j]);
			}
			Mat H1 = (Mat_<double>(3,3) << Hmat[0], Hmat[1], Hmat[2], Hmat[3], Hmat[4], Hmat[5], Hmat[6], Hmat[7], Hmat[8]);
			homographies.push_back(H1);
			
			
			corners.clear();
			corners1.clear();

			int numCorresp;
			fscanf(fp, "%d", &numCorresp);

			float am1, am2, an1, an2;
			float centroid1y =0;
			float centroid1x =0;
			float centroid2y =0;
			float centroid2x =0;
			for(int j = 0; j < numCorresp; j++)
			{
				fscanf(fp, "%f %f %f %f", &am1, &am2, &an1, &an2);
				centroid1x+=am1;
				centroid1y+=am2;
				centroid2x+=an1;
				centroid2y+=an2;
				corners.push_back(Point2f(am1, am2));
				corners1.push_back(Point2f(an1, an2));
			}
			//myfile3<<centroid1x/numCorresp<<" "<<centroid1y/numCorresp<<endl;
			//myfile3<<centroid2x/numCorresp<<" "<<centroid2y/numCorresp<<endl;
			
		//	centroid2x=centroid1x/numCorresp;
	//		centroid2y=centroid1x/numCorresp;
			
			//		cout<<H1<<endl;*/
			
			totcorners.push_back(corners);
			totcorners1.push_back(corners1);
			
		/*	for(int k = 0; k < numCorresp; k++) {
			
				float currX = corners[k].x;
				float currY = corners[k].y;
				float **distArr;
				distArr = (float **) malloc(sizeof(float *) * numCorresp);
				for(int kk = 0; kk < numCorresp; kk++) { 
					distArr[kk] = (float *) malloc(sizeof(float) * 5);
					distArr[kk][0] = 0.0f;
					distArr[kk][1] = 0.0f;
					distArr[kk][2] = 0.0f;
					distArr[kk][3] = 0.0f;
					distArr[kk][4] = 0.0f;
				}

				vector< Point2f > corners_small, corners1_small;

				for(int kk = 0; kk < numCorresp; kk++) { 
					distArr[kk][0] = corners[kk].x;
					distArr[kk][1] = corners[kk].y;
					distArr[kk][3] = corners1[kk].x;
					distArr[kk][4] = corners1[kk].y;

					if(kk == k) { 
						distArr[kk][2] = 100000;	
					} else {
						distArr[kk][2] = sqrt(pow((currX - distArr[kk][0]),2) + pow((currY - distArr[kk][1]),2));
					}
				}

				
				qsort(distArr, numCorresp, sizeof(distArr[0]), compareFn);
		
				corners_small.clear();
				corners1_small.clear();
	
				for(int kk = 0; kk < 10; kk++) { 
					corners_small.push_back(Point2f(distArr[kk][0], distArr[kk][1]));
					corners1_small.push_back(Point2f(distArr[kk][3], distArr[kk][4]));
				}//*/
						
		//		planarSFM(Cam1, corners_small, corners1_small, inliers1, inliers2, H1, R, T,  R1, T1, N, N1, reprojectionError, img5);
		//		planarSFM(Cam1, corners, corners1, inliers1, inliers2, H1, R, T,  R1, T1, N, N1, reprojectionError, img5);
			
	//			cout<<R<<T<<R1<<T1<<N<<N1<<endl;	
	//			cout<<fabs(T1.at<float>(2,0))<<fabs(T.at<float>(2,0))<<endl;
		//	//myfile1<<R<<R1<<endl;
			
			//myfile3<<N<<N1<<endl;
			
	/*			myfile<<N<<N1<<"Normal-------------------"<<endl;
				myfile<<T<<T1<<"Translation-------------------"<<endl;
				myfile<<R<<R1<<"Rotation-------------------"<<endl;*/
				
				
		/*		if(fabs(T1.at<float>(1,0)) > fabs(T.at<float>(1,0)))
				{ 
				/*	if(T1.at<float>(1,0)>0)
					{
						T1=-T1;
						N1=-N1;
					}//*/
		/*			for(int i=0;i<3;i++)
					{
					R1.col(i).copyTo(CPose.col(i));
				}
				T1.copyTo(CPose.col(3));
				R=R1;
				T=T1;
				N=N1;
			//	SFMwithSBA( Cam1,inliers1,inliers2,R,T,4);		
			}
			else
			{		
				
		/*		if(T.at<float>(2,0)>0)
				{
					T=-T;
					N=-N;
				} //*/
		/*		for(int i=0;i<3;i++)
				{
					R.col(i).copyTo(CPose.col(i));
				}
				T.copyTo(CPose.col(3));
			//	SFMwithSBA( Cam1,inliers1,inliers2,R,T,4);
			}
			
	//		cout<<CPose<<endl;
					//	SFMwithSBA( Cam1,inliers1,inliers2,R*/
	//	myfile<<N.at<float>(0,0)<<" "<<N.at<float>(1,0)<<" "<<N.at<float>(2,0)<<endl;			
	 	//myfile<<N.at<float>(0,0)<<" "<<N.at<float>(1,0)<<" "<<N.at<float>(2,0)<<" "<<H1.at<float>(0,0)<<" "<<H1.at<float>(0,1)<<" "<<H1.at<float>(0,2)<<" "<<H1.at<float>(1,0)<<" "<<H1.at<float>(1,1)<<" "<<H1.at<float>(1,2)<<" "<<H1.at<float>(2,0)<<" "<<H1.at<float>(2,1)<<" "<<H1.at<float>(2,2)<<" "<<endl;
	//		myfile<<N<<endl;
		//	for(int i=0;i<corners.size();i++)
		//	{
		//		int rpoint=img3.at<Vec3b>(ceil(corners[k].y),ceil(corners[k].x))[1];
		//		int gpoint=img3.at<Vec3b>(ceil(corners[k].y),ceil(corners[k].x))[2];
		//		int bpoint=img3.at<Vec3b>(ceil(corners[k].y),ceil(corners[k].x))[3];
			//	int ypoint=ceil(corners[i].y);
			//	cout<<xpoint<<ypoint<<img3.at<Vec3b>(xpoint,ypoint)[1]<<endl;
			//	myfile<<corners[k].x<<" "<<corners[k].y<<" "<<corners1[k].x<<" "<<corners1[k].y<<" "<<rpoint<<" "<<gpoint<<" "<<bpoint<<" "<<N.at<float>(0,0)<<" "<<N.at<float>(1,0)<<" "<<N.at<float>(2,0)<<" "<<H1.at<float>(0,0)<<" "<<H1.at<float>(0,1)<<" "<<H1.at<float>(0,2)<<" "<<H1.at<float>(1,0)<<" "<<H1.at<float>(1,1)<<" "<<H1.at<float>(1,2)<<" "<<H1.at<float>(2,0)<<" "<<H1.at<float>(2,1)<<" "<<H1.at<float>(2,2)<<" "<<kk<<" "<<endl;
		//		myfile<<corners[k].x<<" "<<corners[k].y<<" "<<rpoint<<" "<<gpoint<<" "<<bpoint<<" "<<N.at<float>(0,0)<<" "<<N.at<float>(1,0)<<" "<<N.at<float>(2,0)<<" "<<kk<<" "<<endl;//H1.at<float>(0,0)<<" "<<H1.at<float>(0,1)<<" "<<H1.at<float>(0,2)<<" "<<H1.at<float>(1,0)<<" "<<H1.at<float>(1,1)<<" "<<H1.at<float>(1,2)<<" "<<H1.at<float>(2,0)<<" "<<H1.at<float>(2,1)<<" "<<H1.at<float>(2,2)<<" "<<kk<<" "<<endl;
		//	} 
			
	/*		Mat tempA=(Mat_<float>(1,4) << 0,0,0,1);
			Mat CPose1;
			vconcat(CPose,tempA,CPose1);
			C2=C1*CPose1;
		//	myfile<<C2<<endl;
		  }*/
	
	/*		if(p==1)
			{
				C2=C1*CPose1;
			}	*/
	//		C2 = (Mat_<float>(3,4) << 912.27679, 4.3920441, 356.81879, -74.128639,-7.4215641, 905.51685, 263.56335, -31.983906,-0.0022499214, -0.0069899005, 0.99997282, -0.27353272);

  
  
	//		cout<<C2<<endl;
		//	
			/*
			vector<Point3f> points3d = triangulate( corners, corners1, C1, C2);
			
			//myfile4 << "Writing points" << endl << points3d.size()<<endl<<points3d << endl;
			
	//		double error =SFMwithSBA( Cam1,corners,corners1,R,T,4,points3d);
			
	//		//myfile4 << "Writing points SBA" << endl << points3d << endl;
			pcl::PointCloud<pcl::PointXYZ> cloud;
			cloud.width  = points3d.size();
			cloud.height = 1;
			cloud.points.resize (cloud.width * cloud.height);
			
			for (size_t i = 0; i < cloud.points.size (); ++i)
			{
				cloud.points[i].x = points3d[i].x;
				cloud.points[i].y = points3d[i].y;
				cloud.points[i].z = points3d[i].z;
				
			}

			
			pcl::io::savePCDFileASCII ("test_pcd.pcd", cloud);
			pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
		    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
		   // Create the segmentation object
		    pcl::SACSegmentation<pcl::PointXYZ> seg;
		  // Optional
		//    seg.setOptimizeCoefficients (true);
		  // Mandatory
		    seg.setModelType (pcl::SACMODEL_PLANE);
		    seg.setMethodType (pcl::SAC_RANSAC);
		    seg.setDistanceThreshold (1);

		  seg.setInputCloud (cloud.makeShared ());
		  seg.segment (*inliers, *coefficients);

		  if (inliers->indices.size () == 0)
		  {
			PCL_ERROR ("Could not estimate a planar model for the given dataset.");
			return (-1);
		  }

		  //myfile1<< "Model coefficients: " << coefficients->values[0] << " " 
											  << coefficients->values[1] << " "
											  << coefficients->values[2] << " " 
											  << coefficients->values[3] << std::endl<<"--------------"<<endl;;

		   //myfile4 << "Model inliers: " << inliers->indices.size () << std::endl; */

	/*	  for (size_t i = 0; i < inliers->indices.size (); ++i)
			//myfile4 << inliers->indices[i] << "    " << cloud.points[inliers->indices[i]].x << " "
													   << cloud.points[inliers->indices[i]].y << " "
													   << cloud.points[inliers->indices[i]].z << std::endl;

	//	*/	
/*
			cout<<inliers->indices.size ()<<endl;
			for(int i=0;i<4;i++)
			{
				N.at<float>(i,0) =coefficients->values[i];
			}
			cout<<corners.size()<<endl;
	//		C1=C2;
			Points3D.push_back(points3d);
			TPoses.push_back(T);
			RPoses.push_back(R);
			NPoses.push_back(N);
*/
			
	//		putchar(c);
			
				
			/*	for(int i=0;i<inliers1.size();i++)
				{
				line(img5,inliers1[i],inliers2[i], Scalar(0,0,255), 1);
				}		
				imshow("matches3", img5);
				waitKey(500); //**/
	/*		//myfile1 << "Normal1: " << N.t() << endl << "Normal2: " << N1.t() << endl;
			myfile<<R<<endl<<R1<<endl;
			//myfile3<<T.t()<<endl<<T1.t()<<endl;
			//myfile4<<H1<<endl;//<<T1.t()<<endl;*/
		
		 }	
		 
		 float error;
		for(int i=0;i<homographies.size();i++)
		{
			//myfile4<<endl<<"error"<<i<<endl;
			for(int j=0;j<homographies.size();j++)
			{
				
				computeHomographyInliers(totcorners[j], totcorners1[j],homographies[i],inliers1,inliers2,outliers1,outliers2,error,myfile);
			}
			myfile<<endl;
		}
		
	/*	 for(int i=0;i<TPoses.size();i++)
		 {
			 cout<<TPoses[i]*NPoses[i].at<float>(3,0)<<endl;
		 
		 } //*/
		 cout<<2323<<endl;
	return 0;	
	}
	cout<<2323<<endl;
	return 0;	
	}
//		return 0;
    
//}			
			/* */
			
			
