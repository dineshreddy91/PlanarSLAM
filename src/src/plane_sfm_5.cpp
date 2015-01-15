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
//#include "posest/epnp.h"
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
#include <sba/sba.h>

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

const double uc = 319.5;
const double vc = 239.5;
const double fu = 525.0;
const double fv = 525.0;
//Mat F = (Mat_<double>(3,3) << 0, 0, 0, 0, 0, -1, 0, 1, 0);
//Mat C1 = (Mat_<float>(3,4) << 718.856,0.000,607.192,0.000,0.000,718.856,185.215,0.000,0.000, 0.000,1.000,0.000);//389.956, 0, 254.903, 0, 0, 389.956, 201.899, 0, 0, 0, 1,0);
//Mat C2 = (Mat_<float>(3,4) << 718.856,0.000,607.192,-386.144,0.000,718.856,185.215,0.000,0.000, 0.000,1.000,0.000);//389.956, 0, 254.903, 0, 0, 389.956, 201.899, 0, 0, 0, 1,0);
Mat Cam1 = (Mat_<float>(3,3) << 525.0, 0.0, 319.5,0, 525.0, 239.5, 0, 0 ,1.0);//913.1033,0,354.726,0,907.36,257.24,0,0,1);//1856.9,0,0,0,1856.9,0,0,0,1);//////389.956, 0, 254.903, 0, 0, 389.956, 201.899, 0, 0, 0, 1,0);
//Mat Cam1=(Mat_<double>(3,3) << 389.956, 0, 254.903, 0, 389.956, 201.899, 0, 0, 1);//, 1,0)
//Mat C2 = (Mat_<float>(3,4) << 389.956, 0, 254.903, 46.194,0, 389.956, 201.899, 0, 0, 0, 1,0);
Mat Ipose=(Mat_<float>(3,4) << 1, 0, 0, 0, 0, 1, 0, 0, 0,0,1,0);//,0,0,0,1);
const char *lreg, *rreg;
int count1=0;
//typedef Eigen::SparseMatrix<double> SpMat; // declares a column-major sparse matrix type of double
//typedef Eigen::Triplet<double> Tm;

//Mat points3d;
vector<Point2f> Corners,Corners1;
vector<KeyPoint> KeyCorners,KeyCorners1;

ofstream myfile;

//ofstream myfile3;
//m/yfile3.open("/home/prateek/Desktop/example8_o.txt",ios_base::app);

int isLeft(struct dirent const *entry)
{
	//printf("hello %s \n",entry->d_name);
  return !fnmatch(lreg, entry->d_name,0);
}
/*Mat pnp(int n,vector<Point2f> image2d,Mat real3d ,Mat& R,Mat& T)
{
  epnp PnP;
  double R_true[3][3], t_true[3];
  srand(time(0));
  PnP.set_internal_parameters(uc, vc, fu, fv);
  PnP.set_maximum_number_of_correspondences(n);
  for(int i = 0; i < n; i++)
  {
          PnP.add_correspondence(real3d.at<double>(0,i), real3d.at<double>(1,i), real3d.at<double>(2,i), image2d[i].x, image2d[i].y);
  }
	cout<<"here"<<endl;
  Mat pose=Mat(3,4,CV_64FC1);
  double R_est[3][3], t_est[3];
  double err2 = PnP.compute_pose(R_est, t_est);
PnP.print_pose(R_est, t_est);
  //cout<<R<<endl;
  for(int i=0;i<3;i++)
  {
        T.at<double>(i,0)=pose.at<double>(i,3)=t_est[i];
          for(int j=0;j<3;j++)
        {
                R.at<double>(i,j)=pose.at<double>(i,j)=R_est[i][j];
        }
  }
      
    return pose;
    cout << "Found pose:" <<endl;
//      
}
*/
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

void Detection(Mat img1,Mat img2,vector<Point2f>& corners,vector<Point2f>& corners1,vector<KeyPoint>& keycorners,vector<KeyPoint>& keycorners1,vector<int>& indices,ofstream &myfile)
{
		
		int minHessian = 1;
		SurfFeatureDetector detector( minHessian );
		vector<KeyPoint> keypoints_1, keypoints_2,keypoints1,keypoints2;
		detector.detect( img1, keypoints_1 );
		detector.detect( img2, keypoints_2 );//*/
		
		cout<<keypoints_1.size()<<endl;
		//-- Step 2: Calculate descriptors (feature vectors)
		
	/*	for(int i=0;i<keypoints_1.size();i++)
		{
			corners.push_back(keypoints_1[i].pt);
		}
		
		
		Size winSize = Size( 30, 30 );
		Size zeroZone = Size( -1, -1 );
		TermCriteria criteria = TermCriteria( CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 40, 0.001 );
		cornerSubPix( img1, corners, winSize, zeroZone, criteria );
	//	Corners=corners;
		vector<Point2f> corners2=Tracking(img1,img2,corners);
		
		for(int i=0;i<corners2.size();i++)
		{
			KeyPoint temp;
			temp.pt=corners2[i];
			keypoints_2.push_back(temp);
		}*/
		
		
		if(keycorners.size()!=0)
		{
			cout<<"in loop"<<endl;
			keypoints_1=keycorners;
			keypoints_2=keycorners1;
			cout<<keycorners.size()<<keycorners1.size()<<endl;
		}//*/
		
	//	cout<<"here"<<endl;
		SurfDescriptorExtractor extractor;
		Mat descriptors_1, descriptors_2;
		
		extractor.compute( img1, keypoints_1, descriptors_1 );
		extractor.compute( img2, keypoints_2, descriptors_2 );//*/
		
	/*	Mat descriptors_1, descriptors_2;
		vector<KeyPoint> keypoints_1, keypoints_2,keypoints1,keypoints2;
		float scoreType=ORB::HARRIS_SCORE;
		int num_matches=5000;			
		ORB orb(num_matches,1.2f,4,31,0,2,scoreType,31);			
		//vector<Point2f> corners;
//		vector<KeyPoint> keypoints1, keypoints2;
					
		
		Mat descriptors1, descriptors2,mask;			
		orb.operator() (img1,mask,keypoints_1,descriptors_1,false); 
		orb.operator() (img2,mask,keypoints_2,descriptors_2,false); */
		
		
	
		 
		
		//-- Step 3: Matching descriptor vectors with a brute force matcher
	//	BruteForceMatcher< L2<float> > matcher;
	
		BFMatcher matcher(NORM_L2);
		vector<vector<DMatch> > matches12,matches23;
		vector< DMatch > matches3,matches4;
	//	matcher.match( descriptors_1, descriptors_2, matches1 );
	//	matcher.match( descriptors_2, descriptors_1, matches2 );
	//	cout<<"here"<<endl;
	
	/*	FlannBasedMatcher matcher2,matcher3;
		
		vector<Mat> train_desc(1,descriptors_1);
		vector<Mat> train_desc1(1,descriptors_2);
		matcher2.add(train_desc);
		matcher3.add(train_desc1);
		matcher2.train();
		matcher3.train();
		cout<<"here"<<endl;
	//	vector<DMatch> matches3,matches4;
		matcher2.match(descriptors_2,matches3);
		matcher3.match(descriptors_1,matches4);
	//	matcher2.match(descriptors_2,descriptors_1,matches4);//*/
	
	/*	Mat imageMatches;
		drawMatches(img1, keypoints_1, img2, keypoints_2, matches3, imageMatches, Scalar(255,255,255));
		imshow("matched",imageMatches);
		waitKey(0);//*/
	
		
		vector<DMatch> matches1,matches2;
		matcher.knnMatch( descriptors_1, descriptors_2, matches12,3 );
		matcher.knnMatch( descriptors_2, descriptors_1, matches23,3 );
	//	cout<<matches.size()<<keypoints_1.size()<<keypoints_2.size()<<endl;
	//	cout<<matches[0].queryIdx<<matches[0].trainIdx<<matches[0].imgIdx<<matches[0].distance<<endl;
	
		cout<<"before ratio test "<<matches12.size()<<matches23.size()<<endl;
		
	//	if(matches12.size() <= matches23.size())
	//	{
		
			for(int i=0;i<matches12.size();i++)
			{
				DMatch _m;
		//		_m = matches12[i][0];				


				if(matches12[i].size()==1)
				{
						_m = matches12[i][0];
				}
				 else if(matches12[i].size()>1)
				 {
						if(matches12[i][0].distance / matches12[i][1].distance < 0.6)
						{
								_m = matches12[i][0];
						} else {
								continue; // did not pass ratio test
						}
				}
				 else 
				{
						continue; // no match
				}	//*/
				matches1.push_back(_m);
			}
	//	}
	//	else
	//	{
	//	
			for(int i=0;i<matches23.size();i++)
			{
				DMatch _m1;	
		//		_m1 = matches23[i][0];
						
			if(matches23[i].size()==1) {
					_m1 = matches23[i][0];
			} else if(matches23[i].size()>1) {
					if(matches23[i][0].distance / matches23[i][1].distance < 0.6) {
							_m1 = matches23[i][0];
					} else {
							continue; // did not pass ratio test
					}
			} else {
					continue; // no match
			}//	*/
				matches2.push_back(_m1);
			}
	//	}
	//	matches1=_m;
	//	matches2=_m1;
	
	/*	Mat img_matches;
		drawMatches( img1, keypoints_1, img2, keypoints_2, matches3, img_matches );*/
		
		//-- Show detected matches
	//	imshow("Matches", img_matches );
	//	waitKey(0);//*/ 
		cout<<"after ratio test "<<matches1.size()<<matches2.size()<<endl;//*/
		//cout<<matches1.size()<<"matches"<<endl;
		keycorners.clear();
		corners.clear();
		corners1.clear();
		keycorners1.clear();
		int x=0;
		float thresh=0.2;
		if(matches1.size()<=matches2.size())
		{
		/*	for(int i=0;i<matches1.size();i++)
			{
				corners.push_back(keypoints_1[matches1[i].queryIdx].pt);
				corners1.push_back(keypoints_2[matches1[i].trainIdx].pt);
				keycorners.push_back(keypoints_1[matches1[i].queryIdx]);
				keycorners1.push_back(keypoints_2[matches1[i].trainIdx]);
				indices.push_back(matches1[i].queryIdx);
			}*/
			if(matches1.size()==matches2.size())
			{
								
				for(int i=0;i<matches1.size();i++)
				{
					for (int j=0;j<matches2.size();j++)
					{
						if( matches1[i].trainIdx==matches2[j].queryIdx && matches1[i].queryIdx==matches2[j].trainIdx )
						{
			//				cout<<" "<<matches1[i].queryIdx;
							corners.push_back(keypoints_1[matches1[i].queryIdx].pt);
							corners1.push_back(keypoints_2[matches1[i].trainIdx].pt);
							keycorners.push_back(keypoints_1[matches1[i].queryIdx]);
							keycorners1.push_back(keypoints_2[matches1[i].trainIdx]);
							indices.push_back(matches1[i].queryIdx);
						//	matches2.erase(matches2.begin()+j);
						//	j=j-1;
						}
					}
				}
			
			}
			
			else
			{
				for(int i=0;i<matches1.size();i++)
				{
					for (int j=0;j<matches2.size();j++)
					{
						if( matches1[i].trainIdx==matches2[j].queryIdx && matches1[i].queryIdx==matches2[j].trainIdx)
						{
			//				cout<<" "<<matches1[i].queryIdx;
							corners.push_back(keypoints_1[matches1[i].queryIdx].pt);
							corners1.push_back(keypoints_2[matches1[i].trainIdx].pt);
							keycorners.push_back(keypoints_1[matches1[i].queryIdx]);
							keycorners1.push_back(keypoints_2[matches1[i].trainIdx]);
							indices.push_back(matches1[i].queryIdx);
				//			matches2.erase(matches2.begin()+j);
				//			j=j-1;
						}
					}
				}
			}//*/
		}	
		else
		{
			for(int i=0;i<matches2.size();i++)
			{
				
			/*	corners.push_back(keypoints_1[matches2[i].trainIdx].pt);
				corners1.push_back(keypoints_2[matches2[i].queryIdx].pt);
				keycorners.push_back(keypoints_1[matches2[i].trainIdx]);
				keycorners1.push_back(keypoints_2[matches2[i].queryIdx]);
				indices.push_back(matches2[i].trainIdx);*/
				
				for (int j=0;j<matches1.size();j++)
				{
					if(matches2[i].trainIdx==matches1[j].queryIdx && matches2[i].queryIdx==matches1[j].trainIdx)
					{
				//		cout<<" "<<matches1[i].queryIdx;
						corners.push_back(keypoints_1[matches2[i].trainIdx].pt);
						corners1.push_back(keypoints_2[matches2[i].queryIdx].pt);
						keycorners.push_back(keypoints_1[matches2[i].trainIdx]);
						keycorners1.push_back(keypoints_2[matches2[i].queryIdx]);
						indices.push_back(matches2[i].trainIdx);
				//		matches1.erase(matches1.begin()+j);
				//		j=j-1;
					}
				}//*/
			}
	}
		cout<<corners.size()<<corners1.size()<<endl;
/*		for(int i=0; i<corners1.size();i++)
		{
			line(img2,corners[i],corners1[i], Scalar(0,0,255), 1);
		}
		
		
		
		imshow("matches", img2);
		waitKey(0);//*/
			
	/*	for(int i=0;i<matches1.size();i++)
		{
			corners.push_back(keypoints_1[matches1[i].queryIdx].pt);
			corners1.push_back(keypoints_2[matches1[i].trainIdx].pt);
			keycorners.push_back(keypoints_1[matches1[i].queryIdx]);
			keycorners1.push_back(keypoints_2[matches1[i].trainIdx]);
		}//*/
		
	/*	if(matches3.size()<=matches4.size())
		{
			
			cout<<"inloop gee"<<endl;
		/*	for(int i=0;i<matches1.size();i++)
			{
				corners.push_back(keypoints_1[matches1[i].queryIdx].pt);
				corners1.push_back(keypoints_2[matches1[i].trainIdx].pt);
				keycorners.push_back(keypoints_1[matches1[i].queryIdx]);
				keycorners1.push_back(keypoints_2[matches1[i].trainIdx]);
				indices.push_back(matches1[i].queryIdx);
			}*/
		/*	if(matches3.size()==matches4.size())
			{
								
				for(int i=0;i<matches3.size();i++)
				{
					for (int j=0;j<matches4.size();j++)
					{
						if( matches3[i].trainIdx==matches4[j].queryIdx && matches3[i].queryIdx==matches4[j].trainIdx)
						{
							corners.push_back(keypoints_1[matches3[i].queryIdx].pt);
							corners1.push_back(keypoints_2[matches3[i].trainIdx].pt);
							keycorners.push_back(keypoints_1[matches3[i].queryIdx]);
							keycorners1.push_back(keypoints_2[matches3[i].trainIdx]);
							indices.push_back(matches3[i].queryIdx);
						}
					}
				}
			
			}
			
			else
			{
				for(int i=0;i<matches3.size();i++)
				{
					for (int j=0;j<matches4.size();j++)
					{
						if( matches3[i].trainIdx==matches4[j].queryIdx && matches3[i].queryIdx==matches4[j].trainIdx)
						{
							corners.push_back(keypoints_1[matches3[i].queryIdx].pt);
							corners1.push_back(keypoints_2[matches3[i].trainIdx].pt);
							keycorners.push_back(keypoints_1[matches3[i].queryIdx]);
							keycorners1.push_back(keypoints_2[matches3[i].trainIdx]);
							indices.push_back(matches3[i].queryIdx);
						}
					}
				}
			}//*/
	/*	}	
		else
		{
			for(int i=0;i<matches4.size();i++)
			{
				
			/*	corners.push_back(keypoints_1[matches2[i].trainIdx].pt);
				corners1.push_back(keypoints_2[matches2[i].queryIdx].pt);
				keycorners.push_back(keypoints_1[matches2[i].trainIdx]);
				keycorners1.push_back(keypoints_2[matches2[i].queryIdx]);
				indices.push_back(matches2[i].trainIdx);*/
				
	/*			for (int j=0;j<matches4.size();j++)
				{
					if(matches4[i].trainIdx==matches3[j].queryIdx && matches4[i].queryIdx==matches3[j].trainIdx)
					{
						corners.push_back(keypoints_1[matches4[i].trainIdx].pt);
						corners1.push_back(keypoints_2[matches4[i].queryIdx].pt);
						keycorners.push_back(keypoints_1[matches4[i].trainIdx]);
						keycorners1.push_back(keypoints_2[matches4[i].queryIdx]);
						indices.push_back(matches4[i].trainIdx);
					}
				}//*/
	/*		}
		}
	
	//	Corners=corners;
	//	Corners1=corners;
//		
		cout<<corners.size()<<endl;
		Mat tempimg=img1;
		for(int i=0; i<corners.size();i++)
		{
			line(tempimg,corners[i],corners1[i], Scalar(0,0,255), 1);
		}
		
		imshow("matches1", tempimg);
		waitKey(1000);//*/
	
	/*	Size winSize = Size( 30, 30 );
		Size zeroZone = Size( -1, -1 );
		TermCriteria criteria = TermCriteria( CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 40, 0.001 );
		cornerSubPix( img1, corners, winSize, zeroZone, criteria );
	//	Corners=corners;
		vector<Point2f> corners2=Tracking(img1,img2,corners);//*/
		
	/*	for(int i=0; i<corners.size();i++)
		{
			line(img2,corners[i],corners2[i], Scalar(0,0,255), 1);
		}
		
		imshow("matches", img2);
		waitKey(500);//*/
		
//		cout<<corners.size()<<corners1.size()<<endl;
			
//		myfile<<corners1<<endl<<corners2<<endl;
	//	keycorners=KeyCorners;
	
	//	cout<<corners1<<corners2<<corners2.size()<<endl;
/*			int numtemp=0; 
		float winsize=10;
		for(int i =0;i<corners.size();i++)
		{ 
	//	  keypoints2[i].pt=Tracking(img1,img2,keypoints1[i].pt);
		//	cout<<abs(corners2[i].y-corners1[i].y )<<abs(corners2[i].x-corners1[i].x )<<endl;
		  if( (abs(corners2[i].y-corners1[i].y ) > winsize) || (abs(corners2[i].x-corners1[i].x) > winsize) ) //||  corners1[i].x > abs(corners2[i].x+winsize) ||  corners1[i].x < abs(corners2[i].x-winsize) )
		  {
			  corners.erase(corners.begin()+i);
			  keycorners.erase(keycorners.begin()+i);
			  corners1.erase(corners1.begin()+i);
			  keycorners1.erase(keycorners1.begin()+i);
			  corners2.erase(corners2.begin()+i);
			  i=i-1;
			  numtemp+=1;
			  
		//  mask1.push_back(descriptors1.row(num[i]));
		  }
		}//*/
		
//		corners1=corners2;
//		cout<<"in outer "<<numtemp<<endl;
		
//	 	Corners=corners;  //here COMMENT 
//		Corners1=corners1;
	//	cout<<corners1<<corners2<<endl;//*/
	/*	for(int i=0; i<corners1.size();i++)
		{
			line(img2,corners[i],corners1[i], Scalar(0,0,255), 1);
		}
		
		
		
		imshow("matches", img2);
		waitKey(500);//*/
	
	
	    


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


Mat triangulate(vector<Point2f> corners,vector<Point2f> corners1, Mat C1 , Mat C2)
{
		cout<<C1<<" "<<endl;
		cout<<C2<<" "<<endl;
		
		Mat points2d= Mat(2,corners.size(),CV_32FC1);
		Mat points2d1= Mat(2,corners.size(),CV_32FC1);
		for(int i =0;i<corners.size();i++)
		{
			points2d1.at<float>(0,i)=corners[i].x;
			points2d1.at<float>(1,i)=corners[i].y;
		}
		
	//	cout<<C1<<C2<<endl<<corners<<corners1<<endl;
	//	vector<Point3f> points3d;// = Mat(3,corners.size(),CV_64FC1);
//		triangulatePoints(C11,C12,points2d,points2d1,points3d);
//		cout<<corners<<corners1<<endl;
		Mat points3d=Mat(3,corners.size(),CV_32F);
		
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
		//		cout<<wt<<endl;
				A.push_back<float>((corners[i].x*C1.row(2)-C1.row(0))/weight1);
				A.push_back<float>((corners[i].y*C1.row(2)-C1.row(1))/weight1);
				A.push_back<float>((corners1[i].x*C2.row(2)-C2.row(0))/weight);
				A.push_back<float>((corners1[i].y*C2.row(2)-C2.row(1))/weight);
			//	cout<<"---------"<<A<<endl;
				
		//		myfile<<A<<endl;
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
		
				
				points3d.at<float>(0,i)=w.at<float>(0,0);
				points3d.at<float>(1,i)=w.at<float>(1,0);
				points3d.at<float>(2,i)=w.at<float>(2,0);
				myfile<<point3d<<" ";
		//		points3d.push_back(point3d);
			//cout<<A.size()<<endl;
		}	
	myfile<<endl;
	return points3d;
	}
	//*/


vector<Point3f> triangulate_n(vector< vector<Point2f> > corners,vector< vector<Point2f> > corners1, vector<Mat> totPoses,vector<vector<int> >& indices)
{
		
		Mat points2d= Mat(2,corners.size(),CV_32FC1);
		Mat points2d1= Mat(2,corners.size(),CV_32FC1);
	//	for(int i =0;i<corners.size();i++)
	//	{
	//		points2d1.at<float>(0,i)=corners[i].x;
	//		points2d1.at<float>(1,i)=corners[i].y;
	//	}
		cout<<"Corners1"<<corners[0].size()<<endl;
	//	cout<<"here"<<endl;
		vector<Point3f> points3d;
		myfile<<endl;
		int i=0;
		starting:
		if (i < corners[0].size())
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
			
			A.push_back<float>((corners[0][i].x*totPoses[0].row(2)-totPoses[0].row(0))/weight1);
			A.push_back<float>((corners[0][i].y*totPoses[0].row(2)-totPoses[0].row(1))/weight1);
			
	//		cout<<"here"<<corners1.size()<<corners1[1].size()<<i<<endl;
	//		cout<<"here"<<indices.size()<<endl;
		
			for (int j = 0; j < 1 /*(totPoses.size()-1)*/ ; j++)
			{
			// Project the point into the node's image coordinate system.
			//printf("val",(int) j);
//				cout<<"here12"<<j<<endl;
				for(int k=0;k<corners1[j].size();k++)
				{
		//			myfile<<"here"<<indices[j][k]<<" ";
					if(indices[0][i]==indices[j][k])
					{
						A.push_back<float>((corners1[j][k].x*totPoses[j+1].row(2)-totPoses[j+1].row(0))/weight1);
						A.push_back<float>((corners1[j][k].y*totPoses[j+1].row(2)-totPoses[j+1].row(1))/weight1);
						break;
					}
					
				
				}
			}	
			
			if(A.rows ==2)
			{
				i=i+1;
				goto starting;
				
			}
	//		cout<<"here"<<endl;
		//		myfile<<"A------------"<<A<<endl;
	//	cout<<corners.size()<<corners1.size()<<endl;
	/*	for(int i=0;i<corners.size();i++)
		{
			
			//	cout<<wt<<endl;
				A.push_back<float>((corners[i].x*C1.row(2)-C1.row(0))/weight1);
				A.push_back<float>((corners[i].y*C1.row(2)-C1.row(1))/weight1);
				A.push_back<float>((corners1[i].x*C2.row(2)-C2.row(0))/weight);
				A.push_back<float>((corners1[i].y*C2.row(2)-C2.row(1))/weight);*/
				
	//			cout<<A.rows<<endl;
				Mat B(1,A.rows,CV_32FC1);
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
			//		Mat tmp=C1.row(2);
			//	Mat tmp1=C2.row(2);
				//cout<<tmp<<vt.rows<<endl;
		/*		tmp=tmp*vt;
				tmp1=tmp1*vt;
				//cout<<(C1.row(2)*w.t());
				weight=tmp.at<float>(0,0);
				weight1=tmp1.at<float>(0,0);
			//	cout<<weight<<endl;
				tmp2=abs(weight-wt);
				tmp3=abs(weight1-wt1);		*/			
		//	}
		//	cout<<w<<endl;
		//points3d.push_back(w);
	//	cout<<points3d<<endl;
		
			
				point3d.x=w.at<float>(0,0);
				point3d.y=w.at<float>(1,0);
				point3d.z=w.at<float>(2,0);
		//		cout<<point3d<<endl;
				points3d.push_back(point3d);
				i=i+1;
				goto starting;
			//cout<<A.size()<<endl;
		}	
		myfile<<endl;
	
	return points3d;
	}//*/
	
/*
vector<Point3f> triangulate_n(vector< vector<Point2f> > corners,vector< vector<Point2f> > corners1, vector<Mat> totPoses,vector<vector<int> >& indices)
{
		
		Mat points2d= Mat(2,corners.size(),CV_32FC1);
		Mat points2d1= Mat(2,corners.size(),CV_32FC1);
	//	for(int i =0;i<corners.size();i++)
	//	{
	//		points2d1.at<float>(0,i)=corners[i].x;
	//		points2d1.at<float>(1,i)=corners[i].y;
	//	}
		cout<<"Corners1"<<corners[0].size()<<endl;
	//	cout<<"here"<<endl;
		vector<Point3f> points3d;
		myfile<<endl;
		for (int i = 0; i < corners[0].size(); i++)
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
			
			A.push_back<float>((corners[0][i].x*totPoses[0].row(2)-totPoses[0].row(0))/weight1);
			A.push_back<float>((corners[0][i].y*totPoses[0].row(2)-totPoses[0].row(1))/weight1);
			
//			cout<<"here"<<totPoses.size()<<endl;
		
			for (int j = 0; j < 1 /*(totPoses.size()-1)*/ //; j++)
	//		{
			// Project the point into the node's image coordinate system.
			//printf("val",(int) j);
//				cout<<"here12"<<j<<endl;
	/*			for(int k=0;k<corners1[j].size();k++)
				{
					
					if(indices[0][i]==indices[j][k])
					{
				//		myfile<<"here"<<indices[j][k]<<" ";
						A.push_back<float>((corners1[j][k].x*totPoses[j+1].row(2)-totPoses[j+1].row(0))/weight1);
						A.push_back<float>((corners1[j][k].y*totPoses[j+1].row(2)-totPoses[j+1].row(1))/weight1);
						break;
					}
				}
			}	
			
	//		cout<<"here"<<endl;
		//		myfile<<"A------------"<<A<<endl;
	//	cout<<corners.size()<<corners1.size()<<endl;
	/*	for(int i=0;i<corners.size();i++)
		{
			
			//	cout<<wt<<endl;
				A.push_back<float>((corners[i].x*C1.row(2)-C1.row(0))/weight1);
				A.push_back<float>((corners[i].y*C1.row(2)-C1.row(1))/weight1);
				A.push_back<float>((corners1[i].x*C2.row(2)-C2.row(0))/weight);
				A.push_back<float>((corners1[i].y*C2.row(2)-C2.row(1))/weight);*/
				
		//		cout<<A.row(0)<<endl;
	/*			Mat B(1,A.rows,CV_32FC1);
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
			//		Mat tmp=C1.row(2);
			//	Mat tmp1=C2.row(2);
				//cout<<tmp<<vt.rows<<endl;
		/*		tmp=tmp*vt;
				tmp1=tmp1*vt;
				//cout<<(C1.row(2)*w.t());
				weight=tmp.at<float>(0,0);
				weight1=tmp1.at<float>(0,0);
			//	cout<<weight<<endl;
				tmp2=abs(weight-wt);
				tmp3=abs(weight1-wt1);		*/			
		//	}
		//	cout<<w<<endl;
		//points3d.push_back(w);
	//	cout<<points3d<<endl;
		
			
	/*			point3d.x=w.at<float>(0,0);
				point3d.y=w.at<float>(1,0);
				point3d.z=w.at<float>(2,0);
		//		cout<<point3d<<endl;
				points3d.push_back(point3d);
			//cout<<A.size()<<endl;
		}	
		myfile<<endl;
	
	return points3d;
	}//*/
	
/*
int compareFn ( const void *pa, const void *pb ) {
	const float *a = *(const float **)pa;
	const float *b = *(const float **)pb;
	if(a[2] > b[2])
		return 1;
	return 0;
}*/


void chooseDecomposition(Mat& R,Mat& T,Mat& N,Mat& R1,Mat& T1,Mat& N1)
{
	if(fabs(T1.at<float>(0,0)) > fabs(T.at<float>(0,0)))
	{
		if(T1.at<float>(0,0)<0)
		{
			T1=-T1;
			N1=-N1;
		}
		
	
		R=R1;
		T=T1;
		N=N1;
		
	}
	else
	{		
		if(T.at<float>(0,0)<0)
		{
			T=-T;
			N=-N;
		} 
		
	}
}


void sba_new(const Mat& intrinsics, vector<Mat>& rvec, vector<Mat>& tvec, vector<Point3f>& points, const vector<vector<Point2f> >& points1, const vector<vector<Point2f> >& points2,vector<vector<int> >& indices)
{
  printf("sba got %d points\n", (int)points.size());
  printf("sba got %d nodes\n", (int)rvec.size());
  // system
  sba::SysSBA sba0;
  sba0.verbose = 0;

  sba::Node nd0;
  Mat rvec0 = Mat::zeros(3, 1, CV_32F);
  Mat tvec0 = Mat::zeros(3, 1, CV_32F);
  initNode(intrinsics, rvec0, tvec0, nd0);
  nd0.isFixed = true;
  sba0.nodes.push_back(nd0);
  
  for(int i=1;i<rvec.size();i++)
  {
	  
	  
  // set up nodes
	  sba::Node nd1;
	  Mat sbaRvec = rvec[i];//.t();
	  Mat sbaTvec = tvec[i];//-(rvec[i].t()*tvec[i]);
	  Mat sbaR;
	  Rodrigues(sbaRvec, sbaR);
	 // sbaR = sbaR.t();	
	//  sbaTvec = tvec[i];
	//  sbaRvec = sbaRvec;
	  cout<<"vals"<<sbaRvec<<endl<<sbaR<<sbaTvec<<endl;
	  initNode(intrinsics, sbaR, sbaTvec, nd1);
	  
	 
	   nd1.isFixed = false;
	
	   sba0.nodes.push_back(nd1);

	  
  }
 // sba0.nodes.push_back(nd1);
//	cout<<"end"<<endl;
  // set up projections
  for (size_t i=0; i<points.size(); i++)
  {
    // add point
    sba0.addPoint(Vector4d(points[i].x, points[i].y, points[i].z, 1.0));
    //        printf("%f %f %f\n", points[i].x, points[i].y, points[i].z);
  }
  
  Vector2d proj;
  cout<<"added points "<<endl;
  
  for (size_t i = 0; i < points.size(); i++)
  {
	
	Vector2d p2(points1[0][i].x, points1[0][i].y);
	sba0.addMonoProj(0, i, p2);
	
	for (size_t j = 0; j < (rvec.size()-1); j++)
	{
		// Project the point into the node's image coordinate system.
		//printf("val",(int) j);
	//	myfile<<points2[j]<<endl;
		for(size_t k=0;k<points2[j].size();k++)
		{
			if(indices[0][i]==indices[j][k])
			{
				myfile<<indices[j][k]<<" "<<indices[0][i]<<" ";
				Vector2d p1(points2[j][k].x, points2[j][k].y);
				sba0.addMonoProj(j+1, i, p1);
				break;
			}
		//	cout<<k<<endl;
		}
		// sba0.addMonoProj(1, (int)i, p2);

	}	
  }
  
  myfile<<endl;
//  printf("Added %d points, %d tracks\n", (int)sba0.tracks.size(), (int)sba0.tracks.size());

//  double error1 = calcCamProjCost(sba0, 0);
  
  double error2 = calcCamProjCost(sba0, 1);
  printf("Errors after sba initialization: %f \n", error2);

  //    sba.A.setZero(6, 6);
  sba0.nFixed = 1;
  sba0.printStats();
  // cout << "oldpoints.size() = " << sba0.oldpoints.size() << endl;
 // printf("sba pointer: %p\n", &sba0);
  sba0.doSBA(20,10e-5,SBA_DENSE_CHOLESKY);
 int nbad = sba0.removeBad(2.0);
 cout << endl << "Removed " << nbad << " projections > 2 pixels error" << endl;
  sba0.doSBA(20,10e-5,SBA_DENSE_CHOLESKY);
	cout<<"After SBA"<<endl;
  //        cout << endl << sba.nodes[1].trans.transpose().start(3) << endl;
  for(int i=1;i<tvec.size();i++)
  {
	  Eigen::Vector3d trans = sba0.nodes[i].trans.head(3);
	  cout<<tvec[i]<<endl;
	  printf("trans = %f %f %f\n", trans(0), trans(1), trans(2));
  // do the convertion manually as there are
	//  Mat *sbaTvec.ptr<Point3f>(0) = Point3f(trans(0), trans(1), trans(2));

	  Quaterniond q1;
	  q1 = sba0.nodes[i].qrot;
	  Matrix3d rot = q1.toRotationMatrix();
//	  cout<<rvec[i]<<endl;
	  cout<<rot<<endl;
   
	  for(int j = 0; j < 3; j++)
	  {
		  tvec[i].at<float>(j,0)=trans(j);
		  for(int k = 0; k < 3; k++)
		  {
			  rvec[i].at<float>(j,k) = rot(j,k);
		  }
	  }
//	  myfile<<rvec[i]<<endl;
//	  myfile<<tvec[i].t()<<endl;
//	  myfile<<"-----------------"<<endl;
	  
//	  sbaR = sbaR.t();
//	  sbaTvec = -sbaR*sbaTvec;
//	  cout<<sbaR<<sbaTvec<<endl;
	//  Rodrigues(sbaR, rvec);
  } //*/
  
  	
 for(size_t i = 0; i < points.size(); i++)
  {
    points[i] = Point3f(sba0.tracks[i].point(0), sba0.tracks[i].point(1), sba0.tracks[i].point(2));
#if defined(_DEBUG_CONSOLE)
    printf("%f %f %f\n", points[i].x, points[i].y, points[i].z);
#endif
  } 
  
}//


void Poseavg(vector<Mat>& TPoses,int numplanes,vector<float>& D)
{
	Mat Tavg = Mat::zeros((3*(TPoses.size())),3*(ceil((TPoses.size())/numplanes))+numplanes, CV_32F); 
	for(int i=0;i<(ceil((TPoses.size())/numplanes));i++)
	{
		for(int j=0;j<numplanes;j++)
		{
		//	cout<<3*(numplanes*i+j)<<"heree"<<endl;
			if((TPoses.size()) > (numplanes*i+j+1) )
			{
				Mat aux =Tavg.colRange(3*i,3*i+3).rowRange(3*(numplanes*i+j),3*(numplanes*i+j)+3);
				Mat tempeye=Mat::eye(3,3,CV_32F);
				tempeye.copyTo(aux);
		//		cout<<aux<<endl;
				Mat aux1 =Tavg.colRange(3*((TPoses.size())/numplanes)+j,3*((TPoses.size())/numplanes)+j+1).rowRange(3*(numplanes*i+j),3*(numplanes*i+j)+3);
			//	Mat tempeye=Mat::eye(3,3);
				TPoses[(i*numplanes)+j+1]=-TPoses[(i*numplanes)+j+1];
			//	myfile<<-TPoses[(i*numplanes)+j+1]<<endl;
				TPoses[(i*numplanes)+j+1].copyTo(aux1);
			//	cout<<aux1<<endl;
			}
		}
	}
//	myfile<<Tavg<<endl;
	Mat U,S,V;
	SVD::compute(Tavg,S,U,V);
	V=V.t();
//	myfile<<V<<endl;
//	myfile<<V.col((V.cols-1))<<endl;;
	TPoses.clear();
	Mat temp=Mat::zeros(3,1,CV_32F);
	TPoses.push_back(temp);
	for( int i=0;i < ((V.rows-numplanes)/3);i++)
	{
		Mat temp2=(Mat_<float>(3,1) << V.at<float>(3*i,(V.cols-1)),V.at<float>(3*i+1,(V.cols-1)),V.at<float>(3*i+2,(V.cols-1)));
		myfile<<"T"<<temp2<<endl;
		TPoses.push_back(temp2);
	}
	
	for(int i=numplanes;i > 0 ;i--)
	{
		D.push_back(V.at<float>((V.rows-i),(V.cols-1)));
		myfile<<"D"<<V.col(V.cols-1)<<endl;
		myfile<<D[0]<<D[1]<<endl;
	}
	cout<<"Poses"<<TPoses.size()<<endl;	 
			
}			
				
void Poses(Mat T, Mat R, Mat& CPose)
{
	//	R=R.t();
	//	cout<<R<<endl;
		for(int i=0;i<3;i++)
		{
		
			R.col(i).copyTo(CPose.col(i));
		}
	//	T=-(R*T);
	//	cout<<T<<endl;
		T.copyTo(CPose.col(3));
}

float Ransac_scaling(vector<Point3f> points3d,vector<Point3f> points3d_1, Mat points_scale)
{
	float s_1=0;
	float error_least=100000;
	int maxinliers=0;
	cout<<points3d.size()<<points3d_1.size()<<endl;
	for (int i=0;i<points3	d_1.size();i++)
	{

		int inliers=0;
		float s= abs((points3d[i].x/points3d_1[i].x));
		cout<<s<<" ";	
		if(s<10000 && s>0.00001)
		{
			
			
			float error=0;
			float inliers=0;
			points_scale=s*points_scale;
			Mat points2d = Cam1*points_scale;
			for (int j=0;j<points3d_1.size();j++)
			{
				float x=points3d[j].x-s*points3d_1[j].x;
				float y=points3d[j].y-s*points3d_1[j].y;
				float z=points3d[j].z-s*points3d_1[j].z;
				error = (x*x)+(y*y)+(z*z);
				
				
					
				if(error<0.5)
				{
					inliers+=1;
		//			myfile<<" error "<<error<<" "<<i<<" ";
					error_least=error;
				//	s_1=s;
				}//*/
			}
			cout<<" max inliers"<<inliers<<" ";
			if(inliers > maxinliers)
			{
				maxinliers=inliers;
				error_least=error;
				s_1=s;
//				cout<<" max inliers"<<maxinliers<<" ";
			}//*/
		}	
//		myfile<<endl;
	
		
		s=abs(points3d[i].y/points3d_1[i].y);
		cout<<s<<" ";	
	//	myfile<<s<<" ";
		if(s<10000 && s>0.00001)
		{
			
			
			float error=0;
			float inliers=0;
			for (int j=0;j<points3d_1.size();j++)
			{
				float x=points3d[j].x-s*points3d_1[j].x;
				float y=points3d[j].y-s*points3d_1[j].y;
				float z=points3d[j].z-s*points3d_1[j].z;
				error=(x*x)+(y*y)+(z*z);
		
					
				if(error<0.5)
				{
					inliers+=1;
		//			myfile<<" error "<<error<<" "<<i<<" ";
					error_least=error;
				//	s_1=s;
				}//*/
			}
			
			cout<<" max inliers"<<inliers<<" ";
			
			if(inliers > maxinliers)
			{
				maxinliers=inliers;
				error_least=error;
				s_1=s;
//				cout<<" max inliers"<<maxinliers<<" ";
			}//*/
		}	
//		myfile<<endl;
	
		
		s=abs(points3d[i].z/points3d_1[i].z);
		cout<<s<<" ";	
	//	myfile<<s<<" ";
		
		if(s<10000 && s>0.00001)
		{
			
			
			float error=0;
			float inliers=0;
			for (int j=0;j<points3d_1.size();j++)
			{
				float x=points3d[j].x-s*points3d_1[j].x;
				float y=points3d[j].y-s*points3d_1[j].y;
				float z=points3d[j].z-s*points3d_1[j].z;
				error=(x*x)+(y*y)+(z*z);
		
					
				if(error<0.5)
				{
					inliers+=1;
		//			myfile<<" error "<<error<<" "<<i<<" ";
					error_least=error;
				//	s_1=s;
				}//*/
			}
			
			cout<<" max inliers"<<inliers<<" ";
			
			if(inliers > maxinliers)
			{
				maxinliers=inliers;
				error_least=error;
				s_1=s;
//				cout<<" max inliers"<<maxinliers<<" ";
			}//*/
		}	
//		myfile<<endl;
	
			
			
		cout<<" max inliers"<<maxinliers<<" ";
		cout<<endl;
	}
	
/*	if (error_least==100000)//s_1=(1/s_1);
	{
		for (int i=0;i<points3d_1.size();i++)
		{

			int inliers=0;
			float s= abs((points3d[i].x/points3d_1[i].x));
			cout<<s<<" ";	
			float error=10000;
			if(s<10000 && s>0.00001)
			{
					
				for (int j=0;j<points3d_1.size();j++)
				{
					float x=points3d[j].x-s*points3d_1[j].x;
					float y=points3d[j].y-s*points3d_1[j].y;
					float z=points3d[j].z-s*points3d_1[j].z;
					float error_less=(x*x)+(y*y)+(z*z);
			
						
					if(error_less<error)
					{
						error=error_less;
						s_1=s;
					}
				}
			cout<<" error_final s"<<error<<" ";
		}	
//		myfile<<endl;
	
		
		s=abs(points3d[i].y/points3d_1[i].y);
		cout<<s<<" ";	
	//	myfile<<s<<" ";
		if(s<10000 && s>0.00001)
		{
			
			
			for (int j=0;j<points3d_1.size();j++)
			{
				float x=points3d[j].x-s*points3d_1[j].x;
				float y=points3d[j].y-s*points3d_1[j].y;
				float z=points3d[j].z-s*points3d_1[j].z;
				float error_less=(x*x)+(y*y)+(z*z);
		
					
				if(error_less<error)
				{
					error=error_less;
					s_1=s;
				}
				cout<<" error_final s"<<error<<" ";
			}
		}	
//		myfile<<endl;
		
		s=abs(points3d[i].z/points3d_1[i].z);
		cout<<s<<" ";	
	//	myfile<<s<<" ";
		
		if(s<10000 && s>0.00001)
		{
			
			
			for (int j=0;j<points3d_1.size();j++)
			{
				float x=points3d[j].x-s*points3d_1[j].x;
				float y=points3d[j].y-s*points3d_1[j].y;
				float z=points3d[j].z-s*points3d_1[j].z;
				float error_less=(x*x)+(y*y)+(z*z);
		
					
				if(error_less<error)
				{
					error=error_less;
					s_1=s;
				}
			}
							cout<<" error_final s"<<error<<" ";
			
		}	
        //		myfile<<endl;
        cout<<" max inliers"<<error_least;
		cout<<endl;
	}*/
		
	return s_1;	
}

void NormalFitting(vector<Point3f> points3d)			
{			
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
	//		return 0;
		  }

			cout<< "Model coefficients: " << coefficients->values[0] << " " 
											  << coefficients->values[1] << " "
											  << coefficients->values[2] << " " 
											  << coefficients->values[3] << std::endl<<"--------------"<<endl;;

		   cout << "Model inliers: " << inliers->indices.size () << std::endl; //*/

	/*	  for (size_t i = 0; i < inliers->indices.size (); ++i)
			myfile << inliers->indices[i] << "    " << cloud.points[inliers->indices[i]].x << " "
													   << cloud.points[inliers->indices[i]].y << " "
													   << cloud.points[inliers->indices[i]].z << std::endl;//*/



}

void NormalFitting_n(vector<Point3f> points3d,int num)			
{			
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
			pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
		    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
		   // Create the segmentation object
		    pcl::SACSegmentation<pcl::PointXYZ> seg;
		  // Optional
		//    seg.setOptimizeCoefficients (true);
		  // Mandatory
		    seg.setModelType (pcl::SACMODEL_PLANE);
		    seg.setMethodType (pcl::SAC_PROSAC);
		    seg.setDistanceThreshold (0.05);

		  seg.setInputCloud (cloud.makeShared ());
		  seg.segment (*inliers, *coefficients);

		  if (inliers->indices.size () == 0)
		  {
			PCL_ERROR ("Could not estimate a planar model for the given dataset.");
	//		return 0;
		  }

			cout<< "Model coefficients: " << coefficients->values[0] << " " 
											  << coefficients->values[1] << " "
											  << coefficients->values[2] << " " 
											  << coefficients->values[3] << std::endl<<"--------------"<<endl;;

		   cout << "Model inliers: " << inliers->indices.size () << std::endl; //*/

	/*	  for (size_t i = 0; i < inliers->indices.size (); ++i)
			myfile << inliers->indices[i] << "    " << cloud.points[inliers->indices[i]].x << " "
													   << cloud.points[inliers->indices[i]].y << " "
													   << cloud.points[inliers->indices[i]].z << std::endl;//*/
  			pcl::PointCloud<pcl::PointXYZ> cloud1;
  			cloud1.width  = inliers->indices.size ();
			cloud1.height = 1;
			cloud1.points.resize (cloud1.width * cloud1.height);
		  for (size_t i = 0; i < inliers->indices.size (); ++i)
			{
				cloud1.points[i].x = cloud.points[inliers->indices[i]].x;
				cloud1.points[i].y = cloud.points[inliers->indices[i]].y;
				cloud1.points[i].z = cloud.points[inliers->indices[i]].z;
				
			}
			std::ostringstream oss;
            oss << "plane_" << num<<".pcd";
            cout<<oss.str()<<endl;
            string x=oss.str();
		    pcl::io::savePCDFileASCII (x, cloud1);


}


	

int main(int argc, char** argv)
{
	IplImage *limg, *rimg, *img_col;
	struct dirent **limgs, **rimgs;
	int nlimgs,nrimgs=0;	
	RNG rng( 0xFFFFFFFF);
//	Mat D = (Mat_<double>(3,1) << 0,0,0);
	Mat fundamental_matrix;
		float s;
	vector<Point3f> points3d_full;
    s=1;
  int total_planes=0;
			
	

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
 	
	
	
	int z=2000;//(nlimgs)/16 ;
//	cout<<z<<endl;
//	return 0;
	ofstream myfile1;
	vector<Point2f> Fcorners,Fcorners1;
//	vector< vector <Point3f> > Points3D;
	Mat CPose;
	CPose.push_back(Ipose);
	myfile.open("matches.txt",ios_base::app);

	Mat T3d;
	
	
//	fstream //myfile;
//	ofstream //myfile1;
//	ofstream myfile;
	ofstream myfile2;
//
	myfile1.open("example.txt",ios_base::app);
	myfile2.open("example1.txt",ios_base::app);
	//myfile1.open("/home/itachi/Desktop/example6_o.txt",ios_base::app);
		
	//myfile3.open("/home/itachi/Desktop/example8_o.txt",ios_base::app);
	//myfile4.open("/home/itachi/Desktop/example9.txt",ios_base::app);
	
	//myfile1<<"--------------------------------------"<<endl;
	myfile<<"starting--------------------------------------"<<endl;
	myfile1<<"starting--------------------------------------"<<endl;
	
	myfile2<<"--------------------------------------"<<endl;
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

		
	
	Mat ZPose;
	int p=0;
	int num=300;//100;
//	cout<<Ipose.rowRange(0,2)<<endl;
	Mat C1=Cam1*Ipose.rowRange(0,3);
	Mat C2;

//	cout<<C1<<endl;
	vector<Mat> TPoses;
	vector<Mat> RPoses;
	vector<Mat> NPoses;
	Mat img3,img6;
	
	RPoses.push_back(Ipose.colRange(0,3));
	TPoses.push_back(Ipose.col(3));
	
	Mat Rtemp=RPoses[0];
	Mat Ttemp=TPoses[0];
	
	cout<<RPoses[0]<<endl;
	cout<<TPoses[0]<<endl;
	vector<float> all;
	vector< vector<Point2f> > totcorners,totcorners1,final_all_plane;
	vector< vector<Point2f> > Totcorners,Totcorners1;
	vector< vector<Point3f> > plane_wise;//(1000, 1000);
	vector< vector<int> > totindices,final_all_plane_indices;
	vector< vector<int> > Totindices;
	vector < vector<KeyPoint> > plantempkeycorners;
	vector<Point3f>  pointspnp3d_1;//=Mat(3,pnp_points_1.size(),CV_64F);
	vector<int> scale_indices;
	vector<int> scale_indices_1;

	vector < vector<int> > plantempindices;
	vector<Mat> homographies;
	vector<Point3f> Points3d;	
	vector<float> D;
	vector<Mat> totPoses;	
	Mat R_1=Mat(3,3,CV_64F);
	Mat R_2=Mat(3,3,CV_64F);
	Mat R_3=Mat(3,3,CV_64F);
	Mat R_4=Mat(3,3,CV_64F);
	Mat t_1=Mat(3,1,CV_64F);
	Mat t_2=Mat(3,1,CV_64F);
	Mat t_3=Mat(3,1,CV_64F);
	Mat t_4=Mat(3,1,CV_64F);
	Mat t_11=Mat(3,1,CV_64F);
	Mat t_12=Mat(3,1,CV_64F);
	vector<Mat> ToTPoses;	
	vector<Point2f> plane_correspondence;	 // aalll
	int	c=0;
	for(int y=0;y<z;y=y+5)
	{ 		
		
	    vector<Point2f> corners,corners1;
	    vector<KeyPoint> keycorners,keycorners1;
		vector<Point2f> inliers1,inliers2,outliers1,outliers2;	
		Mat img1,img2,opoints3d,img4,img5;

		cout<<(y/5)<<endl;
		fn<<argv[1]<<"/"<<limgs[((y)+count)+num]->d_name;
		fl<<argv[1]<<"/"<<limgs[((y)+count)+num]->d_name;
		printf("fn=%s\n", fn.str().c_str());
		limg = cvLoadImage(fn.str().c_str(), CV_LOAD_IMAGE_GRAYSCALE);
		
	
		if(!limg)
		{
		  printf("image %s was not loaded, hence exiting.\n", limgs[count]->d_name);
		  break;  
		}
		//printf("Number of left images and right images is not equal:\n");
		fn.str("");
		if(argc==4)
		  fn<<argv[1]<<"/"<<rimgs[((y)+count+5)+num]->d_name;
		else
		  fn<<argv[2]<<"/"<<rimgs[((y)+count+5)+num]->d_name;
		printf("fn=%s\n", fn.str().c_str());
		rimg =cvLoadImage(fn.str().c_str(), CV_LOAD_IMAGE_GRAYSCALE);
		img_col=cvLoadImage(fn.str().c_str());
		if(!rimg)
		{
		  printf("image %s was not loaded, hence exiting.\n", rimgs[count]->d_name);
		  break;
		}
		fn.str("");


		img1=limg;
		img2=rimg;
		img5=img_col;
		vector<int> indices;
		img6= img1+img2; 
		
		
		
		start:
		corners.clear();
		corners1.clear();
		indices.clear();
		keycorners.clear();
		keycorners1.clear();
			
		
		
	//	vector<KeyPoint> plankeycorners;
	//	vector< vector < Mat > > planR;
	//	vector< vector < Mat > > planT;
		
		
		
		Detection(img1,img2,corners,corners1,keycorners,keycorners1,indices,myfile);
		
	/*	for(int i=0;i<corners.size();i++)
		{
			line(img5,corners[i],corners1[i], Scalar(0,0,255), 1);
		}		
		imshow("matches3", img5);
		waitKey(500);*/
		vector<Point2f>  pnp_points;
		if(c==0)
		{
	     
            cout<<"ssssssssssssssssssssss   "<<s<<endl;
			int numplanes;
			cout<<"Enter the no of planes "<<endl;
			cin>>numplanes;
			vector<KeyPoint> tempkeycorners;
			vector<int> tempindices;
			if(Points3d.size() != 0)
			{
			    s=1.000001;			
			    
			 }      
			Mat R = Mat::eye(3, 3, CV_32F);
			Mat T = Mat::zeros(3, 1, CV_32F);
			vector<Point2f> plane_correspondence_temp;
			for(int i=0;i<numplanes;i++)
			{ 
				if(Points3d.size() == 0)
				{
					total_planes=0;
				}
				Corners.clear();
		//		Corners=corners;
				img3=img1;
				KeyCorners.clear();
		//		KeyCorners=keycorners;
				cout<<"in loop"<<i<<endl;
				namedWindow("Image");
				setMouseCallback("Image", Mousehandler, 0 );
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
		//		corners1.clear();
				cout<<tempkeycorners.size()<<endl;
				int num=0;
				for(int j=0;j<keycorners.size();j++)
				{
					if ( keycorners[j].pt.x > Corners[0].x && keycorners[j].pt.x < Corners[1].x && keycorners[j].pt.y > Corners[0].y && keycorners[j].pt.y < Corners[2].y )  
					{
						tempindices.push_back(j);
						tempkeycorners.push_back(keycorners[j]);
					}
				}			
//				cout<<"endl;";
				Point2f pt;
				for(int size=0;size<tempkeycorners.size();size++)
				{
					if(final_all_plane.size() != 0)
					{
						for(int plane_check=0;plane_check < final_all_plane.size();plane_check++)
						{
							for(int ji=0;ji<final_all_plane[plane_check].size();ji++)
							{
								if(final_all_plane[plane_check][ji].x==tempkeycorners[size].pt.x && final_all_plane[plane_check][ji].y==tempkeycorners[size].pt.y)
								{
									 num++;
									 scale_indices.push_back(tempindices[size]);
									 scale_indices_1.push_back(final_all_plane_indices[plane_check][ji]);
						//			 cout<<	scale_indices[scale_indices.size()-1]<<endl;
						//			 cout<<"next"	<<scale_indices_1[scale_indices_1.size()-1]<<endl;	
									 pt.x=i;
									 pt.y=plane_check;		 
								}
							}
						}
					}			 
			    }
				
				if(num>5)
				{								 
					plane_correspondence_temp.push_back(pt);
					cout<<" plane checking "<<plane_correspondence_temp.size()<<endl;
				}
		        
		        Corners.clear();
		//		corners.size();
		//		Corners=corners;
				plantempkeycorners.push_back(tempkeycorners);
				plantempindices.push_back(tempindices);
				cout<<"plane inliers"<<tempkeycorners.size()<<endl;
				cout<<"plane inliers2323"<<tempindices.size()<<endl;
				tempkeycorners.clear();
				tempindices.clear();
			}
	
			if(scale_indices_1.size()!=0)
			{
			    vector<int> pnp_indices_all;		
				pnp_indices_all=Totindices[0];
				pointspnp3d_1.clear();
                for(int i=0; i < scale_indices_1.size();i++)
			    {
					int num=scale_indices_1[i];
					for(int j=0;j<pnp_indices_all.size();j++)
					{
						if(num==pnp_indices_all[j])
						{
							num=j;
							break;
						}
					}
					
					Point3f pt_1;
					pt_1.x=Points3d[num].x;
					pt_1.y=Points3d[num].y;
					pt_1.z=Points3d[num].z;
					pointspnp3d_1.push_back(pt_1);
				}
			}
		
			scale_indices_1.clear();
			Points3d.clear();
			Totindices.clear();
			Totcorners1.clear();   
			Totcorners.clear();
			RPoses.clear();
			TPoses.clear();   
			vector<Point2f> plane_correspondence_temp_1;
	
			if(plane_correspondence.size()==0)
			{
				plane_correspondence=plane_correspondence_temp;
				plane_correspondence_temp.clear();
			}
			else
			{
				plane_correspondence_temp_1=plane_correspondence;
				plane_correspondence.clear();
				for(int hi=0;hi<numplanes;hi++)
				{
					Point2f pt_1;
					if(plane_correspondence_temp[hi].x==hi)
					{
						pt_1.x=plane_correspondence_temp[hi].y;
						pt_1.y=plane_correspondence_temp_1[hi].y;
						cout<<final_all_plane.size()<<endl;
						cout<<numplanes<<endl;
						cout<<plane_correspondence_temp.size()<<endl;
					}
					else
					{
						if(final_all_plane.size()<=numplanes && plane_correspondence_temp.size()<numplanes)
						{
							cout<<"Added a plane"<<total_planes<<endl;
							total_planes=total_planes+1;	
							pt_1.x=hi;							
							pt_1.y=total_planes;
												
						}
					}
					plane_correspondence.push_back(pt_1);
				}
			}
			
			plane_correspondence_temp.clear();
			plane_correspondence_temp_1.clear();
			final_all_plane.clear();
			final_all_plane_indices.clear();

			RPoses.push_back(R);
			TPoses.push_back(T);
	//		myfile<<"RPoses"<<RPoses[0]<<endl;
			Corners	=corners;	
			totindices.clear();
			totcorners.clear();
			totcorners1.clear();
			KeyCorners=keycorners;
			
		}
		
		
		corners.clear();
		corners1.clear();
		indices.clear();
		keycorners=KeyCorners;
		Detection(img3,img2,corners,corners1,keycorners,keycorners1,indices,myfile);//*/
		
	
	//	Corners=corners;
	//	cout<<keycorners[0].class_id<<"object id"<<endl;
		float size1=KeyCorners.size()*1.0;
		float size2=keycorners.size()*1.0;
		float ratio1=(size2/size1);
		
		cout<<"Ratio"<<ratio1<<size2<<endl;
		  
		Mat H,R,T,R1,T1,N,N1;
		vector<KeyPoint> tempkeycorners1;
		tempkeycorners1=keycorners1;
		cout<<plantempkeycorners.size()<<endl;

		if((ratio1 > 0.2 && keycorners1.size() > 30) || (c==0) )
		{
			
			vector<Point2f>wholeimagecorners;
			vector<Point2f>wholeimagecorners1;
			vector<int>wholeimageindices;
			
			for(int i=0;i< plantempkeycorners.size();i++)
			{
							
				cout<<"in larger loop"<<i<<endl;
				keycorners=plantempkeycorners[i];
				if (keycorners.size() == 0)
				break;
							
				vector<Point2f>tempcorners;
				vector<Point2f>tempcorners1;
				vector<int>tempindices;
				
				cout<<plantempkeycorners[i].size()<<endl;
				
				for(int j=0;j<indices.size();j++)
				{
					for(int k=0;k<plantempkeycorners[i].size();k++)
					{
						
						if(indices[j] == plantempindices[i][k])
						{
							
							tempcorners.push_back(corners[j]);
							tempcorners1.push_back(corners1[j]);
							tempindices.push_back(indices[j]);
							wholeimagecorners.push_back(corners[j]);
							wholeimagecorners1.push_back(corners1[j]);
							wholeimageindices.push_back(indices[j]);
	//						myfile<<indices[j]<<" "<<plantempindices[i][k]<<" ";
							break;
						}
												
					}
				}
	
							
				cout<<"after detection"<<corners.size()<<"  "<<corners1.size()<<endl;					
	
				
				for(int j=0;j<tempcorners.size();j++)
				{
					line(img5,tempcorners[j],tempcorners1[j], Scalar(0,0,255), 1);
				}		
				imshow("matches3", img5);
				waitKey(500);//*/
				int tempx=0;
			
				if(tempcorners.size() > 10)
				{
				
					double reprojectionError=4;
					cout<<"after detection LOOP"<<endl;	
					Mat H1=findHomography(Mat(tempcorners),Mat(tempcorners1),CV_RANSAC,4);
				
					homographies.push_back(H1);				
					planarSFM(Cam1, tempcorners, tempcorners1, inliers1, inliers2, H1, R, T,  R1, T1, N, N1, reprojectionError, img5);

					myfile1<<R<<T<<N<<endl;
					myfile1<<R1<<T1<<N1<<endl;
					
					
					
					chooseDecomposition(R,T,N,R1,T1,N1);
					
					myfile1<<R<<endl<<T.t()<<endl; 
					myfile1<<c<<"--------------------"<<endl;
					myfile1<<"plane"<<N<<endl;
					if(plantempkeycorners[i].size() > tempx)
					{
						tempx=plantempkeycorners[i].size();
						if(i==0)
						{
							RPoses.push_back(R);
						}
						else
						{
							RPoses.pop_back();
							RPoses.push_back(R);
						}
					}			
					cout<<RPoses.size()<<"R"<<endl;
			//		cout<<RPoses.size()<<"val R"<<endl;
					TPoses.push_back(T);
					totcorners.push_back(tempcorners);
					totcorners1.push_back(tempcorners1);
					totindices.push_back(tempindices);
				
					
			
				}			
				else
				{
					for(int j=0;j<i;j++)
					{
						TPoses.pop_back();
						totcorners.pop_back();
						totcorners1.pop_back();
						totindices.pop_back();
					}
					if(i!=0)
					RPoses.pop_back();
					goto sfm;	
				}							
				keycorners1=tempkeycorners1;
			}
			Totcorners.push_back(wholeimagecorners);
			Totcorners1.push_back(wholeimagecorners1);
			Totindices.push_back(wholeimageindices);
				  
	//		double error =SFMwithSBA( Cam1,corners,corners1,R,T,4,points3d);			
			c+=1;			
			
		}
		else
		{
	//		myfile <<"-------------------------"<<endl;
	
	//		cout<<"heere "<<RPoses.size()<<endl;
			
			sfm:
			if(plantempkeycorners.size() >1)
			{
				Poseavg(TPoses,plantempkeycorners.size(),D);
			}
			else
			{
				D.push_back(1);
			}
			
			cout<<"scaling"<<endl;
			if(pointspnp3d_1.size()!=0)
			{
				Mat tempA_1=(Mat_<float>(1,4) << 0,0,0,1);
				Mat tempB_1=(Mat_<float>(1,4) << 0,0,0,1);
				Poses(TPoses[1],RPoses[1],CPose);// scale the TPoses[i]
				Mat CPose1;
				vconcat(CPose,tempA_1,CPose1);
				Mat CPose2;
				CPose2=ToTPoses[ToTPoses.size()-1];
			//	vconcat(ToTPoses[ToTPoses.size()-1],tempB_1,CPose2);
				Mat C2=C1*CPose1;

				Mat points3d_scale = triangulate(Totcorners[0],Totcorners1[0],C1,C2);
			//	points3d_scale=points3d_scale.t();
				Mat tempcpose=CPose2.colRange(0,3);
				Mat points3d_scale1=(tempcpose.t()*points3d_scale);
			//	cout<<points3d_scale<<endl;
				
				
				Mat tempd = (tempcpose.t()*CPose.col(3));
		//		points3d_scale.col(m)=points3d_scale.col(m)+tempd;
				
				
				vector<Point3f> to_scale;
				
				
				
				cout<<"plane "<<plane_correspondence<<" "<<pointspnp3d_1.size()<<" "<<scale_indices.size()<<endl;
				
				myfile<<"scaling"<<s<<endl;
				for(int scale_3d=0;scale_3d<scale_indices.size();scale_3d++)
				{
					
					//to_scale.push_back(points3d_scale[scale_indices[scale_3d]]);
					pointspnp3d_1[scale_3d].x = pointspnp3d_1[scale_3d].x+tempd.at<float>(0,0);
					pointspnp3d_1[scale_3d].y = pointspnp3d_1[scale_3d].y+tempd.at<float>(0,1);
					pointspnp3d_1[scale_3d].z = pointspnp3d_1[scale_3d].z+tempd.at<float>(0,2);
				}
				myfile<<endl;

				for(int scale_3d=0;scale_3d<scale_indices.size();scale_3d++)
				{
					Point3f tempointd;
					tempointd.x=points3d_scale1.at<float>(0,scale_indices[scale_3d]);
					tempointd.y=points3d_scale1.at<float>(1,scale_indices[scale_3d]);
					tempointd.z=points3d_scale1.at<float>(2,scale_indices[scale_3d]);
					
					to_scale.push_back(tempointd);
			//		myfile<<points3d_scale[scale_indices[scale_3d]].x<<" ";
			//		myfile<<points3d_scale[scale_indices[scale_3d]].y<<" ";
			//		myfile<<points3d_scale[scale_indices[scale_3d]].z<<", ";
				}//*/
								
				s=Ransac_scaling(to_scale,pointspnp3d_1,points3d_scale);
				cout<<"sssssssssssssssssssssssssssss        "<<s<<endl;
				Mat tempfinal=s*tempcpose.t()*points3d_scale;
				for(int scale_3d=0;scale_3d<tempfinal.cols;scale_3d++)
				{
					Mat tempde=tempfinal.col(scale_3d)-tempd;
					myfile<<tempde.t()<<" ";
				}
				

				myfile<<endl;

				scale_indices.clear();
		
			}	
				
		//	s=1;
			
			for(int i=0;i<TPoses.size();i++)
			{
			//	myfile2<<TPoses[i]<<endl<<RPoses[i]<<endl;
				cout<<"TPoses"<<TPoses.size()<<s<<endl;
				TPoses[i]=s*TPoses[i];
				RPoses[i]=RPoses[i];
				Poses(TPoses[i],RPoses[i],CPose);// scale the TPoses[i]
				myfile2<<CPose<<endl;
				Mat tempA=(Mat_<float>(1,4) << 0,0,0,1);
				Mat CPose3;
			//	cout<<" hasgflhwqegfdkljgsahfkljweilugrflakjfg"<<C1<<endl;
				vconcat(CPose,tempA,CPose3);
				if (ToTPoses.size()!=0)
				{
				//	cout<<ToTPoses.size()<<endl;
			//		cout<<ToTPoses[ToTPoses.size()-1]<<endl;
					Mat tempB=(Mat_<float>(1,4) << 0,0,0,1);
					Mat CPose4;
					vconcat(ToTPoses[ToTPoses.size()-1],tempB,CPose4);
					Mat C3=C1*CPose4*CPose3;
				//	myfile<<C2<<"C2             asfdf"<<endl;
					totPoses.push_back(C3);
				}
				else
				{
					Mat C3=C1*CPose3;
					totPoses.push_back(C3);
				}
			}
		
		//	cout<<"hererer"<<endl;
		//	myfile<<Tavg<<endl;
			vector<Point3f> totpoints3d;
	//		myfile<< V<<endl;
		
			int Size1=0;
			
			for(int i=0;i<plantempkeycorners.size();i++)
			{
				Size1= Size1+totcorners[i].size();
			}
				
			
			
			cout<<"indices"<<totindices.size()<<"3dpoints"<<Size1<<endl;
			
			Mat tpoints3d=Mat(Size1,3,CV_32F);
			final_all_plane.clear();
			final_all_plane_indices.clear();

			for(int i=0;i<plantempkeycorners.size();i++)
			{
				vector< vector<Point2f> > tempcorners;
				vector< vector<Point2f> > tempcorners1;
				vector< vector<int> > tempindices;
				
				tempcorners.push_back(totcorners[i]);

				for( int j=0;j<(RPoses.size()-1);j++)
				{
					tempcorners1.push_back(totcorners1[plantempkeycorners.size()*j+i]);
					tempindices.push_back(totindices[plantempkeycorners.size()*j+i]);
					if(j==(RPoses.size()-2)) 
					{
						final_all_plane.push_back(totcorners1[plantempkeycorners.size()*j+i]);
						final_all_plane_indices.push_back(totindices[plantempkeycorners.size()*j+i]);
					}
				}
		//	    cout<<"hereterterrer"<<endl;
				vector<Point3f> points3d = triangulate_n(tempcorners,tempcorners1,totPoses,tempindices);
				myfile<<"3dpoints"<<points3d<<endl;
				float num_of_plane;
				if(plane_correspondence.size()==0)
				{
					num_of_plane=i;
				}
				else
				{
					for(int li=0;li<plane_correspondence.size();li++)
					{
						if(plane_correspondence[li].x==i)
						{
							num_of_plane=plane_correspondence[li].y;
							break;
						}
					}
				}
			    cout<<"hereterterrer"<<endl;
                if(plane_wise.size()<num_of_plane+1)
                {
					plane_wise.push_back(vector<Point3f>());
			    }
			    
				for(int ki=0;ki<points3d.size();ki++)
				{
			//		cout<<plane_correspondence;
			//		cout<<num_of_plane;
					plane_wise[num_of_plane].push_back(points3d[ki]);
			    }
			    
				NormalFitting(points3d);
				for(int j=0;j<points3d.size();j++)
				{
					Points3d.push_back(points3d[j]);
		
				}//*/
			}
			
			for(int kj=0;kj<plane_wise.size();kj++)
			{
				for(int ki=0;ki<plane_wise[kj].size();ki++)
				{
					myfile<<plane_wise[kj][ki]<<" ";
                }
			    myfile<<endl;
			 }
		//	cout<<"poinsafadf"<<final_all_plane<<endl;
		//	cout<<"Rows"<<tpoints3d.rows<<endl;
			
			for(int ki=0;ki<plane_wise.size();ki++)
			{
				NormalFitting_n(plane_wise[ki],ki);
			}
		
			
	//		double error12=SFMwithSBA(Cam1,Totcorners[0],Totcorners1[0],RPoses[1],TPoses[1],4,Points3d);
			cout<<"Points size"<<Points3d.size()<<endl;
		//	myfile<<Points3d<<endl<<Totcorners.size()<<Totcorners1.size()<<endl;
			sba_new(Cam1,RPoses,TPoses,Points3d,Totcorners,Totcorners1,Totindices);
			myfile<<Points3d<<endl<<"size"<<Totcorners.size()<<Totcorners1.size()<<endl;
			vector<Point3f> points3d;
			for(int i=0;i<Points3d.size();i++)
			{
				Point3f tempoint3d;
				tempoint3d.x=Points3d[i].x;			
				tempoint3d.y=Points3d[i].y;			
				tempoint3d.z=Points3d[i].z;	
				points3d.push_back(tempoint3d);
			    points3d_full.push_back(tempoint3d);//dinesh

			}		
			
			
				
			pcl::PointCloud<pcl::PointXYZ> cloud;//dinesh
			cloud.width = points3d_full.size();//dinesh
			cloud.height = 1;//dinesh
			cloud.points.resize (cloud.width * cloud.height);//dinesh
			int jk=0;//dinesh
			for (size_t i = 0; i < points3d_full.size(); ++i)//dinesh
			{//dinesh
                jk++;//dinesh
				cloud.points[i].x = points3d_full[jk].x;//dinesh
				cloud.points[i].y = points3d_full[jk].y;//dinesh
				cloud.points[i].z = points3d_full[jk].z;//dinesh
				
			}//dinesh
			pcl::io::savePCDFileASCII ("test_pcd.pcd", cloud);//dinesh
			
			cout<<"here after sba "<<ToTPoses.size()<<endl;
			
			if(ToTPoses.size()==0)
			{
				cout<<"here after sba "<<endl;
				for( int i=0;i<RPoses.size();i++)
				{
					Mat CPose2=Mat(3,4,CV_32F);
					Poses(TPoses[i],RPoses[i],CPose2);
					myfile<<CPose2<<endl;
					ToTPoses.push_back(CPose2);
				}
			}
			else
			{
				
				int val=(ToTPoses.size()-1);
				for(int i=0;i<RPoses.size();i++)
				{
					cout<<"here after sba "<<endl;
					Mat CPose2=Mat(3,4,CV_32F);
					Poses(TPoses[i],RPoses[i],CPose2);
					Mat tempA=(Mat_<float>(1,4) << 0,0,0,1);
					Mat CPose1;
					vconcat(CPose2,tempA,CPose1);
					Mat C2=ToTPoses[val]*CPose1;
					myfile<<C2<<endl;
					ToTPoses.push_back(C2);
				}
			}
		
			myfile<<endl<<endl<<endl<<endl;
			plantempkeycorners.clear();
			plantempindices.clear();
		//	RPoses.clear();
		//	TPoses.clear();
			totcorners.clear();
			totcorners1.clear();
			totindices.clear();
			totPoses.clear();
			pointspnp3d_1.clear();
		//	Totcorners.clear();
		//	Totcorners1.clear();
		//	Totindices.clear();
		//	Points3d.clear();
		//	myfile1<<"Points"<<points3d<<endl;
			c=0;
			goto start;
		
		}


		
		
	//	RPoses.push_back(R);
	//	TPoses.push_back(T);
	//	totindices.push_back(indices);
	

		 cout<<2323<<endl;
	//return 0;	
	}

	return 0;	
	}
//		return 0;
    
//}			
			/* */
			
			
