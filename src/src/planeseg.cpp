#include <stdio.h>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include "opencv2/video/tracking.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#include <math.h>
#include <fstream>
#include <sstream>
#include <dirent.h>
#include <fnmatch.h>
#include <iostream>
#include <cstdlib>
#include <time.h>



#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include "planarSFM.hpp"


using namespace cv;
using namespace std;
using namespace pe;

Mat Cam1 = (Mat_<float>(3,3) << 1389.187,0.0,672.605,0,1394.598,387.235, 0, 0 ,1.0);//525.0, 0.0, 319.5,0, 525.0, 239.5, 0, 0 ,1.0);//913.1033,0,354.726,0,907.36,257.24,0,0,1);//1856.9,0,0,0,1856.9,0,0,0,1);// ////389.956, 0, 254.903, 0, 0, 389.956, 201.899, 0, 0, 0, 1,0);
Mat Ipose=(Mat_<float>(3,4) << 1, 0, 0, 0, 0, 1, 0, 0, 0,0,1,0);//,0,0,0,1);

vector<Point2f> Corners,Corners1;


int compareFn ( const void *pa, const void *pb ) {
	const float *a = *(const float **)pa;
	const float *b = *(const float **)pb;
	if(a[2] > b[2])
		return 1;
	return 0;
}

void chooseDecomp( Mat& R ,Mat R1, Mat& T, Mat T1, Mat& N, Mat N1)  
{
	if(fabs(T1.at<float>(2,0)) >= fabs(T.at<float>(2,0)))
			{ 
				if(T1.at<float>(2,0)>0)
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
			   if(T.at<float>(2,0)>0)
			  {
				  T=-T;
				  N=-N;
			  }  
	       }	//*/
		
}

int main(int argc, char** argv)
{
	

  /**
   * Preparing a list of left and right images.
   * If left and right images are in the same folder then there will be three args: <path to images folder> <reg expr for left> <reg ecpr for right>
   * If lett and right images are in seperate folder then there will be two args: <path to left images folder> <path to right images folder>
  */


//	fstream //myfile;
//	ofstream //myfile1;
	ofstream myfile;
	ofstream myfile4;
	ofstream myfile3;
//
//	//myfile.open("/home/prateek/Desktop/example.txt",ios::in);
	//myfile1.open("/home/itachi/Desktop/example6_o.txt",ios_base::app);
	
	myfile.open("/home/prateek/plane_segmentation/kf_homog/data_save/data_harit.txt",ios::out);
	myfile3.open("/home/prateek/plane_segmentation/kf_homog/data_save/normal_harit.txt",ios::out);
	myfile4.open("/home/prateek/plane_segmentation/kf_homog/data_save/error_harit.txt",ios::out);
	
	
	
	//myfile1<<"--------------------------------------"<<endl;
//	myfile<<"--------------------------------------"<<endl;
//	myfile3<<"--------------------------------------"<<endl;
//	myfile4<<"--------------------------------------"<<endl;
	
		
	cout<<argv[1]<<endl;
	
	Mat img = imread(argv[1], CV_LOAD_IMAGE_COLOR);
//	imshow("a",img);
//	waitKey(0);
//	cout<<img<<endl;
	Mat img1=img;
	vector<Point2f> corners,corners1;
	Mat H,R,T,R1,T1, N,N1;
	double reprojectionError=4;
	vector<Point2f> inliers1,inliers2,outliers1,outliers2;
	vector< vector<Point2f> > totcorners,totcorners1;
	vector< vector<Point2f> > totcorners2,totcorners3;
	vector<Mat> homographies;
//	vector<Mat> localcorners;

	Mat img_hsv;
	
	cvtColor(img,img_hsv,CV_BGR2HSV);
	
	Mat img_blurn;
	bilateralFilter(img,img_blurn,17,100,60);	
	
	imwrite("saveimage.jpg",img_blurn);
//	imshow("matches",img_blurn);
//	waitKey(100);
		
	
	FILE *fp = fopen("/home/prateek/plane_segmentation/kf_homog/data_save/read.txt","r");

	int numHomographies = 0;

	fscanf(fp, "%d", &numHomographies);
	
	cout<<numHomographies<<endl;
	
	for(int kk = 0; kk < numHomographies; kk++)
	{ 

	//	cout<<"Starting "<<endl;
		double Hmat[9];
		
		for(int j = 0; j < 9; j++) 
		{ 
			fscanf(fp, "%lf", &Hmat[j]);
		}
		
		Mat H1 = (Mat_<double>(3,3) << Hmat[0], Hmat[1], Hmat[2], Hmat[3], Hmat[4], Hmat[5], Hmat[6], Hmat[7], Hmat[8]);
		
		cout<<H1<<endl;
		
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
			corners.push_back(Point2f(am1, am2));
			corners1.push_back(Point2f(an1, an2));
		}
		
		
	    H1 = findHomography(Mat(corners), Mat(corners1), CV_RANSAC, 4);
	    homographies.push_back(H1);
	    
		cout<<numCorresp<<endl;
		
		for(int k = 0; k < numCorresp; k++) 
		{
		
			float currX = corners[k].x;
			float currY = corners[k].y;
			float **distArr;
			distArr = (float **) malloc(sizeof(float *) * numCorresp);
			
			for(int kk = 0; kk < numCorresp; kk++)
			{ 
				distArr[kk] = (float *) malloc(sizeof(float) * 5);
				distArr[kk][0] = 0.0f;
				distArr[kk][1] = 0.0f;
				distArr[kk][2] = 0.0f;
				distArr[kk][3] = 0.0f;
				distArr[kk][4] = 0.0f;
			}

			vector< Point2f > corners_small, corners1_small;

			for(int kk = 0; kk < numCorresp; kk++) 
			{ 
				distArr[kk][0] = corners[kk].x;
				distArr[kk][1] = corners[kk].y;
				distArr[kk][3] = corners1[kk].x;
				distArr[kk][4] = corners1[kk].y;

				if(kk == k)
				{ 
					distArr[kk][2] = 100000;	
				} 
				else
				{
					distArr[kk][2] = sqrt(pow((currX - distArr[kk][0]),2) + pow((currY - distArr[kk][1]),2));
				}
			}

			
			qsort(distArr, numCorresp, sizeof(distArr[0]), compareFn);
	
			corners_small.clear();
			corners1_small.clear();

			for(int kk = 0; kk < 10; kk++)
			{ 
				corners_small.push_back(Point2f(distArr[kk][0], distArr[kk][1]));
				corners1_small.push_back(Point2f(distArr[kk][3], distArr[kk][4]));
			}
			
			totcorners.push_back(corners_small);
			totcorners1.push_back(corners1_small);		
			
			 Mat H_new = findHomography(Mat(corners_small), Mat(corners1_small), CV_RANSAC, 4);
				
			planarSFM(Cam1, corners_small, corners1_small, inliers1, inliers2, H_new, R, T,  R1, T1, N, N1, reprojectionError, img1);//*/
		   
	//	    cout<<T1<<T<<endl;
		    chooseDecomp(R,R1,T,T1,N,N1);
			
//			cout<<R<<T<<R1<<T1<<N<<N1<<endl;	
//			cout<<fabs(T1.at<float>(2,0))<<fabs(T.at<float>(2,0))<<endl;
	//		//myfile1<<R<<R1<<endl;
		
			//myfile3<<N<<N1<<endl;
		
		/*	myfile<<N<<N1<<"Normal-------------------"<<endl;
			myfile<<T<<T1<<"Translation-------------------"<<endl;
			myfile<<T<<T1<<"Rotation-------------------"<<endl;*/
	//		cout<<T1<<endl;
		
		
	//		cout<<R<<T<<N<<endl;
	
			int rpoint=0,gpoint=0,bpoint=0;
		
			for(int i=-2; i < 3 ;i++)
			{
				for(int j=-2; j< 3 ;j++ )
				{
					rpoint+=img_blurn.at<Vec3b>(ceil(corners[k].y-0.5)+i,ceil(corners[k].x-0.5)+j)[1];
					gpoint+=img_blurn.at<Vec3b>(ceil(corners[k].y-0.5)+i,ceil(corners[k].x-0.5)+j)[2];
					bpoint+=img_blurn.at<Vec3b>(ceil(corners[k].y-0.5)+i,ceil(corners[k].x-0.5)+j)[3];
				}	
			} 
			
			rpoint=rpoint/25;
			gpoint=gpoint/25;
			bpoint=bpoint/25;
			
		
		//	myfile<<corners[k].x<<" "<<corners[k].y<<" "<<rpoint<<" "<<gpoint<<" "<<bpoint<<" "<<N.at<float>(0,0)<<" "<<N.at<float>(1,0)<<" "<<N.at<float>(2,0)<<" "<<kk<<" "<<endl;//H1.at<float>(0,0)<<" "<<H1.at<float>(0,1)<<" "<<H1.at<float>(0,2)<<" "<<H1.at<float>(1,0)<<" "<<H1.at<float>(1,1)<<" "<<H1.at<float>(1,2)<<" "<<H1.at<float>(2,0)<<" "<<H1.at<float>(2,1)<<" "<<H1.at<float>(2,2)<<" "<<kk<<" "<<endl;
			myfile<<corners[k].x<<" "<<corners[k].y<<" "<<corners1[k].x<<" "<<corners1[k].y<<" "<<rpoint<<" "<<gpoint<<" "<<bpoint<<" "<<N.at<float>(0,0)<<" "<<N.at<float>(1,0)<<" "<<N.at<float>(2,0)<<" "<<kk<<" "<<endl;
			
		
		}
		
		
		planarSFM(Cam1, corners, corners1, inliers1, inliers2, H1, R, T,  R1, T1, N, N1, reprojectionError, img1);
	//	myfile3<<T<<endl<<T1<<endl<<N<<N1;
		chooseDecomp(R,R1,T,T1,N,N1);		
		myfile3<<N.at<float>(0,0)<<" "<<N.at<float>(1,0)<<" "<<N.at<float>(2,0)<<endl;
	//	myfile3<<N1.at<float>(0,0)<<" "<<N1.at<float>(1,0)<<" "<<N1.at<float>(2,0)<<"Second"<<endl;
		
			
	}
	
	
	for(int j=0; j < totcorners.size(); j++)
	{
		for(int i=0;i<numHomographies;i++)
		{
			float error=4;
		//	cout<<homographies[i]<<endl;
			float sum1=computeHomographyInliers(totcorners[j], totcorners1[j],homographies[i],inliers1,inliers2,outliers1,outliers2,error,myfile4);
	//		cout<<sum1<<endl;
			myfile4<<sum1<<" ";	
		}
		myfile4<<endl;		  
	}  //*/
		
		
//	cout<<2323<<endl;
	return 0;	
	
	}
//		return 0;
    
//}			
			/* */
			
			
