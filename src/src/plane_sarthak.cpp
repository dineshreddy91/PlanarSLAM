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
    #include "posest/pnp_ransac.h"
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
	//#include <pcl/io/ply_io.h>

	#include <pcl/point_types.h>
	#include <pcl/sample_consensus/method_types.h>
	#include <pcl/sample_consensus/model_types.h>
	#include <pcl/segmentation/sac_segmentation.h>
	#include <sba/sba.h>
    #include <pcl/visualization/pcl_visualizer.h>
    #include <boost/thread/thread.hpp>
    #include <vikit/homography.h>
    #include <vikit/math_utils.h>
    #include <sophus/se3.h>
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
    using namespace Sophus;

	//template<typename T>

	const double uc = 319.5;
	const double vc = 239.5;
	const double fu = 525.0;
	const double fv = 525.0;
	//Mat F = (Mat_<double>(3,3) << 0, 0, 0, 0, 0, -1, 0, 1, 0);
	//Mat C1 = (Mat_<float>(3,4) << 718.856,0.000,607.192,0.000,0.000,718.856,185.215,0.000,0.000, 0.000,1.000,0.000);//389.956, 0, 254.903, 0, 0, 389.956, 201.899, 0, 0, 0, 1,0);
	//Mat C2 = (Mat_<float>(3,4) << 718.856,0.000,607.192,-386.144,0.000,718.856,185.215,0.000,0.000, 0.000,1.000,0.000);//389.956, 0, 254.903, 0, 0, 389.956, 201.899, 0, 0, 0, 1,0);
//	Mat Cam1 = (Mat_<float>(3,3) << 913.1033,0,354.726,0,907.36,257.24,0,0,1);//525.0, 0.0, 319.5,0, 525.0, 239.5, 0, 0 ,1.0);///1856.9,0,0,0,1856.9,0,0,0,1);//////389.956, 0, 254.903, 0, 0, 389.956, 201.899, 0, 0, 0, 1,0);
	Mat Cam1 = (Mat_<float>(3,3) << 525.0, 0.0, 319.5,0, 525.0, 239.5, 0, 0 ,1.0);
    Mat Cam2 = (Mat_<double>(3,3) << 525.0, 0.0, 319.5,0, 525.0, 239.5, 0, 0 ,1.0);
;//525.0, 0.0, 319.5,0, 525.0, 239.5, 0, 0 ,1.0);///1856.9,0,0,0,1856.9,0,0,0,1);//////389.956, 0, 254.903, 0, 0, 389.956, 201.899, 0, 0, 0, 1,0);
	//Mat Cam1=(Mat_<double>(3,3) << 389.956, 0, 254.903, 0, 389.956, 201.899, 0, 0, 1);//, 1,0)
	//Mat C2 = (Mat_<float>(3,4) << 389.956, 0, 254.903, 46.194,0, 389.956, 201.899, 0, 0, 0, 1,0);
	Mat Ipose=(Mat_<float>(3,4) << 1, 0, 0, 0, 0, 1, 0, 0, 0,0,1,0);//,0,0,0,1);
	const char *lreg, *rreg;
	int count1=0;
	//typedef Eigen::SparseMatrix<double> SpMat; // declares a column-major sparse matrix type of double
	//typedef Eigen::Triplet<double> Tm;

	//Mat points3d;
	vector<Point2f> Corners,Corners1;
	vector<Point2f> KeyCorners,KeyCorners1,KeyCorners_1;
	Mat TPoses_old;
    vector< vector <Point3f> > tot_points3d;
	ofstream myfile;

	//ofstream myfile3;
	//m/yfile3.open("/home/prateek/Desktop/example8_o.txt",ios_base::app);

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

	void Detection(Mat img1,Mat img2,vector<Point2f>& corners,vector<Point2f>& corners1,vector<Point2f>& keycorners,vector<Point2f>& keycorners1,vector<Point2f>& KeyCorners_1,vector<int>& indices,vector<int>& indices_old,ofstream &myfile)
	{
			
			int minHessian = 300;
			SurfFeatureDetector detector( minHessian );
			vector<KeyPoint> keypoints_12, keypoints_21;
			detector.detect( img1, keypoints_12 );
			detector.detect( img2, keypoints_21 );//*/
			vector<Point2f>  keypoints_1,keypoints_2,keypoints1,keypoints2;
			cv::KeyPoint::convert(keypoints_12,keypoints_1);
			vector<Point2f> keypoints_all=keypoints_1;
		    vector<Point2f>  check_points;
			if(keycorners.size()!=0)
			{
				cout<<"in loop"<<endl;
				keypoints_1=keycorners;
				keypoints_2=keycorners1;
				cout<<keycorners.size()<<keycorners1.size()<<endl;
			}
			else
			{
				KeyCorners_1=keypoints_1;
				cout<<"corners  "<<KeyCorners_1.size()<<KeyCorners_1.size()<<endl;
			}
			vector<Point2f> corners_tracking_3=Tracking(img1,img2,keypoints_1);//dinesh				
		//	Mat H1=findHomography(Mat(keypoints_1),Mat(corners_tracking_3),CV_RANSAC,4);
			check_points=Tracking(img2,img1,corners_tracking_3);//dinesh				
			float reprojectionerror=20;
			vector<Point2f> inliers,inliers1,outliers,outliers1;
			for(int a=0;a<corners_tracking_3.size();a++)
			{
				if(ceil(check_points[a].x)==ceil(keypoints_1[a].x) && ceil(check_points[a].y)==ceil(keypoints_1[a].y))//&& ceil(check_points[a].x)>=keypoints_1[a].x-reprojectionerror && check_points[a].y<=keypoints_1[a].y-reprojectionerror && check_points[a].y>=keypoints_1[a].y+reprojectionerror)
				{
					//cout<<"1"<<endl;
					inliers.push_back(keypoints_1[a]);
					inliers1.push_back(corners_tracking_3[a]);
					}
					else
					{
						//cout<<check_points[a].x<<" 2 "<<keypoints_1[a].x<<endl;
						outliers.push_back(keypoints_1[a]);
					    outliers1.push_back(corners_tracking_3[a]);
					}
				}
			//float error=computeHomographyInliers_new(keypoints_1,corners_tracking_3 ,H1,inliers,inliers1,outliers,outliers1,reprojectionerror);
		//	cout<<"homography"<<inliers.size()<<inliers1.size()<<endl;
		//	inliers=keypoints_1;
		//	inliers1=corners_tracking_3;
					if(indices_old[0] ==10000)
									{
			inliers=	keypoints_1;
			inliers1=	corners_tracking_3;
								}
			keycorners.clear();
			corners.clear();
			corners1.clear();
			keycorners1.clear();
			int x=0;
			for(int i=0;i<inliers.size();i++)
			{
				for (int j=0;j<keypoints_1.size();j++)
				{
					if( inliers[i].x==keypoints_1[j].x && inliers[i].y==keypoints_1[j].y )
					{
						corners.push_back(inliers[i]);
						corners1.push_back(inliers1[i]);
						keycorners.push_back(inliers[i]);
						keycorners1.push_back(inliers1[i]);
						if(indices_old[0] ==10000)
						{
							indices.push_back(j);
						}
						else
						{
						//	cout<<" hiii"<<endl;
							indices.push_back(indices_old[j]);
						}
						break;
												//cout<<" hiii"<<endl;

					//	matches2.erase(matches2.begin()+j);
					//	j=j-1;
					}
				}
			}
					 //for(int i=0;i<corners.size();i++)
					//{
					//line(img1,corners[i],corners1[i], Scalar(0,255,0), 1);
					//}		
					//imshow("matches3", img1);
					//waitKey(500);//*/
			cout<<corners.size()<<corners1.size()<<"shgdjaskghjhAGSDH"<<endl;
		//	return keycorners1;
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

   void compute_normals(vector < vector<Mat> > homographies_all,vector<Mat> TPoses,vector<Mat> Rposes,Mat C1,vector < vector<Mat> >&  Normals)
{
	cout<<"hi"<<homographies_all.size()<<endl;
	vector<Mat> homographies=homographies_all[homographies_all.size()-1];
	for(int i=0;i<homographies.size();i++)
	{
		cout<<"normals_done"<<endl;
		Mat KHKR=homographies[i];//(C1.inv());//*homographies[i]);//*C1);//-Rposes[Rposes.size()-1];
		Mat T=TPoses[TPoses.size()-1];		
		float n1=KHKR.at<float>(0,0)/T.at<float>(0,0);
		float n2=KHKR.at<float>(0,1)/T.at<float>(0,0);
		float n3=KHKR.at<float>(0,2)/T.at<float>(0,0);
		double n_all=(n1*n1)+(n2*n2)+(n3*n3);
		n_all=sqrt(n_all);
		n1=n1/n_all;
		n2=n2/n_all;
		n3=n3/n_all;		
		myfile<<"normal<<"<<n1<<" "<<n2<<" "<<n3<<endl;
		}
	}
	Mat triangulate(vector<Point2f> corners,vector<Point2f> corners1, Mat C1 , Mat C2)
	{
			cout<<corners.size()<<" "<<endl;
			cout<<corners1.size()<<" "<<endl;
			
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
				
			//			cout<<i<<" "<<corners[i].x<<" ";


					A.push_back<float>((corners[i].x*C1.row(2)-C1.row(0))/weight1);
					A.push_back<float>((corners[i].y*C1.row(2)-C1.row(1))/weight1);
					A.push_back<float>((corners1[i].x*C2.row(2)-C2.row(0))/weight);
					A.push_back<float>((corners1[i].y*C2.row(2)-C2.row(1))/weight);
				//	cout<<"---------"<<A<<endl;
					
			//		myfile<<A<<endl;
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


	vector<Point3f> triangulate_n(vector< vector<Point2f> > corners,vector< vector<Point2f> > corners1, vector<Mat> totPoses,vector<vector<int> >& indices,int nloop)
	{
			
			
			cout<<"here in trinagualtion"<<endl;
			
			Mat points2d= Mat(2,corners.size(),CV_32FC1);
			Mat points2d1= Mat(2,corners.size(),CV_32FC1);
		//	for(int i =0;i<corners.size();i++)
		//	{
		//		points2d1.at<float>(0,i)=corners[i].x;
		//		points2d1.at<float>(1,i)=corners[i].y;
		//	}
	//		cout<<totPoses.size()-nloop<<" "<<(totPoses.size()-nloop)+1<<endl;
			myfile<<totPoses[totPoses.size()-nloop]<<endl<<totPoses[(totPoses.size()-nloop)+1]<<endl;
			
			Mat tempmat=(Cam1*totPoses[totPoses.size()-nloop]);//totPoses[totPoses.size()-nloop];
			Mat tempmat1=(Cam1*totPoses[(totPoses.size()-nloop)+1]);//totPoses[(totPoses.size()-nloop)+1];
			
		//	totPoses[0]=
		//	totPoses[1]=;
			
	//		cout<<totPoses[0]<<endl<<totPoses[1]<<endl;
			myfile<<"here in triangulation"<<endl;
			myfile<<corners[0]<<endl<<corners1[0]<<endl;
		
		
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
				
				
				A.push_back<float>((corners[0][i].x*tempmat.row(2)-tempmat.row(0))/weight1);
				A.push_back<float>((corners[0][i].y*tempmat.row(2)-tempmat.row(1))/weight1);
				
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
							A.push_back<float>((corners1[j][k].x*tempmat1.row(2)-tempmat1.row(0))/weight1);
							A.push_back<float>((corners1[j][k].y*tempmat1.row(2)-tempmat1.row(1))/weight1);
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
		
	void chooseDecomposition(Mat& R,Mat& T,Mat& N,Mat& R1,Mat& T1,Mat& N1)
	{
	/*	if(fabs(T1.at<float>(2,0)) > fabs(T.at<float>(2,0)))
		{
			if(T1.at<float>(2,0)<0)
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
			if(T.at<float>(2,0)<0)
			{
				T=-T;
				N=-N;
			} 
			
		}//*/
		
		if(fabs(T1.at<float>(0,0)) > fabs(T.at<float>(0,0)))
		{
	/*		if(T1.at<float>(0,0)<0)
			{
				T1=-T1;
				N1=-N1;
			}*/
			
		
			R=R1;
			T=T1;
			N=N1;
			
		}
		else
		{		
	/*		if(T.at<float>(0,0)<0)
			{
				T=-T;
				N=-N;
			} */
			
		}//*/
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

	void Poses(Mat T, Mat R, Mat& CPose)
	{
		//	R=R.t();
		CPose=(Mat_<float>(3,4) << 1, 0, 0, 0, 0, 1, 0, 0, 0,0,1,0);;
						//cout<<CPose<<endl;

			for(int i=0;i<3;i++)
			{
				R.col(i).copyTo(CPose.col(i));
			}
		//	T=-(R*T);
			cout<<T<<endl;
			T.copyTo(CPose.col(3));
	}
	
	void Poses_opp(Mat& T, Mat& R, Mat CPose)
{
	//	R=R.t();
	//	cout<<R<<endl;
		for(int i=0;i<3;i++)
		{
		
			CPose.col(i).copyTo(R.col(i));
		}
	//	T=-(R*T);
	//	cout<<T<<endl;
		CPose.col(3).copyTo(T);
}

	
    void NormalFitting(vector< vector<Point3f> > points3d)
	{			
                int totsize=0;
                for (int j=0; j < points3d.size();j++)
                {
                    totsize+=points3d[j].size();
                }


                pcl::PointCloud<pcl::PointXYZ> cloud;
                cloud.width  = totsize;
				cloud.height = 1;
				cloud.points.resize (cloud.width * cloud.height);
                int k=0;
                for (int j=0; j < points3d.size();j++)
                {
                    for (size_t i = 0; i < points3d[j].size(); ++i)
                    {
                        cloud.points[k+i].x = points3d[j][i].x;
                        cloud.points[k+i].y = points3d[j][i].y;
                        cloud.points[k+i].z = points3d[j][i].z;
                    }
                    k=k+points3d[j].size();
                }

        /*		pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
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

         /*      pcl::PointCloud<pcl::PointXYZ> cloud1;
               cloud1.width  = inliers->indices.size ();
               cloud1.height = 1;
               cloud1.points.resize (cloud1.width * cloud1.height);
               for (size_t i = 0; i < inliers->indices.size (); ++i)
               {
                   cloud1.points[i].x = cloud.points[inliers->indices[i]].x;
                   cloud1.points[i].y = cloud.points[inliers->indices[i]].y;
                   cloud1.points[i].z = cloud.points[inliers->indices[i]].z;

               }*/
               std::ostringstream oss;
               oss << "test_new.pcd";
               cout<<oss.str()<<endl;
               string x=oss.str();
               pcl::io::savePCDFileASCII (x, cloud);





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

    vector<Point3f> Pointfitting(vector<Point2f> points2d, Mat N)
    {

   //     cout<<"in point fitting"<<endl;
        Mat coord_new= Mat(points2d.size(),3,CV_32FC1);
      //  Mat coord1= Mat(corners.size(),3,CV_64FC1);
        for(int i=0;i<points2d.size();i++)
        {
          coord_new.at<float>(i,0)=points2d[i].x;
          coord_new.at<float>(i,1)=points2d[i].y;
          coord_new.at<float>(i,2)=1;
       //   coord1.at<double>(i,0)=corners1[i].x;
       //   coord1.at<double>(i,1)=corners1[i].y;
      //    coord1.at<double>(i,2)=1;

        }
        vector<Point3f> points_plane3d;
    //    cout<<"in point fitting"<<endl;
        Mat points3d_new = Mat(4,points2d.size(),CV_32FC1);
        for(int i=0;i<points2d.size();i++)
        {
   //         cout<<Cam1.inv()<<coord_new.row(i)<<endl;
            Point3f point3d;
            Mat new_inv = coord_new.row(i);
            Mat ray= Cam1.inv()*new_inv.t();
          //  Mat ray = points3d_new.col(i);
   //         cout<<ray<<N<<"3d points"<<endl;
            points3d_new.at<float>(3,i)=N.dot(ray);
            points3d_new.at<float>(2,i)=ray.at<float>(2,0);
            points3d_new.at<float>(1,i)=ray.at<float>(1,0);
            points3d_new.at<float>(0,i)=ray.at<float>(0,0);
            point3d.x = (points3d_new.at<float>(0,i)/points3d_new.at<float>(3,i));
            point3d.y = (points3d_new.at<float>(1,i)/points3d_new.at<float>(3,i));
            point3d.z = (points3d_new.at<float>(2,i)/points3d_new.at<float>(3,i));
            points_plane3d.push_back(point3d);

    //        cout<<points3d_new.col(i)<<"3d points"<<endl;

        }


        tot_points3d.push_back(points_plane3d);
        NormalFitting(tot_points3d);
        NormalFitting_n(points_plane3d,1);
        return points_plane3d;
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
		vector<Point2f> Fcorners,Fcorners1,plane_1_7;
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
        myfile1.open("example6.txt",ios_base::app);
        myfile2.open("example7.txt",ios_base::app);
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
		int num=0;//100;
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
		vector< vector<int> > Totindices,Totindices_1;
		vector < vector<Point2f> > plantempkeycorners;
		vector<Point3f>  pointspnp3d_1;//=Mat(3,pnp_points_1.size(),CV_64F);
		vector<int> scale_indices;
		vector<int> scale_indices_1;

		vector < vector<int> > plantempindices;
		vector < vector<Mat> > homographies;
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
		vector<Mat> H3;
		vector<Mat> ToTPoses;	
		vector<Point2f> plane_correspondence,plane_correspondence_old;	 // aalll
        vector<Mat>Tot_planes;
		int	c=0;Mat C3;
		int num_of_plane_tpose=0;
        int plane_change=0;
        int num_of_loops=0;
		for(int y=0;y<z;y=y+1)
		{ 		
			plane_correspondence_old=plane_correspondence;
			vector<Point2f> corners,corners1;
			vector<Point2f> keycorners,keycorners1;
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
			fn<<argv[1]<<"/"<<rimgs[((y)+count+1)+num]->d_name;
			else
			fn<<argv[2]<<"/"<<rimgs[((y)+count+1)+num]->d_name;
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
		//	vector<Point2f> plankeycorners;
		//	vector< vector < Mat > > planR;
		//	vector< vector < Mat > > planT;
		/*	for(int i=0;i<corners.size();i++)
			{
				line(img5,corners[i],corners1[i], Scalar(0,0,255), 1);
			}		
			imshow("matches3", img5);
			waitKey(500);*/
			vector<Point2f>  pnp_points;
			if(c==0)
			{   num_of_loops=num_of_loops+1;
				vector<int> tempindices;
				tempindices.push_back(10000);
				Detection(img1,img2,corners,corners1,keycorners,keycorners1,KeyCorners_1,indices,tempindices,myfile);
				tempindices.clear();
				cout<<"ssssssssssssssssssssss   "<<s<<endl;
				int numplanes;
				cout<<"Enter the no of planes "<<endl;
				cin>>numplanes;
			//	numplanes=2;
				vector<Point2f> tempkeycorners;
				Mat R,T;
				
				if(Points3d.size() != 0)
				{
					R=RPoses[RPoses.size()-1];	
					T=TPoses[TPoses.size()-1];	
//					s=1.000001;			
				 }      
				else
				{
					R= Mat::eye(3, 3, CV_32F);
					T = Mat::zeros(3, 1, CV_32F);
				}	
				R= Mat::eye(3, 3, CV_32F);
				T = Mat::zeros(3, 1, CV_32F);
				C3=Mat::eye(3, 4, CV_32F);
				
				myfile<<"RPoses"<<R<<T<<endl;
				vector<Point2f> plane_correspondence_temp,mean_all;
				for(int a=0;a<final_all_plane.size();a++)
				{
				float totalX = 0, totalY = 0;
				Point mean;
				for(int b=0;b<final_all_plane[a].size();b++)
				{
				totalX += final_all_plane[a][b].x;
				totalY += final_all_plane[a][b].y;				
				}
				mean.x = totalX / final_all_plane[a].size();
				mean.y = totalY / final_all_plane[a].size();
				mean_all.push_back(mean);
			    }
				int	points_num=0;
				for(int i=0;i<numplanes;i++)
				{ 
					
					if(Points3d.size() == 0)
					{
						Mat tem=Mat::eye(3, 3, CV_32F);
                        Mat temn=Mat::zeros(3, 1, CV_32F);
                        if(num_of_loops==1)
						{
						H3.push_back(tem);
					    }
						total_planes=numplanes;
                        Tot_planes.push_back(temn);
					}
					if(i==0)
					{
						H3.clear();
						}
					Mat tem=Mat::eye(3, 3, CV_32F);
			        H3.push_back(tem);

					Corners.clear();
			//		Corners=corners;
					img3=img1;
					KeyCorners.clear();
					KeyCorners_1.clear();
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
				 /*   if(i==0 && num_of_loops==1)
				    {
				    Corners.push_back(Point(15,221));
				    Corners.push_back(Point(448,231));
				    Corners.push_back(Point(455,428));
					}
					if(i==1 && num_of_loops==1)
				    {
				    Corners.push_back(Point(495,268));
				    Corners.push_back(Point(629,265));
				    Corners.push_back(Point(630,430));
					}
					if(i==0 && num_of_loops==2)
				    {
				    Corners.push_back(Point(8,227));
				    Corners.push_back(Point(434,230));
				    Corners.push_back(Point(435, 418));
					}
					if(i==1 && num_of_loops==2)
				    {
				    Corners.push_back(Point(464, 283));
				    Corners.push_back(Point(634, 273));
				    Corners.push_back(Point(636, 435));
					}
                    if(i==0 && num_of_loops==3)
				    {
				    Corners.push_back(Point(58, 225));
				    Corners.push_back(Point(395, 232));
				    Corners.push_back(Point(401, 412));
					}
					if(i==1 && num_of_loops==3)
				    {
				    Corners.push_back(Point(428, 278));
				    Corners.push_back(Point(636, 267));
				    Corners.push_back(Point(635, 454));
					}
					if(i==0 && num_of_loops==4)
				    {
				    Corners.push_back(Point(11, 241));
				    Corners.push_back(Point(350, 237));
				    Corners.push_back(Point(356, 405));
					}
					if(i==1 && num_of_loops==4)
				    {
				    Corners.push_back(Point(379, 281));
				    Corners.push_back(Point(635, 266));
				    Corners.push_back(Point(635, 459));
					}
					if(i==0 && num_of_loops==5)
				    {
				    Corners.push_back(Point(12, 241));
				    Corners.push_back(Point(309, 239));
				    Corners.push_back(Point(313, 412));
					}
					if(i==1 && num_of_loops==5)
				    {
				    Corners.push_back(Point(332, 276));
				    Corners.push_back(Point(632, 269));
				    Corners.push_back(Point(636, 469));
					}
					if(num_of_loops>5)
				    {
					for(; ;){
								imshow("Image",img3);
								char d = (char)waitKey(10);
									if(d==27){
									break;
									}
							 }
						 }*/
					cout<<Corners<<endl;
			//		corners1.clear();
					cout<<tempkeycorners.size()<<endl;
					int num=0;
					Point2f pt;
			
					for(int j=0;j<keycorners.size();j++)
					{
						if ( keycorners[j].x > Corners[0].x && keycorners[j].x < Corners[1].x && keycorners[j].y > Corners[0].y && keycorners[j].y < Corners[2].y )  
						{
							tempindices.push_back(indices[j]);
							tempkeycorners.push_back(keycorners[j]);
						}
					}	
			
					if(tempindices.size()>points_num)
					{
						cout<<"dominant plane "<<i<<" "<<points_num<<endl;
						points_num=tempindices.size();
						num_of_plane_tpose=i;
					}
				
					for(int j=0;j<mean_all.size();j++)
					{
						if ( mean_all[j].x > Corners[0].x && mean_all[j].x < Corners[1].x && mean_all[j].y > Corners[0].y && mean_all[j].y < Corners[2].y )  
						{
							             pt.x=i;
										 pt.y=j;
										 num++;
						}
					}			
	//				cout<<"endl;";
					for(int size=0;size<tempkeycorners.size();size++)
					{
						if(final_all_plane.size() != 0)
						{
							for(int plane_check=0;plane_check<final_all_plane.size();plane_check++)
									{
								for(int ji=0;ji<final_all_plane[plane_check].size();ji++)
								{
									//cout<<final_all_plane[plane_check][ji].x<<"h"<<tempkeycorners[size].x<<endl;
									if(ceil(final_all_plane[plane_check][ji].x)<ceil(tempkeycorners[size].x)+2 && ceil(final_all_plane[plane_check][ji].x)>ceil(tempkeycorners[size].x)-2 && ceil(final_all_plane[plane_check][ji].y)<ceil(tempkeycorners[size].y)+2 && ceil(final_all_plane[plane_check][ji].y)>ceil(tempkeycorners[size].y)-2)
									{
										 //num++;
										// cout<<"h"<<endl;
										 scale_indices.push_back(tempindices[size]);
										 scale_indices_1.push_back(final_all_plane_indices[plane_check][ji]);
										 plane_1_7.push_back(final_all_plane[0][ji]);
										// pt.x=i;
										// pt.y=plane_check;		 
									}
								}
							}
						}			 
					}
					
					if(num==1)
					{								 
						plane_correspondence_temp.push_back(pt);
						cout<<" plane checking "<<plane_correspondence_temp.size()<<endl;
					}
					num=0;
					Corners.clear();
			//		corners.size();
			//		Corners=corners;
					plantempkeycorners.push_back(tempkeycorners);
					plantempindices.push_back(tempindices);
					cout<<"plane inliers"<<tempkeycorners.size()<<endl;
					cout<<"plane inliers2323"<<scale_indices_1.size()<<endl;
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
				RPoses.push_back(R);
				TPoses.push_back(T);   
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
                    //        Tot_planes[hi]=
						}
						else
						{
							if(final_all_plane.size()<=numplanes && plane_correspondence_temp.size()<numplanes)
							{
								cout<<"Added a plane"<<total_planes<<endl;
								pt_1.x=hi;							
								pt_1.y=total_planes;
								total_planes=total_planes+1;				
							}
						}
						plane_correspondence.push_back(pt_1);
					}
				}
				
				plane_correspondence_temp.clear();
				plane_correspondence_temp_1.clear();
				final_all_plane.clear();
				final_all_plane_indices.clear();
				

				myfile<<"RPoses"<<TPoses[0]<<endl;
				Corners	=corners;	
				totindices.clear();
				totcorners.clear();
				totcorners1.clear();
				KeyCorners=keycorners;
				KeyCorners_1=KeyCorners;
				Totindices_1.clear();
				Totindices_1.push_back(indices);
			}
			if(plane_correspondence.size()==0)
			{}else
			{
			num_of_plane_tpose=plane_correspondence[num_of_plane_tpose].y;
            }
            corners.clear();
			corners1.clear();
			indices.clear();
			keycorners=KeyCorners;
			cout<<" hmmm"<<KeyCorners_1.size()<<" "<<KeyCorners.size()<<endl;
			
			Detection(img1,img2,corners,corners1,keycorners,keycorners1,KeyCorners_1,indices,Totindices_1[Totindices_1.size()-1],myfile);//*/
			Totindices_1.push_back(indices);
			if(KeyCorners.size()==0)
			{
				KeyCorners_1=keycorners1;
			}
				
			KeyCorners=keycorners1;
			float size1=KeyCorners_1.size()*1.0;
			float size2=KeyCorners.size()*1.0;
			float ratio1=(size2/size1);
			cout<<KeyCorners_1.size()<<endl;
			  
		
			vector<Point2f> tempkeycorners1;
			tempkeycorners1=keycorners1;
            cout<<"ratio"<<ratio1<<plantempkeycorners.size()<<endl;

            if( c <5 ) // (ratio1 > 0.3 && keycorners1.size() > 30) || (c==0) )
			{
				
				vector<Point2f>wholeimagecorners;
				vector<Point2f>wholeimagecorners1;
				vector<int>wholeimageindices;
				vector <Mat> homography_plane;
				for(int i=0;i< plantempkeycorners.size();i++)
				{
								
						Mat H,R,T,R1,T1,N,N1;
					keycorners=plantempkeycorners[i];
					if (keycorners.size() == 0)
					break;
								
					vector<Point2f>tempcorners;
					vector<Point2f>tempcorners1;
					vector<int>tempindices;
					
					cout<<plantempkeycorners[i].size()<<endl;
					
					for(int j=0;j<indices.size();j++)
					{
						myfile<<indices[j]<<" ";
						for(int k=0;k<plantempkeycorners[i].size();k++)
						{
							
							if(plantempindices[i][k] == indices[j])
							{
								tempcorners.push_back(corners[j]);
								tempcorners1.push_back(corners1[j]);
								tempindices.push_back(indices[j]);
								wholeimagecorners.push_back(corners[j]);
								wholeimagecorners1.push_back(corners1[j]);
								wholeimageindices.push_back(indices[j]);
								break;
							}
													
						}
					}
		
		
					for(int j=0;j<tempcorners.size();j++)
					{
						line(img5,tempcorners[j],tempcorners1[j], Scalar(0,0,255), 1);
					}		
					imshow("matches3", img5);
					waitKey(5000);//*/
					int tempx=0;
				
					if(tempcorners.size() > 10)
					{
                      //  if(num_of_loops ==1)
                    //    {
                            cout<<"plane_core"<<plane_correspondence<<endl;
                            double reprojectionError=4;
                            cout<<"after detection LOOP"<<endl;
                            Mat H1;
                            if(plane_correspondence.size()!=0 )
                            {
                                cout<<"here in plane"<<homographies.size()<<endl;
                                Mat H2=findHomography(Mat(tempcorners),Mat(tempcorners1),CV_RANSAC,4);
                                H2.convertTo(H2, CV_32F);
                                H1= H2 ;//*H3[i];
                                H1=(H1/H1.at<float>(2,2));
                                myfile<<H1.at<float>(2,2)<<endl;
                                H3[i]=H1;

                            }
                            else
                            {
                                Mat H2=findHomography(Mat(tempcorners),Mat(tempcorners1),CV_RANSAC,4);
                                cout<<H3[i]<<endl;
                                H2.convertTo(H2, CV_32F);
                                H1= H2; //*H3[i];
                                H1=(H1/H1.at<float>(2,2));
                                H3[i]=H1;
                            }

                            planarSFM(Cam1, tempcorners, tempcorners1, inliers1, inliers2, H1, R, T,  R1, T1, N, N1, reprojectionError, img5);
                         //   Pointfitting(tempcorners)

                            vector<Vector2d, aligned_allocator<Vector2d> > uv_ref(tempcorners.size());
                            vector<Vector2d, aligned_allocator<Vector2d> > uv_cur(tempcorners1.size());
                            for(size_t i=0, i_max=tempcorners.size(); i<i_max; ++i)
                            {
                              Vector2d cur_corners,cur_corners1;
                              cur_corners[0]=tempcorners[i].x;
                              cur_corners[1]=tempcorners[i].y;
                              cur_corners1[0]=tempcorners1[i].x;
                              cur_corners1[1]=tempcorners1[i].y;
                              uv_ref[i] = cur_corners;
                              uv_cur[i] = cur_corners1;
                            }
                            vk::Homography Homography(uv_ref,uv_cur,fu,reprojectionError);
                            Homography.computeSE3fromMatches();

                            vector<vk::HomographyDecomposition> decomposition_new=Homography.decompositions;
                            cout<<decomposition_new.size()<<"Decompostion new sixe"<<endl;
                            myfile2<<decomposition_new[0].R<<endl;
                            myfile2<<decomposition_new[0].n<<endl;
                            myfile2<<decomposition_new[0].d<<endl;
                            myfile2<<decomposition_new[0].t<<endl;

                            SE3 T_cur_from_ref = Homography.T_c2_from_c1;
                            myfile1<<T_cur_from_ref<<"Newone"<<endl;

                            myfile1<<H1<<endl;
                            myfile1<<R<<T<<N<<endl;
                            myfile1<<R1<<T1<<N1<<endl;
                            chooseDecomposition(R,T,N,R1,T1,N1);
                            myfile1<<R<<T<<N<<endl;
                            myfile1<<c<<"--------------------"<<endl;
                            myfile1<<"plane"<<N<<endl;
              //              Tot_planes[i]=N;
             //            }


              //           else
              //          {

            //            N=RPoses[0]*N;
           //             d=1+N.dot(TPoses[0]);
           //             vector<Point3f> new3dpoints=Pointfitting(tempcorners,N,d);
           //             Mat distcoeff=Mat::zeros(8,1,CV_32F);

             //           solvePnPRansac(new3dpoints,tempcorners1,Cam1,distcoeff,R,T);
             //           myfile1<<R<<T<<endl;
         //               }

                        if(num_of_plane_tpose==i)
                        {
                            RPoses.push_back(R);
                            TPoses.push_back(T);
                            myfile1<<R<<endl<<T.t()<<endl;
                            myfile1<<"rrrrrrrrrrrrrrrrrrrrrrrrr"<<endl;
                        }


						cout<<RPoses.size()<<"R"<<endl;
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
				c+=1;			
				
			}
			else
			{
				sfm:
				homographies.push_back(H3);
				//homographies.clear();
			/*	if(plantempkeycorners.size() >1)
				{
					Poseavg(TPoses,plantempkeycorners.size(),D);
				}
				else
				{
					D.push_back(1);
				}*/
				//TPoses.pop_back();
				//TPoses.push_back(T);
				cout<<"Normals"<<endl;
				vector < vector<Mat> >  Normals;
				compute_normals(homographies,TPoses,RPoses,C1,Normals);
				cout<<"scaling"<<endl;
				cout<<plane_correspondence<<endl;
				if(plane_correspondence.size()!=0)
				{	

					if(plane_correspondence[num_of_plane_tpose].y==plane_change)
					{
						vector<Point2f> tempcorners;
						tempcorners=totcorners[num_of_plane_tpose];
						vector<Point2f> tempcorners1=totcorners1[num_of_plane_tpose];
						int old_plane_num_in_old=0;
						int new_plane_num_in_old=1;
						int old_plane_num_in_new=0;
						for(int loop=0;loop<homographies[homographies.size()-2].size();loop++)
						{
							cout<<homographies[homographies.size()-2][loop]<<" "<<plane_change<<endl;
							if(plane_correspondence_old[loop].y==plane_change)
							{
								old_plane_num_in_old=loop;
								break;
							}
							
						}
						for(int loop=0;loop<homographies[homographies.size()-2].size();loop++)
						{
							if(plane_correspondence[num_of_plane_tpose].y==plane_correspondence_old[loop].y)
							{
								new_plane_num_in_old=loop;
								break;
							}
						}
						cout<<old_plane_num_in_old<<" "<<new_plane_num_in_old<<endl;
						for(int loop1=0;loop1<homographies[homographies.size()-1].size();loop1++)
						{
							if(plane_change==plane_correspondence[loop1].y)
							{
								old_plane_num_in_new=loop1;
							}
						}
					
						Mat H_7_14_new=homographies[homographies.size()-1][num_of_plane_tpose];
						Mat H_7_14_old=homographies[homographies.size()-1][old_plane_num_in_new];
						Mat H_1_7_old=homographies[homographies.size()-2][old_plane_num_in_old];
						Mat H_1_7_new=homographies[homographies.size()-2][new_plane_num_in_old];
						Mat H_1_14_old=H_7_14_old*H_1_7_old;
						Mat H_1_14_new=H_7_14_new*H_1_7_new;
						Mat H,R_new,T_new,R1_new,T1_new,N_new,N1_new; 
						float reprojectionError=4;
						planarSFM(Cam1, tempcorners, tempcorners1, inliers1, inliers2, H_7_14_new, R_new, T_new,  R1_new, T1_new, N_new, N1_new, reprojectionError, img5);
						chooseDecomposition(R_new,T_new,N_new,R1_new, T1_new,N1_new);						
						//cout<<R_new<<T_new<<endl;
						Mat R_old,T_old,R1_old,T1_old,N_old,N1_old; 
						planarSFM(Cam1, tempcorners, tempcorners1, inliers1, inliers2, H_1_14_old, R_old, T_old,  R1_old, T1_old, N_old, N1_old, reprojectionError, img5);
						chooseDecomposition(R_old, T_old,N_old,R1_old, T1_old,N1_old);	
						Mat R_1,T_1,R1_1,T1_1,N_1,N1_1; 
						planarSFM(Cam1, tempcorners, tempcorners1, inliers1, inliers2, H_1_7_old, R_1, T_1,  R1_1, T1_1, N_1, N1_1, reprojectionError, img5);
						chooseDecomposition(R_1,T_1,N_1,R1_1, T1_1,N1_1);						
						cout<<H_1_14_old<<endl;
						cout<<H_1_7_old<<old_plane_num_in_old<<endl;
						T_old=T_old-T_1;
						cout<<"values"<<norm(T_old)<<norm(T_new)<<R_old<<T_old<<endl;
						s=s*norm(T_old)/norm(T_new);
					}
					else
					{
						vector<Point2f> tempcorners;
						tempcorners=totcorners[num_of_plane_tpose];
						vector<Point2f> tempcorners1=totcorners1[num_of_plane_tpose];
						int old_plane_num_in_old=0;
						int new_plane_num_in_old=1;
						int old_plane_num_in_new=0;
						for(int loop=0;loop<homographies[homographies.size()-2].size();loop++)
						{
							cout<<homographies[homographies.size()-2][loop]<<" "<<plane_change<<endl;
							if(plane_correspondence_old[loop].y==plane_change)
							{
								old_plane_num_in_old=loop;
								break;
							}
							
						}
						for(int loop=0;loop<homographies[homographies.size()-2].size();loop++)
						{
						if(plane_correspondence[num_of_plane_tpose].y==plane_correspondence_old[loop].y)
							{
								new_plane_num_in_old=loop;
								break;
							}
						}
						cout<<old_plane_num_in_old<<" "<<new_plane_num_in_old<<endl;
						for(int loop1=0;loop1<homographies[homographies.size()-1].size();loop1++)
						{
							if(plane_change==plane_correspondence[loop1].y)
							{
								old_plane_num_in_new=loop1;
							}
						}
						Mat H_7_14_new=homographies[homographies.size()-1][num_of_plane_tpose];
						Mat H_7_14_old=homographies[homographies.size()-1][old_plane_num_in_new];
						Mat H_1_7_old=homographies[homographies.size()-2][old_plane_num_in_old];
						Mat H_1_7_new=homographies[homographies.size()-2][new_plane_num_in_old];
						Mat H_1_14_old=H_7_14_old*H_1_7_old;
						Mat H_1_14_new=H_7_14_new*H_1_7_new;
						Mat H,R_new,T_new,R1_new,T1_new,N_new,N1_new; 
						float reprojectionError=4;
						planarSFM(Cam1, tempcorners, tempcorners1, inliers1, inliers2, H_7_14_new, R_new, T_new,  R1_new, T1_new, N_new, N1_new, reprojectionError, img5);
						chooseDecomposition(R_new,T_new,N_new,R1_new, T1_new,N1_new);						
						//cout<<R_new<<T_new<<endl;
						Mat R_old,T_old,R1_old,T1_old,N_old,N1_old; 
						planarSFM(Cam1, tempcorners, tempcorners1, inliers1, inliers2, H_1_14_old, R_old, T_old,  R1_old, T1_old, N_old, N1_old, reprojectionError, img5);
						chooseDecomposition(R_old, T_old,N_old,R1_old, T1_old,N1_old);	
						Mat R_1,T_1,R1_1,T1_1,N_1,N1_1; 
						planarSFM(Cam1, tempcorners, tempcorners1, inliers1, inliers2, H_1_7_old, R_1, T_1,  R1_1, T1_1, N_1, N1_1, reprojectionError, img5);
						chooseDecomposition(R_1,T_1,N_1,R1_1, T1_1,N1_1);						
						cout<<H_1_14_old<<endl;
						cout<<H_1_7_old<<old_plane_num_in_old<<endl;
						T_old=T_old-T_1;
						cout<<"values"<<norm(T_old)<<norm(T_new)<<T_new<<T_old<<endl;
						s=s*norm(T_old)/norm(T_new);
						plane_change=num_of_plane_tpose;
						
					}
				}
				
					
				cout<<"ssssssssssssssssssssssssssssssssssssssssss"<<s<<TPoses_old<<endl;
                TPoses_old=TPoses[TPoses.size()-1];
				cout<<"ssssssssssssssssssssssssssssssssssssssssss"<<s<<TPoses_old<<endl;

				for(int i=0;i<TPoses.size();i++)
				{
				//	myfile2<<TPoses[i]<<endl<<RPoses[i]<<endl;

				    
					myfile<<TPoses[i]<<RPoses[i]<<endl;
					myfile2<<CPose<<endl;
					TPoses[i]=s*TPoses[i];
					Poses(TPoses[i],RPoses[i],CPose);// scale the TPoses[i]
					cout<<"TPoses"<<TPoses.size()<<endl;
					myfile2<<CPose<<endl;
					Mat tempA=(Mat_<float>(1,4) << 0,0,0,1);
					Mat CPose3;
				//	cout<<" hasgflhwqegfdkljgsahfkljweilugrflakjfg"<<C1<<endl;
					vconcat(CPose,tempA,CPose3);
					
					Mat C3=C1*CPose3;
					myfile<<" totposes"<<C3<<endl;
					totPoses.push_back(C3);
		
				}
			
			//	cout<<"hererer"<<endl;
			//	myfile<<Tavg<<endl;
				vector<Point3f> totpoints3d;
		//		myfile<< V<<endl;
			
				int Size1=0;
				cout<<"indices"<<plantempkeycorners.size()<<"3dpoints"<<totcorners.size()<<endl;
				for(int i=0;i<plantempkeycorners.size();i++)
				{
					Size1= Size1+totcorners[i].size();
				}
					
				
				
				cout<<"indices"<<totindices.size()<<"3dpoints"<<Size1<<endl;
				
				Mat tpoints3d=Mat(Size1,3,CV_32F);
				final_all_plane.clear();
				final_all_plane_indices.clear();
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
						cout<<CPose1<<endl<<ToTPoses[val]<<endl;;
						Mat C2=ToTPoses[val]*CPose1;
						myfile<<C2<<endl;
						ToTPoses.push_back(C2);
					}
				}
				
				cout<<plantempkeycorners.size()<<endl;
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
					vector<Point3f> points3d = triangulate_n(tempcorners,tempcorners1,ToTPoses,tempindices,totPoses.size());
					myfile<<"3dpoints"<<points3d<<endl;
					float num_of_plane;
					cout<<"plane_core1"<<plane_correspondence<<endl;
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
					
					NormalFitting_n(points3d,y+i);
					
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
		//		sba_new(Cam1,RPoses,TPoses,Points3d,Totcorners,Totcorners1,Totindices);
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
				
				
