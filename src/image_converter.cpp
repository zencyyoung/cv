#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <sstream>

using namespace cv;
using namespace std;

#define UNKNOWN_FLOW_THRESH 1e9  

class ImageConverter
{
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Subscriber depth_sub_;
	//image_transport::Publisher image_pub_;
	std::vector<cv_bridge::CvImage > image_list_;
    	std::vector<cv_bridge::CvImage > depth_list_;
	cv::Mat prevGray, nextGray, flow, motion2color;
	int imageCount, clusterCount;

public:
	ImageConverter()
		:it_(nh_), imageCount(0), clusterCount(2)
	{
	        image_sub_ = it_.subscribe("/camera/data_throttled_image_relay", 1, &ImageConverter::imageCb, this);
 		depth_sub_ = it_.subscribe("/camera/data_throttled_image_depth_relay", 1, &ImageConverter::depthCb, this);

	}
	

	void makecolorwheel(vector<cv::Scalar> &colorwheel)  
	{  
		int RY = 15;  
		int YG = 6;  
		int GC = 4;  
		int CB = 11;  
		int BM = 13;  
		int MR = 6;  

		int i;  

		for (i = 0; i < RY; i++) colorwheel.push_back(Scalar(255,       255*i/RY,     0));  
		for (i = 0; i < YG; i++) colorwheel.push_back(Scalar(255-255*i/YG, 255,       0));  
		for (i = 0; i < GC; i++) colorwheel.push_back(Scalar(0,         255,      255*i/GC));  
		for (i = 0; i < CB; i++) colorwheel.push_back(Scalar(0,         255-255*i/CB, 255));  
		for (i = 0; i < BM; i++) colorwheel.push_back(Scalar(255*i/BM,      0,        255));  
		for (i = 0; i < MR; i++) colorwheel.push_back(Scalar(255,       0,        255-255*i/MR));  
	}  



	void motionToColor(Mat flow, Mat &color)  
	{  
		if (color.empty())  
			color.create(flow.rows, flow.cols, CV_8UC3);  

		static vector<Scalar> colorwheel; //Scalar r,g,b  
		if (colorwheel.empty())  
			makecolorwheel(colorwheel);  

		// determine motion range:  
		float maxrad = -1;  

		// Find max flow to normalize fx and fy  
		for (int i= 0; i < flow.rows; ++i)   
		{  
			for (int j = 0; j < flow.cols; ++j)   
			{  
				Vec2f flow_at_point = flow.at<Vec2f>(i, j);  
				float fx = flow_at_point[0];  
				float fy = flow_at_point[1];  
				if ((fabs(fx) >  UNKNOWN_FLOW_THRESH) || (fabs(fy) >  UNKNOWN_FLOW_THRESH))  
					continue;  
				float rad = sqrt(fx * fx + fy * fy);  
				maxrad = maxrad > rad ? maxrad : rad;  
			}  
		}  

		for (int i= 0; i < flow.rows; ++i)   
		{  
			for (int j = 0; j < flow.cols; ++j)   
			{  
				uchar *data = color.data + color.step[0] * i + color.step[1] * j;  
				Vec2f flow_at_point = flow.at<Vec2f>(i, j);  

				float fx = flow_at_point[0] / maxrad;  
				float fy = flow_at_point[1] / maxrad;  
				if ((fabs(fx) >  UNKNOWN_FLOW_THRESH) || (fabs(fy) >  UNKNOWN_FLOW_THRESH))  
				{  
					data[0] = data[1] = data[2] = 0;  
					continue;  
				}  
				float rad = sqrt(fx * fx + fy * fy);  

				float angle = atan2(-fy, -fx) / CV_PI;  
				float fk = (angle + 1.0) / 2.0 * (colorwheel.size()-1);  
				int k0 = (int)fk;  
				int k1 = (k0 + 1) % colorwheel.size();  
				float f = fk - k0;  
				//f = 0; // uncomment to see original color wheel  

				for (int b = 0; b < 3; b++)   
				{  
					float col0 = colorwheel[k0][b] / 255.0;  
					float col1 = colorwheel[k1][b] / 255.0;  
					float col = (1 - f) * col0 + f * col1;  
					if (rad <= 1)  
						col = 1 - rad * (1 - col); // increase saturation with radius  
					else  
						col *= .75; // out of range  
					data[2 - b] = (int)(255.0 * col);  
				}  
			}  
		}  
	}  


	void imageCb(const sensor_msgs::ImageConstPtr& msg){
		
		
		stringstream ss;
		string name = "/home/yud/imagefiles/rgb_image_";
		string type = ".jpg";

		ss << name << (imageCount) << type; 
		string filename = ss.str();
		ss.str("");

		cv_bridge::CvImagePtr cv_ptr;
	
		try{
			cv_ptr = cv_bridge::toCvCopy(msg);
		}
		catch (cv_bridge::Exception& e){
	
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}
	
		//image_list_.push_back(*cv_ptr);
		
		cv::Mat nextImg = cv_ptr->image;
		cvtColor(nextImg, nextGray, CV_BGR2GRAY);
		//cv::imwrite(filename, nextImg);

		if(prevGray.data){

			calcOpticalFlowFarneback(prevGray, nextGray, flow, 0.5, 3, 15, 3, 5, 1.2, 0);  
			motionToColor(flow, motion2color); 
			cv::namedWindow( "Flow window", 1);
			imshow("Flow window", motion2color); 
			//std::cout << "lalalala" << std::endl;

		}

		std::swap(prevGray, nextGray);

		cv::namedWindow( "Image window", CV_WINDOW_AUTOSIZE);
		cv::imshow( "Image window", cv_ptr->image);

		Mat samples(640*480, 1, CV_32FC2);
		Mat centers(clusterCount, 1, samples.type());
	       	Mat labels(640*480, 1, CV_32SC1);
		
//		Vec2f* samplesPoint = (Vec2f*)samples.data;
//		Vec2f* flowPoint = (Vec2f*)flow.data;
//		for (int i = 0; i < 640*480; i++){
//			*samplesPoint = *flowPoint;
//			samplesPoint++;
//			flowPoint++;
//		}

//		uchar* p;
		int i, j ,k = 0;
		for(i = 0; i < flow.rows; i++){
//
//			p = flow.ptr<uchar>(i);
			for(j = 0; j < flow.cols; j++){

				samples.at<Vec2f>(k,0) = flow.at<Vec2f>(i,j);
//				uchar pxl = flow.at<uchar>(i, j*2);
//				samples.at<Vec2f>(k,0)[1] = float(p[j * 2 + 1]);
				k++;
			}
		}
		//vector<int> compression_params;
		//compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
		//compression_params.push_back(9);
				
		cv::waitKey(10);
		imageCount++;
		
		kmeans(samples, clusterCount, labels, TermCriteria( CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 10, 1.0), 3, KMEANS_PP_CENTERS, centers);	
//		std::cout << labels << std::endl;
		
		Mat clusterImage(flow.rows, flow.cols, CV_8UC1);
		float step = 255/(clusterCount - 1);
		k = 0;
		for (k = 0; k < flow.rows*flow.cols; k++){
			int labelNum = labels.at<int>(k,0);
			int i = int(k/(flow.cols));
			int j = int(k%(flow.cols)); 
 			clusterImage.at<uchar>(i,j) = 255 - labelNum * step;
		}
		
		namedWindow("Clusters");
		if(clusterImage.data) imshow("Clusters", clusterImage);

		cout<< clusterImage<<endl;
	}
	
	void depthCb(const sensor_msgs::ImageConstPtr& msg){
		        
		cv_bridge::CvImagePtr cv_ptr;
		try{
			cv_ptr = cv_bridge::toCvCopy(msg);
		}						             
		catch (cv_bridge::Exception& e){
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}
		//depth_list_.push_back(*cv_ptr);
		//
		cv::namedWindow( "Depth window", CV_WINDOW_AUTOSIZE);
		cv::imshow( "Depth window", cv_ptr->image);

		cv::waitKey(10);

	}

};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "image_converter");
	ImageConverter ic;
	ros::spin();
	return 0;
}


