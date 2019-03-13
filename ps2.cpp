#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/video/tracking.hpp> 
#include <iostream>
#include <vector>

using namespace std;
using namespace cv;

int predictsize = 6; 
int measuredsize = 4;  
int control = 0;
 
int main()
{
	int i,j;
	Mat img;
	Mat predict(predictsize, 1, CV_32F);  // [x,y,vx,vy,w,h]
    Mat measured(measuredsize, 1, CV_32F);    // [x,y,w,h]
	KalmanFilter k(predictsize , measuredsize, control, CV_32F);
	
    for(i=0;i<24;i++)
    {
    	if(i==0 || i==7 || i==16|| i==23)
    		 k.measurementMatrix.at<float>(i)=1.0f;
    	else  k.measurementMatrix.at<float>(i)=0.0f;

    }
    setIdentity(k.transitionMatrix);
    setIdentity(k.measurementNoiseCov,Scalar(1e-1));
    setIdentity(k.processNoiseCov, Scalar(1e-2)); 
    k.processNoiseCov.at<float>(14) = 1.0f; 
    k.processNoiseCov.at<float>(21) = 1.0f;
 	VideoCapture cap("3.avi");
    cap >> img;
    VideoWriter video("output.avi",CV_FOURCC('M','J','P','G'),60, Size(img.cols,img.rows));     //saving the video during code    source- https://docs.opencv.org/3.1.0/dd/d43/tutorial_py_video_display.html
 	if (!cap.isOpened())
    {
        cout << "Can't open video\n";
        return -1;
    }
    cout << "\nHit esc to exit";
    char ch = 'a';
    int found = 0;
    int notfound = 0;
    double ticks = 0;
    while(ch!=27)
    {
    	
        cap >> img;
        Mat img1;
        img.copyTo( img1 );
        double precTick = ticks;
        ticks = (double) getTickCount(); 
        double T = (ticks - precTick) / getTickFrequency();
        if(found)
        {
        	k.transitionMatrix.at<float>(2) = T;
            k.transitionMatrix.at<float>(9) = T;
            cout << "T:" << T << endl;
            predict = k.predict();
            cout << " State:" << predict<< endl;
            Rect predRect;
            predRect.width = predict.at<float>(4);
            predRect.height = predict.at<float>(5);
            predRect.x = predict.at<float>(0) - predRect.width / 2;
            predRect.y = predict.at<float>(1) - predRect.height / 2;
            rectangle(img1, predRect, CV_RGB(0,255,0), 2);
            
        }
        Mat smooth;
        GaussianBlur(img, smooth, Size(5, 5), 3.0, 3.0);
        Mat Hsv;
        cvtColor(smooth, Hsv, CV_BGR2HSV);
        Mat result (img.rows,img.cols,CV_8UC1, Scalar(0));
        inRange(Hsv, Scalar(0,0,0),Scalar(180, 255, 60),result);
        erode(result, result, Mat(), Point(-1, -1), 2);
        dilate(result, result , Mat(), Point(-1, -1), 2);
        namedWindow("result",WINDOW_NORMAL);
        imshow("result", result);
 		vector<vector<Point> > contours;
        findContours(result, contours, CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
        vector<vector<Point> > Goals;
        vector<Rect> Goals_Box;
        for (size_t i = 0; i < contours.size(); i++)
        {
            Rect gBox;
            gBox = boundingRect(contours[i]); 
            float ratio = (float) gBox.width / (float) gBox.height;            
            if (ratio >= 0.8 && ratio <= 1.25 && gBox.area() >= 10000) 
            {
                Goals.push_back(contours[i]);
                Goals_Box.push_back(gBox);
            }
        }
        cout << "no of Goals found:" << Goals_Box.size() << endl;
        for (size_t i = 0; i < Goals.size(); i++)
        {
            drawContours(img1, Goals, i, CV_RGB(255,255,255), 1);
            rectangle(img1, Goals_Box[i], CV_RGB(0,255,0), 2);              
        }
        if (Goals.size() == 0)
        {
            notfound++;
        }
        else
        {
        	notfound = 0;
        	measured.at<float>(0) = Goals_Box[0].x + Goals_Box[0].width / 2;
            measured.at<float>(1) = Goals_Box[0].y + Goals_Box[0].height / 2;
            measured.at<float>(2) = (float)Goals_Box[0].width;
            measured.at<float>(3) = (float)Goals_Box[0].height;
            if (!found)                                                
            {
                
                predict.at<float>(0) = measured.at<float>(0);
                predict.at<float>(1) = measured.at<float>(1);
                predict.at<float>(2) = 0;
                predict.at<float>(3) = 0;
                predict.at<float>(4) = measured.at<float>(2);
                predict.at<float>(5) = measured.at<float>(3); 
                k.errorCovPre.at<float>(0) = 1; 
                k.errorCovPre.at<float>(7) = 1; 
                k.errorCovPre.at<float>(14) = 1;
                k.errorCovPre.at<float>(21) = 1;
                k.errorCovPre.at<float>(28) = 1;
                k.errorCovPre.at<float>(35) = 1;
               
                k.statePost = predict;
                
                found = 1;
            }
             else
                k.correct(measured);
            cout << "Measure matrix:" << endl << measured << endl; 

        }
        namedWindow("tracking",WINDOW_NORMAL);
        imshow("tracking", img1);
        video.write(img1);
        ch = waitKey(20);


    }
    cap.release();
    video.release();
    destroyAllWindows();
	return 0;
        

}
