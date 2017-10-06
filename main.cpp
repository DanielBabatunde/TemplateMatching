#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"

#include <iostream>
#include <ctype.h>

using namespace cv;
using namespace std;


bool pointTrackingFlag = false;
Point2f currentPoint;
vector<Point2i>TrackedPoints;


static void onMouse( int event, int x, int y, int, void* )
{

    //Detect Mouse button event
    if( event == EVENT_LBUTTONDOWN )
    {
        //Assign current (x,y) position to currentPoint
        currentPoint = Point2f((float)x, (float)y);

        // set Tracking flag
        pointTrackingFlag = true;
    }
}

int main( int argc, char** argv)
{
    VideoCapture cap;
    Mat frame;
    Mat image;
    Mat prevGrayImage;
    Mat curGrayImage;
    //float scalingFactor = 0.5;
    vector <Point2f> trackingPoints[2];
    Size winSize(31,31);
    bool playVideo = false;
    double distance;
    double velocity;



    namedWindow( "Demo", 1 );

    setMouseCallback("Demo", onMouse, 0);

     cap.open("C:/Users/Jsquadron/Downloads/sample.mp4" );

     double fps = cap.get(CV_CAP_PROP_FPS);
     double height = cap.get(CV_CAP_PROP_FRAME_HEIGHT);
     double width = cap.get(CV_CAP_PROP_FRAME_WIDTH);


    if(!cap.isOpened())
    {
        cerr << "Unable to open video" << endl;
        return -1;
    }

    // Termination criteria for tracking the points in the video
   TermCriteria termcrit(TermCriteria::COUNT|TermCriteria::EPS,20,0.03);
     cap >> frame;

    while (true)
    {
       if(playVideo)
        cap >> frame;

        if(frame.empty())
            break;

        // Resize frame
        //resize(frame, frame, Size(), scalingFactor, scalingFactor, INTER_AREA)

        //Copy input frame
        frame.copyTo(image);

        // Convert image to grayScale
        cvtColor(image, curGrayImage, COLOR_BGR2GRAY);

        // Check if there are points to track.
        if(!trackingPoints[0].empty())
        {
            //Status vector to check if
            //flow of corresponding features have been found
            vector<uchar> statusVector;

            //Error Vector to indicate error for corresponing features
            vector<float> errorVector;

            //is previous image empty?
            if(prevGrayImage.empty())
            {
                //Copy current gray image to previous gray image
                curGrayImage.copyTo(prevGrayImage);
            }

            //Calculate Optical Flow using LK Algorithm
            calcOpticalFlowPyrLK(prevGrayImage, curGrayImage, trackingPoints[0], trackingPoints[1],
                                  statusVector, errorVector, winSize,3, termcrit, 0, 0.001);

            int count = 0;

            //Minumum distance between any tracking points
            int minDist =3;

            for(int i = 0; i < trackingPoints[1].size(); i++)
            {
                // if new point is within minimum distance from
                //an existing point, it won't be tracked
                if(norm(currentPoint - trackingPoints[1][i]) <= minDist)
                {
                    pointTrackingFlag = false;
                    continue;
                }

                // Check status vector
                if(!statusVector[i])
                {
                    continue;
                }

                trackingPoints[1][count++] = trackingPoints[1][i];
                TrackedPoints.push_back(trackingPoints[1][i]);

                for(int i = 0; i<TrackedPoints.size(); i++)
                {
                     distance = norm(TrackedPoints[i+1]-TrackedPoints[i]);
                    circle(image,Point(TrackedPoints[i].x, TrackedPoints[i].y), 4, Scalar(0,0,255), 8, 4, 0);
                    velocity = distance/fps;

                    //cout << "Distance = " << distance << endl;
                    cout << "Velocity =" << velocity << "pixels per second" << endl;
                }

                //Draw a circle for each tracked point
                //circle(image, trackingPoints[1][i], 8, Scalar(255,0,0), 2, 8);

            }

            trackingPoints[1].resize(count);


        }

        int maxPoints = 500;

        //Refine Location for tracking points
        if(pointTrackingFlag && trackingPoints[1].size() < maxPoints)
        {
            vector<Point2f> tempPoints;
            // Add current mouse point to tracking point vector
            tempPoints.push_back(currentPoint);

            //Refine location of pixels to subpixel accuracy
            // Pixel = image patch of Window Size not image pixel
            cornerSubPix(curGrayImage, tempPoints, winSize, cvSize(-1, -1), termcrit);

            // Add first element of vector to current tracking points
            trackingPoints[1].push_back(tempPoints[0]);
            pointTrackingFlag = false;

        }

        // Display image with tracking points

        imshow("Demo", image);

        // Is Esc key pressed?  // change this to a string
        char c = (char)waitKey(10);
              if( c == 27 )
                  break;

          switch (c)
          {
            case 'p':
             playVideo = !playVideo;

          }

         //Swap "points vector to update previous point to current
         std::swap(trackingPoints[1], trackingPoints[0]);

         //Swap images to update previous point to current
         cv::swap(prevGrayImage, curGrayImage);


    }
    return 0;
}
