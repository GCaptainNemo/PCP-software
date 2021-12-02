#include <sys/types.h>
#include <dirent.h>
#include <unistd.h>

#include <opencv/cv.h> // IplImage
#include <opencv2/opencv.hpp>

extern std::vector<cv::Point2f> corners;
extern std::vector<cv::Point2f> ir_corners;
extern cv::Mat range_img;
extern std::vector<float> depth_vector;



int IsFolderExist(const char* path)
{
    DIR *dp;
    if ((dp = opendir(path)) == NULL)
    {
        return 0;
    }
 
    closedir(dp);
    return -1;
}

int IsFileExist(const char* path)
{
    return !access(path, F_OK);
}


void plot_corners(IplImage *timg)
{
    CvFont font;
    cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 0.8, 0.8, 0, 1, CV_AA);
    int col_num = timg->width / 3;
    for(int i=0; i<corners.size(); ++i)
    {
        char temp[32];
        CvPoint pt = cvPoint(corners[i].x, corners[i].y);
        CvPoint next_pt = cvPoint(corners[i].x + col_num, corners[i].y);
        // ROS_INFO("%d: (%f,%f)", i, corners[i].x, corners[i].y);
        sprintf(temp, "%d: (%d,%d) d: %f", i + 1, int(corners[i].x), int(corners[i].y), depth_vector[i]);
        cvPutText(timg, temp, pt, &font, CV_RGB(255,0,0));
        cvPutText(timg, temp, next_pt, &font, CV_RGB(255,0,0));
        cvCircle(timg, pt, 2, cvScalar(255, 0, 0, 0), CV_FILLED, CV_AA, 0);
        cvCircle(timg, next_pt, 2, cvScalar(255, 0, 0, 0), CV_FILLED, CV_AA, 0);
    }
    for(int i = 0; i < ir_corners.size(); ++i)
    {
        char temp[16];
        CvPoint pt = cvPoint(ir_corners[i].x + 2 * col_num, ir_corners[i].y);
        sprintf(temp, "%d: (%d,%d)", i + 1, int(ir_corners[i].x), int(ir_corners[i].y));
        cvPutText(timg, temp, pt, &font, CV_RGB(255,0,0));
        cvCircle(timg, pt, 2, cvScalar(255, 0, 0, 0), CV_FILLED, CV_AA, 0);
    }
    cvShowImage("src", timg);
    cvReleaseImage(&timg);
}

void on_mouse(int event, int x, int y, int flags, void* img)
{
    IplImage *timg = cvCloneImage((IplImage *)img);
    int col_num = timg->width / 3;
        
    if (event == CV_EVENT_RBUTTONDOWN)
    {
        
        if (x >= 2 * col_num)
        {
            x -= 2 * col_num;
            cv::Point2f p;
            p.x = x;
            p.y = y;
            ir_corners.push_back(p);
            float depth = range_img.at<float>(y, x);
            depth_vector.push_back(depth);
        }
        else if (x >= col_num )
        {
            x -= col_num;
            cv::Point2f p;
            p.x = x;
            p.y = y;
            corners.push_back(p);
            float depth = range_img.at<float>(y, x);
            depth_vector.push_back(depth);
        
        }
        else{
            cv::Point2f p;
            p.x = x;
            p.y = y;
            corners.push_back(p);
        };  
        plot_corners(timg);

    }
    else if( event==CV_EVENT_MBUTTONDOWN) // 中健点击
    {
        if (x < 2 * col_num && corners.size() != 0)
        {
            corners.pop_back();
            depth_vector.pop_back();
        }
        else if(x >= 2 * col_num && ir_corners.size() != 0){
            ir_corners.pop_back();
        }
        plot_corners(timg);

    }
}
