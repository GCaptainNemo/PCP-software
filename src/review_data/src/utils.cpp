#include <opencv/cv.h> // IplImage
#include <opencv2/opencv.hpp>

extern std::vector<cv::Point2f> corners;
extern std::vector<cv::Point2f> ir_corners;
extern cv::Mat range_img;
extern cv::Mat dense_range_img;
extern std::vector<float> depth_vector;
extern std::vector<float> dense_depth_vector;



void plot_corners(IplImage *timg)
{
    CvFont font;
    cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 0.8, 0.8, 0, 1, CV_AA);
    int col_num = timg->width / 2;
    int row_num = timg->height / 2;
    for(int i=0; i<corners.size(); ++i)
    {
        char rgb_char[32];
        char r_img_char[32];
        char dense_r_img_char[32];
        CvPoint pt = cvPoint(corners[i].x, corners[i].y);
        CvPoint r_img_pt = cvPoint(corners[i].x + col_num, corners[i].y);
        CvPoint dense_r_img_pt = cvPoint(corners[i].x + col_num, corners[i].y + row_num);
        
        // ROS_INFO("%d: (%f,%f)", i, corners[i].x, corners[i].y);
        sprintf(rgb_char, "%d: (%d,%d) ", i + 1, int(corners[i].x), int(corners[i].y));
        sprintf(r_img_char, "%d: (%d,%d) d: %f", i + 1, int(corners[i].x), int(corners[i].y), depth_vector[i]);
        sprintf(dense_r_img_char, "%d: (%d,%d) d: %f", i + 1, int(corners[i].x), int(corners[i].y), dense_depth_vector[i]);
        
        cvPutText(timg, rgb_char, pt, &font, CV_RGB(255,0,0));
        cvPutText(timg, r_img_char, r_img_pt, &font, CV_RGB(255,0,0));
        cvPutText(timg, dense_r_img_char, dense_r_img_pt, &font, CV_RGB(255,0,0));

        cvCircle(timg, pt, 2, cvScalar(255, 0, 0, 0), CV_FILLED, CV_AA, 0);
        cvCircle(timg, r_img_pt, 2, cvScalar(255, 0, 0, 0), CV_FILLED, CV_AA, 0);
        cvCircle(timg, dense_r_img_pt, 2, cvScalar(255, 0, 0, 0), CV_FILLED, CV_AA, 0);

    }
    for(int i = 0; i < ir_corners.size(); ++i)
    {
        char temp[16];
        CvPoint pt = cvPoint(ir_corners[i].x, ir_corners[i].y + row_num);
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
    int col_num = timg->width / 2;
    int row_num = timg->height / 2;

        
    if (event == CV_EVENT_RBUTTONDOWN)
    {
        
        if (x < col_num && y >= row_num)
        {
            // ir img
            y -= row_num;
            cv::Point2f p;
            p.x = x;
            p.y = y;
            ir_corners.push_back(p);
        }
        else 
        {
            // rgb img and range image

            while(x >= col_num)
            {
                x -= col_num;
            }
            while(y >= row_num)
            {
                y -= row_num;
            }
            cv::Point2f p;
            p.x = x;
            p.y = y;
            corners.push_back(p);
            float depth = range_img.at<float>(y, x);
            float dense_depth = dense_range_img.at<float>(y, x);
            depth_vector.push_back(depth);
            dense_depth_vector.push_back(dense_depth);
        }
        plot_corners(timg);

    }
    else if( event==CV_EVENT_MBUTTONDOWN) // 中健点击
    {
        if (x < col_num && y >= row_num)
        {
            // ir img
            ir_corners.pop_back();
        }
        else
        {
            // rgb_img and range_img
            corners.pop_back();
            depth_vector.pop_back();
            dense_depth_vector.pop_back();
        }
        
        plot_corners(timg);

    }
}

// use morphology and traditional filter method to complete range image
void range_image_complete(const cv::Mat &sparse_r_img, cv::Mat &dense_r_img, const bool &is_extrapolate, const std::string &blur_type)
{
    float max_r_img_val = 0.0;
    int range_img_row = sparse_r_img.rows;
    int range_img_col = sparse_r_img.cols;
    dense_r_img = sparse_r_img.clone();
    
    for(int i=0; i < range_img_row; ++i)
    {
        for(int j = 0; j < range_img_col; ++j)
        {
            float depth = dense_r_img.at<float>(i, j);
            if (depth > max_r_img_val){max_r_img_val = depth;}
        }
    }
    // invert
    float invert_buffer = 20.0;
    for(int i=0; i < range_img_row; ++i)
    {
        for(int j = 0; j < range_img_col; ++j)
        {
            float depth = dense_r_img.at<float>(i, j);
            if (depth > 0.1){dense_r_img.at<float>(i, j) = max_r_img_val + invert_buffer - depth;}
        }
    }
    // Dilate 整体形态学膨胀
    int a[5][5] = { 0, 0, 1, 0, 0, 
                    0, 1, 1, 1, 0, 
                    1, 1, 1, 1, 1, 
                    0, 1, 1, 1, 0, 
                    0, 0, 1, 0, 0};
    cv::Mat diamond_kernel_5(5, 5, CV_8UC1, a);
    // std::cout << "start dilation: " << diamond_kernel_5 << std::endl;
    
    cv::dilate(dense_r_img, dense_r_img, diamond_kernel_5);

    // Hole closing 整体形态学闭操作
    cv::Mat full_kernel_5 = cv::Mat::ones(5, 5, CV_8UC1);
    cv::morphologyEx(dense_r_img, dense_r_img, cv::MORPH_CLOSE, full_kernel_5);
    
    // Fill empty spaces with dilated values// 专门针对空的区域进行填充(Holl filling)
    cv::Mat specific_range_img = dense_r_img.clone();
    cv::Mat full_kernel_7 = cv::Mat::ones(7, 7, CV_8UC1);
    cv::dilate(specific_range_img, specific_range_img, full_kernel_7);
    for(int i=0; i < range_img_row; ++i)
    {
        for(int j = 0; j < range_img_col; ++j)
        {
            float depth = dense_r_img.at<float>(i, j);
            if (depth < 0.1){dense_r_img.at<float>(i, j) = specific_range_img.at<float>(i, j);}
        }
    }
    // Extend highest pixel to top of image
    if (is_extrapolate)
    {
        
        for(int j = 0; j < range_img_col; ++j)
        {
            int max_index = 0;
            float max_val = -1;
            float min_val = 100;
            int min_index = range_img_row - 1;
            for(int i = 0; i < range_img_row; ++i)
            {
                if (dense_r_img.at<float>(i, j) > 0.1)
                {
                    max_index = i; 
                    max_val = dense_r_img.at<float>(i, j);
                };
                if (dense_r_img.at<float>(range_img_row - 1 - i, j) > 0.1)
                {
                    min_index = range_img_row - 1 - i; 
                    min_val = dense_r_img.at<float>(range_img_row - 1 - i, j);
                };
            }
            for(int i = max_index; i < range_img_row; ++i)
            {
                dense_r_img.at<float>(i, j) = max_val;
            }
            for(int i = min_index; i >= 0; --i)
            {
                dense_r_img.at<float>(i, j) = min_val;
            }
        }
        //  Large Fill（对为空的区域再膨胀一次用31）
        cv::Mat full_kernel_31 = cv::Mat::ones(31, 31, CV_8UC1);
        while(true)
        {
            specific_range_img = dense_r_img.clone();
            cv::dilate(specific_range_img, specific_range_img, full_kernel_31);
            int hole_pixel_count = 0;
            for(int i=0; i < range_img_row; ++i)
            {
                for(int j = 0; j < range_img_col; ++j)
                {
                    float depth = dense_r_img.at<float>(i, j);
                    if (depth < 0.1){dense_r_img.at<float>(i, j) = specific_range_img.at<float>(i, j); hole_pixel_count++;}
                }
            }
            if (hole_pixel_count == 0){break;}
        }
    }

    // Median blur 整体模糊
    cv::medianBlur(dense_r_img, dense_r_img, 5);

    // Bilateral or Gaussian blur
    if(blur_type == std::string("bilateral"))
    {
        // Bilateral blur（对所有像素用双边滤波）
        cv::bilateralFilter(dense_r_img, dense_r_img, 5, 1.5, 2.0);
    }
    else if(blur_type == std::string("gaussian"))
    {
        // Gaussian blur 仅对深度值有定义的地方进行高斯模糊
        specific_range_img = dense_r_img.clone();
        cv::GaussianBlur(specific_range_img, specific_range_img, cv::Size(5, 5), 0);

        for(int i=0; i < range_img_row; ++i)
        {
            for(int j = 0; j < range_img_col; ++j)
            {
                float depth = dense_r_img.at<float>(i, j);
                if (depth > 0.1){dense_r_img.at<float>(i, j) = specific_range_img.at<float>(i, j);}
            }
        }
    }
    // Invert
    for(int i=0; i < range_img_row; ++i)
    {
        for(int j = 0; j < range_img_col; ++j)
        {
            float depth = dense_r_img.at<float>(i, j);
            if (depth > 0.1)
            {
                dense_r_img.at<float>(i, j) = max_r_img_val + invert_buffer- dense_r_img.at<float>(i, j);
            }
        }
    }
}


