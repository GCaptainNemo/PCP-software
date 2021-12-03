#pragma once

int IsFolderExist(const char* path);

int IsFileExist(const char* path);

void plot_corners(IplImage *timg);

void on_mouse(int event, int x, int y, int flags, void* img);

// range image completion
void range_image_complete(const cv::Mat &sparse_range_img, cv::Mat &dense_range_img, const bool &is_extrapolate, const std::string &blur_type);

