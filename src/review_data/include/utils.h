#pragma once

int IsFolderExist(const char* path);

int IsFileExist(const char* path);

void plot_corners(IplImage *timg);

void on_mouse(int event, int x, int y, int flags, void* img);
