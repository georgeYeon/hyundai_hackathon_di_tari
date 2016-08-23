#pragma once
#include <opencv2/highgui/highgui.hpp>

#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>
using namespace cv;

using namespace std;

void OCRinit();

void DoOCR(Mat sub, string &out);

void OCRend();