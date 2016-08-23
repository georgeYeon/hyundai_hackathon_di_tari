// Main.h

#ifndef MY_MAIN         // used MY_MAIN for this include guard rather than MAIN just in case some compilers or environments #define MAIN already
#define MY_MAIN

#define FILENAME "test3.png"


#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>

#include "DetectPlates.h"
#include "PossiblePlate.h"
#include "DetectChars.h"
#include "OCR.h"

#include<iostream>

#define UPPERTHRESHOLD 0.2
#define LOWERTHRESHOLD 0.8


//#define SHOW_STEPS            // un-comment or comment this line to show steps or not

// global constants /////////////////////////////////////////////////////////////////////////+//////
const cv::Scalar SCALAR_BLACK = cv::Scalar(0.0, 0.0, 0.0);
const cv::Scalar SCALAR_WHITE = cv::Scalar(255.0, 255.0, 255.0);
const cv::Scalar SCALAR_YELLOW = cv::Scalar(0.0, 255.0, 255.0);
const cv::Scalar SCALAR_GREEN = cv::Scalar(0.0, 255.0, 0.0);
const cv::Scalar SCALAR_RED = cv::Scalar(0.0, 0.0, 255.0);

// function prototypes ////////////////////////////////////////////////////////////////////////////
int main(void);


bool CompareString(string &a, string &b, int aSize, int bSize);


void drawRedRectangleAroundPlate(cv::Mat &imgOriginalScene, PossiblePlate &licPlate);
void writeLicensePlateCharsOnImage(cv::Mat &imgOriginalScene, PossiblePlate &licPlate);
Rect passRectInfo(cv::Mat realImg, PossiblePlate licPlate);
# endif	// MAIN

