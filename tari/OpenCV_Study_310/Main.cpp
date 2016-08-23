// Main.cpp

#include "Main.h"
#include "opencv.hpp"

///////////////////////////////////////////////////////////////////////////////////////////////////
using namespace cv;
using namespace std;
#define DIST_TH 0.4 //threshold for histogram matching
#define DETECT_TIME 48 // 번호판 인식 사잇 시간
Rect selection[10];
string str[10];
typedef enum { INIT, CALC_HIST, TRACKING }STATUS;
STATUS trackingMode[10];
bool tag[10] = { false, };
Rect trackWindow[10];                       //추적 윈도우
KalmanFilter KF[10];
Mat hist1[10];

int main(void) {

	OCRinit();

	bool blnKNNTrainingSuccessful = loadKNNDataAndTrainKNN();           // attempt KNN training

	if (blnKNNTrainingSuccessful == false) {                            // if KNN training was not successful
																		// show error message
		std::cout << std::endl << std::endl << "error: error: KNN traning was not successful" << std::endl << std::endl;
		return(0);                                                      // and exit program
	}

	VideoCapture inputVideo("IMG_6522.mov");

	if (!inputVideo.isOpened()) {
		cout << "Can not open inputVideo!" << endl;
		return 0;
	}

	//입력 영상의 크기와 fps를 받음
	Size size = Size((int)inputVideo.get(CV_CAP_PROP_FRAME_WIDTH), (int)inputVideo.get(CV_CAP_PROP_FRAME_HEIGHT));
	int fps = (int)(inputVideo.get(CV_CAP_PROP_FPS));
	if (fps == 0) fps = 24; //for camera

	Mat dstImage;
	namedWindow("dstImage", 1);
	//resizeWindow("dstImage", 540, 360);

	int histSize = 8;                        //히스토에 관한 정보들 시작 ~
	float valueRange[] = { 0,180 }; //hue's maximum is 180
	const float* ranges[] = { valueRange };
	int channels = 0;                        // ~ 끝
	Mat hist, backProject;

	TermCriteria criteria = TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 10, 2);  //meanShift의 종료 판정

	Mat frame, hImage, hsvImage, mask;
	int delay = 1000 / fps;
	//칼만필터//
	Point2f ptPredicted;   //예측된 중심 좌표
	Point2f ptEstimated;   //갱신된 중심 좌표
	Point2f ptMeasured;    //meanShift에 의한 관측 좌표

	for (int i = 0; i < 10; i++) {
		KF[i] = KalmanFilter(4, 2, 0);
		trackingMode[i] = INIT;
	}

	Mat measurement(2, 1, CV_32F);          //측정값을 위한 행렬 객체 

	float dt = 1.0;   //시간 간격은 1
					  //Transition matrix A describes model parameters at k-1 and k
	const float A[] = { 1,0,dt,0,   //변환 행렬
		0,1,0,dt,
		0,0,1,0,
		0,0,0,1 };
	for (int i = 0; i < 10; i++) {
		memcpy(KF[i].transitionMatrix.data, A, sizeof(A));
		//cout << "KF.transitionMatrix = " << KF.transitionMatrix << endl;
	}
	//Initialize Kalman parameters
	double Q = 1e-5;
	double R = 0.0001;
	const float H[] = { 1,0,0,0,
		0,1,0,0 };
	for (int i = 0; i < 10; i++) {
		memcpy(KF[i].measurementMatrix.data, H, sizeof(H));
		//cout << "KF.measurementMatrix = " << KF.measurementMatrix << endl;
		setIdentity(KF[i].processNoiseCov, Scalar::all(Q));
		KF[i].processNoiseCov.at<float>(2, 2) = 0;
		KF[i].processNoiseCov.at<float>(3, 3) = 0;
		//cout << "KF.processNoiseCov = " << KF.processNoiseCov << endl;

		setIdentity(KF[i].measurementNoiseCov, Scalar::all(R));
		//cout << "KF.measurementNoiseCov = " << KF.measurementNoiseCov << endl;
	}
	Mat hist2; //for histogram matching

	int detect_time = 0;
	
	int cnt = 0;
	for (;;) {
		cnt++;
		inputVideo >> frame;       //frame을 받아서
		if (frame.empty())
			break;
		//resize(frame, frame, Size(800, 800));
		cvtColor(frame, hsvImage, COLOR_BGR2GRAY);   //hsv에 그레이 스케일 저장
		dstImage = frame.clone();                      //dstImage에 복사

		if (++detect_time > DETECT_TIME) {
			detect_time = 0;
			cv::Mat imgOriginalScene;           // input image

			imgOriginalScene = dstImage.clone();         // open image

			cv::Mat realImg = imgOriginalScene.clone();

			imgOriginalScene = imgOriginalScene(Rect(Point(0, imgOriginalScene.rows * LOWERTHRESHOLD), Point(imgOriginalScene.cols, imgOriginalScene.rows * UPPERTHRESHOLD)));

			//파일이 없는 파일이면
			if (imgOriginalScene.empty()) {                             // if unable to open image
				std::cout << "error: image not read from file\n\n";     // show error message on command line
				return(0);                                              // and exit program
			}


			//번호판 찾기
			std::vector<PossiblePlate> vectorOfPossiblePlates = detectPlatesInScene(imgOriginalScene);          // detect plates


			vectorOfPossiblePlates = detectCharsInPlates(vectorOfPossiblePlates);                               // detect chars in plates



		//	cv::imshow("imgOriginalScene", imgOriginalScene);           // show scene image

			if (vectorOfPossiblePlates.empty()) {                                               // if no plates were found
				std::cout << std::endl << "no license plates were detected" << std::endl;       // inform user no plates were found
			}
			else {                                                                            // else
																				  // if we get in here vector of possible plates has at leat one plate
				std::cout << vectorOfPossiblePlates.size() << " possible plate found" << std::endl;

				for (int num = 0; num < vectorOfPossiblePlates.size(); num++) {
					// sort the vector of possible plates in DESCENDING order (most number of chars to least number of chars)
					std::sort(vectorOfPossiblePlates.begin(), vectorOfPossiblePlates.end(), PossiblePlate::sortDescendingByNumberOfChars);
					// suppose the plate with the most recognized chars (the first plate in sorted by string length descending order) is the actual plate
					PossiblePlate licPlate = vectorOfPossiblePlates[num];

					//					cv::imshow("imgPlate", licPlate.imgPlate);            // show crop of plate and threshold of plate

					string out;
					DoOCR(licPlate.imgPlate, out);
					if (out.empty()) {
						continue;
					}

					bool tagAlready = false;
					int tagIndex = -1;
					for (int i = 0; i < 10 && tag[i]; i++) {
						if (CompareString(out, str[i], out.size(), str[i].size())) {
							tagAlready = true;
							tagIndex = i;
							break;
						}

					}
					if (!tagAlready) {
						Rect plateLocation = passRectInfo(realImg, licPlate);

						Point beforeTag(selection[num].x, selection[num].y);
						Point afterTag(selection[tagIndex].x, selection[tagIndex].y);
						double dist = norm(beforeTag - afterTag);
						if (dist < 100) {
							int W = MIN(selection[num].width, selection[tagIndex].width);
							int H = MIN(selection[num].height, selection[tagIndex].height);
							selection[tagIndex] = selection[num];
							selection[tagIndex].width = W;
							selection[tagIndex].height = H;

							tag[num] = false;
							continue;
						}

						for (int i = 0; i < 10; i++) {
							if (tag[i] == false) {
								int centerX = (plateLocation.x + plateLocation.width / 2.0);
								int centerY = (plateLocation.y + plateLocation.height / 2.0);

								int dis = plateLocation.width;
								selection[i] = Rect(MAX(0, centerX - dis), MAX(centerY - dis, 0), MIN(2 * dis, dstImage.cols - centerX), MIN(2 * dis, dstImage.rows - centerY));
								str[i] = out;

								trackingMode[i] = CALC_HIST;                      //모드를 hist 계산할 수 있게 flag를 바꿔줌
								tag[i] = true;
								break;
							}
						}

					}

					//drawRedRectangleAroundPlate(realImg, licPlate);                // draw red rectangle around plate
				}

			}
		}

		for (int i = 0; i < 10 && tag[i]; i++) {
			//			if (0 < selection[i].width && 0 < selection[i].height) {

			Mat dstROI = dstImage(Rect(selection[i])).clone();                  //드래그 하고 있는 관심 영역에 대해  ///////////HERE!!///////////////////////////////


			bitwise_xor(dstROI, Scalar::all(255), dstROI);     //이진화 

	//////	}
			if (trackingMode) {//CLAC_HIST or TRACKING
						   //crate mask image

				int vmin = 50, vmax = 256, smin = 50;
				inRange(hsvImage, Scalar(0, smin, MIN(vmin, vmax)), Scalar(180, 256, MAX(vmin, vmax)), mask);  //mask는 이진화 영상
																											   //imshow("mask",mask);
				int ch[] = { 0,0 };
				hImage.create(hsvImage.size(), CV_8U);           //hImage를 그레이 스케일 프레임을 받음
				mixChannels(&hsvImage, 1, &hImage, 1, ch, 1);    //mixChannel ???
																 //imshow("hImage",hImage);
				if (trackingMode[i] == CALC_HIST) {      //히스토그램을 만듦
					Mat hImageROI(hImage, selection[i]), maskROI(mask, selection[i]); //그레이스케일에서 관심영역, 이진화 영역에서 관심영역
					calcHist(&hImageROI, 1, &channels, maskROI, hist, 1, &histSize, ranges); //hImage의 히스토그램 계산
					hist.copyTo(hist1[i]);       //hist1에 복사
					normalize(hist1[i], hist1[i], 1.0);  //히스토그램 정규화
					normalize(hist, hist, 0, 255, NORM_MINMAX); //for backprojection
					trackWindow[i] = selection[i];           //관심영역을 추적영역으로 설정
					trackingMode[i] = TRACKING;
					//initialize the state vector(position and velocty)
					ptMeasured = Point2f(trackWindow[i].x + trackWindow[i].width / 2.0, trackWindow[i].y + trackWindow[i].height / 2.0); //meanShift에 의한 측정값 초기화

					KF[i].statePost.at<float>(0, 0) = ptMeasured.x;   //상태 벡터 초기화                                  
					KF[i].statePost.at<float>(1, 0) = ptMeasured.y;
					KF[i].statePost.at<float>(2, 0) = 0;
					KF[i].statePost.at<float>(3, 0) = 0;

					setIdentity(KF[i].errorCovPost, Scalar::all(1));   //error 초기화
				}
				Mat prediction = KF[i].predict(); //predict 예측 수행 
				ptPredicted.x = prediction.at<float>(0, 0);   //예측좌표를 ptPredicted에 저장
				ptPredicted.y = prediction.at<float>(1, 0);

				//TRACKING:
				calcBackProject(&hImage, 1, &channels, hist, backProject, ranges);
				backProject &= mask;
				//bitwise_and(backProject, mask, backProject);
				//imshow("backProject",backProject);

				meanShift(backProject, trackWindow[i], criteria);
				Point pt1 = Point2f(trackWindow[i].x, trackWindow[i].y);
				Point pt2 = Point2f(pt1.x + trackWindow[i].width, pt1.y + trackWindow[i].height);
				//rectangle(dstImage, pt1, pt2, Scalar(0, 0, 255), 2);

				//Validate the result of cvMeanShift
				Mat hImageROI(hImage, trackWindow[i]), maskROI(mask, trackWindow[i]);
				calcHist(&hImageROI, 1, &channels, maskROI, hist2, 1, &histSize, ranges);
				normalize(hist2, hist2, 1.0);
				double dist = compareHist(hist1[i], hist2, CV_COMP_BHATTACHARYYA);  //고침

				if (dist < DIST_TH) { //A tracknig object is detected by meanShift
					ptMeasured = Point2f(trackWindow[i].x + trackWindow[i].width / 2.0, trackWindow[i].y + trackWindow[i].height / 2.0);

					//measurements : the center point of the track_window
					measurement.at<float>(0, 0) = ptMeasured.x;
					measurement.at<float>(1, 0) = ptMeasured.y;

					Mat estimated = KF[i].correct(measurement); //update

					ptEstimated.x = estimated.at<float>(0, 0);
					ptEstimated.y = estimated.at<float>(1, 0);

					trackWindow[i] = Rect(ptEstimated.x - selection[i].width / 2, ptEstimated.y - selection[i].height / 2, selection[i].width, selection[i].height);

					pt1 = Point(ptMeasured.x - trackWindow[i].width / 2, ptMeasured.y - trackWindow[i].height / 2);
					pt2 = Point(ptMeasured.x + trackWindow[i].width / 2, ptMeasured.y + trackWindow[i].height / 2);
					//rectangle(dstImage, pt1, pt2, Scalar(0, 0, 255), 2);
					if(i == 1 && cnt > 200)
						circle(dstImage, ptMeasured, 20, Scalar(0, 0, 255), 2);

					pt1 = Point(ptEstimated.x - trackWindow[i].width / 2, ptEstimated.y - trackWindow[i].height / 2);
					pt2 = Point(ptEstimated.x + trackWindow[i].width / 2, ptEstimated.y + trackWindow[i].height / 2);
					//rectangle(dstImage, pt1, pt2, Scalar(255, 0, 0), 2);
					//circle(dstImage, ptMeasured, 5, Scalar(0, 255, 0), 2);
				}
				else {   //추적하다 놓침
					trackWindow[i] = Rect(ptPredicted.x - selection[i].width / 2, ptPredicted.y - selection[i].height / 2, selection[i].width, selection[i].height);

					pt1 = Point(ptPredicted.x - trackWindow[i].width / 2, ptPredicted.y - trackWindow[i].height / 2);
					pt2 = Point(ptPredicted.x + trackWindow[i].width / 2, ptPredicted.y + trackWindow[i].height / 2);
					//rectangle(dstImage, pt1, pt2, Scalar(0, 255, 0), 2);
					//circle(dstImage, ptPredicted, 5, Scalar(0, 255, 0), 2);
					tag[i] = false;       //태그 놓치기
				}
			}
			
		}

		imshow("dstImage", dstImage);
		int ckey = waitKey(delay);
		if (ckey == 27) break;
	}
	OCRend();

	return(0);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

bool CompareString(string &a, string &b, int aSize, int bSize) {
	//	int COUNT = 0;
	Mat A = Mat::zeros(Size(10, 1), CV_8U);
	Mat B = Mat::zeros(Size(10, 1), CV_8U);
	for (int i = 0; i < aSize; i++)
		if (('0' <= a[i]) && (a[i] <= '9'))
			A.at<uchar>(Point(a[i] - '0', 0))++;
	for (int i = 0; i < bSize; i++)
		if (('0' <= b[i]) && (b[i] <= '9')) {
			//COUNT++;
			B.at<uchar>(Point(b[i] - '0', 0))++;
		}
	double dist = norm(A, B);
	if (dist < 3)	return true;
	else return false;
}


Rect passRectInfo(cv::Mat realImg, PossiblePlate licPlate) {

	cv::Point2f p2fRectPoints[4];

	licPlate.rrLocationOfPlateInScene.points(p2fRectPoints);            // get 4 vertices of rotated rect

	return Rect(cv::Point(p2fRectPoints[0].x, p2fRectPoints[0].y + realImg.rows * UPPERTHRESHOLD), cv::Point(p2fRectPoints[2].x, p2fRectPoints[2].y + realImg.rows * UPPERTHRESHOLD));
}

void drawRedRectangleAroundPlate(cv::Mat &imgOriginalScene, PossiblePlate &licPlate) {
	cv::Point2f p2fRectPoints[4];

	licPlate.rrLocationOfPlateInScene.points(p2fRectPoints);            // get 4 vertices of rotated rect

	for (int i = 0; i < 4; i++) {                                       // draw 4 red lines
		cv::line(imgOriginalScene, p2fRectPoints[i], p2fRectPoints[(i + 1) % 4], SCALAR_RED, 2);
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////
void writeLicensePlateCharsOnImage(cv::Mat &imgOriginalScene, PossiblePlate &licPlate) {
	cv::Point ptCenterOfTextArea;                   // this will be the center of the area the text will be written to
	cv::Point ptLowerLeftTextOrigin;                // this will be the bottom left of the area that the text will be written to

	int intFontFace = CV_FONT_HERSHEY_SIMPLEX;                              // choose a plain jane font
	double dblFontScale = (double)licPlate.imgPlate.rows / 30.0;            // base font scale on height of plate area
	int intFontThickness = (int)std::round(dblFontScale * 1.5);             // base font thickness on font scale
	int intBaseline = 0;

	cv::Size textSize = cv::getTextSize(licPlate.strChars, intFontFace, dblFontScale, intFontThickness, &intBaseline);      // call getTextSize

	ptCenterOfTextArea.x = (int)licPlate.rrLocationOfPlateInScene.center.x;         // the horizontal location of the text area is the same as the plate

	if (licPlate.rrLocationOfPlateInScene.center.y < (imgOriginalScene.rows * 0.75)) {      // if the license plate is in the upper 3/4 of the image
																							// write the chars in below the plate
		ptCenterOfTextArea.y = (int)std::round(licPlate.rrLocationOfPlateInScene.center.y) + (int)std::round((double)licPlate.imgPlate.rows * 1.6);
	}
	else {                                                                                // else if the license plate is in the lower 1/4 of the image
																						  // write the chars in above the plate
		ptCenterOfTextArea.y = (int)std::round(licPlate.rrLocationOfPlateInScene.center.y) - (int)std::round((double)licPlate.imgPlate.rows * 1.6);
	}

	ptLowerLeftTextOrigin.x = (int)(ptCenterOfTextArea.x - (textSize.width / 2));           // calculate the lower left origin of the text area
	ptLowerLeftTextOrigin.y = (int)(ptCenterOfTextArea.y + (textSize.height / 2));          // based on the text area center, width, and height

																							// write the text on the image
	cv::putText(imgOriginalScene, licPlate.strChars, ptLowerLeftTextOrigin, intFontFace, dblFontScale, SCALAR_YELLOW, intFontThickness);
}