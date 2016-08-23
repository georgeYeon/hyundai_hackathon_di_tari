#include <baseapi.h>

#include <allheaders.h>

#include <opencv2\opencv.hpp>

#include <opencv2/core/core.hpp> 

#include "OCR.h"

static tesseract::TessBaseAPI api;

static const char* NONUMBER = "no number";


void OCRinit(){
	api.Init(NULL, "eng", tesseract::OEM_DEFAULT);
	api.SetPageSegMode(tesseract::PSM_SINGLE_BLOCK);

	// read digits only 
	api.SetVariable("tessedit_char_whitelist", "0123456789");
}

void DoOCR(Mat sub, string &out)

{
	
	api.SetImage((uchar*)sub.data, sub.size().width, sub.size().height, sub.channels(), sub.step1());
	api.Recognize(0);
	out = api.GetUTF8Text();

	bool IsNumber = false;
	int COUNT = 0;
	for (int i = 0; i < out.size(); i++) {
		if ('0' <= out[i] && out[i]<= '9' && COUNT < 4) {
			COUNT++;
		}
		else if (COUNT == 4) {
			IsNumber = true;
		}
		else if (8 < COUNT) {
			IsNumber = false;
		}
		else if ('0' <= out[i] && out[i] <= '9') {
			COUNT++;
		}
		else {
			IsNumber = false;
			COUNT = 0;
		}
	}
	if (IsNumber);
	else {
	//	printf("%d\n", -1);
		out.clear();
	}
	api.Clear();
}

void OCRend() {
	api.End();
}