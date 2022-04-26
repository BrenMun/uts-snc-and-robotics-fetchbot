#include <string>
#include <sstream>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

void on_trackbar( int, void* ) {}
std::string trackbarWindowName = "Trackbar";
int hsvMinMax[6] = {0,256,0,256,0,256};

void createTrackbars(){
	//create window for trackbars
  cv::namedWindow(trackbarWindowName,0);

  //HSV value names shown on trackbar
  std::string hsv_string[6] = {"H_MIN", "H_MAX", "S_MIN", "S_MAX", "V_MIN", "V_MAX"};

  //create memory to store trackbar name on window
  for (int i=0; i<6; i++){
      std::stringstream trackbarName;
      trackbarName << hsv_string[i] << hsvMinMax[i];
  }
  //trackbar for min hsv values
  for (int i=0; i<5; i+=2)
    cv::createTrackbar(hsv_string[i], trackbarWindowName, &hsvMinMax[i], hsvMinMax[i+1], on_trackbar);
  //trackbar for max hsv values
  for (int i=1; i<6; i+=2)
    cv::createTrackbar(hsv_string[i], trackbarWindowName, &hsvMinMax[i], hsvMinMax[i], on_trackbar);
}

cv::Mat getIsolatedObject(cv::Mat hsv){
  cv::Mat threshold;
  while(true){
    //filter HSV image between values and store filtered image to threshold matrix
    inRange(
      hsv,
      cv::Scalar(hsvMinMax[0],hsvMinMax[2],hsvMinMax[4]), //hsv min
      cv::Scalar(hsvMinMax[1],hsvMinMax[3],hsvMinMax[5]), //hsv max
      threshold
    );
    imshow("threshold image",threshold);
    if (cv::waitKey(30) >= 0) break;
  }    
  cv::destroyAllWindows();
  return threshold;
}