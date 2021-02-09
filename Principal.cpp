#include "Principal.hpp"

using namespace cv;
using namespace std;
//CascadeClassifier face_cascade;  // Para rostros
//CascadeClassifier eyes_cascade; // Para ojos
Mat frame, frameOut, handMask, foreground, fingerCountDebug;
Prueba skinDetector, backgroundRemover, faceDetector, fingerCount;
Prueba num;

void on_trackbarHmin( int v, void *pP) {
    // Creamos el structring element (que puede ser una cruz, un rectángulo o una elipse):
    // MORPH_CROSS, MORPH_RECT, MORPH_ELLIPSE
    //Mat elemento = getStructuringElement(MORPH_CROSS, Size(alpha_slider+1,alpha_slider+1), Point(-1,-1));
    // Aplicamos la operación de dilatación
    //morphologyEx(imagen, frame, MORPH_DILATE,elemento); 
    cvtColor(frame,COLOR_BGR2HSV);

    cout << "trackbarHmin " << v << endl;   
}

int main(int, char**) {


	VideoCapture videoCapture(0);


    namedWindow("Original", WINDOW_AUTOSIZE);
    namedWindow("output", WINDOW_AUTOSIZE);
    namedWindow("foreground", WINDOW_AUTOSIZE);
    namedWindow("handMask", WINDOW_AUTOSIZE);
    namedWindow("handDetection", WINDOW_AUTOSIZE);
    
    num.alpha_slider1 = 0;
    num.alpha_slider2 = 0;
    num.alpha_slider3 = 0;
    num.alpha_slider4 = 0;
    num.alpha_slider5 = 0;
    num.alpha_slider6 = 0;
    num.alpha_slider7 = 0;
    num.alpha_slider8 = 0;
    num.alpha_slider9 = 0;
    num.alpha_slider10 = 0;
    num.alpha_slider11 = 0;
    num.alpha_slider12 = 0;

    char TrackbarName1[50];
    char TrackbarName2[50];
    char TrackbarName3[50];
    char TrackbarName4[50];
    char TrackbarName5[50];
    char TrackbarName6[50];
    char TrackbarName7[50];
    char TrackbarName8[50];
    char TrackbarName9[50];
    char TrackbarName10[50];
    char TrackbarName11[50];
    char TrackbarName12[50];

    sprintf(TrackbarName1, "H-Min %d", num.alpha_slider_max1);
    sprintf(TrackbarName2, "S-Min %d", num.alpha_slider_max2);
    sprintf(TrackbarName3, "V-Min %d", num.alpha_slider_max3);
    sprintf(TrackbarName4, "H-Max %d", num.alpha_slider_max4);
    sprintf(TrackbarName5, "S-Max %d", num.alpha_slider_max5);
    sprintf(TrackbarName6, "V-Max %d", num.alpha_slider_max6);
    sprintf(TrackbarName7, "Y-Min %d", num.alpha_slider_max7);
    sprintf(TrackbarName8, "Cr-Min %d", num.alpha_slider_max8);
    sprintf(TrackbarName9, "Cb-Min %d", num.alpha_slider_max9);
    sprintf(TrackbarName10, "Y-Max %d", num.alpha_slider_max10);
    sprintf(TrackbarName11, "Cr-Max %d", num.alpha_slider_max11);
    sprintf(TrackbarName11, "Cb-Max %d", num.alpha_slider_max12);

    createTrackbar(TrackbarName1, "Original", &num.alpha_slider1, num.alpha_slider_max1, num.on_trackbarHmin);
    createTrackbar(TrackbarName2, "Original", &num.alpha_slider2, num.alpha_slider_max2, num.on_trackbarSmin);
    createTrackbar(TrackbarName3, "Original", &num.alpha_slider3, num.alpha_slider_max3, num.on_trackbarVmin);
    createTrackbar(TrackbarName4, "Original", &num.alpha_slider4, num.alpha_slider_max4, num.on_trackbarHmax);
    createTrackbar(TrackbarName5, "Original", &num.alpha_slider5, num.alpha_slider_max5, num.on_trackbarSmax);
    createTrackbar(TrackbarName6, "Original", &num.alpha_slider6, num.alpha_slider_max6, num.on_trackbarVmax);
    createTrackbar(TrackbarName7, "Original", &num.alpha_slider7, num.alpha_slider_max7, num.on_trackbarYmin);
    createTrackbar(TrackbarName8, "Original", &num.alpha_slider8, num.alpha_slider_max8, num.on_trackbarCrmin);
    createTrackbar(TrackbarName9, "Original", &num.alpha_slider9, num.alpha_slider_max9, num.on_trackbarCbmin);
    createTrackbar(TrackbarName10, "Original", &num.alpha_slider10, num.alpha_slider_max10, num.on_trackbarYmax);
    createTrackbar(TrackbarName11, "Original", &num.alpha_slider11, num.alpha_slider_max11, num.on_trackbarCrmax);
    createTrackbar(TrackbarName12, "Original", &num.alpha_slider12, num.alpha_slider_max12, num.on_trackbarCbmax);

    if(videoCapture.isOpened()){
        Prueba detector;
        
        while (3==3) {
            videoCapture >> frame;

            frameOut = frame.clone();

            
            num.on_trackbarHmin(num.alpha_slider1, 0);
            num.on_trackbarSmin(num.alpha_slider2, 0);
            num.on_trackbarVmin(num.alpha_slider3, 0);
            num.on_trackbarHmax(num.alpha_slider4, 0);
            num.on_trackbarSmax(num.alpha_slider5, 0);
            num.on_trackbarVmax(num.alpha_slider6, 0);
            num.on_trackbarYmin(num.alpha_slider7, 0);
            num.on_trackbarCrmin(num.alpha_slider8, 0);
            num.on_trackbarCbmin(num.alpha_slider9, 0);
            num.on_trackbarYmax(num.alpha_slider10, 0);
            num.on_trackbarCrmax(num.alpha_slider11, 0);
            num.on_trackbarCbmax(num.alpha_slider12, 0);
            imshow("Original", frame);


            detector.drawSkinColorSampler(frameOut);
            foreground = detector.getForeground(frame);
            detector.removeFaces(frame, foreground);
            handMask = detector.getSkinMask(foreground);
            fingerCountDebug = detector.findFingersCount(handMask, frameOut); 
           
            
            imshow("handMask", handMask);
            imshow("handDetection", fingerCountDebug);
            imshow("output", frameOut);
            imshow("foreground", foreground);

            int key = waitKey(1);

            if (key == 27) // esc
                break;
            else if (key == 98) // b
                detector.calibrate1(frame);
            else if (key == 115) // s
                detector.calibrate(frame);
        }

        videoCapture.release();
        destroyAllWindows();
        return 0;
        
    }

}
