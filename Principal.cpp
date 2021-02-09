#include "Principal.hpp"

using namespace cv;
using namespace std;

//--------------------------------------------------------
        const int alpha_slider_max1 = 179;
		const int alpha_slider_max2 = 255;
		const int alpha_slider_max3 = 255;
		const int alpha_slider_max4 = 179;
		const int alpha_slider_max5 = 255;
		const int alpha_slider_max6 = 255;
		const int alpha_slider_max7 = 100;
		const int alpha_slider_max8 = 100;
		const int alpha_slider_max9 = 100;
		const int alpha_slider_max10 = 100;
		const int alpha_slider_max11 = 100;
		const int alpha_slider_max12 = 100;
		int alpha_slider1 = 0;
		int alpha_slider2 = 0;
		int alpha_slider3 = 0;
		int alpha_slider4 = 0;
		int alpha_slider5 = 0;
		int alpha_slider6 = 0;
		int alpha_slider7 = 0;
		int alpha_slider8 = 0; 
		int alpha_slider9 = 0;
		int alpha_slider10 = 0;
		int alpha_slider11 = 0;
		int alpha_slider12 = 0;
		double alpha1;
		double alpha2;
		double alpha3;
		double alpha4;
		double alpha5;
		double alpha6;
		double alpha7;
		double alpha8;
		double alpha9;
		double alpha10;
		double alpha11;
		double alpha12;
//--------------------------------------------------------
//CascadeClassifier face_cascade;  // Para rostros
//CascadeClassifier eyes_cascade; // Para ojos
Mat frame, frameOut, handMask, foreground, fingerCountDebug;
Prueba skinDetector, backgroundRemover, faceDetector, fingerCount;
Prueba num;

void on_trackbarHmin( int v, void*) {
    // Creamos el structring element (que puede ser una cruz, un rectángulo o una elipse):
    // MORPH_CROSS, MORPH_RECT, MORPH_ELLIPSE
    //Mat elemento = getStructuringElement(MORPH_CROSS, Size(alpha_slider+1,alpha_slider+1), Point(-1,-1));
    // Aplicamos la operación de dilatación
    //morphologyEx(imagen, frame, MORPH_DILATE,elemento); 
    cvtColor(frame,frame,COLOR_BGR2HSV);

    cout << "trackbarHmin " << v << endl;   
}
void on_trackbarSmin( int v, void *) {
    // Creamos el structring element (que puede ser una cruz, un rectángulo o una elipse):
    // MORPH_CROSS, MORPH_RECT, MORPH_ELLIPSE
    //Mat elemento = getStructuringElement(MORPH_CROSS, Size(alpha_slider+1,alpha_slider+1), Point(-1,-1));
    // Aplicamos la operación de dilatación
    //morphologyEx(imagen, frame, MORPH_DILATE,elemento);
    cout << "trackbarsmin " << v << endl; 
}
void on_trackbarVmin( int v, void *) {
    // Creamos el structring element (que puede ser una cruz, un rectángulo o una elipse):
    // MORPH_CROSS, MORPH_RECT, MORPH_ELLIPSE
    //Mat elemento = getStructuringElement(MORPH_CROSS, Size(alpha_slider+1,alpha_slider+1), Point(-1,-1));
    // Aplicamos la operación de dilatación
    //morphologyEx(imagen, frame, MORPH_DILATE,elemento);
    cout << "trackbarvmin " << v << endl; 
    
}
void on_trackbarHmax( int v, void *) {
    // Creamos el structring element (que puede ser una cruz, un rectángulo o una elipse):
    // MORPH_CROSS, MORPH_RECT, MORPH_ELLIPSE
    //Mat elemento = getStructuringElement(MORPH_CROSS, Size(alpha_slider+1,alpha_slider+1), Point(-1,-1));
    // Aplicamos la operación de dilatación
    //morphologyEx(imagen, frame, MORPH_DILATE,elemento); 
   
    cout << "trackbarHmax " << v << endl; 
}
void on_trackbarSmax( int v, void *pP) {
    // Creamos el structring element (que puede ser una cruz, un rectángulo o una elipse):
    // MORPH_CROSS, MORPH_RECT, MORPH_ELLIPSE
    //Mat elemento = getStructuringElement(MORPH_CROSS, Size(alpha_slider+1,alpha_slider+1), Point(-1,-1));
    // Aplicamos la operación de dilatación
    //morphologyEx(imagen, frame, MORPH_DILATE,elemento);
    cout << "trackbarsmax " << v << endl; 
}
void on_trackbarVmax( int v, void *) {
    // Creamos el structring element (que puede ser una cruz, un rectángulo o una elipse):
    // MORPH_CROSS, MORPH_RECT, MORPH_ELLIPSE
    //Mat elemento = getStructuringElement(MORPH_CROSS, Size(alpha_slider+1,alpha_slider+1), Point(-1,-1));
    // Aplicamos la operación de dilatación
    //morphologyEx(imagen, frame, MORPH_DILATE,elemento);
     
    cout << "trackbarvmax " << v << endl; 
}
void on_trackbarYmin( int v, void *) {
    // Creamos el structring element (que puede ser una cruz, un rectángulo o una elipse):
    // MORPH_CROSS, MORPH_RECT, MORPH_ELLIPSE
    //Mat elemento = getStructuringElement(MORPH_CROSS, Size(alpha_slider+1,alpha_slider+1), Point(-1,-1));
    // Aplicamos la operación de dilatación
    //morphologyEx(imagen, frame, MORPH_DILATE,elemento);
    cout << "on_trackbarYmin " << v << endl; 
}    
void on_trackbarCrmin( int v, void *) {
    // Creamos el structring element (que puede ser una cruz, un rectángulo o una elipse):
    // MORPH_CROSS, MORPH_RECT, MORPH_ELLIPSE
    //Mat elemento = getStructuringElement(MORPH_CROSS, Size(alpha_slider+1,alpha_slider+1), Point(-1,-1));
    // Aplicamos la operación de dilatación
    //morphologyEx(imagen, frame, MORPH_DILATE,elemento);
    cout << "on_trackbarcrmin " << v << endl; 
}
void on_trackbarCbmin( int v, void *) {
    // Creamos el structring element (que puede ser una cruz, un rectángulo o una elipse):
    // MORPH_CROSS, MORPH_RECT, MORPH_ELLIPSE
    //Mat elemento = getStructuringElement(MORPH_CROSS, Size(alpha_slider+1,alpha_slider+1), Point(-1,-1));
    // Aplicamos la operación de dilatación
    //morphologyEx(imagen, frame, MORPH_DILATE,elemento);
    cout << "on_trackbarcbmin " << v << endl; 
    
}
void on_trackbarYmax( int v, void *) {
    // Creamos el structring element (que puede ser una cruz, un rectángulo o una elipse):
    // MORPH_CROSS, MORPH_RECT, MORPH_ELLIPSE
    //Mat elemento = getStructuringElement(MORPH_CROSS, Size(alpha_slider+1,alpha_slider+1), Point(-1,-1));
    // Aplicamos la operación de dilatación
    //morphologyEx(imagen, frame, MORPH_DILATE,elemento);    
    cout << "on_trackbarYmax " << v << endl; 
}
void on_trackbarCrmax( int v, void *) {
    // Creamos el structring element (que puede ser una cruz, un rectángulo o una elipse):
    // MORPH_CROSS, MORPH_RECT, MORPH_ELLIPSE
    //Mat elemento = getStructuringElement(MORPH_CROSS, Size(alpha_slider+1,alpha_slider+1), Point(-1,-1));
    // Aplicamos la operación de dilatación
    //morphologyEx(imagen, frame, MORPH_DILATE,elemento);
    cout << "on_trackbarcrmax " << v << endl; 
}
void on_trackbarCbmax( int v, void *) {

    // Creamos el structring element (que puede ser una cruz, un rectángulo o una elipse):
    // MORPH_CROSS, MORPH_RECT, MORPH_ELLIPSE
    //Mat elemento = getStructuringElement(MORPH_CROSS, Size(alpha_slider+1,alpha_slider+1), Point(-1,-1));
    // Aplicamos la operación de dilatación
    //morphologyEx(imagen, frame, MORPH_DILATE,elemento);
    cout << "on_trackbarcbmax " << v << endl; 
    
}



int main(int, char**) {


	VideoCapture videoCapture(0);


    namedWindow("Original", WINDOW_AUTOSIZE);
    namedWindow("output", WINDOW_AUTOSIZE);
    namedWindow("foreground", WINDOW_AUTOSIZE);
    namedWindow("handMask", WINDOW_AUTOSIZE);
    namedWindow("handDetection", WINDOW_AUTOSIZE);
    
     alpha_slider1 = 0;
     alpha_slider2 = 0;
     alpha_slider3 = 0;
     alpha_slider4 = 0;
     alpha_slider5 = 0;
     alpha_slider6 = 0;
     alpha_slider7 = 0;
     alpha_slider8 = 0;
     alpha_slider9 = 0;
     alpha_slider10 = 0;
     alpha_slider11 = 0;
     alpha_slider12 = 0;

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

    sprintf(TrackbarName1, "H-Min %d",  alpha_slider_max1);
    sprintf(TrackbarName2, "S-Min %d",  alpha_slider_max2);
    sprintf(TrackbarName3, "V-Min %d",  alpha_slider_max3);
    sprintf(TrackbarName4, "H-Max %d",  alpha_slider_max4);
    sprintf(TrackbarName5, "S-Max %d",  alpha_slider_max5);
    sprintf(TrackbarName6, "V-Max %d",  alpha_slider_max6);
    sprintf(TrackbarName7, "Y-Min %d",  alpha_slider_max7);
    sprintf(TrackbarName8, "Cr-Min %d",  alpha_slider_max8);
    sprintf(TrackbarName9, "Cb-Min %d",  alpha_slider_max9);
    sprintf(TrackbarName10, "Y-Max %d",  alpha_slider_max10);
    sprintf(TrackbarName11, "Cr-Max %d",  alpha_slider_max11);
    sprintf(TrackbarName11, "Cb-Max %d",  alpha_slider_max12);

    createTrackbar(TrackbarName1, "Original", &alpha_slider1,  alpha_slider_max1,  on_trackbarHmin);
    createTrackbar(TrackbarName2, "Original", &alpha_slider2,  alpha_slider_max2,  on_trackbarSmin);
    createTrackbar(TrackbarName3, "Original", &alpha_slider3,  alpha_slider_max3,  on_trackbarVmin);
    createTrackbar(TrackbarName4, "Original", &alpha_slider4,  alpha_slider_max4,  on_trackbarHmax);
    createTrackbar(TrackbarName5, "Original", &alpha_slider5,  alpha_slider_max5,  on_trackbarSmax);
    createTrackbar(TrackbarName6, "Original", &alpha_slider6,  alpha_slider_max6,  on_trackbarVmax);
    createTrackbar(TrackbarName7, "Original", &alpha_slider7,  alpha_slider_max7,  on_trackbarYmin);
    createTrackbar(TrackbarName8, "Original", &alpha_slider8,  alpha_slider_max8,  on_trackbarCrmin);
    createTrackbar(TrackbarName9, "Original", &alpha_slider9,  alpha_slider_max9,  on_trackbarCbmin);
    createTrackbar(TrackbarName10, "Original", &alpha_slider10,  alpha_slider_max10,  on_trackbarYmax);
    createTrackbar(TrackbarName11, "Original", &alpha_slider11,  alpha_slider_max11,  on_trackbarCrmax);
    createTrackbar(TrackbarName12, "Original", &alpha_slider12,  alpha_slider_max12,  on_trackbarCbmax);

    if(videoCapture.isOpened()){
        Prueba detector;
        
        while (3==3) {
            videoCapture >> frame;

            frameOut = frame.clone();

            
             on_trackbarHmin( alpha_slider1, 0);
             on_trackbarSmin( alpha_slider2, 0);
             on_trackbarVmin( alpha_slider3, 0);
             on_trackbarHmax( alpha_slider4, 0);
             on_trackbarSmax( alpha_slider5, 0);
             on_trackbarVmax( alpha_slider6, 0);
             on_trackbarYmin( alpha_slider7, 0);
             on_trackbarCrmin( alpha_slider8, 0);
             on_trackbarCbmin( alpha_slider9, 0);
             on_trackbarYmax( alpha_slider10, 0);
             on_trackbarCrmax( alpha_slider11, 0);
             on_trackbarCbmax( alpha_slider12, 0);
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
