
#include <cmath> // Esta librería contiene las funciones para realizar operaciones matemáticas (sin, cos, tan, log, exp, etc.)

// Cuando se carga la cabecer opencv.hpp automáticamente se cargan las demás cabeceras
//#include <opencv2/opencv.hpp>

#include <opencv2/core/core.hpp> // Contiene los elementos básicos como el objeto Mat (matriz que representa la imagen)
#include <opencv2/highgui/highgui.hpp> // Contiene los elementos para crear una interfaz gráfica básica
// OpenCV no está pensado para crear interfaces gráficas potentes. Se centra en la visión artificial y PDI. Si se desea crear una interfaz gráfica completa, se debe usar QT

#include <opencv2/imgcodecs/imgcodecs.hpp> // Contiene las funcionalidad para acceder a los códecs que permiten leer diferentes formatos de imagen (JPEG, JPEG-2000, PNG, TIFF, GIF, etc.)

// Librerías para acceder al video y para poder escribir vídeos en disco
#include <opencv2/video/video.hpp> 
#include <opencv2/videoio/videoio.hpp>

#include <opencv2/imgproc/imgproc.hpp> // Librería para realizar operaciones de PDI 

#include <ctime>
#include <iostream>

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
        
        int alpha_slider1 ;
		int alpha_slider2 ;
		int alpha_slider3 ;
		int alpha_slider4 ;
		int alpha_slider5 ;
		int alpha_slider6 ;
		int alpha_slider7 ;
		int alpha_slider8 ; 
		int alpha_slider9 ;
		int alpha_slider10;
		int alpha_slider11;
		int alpha_slider12;
//--------------------------------------------------------
//CascadeClassifier face_cascade;  // Para rostros
//CascadeClassifier eyes_cascade; // Para ojos
Mat frame, frameOut, handMask, foreground, fingerCountDebug;
//Prueba skinDetector, backgroundRemover, faceDetector, fingerCount;
//Prueba num;
int v=1;
void on_trackbarHmin( int, void*) {
    // Creamos el structring element (que puede ser una cruz, un rectángulo o una elipse):
    // MORPH_CROSS, MORPH_RECT, MORPH_ELLIPSE
    //Mat elemento = getStructuringElement(MORPH_CROSS, Size(alpha_slider+1,alpha_slider+1), Point(-1,-1));
    // Aplicamos la operación de dilatación
    //morphologyEx(imagen, frame, MORPH_DILATE,elemento); 
    //cvtColor(frame,frame,COLOR_BGR2HSV);

    cout << "trackbarHmin " << v << endl;   
}
void on_trackbarSmin( int, void *) {
    // Creamos el structring element (que puede ser una cruz, un rectángulo o una elipse):
    // MORPH_CROSS, MORPH_RECT, MORPH_ELLIPSE
    //Mat elemento = getStructuringElement(MORPH_CROSS, Size(alpha_slider+1,alpha_slider+1), Point(-1,-1));
    // Aplicamos la operación de dilatación
    //morphologyEx(imagen, frame, MORPH_DILATE,elemento);
    cout << "trackbarsmin " << v << endl; 
}
void on_trackbarVmin( int, void *) {
    // Creamos el structring element (que puede ser una cruz, un rectángulo o una elipse):
    // MORPH_CROSS, MORPH_RECT, MORPH_ELLIPSE
    //Mat elemento = getStructuringElement(MORPH_CROSS, Size(alpha_slider+1,alpha_slider+1), Point(-1,-1));
    // Aplicamos la operación de dilatación
    //morphologyEx(imagen, frame, MORPH_DILATE,elemento);
    cout << "trackbarvmin " << v << endl; 
    
}
void on_trackbarHmax( int, void *) {
    // Creamos el structring element (que puede ser una cruz, un rectángulo o una elipse):
    // MORPH_CROSS, MORPH_RECT, MORPH_ELLIPSE
    //Mat elemento = getStructuringElement(MORPH_CROSS, Size(alpha_slider+1,alpha_slider+1), Point(-1,-1));
    // Aplicamos la operación de dilatación
    //morphologyEx(imagen, frame, MORPH_DILATE,elemento); 
   
    cout << "trackbarHmax " << v << endl; 
}
void on_trackbarSmax( int, void *pP) {
    // Creamos el structring element (que puede ser una cruz, un rectángulo o una elipse):
    // MORPH_CROSS, MORPH_RECT, MORPH_ELLIPSE
    //Mat elemento = getStructuringElement(MORPH_CROSS, Size(alpha_slider+1,alpha_slider+1), Point(-1,-1));
    // Aplicamos la operación de dilatación
    //morphologyEx(imagen, frame, MORPH_DILATE,elemento);
    cout << "trackbarsmax " << v << endl; 
}
void on_trackbarVmax( int, void *) {
    // Creamos el structring element (que puede ser una cruz, un rectángulo o una elipse):
    // MORPH_CROSS, MORPH_RECT, MORPH_ELLIPSE
    //Mat elemento = getStructuringElement(MORPH_CROSS, Size(alpha_slider+1,alpha_slider+1), Point(-1,-1));
    // Aplicamos la operación de dilatación
    //morphologyEx(imagen, frame, MORPH_DILATE,elemento);
     
    cout << "trackbarvmax " << v << endl; 
}
void on_trackbarYmin( int, void *) {
    // Creamos el structring element (que puede ser una cruz, un rectángulo o una elipse):
    // MORPH_CROSS, MORPH_RECT, MORPH_ELLIPSE
    //Mat elemento = getStructuringElement(MORPH_CROSS, Size(alpha_slider+1,alpha_slider+1), Point(-1,-1));
    // Aplicamos la operación de dilatación
    //morphologyEx(imagen, frame, MORPH_DILATE,elemento);
    cout << "on_trackbarYmin " << v << endl; 
}    
void on_trackbarCrmin( int, void *) {
    // Creamos el structring element (que puede ser una cruz, un rectángulo o una elipse):
    // MORPH_CROSS, MORPH_RECT, MORPH_ELLIPSE
    //Mat elemento = getStructuringElement(MORPH_CROSS, Size(alpha_slider+1,alpha_slider+1), Point(-1,-1));
    // Aplicamos la operación de dilatación
    //morphologyEx(imagen, frame, MORPH_DILATE,elemento);
    cout << "on_trackbarcrmin " << v << endl; 
}
void on_trackbarCbmin( int, void *) {
    // Creamos el structring element (que puede ser una cruz, un rectángulo o una elipse):
    // MORPH_CROSS, MORPH_RECT, MORPH_ELLIPSE
    //Mat elemento = getStructuringElement(MORPH_CROSS, Size(alpha_slider+1,alpha_slider+1), Point(-1,-1));
    // Aplicamos la operación de dilatación
    //morphologyEx(imagen, frame, MORPH_DILATE,elemento);
    cout << "on_trackbarcbmin " << v << endl; 
    
}
void on_trackbarYmax( int, void *) {
    // Creamos el structring element (que puede ser una cruz, un rectángulo o una elipse):
    // MORPH_CROSS, MORPH_RECT, MORPH_ELLIPSE
    //Mat elemento = getStructuringElement(MORPH_CROSS, Size(alpha_slider+1,alpha_slider+1), Point(-1,-1));
    // Aplicamos la operación de dilatación
    //morphologyEx(imagen, frame, MORPH_DILATE,elemento);    
    cout << "on_trackbarYmax " << v << endl; 
}
void on_trackbarCrmax( int, void *) {
    // Creamos el structring element (que puede ser una cruz, un rectángulo o una elipse):
    // MORPH_CROSS, MORPH_RECT, MORPH_ELLIPSE
    //Mat elemento = getStructuringElement(MORPH_CROSS, Size(alpha_slider+1,alpha_slider+1), Point(-1,-1));
    // Aplicamos la operación de dilatación
    //morphologyEx(imagen, frame, MORPH_DILATE,elemento);
    cout << "on_trackbarcrmax " << v << endl; 
}
void on_trackbarCbmax( int, void *) {

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
        //Prueba detector;
        
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


            /*detector.drawSkinColorSampler(frameOut);
            foreground = detector.getForeground(frame);
            detector.removeFaces(frame, foreground);
            handMask = detector.getSkinMask(foreground);
            fingerCountDebug = detector.findFingersCount(handMask, frameOut); */
           
            
            imshow("handMask", handMask);
            imshow("handDetection", fingerCountDebug);
            imshow("output", frameOut);
            imshow("foreground", foreground);

            int key = waitKey(1);

            if (key == 27) // esc
                break;
            //else if (key == 98) // b
               // detector.calibrate1(frame);
            //else if (key == 115) // s
              //  detector.calibrate(frame);
        }

        videoCapture.release();
        destroyAllWindows();
        return 0;
        
    }

}
