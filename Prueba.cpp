#include "Principal.hpp"
//#include"opencv2\opencv.hpp"

Rect getFaceRect(Mat input);

String faceClassifierFileName = "src/haarcascade_frontalface_alt.xml";
CascadeClassifier faceCascadeClassifier;

#define LIMIT_ANGLE_SUP 60
#define LIMIT_ANGLE_INF 5
#define BOUNDING_RECT_FINGER_SIZE_SCALING 0.3
#define BOUNDING_RECT_NEIGHBOR_DISTANCE_SCALING 0.05

Prueba::Prueba(void) {
    
    //SKINDETECTOR
	hLowThreshold = 0;
	hHighThreshold = 0;
	sLowThreshold = 0;
	sHighThreshold = 0;
	vLowThreshold = 0;
	vHighThreshold = 0;
	calibrated = false;
    calibrated1 = false;
	skinColorSamplerRectangle1, skinColorSamplerRectangle2;

    //FACEDETECTOR
    if (!faceCascadeClassifier.load(faceClassifierFileName))
		throw runtime_error("can't load file " + faceClassifierFileName);

    //BACKGROUND
    background;

    //FINGERCOUNT
    color_blue = Scalar(255, 0, 0);
	color_green = Scalar(0, 255, 0);
	color_red = Scalar(0, 0, 255);
	color_black = Scalar(0, 0, 0);
	color_white = Scalar(255, 255, 255);
	color_yellow = Scalar(0, 255, 255);
	color_purple = Scalar(255, 0, 255);
}

// Esto es SKINDETECTOR

void Prueba::drawSkinColorSampler(Mat input) {
	int frameWidth = input.size().width, frameHeight = input.size().height;

	int rectangleSize = 20;
	Scalar rectangleColor = Scalar(255, 0, 255);

	skinColorSamplerRectangle1 = Rect(frameWidth / 5, frameHeight / 2, rectangleSize, rectangleSize);
	skinColorSamplerRectangle2 = Rect(frameWidth / 5, frameHeight / 3, rectangleSize, rectangleSize);

	rectangle(
		input,
		skinColorSamplerRectangle1,
		rectangleColor
	);

	rectangle(
		input,
		skinColorSamplerRectangle2,
		rectangleColor
	);
}

void Prueba::calibrate(Mat input) {
	
	Mat hsvInput;
	cvtColor(input, hsvInput, CV_BGR2HSV);

	Mat sample1 = Mat(hsvInput, skinColorSamplerRectangle1);
	Mat sample2 = Mat(hsvInput, skinColorSamplerRectangle2);

	calculateThresholds(sample1, sample2);

	calibrated = true;
}

void Prueba::calculateThresholds(Mat sample1, Mat sample2) {
	int offsetLowThreshold = 80;
	int offsetHighThreshold = 30;

	Scalar hsvMeansSample1 = mean(sample1);
	Scalar hsvMeansSample2 = mean(sample2);

	hLowThreshold = min(hsvMeansSample1[0], hsvMeansSample2[0]) - offsetLowThreshold;
	hHighThreshold = max(hsvMeansSample1[0], hsvMeansSample2[0]) + offsetHighThreshold;

	sLowThreshold = min(hsvMeansSample1[1], hsvMeansSample2[1]) - offsetLowThreshold;
	sHighThreshold = max(hsvMeansSample1[1], hsvMeansSample2[1]) + offsetHighThreshold;

	// the V channel shouldn't be used. By ignorint it, shadows on the hand wouldn't interfire with segmentation.
	// Unfortunately there's a bug somewhere and not using the V channel causes some problem. This shouldn't be too hard to fix.
	vLowThreshold = min(hsvMeansSample1[2], hsvMeansSample2[2]) - offsetLowThreshold;
	vHighThreshold = max(hsvMeansSample1[2], hsvMeansSample2[2]) + offsetHighThreshold;
	//vLowThreshold = 0;
	//vHighThreshold = 255;
}

Mat Prueba::getSkinMask(Mat input) {
	Mat skinMask;

	if (!calibrated) {
		skinMask = Mat::zeros(input.size(), CV_8UC1);
		return skinMask;
	}

	Mat hsvInput;
	cvtColor(input, hsvInput, CV_BGR2HSV);

	inRange(
		hsvInput,
		Scalar(hLowThreshold, sLowThreshold, vLowThreshold),
		Scalar(hHighThreshold, sHighThreshold, vHighThreshold),
		skinMask);

	performOpening(skinMask, MORPH_ELLIPSE, { 3, 3 });
	dilate(skinMask, skinMask, Mat(), Point(-1, -1), 3);

	return skinMask;
}

void Prueba::performOpening(Mat binaryImage, int kernelShape, Point kernelSize) {
	Mat structuringElement = getStructuringElement(kernelShape, kernelSize);
	morphologyEx(binaryImage, binaryImage, MORPH_OPEN, structuringElement);
}

// Esto es BACKGROUNDREMOVER

void Prueba::calibrate1(Mat input) {
	cvtColor(input, background, CV_BGR2GRAY);
	calibrated1 = true;
}

Mat Prueba::getForeground(Mat input) {
	Mat foregroundMask = getForegroundMask(input);

	//imshow("foregroundMask", foregroundMask);

	Mat foreground;
	input.copyTo(foreground, foregroundMask);

	return foreground;
}

Mat Prueba::getForegroundMask(Mat input) {
	Mat foregroundMask;

	if (!calibrated1) {
		foregroundMask = Mat::zeros(input.size(), CV_8UC1);
		return foregroundMask;
	}

	cvtColor(input, foregroundMask, CV_BGR2GRAY);

	removeBackground(foregroundMask, background);
	
	return foregroundMask;
}

void Prueba::removeBackground(Mat input, Mat background) {
	int thresholdOffset = 10;

	for (int i = 0; i < input.rows; i++) {
		for (int j = 0; j < input.cols; j++) {
			uchar framePixel = input.at<uchar>(i, j);
			uchar bgPixel = background.at<uchar>(i, j);

			if (framePixel >= bgPixel - thresholdOffset && framePixel <= bgPixel + thresholdOffset)
				input.at<uchar>(i, j) = 0;
			else
				input.at<uchar>(i, j) = 255;
		}
	}
}

// Esto es FACEDETECTOR

void Prueba::removeFaces(Mat input, Mat output) {
	vector<Rect> faces;
	Mat frameGray;

	cvtColor(input, frameGray, CV_BGR2GRAY);
	equalizeHist(frameGray, frameGray);

	faceCascadeClassifier.detectMultiScale(frameGray, faces, 1.1, 2, 0 | CV_HAAR_SCALE_IMAGE, Size(120, 120));

	for (size_t i = 0; i < faces.size(); i++) {
		rectangle(
			output,
			Point(faces[i].x, faces[i].y),
			Point(faces[i].x + faces[i].width, faces[i].y + faces[i].height),
			Scalar(0, 0, 0),
			-1
		);
	}
}

Rect getFaceRect(Mat input) {
	vector<Rect> faceRectangles;
	Mat inputGray;

	cvtColor(input, inputGray, CV_BGR2GRAY);
	equalizeHist(inputGray, inputGray);

	faceCascadeClassifier.detectMultiScale(inputGray, faceRectangles, 1.1, 2, 0 | CV_HAAR_SCALE_IMAGE, Size(120, 120));

	if (faceRectangles.size() > 0)
		return faceRectangles[0];
	else
		return Rect(0, 0, 1, 1);
}

// Esto es FINGERCOUNT

Mat Prueba::findFingersCount(Mat input_image, Mat frame) {
	Mat contours_image = Mat::zeros(input_image.size(), CV_8UC3);

	// check if the source image is good
	if (input_image.empty())
		return contours_image;

	// we work only on the 1 channel result, since this function is called inside a loop we are not sure that this is always the case
	if (input_image.channels() != 1)
		return contours_image;

	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;

	findContours(input_image, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

	// we need at least one contour to work
	if (contours.size() <= 0)
		return contours_image;

	// find the biggest contour (let's suppose it's our hand)
	int biggest_contour_index = -1;
	double biggest_area = 0.0;

	for (int i = 0; i < contours.size(); i++) {
		double area = contourArea(contours[i], false);
		if (area > biggest_area) {
			biggest_area = area;
			biggest_contour_index = i;
		}
	}

	if (biggest_contour_index < 0)
		return contours_image;

	// find the convex hull object for each contour and the defects, two different data structure are needed by the OpenCV api
	vector<Point> hull_points;
	vector<int> hull_ints;

	// for drawing the convex hull and for finding the bounding rectangle
	convexHull(Mat(contours[biggest_contour_index]), hull_points, true);

	// for finding the defects
	convexHull(Mat(contours[biggest_contour_index]), hull_ints, false);

	// we need at least 3 points to find the defects
	vector<Vec4i> defects;
	if (hull_ints.size() > 3)
		convexityDefects(Mat(contours[biggest_contour_index]), hull_ints, defects);
	else
		return contours_image;

	// we bound the convex hull
	Rect bounding_rectangle = boundingRect(Mat(hull_points));

	// we find the center of the bounding rectangle, this should approximately also be the center of the hand
	Point center_bounding_rect(
		(bounding_rectangle.tl().x + bounding_rectangle.br().x) / 2,
		(bounding_rectangle.tl().y + bounding_rectangle.br().y) / 2
	);

	// we separate the defects keeping only the ones of intrest
	vector<Point> start_points;
	vector<Point> far_points;

	for (int i = 0; i < defects.size(); i++) {
		start_points.push_back(contours[biggest_contour_index][defects[i].val[0]]);

		// filtering the far point based on the distance from the center of the bounding rectangle
		if (findPointsDistance(contours[biggest_contour_index][defects[i].val[2]], center_bounding_rect) < bounding_rectangle.height * BOUNDING_RECT_FINGER_SIZE_SCALING)
			far_points.push_back(contours[biggest_contour_index][defects[i].val[2]]);
	}

	// we compact them on their medians
	vector<Point> filtered_start_points = compactOnNeighborhoodMedian(start_points, bounding_rectangle.height * BOUNDING_RECT_NEIGHBOR_DISTANCE_SCALING);
	vector<Point> filtered_far_points = compactOnNeighborhoodMedian(far_points, bounding_rectangle.height * BOUNDING_RECT_NEIGHBOR_DISTANCE_SCALING);

	// now we try to find the fingers
	vector<Point> filtered_finger_points;

	if (filtered_far_points.size() > 1) {
		vector<Point> finger_points;
		
		for (int i = 0; i < filtered_start_points.size(); i++) {
			vector<Point> closest_points = findClosestOnX(filtered_far_points, filtered_start_points[i]);
			
			if (isFinger(closest_points[0], filtered_start_points[i], closest_points[1], LIMIT_ANGLE_INF, LIMIT_ANGLE_SUP, center_bounding_rect, bounding_rectangle.height * BOUNDING_RECT_FINGER_SIZE_SCALING))
				finger_points.push_back(filtered_start_points[i]);
		}

		if (finger_points.size() > 0) {

			// we have at most five fingers usually :)
			while (finger_points.size() > 5)
				finger_points.pop_back();

			// filter out the points too close to each other
			for (int i = 0; i < finger_points.size() - 1; i++) {
				if (findPointsDistanceOnX(finger_points[i], finger_points[i + 1]) > bounding_rectangle.height * BOUNDING_RECT_NEIGHBOR_DISTANCE_SCALING * 1.5)
					filtered_finger_points.push_back(finger_points[i]);
			}

			if (finger_points.size() > 2) {
				if (findPointsDistanceOnX(finger_points[0], finger_points[finger_points.size() - 1]) > bounding_rectangle.height * BOUNDING_RECT_NEIGHBOR_DISTANCE_SCALING * 1.5)
					filtered_finger_points.push_back(finger_points[finger_points.size() - 1]);
			}
			else
				filtered_finger_points.push_back(finger_points[finger_points.size() - 1]);
		}
	}
	
	// we draw what found on the returned image 
	drawContours(contours_image, contours, biggest_contour_index, color_green, 2, 8, hierarchy);
	polylines(contours_image, hull_points, true, color_blue);
	rectangle(contours_image, bounding_rectangle.tl(), bounding_rectangle.br(), color_red, 2, 8, 0);
	circle(contours_image, center_bounding_rect, 5, color_purple, 2, 8);
	drawVectorPoints(contours_image, filtered_start_points, color_blue, true);
	drawVectorPoints(contours_image, filtered_far_points, color_red, true);
	drawVectorPoints(contours_image, filtered_finger_points, color_yellow, false);
	putText(contours_image, to_string(filtered_finger_points.size()), center_bounding_rect, FONT_HERSHEY_PLAIN, 3, color_purple);

	// and on the starting frame
	drawContours(frame, contours, biggest_contour_index, color_green, 2, 8, hierarchy);
	circle(frame, center_bounding_rect, 5, color_purple, 2, 8);
	drawVectorPoints(frame, filtered_finger_points, color_yellow, false);
	putText(frame, to_string(filtered_finger_points.size()), center_bounding_rect, FONT_HERSHEY_PLAIN, 3, color_purple);

	return contours_image;
}

double Prueba::findPointsDistance(Point a, Point b) {
	Point difference = a - b;
	return sqrt(difference.ddot(difference));
}

vector<Point> Prueba::compactOnNeighborhoodMedian(vector<Point> points, double max_neighbor_distance) {
	vector<Point> median_points;
	
	if (points.size() == 0)		
		return median_points;

	if (max_neighbor_distance <= 0)
		return median_points;

	// we start with the first point
	Point reference = points[0];
	Point median = points[0];

	for (int i = 1; i < points.size(); i++) {
		if (findPointsDistance(reference, points[i]) > max_neighbor_distance) {
			
			// the point is not in range, we save the median
			median_points.push_back(median);

			// we swap the reference
			reference = points[i];
			median = points[i];
		}
		else
			median = (points[i] + median) / 2;
	}

	// last median
	median_points.push_back(median);

	return median_points;
}

double Prueba::findAngle(Point a, Point b, Point c) {
	double ab = findPointsDistance(a, b);
	double bc = findPointsDistance(b, c);
	double ac = findPointsDistance(a, c);
	return acos((ab * ab + bc * bc - ac * ac) / (2 * ab * bc)) * 180 / CV_PI;
}

bool Prueba::isFinger(Point a, Point b, Point c, double limit_angle_inf, double limit_angle_sup, Point palm_center, double min_distance_from_palm) {
	double angle = findAngle(a, b, c);
	if (angle > limit_angle_sup || angle < limit_angle_inf)
		return false;

	// the finger point sohould not be under the two far points
	int delta_y_1 = b.y - a.y;
	int delta_y_2 = b.y - c.y;
	if (delta_y_1 > 0 && delta_y_2 > 0)
		return false;

	// the two far points should not be both under the center of the hand
	int delta_y_3 = palm_center.y - a.y;
	int delta_y_4 = palm_center.y - c.y;
	if (delta_y_3 < 0 && delta_y_4 < 0)
		return false;

	double distance_from_palm = findPointsDistance(b, palm_center);
	if (distance_from_palm < min_distance_from_palm)
		return false;
	
	// this should be the case when no fingers are up
	double distance_from_palm_far_1 = findPointsDistance(a, palm_center);
	double distance_from_palm_far_2 = findPointsDistance(c, palm_center);
	if (distance_from_palm_far_1 < min_distance_from_palm / 4 || distance_from_palm_far_2 < min_distance_from_palm / 4)
		return false;

	return true;
}

vector<Point> Prueba::findClosestOnX(vector<Point> points, Point pivot) {
	vector<Point> to_return(2);

	if (points.size() == 0)
		return to_return;

	double distance_x_1 = DBL_MAX;
	double distance_1 = DBL_MAX;
	double distance_x_2 = DBL_MAX;
	double distance_2 = DBL_MAX;
	int index_found = 0;

	for (int i = 0; i < points.size(); i++) {
		double distance_x = findPointsDistanceOnX(pivot, points[i]);
		double distance = findPointsDistance(pivot, points[i]);

		if (distance_x < distance_x_1 && distance_x != 0 && distance <= distance_1) {
			distance_x_1 = distance_x;
			distance_1 = distance;
			index_found = i;
		}
	}

	to_return[0] = points[index_found];

	for (int i = 0; i < points.size(); i++) {
		double distance_x = findPointsDistanceOnX(pivot, points[i]);
		double distance = findPointsDistance(pivot, points[i]);

		if (distance_x < distance_x_2 && distance_x != 0 && distance <= distance_2 && distance_x != distance_x_1) {
			distance_x_2 = distance_x;
			distance_2 = distance;
			index_found = i;
		}
	}

	to_return[1] = points[index_found];

	return to_return;
}

double Prueba::findPointsDistanceOnX(Point a, Point b) {
	double to_return = 0.0;

	if (a.x > b.x)
		to_return = a.x - b.x;
	else
		to_return = b.x - a.x;

	return to_return;
}

void Prueba::drawVectorPoints(Mat image, vector<Point> points, Scalar color, bool with_numbers) {
	for (int i = 0; i < points.size(); i++) {
		circle(image, points[i], 5, color, 2, 8);
		if(with_numbers)
			putText(image, to_string(i), points[i], FONT_HERSHEY_PLAIN, 3, color);
	}
}

// CONTROLES DE ASCPECTO
/*
void Prueba::on_trackbarHmin( int v, void *pP) {
    // Creamos el structring element (que puede ser una cruz, un rectángulo o una elipse):
    // MORPH_CROSS, MORPH_RECT, MORPH_ELLIPSE
    //Mat elemento = getStructuringElement(MORPH_CROSS, Size(alpha_slider+1,alpha_slider+1), Point(-1,-1));
    // Aplicamos la operación de dilatación
    //morphologyEx(imagen, frame, MORPH_DILATE,elemento); 
    alpha_slider1 = 0;
    alpha_slider_max1 = 179;
    cvtColor(frame,COLOR_BGR2HSV);

    cout << "trackbarHmin " << v << endl;   
}
*/

void Prueba::on_trackbarSmin( int v, void *pP) {
    // Creamos el structring element (que puede ser una cruz, un rectángulo o una elipse):
    // MORPH_CROSS, MORPH_RECT, MORPH_ELLIPSE
    //Mat elemento = getStructuringElement(MORPH_CROSS, Size(alpha_slider+1,alpha_slider+1), Point(-1,-1));
    // Aplicamos la operación de dilatación
    //morphologyEx(imagen, frame, MORPH_DILATE,elemento);
    alpha_slider2 = 0;
    alpha_slider_max2 = 255;
    cout << "trackbarsmin " << v << endl; 
}

void Prueba::on_trackbarVmin( int v, void *pP) {
    // Creamos el structring element (que puede ser una cruz, un rectángulo o una elipse):
    // MORPH_CROSS, MORPH_RECT, MORPH_ELLIPSE
    //Mat elemento = getStructuringElement(MORPH_CROSS, Size(alpha_slider+1,alpha_slider+1), Point(-1,-1));
    // Aplicamos la operación de dilatación
    //morphologyEx(imagen, frame, MORPH_DILATE,elemento);
    alpha_slider3 = 0;
    alpha_slider_max3 = 255;
    cout << "trackbarvmin " << v << endl; 
    
}

void Prueba::on_trackbarHmax( int v, void *pP) {
    // Creamos el structring element (que puede ser una cruz, un rectángulo o una elipse):
    // MORPH_CROSS, MORPH_RECT, MORPH_ELLIPSE
    //Mat elemento = getStructuringElement(MORPH_CROSS, Size(alpha_slider+1,alpha_slider+1), Point(-1,-1));
    // Aplicamos la operación de dilatación
    //morphologyEx(imagen, frame, MORPH_DILATE,elemento); 
    alpha_slider4 = 0 ;
    alpha_slider_max4 = 179;   
    cout << "trackbarHmax " << v << endl; 
}

void Prueba::on_trackbarSmax( int v, void *pP) {
    // Creamos el structring element (que puede ser una cruz, un rectángulo o una elipse):
    // MORPH_CROSS, MORPH_RECT, MORPH_ELLIPSE
    //Mat elemento = getStructuringElement(MORPH_CROSS, Size(alpha_slider+1,alpha_slider+1), Point(-1,-1));
    // Aplicamos la operación de dilatación
    //morphologyEx(imagen, frame, MORPH_DILATE,elemento);
    alpha_slider5 = 0 ;
    alpha_slider_max5 = 255;   
    cout << "trackbarsmax " << v << endl; 
}


void Prueba::on_trackbarVmax( int v, void *pP) {
    // Creamos el structring element (que puede ser una cruz, un rectángulo o una elipse):
    // MORPH_CROSS, MORPH_RECT, MORPH_ELLIPSE
    //Mat elemento = getStructuringElement(MORPH_CROSS, Size(alpha_slider+1,alpha_slider+1), Point(-1,-1));
    // Aplicamos la operación de dilatación
    //morphologyEx(imagen, frame, MORPH_DILATE,elemento);
    alpha_slider6 = 0 ;
    alpha_slider_max6 = 255; 
    cout << "trackbarvmax " << v << endl; 
}


void Prueba::on_trackbarYmin( int v, void *pP) {
    // Creamos el structring element (que puede ser una cruz, un rectángulo o una elipse):
    // MORPH_CROSS, MORPH_RECT, MORPH_ELLIPSE
    //Mat elemento = getStructuringElement(MORPH_CROSS, Size(alpha_slider+1,alpha_slider+1), Point(-1,-1));
    // Aplicamos la operación de dilatación
    //morphologyEx(imagen, frame, MORPH_DILATE,elemento);
    cout << "on_trackbarYmin " << v << endl; 
}    


void Prueba::on_trackbarCrmin( int v, void *pP) {
    // Creamos el structring element (que puede ser una cruz, un rectángulo o una elipse):
    // MORPH_CROSS, MORPH_RECT, MORPH_ELLIPSE
    //Mat elemento = getStructuringElement(MORPH_CROSS, Size(alpha_slider+1,alpha_slider+1), Point(-1,-1));
    // Aplicamos la operación de dilatación
    //morphologyEx(imagen, frame, MORPH_DILATE,elemento);
    cout << "on_trackbarcrmin " << v << endl; 
}

void Prueba::on_trackbarCbmin( int v, void *pP) {
    // Creamos el structring element (que puede ser una cruz, un rectángulo o una elipse):
    // MORPH_CROSS, MORPH_RECT, MORPH_ELLIPSE
    //Mat elemento = getStructuringElement(MORPH_CROSS, Size(alpha_slider+1,alpha_slider+1), Point(-1,-1));
    // Aplicamos la operación de dilatación
    //morphologyEx(imagen, frame, MORPH_DILATE,elemento);
    cout << "on_trackbarcbmin " << v << endl; 
    
}

void Prueba::on_trackbarYmax( int v, void *pP) {
    // Creamos el structring element (que puede ser una cruz, un rectángulo o una elipse):
    // MORPH_CROSS, MORPH_RECT, MORPH_ELLIPSE
    //Mat elemento = getStructuringElement(MORPH_CROSS, Size(alpha_slider+1,alpha_slider+1), Point(-1,-1));
    // Aplicamos la operación de dilatación
    //morphologyEx(imagen, frame, MORPH_DILATE,elemento);    
    cout << "on_trackbarYmax " << v << endl; 
}

void Prueba::on_trackbarCrmax( int v, void *pP) {
    // Creamos el structring element (que puede ser una cruz, un rectángulo o una elipse):
    // MORPH_CROSS, MORPH_RECT, MORPH_ELLIPSE
    //Mat elemento = getStructuringElement(MORPH_CROSS, Size(alpha_slider+1,alpha_slider+1), Point(-1,-1));
    // Aplicamos la operación de dilatación
    //morphologyEx(imagen, frame, MORPH_DILATE,elemento);
    cout << "on_trackbarcrmax " << v << endl; 
}

void Prueba::on_trackbarCbmax( int v, void *pP) {

    // Creamos el structring element (que puede ser una cruz, un rectángulo o una elipse):
    // MORPH_CROSS, MORPH_RECT, MORPH_ELLIPSE
    //Mat elemento = getStructuringElement(MORPH_CROSS, Size(alpha_slider+1,alpha_slider+1), Point(-1,-1));
    // Aplicamos la operación de dilatación
    //morphologyEx(imagen, frame, MORPH_DILATE,elemento);
    cout << "on_trackbarcbmax " << v << endl; 
    
}















