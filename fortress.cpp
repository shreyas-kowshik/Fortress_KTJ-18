#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/video/tracking.hpp"
#include <cmath>
#include <iostream>
#include <queue>
#include <string.h>
#include <iostream>
#include <stdio.h>
#include <sstream>
#include <math.h>

using namespace cv;
using namespace std;

//name of the template containing images
#define template_name "IMG_5.jpg"
#define Pi 3.14159
typedef pair<float, float> Pair;

string colors[4];

//thresholds
int disThresh = 30;
double thetaThresh = 10;

//stores the final node points
vector<Point> nodes;
vector<Point> path;//path contains both the nodes and alternatively the home center

//for contours
vector<Vec4i> hierarchy;
vector<vector<Point> > contours;

//Bot front and back and center
Point bot_front,bot_back,bot_center;

//Home
Point home;

//Images
Mat frame,arena,imgBotF,imgBotB,tem;

//color arrays--use RGB values here
int first[6] = {230,255,230,255,0,25};
int second[6] = {0,25,0,25,230,255};
int third[6] = {0,25,230,255,0,25};
int fourth[6] = {230,255,0,25,0,25};
int fifth[6] = {230,255,0,25,230,255};

int arenaColor[6] = {0,255,0,255,231,255};
int botFrontColor[6] = {116,146,188,255,167,255};
int botBackColor[6] = {173,200,131,255,106,161};

int homeColor[6] = {};

//vectors
vector<Rect> arenaBounds;//to store the bounding rectangles for the arena shapes
vector<Mat> shapes;
vector<Mat> templates,templates_t;

//Area thresholding
int shapeMinArea = 2000;
int shapeMaxArea = 156781;
int botMinArea = 750;
int botMaxArea = 1000000;

//number of frames after which templates are computed and nodes are freezed
int frameThreshNumer = 20;

//to locate the next point to move to
int moveIndex = 0;

//Maybe temporary
int state = 0;
int homeMinArea;
int homeMaxArea;

Mat mapImage(Mat t,int a[6]) {
	//order l&h,b&g&r
	Mat f(t.rows,t.cols,CV_8UC1,Scalar(0));
	for(int i = 0;i<t.rows;i++) {
		for(int j = 0;j<t.cols;j++) {
			if(t.at<Vec3b>(i,j)[0] >= a[0] & t.at<Vec3b>(i,j)[0] <= a[1] & t.at<Vec3b>(i,j)[1] >= a[2] & t.at<Vec3b>(i,j)[1] <= a[3] & t.at<Vec3b>(i,j)[2] >= a[4] & t.at<Vec3b>(i,j)[2] <= a[5]) {
				f.at<uchar>(i,j) = 255;
			}
			else
				f.at<uchar>(i,j) = 0;
		}
	}
	return f;
}

//This will get the bounding boxes from the mapped image and seperate each image 
//into the shapes vector
void getArenaRects(Mat t,int minA,int maxA) {
	arenaBounds.clear();
	shapes.clear();
	Mat x;
	t.copyTo(x);
	findContours(t, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);
	for(int i = 0;i<contours.size();i++) {
		Rect bound = boundingRect(contours[i]);
		if(contourArea(contours[i]) <= maxA & contourArea(contours[i]) >= minA) {
		arenaBounds.push_back(bound);
	}
	}

	for(int i = 0;i<arenaBounds.size();i++) {
		shapes.push_back(x(arenaBounds[i]));
	}

}

void setUpTemplates() {
	tem = imread(template_name);


	templates_t.push_back(mapImage(tem,first));
	templates_t.push_back(mapImage(tem,second));
	templates_t.push_back(mapImage(tem,third));
	templates_t.push_back(mapImage(tem,fourth));
	templates_t.push_back(mapImage(tem,fifth));

	for(int i = 0;i<templates_t.size();i++) {
		Mat x;
		templates_t[i].copyTo(x);
		contours.clear();
		hierarchy.clear();
		int dilation_size = 1;
		Mat element = getStructuringElement(MORPH_CROSS,
                                       Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                       Point( dilation_size, dilation_size ) );
  	  	/// Apply the dilation operation
  	  	dilate(templates_t[i],templates_t[i], element );
		findContours(templates_t[i], contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);
		//resize(x,x,Size(4380,2820));
		//Dunno why this is the part that is not easy to explain
		//check for something other than hardcoding
		for(int i = 0;i<contours.size();i++) {
		//hardcoded area constraints for the template
		//if(contourArea(contours[i]) > 2000 & contourArea(contours[i]) < 1000000) {
		Rect boundA = boundingRect(contours[i]);
		rectangle(x,boundA.tl(),boundA.br(),Scalar(255),2,8,0);
		templates.push_back(x(boundA));
	}
	}
	}

//}

//sanity check to see if all is working
void seeTemplates() {

}

void matchTem() {
	for(int j = 0;j<templates.size();j++) {
	double min = -10;
	int index = 1; 
	for(int i = 0;i<shapes.size();i++) {
		int dilation_size = 1;
		Mat element = getStructuringElement(MORPH_CROSS,
                                       Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                       Point( dilation_size, dilation_size ) );
  	  /// Apply the dilation operation
  	  dilate(shapes[i],shapes[i], element );
	  resize(templates[j],templates[j],shapes[i].size(),0,0,INTER_CUBIC);
	  Mat result = Mat(shapes[i].rows-templates[j].rows+1,shapes[i].cols-templates[j].cols+1,CV_32FC1);

	  /// Do the Matching
	  matchTemplate( shapes[i], templates[j], result, 1 );

	  /// Localizing the best match with minMaxLoc
	  double minVal; double maxVal; Point minLoc; Point maxLoc;
	  Point matchLoc;
	  minMaxLoc( result, &minVal, &maxVal, &minLoc, &maxLoc, Mat() );
	  if(min < 0){
	  	min = minVal;
	  	index = i;
	  }
	  else {
	  	if(minVal < min){
	  		min = minVal;
	  		index = i;
	  	}
	  }
	  /// For SQDIFF and SQDIFF_NORMED, the best matches are lower values. For all the other methods, the higher the better
	  
	 }
	 nodes.push_back(Point(arenaBounds[index].x + arenaBounds[index].width/2,arenaBounds[index].y+arenaBounds[index].height/2));
	 path.push_back(nodes[j]);
}
}

//Plots node points in the correct order
void plotNodes() {
	ostringstream ss;
	for(int i = 0;i<nodes.size();i++) {
		ss.clear();
		ss << (i+1);
		string pt = ss.str();
		putText(frame, pt, Point(nodes[i].x,nodes[i].y), FONT_HERSHEY_COMPLEX_SMALL, 0.8, Scalar(255,255,255), 1, CV_AA);		
	}
}

Point getCenters(Mat t,int minA,int maxA) {
	Mat x;
	t.copyTo(x);
	Point cen;
	findContours(x, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);
	for(int i = 0;i<contours.size();i++) {
		Rect bound = boundingRect(contours[i]);
		if(contourArea(contours[i]) <= maxA & contourArea(contours[i]) >= minA) {
			cen = Point(bound.x + bound.width/2,bound.y + bound.height/2);
	}
	}
	return cen;
}

//Serial communication
void sendCommand(char command) {
	char send = command;
	FILE *serport = fopen("/dev/ttyACM0","w");

	if(serport!=NULL) {
		fprintf(serport,"%c\n",send);
		fclose(serport);
		printf("%c\n",send);
	}
	else {
		printf("File not open");
	}
}

void blink(int n) {
	for(int i = 0;i<n;i++) {
		sendCommand('B');
		waitKey(2000);
	}
}

//Maybe temporary 
void update(int t,void *a) {
 
}

int dis(Point a,Point b) {
	return abs(a.x-b.x) + abs(a.y-b.y);
}

void moveBot(Point target) {
	Pair node,center,front,back;
	double robo_slope,node_slope,theta,tan_theta,dist1,dist2;
	
	node.first=target.x;
	node.second=target.y;
	front.first=bot_front.x;
	front.second=bot_front.y;
	back.first=bot_back.x;
	back.second=bot_back.y;

	center.first=front.first/2 + back.first/2;
	center.second=front.second/2 + back.second/2;

	if(front.first==back.first){
		front.first+=0.001;
	}
	if(node.first==center.first){
		node.first+=0.001;
	}
		robo_slope=(front.second-back.second)/(front.first-back.first);
		node_slope=(node.second-center.second)/(node.first-center.first);

	if(1+robo_slope*node_slope==0){
	tan_theta=(robo_slope-node_slope)/(1.001+robo_slope*node_slope);	
	}
	else{tan_theta=(robo_slope-node_slope)/(1+robo_slope*node_slope);}
		
	theta= (atan(tan_theta) * 180)/Pi;

	if(abs(theta) > thetaThresh) {
		if(theta>0){
			sendCommand('A');
		}
		else{
			sendCommand('D');
		}
	}


	else  {
	dist1=(front.first-node.first)*(front.first-node.first) + (front.second-node.second)*(front.second-node.second);
	dist2=(back.first-node.first)*(back.first-node.first) + (back.second-node.second)*(back.second-node.second);

	if(dist1<=dist2){
		sendCommand('W');
	}
	else{
		sendCommand('S');
	}
	}
}

int main() {
	namedWindow("A",WINDOW_NORMAL);
	setUpTemplates();
	seeTemplates();

	namedWindow("B",WINDOW_NORMAL);

	VideoCapture vid(1);

	vid >> frame;

	//createTrackbar("A","State",&state,1,update);
	int num = 0;
	int flag = 0;

	while(!frame.empty()) {
		vid >> frame;

  		bot_front = getCenters(mapImage(frame,botFrontColor),botMinArea,botMaxArea);
  		bot_back = getCenters(mapImage(frame,botBackColor),botMinArea,botMaxArea);
  		bot_center = Point((bot_front.x + bot_back.x)/2,(bot_front.y + bot_back.y)/2);


		//-----------START OF PREPROCESSING CODE-----------
		//start of if flag
		if(flag == 0) {
		path.clear();
		nodes.clear();
		arena = mapImage(frame,arenaColor);

		//---------OPTIONAL---------
		Mat arenaT;
		arena.copyTo(arenaT);
		int dilation_size = 2;//2 for round 1 and 1 for round 2
		Mat element = getStructuringElement(MORPH_CROSS,
                                       Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                       Point( dilation_size, dilation_size ) );
  		/// Apply the dilation operation
  		dilate(arenaT,arena, element );
  		//--------OPTIONAL----------

  		getArenaRects(arena,shapeMinArea,shapeMaxArea);
  		//Now vector shapes contains the diffferent shapes in the arena
  		//Also arenaBounds contains the bounding boxes of each shape in the original image
  		}
  		//end of if flag

  		for(int i = 0;i<arenaBounds.size();i++) {
  			rectangle(frame,arenaBounds[i].tl(),arenaBounds[i].br(),Scalar(0,0,255),2,8,0);
  		}

  		if(flag <= 0) {
  			if(num >= frameThreshNumer) {
  				flag = 1;
  				matchTem();
  				path.push_back(home);
			}
  		}

  		plotNodes();

  	num++;

  	if(num < frameThreshNumer) {
  	home.x = bot_center.x;
  	home.y = bot_center.y;
    }

    for(int i = 0;i<arenaBounds.size();i++) {
    	if(dis(bot_center,Point(arenaBounds[i].x + arenaBounds[i].width/2,arenaBounds[i].y + arenaBounds[i].height/2)) < disThresh) {
    		line(frame,bot_center,Point(arenaBounds[i].x + arenaBounds[i].width/2,arenaBounds[i].y + arenaBounds[i].height/2), Scalar(255,0,0), 2, CV_AA);
    	}
    }

  	//draw home and bot centers
  	circle(frame,Point(bot_front.x,bot_front.y),5,Scalar(0,255,0),2,8,0);
  	circle(frame,Point(bot_back.x,bot_back.y),3,Scalar(0,255,0),2,8,0);
  	circle(frame,Point(bot_center.x,bot_center.y),3,Scalar(0,0,255),2,8,0);
  	circle(frame,Point(home.x,home.y),3,Scalar(255,255,0),2,8,0);
  	
  	//------------END OF PREPROCESSING AND RENDERING PART---------------


  	if(moveIndex < path.size()) {

  	}

  	//Now let's do something interesting
  	//How about move the bot...
  	//the vector path contains the complete path to move
  	cout << moveIndex << endl;
  	if(flag > 0) {
  	if(moveIndex < path.size()) {
  	Point target = path[moveIndex];

  	line(frame,bot_center,path[moveIndex], Scalar(255,0,0), 2, CV_AA);
  	line(frame,bot_front,bot_back, Scalar(255,255,255), 2, CV_AA);

  	if(dis(target,bot_center) > disThresh) {
  		moveBot(target);
  	}
  	else {
  		sendCommand('X');
  		blink(moveIndex+1);
  		moveIndex++;
  	}
  	}
  	else {
  		break;
  	}
  	}
  	imshow("A",frame);
  	waitKey(1);
	}
	return 0;
}