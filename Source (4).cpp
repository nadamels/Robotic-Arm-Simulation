#include <iostream>
#include<cmath>
#include "pbPlots.hpp"
#include"supportLib.hpp"
#include<vector>

using namespace std;
//Class point for storing cartesian points
class Point {
private:
	//x and y coordiantes
	double x;
	double y;

public:
	//Default constructors
	Point() {
		x = 0;
		y = 0;
	}
	//Non Default constructor
	Point(double x_1, double y_1) {
		x = x_1;
		y = y_1;
	}
	//getters and setters
	double getX() {return(x);}
	double getY() {	return(y);}
	void setX(double new_x) {x = new_x;}
	void setY(double new_y) {y = new_y;}
};

//Segment Class
class Segment {
private:
	//Starting point
	Point p1;
	//Ending point
	Point ep;
	//Length of segment
	double length;
	//Angle of segment
	double angle;
	//This will be used as a refernce to the parent segment
	Segment* parent;

	//endpoint is not determined by user, but rather calculated using forward kinematics prinicples
	void endPoint()
	{
		double dx = length * cos(angle);
		double dy = length * sin(angle);
		//x vlaue of endpoint
		ep.setX(p1.getX() + dx);
		//y value of endpoint
		ep.setY(p1.getY() + dy);
	}

public:
	//Defualt constructor
	Segment() {
		p1.setX(0);
		p1.setY(0);
		length = 1;
		angle = 0;
		endPoint();
	}
	//Non default constructor
	//This is to initialize root segemnt with no parent
	Segment(Point p_1, double len, double ang) {
		p1 = p_1;
		//This is to check that length is always a positive number
		if (len > 0) {
			length = len;
		}
		else if (len == 0) {
			exit(-1);
		}
		else if (len < 0) {
			length = abs(len);
		}
		angle = ang;
		endPoint();
		//no parent because this is used for root segment
		parent = NULL;
	}
	//Another non default constructor for non-root segments
	Segment(Segment parent_, double len, double ang) {
		//assignig the address of the parent segment to the pointer
		parent = &parent_;
		//The parent pointer is pointing towrds the endpoint of the parent
		p1 = parent->ep;
		if (len > 0) {
			length = len;
		}
		else if (len == 0) {
			exit(-1);
		}
		else if (len < 0) {
			length = abs(len);
		}
		angle = ang;
		endPoint();

	}
	//getters and setters
	Point getPoint() { return (p1); }
	double getlen() { return(length); }
	double getang() { return(angle); }
	Point getEndPoint() { return(ep); }
	//end point is recalculated everytime angle, point or length change happens
	void setPoint(Point new_p1) {
		p1 = new_p1;
		endPoint();
	}
	void setlength(double len) {
		if (len > 0) {
			length = len;
		}
		else if (len == 0) {
			exit(-1);
		}
		else if (len < 0) {
			length = abs(len);
		}
		endPoint();
	}
	void setAngle(double ang) {
		angle = ang;
		endPoint();
	}
	//function to print segment information
	void printSegmentInfo() {
		cout << "Starting Point: " << p1.getX() << "," << p1.getY() << endl;;
		cout << "Angle: " << angle << endl;
		cout << "Length: " << length << endl;
		cout << "End Point: " << ep.getX() << "," << ep.getY() << endl;
	}
};

//robot class
class Robot {
private:
	//number of segments
	int numOfSegs;
	//initial point
	Point initial;
	//this will be used to create a dynamic array of segments
	Segment* chain;
	//root segment
	Segment root;
	//this segment object will be used to determine the sequence of the chain of segments
	Segment current;


public:
	//default constructor
	Robot() {
		numOfSegs = 1;
		initial.setX(0);
		initial.setY(0);
		root.setAngle(0);
		root.setlength(1);
		root.setPoint(initial);
		chain = new Segment[numOfSegs];
		chain[0] = root;
		current = root;
	}

	//non defualt constructor
	Robot(int num, double angle[], double length[], Point initial_) {
		initial = initial_;
		numOfSegs = num;
		Segment root_(initial_, length[0], angle[0]);
		chain = new Segment[numOfSegs];
		//assigning each respective angle and length to the segments
		chain[0] = root_;
		current = root_;
		if (numOfSegs > 1) {
			Segment next(root_, length[1], angle[1]);
			chain[1] = next;
			for (int i = 2; i < numOfSegs; i++) {
				current = next;
				Segment temp(current.getEndPoint(), length[i], angle[i]);
				next = temp;
				chain[i] = next;

			}
		}
	}
	//getters and setters
	int getNumOfSegs() { return(numOfSegs); }
	Point getInitial() { return(initial); }
	Segment getroot() { return(root); }
	void setNumOfSegs(int newSeg) { numOfSegs = newSeg; }
	void setInitial(Point point_) { initial = point_; }
	void setRoot(Segment root_) { root = root_; };
	//function to add segments from endpoint
	void addSegment(int addnumseg, double angles_[], double lengths_[]) {
		int newNum = numOfSegs + addnumseg;

		if (newNum > numOfSegs) {
			Segment* temp = new Segment[newNum];
			//moving contents of chain into temp
			for (int i = 0; i < numOfSegs; i++) {
				temp[i] = chain[i];
			}

			//adding the new segments
			int count = 1;
			current = chain[numOfSegs - 1];
			Segment next(current, lengths_[0], angles_[0]);
			temp[numOfSegs] = next;
			for (int i = numOfSegs + 1; i < newNum; i++) {
				current = next;
				Segment temp_(current.getEndPoint(), lengths_[count], angles_[count]);
				next = temp_;
				temp[i] = next;
				count++;
			}
			//creeating new resized chain
			delete[]chain;
			numOfSegs = newNum;
			chain = new Segment[numOfSegs];
			for (int i = 0; i < numOfSegs; i++) {
				chain[i] = temp[i];
			}
		}
		//incase the user enter a negative number
		else {
			cout << "You have entered an INVALID number of segments to ADD" << endl;

		}
	}
	//this function removes segments
	void removeSegment(int removeSeg) {
		//check if the user wants to remove more than the number of prexisting segments
		if (removeSeg < numOfSegs) {
			//resizing the number of segments
			int newNum = numOfSegs - removeSeg;
			Segment* temp = new Segment[newNum];
			for (int i = 0; i < newNum; i++) {
				temp[i] = chain[i];
			}
			//resizing the dynamic array of segments
			delete[]chain;
			numOfSegs = newNum;
			chain = new Segment[numOfSegs];
			for (int i = 0; i < numOfSegs; i++) {
				chain[i] = temp[i];
			}
		}
		else
			cout << "You have entered an INVALID number to REMOVE" << endl;
	}
	//this funciton resets the chain to a default positoin
	void reset() {
		//Resestting ALL the segments to the POSITIVE x-axis with ZERO angles
		//This will change the angle and start-end point of each segment 
		Point aPoint(0, 0);
		chain[0].setAngle(0);
		chain[0].setPoint(aPoint);
		for (int i = 1; i < numOfSegs; i++) {
			Point aPoint_(abs(chain[i - 1].getEndPoint().getX()), 0);
			chain[i].setPoint(aPoint_);
			chain[i].setAngle(0);
		}
	}
	//This calculates the position of end effector and changes the position of the angles if wanted by user
	Point ForwardKinematics(double angles[]) {
		int sizeOfAngles;
		Point endEffector;
		//incase the user just wants to know the current position of the end effector after initializing the robot
		//without changing the angles' values
		if (angles == NULL) {
			endEffector = chain[numOfSegs - 1].getEndPoint();
		}
		else {
			chain[0].setAngle(angles[0]);
			for (int i = 1; i < numOfSegs; i++) {
				chain[i].setAngle(angles[i]);
				Point p1_(chain[i - 1].getEndPoint().getX(), chain[i - 1].getEndPoint().getY());
				chain[i].setPoint(p1_);
			}
			endEffector = chain[numOfSegs - 1].getEndPoint();
		}
		cout << "The position of the end effector is " << endEffector.getX() << "," << endEffector.getY();
		return(endEffector);
	}
	//prints coordiantes
	void PrintCoordinates() {
		for (int i = 0; i < numOfSegs; i++) {
			cout << chain[i].getPoint().getX() << "," << chain[i].getPoint().getY() << " and " << chain[i].getEndPoint().getX() << "," << chain[i].getEndPoint().getY() << endl << endl;
		}
	}
	//this fucntion creates PNG image of robot chain
	void VisualizePose() {
		//allocating x and ys into seperate dyanmic arrays
		double* x = new double[numOfSegs + 1];
		double* y = new double[numOfSegs + 1];

		for (int i = 0; i < numOfSegs; i++) {
			x[i] = chain[i].getPoint().getX();
			y[i] = chain[i].getPoint().getY();
		}
		x[numOfSegs] = chain[numOfSegs - 1].getEndPoint().getX();
		y[numOfSegs] = chain[numOfSegs - 1].getEndPoint().getY();
		//creating an image refernce
		RGBABitmapImageReference* imageReference = CreateRGBABitmapImageReference();
		//creating a vector of x and y coordiantes
		vector<double>xs, ys;
		for (int i = 0; i < numOfSegs + 1; i++) {
			xs.push_back(x[i]);
			ys.push_back(y[i]);
		}
		//generting image
		DrawScatterPlot(imageReference, 600, 400, &xs, &ys);

		vector<double>* PNGdata = ConvertToPNG(imageReference->image);
		WriteToFile(PNGdata, "plot.png");
		DeleteImage(imageReference->image);
	}
};

int main() {
	const double pi = acos(-1);
	double angles[4] = { pi/3 ,pi/4,pi / 6};
	double lengths[4] = { 10,20,10 };
	Point p1(0, 0);

	Segment s1(p1, 10, pi/2);

	s1.printSegmentInfo();

	Robot R1(3, angles, lengths, p1);
	double addangles[1] = { pi / 6};
	double addlength[1] = { 10 };
	

	Robot R1;
	R1.reset();
	R1.addSegment(1, addangles, addlength);
	R1.removeSegment(1);
	R1.ForwardKinematics(angles);
	R1.PrintCoordinates();
	R1.ForwardKinematics(NULL);
	

	R1.VisualizePose();

	return 0;
}