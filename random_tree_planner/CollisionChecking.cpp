///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 2
// Authors: Yue Pan&YuHao Gong
// Date:13/09/2016
//////////////////////////////////////

#include "CollisionChecking.h"
#include <iostream>
#include <math.h>
#define PI 3.1415926
using namespace std;
// Intersect the point (x,y) with the set of rectangles.  If the point lies outside of all obstacles, return true.
bool isValidPoint(double x, double y, const std::vector<Rectangle>& obstacles)
{
    // IMPLEMENT ME!
    bool result = true;
    int i,size;
    size = obstacles.size();

    for(i = 0;i<size;i++) {
       double x_min = obstacles[i].x;
       double y_min = obstacles[i].y;
       double x_max = obstacles[i].x + obstacles[i].width;
       double y_max = obstacles[i].y + obstacles[i].height;
        if ( (x >= x_min) && (x <= x_max) && (y <= y_max) && (y >= y_min) ) {
                result = false;
                break;
    }
  }
     return result;

}

// Intersect a circle with center (x,y) and given radius with the set of rectangles.  If the circle
// lies outside of all obstacles, return true
bool isValidCircle(double x, double y, double radius, const std::vector<Rectangle>& obstacles)
{
    bool result = true;
    int i,size;
    //double r=0.2;
    size = obstacles.size();

    for(i = 0;i<size;i++) {
       double x_min = obstacles[i].x;
       double y_min = obstacles[i].y;
       double x_max = obstacles[i].x + obstacles[i].width;
       double y_max = obstacles[i].y + obstacles[i].height;
        if ( ((x >= (x_min-radius)) && (x <= (x_max+radius)) && (y <= y_max) && (y >= y_min)) || 
             ((x >= x_min) && (x <= x_max) && (y <= (y_max+radius)) && (y >= (y_min-radius))) || 
             (sqrt(pow((x-x_min),2)+pow((y-y_min),2))<=radius) || (sqrt(pow((x-x_max),2)+pow((y-y_min),2))<=radius) ||
             (sqrt(pow((x-x_min),2)+pow((y-y_max),2))<=radius) || (sqrt(pow((x-x_max),2)+pow((y-y_max),2))<=radius) )  {
                 result = false;
                 break;
          }
      }
      return result;
    
   
}


// Intersect a square with center at (x,y), orientation theta, and the given side length with the set of rectangles.
// If the square lies outside of all obstacles, return true


double min(double x,double y) {  
    return x<y?x:y;  
}  
double max(double x,double y) {  
    return x>y?x:y;  
}  
bool onsegment(Point pi,Point pj,Point pk) {  
    if(min(pi.x,pj.x)<=pk.x&&pk.x<=max(pi.x,pj.x)) {  
        if(min(pi.y,pj.y)<=pk.y&&pk.y<=max(pi.y,pj.y)) {  
            return true;  
        }  
    }  
    return false;  
}  
double direction(Point pi,Point pj,Point pk) {  
    return (pi.x-pk.x)*(pi.y-pj.y)-(pi.y-pk.y)*(pi.x-pj.x);  
}  
bool judge(Point p1,Point p2,Point p3,Point p4) {  
    double d1 = direction(p3,p4,p1);  
    double d2 = direction(p3,p4,p2);  
    double d3 = direction(p1,p2,p3);  
    double d4 = direction(p1,p2,p4);  
    if(d1*d2<0&&d3*d4<0) {
        return true;  
    }
    if(d1==0&&onsegment(p3,p4,p1)) {
        return true;  
    }
    if(d2==0&&onsegment(p3,p4,p2)) {
        return true;  
    }
    if(d3==0&&onsegment(p1,p2,p3)) {
        return true;  
    }
    if(d4==0&&onsegment(p1,p2,p4)) {
        return true;  
    }

    return false;  
}  

bool isValidSquare(double x, double y, double theta, double sideLength, const std::vector<Rectangle>& obstacles)
{
    // IMPLEMENT ME!
    bool result=true;
    double L = sideLength;
    //4 points of square (x,y)
    Point a = {x - (sqrt(2)/2) * L * cos(theta + PI/4), y - (sqrt(2)/2) * L * sin(theta + PI/4)};
    Point b = {x - (sqrt(2)/2) * L * sin(theta + PI/4), y + (sqrt(2)/2) * L * cos(theta + PI/4)};
    Point c = {x + (sqrt(2)/2) * L * cos(theta + PI/4), y + (sqrt(2)/2) * L * sin(theta + PI/4)};
    Point d = {x + (sqrt(2)/2) * L * sin(theta + PI/4), y - (sqrt(2)/2) * L * cos(theta + PI/4)};
    
    double r_xmin = x - L/2;
    double r_xmax = x + L/2;
    double r_ymin = y - L/2;
    double r_ymax = y + L/2;
    //4 points of obstacles
    int i,size;
    size = obstacles.size();

    for(i = 0;i<size;i++) {
       double x_min = obstacles[i].x;
       double y_min = obstacles[i].y;
       double x_max = obstacles[i].x + obstacles[i].width;
       double y_max = obstacles[i].y + obstacles[i].height;
       Point a2 = {x_min,y_min};
       Point b2 = {x_max,y_min};
       Point c2 = {x_max,y_max};
       Point d2 = {x_min,y_max};

       double alpha = atan( tan( (y_min - y)/(x_min - x) ) );
       double dis = sqrt( ((x-x_min)*(x-x_min)) + ((y-y_min)*(y-y_min)) );
       double x_min2 = dis * cos(alpha - theta);
       double y_min2 = dis * sin(alpha - theta);


        if((x >= x_min) && (x <= x_max) && (y <= y_max) && (y >= y_min)) {
            result = false;
            break;
          } else if(x_min2 >= r_xmin && x_min2 <= r_xmax && y_min2 >= r_ymin && y_min2 <= r_ymax){
			result = false;
			break;


         }  else if ( judge(a,b,a2,b2)||judge(a,b,b2,c2)||judge(a,b,c2,d2)||judge(a,b,d2,a2)
                    ||judge(b,c,a2,b2)||judge(b,c,b2,c2)||judge(b,c,c2,d2)||judge(b,c,d2,a2)
                    ||judge(c,d,a2,b2)||judge(c,d,b2,c2)||judge(c,d,c2,d2)||judge(c,d,d2,a2)
	            ||judge(a,d,a2,b2)||judge(a,d,b2,c2)||judge(a,d,c2,d2)||judge(a,d,d2,a2)) {
                              result = false;
                              break;
                    }
    
    }  
    return result;
}

// Add any custom debug / development code here.  This code will be executed instead of the
// statistics checker (Project2.cpp).  Any code submitted here MUST compile, but will not be graded.
void debugMode(const std::vector<Robot>& /*robots*/, const std::vector<Rectangle>& /*obstacles*/, const std::vector<bool>& /*valid*/)
{
}
