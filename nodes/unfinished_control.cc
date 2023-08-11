#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <cv_bridge/cv_bridge.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <iostream>
#include <vector>
#include <typeinfo>
#include <unistd.h>
#include <algorithm>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc_c.h>

using namespace std::chrono_literals;

using namespace cv;
using namespace std;

using std::placeholders::_1;

const int N = 960;
const int M = 540;

std::vector<double> x;
std::vector<double> y;
double curvature=0;
int count_first_time=0;


double vmax=0.3;  //Assum the maximum  linear speed is 0.3 m/s
double omega_output;
/* This example creates a subclass of Node and uses std::bind() to register a* member function as a callback from the timer. */


double calaccel(double kp,double kd,double distance,double curvature,double angle){

// double deri;
// double velocity_now,velocity_next;
// double velocity_p;
double velocity_p_d;
// double accel;

// double angle_as;
double curvature_as;
double distance_as;
distance_as=distance*0.001; //Convert the number of pixels to value in meters 
curvature_as=curvature*1000;

 // if (angle<0){angle_abs=-angle;}
 // else{angle_abs=angle;}

 // if (curvature<0){curvature_abs=-curvature*1000;}else{curvature_abs=curvature*1000;} 
 // velocity_p=abs(-kp*distance_abs);

 // if (angle_abs<=0.1){velocity_p=0;}
 // cout<<"velocity_p"<<velocity_p<<endl;

  velocity_p_d=-kp*distance_as-kp*kd*sin(angle)+curvature_as; //Calculate the path after proportional and differential control (curvature)
 // velocity_p_d=-kp*distance_as-kp*kd*sin(angle)+curvature_as;

  // if (abs(angle)<=0.1){velocity_p_d=0;}
  cout<<"velocity_p_d"<<velocity_p_d<<endl;

  double omega;
  omega=vmax*velocity_p_d;

  // if (angle<0){omega=-omega;}
  // if(abs(omega)>=5){omega=10;}
  
  omega_output=omega;  //Calculate the angular velocity
  cout<<"omega_output: "<<omega_output<<endl;  
  
  // deri=cos(angle_abs)*(-kp*distance_abs*pow(cos(angle_abs),2)/(1-curvature_abs*distance_abs)+sin(angle_abs)*(curvature_abs*sin(angle_abs)-kd*cos(angle_abs)))/(1-curvature_abs*distance_abs);
  // cout<<"deri"<<deri<<endl;

  // if(count_first_time==0){velocity_now=vmax; count_first_time=1; }

  // accel=pow(velocity_now,2)*curvature;

  // velocity_next=min(sqrt(abs(accel/deri)),vmax);
  // cout<<"accel: "<<accel<<endl;
  // cout<<"velocity_next: "<<velocity_next<<endl;
  // omega=accel/velocity_now;
  // cout<<"omega: "<<omega<<endl;
  // velocity_now=velocity_next;

  return velocity_p_d/1000;
}

void ShengJin(double a,double b,double c,double d,std::vector<double> &X123)  //ShengJin formula. It can be used to solve a function in the format of a*x^3+b*x^2+c*x+d=0
{
	double A=b*b-3*a*c;
	double B=b*c-9*a*d;
	double C=c*c-3*b*d;
	double f=B*B-4*A*C;
	double i_value;
	double Y1,Y2;
	if (fabs(A)<1e-6 && fabs(B)<1e-6)
	{
		X123.push_back(-b/(3*a));
		X123.push_back(-b/(3*a));
		X123.push_back(-b/(3*a));
	}
	else if (fabs(f)<1e-6) 
	{
		double K=B/A;
		X123.push_back(-b/a+K);
		X123.push_back(-K/2);
		X123.push_back(-K/2);
	}
	else if (f>1e-6) 
	{
		Y1=A*b+3*a*(-B+sqrt(f))/2;
		Y2=A*b+3*a*(-B-sqrt(f))/2;
		double Y1_value=(Y1/fabs(Y1))*pow((double)fabs(Y1),1.0/3);
		double Y2_value=(Y2/fabs(Y2))*pow((double)fabs(Y2),1.0/3);
		X123.push_back((-b-Y1_value-Y2_value)/(3*a));

		i_value=sqrt(3.0)/2*(Y1_value-Y2_value)/(3*a);
		if (fabs(i_value)<1e-1)
		{
			X123.push_back((-b+0.5*(Y1_value+Y2_value))/(3*a));
		}
	}
	else if (f<-1e-6)   
	{
		double T=(2*A*b-3*a*B)/(2*A*sqrt(A));
		double S=acos(T);
		X123.push_back((-b-2*sqrt(A)*cos(S/3))/(3*a));
		X123.push_back((-b+sqrt(A)*(cos(S/3)+sqrt(3.0)*sin(S/3)))/(3*a));
		X123.push_back((-b+sqrt(A)*(cos(S/3)-sqrt(3.0)*sin(S/3)))/(3*a));
	}
}

vector<double> mindis(double mat_k_2, double mat_k_0){

    double a0, a1,a2;
    double a,b,c,d;

    double deltax,deltay,distance;
    double angle;

    double only_one_y_123; 
    double calaccel_return;
   
    std::vector<double> X123; 
    std::vector<double> return_x_y; 

    a2=mat_k_2;
    a1=0;
    a0=mat_k_0;

    a=2*(pow(a2,2));  //Using the relation that the product of the slope of two perpendicular lines is -1 to derive the equation in the format of a*x^3+b*x^2+c*x+d=0
    b=3*a2*a1;
    c=pow(a1,2)-2*a2*480+1+2*a2*a0;
    d=-a1*480-160+a1*a0;

    cout<<"a: "<<a<<" b: "<<b<<" c: "<<c<<" d: "<<d<<endl;
    
    ShengJin(a,b,c,d,X123); //Solve this equation using the ShengJin formula

    cout<<"X123[0]: "<<X123[0]<<" X123[1]: "<<X123[1]<<" X123[2]: "<<X123[2]<<endl;

    if (X123[0]==0&&X123[1]==0&&X123[2]==0){ return_x_y.push_back(0); return_x_y.push_back(a0);}  //Choose the one that has physical meaning among the three possible solutions
    else if(X123[0]>0){ return_x_y.push_back(X123[0]); return_x_y.push_back(pow(X123[0],2)*a2+X123[0]*a1+a0);}
    else if(X123[1]>0){ return_x_y.push_back(X123[1]);return_x_y.push_back(pow(X123[1],2)*a2+X123[1]*a1+a0);}
    else { return_x_y.push_back(X123[2]);return_x_y.push_back(pow(X123[2],2)*a2+X123[2]*a1+a0);}
      
    cout<<"return_x_y[0]: "<<return_x_y[0]<< " return_x_y[1]: "<<return_x_y[1]<<endl;

    deltax=abs(return_x_y[0]-160);
	  deltay=abs(return_x_y[1]-480);
	    
    distance=sqrt(deltax*deltax+deltay*deltay); //Calculate the magnitude of the distance

    if((return_x_y[0]-160)*(return_x_y[1]-480)>=0)  //right //Add direction to distance: outer lane or inner lane
    {

      if((return_x_y[0]-160)>=0)  //outside
        {
          distance=-distance;
        }
      else
        { 
          distance=distance;
        }
    }
    
    else
    { 
      if((return_x_y[0]-160)>=0)  //inside
      {
        distance=-distance;
      }
      else
      {
        distance=distance;
      }
    }

    cout<<"distance"<<distance<<endl;

    curvature=2*a2/pow(sqrt(1+pow((2*a2*return_x_y[0]+a1),2)),3);   //Calculate the curvature
    cout<<"curvature"<<curvature<<endl;
    return_x_y.push_back(1/curvature);  //return_x_y[2]

    angle=atan(2*a2*return_x_y[0]+a1);  
    cout<<"angle"<<angle<<endl;

    double kp=0.7;
    double kd=0.3;
    calaccel_return=calaccel(kp,kd,distance,curvature,angle);
    return_x_y.push_back(1/calaccel_return);  //return_x_y[3]
    return return_x_y;
}

Mat polyfit(vector<Point>& in_point)  //The vertex of the quadratic function is supposed to be at the bottom of the Field of View. After image transformation, b should equal to 0 (y=a*x^2+b*x+c);
{
  int size = in_point.size();
  int x_num = 2;

  Mat mat_u(size, x_num, CV_64F); // Construct the matrices U and Y
  Mat mat_y(size, 1, CV_64F);

  for (int i = 0; i < mat_u.rows; ++i)
  {
    mat_u.at<double>(i, 0) = pow(in_point[i].x, 0);
    mat_u.at<double>(i, 1) = pow(in_point[i].x, 2);
  }

  for (int i = 0; i < mat_y.rows; ++i)
    mat_y.at<double>(i, 0) = in_point[i].y;

  Mat mat_k(x_num, 1, CV_64F); // matrix operations to obtain the coefficient matrix K
  mat_k = (mat_u.t()*mat_u).inv() * mat_u.t() * mat_y;
  cout << mat_k << endl;
  return mat_k;
}

class MinimalPublisher : public rclcpp::Node{
public:
  MinimalPublisher()
  : Node("trajectory_following"), count_(0)
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/csi_cam", 10, std::bind(&MinimalPublisher::process, this, _1));

    timer_ = this->create_wall_timer(
    200ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = geometry_msgs::msg::Twist();
    message.linear.x = vmax;   
    message.angular.z = -omega_output;
    //RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }

void process(const sensor_msgs::msg::Image::SharedPtr msg)
{

  Mat img = cv_bridge::toCvShare(msg, "bgr8")->image; //Convert the ROS message into a format that can be used by OpenCV
  
  Mat gray;
  Mat otsu;
  Mat morph;
  Mat eroded;

  cvtColor(img, gray, COLOR_BGR2GRAY);  //Convert the image into a single-channel grayscale image
  threshold(gray, otsu, 0, 255, THRESH_BINARY_INV + THRESH_OTSU); //Convert the image into binary format
  morphologyEx(otsu, morph, MORPH_OPEN, (9,9)); //First dilate and then erode the image
  
  auto kernel = getStructuringElement(MORPH_RECT, Size(20, 20));  
  erode(morph, eroded, kernel); //Erode the image

  Point p[5]; //Record the size of the original image
  p[0].x=0;
  p[0].y=0;
  p[1].x=960;
  p[1].y=0;
  p[2].x=0;
  p[2].y=540;
  p[3].x=960;
  p[3].y=540;   

  vector<Point2f> src(4); 
  src[0] = p[0];
  src[1] = p[1];
  src[2] = p[2];
  src[3] = p[3];

  vector<Point2f> dst(4); //Record the size of the image after scaling
  dst[0] = Point2f(0, 0);
  dst[1] = Point2f(960, 0);
  dst[2] = Point2f(380, 540);
  dst[3] = Point2f(580, 540);

  Mat trans = getPerspectiveTransform(src, dst);  //Obtain the Bird’s-eye view
  Mat transformed;
  warpPerspective(eroded, transformed, trans, Size(N, M), INTER_LINEAR);

  Mat cov_ker = (Mat_<double>(5, 5) << -1, -1, -1, -1, -1,
                                       -1,  0,  0,  0, -1,
                                       -1,  0, 16,  0, -1,
                                       -1,  0,  0,  0, -1,
                                       -1, -1, -1, -1, -1);
  Mat line;
  filter2D(transformed, line, -1 , cov_ker, Point(-1, -1), 0, 4); //Extract the information of the road edges

  std::vector<Point> pts; //Record all pixels information on the path
  for (int i = 0; i < 960; ++i)
    for (int j = 0; j < 540; ++j)
      if(line.at<bool>(j, i) == true) // j is row, j is y
        pts.push_back(Point((540-j), i));

  // int order = 2;
  // Mat mat_k = polyfit(pts, order);

  if (pts.size() == 0) //If no line is detected then the calculation will not be performed. To prevent the generation of empty matrix
  {
    return;
  }

  Mat mat_k = polyfit(pts); //Fit the path with a quadratic function

  // for (int i = 0; i < 540; i++)  //Draw the fitted line. All the codes for display are commented because it will slow down the system
	// {
  //   Point2d ipt;
  //   Point2d ipt_map;

  //   ipt.x = i;
  //   ipt_map.y=540-ipt.x;
  //   ipt.y = 0;
  //   ipt_map.x=ipt.y;    
  //   // for (int j = 0; j < order + 1; ++j)
  //   // {
  //   //   ipt.y += mat_k.at<double>(j, 0) * pow(i, j);
  //   //   ipt_map.x=ipt.y;   
  //   // }
  //   ipt.y += mat_k.at<double>(0, 0) * pow(i, 0);
  //   ipt.y += mat_k.at<double>(1, 0) * pow(i, 2);
  //   ipt_map.x=ipt.y;

  //   circle(line, ipt_map, 1, Scalar(150), -1);

	// }

    std::vector<double> X123_return;  
    Point2d X123_return_map;
	  X123_return=mindis(mat_k.at<double>(1, 0),mat_k.at<double>(0, 0));  //Calculate the intersection point, minimum distance, curvature and rotation angle

    // X123_return_map.y=540-X123_return[0];  //Draw the intersection point
    // X123_return_map.x=X123_return[1];
    // circle(line, X123_return_map, 5, Scalar(150), -1);

    // cout<<"X123_return[0]: "<<X123_return[0]<< " X123_return[1]: "<<X123_return[1]<<endl;  //Draw the vertical line from chosen point (480, 540-160) with the quadratic function itself
    // for (int i = 0; i < 540; i++)
    // {
    //   Point2d straight;
    //   Point2d straight_map;

    //   straight.x = i;
    //   straight_map.y=540-straight.x;
    //   straight.y = 0;
    //   straight_map.x=straight.y;    
    //   // for (int j = 0; j < order + 1; ++j)
    //   // {
    //   //   ipt.y += mat_k.at<double>(j, 0) * pow(i, j);
    //   //   ipt_map.x=ipt.y;   
    //   // }
    //   straight.y +=(X123_return[1]-480)*i/(X123_return[0]-160)+(X123_return[0]*480-X123_return[1]*160)/(X123_return[0]-160) ;
    //   straight_map.x=straight.y;   
    //   //cout<<"straight.x: "<<straight.x<< " straight.y: "<<straight.y<<endl;
    //   //cout<<"straight_map.x: "<<straight_map.x<< " straight_map.y: "<<straight_map.y<<endl;
    //   //circle(line, straight_map, 1, Scalar(150), -1);
      
    // }

    // Point2d circle_curvature;  //Draw the circle at the intersection point with the curvature calculated
    // if (X123_return[2]<0){X123_return[2]=-X123_return[2];}
    // circle_curvature.x=X123_return[1]-X123_return[2]*sin(atan((X123_return[1]-480)/(X123_return[0]-160)));
    // circle_curvature.y=540-(X123_return[0]-X123_return[2]*cos(atan((X123_return[1]-480)/(X123_return[0]-160))));
    // cout<<"circle_curvature.x: "<<circle_curvature.x<< " circle_curvature.y: "<<circle_curvature.y<< " X123_return[2]:"<<X123_return[2]<<endl;
    // circle(line, circle_curvature, X123_return[2], Scalar(150), 2);

    // cout<<"X123_return[3]: "<<X123_return[3]<<endl;  //Draw the planned trajectory
    // Point2d car;
    // car.x=480-X123_return[3]*sin(atan((X123_return[1]-480)/(X123_return[0]-160)));
    // car.y=540-(160-X123_return[3]*cos(atan((X123_return[1]-480)/(X123_return[0]-160))));
    // circle(line, car, X123_return[3], Scalar(150), 2);

    // imwrite("/ws/follow/my_test01/src/test_publish/src/test.png", line);
}

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  size_t count_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};
 
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
}
