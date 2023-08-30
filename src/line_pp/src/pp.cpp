#include  "ros/ros.h"
#include "iostream"
/*#include "fsd_common_msgs/ASENSING.h"
#include "fsd_common_msgs/HUAT_rviz.h"
#include "fsd_common_msgs/ins_p2.h"
#include "fsd_common_msgs/ControlCommand.h"*/
#include "huat_msgs/HUAT_ASENSING.h"
#include "huat_msgs/HUAT_ins_p2.h"
#include "huat_msgs/HUAT_ControlCommand.h"
#include <fstream>
#define l 1.55                                                //车轴距
#define pi 3.14159265358979
using namespace std;
vector<vector<double>> points; 
vector<vector<double>>re;
vector<double>idx1;
class pp
{
    public:
    pp(ros::NodeHandle code);
    ros::NodeHandle nh;
    ros::Subscriber   sub_north;
    huat_msgs::HUAT_ASENSING my_ins;
    float longError,longCurrent,longLast,currentSpeed;
    double  enu_xyz[3];
    double first_lat=0;
    double first_lon=0;
    double first_alt = 0;
    double ex, ey, ez, gheading;
	double gx, gy, gz;
	double a=0,b=0,c=0;
	int flag=0;
	int label_for_pp = 0; 
	vector<double> refx;
	vector<double> refy;
    huat_msgs::HUAT_ControlCommand  cmd;
    ros::Publisher pub;
	void cal_point(const string s);
    void northcallback(const huat_msgs::HUAT_ins_p2 north);
	void  GeoDetic_TO_ENU(double lat, double lon, double h, double lat0, double lon0, double h0, double enu_xyz[3]);
    void execute();
};
pp::pp(ros::NodeHandle code)
{
    nh=code;
	ros::Duration    du(2);
    du.sleep();
	cal_point("/home/tb/line_creat2/duan.txt");
}
void pp::cal_point(const string s)
{
	ROS_INFO("构造函数进入");
	ifstream infile;
	infile.open(s);
	if (!infile)
	{
		cout << "Unable to open file";
		exit(1);
	}
	double temp1,temp2;
	while(infile)
		{
			infile >>temp1>>temp2;
			refx.push_back(temp1);
			refy.push_back(temp2);
		}
}
void pp::northcallback(const huat_msgs::HUAT_ins_p2 north)
{
    //cout<<"进入ins回调"<<endl;
    my_ins.east_velocity=north.Ve;
	my_ins.north_velocity =north.Vn;
    my_ins.ground_velocity =north.Vu;
    if(flag==0)
	{
           first_lat=north.Lat;
		   first_lon=north.Lon;
		   first_alt=north.Altitude;
		   flag=1;
		   return;
	}
	double gheading0 = (north.Heading)*pi/180;    // 这里做了转换转为了弧度		
	gheading = (gheading0 > pi) ? (gheading0 - 2 * pi) : (gheading0 < -pi) ? (gheading0 + 2 * pi) : gheading0;

    GeoDetic_TO_ENU( (north.Lat)*pi /180, (north.Lon)*pi /180, north.Altitude, 
														first_lat*pi /180,first_lon*pi /180,first_alt, &enu_xyz[0]);  
	ofstream m;
    m.open("/home/tb/line_creat2/gx.txt",ios::out|ios::app);      
	m << std::setprecision(20);           
    m<<"  "<<gx<<"   "<<gy<<endl;
	m.close();
	execute();
   m << std::setprecision(20);           
   /*m.open("/home/tang/line_creat2/trick.txt",ios::out|ios::app);     
   m<<a<<"    "<<b<<"     "<<c<<endl; 
   m.close();
   a=0,b=0,c=0;*/
}
void pp::GeoDetic_TO_ENU(double lat, double lon, double h, double lat0, double lon0, double h0, double enu_xyz[3])
{
     double a, b, f, e_sq;
	 a = 6378137;
	 b = 6356752.3142;
	 f = (a - b) / a;e_sq = f * (2 - f);
	// 站点（非原点）
	double lamb, phi, s, N, sin_lambda, cos_lambda, sin_phi, cos_phi, x, y, z;
	lamb = lat;  
	phi = lon;
	s = sin(lamb);
	N = a / sqrt(1 - e_sq * s * s);

	sin_lambda = sin(lamb);
	cos_lambda = cos(lamb);
	sin_phi = sin(phi);
	cos_phi = cos(phi);

	x = (h + N) * cos_lambda * cos_phi;
	y = (h + N) * cos_lambda * sin_phi;
	z = (h + (1 - e_sq) * N) * sin_lambda;
	// 原点坐标转换
	double lamb0, phi0, s0, N0, sin_lambda0, cos_lambda0, sin_phi0, cos_phi0, x0, y0, z0;
	lamb0 = lat0;
	phi0 = lon0;
	s0 = sin(lamb0);
	N0 = a / sqrt(1 - e_sq * s0 * s0);

	sin_lambda0 = sin(lamb0);
	cos_lambda0 = cos(lamb0);
	sin_phi0 = sin(phi0);
	cos_phi0 = cos(phi0);

	x0 = (h0 + N0) * cos_lambda0 * cos_phi0;
	y0 = (h0 + N0) * cos_lambda0 * sin_phi0;
	z0 = (h0 + (1 - e_sq) * N0) * sin_lambda0;
	// ECEF 转 ENU
	double xd, yd, zd, t;
	xd = x - x0;
	yd = y - y0;
	zd = z - z0;
	t = -cos_phi0 * xd - sin_phi0 * yd;

	enu_xyz[0] = -sin_phi0 * xd + cos_phi0 * yd;
	enu_xyz[1] = t * sin_lambda0 + cos_lambda0 * zd;
	enu_xyz[2] = cos_lambda0 * cos_phi0 * xd + cos_lambda0 * sin_phi0 * yd + sin_lambda0 * zd;

	gx = enu_xyz[0];
	gy = enu_xyz[1];
	gz = enu_xyz[2];
}
void pp::execute()
{
    // 检查 refx 和 refy 的大小
    if (refx.size() != refy.size()) {
        ROS_WARN("refx and refy have different sizes!");
        return;
    }
    // 如果 refx 和 refy 为空，则初始化
    if (refx.empty() || refy.empty()) {
        ROS_WARN("refx and/or refy is empty, cannot execute!");
        return;
    }
    // 在此执行您的代码
    int idx = 0;
	int temp_idx = label_for_pp;
	float delta_max = 0.4;

	float ld1 =12;     

	for (int i = label_for_pp; i <refx.size(); i++)             // 这里1058要变
	{
		float dis = sqrt(pow(gy - refy[i], 2) + pow(gx - refx[i], 2));
		if (dis <= ld1)
		{
			temp_idx = i;		
		}
		else
		{
			break;
		}
	}
	label_for_pp = temp_idx;
	idx = temp_idx;
 //转角
	cout<<refx[idx]<<"    "<<refy[idx]<<"   "<<idx<<endl;
	float alpha = atan2(refx[idx] - gx,refy[idx] - gy) - gheading;                                // -atan2  
	float delta_pp = atan2(2 * l * sin(alpha) / ld1, 1.0);
	float delta = delta_pp;
    delta = max(min(delta_max, delta), -delta_max);
	cmd.steering_angle.data= delta;                                         // delta

//速度
	currentSpeed = sqrt(pow(my_ins.east_velocity,2)+pow(my_ins.north_velocity,2)+pow(my_ins.ground_velocity,2));
	//cout << "currentSpeed:=" << currentSpeed<< endl;
	longError = 3.0 - currentSpeed;                                     //表示期望速度1.0跟当前速度的差值 
    longCurrent = 0.5*longError + longLast;                   //表示下一时刻机器人应该达到的速度
    longLast = longCurrent;                                                  //longlast上一时刻的数度
    ///////////////// 这里到时候会跟着速度改变而改变//////////////////////////////////////////////
    if(currentSpeed >2){
            longCurrent = 10;
        }

    if(longCurrent > 30){
            longCurrent = 30;
        }

    if (currentSpeed <= 0.3){
            longCurrent = 50;
        }
	cmd.throttle.data = int(longCurrent);                       //踏板率
    //cout<<"cmd.throttle.data =  "<<cmd.throttle.data<<endl;

	pub.publish(cmd);
	if (idx >= refx.size() - 1) {
            cmd.steering_angle.data = 0;
			cmd.throttle.data = 0;
			cmd.racing_status = 4;
            pub.publish(cmd);
            cout << "已经到达离线轨迹的最后一个点，停止跟踪！" << endl;
            ros::shutdown();
    }	
}
int main(int argc, char  *argv[])
{
	ofstream m;
    //m.open("/home/tang/line_creat2/refx_.txt",ios::out);      
	//m.close();
	m.open("/home/tb/line_creat2/gx.txt",ios::out);      
	m.close();
	m.open("/home/tb/line_creat2/trick.txt",ios::out);      
	m.close();
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"line_pp");
    ros::NodeHandle nodehandle("~");
    pp line_pp(nodehandle);
	 line_pp.sub_north=nodehandle.subscribe("/insMsg",1,&pp::northcallback,&line_pp);
    line_pp.pub=nodehandle.advertise<huat_msgs::HUAT_ControlCommand>("/control/pure_pursuit/control_command", 1);
	ros::spin();
    return 0;
}
