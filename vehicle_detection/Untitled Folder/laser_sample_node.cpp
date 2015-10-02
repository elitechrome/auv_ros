 #include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "visualization_msgs/Marker.h"

float car_posx;
float car_posy;
float vel=0;
bool tracking_int=false;
bool tracking_con=false;

class LaserScanProcessor{

public:

    ros::NodeHandle n_;
    ros::Subscriber laser_sub_;
    ros::Publisher marker_pub_;

    LaserScanProcessor(ros::NodeHandle n) :
    n_(n)
    {
        marker_pub_ = n_.advertise<visualization_msgs::Marker>("visualization_marker", 1);
        laser_sub_ = n_.subscribe<sensor_msgs::LaserScan>("lidar/scan", 1, &LaserScanProcessor::scanCallback,this);
    }

    void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
    {
        //Write down codes
        //http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html
        //http://wiki.ros.org/rviz/Tutorials/Markers%3A%20Basic%20Shapes
	ROS_INFO("laser data received.\n");

        int edges[1081]={0};
        float ang_res = scan_in.get()->angle_increment;
        float ang_min= scan_in.get()->angle_min;
        std::vector<float> ranges = scan_in.get()->ranges;

        // coordinates trasform
        float x[1081]={0};
        float y[1081]={0};
        for(int i=0;i<1081;i++)
        {
            x[i]=ranges[i]*sin(ang_min+ang_res*i);
            y[i]=ranges[i]*cos(ang_min+ang_res*i);

        }

        ROS_INFO("%f,%f, %f %f,%f, \n",x[541],ranges[541],ang_res, ang_min, scan_in.get()->angle_max);


        float xyz=ranges[1080];
        int count=0;
        int ind=0;
        int se=0;

//         // edge detection
        for(int i=0;i<1081;i++)
        {
            if(i<1080)
            {
                float dis=ranges[i+1]-ranges[i];
                dis=dis*dis;
                if(dis<0.09)
                {

                    if(ind==0)
                    {
                        edges[i]=1.0;
                        se=i;
                        ind=1;
                    }
                    count++;
                }
                else if(ind==1)
                {
                   if(count>6)
                   {
                       //edges[i]=3.0;
                       count=0;
                       ind=0;
                   }
                   else
                   {
                       count=0;
                       ind=0;
                       edges[se]=0;
                   }
                }
               }
            else
            {
                if(count>6)
                {
                    //edges[1]=3;
                    count=0;
                    ind=0;
                }
                else
                {
                    count=0;
                    ind=0;
                    edges[se]=0;
                }
            }


        }


        // car candidate
        int start= 0;
        float disx=0;
        float disy=0;\
        float disxy=0;
        int midpoint[1081];
        int midpnum=0;
        for(int i=0; i<1081; i++)
        {
            if(edges[i]==1)
            {
                start=i;
            }
            else if(edges[i]==3)
            {
                disx=x[start]-x[i];
                disx=disx*disx;
                disy=y[start]-y[i];
                disy=disy*disy;
                disxy=disx+disy;

                if((disxy>=0.25)&&(disxy<=25))
                    midpoint[midpnum]=(start+i)/2;
                    midpnum++;


            }
        }
        float *car_can_x = new float[midpnum];
        float *car_can_y = new float[midpnum];

        //tracking - initialize
        for(int i=0; i<midpnum++; i++)
        {
            car_can_x[i]= x[midpoint[i]];
            car_can_y[i]= y[midpoint[i]];

        }


        //tracking - calculate velocity & distance
        if(tracking_int==false)

            for(int i=0; i<midpnum++; i++)
            {

                disx=car_can_x[i];
                disy=car_can_y[i]*car_can_y[i];
                if((disx<40)&&(disy<9))
                {
                    car_posx=car_can_x[i];
                    car_posy=car_can_y[i];

                    tracking_con=true;
                    break;

                }

            }
        float dist=0;
        float velocity =0;
        if(tracking_con==true)
        {
            tracking_con=false;
            for(int i=0; i<midpnum++; i++)
            {
                disx=car_posx-car_can_x[i];
                disy=car_posx-car_can_y[i];
                disxy=disx*disx+disy*disy;
                if(disxy<0.12)
                {
                    car_posx=car_can_x[i];
                    car_posy=car_can_y[i];
                    tracking_con=true;
                    dist=disy;
                    velocity=disy*25;
                    break;
               }
            }

        }

        if(tracking_con==false)
        {
            dist=100;
            velocity=17;
        }

        //lowpass filter for velocity

        float Tf=5;
        float Ts=0.1;
        float a=Ts/(Tf+Ts);

       velocity=(1-a)*vel+a*velocity;


       //calculate V_des

       float V_max=17;
       float t_final=6;
       float R_min=8;
       float alpha=1;
       float beta=0.5;

       float V;// Vehicle velocity
       float RV=velocity;
       float R=dist;

       float RV_des;
       float V_des;

       float V_f=V+RV;
       float R_final=R_min+V*beta;

       if(R<R_min)
       {
           RV_des=(R_final-R)*alpha;
       }
       else
       {
           RV_des= -sqrt(((R-R_final)*2*V_max/t_final));
       }

       V_des=V_f-RV_des;

       if(V<(V_des-5))
       {
           V_des=V_max;
       }

       if (V_des<0)
       {
           V_des=0;
       }
       else if(V_des>V_max)
       {
            V_des=V_max;
       }

       if(R==0)
       {
            V_des=V_max;
       }


        delete[] car_can_x;
        delete[] car_can_y;









        visualization_msgs::Marker marker;
        marker.header.frame_id = "/laser";
        marker.header.stamp = ros::Time::now();

        marker.ns = "basic_shapes";
        marker.id = 0;

        marker.type = visualization_msgs::Marker::CUBE;

        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = 0;
        marker.pose.position.y = 0;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = 1.0;
        marker.scale.y = 1.0;
        marker.scale.z = 1.0;

        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;

        marker.lifetime = ros::Duration();
	marker_pub_.publish(marker);
    }
};

int main(int argc, char** argv)
{

  ros::init(argc, argv, "laser_sample_node");
  ros::NodeHandle n;
  LaserScanProcessor lsp(n);
  ROS_INFO("laser_sample_node initialized.\n");
  ros::spin();

  return 0;
}
