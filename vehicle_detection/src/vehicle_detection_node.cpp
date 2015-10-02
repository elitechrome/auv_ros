#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "visualization_msgs/Marker.h"
#include <clothoid_msgs/katech_KCAN.h>
#include <clothoid_msgs/VehicleDetectStatus.h>

float car_posx;
float car_posy;
float vel=0;
bool tracking_int=false;
bool tracking_con=false;

class VehicleDetectNode{

public:

   ros::NodeHandle n_;
   ros::Subscriber laser_sub_;
   ros::Subscriber carmsg_sub_;
   ros::Publisher marker_pub_;
   ros::Publisher plot_pub_;

private:
   clothoid_msgs::katech_KCAN vehicle_status;
   clothoid_msgs::VehicleDetectStatus plot;
public:
   VehicleDetectNode(ros::NodeHandle n) :
   n_(n)
   {
       marker_pub_ = n_.advertise<visualization_msgs::Marker>("visualization_marker", 1);
       plot_pub_ = n_.advertise<clothoid_msgs::VehicleDetectStatus>("recognition_status", 1);
       laser_sub_ = n_.subscribe<sensor_msgs::LaserScan>("lidar/scan", 1, &VehicleDetectNode::scanCallback,this);
       carmsg_sub_ = n_.subscribe<clothoid_msgs::katech_KCAN>("vehicle_status" , 1, &VehicleDetectNode::carmsgCallback, this);
   }

   void carmsgCallback(const clothoid_msgs::katech_KCANConstPtr& msg_in)
   {
       vehicle_status.CurrentVehicleSpeed = msg_in.get()->CurrentVehicleSpeed;
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
           if (ranges[i]==0)
           {

           }
           x[i]=ranges[i]*cos(ang_min+ang_res*i);
           y[i]=ranges[i]*sin(ang_min+ang_res*i);

       }



       float xyz=ranges[1080];
       int count=0;
       int ind=0;
       int se=-1;

//         // edge detection
       for(int i=0;i<1081;i++)
       {
           if((i<1080)&&(ranges[i]!=0))
           {
               float dis=ranges[i+1]-ranges[i];
               dis=dis*dis;
               if(dis<0.25)
               {

                   if(ind==0)
                   {
                       edges[i]=1;
                       se=i;
                       ind=1;
                   }
                   count++;
               }
               else if(ind==1)
               {
                  if(count>2)
                  {
                      edges[i]=3;
                      count=0;
                      ind=0;
                      se=-1;
                  }
                  else
                  {
                      count=0;
                      ind=0;
                      if(se>0)
                      {
                      edges[se]=0;
                      }
                      se=-1;
                  }
               }
              }
           else
           {
               if(count>2)
               {
                   edges[i]=3;
                   count=0;
                   ind=0;
                   se=-1;
               }
               else
               {
                   count=0;
                   ind=0;
                   if(se>0)
                   {
                   edges[se]=0;
                   }
                   se=-1;
               }
           }


       }
       float disx=0;
       float disy=0;\
       float disxy=0;
       int j=0;

       for(int i=0;i<1081;i++)
       {
           if(edges[i]==3)
           {

               j=i;
               while(j<=1080)
               {
                   j++;

                  if(edges[j]==1)
                  {

                      disx=x[j]-x[i];
                      disx=disx*disx;
                      disy=y[j]-y[i];
                      disy=disy*disy;
                      disxy=disx+disy;
                      if(disxy<25)
                      {
                          edges[i]=0;
                          edges[j]=0;
                          for(int m=i;m<=j;m++)
                          {
                              x[m]=(x[i]+x[j])/2;
                              y[m]=(y[i]+y[j])/2;
                          }

                      }

                  }
               }
           }
       }


       // car candidate
       int start= 0;

       int midpoint[1081]={0};
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

               if((disxy<=9)&&(disxy>=2.25)&&(disx<=0.25))
               {
                   midpoint[midpnum]=(start+i)/2;
                   midpnum++;
               }


           }
       }
       float *car_can_x = new float[midpnum];
       float *car_can_y = new float[midpnum];

       //tracking - initialize
       int data_index;
       int k=0;
       for(int i=0; i<midpnum; i++)
       {
           data_index=midpoint[i];

           car_can_x[i]= x[data_index];
           car_can_y[i]= y[data_index];

       }


       //tracking - calculate velocity & distance
       if(tracking_int==false)
       {

           for(int i=0; i<midpnum; i++)
           {

               disx=car_can_x[i];
               disy=car_can_y[i]*car_can_y[i];
               if((disx<60)&&(disy<2.25))
               {
                   car_posx=car_can_x[i];
                   car_posy=car_can_y[i];

                   tracking_con=true;
                   break;

               }

           }
       }

       float R_velocity =0;
       if(tracking_con==true)
       {
           tracking_con=false;
           for(int i=0; i<midpnum; i++)
           {
               disx=car_posx-car_can_x[i];
               disy=car_posy-car_can_y[i];
               disxy=disx*disx+disy*disy;
               if(disxy<0.0144)
               {
                   car_posx=car_can_x[i];
                   car_posy=car_can_y[i];
                   tracking_con=true;

                   R_velocity=disx*25;
                   break;

              }

           }

       }



       //lowpass filter for velocity

       float Tf=5;
       float Ts=0.1;
       float a=Ts/(Tf+Ts);

      R_velocity=(1-a)*vel+a*R_velocity;
      vel=R_velocity;

      float V_des;
      float temp;


      if(tracking_con==true)
      {


        //calculate V_des

        float V_max=17;
        float t_final=6;
        float R_min=8;
        float alpha=1;
        float beta=0.5;

        float V= vehicle_status.CurrentVehicleSpeed/3.6;// Vehicle velocity
        float RV=R_velocity;
        float R= car_posx;       
        float V_f=V+RV;
        float RV_des;

        float R_final=R_min+V*beta;

        if(R<R_min)
        {
            RV_des=(R_final-R)*alpha;
        }
        else
        {

            temp=((R-R_final)*2*V_max/t_final);
            if(temp<0){temp=0;}
            RV_des= -sqrt(temp);
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


        V_des=V_des*3.6;

      }
      else
      {
          V_des=25;
      }



      ROS_INFO("V_des: %f , R: %f Temp :%f \n", V_des,car_posx,temp);



       delete[] car_can_x;
       delete[] car_can_y;









       visualization_msgs::Marker marker;
       marker.header.frame_id = "/laser";
       marker.header.stamp = ros::Time::now();

       marker.ns = "basic_shapes";
       marker.id = 0;

       marker.type = visualization_msgs::Marker::CUBE;

       marker.action = visualization_msgs::Marker::ADD;

       marker.pose.position.x = car_posx;
       marker.pose.position.y = car_posy;
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
       marker.color.a = 1.0f;

       marker.lifetime = ros::Duration();
       marker_pub_.publish(marker);

       plot.CruizeVehicleSpeed = V_des;
       //plot.ForwardVehicleSpeed;
       plot_pub_.publish(plot);

   }
};

int main(int argc, char** argv)
{

 ros::init(argc, argv, "vehicle_detection_node");
 ros::NodeHandle n;
 VehicleDetectNode lsp(n);
 ROS_INFO("vehicle_detection_node initialized.\n");
 ros::spin();

 return 0;
}
