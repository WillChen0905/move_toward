#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <math.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <algorithm>
using namespace std;

class Object_detect
{
public:
    Object_detect(ros::NodeHandle &node) :
        nh_(node)
    {
        vel_pub = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
        stop_pub = nh_.advertise<std_msgs::Bool>("nav_goal", 10);
        sub_bool = nh_.subscribe("tracker",10, &Object_detect::trackerCB, this);
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher vel_pub;
    ros::Publisher stop_pub;
    ros::Subscriber sub_cam;
    ros::Subscriber sub_bool;
    tf::TransformBroadcaster br_;
    tf::TransformListener listener_;
    geometry_msgs::Point init;
    vector<int> buf;
    ////////////////
    static bool cmp_dist(geometry_msgs::Quaternion a, geometry_msgs::Quaternion b){
        return a.w < b.w;
    }
    ////////////////


    void move2person (const geometry_msgs::Point Position){
        geometry_msgs::Twist vel;
        std_msgs::Bool stop;
        double vxMax = 0.15, vYaw = 0.2;
        double thi = atan2(Position.y, Position.x)*180/M_PI;
        cout << "x=" << Position.x << endl;
        cout << "y=" << Position.y << endl;
        cout << "thi=" << thi << endl;
        if (thi > 5){
            vel.angular.z = vYaw;
            vel_pub.publish(vel);
        }else if( thi < -5){
            vel.angular.z = -vYaw;
            vel_pub.publish(vel);
            vel_pub.publish(vel);
        }else if(Position.x > 1.0){
            vel.linear.x = vxMax;
            vel_pub.publish(vel);
        }else if(Position.x > 0.8 && Position.x < 1.0){
          vel.linear.x = 0;
          for(int i=0; i<10; i++){
            vel_pub.publish(vel);
            ros::Duration(0.1).sleep();
          }
          sub_cam.shutdown();
          stop.data = true;
          stop_pub.publish(stop);
        }else if (Position.x < 0.8){
            vel.linear.x = -0.15;
        }
    }

    void TransForm (const geometry_msgs::Point Person1, string frame_id){
        tf::StampedTransform transform;
        geometry_msgs::Transform cam2obj;
        geometry_msgs::Transform base2obj;
        tf::Transform cam2obj_tf;
        geometry_msgs::Point Position;
        cam2obj.translation.x = Person1.x;
        cam2obj.translation.y = Person1.y;
        cam2obj.translation.z = Person1.z;
        cam2obj.rotation.w = 1;
        tf::transformMsgToTF(cam2obj,cam2obj_tf);
        string errmsg;
        if(!listener_.waitForTransform("base_footprint", frame_id, ros::Time(0), ros::Duration(0.1), ros::Duration(0.02), &errmsg)){
             ROS_ERROR_STREAM("Unable to get pose from TF: " << errmsg);
        }else {
            try{
              listener_.lookupTransform("base_footprint", frame_id, ros::Time(0), transform);
            }
            catch (tf::TransformException ex){
              ROS_ERROR("%s",ex.what());
              ros::Duration(1.0).sleep();
            }
          }

        tf::Transform base2obj_tf = static_cast<tf::Transform>(transform) * cam2obj_tf;
        tf::transformTFToMsg(base2obj_tf,base2obj);

        br_.sendTransform(tf::StampedTransform(base2obj_tf, ros::Time::now(), "base_footprint", "objective"));

        Position.x = base2obj.translation.x;
        Position.y = base2obj.translation.y;
        move2person(Position);

    }


    void camerafilterCB (const geometry_msgs::PoseArray::ConstPtr &Person){
        geometry_msgs::Quaternion distance;
        vector<geometry_msgs::Quaternion> Person0;
        geometry_msgs::Point Person1;
        string frame_id = Person->header.frame_id;

        int sum = 0;
        if(buf.size()<10){
            if(Person->poses.size()==0)
            {
                buf.push_back(0);
            }else{
                buf.push_back(1);
            }
        }else{
            if(Person->poses.size()==0)
            {
                buf.erase(buf.begin());
                buf.push_back(0);
                for(int i=0; i<buf.size(); i++){
                    if(buf[i]=1){
                        sum=sum+1;
                    }
                }
                if(sum>7){
                    if(Person->poses.size()!=0){
            cout << "check point segment" << endl;
                        for(int i=0; i<Person->poses.size(); i++){
                            if(pow(Person->poses[i].position.x,2)+pow(Person->poses[i].position.y,2) < 25){
                                distance.x=Person->poses[i].position.x;
                                distance.y=Person->poses[i].position.y;
                                distance.z=Person->poses[i].position.z;
                                distance.w=sqrt(pow(Person->poses[i].position.x,2)+pow(Person->poses[i].position.z,2));
                                Person0.push_back(distance);
                            }
                        }
                        sort(Person0.begin(), Person0.end(), cmp_dist);
                        Person1.x = Person0[0].x;
                        Person1.y = Person0[0].y;
                        Person1.z = Person0[0].z;
                        TransForm(Person1,frame_id);
                    }
                }
            }else{
                buf.erase(buf.begin());
                buf.push_back(1);
                for(int i=0; i<buf.size(); i++){
                    if(buf[i]=1){
                        sum=sum+1;
                    }
                }
                if(sum>7){
                    if(Person->poses.size()!=0){
                        for(int i=0; i<Person->poses.size(); i++){
                            if(pow(Person->poses[i].position.x,2)+pow(Person->poses[i].position.y,2) < 25){
                                distance.x=Person->poses[i].position.x;
                                distance.y=Person->poses[i].position.y;
                                distance.z=Person->poses[i].position.z;
                                distance.w=sqrt(pow(Person->poses[i].position.x,2)+pow(Person->poses[i].position.z,2));
                                Person0.push_back(distance);
                            }
                        }
                        sort(Person0.begin(), Person0.end(), cmp_dist);
                        Person1.x = Person0[0].x;
                        Person1.y = Person0[0].y;
                        Person1.z = Person0[0].z;
                        TransForm(Person1,frame_id);
                    }
                }
            }
        }

    }

    void trackerCB (const std_msgs::Bool::ConstPtr &tracker){
        if(tracker->data){
            sub_cam = nh_.subscribe("ai_targets",10, &Object_detect::camerafilterCB, this);
        }
    }
};


int main(int argc, char **argv) {
  ros::init(argc, argv, "move_toward");
  ros::NodeHandle nh;
  Object_detect OD(nh);
  ros::spin();
  return 0;
}
