#include <ros/ros.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>

#define xi -3
#define yi 1.5
#define xi1 4
#define yi1 1.5
#define delta 0.4

geometry_msgs::Twist v;

ros::Publisher v_pub;

double normalizeAngle(double angle)
{
    if(angle > M_PI){
        return(angle - 2*M_PI);
    }else if(angle < -M_PI){
        return (angle + 2*M_PI);
    }else{
        return angle;
    }
}

void odomCallback(const nav_msgs::OdometryConstPtr &msg){
    
    double px, py, pyaw, tetha, tethau, betha, tethad, dtethad, xc, yc, r, ru, d;
    
    geometry_msgs::Quaternion qt;
    
    qt = msg->pose.pose.orientation; 
    
    pyaw = tf::getYaw(qt);
    
    px = msg->pose.pose.position.x;
    py = msg->pose.pose.position.y;
    
    //tethau = atan2 wi e p
    tethau = atan2((yi - py),(xi - px));
    //tetha = atan2 wi+1 e wi
    tetha = atan2((yi1 - yi),(xi1 - xi));
    
    betha = tetha - tethau;
    std::cout << "betha: " << betha << std::endl;
    
    ru = sqrt((px - xi)*(px - xi) + (py - yi)*(py - yi));
    std::cout << "ru: " << ru << std::endl;
    
    r = ru*sqrt(1 - sin(betha)*sin(betha));
    std::cout << "r: " << r << std::endl;
    
    //calculando o ponto c
    xc = (r + delta)*cos(tetha) + xi;
    yc = (r + delta)*sin(tetha) + yi;
    
    d = sqrt(((xc - px)*(xc - px)) + ((yc - py)*(yc - py)));
    
    //tetha entre p e c
    tethad = atan2((py - yc),(px - xc));
    std::cout << "tethad: " << tethad << std::endl;
    
    //deltatetha: diferença entre tetha e a orientação
    dtethad = normalizeAngle(pyaw - tethad);
    std::cout << "dtetha: " << dtethad << std::endl;
    
    if(d > 0.1){
            v.linear.x = 1.5;
            v.angular.z = 0.8*dtethad;
            if(v.angular.z > 1){
                v.angular.z = 1;
            }
            else if(v.angular.z < -1){
                v.angular.z = -1;
            }else{  
            }
    }
    std::cout << "xc: " << xc << std::endl;
    std::cout << "yc: " << yc << std::endl;
    
    v_pub.publish(v);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "carrot_node");
    
    ros::NodeHandle carrot_node;
    
    v_pub = carrot_node.advertise<geometry_msgs::Twist>("Velocity", 1);
    
    ros::Subscriber carrot_sub = carrot_node.subscribe("/vrep/vehicle/odometry", 1, odomCallback);
    
    ros::spin();
}