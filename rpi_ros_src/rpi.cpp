#include "ros/ros.h"
#include "forbot_rpi/Ramka.h"
#include "wiringPi.h"
#include "wiringSerial.h"
#include "errno.h"

int servo1 = 4, servo2 = 5;
int descriptor=0,i=0, servostart;


void myCallback(const forbot_rpi::Ramka::ConstPtr& msg)
{

        for(i=0; i<10; i++)
        {
                serialPutchar(descriptor,msg->ramka_byte[i]);
                ROS_INFO("%d",msg->ramka_byte[i]);
                delay(3);
                serialFlush(descriptor);

        }
}


int main(int argc, char** argv)
{
        //Inicjacja wiringPi
        if (wiringPiSetup() == -1)
        {
                ROS_INFO("%s","Nie mozna wystartowac wiringPi!\n");
                return 1;
        }
        descriptor = serialOpen("/dev/serial0", 115200);
        if(descriptor<0)
       {
                ROS_INFO("%s %s", "Nie otworzono UART\n",strerror(errno));
        }

        //Inicjacja w systemie ROS
        ros::init(argc, argv, "rpi_main");

        //Uchwyt do node'a
        ros::NodeHandle nh_;

ROS_INFO("%s", "Otworzono UART\n");

        //Publisher
        ros::Subscriber sub_rpi = nh_.subscribe<forbot_rpi::Ramka>("ramka_state",10, myCallback);

        ros::spin();

        return 0;

