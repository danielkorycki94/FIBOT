#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <learning_joy/Ramka.h>
#define PROG 0.1
#define VMAX 750
uint8_t servo1_temp = 90, servo2_temp = 90;

struct parametr
{
        float x_lin;
        float z_ang;
        float v_maks;
        int servo1;
        int servo2;
        int enable_drive;
        uint8_t servo_default;
        uint8_t led;
        uint8_t buzzer;
};

class Sterowanie
{
public:
  Sterowanie();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

  ros::NodeHandle nh_;

  int linear_, angular_, v_max_;
  int servo_x_, servo_y_, driver_alive_,servo_default_,led_,buzzer_;
  ros::Subscriber joy_sub_;
  ros::Publisher ramka_pub_;
};


Sterowanie::Sterowanie():
  linear_(1),
  angular_(0),
  v_max_(3),
  servo_x_(4),
  servo_y_(5),
  driver_alive_(0),
  servo_default_(1),
  led_(2),
  buzzer_(3)
{

  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &Sterowanie::joyCallback, this);
  ramka_pub_ = nh_.advertise<learning_joy::Ramka>("ramka_state", 10);
}

void Sterowanie::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
   parametr structparam;
   structparam.x_lin = joy->axes[angular_];
   structparam.z_ang = joy->axes[linear_];
   structparam.v_maks = joy -> axes[v_max_];
   structparam.servo1 = int(joy->axes[servo_x_]);
   structparam.servo2 = int(joy->axes[servo_y_]);
   structparam.enable_drive = joy->buttons[driver_alive_];
   structparam.servo_default = uint8_t(joy->buttons[servo_default_]);
   structparam.led = uint8_t(joy->buttons[led_]);
   structparam.buzzer = uint8_t(joy->buttons[buzzer_]);
   learning_joy::Ramka ramka;

   float predkL=0, predkP=0;
   uint16_t predkLtemp = 0, predkPtemp = 0;
   uint8_t kierL = 0, kierP=0;
   //na podstawie danych z joysticka wyliczamy predkosci  kół i kierunek ich obrotu
   //tylko i wyłącznie gdy guzik jest wciśnięty
   if (structparam.enable_drive == 1)
   {
     if(structparam.x_lin >= -PROG && structparam.x_lin <= PROG && structparam.z_ang >= -PROG && structparam.z_ang <= PROG)
     {
       kierL=0;
       kierP=0;
       predkL=0;
       predkP=0;
     }
     if(structparam.x_lin < -PROG && structparam.z_ang > -PROG && structparam.z_ang < PROG)
     {
       kierL=1;
       kierP=2;
       predkL=(-structparam.x_lin)*VMAX;
       predkP=(-structparam.x_lin)*VMAX;
     }
     if(structparam.x_lin > PROG && structparam.z_ang > -PROG && structparam.z_ang < PROG)
     {
       kierL=2;
       kierP=1;
       predkL=(structparam.x_lin)*VMAX;
       predkP=(structparam.x_lin)*VMAX;
     }
     if(structparam.x_lin <  -PROG && structparam.z_ang > PROG )
     {
       kierL=1;
       kierP=1;
       predkL=VMAX*structparam.z_ang;
       predkP=VMAX*structparam.z_ang - VMAX*structparam.z_ang*(-structparam.x_lin);
     }
     if(structparam.x_lin > PROG && structparam.z_ang > PROG )
     {
       kierL=1;
       kierP=1;
       predkL=VMAX*structparam.z_ang - VMAX*structparam.z_ang*(structparam.x_lin);
       predkP=VMAX*structparam.z_ang;
     }
     if(structparam.x_lin < -PROG && structparam.z_ang < -PROG )
     {
       kierL=2;
       kierP=2;
       predkL=VMAX*(-structparam.z_ang);
       predkP=VMAX*(-structparam.z_ang) - VMAX*(-structparam.z_ang)*(-structparam.x_lin);
     }
     if(structparam.x_lin > PROG && structparam.z_ang < -PROG )
     {
       kierL=2;
       kierP=2;
       predkL=VMAX*(-structparam.z_ang) - VMAX*(-structparam.z_ang)*(structparam.x_lin);
       predkP=VMAX*(-structparam.z_ang);
     }
     if(structparam.x_lin > -PROG && structparam.x_lin < PROG && structparam.z_ang > PROG)
     {
       kierL=1;
       kierP=1;
       predkL=VMAX*(structparam.z_ang);
       predkP=VMAX*(structparam.z_ang);
     }
     if(structparam.x_lin > -PROG && structparam.x_lin < PROG && structparam.z_ang < -PROG)
     {
       kierL=2;
       kierP=2;
       predkL=VMAX*(-structparam.z_ang);
       predkP=VMAX*(-structparam.z_ang);
     }

   }
   else {
     kierL=0;
     kierP=0;
     predkL=0;
     predkP=0;
   }
   switch (structparam.servo1)
   {
   case 0:
     break;
   case 1:
     if(servo1_temp<180)
     {
       servo1_temp++;
     }
     break;
   case -1:
     if(servo1_temp>0)
     {
       servo1_temp--;
     }
     break;
   default:
       break;
   }

   switch (structparam.servo2)
   {
   case 0:
     break;
   case 1:
     if(servo2_temp<180)
     {
       servo2_temp++;
     }
     break;
   case -1:
     if(servo2_temp>0)
     {
       servo2_temp--;
     }
     break;
   default:
       break;
   }

   //przywracanie wartosci domyslnych na serwa
   if (structparam.servo_default == 1)
   {
     servo1_temp = 90;
     servo2_temp = 90;
   }


   ramka.ramka_byte[0] = 97;
   ramka.ramka_byte[1] = kierL;
   ramka.ramka_byte[2] = kierP;
   predkLtemp = uint16_t (predkL);
   predkPtemp = uint16_t (predkP);

   ramka.ramka_byte[4] = uint8_t(predkLtemp);
   ramka.ramka_byte[3] = uint8_t(predkLtemp>>8);


   ramka.ramka_byte[6] = uint8_t(predkPtemp);
   ramka.ramka_byte[5] = uint8_t(predkPtemp>>8);

   ramka.ramka_byte[7] = servo1_temp;
   ramka.ramka_byte[8] = servo2_temp;
   ramka.ramka_byte[9] = ((structparam.buzzer << 1) | structparam.led);
   ramka_pub_.publish(ramka);



   for (int i=0; i<10 ; i++)
   {
     ROS_INFO("%d",ramka.ramka_byte[i]);
   }


}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "ster");
  Sterowanie ster;


  
  ros::spin();
}
