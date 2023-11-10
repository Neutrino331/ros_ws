//serial_imu.cpp
#include <ros/ros.h>
#include <serial/serial.h>
#include <iostream>
#include <stdio.h>
#include <sys/select.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <pthread.h>
#include <sensor_msgs/Imu.h>
#include <signal.h>
#include <string>


#include <stdexcept>

#include <stdlib.h>
#include <string.h>
#include <fstream>
#include <sstream> //std::stringstream
#include  <cstdlib>//exit
#define CFG_SRC   "~/start_robot/dev_discriminate.config"
#define DEV_NAME   "usb_02"

#ifdef __cplusplus 
extern "C"{
#endif

#include <stdint.h>
#include <stdbool.h>
#include "packet.h"
#include "imu_data_decode.h"

#define IMU_SERIAL   "/dev/ttyUSB2"
#define BAUD         (115200)
#define GRA_ACC      (9.8)
#define DEG_TO_RAD   (0.01745329)

int imu_data_decode_init(void);
typedef void (*on_data_received_event)(packet_t *ptr);
void packet_decode_init(packet_t *pkt, on_data_received_event rx_handler);
uint32_t packet_decode(uint8_t);

void cfg_usb2tty(const char*,const char*);
std::string exec(const char* cmd);
std::string get_ID_SERIAL(std::string);
std::string find_cfg(const char*,const char*);
const char* get_dev(const char* ,const char*);

#ifdef __cplusplus
}
#endif
void publish_imu_data(receive_imusol_packet_t *data, sensor_msgs::Imu *imu_data);

void dump_data_packet(receive_imusol_packet_t *data);

static int frame_rate;

static uint8_t buf[2048];

void timer(int sig)
{
	if(SIGALRM == sig)
	{
		frame_rate = frame_count;
		frame_count = 0;
		alarm(1);
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "serial_imu");
	ros::NodeHandle n;

	ros::Publisher IMU_pub = n.advertise<sensor_msgs::Imu>("imu_data", 20);

	serial::Serial sp;

	serial::Timeout to = serial::Timeout::simpleTimeout(100);

    char learn_port[40];
    sprintf(learn_port,"/dev/%s",get_dev(CFG_SRC,DEV_NAME));
	sp.setPort(learn_port);

	sp.setBaudrate(BAUD);

	sp.setTimeout(to);
	
	
	imu_data_decode_init();
	signal(SIGALRM,timer);

	try
	{
		sp.open();
	}
	catch(serial::IOException& e)
	{
		ROS_ERROR_STREAM("Unable to open port.");
		return -1;
	}
    
	if(sp.isOpen())
	{
		ROS_INFO_STREAM("/dev/ttyUSB0 is opened.");
	}
	else
	{
		return -1;
	}
	
	alarm(1);
	
	ros::Rate loop_rate(500);
	sensor_msgs::Imu imu_data;

	while(ros::ok())
	{
		size_t num = sp.available();
		if(num!=0)
		{
			uint8_t buffer[1024];

			num = sp.read(buffer, num);
			if(num > 0)
			{
				for(int i = 0; i < num; i++)
					packet_decode(buffer[i]);

				imu_data.header.stamp = ros::Time::now();
				imu_data.header.frame_id = "base_link";

				puts("\033c");
				if(receive_gwsol.tag != KItemGWSOL)
				{
					dump_data_packet(&receive_imusol);
					publish_imu_data(&receive_imusol, &imu_data);
					IMU_pub.publish(imu_data);
					puts("Pleaes enter ctrl + 'c' to quit....");
				}
				else
				{
					printf("       GW ID: %4d\n", receive_gwsol.gw_id);
					for(int i = 0; i < receive_gwsol.n; i++)
					{
						dump_data_packet(&receive_gwsol.receive_imusol[i]);
						publish_imu_data(&receive_gwsol.receive_imusol[i], &imu_data);
						IMU_pub.publish(imu_data);
						puts("");
					}
					puts("Please enter ctrl + 'c' to quit...");
				}
			}
		}
		loop_rate.sleep();
	}
    
	sp.close();
 
	return 0;
}

void dump_data_packet(receive_imusol_packet_t *data)
{
	if(bitmap & BIT_VALID_ID)
		printf("     Devie ID:%6d\n",data->id);

	if(bitmap & BIT_VALID_TIMES)
		printf("    Run times: %d days  %d:%d:%d:%d\n",data->times / 86400000, data->times / 3600000 % 24, data->times / 60000 % 60, data->times / 1000 % 60, data->times % 1000);

	printf("  Frame Rate:  %4dHz\r\n",frame_rate);
	if(bitmap & BIT_VALID_ACC)
		printf("       Acc(G):%8.3f %8.3f %8.3f\r\n", data->acc[0], data->acc[1], data->acc[2]);

	if(bitmap & BIT_VALID_GYR)
		printf("   Gyr(deg/s):%8.2f %8.2f %8.2f\r\n", data->gyr[0], data->gyr[1], data->gyr[2]);

	if(bitmap & BIT_VALID_MAG)
		printf("      Mag(uT):%8.2f %8.2f %8.2f\r\n", data->mag[0], data->mag[1], data->mag[2]);

	if(bitmap & BIT_VALID_EUL)
		printf("   Eul(R P Y):%8.2f %8.2f %8.2f\r\n", data->eul[0], data->eul[1], data->eul[2]);

	if(bitmap & BIT_VALID_QUAT)
		printf("Quat(W X Y Z):%8.3f %8.3f %8.3f %8.3f\r\n", data->quat[0], data->quat[1], data->quat[2], data->quat[3]);
}

void publish_imu_data(receive_imusol_packet_t *data, sensor_msgs::Imu *imu_data)
{	
	imu_data->orientation.x = data->quat[1];
	imu_data->orientation.y = data->quat[2];
	imu_data->orientation.z = data->quat[3];
	imu_data->orientation.w = data->quat[0];
	imu_data->angular_velocity.x = data->gyr[0] * DEG_TO_RAD;
	imu_data->angular_velocity.y = data->gyr[1] * DEG_TO_RAD;
	imu_data->angular_velocity.z = data->gyr[2] * DEG_TO_RAD;
	imu_data->linear_acceleration.x = data->acc[0] * GRA_ACC;
	imu_data->linear_acceleration.y = data->acc[1] * GRA_ACC;
	imu_data->linear_acceleration.z = data->acc[2] * GRA_ACC;
}

void cfg_usb2tty(const char* cgf_file_src,const char* usb_str)
{	
    std::cout << "Hello World!";
    printf("%s",cgf_file_src);
}

std::string exec(const char* cmd) {
    char buffer[128];
    std::string result = "";
    FILE* pipe = popen(cmd, "r");
    if (!pipe) throw std::runtime_error("popen() failed!");
    try {
        while (fgets(buffer, sizeof buffer, pipe) != NULL) {
            result += buffer;
        }
    } catch (...) {
        pclose(pipe);
        throw;
    }
    pclose(pipe);
    return result;
}

const char* get_dev(const char* cgf_file_src,const char* find_dev)
{
    int i;
    std::string find_str=find_cfg(cgf_file_src,find_dev);
    std::string err="not found";
    if (find_str==err)
    {
        std::cout <<"error : not found "<<find_dev<<" in "<<cgf_file_src<<"\n";
        exit(0);
    }
    char c1[20];
    const char* c2;
    for(i=0;i<=10;i++)
    {
        sprintf(c1,"ttyACM%d",i);
        if(find_str==get_ID_SERIAL(c1))
        {   
            c2=c1;
            return c2;
        }
    }
    for(i=0;i<=10;i++)
    {
        sprintf(c1,"ttyUSB%d",i);
        if(find_str==get_ID_SERIAL(c1))
        {   
            c2=c1;
            return c2;
        }
    }
    for(i=0;i<=10;i++)
    {
        sprintf(c1,"video%d",i);
        if(find_str==get_ID_SERIAL(c1))
        {   
            c2=c1;
            return c2;
        }
    }
    std::cout<<find_dev<<" not connected \n";
    exit(0);
    return "not connected";
    
}


//尋找是否學過 "key = "
std::string find_cfg(const char* cgf_file_src,const char* find_str)
{
    std::string key1=find_str;
    std::string key2=" = ";
    std::string err="not found";
    key1=key1+key2;
    std::ifstream inFile;
    inFile.open(cgf_file_src);
    std::stringstream strStream;
    strStream << inFile.rdbuf();
    std::string str = strStream.str();
    int ans=str.find(key1);
    int n2;
    if (ans==-1) return err;
    else
    {
        //取出學裝置序號
        key1.length();
        n2=str.find("\n",ans);
        if (n2==-1)
        {
            std::cout <<"error: not find \\n in end"<<"\n";
            exit(0);
        }
        ans=str.find(key1);
        return str.substr(ans+key1.length(),n2-ans-key1.length());
    }   
    return str;
}

/*
#取得裝置序號ID
#device 裝置名稱
#回傳字串結果*/
std::string get_ID_SERIAL(std::string device)
{
    std::string output;
    std::string cmd="udevadm info --query=all --name=/dev/";
    cmd=cmd+device+" 2>/dev/null";
    char *cstr=&cmd[0];
    output=exec(cstr);
    char *cstr2=&output[0];
    char *loc =strstr(cstr2, "ID_SERIAL=" );
    int n1,n2;
    if(loc == NULL) {
        std::string rt="device node not found";
        return rt;
    }
    else {
        n1=output.find("ID_SERIAL=");
        n2=output.find("\n",n1);
        return output.substr(n1+10,n2-n1-10);
    }
    
}