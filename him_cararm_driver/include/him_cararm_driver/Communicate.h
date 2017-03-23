#ifndef COMMUNICATE_H_
#define COMMUNICATE_H_
#include <serial/serial.h>
#include "ros/ros.h"
#include "math.h"
#include <thread>
#include <mutex>
#include <iostream>  
#include "ros/exception.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Twist.h"
#include "common.h"

class Communicate{
	private:
		serial::Serial ser_fpga;
		serial::Serial ser_cmd;	
//		std::string cmd_port_device;
//		std::string fpga_port_device;
		char cmd_port_device[15];
		char fpga_port_device[15];
		int baudrate;
		int timeout;
		bool isExecuteTraj;
		bool isCommunicate;
		bool existData;
		bool enableVs;
		geometry_msgs::Twist vs_info;
		std::mutex mtx_com;	// a lock to protect the integrity of sended data between communicate data and real cmd data.
		std::mutex mtx_q;
		std::mutex mtx_vs_info;
		int jointCycleZeroValue[2];
		int jointAngleZeroValue[AXES_NUM];
		float q[AXES_NUM];	//the realtime current joint value
		std::thread* rt_keep_vs_thread;
 		unsigned char preData[BUF_SIZE];
		int zeroCount;
		int get_ttyUSB_id(const char* physical_id);
	
	public:
		Communicate(int baudrate, int timeout);
		~Communicate();
		int read_from_Ti_serial_port();
		void write_empty_data_to_Ti();
		void keep_communicate();
		template<typename Tp>
		void little_endian_read_4_bytes(unsigned char *data, Tp &tmp);
		template<typename Tp>
		void little_endian_write_4_bytes(unsigned char *data, Tp tmp);
		void send_key_pose_to_Ti(double t_point[], geometry_msgs::PoseArray keyPoses, int IK_choice[]);
		void send_vs_info(const geometry_msgs::Twist &vs);
		void packet_data_with_CRC16(unsigned char data[], int data_len);
		std::vector<double> read_current_joint_value();
		void setIsCommunicate(bool value);
		bool getIsExecuteTraj();
		void setIsExecuteTraj(bool value);
		bool recognize_instruction_code(unsigned char data[]);
		unsigned short int CRC16(const unsigned char * data, unsigned short int dataLength);
		void open_cmd_port();
		void open_fpga_port();
		void get_jacobian_inverse(float J_inverse[AXES_NUM][AXES_NUM]);
		void get_joint_velocities(float J_inverse[AXES_NUM][AXES_NUM], const geometry_msgs::Twist &vs, float v[AXES_NUM]);
		void set_enableVs(bool value);
		void set_q(float current_joint_value[AXES_NUM]);
		void do_visual_servoing();
		void set_joint_speed_zero();
		int emergency_stop(float q[], float vs[]);
		void save_crc_data();
};


#endif
