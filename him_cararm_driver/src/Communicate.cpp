#include "him_cararm_driver/Communicate.h"


Communicate::Communicate(int baudrate, int timeout):timeout(timeout),baudrate(baudrate){
	
	const char *fpga_physical_id = "1-3";
	const char *ti_physical_id = "1-4";
	int fpga_ttyUSB_id = get_ttyUSB_id(fpga_physical_id);	
	int ti_ttyUSB_id = get_ttyUSB_id(ti_physical_id);

//	fpga_port_device = (char *)malloc(15); 
//	cmd_port_device =  (char *)malloc(15);	
/*
	if(fpga_ttyUSB_id==0 && ti_ttyUSB_id==1){
		fpga_port_device = "/dev/ttyUSB0";
		cmd_port_device = "/dev/ttyUSB1";
	}
	else
		if(fpga_ttyUSB_id==1 && ti_ttyUSB_id==0){
			fpga_port_device = "/dev/ttyUSB1";
			cmd_port_device = "/dev/ttyUSB0";
		}
		else{
			printf("com init error.\n");
		}
*/

	sprintf(fpga_port_device, "/dev/ttyUSB%d", fpga_ttyUSB_id);
	sprintf(cmd_port_device, "/dev/ttyUSB%d", ti_ttyUSB_id);
	
	printf("%d, %d, %s,%s\n", fpga_ttyUSB_id, ti_ttyUSB_id, fpga_port_device, cmd_port_device);

	existData = false;
	isExecuteTraj = false;
	isCommunicate = true;
	enableVs = false;
	zeroCount = 0;

	jointCycleZeroValue[0] = 1;
	jointCycleZeroValue[1] = 6;

	jointAngleZeroValue[0] = 33045;
	jointAngleZeroValue[1] = 124973;
	jointAngleZeroValue[2] = 78488;
	jointAngleZeroValue[3] = 94708;
	jointAngleZeroValue[4] = 56266;
	jointAngleZeroValue[5] = 19794;
}

Communicate::~Communicate(){
	//free(fpga_port_device);
	//free(cmd_port_device);
}

int Communicate::get_ttyUSB_id(const char* physical_id){
	char cmd[256] = "ls -l /sys/class/tty/ | grep ";
	strcat(cmd, physical_id);
	FILE *fp = popen(cmd, "r");
	char buf[256];
	fgets(buf, sizeof(buf), fp);
	pclose(fp);
	printf("port_id:%s,   %s\n", physical_id, buf);
	
	
	if(strstr(buf, "ttyUSB0")){
		memset(buf, 0, sizeof(buf));
		return 0;
	}
	
	if(strstr(buf, "ttyUSB1")){
		memset(buf, 0, sizeof(buf));
		return 1;
	}

	if(strstr(buf, "ttyUSB2")){
		memset(buf, 0, sizeof(buf));
		return 2;
	}
	
	if(strstr(buf, "ttyUSB3")){
		memset(buf, 0, sizeof(buf));
		return 3;
	}

	else {
		memset(buf, 0, sizeof(buf));	
		return -1;
	}
}

void Communicate::open_cmd_port(){
	serial::Timeout to = serial::Timeout::simpleTimeout(timeout);
	
	ser_cmd.setPort(cmd_port_device);
	ser_cmd.setBaudrate(baudrate);
	ser_cmd.setTimeout(to);
	ser_cmd.open();
	//add for serial port
	if(ser_cmd.isOpen()){
		ROS_WARN("the serial port %s is opened...", cmd_port_device);
		return;		
	}else{
		ROS_ERROR("the serial port %s is not opened...", cmd_port_device);
		return;		
	}
}


void Communicate::open_fpga_port(){
	serial::Timeout to = serial::Timeout::simpleTimeout(timeout);
		
	ser_fpga.setPort(fpga_port_device);
	ser_fpga.setBaudrate(baudrate);
	ser_fpga.setTimeout(to);
	ser_fpga.open();
	//add for serial port
	if(ser_fpga.isOpen()){
		ROS_WARN("the serial port %s is opened...", fpga_port_device);
		return;		
	}else{
		ROS_ERROR("the serial port %s is not opened...", fpga_port_device);
		return;		
	}
}


template<typename Tp>
void Communicate::little_endian_read_4_bytes(unsigned char *data, Tp &tmp){
	tmp = *((Tp *)data);
}

template<typename Tp>
void Communicate::little_endian_write_4_bytes(unsigned char *data, Tp tmp){
	data[0] = *((unsigned char *)(&tmp));
	data[1] = *((unsigned char *)(&tmp)+1);
	data[2] = *((unsigned char *)(&tmp)+2);
	data[3] = *((unsigned char *)(&tmp)+3); 
}

int Communicate::read_from_Ti_serial_port(){	
	unsigned char buf[128];
	int len, status;
	if(ser_cmd.waitReadable()){
		//len = ser_cmd.read(buf, ser_cmd.available());
		len = ser_cmd.read(buf, 4);
		if(len==4){
			little_endian_read_4_bytes(buf, status);			
			printf("client : read %d bytes : %d\n", len, status);	
			return status;		
		}						
		else{
			ROS_WARN("client : received %d bytes", len);
			printf("status: ");
			for(int i=0;i<len/4;i++){
				little_endian_read_4_bytes(buf+4*i, status);
				printf("%d ", status);
			}
			printf("\n");
			return -1;
		}
	}else{
		printf("ser_cmd serial port is not readable\n");
		return -1;
	}	 
}

void Communicate::packet_data_with_CRC16(unsigned char data[], int data_len){
	unsigned short int crc_result = CRC16(data, data_len - 2);
	data[data_len-2] = (unsigned char)crc_result;
	data[data_len-1] = (unsigned char)(crc_result>>8);
}

void Communicate::write_empty_data_to_Ti(){
	unsigned char data[BUF_SIZE] = {0};
	packet_data_with_CRC16(data, BUF_SIZE);

	printf("CRC-16: %x,%x\n",data[BUF_SIZE-2],data[BUF_SIZE-1]);
	
	int len = ser_cmd.write(data, BUF_SIZE);

	memcpy(preData, data, BUF_SIZE);
	printf("client: send %d bytes data\n", len);
}

void Communicate::setIsCommunicate(bool value){
	isCommunicate = value;
}

bool Communicate::getIsExecuteTraj(){
	return isExecuteTraj;
}

void Communicate::setIsExecuteTraj(bool value){
	isExecuteTraj = value;
}


void Communicate::save_crc_data(){
	FILE *fp = fopen("../Desktop/crc_data.txt","a");
	for(int i=0;i<BUF_SIZE;i++){
		fprintf(fp, "%x ", preData[i]);
	}		
	fprintf(fp, "\n------------------------------------\n");			
	fclose(fp);
}

void Communicate::keep_communicate(){
	int status;
	while(isCommunicate){
		mtx_com.lock();
	//	ROS_INFO("keep_communicate: locked mtx_com");
		if(!existData){
			if((status=read_from_Ti_serial_port())!=-1){
				if(status==2){
					isExecuteTraj = false;
				}
				
				if(status==0){
					save_crc_data();
					if(zeroCount<5){
						zeroCount++;			
					}
					else{
						zeroCount = 0;
						mtx_com.unlock();
						continue;
					}
				}

				if(status!=0 && status!=1 && status!=2){
					mtx_com.unlock();
					continue;
				}				
				write_empty_data_to_Ti();
			}else{
				;
			}	
		}
	//	ROS_INFO("keep_communicate: unlocked mtx_com and sleep 1.0 sec");
		mtx_com.unlock();
		usleep(1000000); // 1.0 sec
	}
}

void Communicate::send_key_pose_to_Ti(double t_point[], geometry_msgs::PoseArray keyPoses, int IK_choice[]){
	int count = 0;
	int error_count = 0;
	int status;
	int write_len;
	float tmp;
	existData = true;
	mtx_com.lock();
	ROS_INFO("send key pose: locked mtx_com");
		
	unsigned char data[BUF_SIZE];

	for(int i=0;i<keyPoses.poses.size();i++){
		
		tmp = (float)t_point[i];
		little_endian_write_4_bytes(data+4+count*36+0, tmp);

		tmp = (float)keyPoses.poses[i].position.x;
		little_endian_write_4_bytes(data+4+count*36+4, tmp);

		tmp = (float)keyPoses.poses[i].position.y;
		little_endian_write_4_bytes(data+4+count*36+8, tmp);
	
		tmp = (float)keyPoses.poses[i].position.z;
		little_endian_write_4_bytes(data+4+count*36+12, tmp);

		tmp = (float)keyPoses.poses[i].orientation.x;
	    	little_endian_write_4_bytes(data+4+count*36+16, tmp); 

		tmp = (float)keyPoses.poses[i].orientation.y;
		little_endian_write_4_bytes(data+4+count*36+20, tmp);

		tmp = (float)keyPoses.poses[i].orientation.z;
		little_endian_write_4_bytes(data+4+count*36+24, tmp);

		tmp = (float)keyPoses.poses[i].orientation.w;
		little_endian_write_4_bytes(data+4+count*36+28, tmp);
		
		little_endian_write_4_bytes(data+4+count*36+32, IK_choice[i]);

		count++;

		if(count==POINTS_MAX){
			data[0] = *((unsigned char *)(&count));
			data[1] = *((unsigned char *)(&count)+1);
			data[2] = *((unsigned char *)(&count)+2);
			data[3] = *((unsigned char *)(&count)+3);
			packet_data_with_CRC16(data, BUF_SIZE);
		
		
			while((status=read_from_Ti_serial_port())==0){

				save_crc_data();

				write_len = ser_cmd.write(preData, BUF_SIZE);
				printf("client: received status=%d, send the previous %d bytes\n", status, write_len);	
				error_count++;
			}
			if(status==1){
				//send the next group waypoints
				write_len = ser_cmd.write(data, BUF_SIZE);
				printf("client: send the next %d bytes, including %d poses\n", write_len, count);
				memcpy(preData, data, BUF_SIZE);
				count = 0; //reset the count to zero
			}else{
				ROS_ERROR("status=%d, the serial port is not readable, returned.", status);
				existData = false;
				mtx_com.unlock();
				return;					
			}					
		}
	}	

	little_endian_write_4_bytes(data, count);
	
	// filled the remained bytes with 0
	memset(data+4+count*36, 0, BUF_SIZE-2-(4+count*36));
	
	packet_data_with_CRC16(data, BUF_SIZE);

	while((status=read_from_Ti_serial_port())==0){

		save_crc_data();

		write_len = ser_cmd.write(preData, BUF_SIZE);
		printf("client: received status=%d, send the previous %d bytes\n", status, write_len);	
		error_count++;
	}

	if(status==1){
		//send the last group waypoints
		write_len = ser_cmd.write(data, BUF_SIZE);
		memcpy(preData, data, BUF_SIZE);
		printf("client: send the next %d bytes, including %d poses\n", write_len, count);
		while((status=read_from_Ti_serial_port())==0){
		
			save_crc_data();			

			write_len = ser_cmd.write(preData, BUF_SIZE);
			printf("client: received status=%d, send the previous %d bytes\n", status, write_len);
			error_count++;								
		}
		if(status==1){
			//this block is the normal ends.
			write_empty_data_to_Ti();					
			sleep(1);
			ROS_INFO("send key pose: unlocked mtx_com");
			existData = false;				
			mtx_com.unlock();				
			printf("error_count: %d\n", error_count);
		}else{
			ROS_ERROR("status=%d, the serial port is not readable, returned.", status);
			existData = false;			
			mtx_com.unlock();
			return;	
		}
	}else{
		ROS_ERROR("status=%d, the serial port is not readable, returned.", status);
		existData = false;
		mtx_com.unlock();
		return;					
	}	
}

void Communicate::set_q(float current_joint_value[AXES_NUM]){
	mtx_q.lock();
	for(int i=0;i<AXES_NUM;i++){
		q[i] = current_joint_value[i];
	}
	mtx_q.unlock();
}


void Communicate::get_jacobian_inverse(float J_inverse[AXES_NUM][AXES_NUM]){
	float m = 421.0, n = 380.0;

	mtx_q.lock();
	float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3], q5 = q[4], q6 = q[5];
	mtx_q.unlock();

	float sq2 = sin(q2);
	float cq2 = cos(q2);
	float sq3 = sin(q3);
	float cq3 = cos(q3);
	float sq4 = sin(q4);
	float cq4 = cos(q4);
	float sq5 = sin(q5);
	float cq5 = cos(q5);
	float sq6 = sin(q6);
	float cq6 = cos(q6);

	J_inverse[0][0] = (cq5*sq4)/(n*sin(q2 + q3) + m*sq2);
	J_inverse[0][1] = (cq4*cq6 + sq4*sq5*sq6)/(n*sin(q2 + q3) + m*sq2);
	J_inverse[0][2] = -(cq4*sq6 - cq6*sq4*sq5)/(n*sin(q2 + q3) + m*sq2);
	J_inverse[0][3] = 0;
	J_inverse[0][4] = 0;
	J_inverse[0][5] = 0;
	
	J_inverse[1][0] = -sq5/(m*sq3);
	J_inverse[1][1] = (cq5*sq6)/(m*sq3);
	J_inverse[1][2] = (cq5*cq6)/(m*sq3);
	J_inverse[1][3] = 0;
	J_inverse[1][4] = 0;
	J_inverse[1][5] = 0;

	J_inverse[2][0] = (n*sq5 + m*cq3*sq5 + m*cq4*cq5*sq3)/(m*n*sq3);
	J_inverse[2][1] = -(n*cq5*sq6 + m*cq3*cq5*sq6 + m*cq6*sq3*sq4 - m*cq4*sq3*sq5*sq6)/(m*n*sq3);
	J_inverse[2][2] = -(n*cq5*cq6 + m*cq3*cq5*cq6 - m*sq3*sq4*sq6 - m*cq4*cq6*sq3*sq5)/(m*n*sq3);
	J_inverse[2][3] = 0;
	J_inverse[2][4] = 0;
	J_inverse[2][5] = 0;

	J_inverse[3][0] = -(sq4*(m*cq3*sq2 + n*pow(cq3,2)*sq2 - n*pow(cq5,2)*sq2 + n*cq2*cq3*sq3 - m*cq3*pow(cq5,2)*sq2 + m*cq4*cq5*sq2*sq3*sq5))/(n*cq5*(n*cq2 + m*sq2*sq3 - n*cq2*pow(cq3,2) + n*cq3*sq2*sq3));
	J_inverse[3][1] = (n*cq2*cq6*sq5 + n*cq4*cq5*cq6*sq2 + m*cq6*sq2*sq3*sq5 - n*cq2*pow(cq3,2)*cq6*sq5 + n*cq3*cq6*sq2*sq3*sq5 - m*cq4*sq2*sq3*sq4*sq6 + n*cq5*sq2*sq4*sq5*sq6 - n*pow(cq3,2)*cq4*cq5*cq6*sq2 - m*pow(cq4,2)*cq6*sq2*sq3*sq5 + m*cq3*cq5*sq2*sq4*sq5*sq6 + m*cq4*pow(cq5,2)*sq2*sq3*sq4*sq6 - n*cq2*cq3*cq4*cq5*cq6*sq3)/(n*cq5*(n*cq2 + m*sq2*sq3 - n*cq2*pow(cq3,2) + n*cq3*sq2*sq3));
	J_inverse[3][2] = (n*cq2*pow(cq3,2)*sq5*sq6 - n*cq4*cq5*sq2*sq6 - m*sq2*sq3*sq5*sq6 - n*cq2*sq5*sq6 - m*cq4*cq6*sq2*sq3*sq4 + n*cq5*cq6*sq2*sq4*sq5 - n*cq3*sq2*sq3*sq5*sq6 + n*pow(cq3,2)*cq4*cq5*sq2*sq6 + m*pow(cq4,2)*sq2*sq3*sq5*sq6 + m*cq4*pow(cq5,2)*cq6*sq2*sq3*sq4 + n*cq2*cq3*cq4*cq5*sq3*sq6 + m*cq3*cq5*cq6*sq2*sq4*sq5)/(n*cq5*(n*cq2 + m*sq2*sq3 - n*cq2*pow(cq3,2) + n*cq3*sq2*sq3));
	J_inverse[3][3] = 0;
	J_inverse[3][4] = sq6/cq5;
	J_inverse[3][5] = cq6/cq5;

	J_inverse[4][0] = -(n*cq2*cq5 - n*cq2*pow(cq3,2)*cq5 + m*cq3*cq4*sq2*sq5 + n*cq3*cq5*sq2*sq3 + m*pow(cq4,2)*cq5*sq2*sq3 + n*pow(cq3,2)*cq4*sq2*sq5 + n*cq2*cq3*cq4*sq3*sq5)/(pow(n,2)*cq2 - pow(n,2)*cq2*pow(cq3,2) + m*n*sq2*sq3 + pow(n,2)*cq3*sq2*sq3);
	J_inverse[4][1] = (n*cq2*pow(cq3,2)*sq5*sq6 - n*cq2*sq5*sq6 + m*cq3*cq4*cq5*sq2*sq6 + m*cq4*cq6*sq2*sq3*sq4 - n*cq3*sq2*sq3*sq5*sq6 + n*pow(cq3,2)*cq4*cq5*sq2*sq6 - m*pow(cq4,2)*sq2*sq3*sq5*sq6 + n*cq2*cq3*cq4*cq5*sq3*sq6)/(pow(n,2)*cq2 - pow(n,2)*cq2*pow(cq3,2) + m*n*sq2*sq3 + pow(n,2)*cq3*sq2*sq3);
	J_inverse[4][2] = -(n*cq2*cq6*sq5 - n*cq2*pow(cq3,2)*cq6*sq5 - m*cq3*cq4*cq5*cq6*sq2 + n*cq3*cq6*sq2*sq3*sq5 + m*cq4*sq2*sq3*sq4*sq6 - n*pow(cq3,2)*cq4*cq5*cq6*sq2 + m*pow(cq4,2)*cq6*sq2*sq3*sq5 - n*cq2*cq3*cq4*cq5*cq6*sq3)/(pow(n,2)*cq2 - pow(n,2)*cq2*pow(cq3,2) + m*n*sq2*sq3 + pow(n,2)*cq3*sq2*sq3);
	J_inverse[4][3] = 0;
	J_inverse[4][4] = cq6;
	J_inverse[4][5] = -sq6;

	J_inverse[5][0] = -(sq4*(m*cq3*sq2*sq5 + n*pow(cq3,2)*sq2*sq5 + m*cq4*cq5*sq2*sq3 + n*cq2*cq3*sq3*sq5))/(n*cq5*(n*cq2 + m*sq2*sq3 - n*cq2*pow(cq3,2) + n*cq3*sq2*sq3));
	J_inverse[5][1] = (n*cq2*cq6 + m*cq6*sq2*sq3 - n*cq2*pow(cq3,2)*cq6 + n*cq3*cq6*sq2*sq3 - m*pow(cq4,2)*cq6*sq2*sq3 + m*cq3*cq5*sq2*sq4*sq6 + n*pow(cq3,2)*cq5*sq2*sq4*sq6 - m*cq4*sq2*sq3*sq4*sq5*sq6 + n*cq2*cq3*cq5*sq3*sq4*sq6)/(n*cq5*(n*cq2 + m*sq2*sq3 - n*cq2*pow(cq3,2) + n*cq3*sq2*sq3));
	J_inverse[5][2] = (n*cq2*pow(cq3,2)*sq6 - m*sq2*sq3*sq6 - n*cq2*sq6 - n*cq3*sq2*sq3*sq6 + m*pow(cq4,2)*sq2*sq3*sq6 + m*cq3*cq5*cq6*sq2*sq4 + n*pow(cq3,2)*cq5*cq6*sq2*sq4 - m*cq4*cq6*sq2*sq3*sq4*sq5 + n*cq2*cq3*cq5*cq6*sq3*sq4)/(n*cq5*(n*cq2 + m*sq2*sq3 - n*cq2*pow(cq3,2) + n*cq3*sq2*sq3));
	J_inverse[5][3] = 1;
	J_inverse[5][4] = (sq5*sq6)/cq5;
	J_inverse[5][5] = (cq6*sq5)/cq5;
}


void Communicate::get_joint_velocities(float m[AXES_NUM][AXES_NUM], const geometry_msgs::Twist &vs, float v[AXES_NUM]){

	v[0] = m[0][0] * vs.linear.x + m[0][1] * vs.linear.y + m[0][2] * vs.linear.z + m[0][3] * vs.angular.x + m[0][4] * vs.angular.y + m[0][5] * vs.angular.z;
	v[1] = m[1][0] * vs.linear.x + m[1][1] * vs.linear.y + m[1][2] * vs.linear.z + m[1][3] * vs.angular.x + m[1][4] * vs.angular.y + m[1][5] * vs.angular.z;
	v[2] = m[2][0] * vs.linear.x + m[2][1] * vs.linear.y + m[2][2] * vs.linear.z + m[2][3] * vs.angular.x + m[2][4] * vs.angular.y + m[2][5] * vs.angular.z;
	v[3] = m[3][0] * vs.linear.x + m[3][1] * vs.linear.y + m[3][2] * vs.linear.z + m[3][3] * vs.angular.x + m[3][4] * vs.angular.y + m[3][5] * vs.angular.z;
	v[4] = m[4][0] * vs.linear.x + m[4][1] * vs.linear.y + m[4][2] * vs.linear.z + m[4][3] * vs.angular.x + m[4][4] * vs.angular.y + m[4][5] * vs.angular.z;
	v[5] = m[5][0] * vs.linear.x + m[5][1] * vs.linear.y + m[5][2] * vs.linear.z + m[5][3] * vs.angular.x + m[5][4] * vs.angular.y + m[5][5] * vs.angular.z;
}


int Communicate::emergency_stop(float q[], float vs[]){	
	printf("current joint value: %f %f %f %f %f %f\n", q[0], q[1], q[2], q[3], q[4], q[5]);

	if(fabs(q[0])>3.04&&(q[0]*vs[0]>0))
		return 1;

	if(fabs(q[1])>2.00&&(q[1]*vs[1]>0))
		return 2;
	
	if(fabs(q[2])>2.00&&(q[2]*vs[2]>0))
		return 3;
	
	if(fabs(q[3])>3.04&&(q[3]*vs[3]>0))
		return 4;

	if(fabs(q[4])>0.519&&(q[4]*vs[4]>0))
		return 5;

	if(fabs(q[5])>0.519&&(q[5]*vs[5]>0))
		return 6;

	return 0;
}


void Communicate::do_visual_servoing(){
	float J_inverse[AXES_NUM][AXES_NUM], joint_velocity[AXES_NUM]; 
	existData = true;
	mtx_com.lock();

	unsigned char data[BUF_SIZE];
	data[0] = 0x01;
	data[1] = 0x00;
	data[2] = 0x01;
	data[3] = 0x00;
	while(enableVs){
		get_jacobian_inverse(J_inverse);
		mtx_vs_info.lock();
		get_joint_velocities(J_inverse, vs_info, joint_velocity);
		mtx_vs_info.unlock();	
		printf("joint_velocity: %f %f %f %f %f %f\n", joint_velocity[0], joint_velocity[1], joint_velocity[2], joint_velocity[3], joint_velocity[4], joint_velocity[5]);	

		float vs[6];
		vs[0] = vs_info.linear.x;
		vs[1] = vs_info.linear.y;
		vs[2] = vs_info.linear.z;
		vs[3] = vs_info.angular.x;
		vs[4] = vs_info.angular.y;
		vs[5] = vs_info.angular.z;

		
		int jointNo = 0;
		mtx_q.lock();
		if(jointNo = emergency_stop(q, joint_velocity)){
			mtx_q.unlock();
			set_joint_speed_zero();
			ROS_WARN("joint %d is close to limit. Emergency Stop!", jointNo);
			continue;
		}
		mtx_q.unlock();

		for(int i=0;i<6;i++){
			little_endian_write_4_bytes(data+4*(i+1), vs[i]);
		}

		
		memset(data+4+24, 0, BUF_SIZE-2-(4+24));
		packet_data_with_CRC16(data, BUF_SIZE);
	
		read_from_Ti_serial_port();
		int write_len = ser_cmd.write(data, BUF_SIZE);
		printf("client: send the vs info: %d bytes\n", write_len);
	}
	set_joint_speed_zero();
	existData = false;
	mtx_com.unlock();
}


void Communicate::send_vs_info(const geometry_msgs::Twist &vs){
	mtx_vs_info.lock();
	vs_info = vs;
	printf("Twist vs: %f %f %f %f %f %f\n", vs.linear.x, vs.linear.y, vs.linear.z, vs.angular.x, vs.angular.y, vs.angular.z);

	mtx_vs_info.unlock();
	if(existData){
		;	
	}else{
		rt_keep_vs_thread = new std::thread(boost::bind(&Communicate::do_visual_servoing, this));
	}	
}


void Communicate::set_joint_speed_zero(){
	unsigned char data[BUF_SIZE];
	data[0] = 0x01;
	data[1] = 0x00;
	data[2] = 0x01;
	data[3] = 0x00;
	
	for(int i=0;i<6;i++){
		little_endian_write_4_bytes(data+4*(i+1), 0.0);
	}
	memset(data+4+24, 0, BUF_SIZE-2-(4+24));
	packet_data_with_CRC16(data, BUF_SIZE);
	
	read_from_Ti_serial_port();
	int write_len = ser_cmd.write(data, BUF_SIZE);
	printf("client: send the vs info: %d bytes, set speed zero.\n", write_len);
}

void Communicate::set_enableVs(bool value){
	enableVs = value;
}

bool Communicate::recognize_instruction_code(unsigned char data[]){
	int len;
	bool isPrepared = false;
	bool preByte = false;
	unsigned char *ch  = (unsigned char *)malloc(sizeof(unsigned char));
	//recognize the first two byte instruction code  0xE8 0x00
	//ROS_INFO("waiting for reading serial port instruction data...");
	if(ser_fpga.waitReadable()){
		while((len=(ser_fpga.read(ch, 1)))==1){
			if(*ch==0xE8||preByte){
				if(*ch==0xE8){
					len = ser_fpga.read(ch, 1);
				}
					
				if(*ch==0x00){
					data[0] = 0xE8;
					data[1] = 0x00;
					isPrepared = true;
					break;						
				}
				else if(*ch==0xE8){
					preByte = true;						
				}else{
					preByte = false;						
				}						
			}	
		}
	}else{
			printf("ser_fpga serial not readable\n");
	}	

	delete ch;
	
	if(isPrepared) {
		return true;
	}
    	else{
		return false;	
	}
}


std::vector<double> Communicate::read_current_joint_value(){
	int jointCycleEncoderReading[2];
	int jointAngleEncoderReading[AXES_NUM];
	float jointAngleReadingCal[AXES_NUM];

    	std::vector<double> jointTheta(6,-9);
	
	unsigned char data[8192];
	int len;
	
	//loop when not recognize the instruction code
	while(!recognize_instruction_code(data));
			
	len = ser_fpga.read(data+2, 54);			
			
	unsigned short int result = CRC16(data, 54);
	//check CRC16 result
	if((data[55] == (unsigned char)(result>>8)) && (data[54] == (unsigned char)result)){					
		little_endian_read_4_bytes(data+2,jointCycleEncoderReading[0]);
		little_endian_read_4_bytes(data+6,jointCycleEncoderReading[1]);
		little_endian_read_4_bytes(data+10,jointAngleEncoderReading[0]);
		little_endian_read_4_bytes(data+14,jointAngleEncoderReading[1]);
		little_endian_read_4_bytes(data+18,jointAngleEncoderReading[2]);
		little_endian_read_4_bytes(data+22,jointAngleEncoderReading[3]);
		little_endian_read_4_bytes(data+26,jointAngleEncoderReading[4]);
		little_endian_read_4_bytes(data+30,jointAngleEncoderReading[5]);
		for(int i = 0; i < AXES_NUM; i++){
			if((jointAngleZeroValue[i]>(ENCODER_FULL_RANGE/2)) &&(jointAngleEncoderReading[i]<jointAngleZeroValue[i]-(ENCODER_FULL_RANGE/2)))
			{
				jointAngleReadingCal[i] = jointAngleEncoderReading[i] + ENCODER_FULL_RANGE - jointAngleZeroValue[i];
			}
			else if((jointAngleZeroValue[i]<(ENCODER_FULL_RANGE/2)) &&(jointAngleEncoderReading[i]>jointAngleZeroValue[i]+(ENCODER_FULL_RANGE/2)))
			{
				jointAngleReadingCal[i] = -(jointAngleZeroValue[i] + ENCODER_FULL_RANGE - jointAngleEncoderReading[i]);
			}
			else
			{
				jointAngleReadingCal[i] = jointAngleEncoderReading[i] - jointAngleZeroValue[i];
			}
		}
		jointTheta[0] = -(jointCycleEncoderReading[0]*ENCODER_FULL_RANGE+jointAngleEncoderReading[0] - jointCycleZeroValue[0]*ENCODER_FULL_RANGE-jointAngleZeroValue[0])*
                        (2*pi/ENCODER_FULL_RANGE/DIAMETER_RATIO_1);
		jointTheta[1] = jointAngleReadingCal[1]*(2*pi/ENCODER_FULL_RANGE);
		jointTheta[2] = jointAngleReadingCal[2]*(2*pi/ENCODER_FULL_RANGE);



		jointTheta[3] = (jointCycleEncoderReading[1]*ENCODER_FULL_RANGE+jointAngleEncoderReading[3] - jointCycleZeroValue[1]*ENCODER_FULL_RANGE-jointAngleZeroValue[3]) *
						(2*pi/ENCODER_FULL_RANGE/DIAMETER_RATIO_4);
		jointTheta[4] = -jointAngleReadingCal[4]*(2*pi/ENCODER_FULL_RANGE);
		jointTheta[5] = -jointAngleReadingCal[5]*(2*pi/ENCODER_FULL_RANGE);

			
		if(len=ser_fpga.read(data, ser_fpga.available())){
		//			printf("\ndiscarding %d bytes\n", len);
		}
		
	}
	return jointTheta;	
}

static unsigned char CRCHighByte[] =
{
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
	0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
	0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
	0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81,
	0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
	0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
	0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
	0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
	0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
	0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
	0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
	0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
	0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
	0x40
};

/* Table of CRC values for low-order byte*/
static unsigned char CRCLowByte[] =
{
	0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4,
	0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
	0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD,
	0x1D, 0x1C, 0xDC, 0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
	0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7,
	0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
	0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE,
	0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
	0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2,
	0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
	0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB,
	0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
	0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0, 0x50, 0x90, 0x91,
	0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
	0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88,
	0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
	0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80,
	0x40
};

unsigned short int Communicate::CRC16(const unsigned char * data, unsigned short int dataLength)
{
	unsigned char CRCHi = 0xFF;                      /* high byte of CRC initialized */
	unsigned char CRCLo = 0xFF;                      /* low byte of CRC initialized  */
	unsigned char index;                             /* index into CRC lookup table  */

    while (dataLength--)                             /* pass through message buffer  */
    {
        index = CRCLo ^ (*(data++));                 /* calculate the CRC            */
        CRCLo = CRCHi ^ CRCHighByte[index];
        CRCHi = CRCLowByte[index];
    }
    return (CRCHi << 8 | CRCLo);
}

