#include "him_cararm_driver/motion_planning.h"


std::vector<float> interpolation(const std::vector<float> a, const std::vector<float> b, const float t){
	std::vector<float> r(4, 0.0);
	float t_ = 1 - t;
	float Wa, Wb;
	float temp = a[0]*b[0] + a[1]*b[1] + a[2]*b[2] + a[3]*b[3];
	std::vector<float> bb(4, 0.0);
	for(int i=0;i<4;i++){
		bb[i] = b[i];			
	}

	
	if(fabs(1-temp) < 1e-4){
		printf("orientation unchanged\n");
		r[0] = a[0];
		r[1] = a[1];
		r[2] = a[2];
		r[3] = a[3];
		return r;
	}else{
		
		float theta = acos(temp);
		if(theta>=pi/2){
			
			for(int i=0;i<4;i++){
				bb[i] = -b[i];			
			}
			temp = a[0]*bb[0] + a[1]*bb[1] + a[2]*bb[2] + a[3]*bb[3];
			theta = acos(temp);
			printf("> pi/2 theta:%f\n",theta);
		}

		float sn = sin(theta);
		Wa = sin(t_*theta) / sn;
		Wb = sin(t*theta) / sn;
		r[0] = Wa*a[0] + Wb*bb[0];
		r[1] = Wa*a[1] + Wb*bb[1];
		r[2] = Wa*a[2] + Wb*bb[2];
		r[3] = Wa*a[3] + Wb*bb[3];
		return r;
	}
}

void changeEndffectorPoseToLink6Pose(geometry_msgs::PoseArray &waypointPoses){
		for(int i=0;i<waypointPoses.poses.size();i++){
			tf::Quaternion q(waypointPoses.poses[i].orientation.x, waypointPoses.poses[i].orientation.y, waypointPoses.poses[i].orientation.z, waypointPoses.poses[i].orientation.w);	
			tf::Matrix3x3 m(q);
		
			waypointPoses.poses[i].position.x = waypointPoses.poses[i].position.x - END_EFFECTOR_LENGTH * m[0][2];
			waypointPoses.poses[i].position.y = waypointPoses.poses[i].position.y - END_EFFECTOR_LENGTH * m[1][2];
			waypointPoses.poses[i].position.z = waypointPoses.poses[i].position.z - END_EFFECTOR_LENGTH * m[2][2];
		}		
}

void FK_add_end_effector_length(float &x, float &y, float &z, const float ax, const float ay, const float az){
	x = x + END_EFFECTOR_LENGTH * ax;
	y = y + END_EFFECTOR_LENGTH * ay;
	z = z + END_EFFECTOR_LENGTH * az;
}

void poseToMatrix(float matrix[][16], const geometry_msgs::PoseArray &keyPoses){
	for(int i=0;i<keyPoses.poses.size();i++){
		tf::Quaternion q(keyPoses.poses[i].orientation.x, keyPoses.poses[i].orientation.y, keyPoses.poses[i].orientation.z, keyPoses.poses[i].orientation.w);	
		tf::Matrix3x3 m(q);
		matrix[i][0] = m[0][0];
		matrix[i][1] = m[0][1];
		matrix[i][2] = m[0][2];
		matrix[i][3] = keyPoses.poses[i].position.x;
		matrix[i][4] = m[1][0];
		matrix[i][5] = m[1][1];
		matrix[i][6] = m[1][2];
		matrix[i][7] = keyPoses.poses[i].position.y;
		matrix[i][8] = m[2][0];
		matrix[i][9] = m[2][1];
		matrix[i][10] = m[2][2];
		matrix[i][11] = keyPoses.poses[i].position.z;
		matrix[i][12] = 0;
		matrix[i][13] = 0;
		matrix[i][14] = 0;
		matrix[i][15] = 1;
	}		
}

void key_points_with_fixed_num(geometry_msgs::PoseArray waypointPoses, geometry_msgs::PoseArray &keyPoses){
	int waypointNum = waypointPoses.poses.size();
	double dis_x, dis_y, dis_z;
	std::vector<float> a,b,c;
	geometry_msgs::Pose p;		
	for(int j=0;j<waypointNum-1;j++){
		dis_x = (waypointPoses.poses[j+1].position.x - waypointPoses.poses[j].position.x)/SEGMENTS;			
		dis_y = (waypointPoses.poses[j+1].position.y - waypointPoses.poses[j].position.y)/SEGMENTS;
		dis_z = (waypointPoses.poses[j+1].position.z - waypointPoses.poses[j].position.z)/SEGMENTS;
		
		a.push_back(waypointPoses.poses[j].orientation.x);
		a.push_back(waypointPoses.poses[j].orientation.y);
		a.push_back(waypointPoses.poses[j].orientation.z);
		a.push_back(waypointPoses.poses[j].orientation.w);
		b.push_back(waypointPoses.poses[j+1].orientation.x);
		b.push_back(waypointPoses.poses[j+1].orientation.y);
		b.push_back(waypointPoses.poses[j+1].orientation.z);
		b.push_back(waypointPoses.poses[j+1].orientation.w);
		
		for(int k=0;k<SEGMENTS;k++){
			p.position.x = waypointPoses.poses[j].position.x + k * dis_x;
			p.position.y = waypointPoses.poses[j].position.y + k * dis_y;
			p.position.z = waypointPoses.poses[j].position.z + k * dis_z;
			
			
			c = interpolation(a, b, k*1.0/SEGMENTS);
		
			if(k==0){
				printf("j=%d, k=%d, %f %f %f %f\n", j, k, c[0], c[1], c[2], c[3]);		
			}

			p.orientation.x = c[0];
			p.orientation.y = c[1];
			p.orientation.z = c[2];
			p.orientation.w = c[3];
			keyPoses.poses.push_back(p);
		}	
		a.clear();
		b.clear();
		c.clear();
	}
	p = waypointPoses.poses.back();
	keyPoses.poses.push_back(p);
}


void key_points_with_fixed_distance(geometry_msgs::PoseArray waypointPoses, geometry_msgs::PoseArray &keyPoses){
	int waypointNum = waypointPoses.poses.size(), interpolation_num;
	double dis_x, dis_y, dis_z, dis;
	std::vector<float> a,b,c;
	geometry_msgs::Pose p;		
	for(int j=0;j<waypointNum-1;j++){
		dis_x = waypointPoses.poses[j+1].position.x - waypointPoses.poses[j].position.x;			
		dis_y = waypointPoses.poses[j+1].position.y - waypointPoses.poses[j].position.y;
		dis_z = waypointPoses.poses[j+1].position.z - waypointPoses.poses[j].position.z;
		
		dis = sqrt( pow(dis_x,2) + pow(dis_y,2) + pow(dis_z,2) );
		interpolation_num =  dis / INTERPOLATION_DIS;

		a.push_back(waypointPoses.poses[j].orientation.x);
		a.push_back(waypointPoses.poses[j].orientation.y);
		a.push_back(waypointPoses.poses[j].orientation.z);
		a.push_back(waypointPoses.poses[j].orientation.w);
		b.push_back(waypointPoses.poses[j+1].orientation.x);
		b.push_back(waypointPoses.poses[j+1].orientation.y);
		b.push_back(waypointPoses.poses[j+1].orientation.z);
		b.push_back(waypointPoses.poses[j+1].orientation.w);

		for(int i=0;i<=interpolation_num;i++){
			p.position.x = waypointPoses.poses[j].position.x + i*INTERPOLATION_DIS/dis * dis_x;
			p.position.y = waypointPoses.poses[j].position.y + i*INTERPOLATION_DIS/dis * dis_y;
			p.position.z = waypointPoses.poses[j].position.z + i*INTERPOLATION_DIS/dis * dis_z;

			c = interpolation(a, b, i*INTERPOLATION_DIS/dis);
			p.orientation.x = c[0];
			p.orientation.y = c[1];
			p.orientation.z = c[2];
			p.orientation.w = c[3];
			keyPoses.poses.push_back(p);
		}	
		a.clear();
		b.clear();
		c.clear();
	}
	p = waypointPoses.poses.back();
	keyPoses.poses.push_back(p);
}

/*
double distance(double point1[3], double point2[3]){
	double len = 0;
	for (int i=0; i < 3; ++i){
	    len += pow((point1[i] - point2[i]), 2);
	}
	len = sqrt(len);
	return len;
}

double dot_product(double vector1[3], double vector2[3]){
	double product = 0;
	for (int i=0; i < 3;  ++i){
	    product += vector1[i] * vector2[i];
	}
	return product; 
}


double model(double vector[3]){
	double zero[3];
	for (int i = 0; i < 3; i++){
		zero[i] = 0;
	}
	return distance(vector, zero);
}



double sin_half_alpha(double vector1[3], double vector2[3]){
	double cos_alpha = dot_product(vector1, vector2) / (model(vector1) * model(vector2));
	double sin_half_alpha;
	//if cos_alpha==1
	if(fabs(cos_alpha-1)<0.00001){
		sin_half_alpha=0.000001;
	}
	else{
		sin_half_alpha = sqrt((1 - cos_alpha) / 2);
	}
	return sin_half_alpha;
}

void vector(double beg_point[3], double end_point[3], double vector[3]){       
	for (int i = 0; i < 3; ++i){
	    vector[i] = end_point[i] - beg_point[i];
	}
}

void copy(double *vs, double *coped_vs, int points_num){
	for (int i = 0; i < points_num; ++i){
	    coped_vs[i] = vs[i];
	}
}
*/

void cubic(double joint[][7],int flag, int points_num, ros::Publisher &joint_pub, sensor_msgs::JointState &joint_msg){
	double x[points_num],y1[points_num],y2[points_num],y3[points_num],y4[points_num],y5[points_num],y6[points_num],l[points_num];
	double AA[points_num][points_num],L[points_num][points_num],U[points_num][points_num];    
	double d1[points_num],d2[points_num],d3[points_num],d4[points_num],d5[points_num],d6[points_num];    //A*M=d
	double b1[points_num],b2[points_num],b3[points_num],b4[points_num],b5[points_num],b6[points_num];    //b save Intermediate variable，LUM=d，Lb=d，UM=b

	double A1[points_num],B1[points_num],C1[points_num],D1[points_num];
	double A2[points_num],B2[points_num],C2[points_num],D2[points_num];
	double A3[points_num],B3[points_num],C3[points_num],D3[points_num];
	double A4[points_num],B4[points_num],C4[points_num],D4[points_num];
	double A5[points_num],B5[points_num],C5[points_num],D5[points_num];
	double A6[points_num],B6[points_num],C6[points_num],D6[points_num];

	double joint_value[6];

	for (int i = 0; i < points_num; ++i)
	{
		x[i] = joint[i][0];
		y1[i] = joint[i][1];
		y2[i] = joint[i][2];
		y3[i] = joint[i][3];
		y4[i] = joint[i][4];
		y5[i] = joint[i][5];
		y6[i] = joint[i][6];
		                            
	}


	for(int i = 0; i < points_num - 1; ++i)
	{
		l[i] = x[i+1] - x[i];         //define l[i]
	}

	for(int i1 = 0; i1 < points_num; ++i1)
	{
		for(int j1 = 0; j1 < points_num; ++j1)
		{
			AA[i1][j1] = 0;
		}                        //initialising matrix A
	}

	for(int i = 0; i < points_num; ++i)
	{
		d1[i]=0.0;d2[i]=0.0;d3[i]=0.0;d4[i]=0.0;d5[i]=0.0;d6[i]=0.0;
		b1[i]=0.0;b2[i]=0.0;b3[i]=0.0;b4[i]=0.0;b5[i]=0.0;b6[i]=0.0;
	}

	for(int k1 = 0; k1 < points_num; ++k1)
	{
		if(k1 == 0)
		{
	
			AA[k1][k1] = 1.0;
		}
		else if(k1 == points_num - 1)
		{
	
			AA[k1][k1] = 1.0;
		
		}
		else 
		{
			AA[k1][k1-1] =l[k1-1] / 6.0;
			AA[k1][k1+1] = l[k1] / 6.0;
			AA[k1][k1] = (l[k1] + l[k1-1]) / 3.0;                  //matrix A is only relate to x
	
		}
	}

	for(int i1 = 0; i1 < points_num; ++i1)               //initialising  d
	{
		if(i1 == 0)
		{
			d1[0] = 0.0;
			d2[0] = 0.0;
			d3[0] = 0.0;
			d4[0] = 0.0;
			d5[0] = 0.0;
			d6[0] = 0.0;
		
		}
		else if(i1 == points_num - 1)
		{
			d1[0] = 0.0;
			d2[0] = 0.0;
			d3[0] = 0.0;
			d4[0] = 0.0;
			d5[0] = 0.0;
			d6[0] = 0.0;
		}
		else
		{
			d1[i1] = (y1[i1+1]-y1[i1]) / l[i1]-(y1[i1]-y1[i1-1]) / l[i1-1];
			d2[i1] = (y2[i1+1]-y2[i1]) / l[i1]-(y2[i1]-y2[i1-1]) / l[i1-1];
			d3[i1] = (y3[i1+1]-y3[i1]) / l[i1]-(y3[i1]-y3[i1-1]) / l[i1-1];	
			d4[i1] = (y4[i1+1]-y4[i1]) / l[i1]-(y4[i1]-y4[i1-1]) / l[i1-1];	
			d5[i1] = (y5[i1+1]-y5[i1]) / l[i1]-(y5[i1]-y5[i1-1]) / l[i1-1];	
			d6[i1] = (y6[i1+1]-y6[i1]) / l[i1]-(y6[i1]-y6[i1-1]) / l[i1-1];			
		}
	}

	/*
	* solve the matrix M
	*
	*/
	double *M1=(double *)malloc(points_num * points_num * sizeof(double));
	double *M2=(double *)malloc(points_num * points_num * sizeof(double));
	double *M3=(double *)malloc(points_num * points_num * sizeof(double));
	double *M4=(double *)malloc(points_num * points_num * sizeof(double));
	double *M5=(double *)malloc(points_num * points_num * sizeof(double));
	double *M6=(double *)malloc(points_num * points_num * sizeof(double));


	for(int i = 0; i < points_num; ++i)
	{
		for(int j=0; j < points_num; ++j)
		{
	
	    	L[i][j] = 0;
		    U[i][j] = 0;               //  initialising  L U
		}
	}
	for(int i = 0; i < points_num; ++i)
	{
		L[i][i] = 1;
		U[i][i+1] = AA[i][i+1];

     	if(i == 0)
		{
	    	U[0][0] = AA[0][0];
		
		}
	
	   if(i > 0)
		{		
			L[i][i-1] = AA[i][i-1] / U[i-1][i-1];
			U[i][i] = AA[i][i] - L[i][i-1] * U[i-1][i];
		}
	}                                                  // get the L U output
	// solve L*b=d
	b1[0] = d1[0];
	b2[0] = d2[0];
	b3[0] = d3[0];
	b4[0] = d4[0];
	b5[0] = d5[0];
	b6[0] = d6[0];

	for(int j = 1; j < points_num; ++j)
	{
	   b1[j] = d1[j] - L[j][j-1] * b1[j-1];  
	   b2[j] = d2[j] - L[j][j-1] * b2[j-1]; 
	   b3[j] = d3[j] - L[j][j-1] * b3[j-1]; 
	   b4[j] = d4[j] - L[j][j-1] * b4[j-1]; 
	   b5[j] = d5[j] - L[j][j-1] * b5[j-1]; 
	   b6[j] = d6[j] - L[j][j-1] * b6[j-1]; 

	}                                              //b saved middle parament
	
	// solve U*M=b
	M1[points_num-1]=b1[points_num-1]/U[points_num-1][points_num-1];
	M2[points_num-1]=b2[points_num-1]/U[points_num-1][points_num-1];
	M3[points_num-1]=b3[points_num-1]/U[points_num-1][points_num-1];
	M4[points_num-1]=b4[points_num-1]/U[points_num-1][points_num-1];
	M5[points_num-1]=b5[points_num-1]/U[points_num-1][points_num-1];
	M6[points_num-1]=b6[points_num-1]/U[points_num-1][points_num-1];

	for(int k = points_num - 2; k >= 0; k--)
	{
	
		M1[k]=(b1[k]-AA[k][k+1]*M1[k+1])/U[k][k];
		M2[k]=(b2[k]-AA[k][k+1]*M2[k+1])/U[k][k];
		M3[k]=(b3[k]-AA[k][k+1]*M3[k+1])/U[k][k];
		M4[k]=(b4[k]-AA[k][k+1]*M4[k+1])/U[k][k];
		M5[k]=(b5[k]-AA[k][k+1]*M5[k+1])/U[k][k];
		M6[k]=(b6[k]-AA[k][k+1]*M6[k+1])/U[k][k];
	
	}

	for(int i = 0; i < points_num - 1; ++i)
	{
		D1[i]=(M1[i+1]-M1[i])/(6.0*l[i]);
		C1[i]=M1[i]/2;
		B1[i]=(y1[i+1]-y1[i])/l[i]-(l[i]*(M1[i+1]+2*M1[i]))/6.0;
		A1[i]=y1[i];

		D2[i]=(M2[i+1]-M2[i])/(6.0*l[i]);
		C2[i]=M2[i]/2;
		B2[i]=(y2[i+1]-y2[i])/l[i]-(l[i]*(M2[i+1]+2*M2[i]))/6.0;
		A2[i]=y2[i];

		D3[i]=(M3[i+1]-M3[i])/(6.0*l[i]);
		C3[i]=M3[i]/2;
		B3[i]=(y3[i+1]-y3[i])/l[i]-(l[i]*(M3[i+1]+2*M3[i]))/6.0;
		A3[i]=y3[i];

		D4[i]=(M4[i+1]-M4[i])/(6.0*l[i]);
		C4[i]=M4[i]/2;
		B4[i]=(y4[i+1]-y4[i])/l[i]-(l[i]*(M4[i+1]+2*M4[i]))/6.0;
		A4[i]=y4[i];

		D5[i]=(M5[i+1]-M5[i])/(6.0*l[i]);
		C5[i]=M5[i]/2;
		B5[i]=(y5[i+1]-y5[i])/l[i]-(l[i]*(M5[i+1]+2*M5[i]))/6.0;
		A5[i]=y5[i];

		D6[i]=(M6[i+1]-M6[i])/(6.0*l[i]);
		C6[i]=M6[i]/2;
		B6[i]=(y6[i+1]-y6[i])/l[i]-(l[i]*(M6[i+1]+2*M6[i]))/6.0;
		A6[i]=y6[i];
		
	}

	std::ofstream out("/home/bzj/Desktop/cubicspline.txt");
	double jj;
	std::vector<double> jointTheta(6,0);

	for(int i = 0;i < points_num - 1; ++i)
	{
		for(jj=x[i];jj<(x[i+1]-TT/2);jj=jj+TT)

		{
			jointTheta[0] = D1[i]*pow((jj-x[i]),3)+C1[i]*pow((jj-x[i]),2)+B1[i]*(jj-x[i])+A1[i];
			jointTheta[1] = D2[i]*pow((jj-x[i]),3)+C2[i]*pow((jj-x[i]),2)+B2[i]*(jj-x[i])+A2[i];
			jointTheta[2] = D3[i]*pow((jj-x[i]),3)+C3[i]*pow((jj-x[i]),2)+B3[i]*(jj-x[i])+A3[i];
			jointTheta[3] = D4[i]*pow((jj-x[i]),3)+C4[i]*pow((jj-x[i]),2)+B4[i]*(jj-x[i])+A4[i];
			jointTheta[4] = D5[i]*pow((jj-x[i]),3)+C5[i]*pow((jj-x[i]),2)+B5[i]*(jj-x[i])+A5[i];
			jointTheta[5] = D6[i]*pow((jj-x[i]),3)+C6[i]*pow((jj-x[i]),2)+B6[i]*(jj-x[i])+A6[i];
		  	
			for(int k=0;k<6;k++){
				out<<jointTheta[k]<<"    ";
			}
			out<<std::endl;

			joint_msg.header.stamp = ros::Time::now();
		
			joint_msg.position = jointTheta;			
			
			joint_pub.publish(joint_msg);
			usleep(TT*1000000);
			
			
		}	
	}
	//写最后一行
	out<<y1[points_num-1]<<"    "<<y2[points_num-1]<<"    "<<y3[points_num-1]<<"    "<<y4[points_num-1]<<"    "<<y5[points_num-1]<<"    "<<y6[points_num-1]<<std::endl;

	out.close();

	free(M1);free(M2);free(M3);
	free(M4);free(M5);free(M6);

}



