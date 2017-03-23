#include "him_cararm_driver/fk_ik.h"

float K_matrix[12];

uint8_t choice_flag[8];

float AXIS_MAX[6] = { 180.0, 120.0, 120.0, 180.0, 35.0, 35.0};
	
float AXIS_MIN[6] = { -180.0, -120.0, -120.0, -180, -35.0, -35.0};
        
float rod_length[3] = { 225.0, 421.0, 380.0};

int IK_count = 0;

int POINTS_NUM = 0;

float cal_axis_1(float * matrix,uint8_t condition){
		float temp,temp_q1;
		float px,py;
		  px = *(matrix+3); py = *(matrix+7);
		if((fabsf(py)<=Epsilon)&&(fabsf(px)>=Epsilon))
					  {
						temp=px/Tolerance;
					  }
				else if((fabsf(py)<=Epsilon)&&(fabsf(px)<=Epsilon))
					  {
						temp=0;
					  }
				else
					  {
						temp=px/py;
					  }

		 temp_q1=-atanf(temp);

		if(condition==NORMAL)//axis_1 (-pi/2, pi/2)
					  {
						  temp_q1=temp_q1;
					  }
				else if(condition==ABNORMAL)//axis_1 not (-pi/2, pi/2)
					  {
							  if(temp_q1>0)
								  {
									temp_q1=temp_q1-pi;
								  }
							  else if(temp_q1<0)
								  {
									temp_q1=temp_q1+pi;
								  }
					  }
	     return temp_q1;

	   }

float cal_axis_2(float q1,float q3,float * rod_length,float * matrix)
   {
	float temp_cosf_0,temp_cosf_1,temp_sinf_0,temp_sinf_1,tempcosf,tempsinf,temptanf,temp_q2;
	float px,py,pz;
	float l,m,n;
	float C1,S3,C3;
	l=*rod_length;
	m=*(rod_length+1);
	n=*(rod_length+2);
	px= *(matrix+3);py= *(matrix+7);pz=*(matrix+11);
	C1=cosf(q1);S3=sinf(q3);C3=cosf(q3);
//printf("cosf(q1)=%.10f,sinf(q3)=%.10f,C3+m/n=%.10f",cosf(q1),sinf(q3),C3+m/n);
			if((fabsf(C1)<=Epsilon)&&(fabsf(S3)>=Epsilon)&&(fabsf(C3+m/n)>=Epsilon))
						{
	                    temp_cosf_0=(pz-l)*(n*C3+m)+n*S3*px/sinf(q1);
						temp_cosf_1=n*S3*n*S3+(n*C3+m)*(n*C3+m);
	     	    		temp_sinf_0=(pz-l)*n*S3-(n*C3+m)*px/sinf(q1);
	        	    	temp_sinf_1=temp_cosf_1;
						}
			else if((fabsf(C1)>=Epsilon)&&(fabsf(S3)<=Epsilon)&&(fabsf(C3+m/n)>=Epsilon))
						{
						temp_sinf_0=-py/C1;
						temp_sinf_1=n*C3+m;
						temp_cosf_0=pz-l;
					    temp_cosf_1=temp_sinf_1;
						}
			else if((fabsf(C1)>=Epsilon)&&(fabsf(S3)>=Epsilon)&&(fabsf(C3+m/n)<=Epsilon))
						{
						 temp_sinf_0=pz-l;
						 temp_sinf_1=n*S3;
						 temp_cosf_0=-py/C1;
		    		     temp_cosf_1=temp_sinf_1;
						}
			else if((fabsf(C1)<=Epsilon)&&(fabsf(S3)<=Epsilon)&&(fabsf(C3+m/n)>=Epsilon))
						{
			     	 	temp_sinf_0=px/sinf(q1);
						temp_sinf_1=n*C3+m;
						temp_cosf_0=pz-l;
						temp_cosf_1=temp_sinf_1;
						}
			else if((fabsf(C1)<=Epsilon)&&(fabsf(S3)>=Epsilon)&&(fabsf(C3+m/n)<=Epsilon))
						{
						temp_sinf_0=-(pz-l);
						temp_sinf_1=n*S3;
						temp_cosf_0=px/sinf(q1);
						temp_cosf_1=temp_sinf_1;
						}
			else
						{
//						temp_cosf_0=(pz-l)*n*S3*C1+py*(n*C3+m);
//						temp_cosf_1=n*n*S3*S3*C1+(n*C3+m)*(n*C3+m)*C1;
//						temp_sinf_0=(pz-l)*C1*(n*C3+m)-py*n*S3;
//					    temp_sinf_1=n*n*S3*S3*C1+(n*C3+m)*(n*C3+m)*C1;
						temp_cosf_0=(pz-l)*(n*C3+m)-py*n*S3/C1;
						temp_cosf_1=n*n*S3*S3+(n*C3+m)*(n*C3+m);
						temp_sinf_0=-(pz-l)*n*S3-py*(n*C3+m)/C1;
						temp_sinf_1=n*n*S3*S3+(n*C3+m)*(n*C3+m);

						}
	    tempcosf=temp_cosf_0/temp_cosf_1;
        tempsinf=temp_sinf_0/temp_sinf_1;
        temptanf=temp_sinf_0/temp_cosf_0;

		if((tempcosf>=0)&&(tempsinf>=0))
						{
						 temp_q2=atanf(temptanf);
						 }
				else if ((tempcosf>=0)&&(tempsinf<=0))
						 {
						  temp_q2=atanf(temptanf);
						 }
			   else if ((tempcosf<=0)&&(tempsinf<=0))
						 {
						  temp_q2=atanf(temptanf)-pi;
						 }
			   else if ((tempcosf<=0)&&(tempsinf>=0))
						 {
						   temp_q2=atanf(temptanf)+pi;
						 }
    return temp_q2;

   }


float cal_denominator_0(float numerator,float denominator)
       {

       float cal_result;
        if(denominator==0&&numerator>0)
        {
    	  	  	 cal_result=numerator/Tolerance;

        }
        else if(denominator==0&&numerator<0)
        	    cal_result=-numerator/Tolerance;
        else if((fabsf(denominator)<fabsf(numerator))&&(fabsf(denominator)-fabsf(numerator)<=0.0001))
              {
        	   if(numerator*denominator>0)
        		   cal_result=1;
        	   else if(numerator*denominator<0)
        		   cal_result=-1;
              }
        else
    	  	  cal_result=numerator/denominator;

         return cal_result;
       }

float cal_axis_3(float q1,float * rod_length,float * matrix,uint8_t condition)
   {
	float temp0,temp1,tempcosf,temp_q3;
	float l,m,n;
	float px,py,pz;
	float C1;
	l=*rod_length;
	m=*(rod_length+1);
	n=*(rod_length+2);
	px= *(matrix+3);py= *(matrix+7);pz=*(matrix+11);
	C1=cosf(q1);
	//printf("m=%f,n=%f,C1=%f\n",m,n,C1);
	if(fabsf(C1)<=Epsilon)
					{
					temp0=(pz-l)*(pz-l)+px*px-m*m-n*n;
					temp1=2*m*n;
					}
				else if((fabsf(py)<=Epsilon)&&(fabsf(px)<=Epsilon))
					{
					temp0=(pz-l)*(pz-l)+px*px-m*m-n*n;
					temp1=2*m*n;
					}
				else
					{
					temp0=(pz-l)*(pz-l)*C1*C1+py*py-m*m*C1*C1-n*n*C1*C1;
					temp1=2*m*n*C1*C1;
					//printf("temp0=%f,temp1=%f\n", temp0, temp1);
					}

	tempcosf=cal_denominator_0(temp0,temp1);


	if(condition==NORMAL)////axis_3 (0,pi)
				  {
	              	temp_q3=acosf(tempcosf);
				  }
			else if(condition==ABNORMAL)//axis_3 (-pi,0)
			      {
				    temp_q3=-acosf(tempcosf);
			      }
	return temp_q3;

   }

float cal_axis_4(float q1,float q2,float q3,float q5,float * matrix)
   {
	float tempcosf,tempsinf,temptanf,temp_q4;
	float nx,ny,nz;
	float C1,S1,S23,C23,C5;
	nx = *matrix;ny=*(matrix+4);nz=*(matrix+8);
	C1=cosf(q1);S1=sinf(q1);S23=sinf(q2+q3);C23=cosf(q2+q3);C5=cosf(q5);
	if(C5==0)
		{
			temp_q4=0;
		}
	else
		{
		tempsinf=(C1*nx + S1*ny)/C5;
		tempcosf=(S1*C23*nx - C1*C23*ny -S23*nz)/C5;
		temptanf=(C1*nx + S1*ny)/(S1*C23*nx - C1*C23*ny -S23*nz);
		if((tempcosf>=0)&&(tempsinf>=0))
						{
						 temp_q4=atanf(temptanf);
						 }
				else if ((tempcosf>=0)&&(tempsinf<=0))
						 {
						  temp_q4=atanf(temptanf);
						 }
			   else if ((tempcosf<=0)&&(tempsinf<=0))
						 {
						  temp_q4=atanf(temptanf)-pi;
						 }
			   else if ((tempcosf<=0)&&(tempsinf>=0))
						 {
						   temp_q4=atanf(temptanf)+pi;
						 }
		}
               return temp_q4;
   }

float cal_axis_5(float q1,float q2,float q3,float * matrix,uint8_t condition)
   {
	float temp_q5;
	float nx,ny,nz;
	float C1,S1,S23,C23;
	nx = *matrix;ny=*(matrix+4);nz=*(matrix+8);
		C1=cosf(q1);S1=sinf(q1);S23=sinf(q2+q3);C23=cosf(q2+q3);
	temp_q5=-(S23*S1*nx -S23*C1*ny + C23*nz);

    		   temp_q5=asinf(temp_q5);
    		       if(condition==NORMAL)//axis_5 (-pi/2,pi/2)
    		       					  {
    		       						  temp_q5=temp_q5;
    		       					  }
    		       				else if(condition==ABNORMAL)//axis_5 not (-pi/2,pi/2)
    		       					  {
    		       					      if(temp_q5>=0)
    		       					                     {
    		       					                       temp_q5=pi-temp_q5;
    		       					                     }
    		       					            else if (temp_q5<=0)
    		       					                     {
    		       					            	     temp_q5=-pi-temp_q5;
    		       					                     }
    		       					  }

       return temp_q5;
   }

float cal_axis_6(float q1,float q2,float q3,float q4,float q5,float * matrix)
   {
	float tempcosf,tempsinf,temp_tan,temp_q6;
	float ox,oy,oz,ax,ay,az;
	float C1,S1,S23,C23,C5,S5;
	ox=*(matrix+1);ax= *(matrix+2);oy= *(matrix+5);ay= *(matrix+6);oz=*(matrix+9);az=*(matrix+10);
	C1=cosf(q1);S1=sinf(q1);S23=sinf(q2+q3);C23=cosf(q2+q3);C5=cosf(q5);S5=sinf(q5);
	if(C5==0)
	{
	tempsinf=S23*S1*ox - S23*C1*oy +C23*oz;
	tempcosf=S23*S1*ax - S23*C1*ay +C23*az;
	    if(S5==1)
	    {
	     ;
	    }
	    else if(S5==-1)
	    {
	    	tempsinf=-tempsinf;
	    	tempcosf=-tempcosf;
	    }
	    temp_tan= tempsinf/ tempcosf;
	}
	else
	{

	tempsinf=(S23*S1*ox -S23*C1*oy + C23*oz)/C5;
	tempcosf=(S23*S1*ax -S23*C1*ay + C23*az)/C5;
	temp_tan=(S23*S1*ox -S23*C1*oy + C23*oz)/(S23*S1*ax -S23*C1*ay + C23*az);


	   if((tempcosf>=0)&&(tempsinf>=0))
	                    {
	    	             temp_q6=atan(temp_tan);
	                     }
	            else if ((tempcosf>=0)&&(tempsinf<=0))
	                     {
	            	      temp_q6=atan(temp_tan);
	                     }
	     	   else if ((tempcosf<=0)&&(tempsinf<=0))
	     	             {
	     		          temp_q6=atan(temp_tan)-pi;
	     	             }
	     	   else if ((tempcosf<=0)&&(tempsinf>=0))
	     	             {
	     		           temp_q6=atan(temp_tan)+pi;
	     	             }
	}
	               return temp_q6;
   }

void calculate_6_axes(float q[][6],float * matrix,float * rod_length,uint8_t i)
{
	switch(i)
	{
	         case 0://case0 q1=(-pi/2,pi/2),q3>0,q5=(-pi/2,pi/2)

	                //calculate q1;
						q[0][0]=cal_axis_1(matrix,NORMAL);
					//calculate q3;
						q[0][2]=cal_axis_3(q[0][0],rod_length,matrix,NORMAL);
					//calculate q2
						q[0][1]=cal_axis_2(q[0][0],q[0][2],rod_length,matrix);
					//calculate q5
					    q[0][4]=cal_axis_5(q[0][0],q[0][1],q[0][2],matrix,NORMAL);
					//calculate q4
					    q[0][3]=cal_axis_4(q[0][0],q[0][1],q[0][2],q[0][4],matrix);
					//calculate q6
					    q[0][5]=cal_axis_6(q[0][0],q[0][1],q[0][2],q[0][3],q[0][4],matrix);
					    break;
	         case 1://case1 q1=(-pi/2,pi/2),q3<0,q5=(-pi/2,pi/2)

	             	//calculate q1;
						q[1][0]=cal_axis_1(matrix,NORMAL);
					//calculate q3;
						q[1][2]=cal_axis_3(q[1][0],rod_length,matrix,ABNORMAL);
					//calculate q2
						q[1][1]=cal_axis_2(q[1][0],q[1][2],rod_length,matrix);
					//calculate q5
					    q[1][4]=cal_axis_5(q[1][0],q[1][1],q[1][2],matrix,NORMAL);
					//calculate q4
					    q[1][3]=cal_axis_4(q[1][0],q[1][1],q[1][2],q[1][4],matrix);
					//calculate q6
					    q[1][5]=cal_axis_6(q[1][0],q[1][1],q[1][2],q[1][3],q[1][4],matrix);
					    break;
	         case 2://case2 q1!=(-pi/2,pi/2),q3>0,q5=(-pi/2,pi/2)

					//calculate q1;
					    q[2][0]=cal_axis_1(matrix,ABNORMAL);
					//calculate q3;
						q[2][2]=cal_axis_3(q[2][0],rod_length,matrix,NORMAL);
					//calculate q2
						q[2][1]=cal_axis_2(q[2][0],q[2][2],rod_length,matrix);
					//calculate q5
					    q[2][4]=cal_axis_5(q[2][0],q[2][1],q[2][2],matrix,NORMAL);
					//calculate q4
					    q[2][3]=cal_axis_4(q[2][0],q[2][1],q[2][2],q[2][4],matrix);
					//calculate q6
					    q[2][5]=cal_axis_6(q[2][0],q[2][1],q[2][2],q[2][3],q[2][4],matrix);
	                    break;
	         case 3://case3 q1!=(-pi/2,pi/2),q3<0,q5=(-pi/2,pi/2)

	        	    //calculate q1;
	        	        q[3][0]=cal_axis_1(matrix,ABNORMAL);
					//calculate q3;
						q[3][2]=cal_axis_3(q[3][0],rod_length,matrix,ABNORMAL);
					//calculate q2
						q[3][1]=cal_axis_2(q[3][0],q[3][2],rod_length,matrix);
					//calculate q5
					    q[3][4]=cal_axis_5(q[3][0],q[3][1],q[3][2],matrix,NORMAL);
					//calculate q4
					    q[3][3]=cal_axis_4(q[3][0],q[3][1],q[3][2],q[3][4],matrix);
					//calculate q6
					    q[3][5]=cal_axis_6(q[3][0],q[3][1],q[3][2],q[3][3],q[3][4],matrix);
					    break;
	         case 4://case4 q1=(-pi/2,pi/2),q3>0,q5!=(-pi/2,pi/2)

	        	    //calculate q1;
	        	        q[4][0]=cal_axis_1(matrix,NORMAL);
					//calculate q3;
						q[4][2]=cal_axis_3(q[4][0],rod_length,matrix,NORMAL);
					//calculate q2
						q[4][1]=cal_axis_2(q[4][0],q[4][2],rod_length,matrix);
					//calculate q5
			  	        q[4][4]=cal_axis_5(q[4][0],q[4][1],q[4][2],matrix,ABNORMAL);
					//calculate q4
					    q[4][3]=cal_axis_4(q[4][0],q[4][1],q[4][2],q[4][4],matrix);
					//calculate q6
					    q[4][5]=cal_axis_6(q[4][0],q[4][1],q[4][2],q[4][3],q[4][4],matrix);
					    break;
	         case 5://case5 q1=(-pi/2,pi/2),q3<0,q5!=(-pi/2,pi/2)

					//calculate q1;
						q[5][0]=cal_axis_1(matrix,NORMAL);
					//calculate q3;
						q[5][2]=cal_axis_3(q[5][0],rod_length,matrix,ABNORMAL);
					//calculate q2
						q[5][1]=cal_axis_2(q[5][0],q[5][2],rod_length,matrix);
					//calculate q5
						q[5][4]=cal_axis_5(q[5][0],q[5][1],q[5][2],matrix,ABNORMAL);
					//calculate q4
						q[5][3]=cal_axis_4(q[5][0],q[5][1],q[5][2],q[5][4],matrix);
					//calculate q6
						q[5][5]=cal_axis_6(q[5][0],q[5][1],q[5][2],q[5][3],q[5][4],matrix);
						break;
	         case 6://case6 q1!=(-pi/2,pi/2),q3>0,q5!=(-pi/2,pi/2)

					//calculate q1;
						q[6][0]=cal_axis_1(matrix,ABNORMAL);
					//calculate q3;
						q[6][2]=cal_axis_3(q[6][0],rod_length,matrix,NORMAL);
					//calculate q2
						q[6][1]=cal_axis_2(q[6][0],q[6][2],rod_length,matrix);
					//calculate q5
						q[6][4]=cal_axis_5(q[6][0],q[6][1],q[6][2],matrix,ABNORMAL);
					//calculate q4
						q[6][3]=cal_axis_4(q[6][0],q[6][1],q[6][2],q[6][4],matrix);
					//calculate q6
						q[6][5]=cal_axis_6(q[6][0],q[6][1],q[6][2],q[6][3],q[6][4],matrix);
						break;
	         case 7://case7 q1!=(-pi/2,pi/2),q3<0,q5!=(-pi/2,pi/2)

					//calculate q1;
						q[7][0]=cal_axis_1(matrix,ABNORMAL);
					//calculate q3;
						q[7][2]=cal_axis_3(q[7][0],rod_length,matrix,ABNORMAL);
					//calculate q2
						q[7][1]=cal_axis_2(q[7][0],q[7][2],rod_length,matrix);
					//calculate q5
						q[7][4]=cal_axis_5(q[7][0],q[7][1],q[7][2],matrix,ABNORMAL);
					//calculate q4
						q[7][3]=cal_axis_4(q[7][0],q[7][1],q[7][2],q[7][4],matrix);
					//calculate q6
						q[7][5]=cal_axis_6(q[7][0],q[7][1],q[7][2],q[7][3],q[7][4],matrix);
						break;
	}
}

void End_effector_error(float * rod_length, float * matrix,float q[][6], int * choice)
{
	float p[7];
	float temp[12];
	int s,t;
	float l,m,n;
	float nx,ny,nz,ox,oy,oz,ax,ay,az,px,py,pz;
	float S0,C0,S1,C1,S12,C12,S3,C3,S4,C4,S5,C5;
	float r1=0.001,r2=3;
	l=rod_length[0];
	m=rod_length[1];
	n=rod_length[2];

	nx = *matrix; ox=*(matrix+1);ax= *(matrix+2);px= *(matrix+3);
	ny=*(matrix+4);oy= *(matrix+5);ay= *(matrix+6);py= *(matrix+7);
	nz=*(matrix+8);oz=*(matrix+9);az=*(matrix+10);pz=*(matrix+11);

	   for(t=0;t<8;t++)
	      {
		   	if(choice[t]==1)
		   		{
					 for(s=0;s<AXES_NUM;s++)
							{
							 p[s]=q[t][s];
							}
								 S0=sinf(p[0]);
								 C0=cosf(p[0]);
								 S1=sinf(p[1]);
								 C1=cosf(p[1]);
								S12=sinf(p[1] + p[2]);
								C12=cosf(p[1] + p[2]);
								S3=sinf(p[3]);
								C3=cosf(p[3]);
								S4=sinf(p[4]);
								C4=cosf(p[4]);
								S5=sinf(p[5]);
								C5=cosf(p[5]);
								temp[0]= C3*C4*S0*C12 + C0*C4*S3- S0*S12*S4-nx;
								temp[1]=C3*S4*S5*S0*C12+ C0*S4*S3*S5+ C4*S5*S0*S12- S3*C5*S0*C12 + C0*C3*C5-ox;
								temp[2]=C3*S4*C5*S0*C12+C0*S4*S3*C5+C4*C5*S0*S12+S5*S3*S0*C12-C0*C3*S5-ax;
								temp[3]=S0*(S12*n + S1*m)-px;
								temp[4]=-C0*C3*C4*C12+C0*S12*S4+C4*S3*S0-ny;
								temp[5]=-C0*C3*S4*S5*C12-C0*C4*S5*S12+C0*S3*C5*C12+S4*S3*S5*S0+C3*C5*S0-oy;
								temp[6]=-C0*C3*S4*C5*C12-C0*C4*C5*S12-C0*S5*S3*C12+S4*S3*C5*S0-C3*S5*S0-ay;
								temp[7]=-C0*(S12*n + S1*m)-py;
								temp[8]=-S12*C3*C4-C12*S4-nz;
								temp[9]=-C3*S4*S5*S12+C4*S5*C12+S12*S3*C5-oz;
								temp[10]=-C3*S4*C5*S12+C4*C5*C12-S12*S3*S5-az;
								temp[11]=C12*n + C1*m + l-pz;								
							if(
								 fabsf(temp[0])<r1&&fabsf(temp[1])<r1&&fabsf(temp[2])<r1&&fabsf(temp[3])<r2
							   &&fabsf(temp[4])<r1&&fabsf(temp[5])<r1&&fabsf(temp[6])<r1&&fabsf(temp[7])<r2
							   &&fabsf(temp[8])<r1&&fabsf(temp[9])<r1&&fabsf(temp[10])<r1&&fabsf(temp[11])<r2
							   )
								{
								choice[t]=1;
								}
							else
								{
		//						printf("line %d is not accurate\n", t);
		//						printf("%f\n%f\n%f\n%f\n%f\n%f\n%f\n%f\n%f\n%f\n%f\n%f\n", temp[0][1],temp[0][2],temp[0][3],temp[0][4],temp[1][1],temp[1][2],temp[1][3],temp[1][4],temp[2][1],temp[2][2],temp[2][3],temp[2][4]);
								choice[t]=0;
								}
	               }
	       }

}


void Mechanical_limit(float q[][6], int * choice)
{
	uint8_t t,s;
	uint8_t temp_flag;
	//float step_max = 1;
	//step_max=CRU_V*2*pi/60/1000*4;

	for(t=0;t<8;t++)
    {
		temp_flag=0;
		   for(s=0;s<=AXES_NUM-1;s++)
		     {
				 if( 
					 (q[t][s]<=AXIS_MAX[s]*pi/180) 
					 && (q[t][s]>=AXIS_MIN[s]*pi/180) 
					// && (fabsf(q[t][s]-preJointValue[s])<step_max) 
				   )
				 {
					 temp_flag++;
				 }
		     }
		    if(temp_flag==AXES_NUM)
			{
				choice[t]=1;
			}
			else
			{
				choice[t]=0;
			}
     }

}



void fk(float p[6], float temp[12]){
	float l,m,n;
	float nx,ny,nz,ox,oy,oz,ax,ay,az,px,py,pz;
	float S0,C0,S1,C1,S12,C12,S3,C3,S4,C4,S5,C5;
	float r1=0.001,r2=3;
	l=rod_length[0];
	m=rod_length[1];
	n=rod_length[2];

	S0=sinf(p[0]);
	C0=cosf(p[0]);
	S1=sinf(p[1]);
	C1=cosf(p[1]);
	S12=sinf(p[1] + p[2]);
	C12=cosf(p[1] + p[2]);
	S3=sinf(p[3]);
	C3=cosf(p[3]);
	S4=sinf(p[4]);
	C4=cosf(p[4]);
	S5=sinf(p[5]);
	C5=cosf(p[5]);
	temp[0]= C3*C4*S0*C12 + C0*C4*S3- S0*S12*S4;
	temp[1]=C3*S4*S5*S0*C12+ C0*S4*S3*S5+ C4*S5*S0*S12- S3*C5*S0*C12 + C0*C3*C5;
	temp[2]=C3*S4*C5*S0*C12+C0*S4*S3*C5+C4*C5*S0*S12+S5*S3*S0*C12-C0*C3*S5;
	temp[3]=S0*(S12*n + S1*m);
	temp[4]=-C0*C3*C4*C12+C0*S12*S4+C4*S3*S0;
	temp[5]=-C0*C3*S4*S5*C12-C0*C4*S5*S12+C0*S3*C5*C12+S4*S3*S5*S0+C3*C5*S0;
	temp[6]=-C0*C3*S4*C5*C12-C0*C4*C5*S12-C0*S5*S3*C12+S4*S3*C5*S0-C3*S5*S0;
	temp[7]=-C0*(S12*n + S1*m);
	temp[8]=-S12*C3*C4-C12*S4;
	temp[9]=-C3*S4*S5*S12+C4*S5*C12+S12*S3*C5;
	temp[10]=-C3*S4*C5*S12+C4*C5*C12-S12*S3*S5;
	temp[11]=C12*n + C1*m + l;
}




void Motion_Init(float * matrix,  float result[][6], int all_choice[][8])
{
 	int choice[8]={0},t=0;
    	float Command_angle[8][6];

	calculate_6_axes(Command_angle,matrix,rod_length,0);
	calculate_6_axes(Command_angle,matrix,rod_length,1);
	calculate_6_axes(Command_angle,matrix,rod_length,2);
	calculate_6_axes(Command_angle,matrix,rod_length,3);
	calculate_6_axes(Command_angle,matrix,rod_length,4);
	calculate_6_axes(Command_angle,matrix,rod_length,5);
	calculate_6_axes(Command_angle,matrix,rod_length,6);
	calculate_6_axes(Command_angle,matrix,rod_length,7);

	Mechanical_limit(Command_angle, choice);

	std::ofstream fout("../Desktop/ik_results.txt", std::ofstream::app);
	fout<<"---------------------------------------------------------------------"<<std::endl;
	fout<<"IK_count:"<<IK_count<<std::endl;
	for(int j=0;j<8;j++){
		fout<<"choice "<<j<<": "<<choice[j];
		for(int k=0;k<6;k++){
			fout<<" Joint "<<k+1<<": "<<Command_angle[j][k];	
		}
		fout<<std::endl;	
	}
	fout<<"---------------------------------------------------------------------"<<std::endl;
	fout.close();


	t = 0;
	while (choice[t] == 0)
	{
	  t++;
	  if (t>7)
	  {
		  printf("mechanical limit\n");
	  }

	}

	for(int i=0;i<8;i++){

		all_choice[IK_count][i] = choice[i];

		for(int j=0;j<6;j++){
			result[i][j] = Command_angle[i][j];	
		}
	}

}
