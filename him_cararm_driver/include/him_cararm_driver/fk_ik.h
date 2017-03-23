#ifndef _FK_IK_H_
#define _FK_IK_H_
#include "common.h"
#include <math.h>
#include <fstream>

float cal_axis_1(float * matrix,uint8_t condition);
float cal_axis_3(float q1,float * rod_length,float * matrix,uint8_t condition);
float cal_axis_2(float q1,float q3,float * rod_length,float * matrix);
float cal_axis_5(float q1,float q2,float q3,float * matrix,uint8_t condition);
float cal_axis_4(float q1,float q2,float q3,float q5,float * matrix);
float cal_axis_6(float q1,float q2,float q3,float q4, float q5,float * matrix);
void End_effector_error(float * rod_length,float * matrix,float q[8][6], int * choice);
void calculate_6_axes(float q[][6],float * matrix,float * rod_length,uint8_t i);

void Mechanical_limit(float q[][6], int * choice);
void Motion_Init(float * matrix,  float result[][6], int all_choice[][8]);
float cal_denominator_0(float numerator,float denominator);
void fk(float p[], float temp[]);
extern float K_matrix[12];

extern uint8_t choice_flag[8];

extern float AXIS_MAX[6];
	
extern float AXIS_MIN[6];
        
extern float rod_length[3];

extern int IK_count;

extern int POINTS_NUM;

#endif 
