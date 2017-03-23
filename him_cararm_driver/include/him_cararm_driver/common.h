#ifndef _COMMON_H_
#define _COMMON_H_

#define JOINT_MOVE_STEP_MAX 3.14
#define INTERPOLATION_STEP_MAX 1.57

#define SEGMENTS 10
#define INTERPOLATION_DIS 100
#define T_STEP 0.2

#define END_EFFECTOR_LENGTH    122.5

#define ENCODER_FULL_RANGE     0x20000
#define POINTS_MAX 20
#define BUF_SIZE (4+(POINTS_MAX)*4*9+2)
#define DIAMETER_RATIO_1    1.8
#define DIAMETER_RATIO_4    1.2
#define  pi               3.1415926535897932384626433832795
#define Tolerance         1e-10
#define Epsilon           1e-6
#define NORMAL            0
#define ABNORMAL          1

#define ROD_L               225
#define ROD_M               421
#define ROD_N               380
#define DIAMETER_RATIO_1    1.8

#define DIAMETER_RATIO_2    1.94
#define DIAMETER_RATIO_3    1.74
#define DIAMETER_RATIO_4    1.2
#define DIAMETER_RATIO_5    0.94
#define DIAMETER_RATIO_6    0.94
#define RETARDER_RATIO_32    32
#define RETARDER_RATIO_64    64
#define AXIS_1_MAX          170
#define AXIS_1_MIN         -170
#define AXIS_2_MAX 	    120
#define AXIS_2_MIN 	   -120
#define AXIS_3_MAX	    120
#define AXIS_3_MIN	   -120
#define AXIS_4_MAX	    180
#define AXIS_4_MIN	   -180
#define AXIS_5_MAX	     35
#define AXIS_5_MIN	    -35
#define AXIS_6_MAX	     35
#define AXIS_6_MIN	    -35
#define CRU_V                2
#define AXES_NUM             6
#endif
