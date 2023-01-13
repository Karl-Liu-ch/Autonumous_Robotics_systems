#define ED1 0.9919
#define ED2 0.9963
#define WHEEL_SEPARATION 0.2369 /* m */
// #define ED1 1
// #define ED2 1
// #define WHEEL_SEPARATION 0.26 /* m */
#define WHEEL_DIAMETER_L 0.06522 * (1 + (1 - ED1) / 2) * (1 + (1 - ED2) / 2)/* m */
#define WHEEL_DIAMETER_R 0.06522 * (1 - (1 - ED1) / 2) * (1 - (1 - ED2) / 2)
#define DELTA_M_L (M_PI * WHEEL_DIAMETER_L / 2000)
#define DELTA_M_R (M_PI * WHEEL_DIAMETER_R / 2000)
#define DEFAULT_ROBOTPORT 24902
#define EPSILON 0.01
#define K 2.0
#define KP 5.0
#define KI 0.0
#define KD 0.6
#define ACCELERATION 1.0

// real robot
#define BLACK_VALUE(i) (i == 0 ? 76 : \
	       i == 1 ? 55 : \
	       i == 2 ? 56 : \
	       i == 3 ? 61 : \
	       i == 4 ? 55 : \
	       i == 5 ? 55 : \
	       i == 6 ? 55 : \
	       i == 7 ? 56 : \
	       0 )
#define WHITE_VALUE(i) (i == 0 ? 108 : \
	       i == 1 ? 69 : \
	       i == 2 ? 72 : \
	       i == 3 ? 82 : \
	       i == 4 ? 69 : \
	       i == 5 ? 70 : \
	       i == 6 ? 70 : \
	       i == 7 ? 70 : \
	       0 )
// simulation
// #define BLACK_VALUE(i) (i == 0 ? 0 : \
// 	       i == 1 ? 0 : \
// 	       i == 2 ? 0 : \
// 	       i == 3 ? 0 : \
// 	       i == 4 ? 0 : \
// 	       i == 5 ? 0 : \
// 	       i == 6 ? 0 : \
// 	       i == 7 ? 0 : \
// 	       0 )
// #define WHITE_VALUE(i) (i == 0 ? 255 : \
// 	       i == 1 ? 255 : \
// 	       i == 2 ? 255 : \
// 	       i == 3 ? 255 : \
// 	       i == 4 ? 255 : \
// 	       i == 5 ? 255 : \
// 	       i == 6 ? 255 : \
// 	       i == 7 ? 255 : \
// 	       0 )

#define BLACK_LINE_FOUND_VALUE 0.3 /* If all the line sensors are below this value, then the robot must have crossed a black line */
#define WHITE_LINE_FOUND_VALUE 0.7 /* If all the line sensors are above this value, then the robot must have crossed a white line */
#define kA(i) (i == 0 ? 15.7642 : \
	       i == 1 ? 14.5691 : \
	       i == 2 ? 14.6813 : \
	       i == 3 ? 13.5049 : \
	       i == 4 ? 15.5932 : \
	       0 )

#define kB(i) (i == 0 ? 71.9962 : \
	       i == 1 ? 88.3791 : \
	       i == 2 ? 82.9057 : \
	       i == 3 ? 73.8117 : \
	       i == 4 ? 53.2018 : \
	       0 )
#define IR_MIN_VALUE(i) (i == 0 ? 95 : \
                         i == 1 ? 109 : \
                         i == 2 ? 104 : \
                         i == 3 ? 94 : \
                         i == 4 ? 75 : \
                         0 )