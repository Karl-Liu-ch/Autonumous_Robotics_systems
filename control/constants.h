
// real robot
#define ED1 0.9919
#define ED2 0.9963
#define WHEEL_SEPARATION 0.2369 /* m */
// smr8
#define BLACK_VALUE(i) (i == 0 ? 71.07 : \
	       i == 1 ? 55.12 : \
	       i == 2 ? 55.31 : \
	       i == 3 ? 59.11 : \
	       i == 4 ? 55.03 : \
	       i == 5 ? 55.16 : \
	       i == 6 ? 55.27 : \
	       i == 7 ? 55.93 : \
	       0 )
#define WHITE_VALUE(i) (i == 0 ? 102.26 : \
	       i == 1 ? 69.05 : \
	       i == 2 ? 71.81 : \
	       i == 3 ? 80.16 : \
	       i == 4 ? 69.59 : \
	       i == 5 ? 69.74 : \
	       i == 6 ? 69.85 : \
	       i == 7 ? 69.98 : \
	       0 )
// smr 12
// #define BLACK_VALUE(i) (i == 0 ? 53 : \
// 	       i == 1 ? 53 : \
// 	       i == 2 ? 56 : \
// 	       i == 3 ? 56 : \
// 	       i == 4 ? 56 : \
// 	       i == 5 ? 56 : \
// 	       i == 6 ? 57 : \
// 	       i == 7 ? 58 : \
// 	       0 )
// #define WHITE_VALUE(i) (i == 0 ? 94 : \
// 	       i == 1 ? 112 : \
// 	       i == 2 ? 117 : \
// 	       i == 3 ? 124 : \
// 	       i == 4 ? 127 : \
// 	       i == 5 ? 124 : \
// 	       i == 6 ? 120 : \
// 	       i == 7 ? 99 : \
// 	       0 )
#define GATE_THRESHOLD 0.6
#define FWD_DIST 0.7
#define ROBOTLENGTH 0.5

// simulation
// #define ED1 1
// #define ED2 1
// #define WHEEL_SEPARATION 0.26 /* m */
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
// #define GATE_THRESHOLD 0.5
// #define FWD_DIST 0.6
// #define ROBOTLENGTH 0.23

#define WHEEL_DIAMETER_L 0.06522 * (1 + (1 - ED1) / 2) * (1 + (1 - ED2) / 2)/* m */
#define WHEEL_DIAMETER_R 0.06522 * (1 - (1 - ED1) / 2) * (1 - (1 - ED2) / 2)
#define DELTA_M_L (M_PI * WHEEL_DIAMETER_L / 2000)
#define DELTA_M_R (M_PI * WHEEL_DIAMETER_R / 2000)
#define DEFAULT_ROBOTPORT 24902
#define EPSILON 0.01
#define K 2.0
#define KP 12.0
#define KI 0.0
#define KD 60.0
#define ACCELERATION 1.0

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