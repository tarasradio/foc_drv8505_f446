#define LEFT
//#define RIGHT

#define L_90_VALUE 0.28
#define R_90_VALUE 1.43

#ifdef LEFT

#define ANGLE_SHIFT PI/2 - L_90_VALUE
#define UP_LIMIT 150
#define DOWN_LIMIT 20

#define UP_SPEED 100
#define DOWN_SPEED -100

#define DRIVE_ANGLE_MSG 0x34A
#define HUB_READY_MSG 0x25A
#define DRIVE_READY_MSG 0x35A

#endif

#ifdef RIGHT

#define ANGLE_SHIFT PI/2 - R_90_VALUE
#define UP_LIMIT 150
#define DOWN_LIMIT 20

#define UP_SPEED -100
#define DOWN_SPEED 100

#define DRIVE_ANGLE_MSG 0x34B
#define HUB_READY_MSG 0x25B
#define DRIVE_READY_MSG 0x35B

#endif