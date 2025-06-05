#define LEFT
//#define RIGHT

#define L_0_VALUE 1.46
#define R_0_VALUE 0.32

#ifdef LEFT

#define ANGLE_SHIFT 0 - L_0_VALUE

#define REVERSE_ANGLES true

#define UP_LIMIT 50
#define DOWN_LIMIT -70

#define UP_SPEED 500
#define DOWN_SPEED -500

#define DRIVE_ANGLE_MSG 0x34A
#define HUB_READY_MSG 0x25A
#define DRIVE_READY_MSG 0x35A

#endif

#ifdef RIGHT

#define REVERSE_ANGLES false

#define ANGLE_SHIFT 0 - R_0_VALUE
#define UP_LIMIT 50
#define DOWN_LIMIT -70

#define UP_SPEED 300
#define DOWN_SPEED -300

#define DRIVE_ANGLE_MSG 0x34B
#define HUB_READY_MSG 0x25B
#define DRIVE_READY_MSG 0x35B

#endif