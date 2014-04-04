// CONFIG
static const s32 table_width = 3000;
static const s32 table_height = 2000;
static const s32 robot_width = 430;
static const s32 robot_height = 250;

static const s32 center_y_offset = 50;

static const s32 buffet_height = 300;


// RED
static const s32 RED_A_INIT = 0;
static const s32 RED_X_INIT = -(table_width - robot_height) / 2;
static const s32 RED_Y_INIT = (table_height - robot_height) / 2 - buffet_height + center_y_offset;
static const s32 RED_ANGLE_CALIB = 90;

// YELLOW
static const s32 YELLOW_A_INIT = 180;
static const s32 YELLOW_X_INIT = (table_width - robot_height) / 2;
static const s32 YELLOW_Y_INIT = (table_height - robot_height) / 2 - buffet_height + center_y_offset;
static const s32 YELLOW_ANGLE_CALIB = 90;

// GENERAL

static s32 A_INIT = 180;
static s32 X_INIT = (table_width - robot_height) / 2;
static s32 Y_INIT = (table_height - robot_height) / 2 - buffet_height;
static s32 ANGLE_CALIB = -90;

// RED POINTS
static const Vect<2, s32> BEGIN_POINT(-1100, 450);
static const Vect<2, s32> FIRST_FIRE_POINT(-1100, 0);
static const Vect<2, s32> SECOND_FIRE_POINT(-600, 550);

static const Vect<2, s32> FRONT_FRESQ_POINT(0, 550);
static const Vect<2, s32> FRESQ_POINT(0, 2000);
