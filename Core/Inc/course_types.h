#ifndef COURSE_TYPES_H
#define COURSE_TYPES_H
/* Internal representation and constructors. Tune course_config.h instead. */
enum { S1, S2, S3, S4, S5, S6, S7, S8, S9, S10 };
enum { PICK_LEFT=-1, PICK_RIGHT=1 };
enum { CP_CROSS=1, CP_ISLAND=2 };
enum { EIGHT_OFF, EIGHT_LARGE_IN, EIGHT_SMALL, EIGHT_LARGE_OUT };
typedef struct {
    unsigned checkpoint, passes;
    float speed;
    int island_side, exit_side;
    unsigned gap, eight;
    float gap_arm_min_m, gap_arm_max_m;
    float finish_after_last_bar_m;
} CourseSegment;
/* Unused fields are filled here, not in the user's route table. */
#define CROSS_SECTION(v,n) \
    {CP_CROSS,(n),(v),PICK_LEFT,PICK_RIGHT,0,0,0,0,0}
#define ISLAND_SECTION(v,in_side,out_side) \
    {CP_ISLAND,1,(v),(in_side),(out_side),0,0,0,0,0}
#define GAP_SECTION(v,n,earliest,latest) \
    {CP_CROSS,(n),(v),PICK_LEFT,PICK_RIGHT,1,0,(earliest),(latest),0}
#define EIGHT_SECTION(v) \
    {CP_ISLAND,1,(v),EIGHT_ENTRY_SIDE,EIGHT_EXIT_SIDE,0,1,0,0,0}
#define FINISH_SECTION(v,n,distance) \
    {CP_CROSS,(n),(v),PICK_LEFT,PICK_RIGHT,0,0,0,0,(distance)}
#endif
