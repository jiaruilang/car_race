#ifndef COURSE_CONFIG_H
#define COURSE_CONFIG_H
#include "course_types.h"
/* Course-level landmarks. Existing optical classifiers stay in race_app.h.
   Sx includes its END landmark: advance only after the landmark is handled.
   All lengths in metres, angles in radians, times in milliseconds. */
#define COURSE_START_SEGMENT 1 /* 1..10; for isolated tests only, set start before its end landmark */
#define COURSE_TOTAL_MS 240000u
#define COURSE_SEGMENT_MS 90000u
#define COURSE_REARM_M 0.02f
#define COURSE_REARM_MS 30u
#define COURSE_GAP_SPEED 0.20f
#define COURSE_GAP_MIN_M 0.40f /* reject an early short blank as successful gap traversal */
#define COURSE_GAP_MAX_M 0.85f /* 0.65 m nominal blank + tolerance; NOT measured wheel travel */
#define COURSE_GAP_MAX_MS 6000u
#define COURSE_GAP_APPROACH_MS 100u
#define COURSE_GAP_E_MAX 0.25f
#define COURSE_GAP_GYRO_MAX 0.35f
#define COURSE_GAP_REACQUIRE_MS 25u
/* S4: offset the heading target to the right after losing the line. */
#define COURSE_GAP_TURN_RAD 0.42f  /* target yaw offset, rad (~20 deg) */
#define COURSE_GAP_TURN_DIR 1      /* +1 = right, -1 = left */
#define COURSE_GAP_TURN_W 1.20f    /* max |w| while rotating, rad/s */
/* Start straight on the start bar; stop after the final finish bar. */
#define COURSE_START_HOLD_FRAMES 20u  /* frames x 5 ms = 100 ms */
#define COURSE_FINISH_BAR_SPEED 0.20f /* cap after the first finish cross */
#define COURSE_LOSS_SEARCH_DIR -1     /* spin side when edge_side is unknown */
#if COURSE_LOSS_SEARCH_DIR != 1 && COURSE_LOSS_SEARCH_DIR != -1
#error Invalid COURSE_LOSS_SEARCH_DIR
#endif
#if COURSE_START_HOLD_FRAMES > 400u
#error Invalid COURSE_START_HOLD_FRAMES
#endif
/* Eight: large ring first, then opposite-direction small ring, then large
   ring's remaining half. Pattern alone cannot distinguish the three joins. */
#define EIGHT_FIRST_MIN_M 0.50f
#define EIGHT_FIRST_MIN_YAW 1.0f
#define EIGHT_SMALL_MIN_M 0.65f
#define EIGHT_SMALL_MIN_YAW 4.0f
#define EIGHT_RETURN_MIN_M 0.25f
#define EIGHT_JOIN_CLEAR_MS 30u
#define EIGHT_STAGE_MAX_M 3.0f
#define EIGHT_STAGE_MAX_MS 25000u
#define EIGHT_SMALL_SIDE 1 /* +1 right run; mirrors default large-ring left pick */
#define EIGHT_ENTRY_SIDE PICK_LEFT
#define EIGHT_EXIT_SIDE PICK_RIGHT
/* F9 unexpected/missing landmark; F10 gap window exceeded; F11 eight phase
   uncertain; F12 element controller timed out; F13 invalid configuration. */
/* Only fill arguments relevant to this section type.
   CROSS_SECTION(speed, passes)
   ISLAND_SECTION(speed, entry_side, exit_side)
   GAP_SECTION(speed, cross_passes, earliest_gap_m, latest_gap_m)
   EIGHT_SECTION(speed) -- detailed settings in EIGHT_* above
   FINISH_SECTION(speed, bar_count)
   S1..S10 are array labels; COURSE_START_SEGMENT remains human 1..10. */
static const CourseSegment course_map[10]={
    [S1]  = CROSS_SECTION(0.30f, 2),
    [S2]  = ISLAND_SECTION(0.25f, PICK_LEFT, PICK_RIGHT),
    [S3]  = CROSS_SECTION(0.25f, 2), /* octagon: CROSS entry and return */
    [S4]  = GAP_SECTION(0.25f, 2, 0.40f, 2.50f),
    [S5]  = CROSS_SECTION(0.30f, 2),
    [S6]  = CROSS_SECTION(0.30f, 2),
    [S7]  = CROSS_SECTION(0.30f, 2),
    [S8]  = ISLAND_SECTION(0.25f, PICK_RIGHT, PICK_LEFT),
    [S9]  = EIGHT_SECTION(0.25f),
    [S10] = FINISH_SECTION(0.30f, 2)
};
#endif
