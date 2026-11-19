#ifndef COURSE_CONFIG_H
#define COURSE_CONFIG_H
#include "course_types.h"
/* Course-level landmarks. Existing optical classifiers stay in race_app.h.
   Sx includes its END landmark: advance only after the landmark is handled.
   All lengths in metres, angles in radians, times in milliseconds. */
#define COURSE_ENABLE 1
#define COURSE_START_SEGMENT 1 /* 1..10; for isolated tests only, set start before its end landmark */
#define COURSE_TOTAL_MS 480000u
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
#define COURSE_GAP_W_MAX 0.35f
#define COURSE_GAP_REACQUIRE_MS 25u
/* ---- S4 gap: LOST LINE -> steer RIGHT by a fixed angle ------------------
   The real S4 blank is NOT a straight continuation: after the line is lost
   the car must yaw RIGHT to find the line on the far side. The 11-08
   framework only held the entry heading (drove straight) and would sail
   past it. COURSE_GAP_TURN_ENABLE 0 restores that old behaviour for A/B.
   Tune with APP_SCOPE_STATE 5 (CH4 gap, CH6 segment mm, CH7 fault):
     F10 / never re-acquiring  -> increase RAD or W
     re-acquires too early      -> increase COURSE_GAP_MIN_M
     turns the wrong way        -> set DIR -1                              */
#define COURSE_GAP_TURN_ENABLE 1   /* 1 = new right-turn behaviour; 0 = old straight */
#define COURSE_GAP_TURN_RAD 0.42f  /* target yaw offset, rad (~20 deg) */
#define COURSE_GAP_TURN_DIR 1      /* +1 = right, -1 = left */
#define COURSE_GAP_TURN_W 1.20f    /* max |w| while rotating, rad/s */
/* From the completed finish-line crossing to motor cut-off, in metres. */
#define COURSE_PARK_DISTANCE_M 0.20f
#define COURSE_PARK_SPEED_MPS 0.20f
#define COURSE_RACE_LAPS 2u
#define ISLAND_PULSE_TRIGGER_E 0.45f
#define ISLAND_PULSE_RETRIGGER_MS 200u /* minimum start-to-start interval */
#define ISLAND_PULSE_W 4.50f /* rad/s */
#define ISLAND_PULSE_MAX_RAD 1.22173048f /* 70 degrees, signed turn from trigger */
#define ISLAND_PULSE_RETRIGGER_MIN_RAD 0.52359878f /* 30 degrees along last command */
#define COURSE_LOSS_SEARCH_ENABLE 1   /* any lost line outside S4 enters SEARCH */
#define COURSE_LOSS_SEARCH_DIR -1     /* spin side when edge_side is unknown */
#if COURSE_LOSS_SEARCH_ENABLE < 0 || COURSE_LOSS_SEARCH_ENABLE > 1
#error Invalid COURSE_LOSS_SEARCH_ENABLE
#endif
#if COURSE_LOSS_SEARCH_DIR != 1 && COURSE_LOSS_SEARCH_DIR != -1
#error Invalid COURSE_LOSS_SEARCH_DIR
#endif
/* Eight: large ring first, then opposite-direction small ring, then large
   ring's remaining half. Pattern alone cannot distinguish the three joins. */
#define EIGHT_FIRST_MIN_M 0.50f
#define EIGHT_FIRST_MIN_YAW 1.0f
/* 2026-11-18: the first leg has no meaningful length of its own - the entry
   junction is right there - so its gate is switched off. The junction
   pattern alone advances IN -> SMALL.
   The trailing "2" of the same junction still cannot advance twice: after
   the hop the stage becomes SMALL, whose own gate (EIGHT_SMALL_MIN_*) is
   0.65 m / 4.0 rad away.
   1 = restore the old 0.50 m / 1.0 rad requirement, 0 = no first-leg gate. */
#define EIGHT_FIRST_GATE 0
#if EIGHT_FIRST_GATE < 0 || EIGHT_FIRST_GATE > 1
#error Invalid EIGHT_FIRST_GATE
#endif
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
   FINISH_SECTION(speed, bar_count, distance_after_last_bar_m)
   S1..S10 are array labels; COURSE_START_SEGMENT remains human 1..10. */
static const CourseSegment course_map[10]={
    [S1]  = CROSS_SECTION(0.40f, 2),
    [S2]  = ISLAND_SECTION(0.40f, PICK_LEFT, PICK_RIGHT),
    [S3]  = CROSS_SECTION(0.40f, 2), /* octagon: CROSS entry and return */
    [S4]  = GAP_SECTION(0.40f, 2, 0.40f, 2.50f),
    [S5]  = CROSS_SECTION(0.40f, 2),
    [S6]  = CROSS_SECTION(0.40f, 2),
    [S7]  = CROSS_SECTION(0.40f, 2),
    [S8]  = ISLAND_SECTION(0.40f, PICK_RIGHT, PICK_LEFT),
    [S9]  = EIGHT_SECTION(0.40f),
    [S10] = FINISH_SECTION(0.40f, 1, COURSE_PARK_DISTANCE_M)
};
#endif
