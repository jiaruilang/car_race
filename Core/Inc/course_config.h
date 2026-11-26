#ifndef COURSE_CONFIG_H
#define COURSE_CONFIG_H
#include "course_types.h"
/* Course-level landmarks. Existing optical classifiers stay in race_app.h.
   Sx includes its END landmark: advance only after the landmark is handled.
   All lengths in metres, angles in radians, times in milliseconds. */
#define COURSE_ENABLE 1
#ifndef FORK_TEST_MODE
#define FORK_TEST_MODE 0 /* 1: single island segment with fork telemetry */
#endif
#ifndef FORK_TEST_SEGMENT
#define FORK_TEST_SEGMENT 9 /* 2, 8 or 9; start before the island entrance */
#endif
#if FORK_TEST_MODE && FORK_TEST_SEGMENT != 2 && FORK_TEST_SEGMENT != 8 && FORK_TEST_SEGMENT != 9
#error Invalid FORK_TEST_SEGMENT
#endif
#ifndef RACE_FORK_SINGLE_FRAME
#define RACE_FORK_SINGLE_FRAME 1 /* Race A/B: 0 = original thresholds, 1 = one frame */
#endif
#if RACE_FORK_SINGLE_FRAME != 0 && RACE_FORK_SINGLE_FRAME != 1
#error Invalid RACE_FORK_SINGLE_FRAME
#endif
#define FORK_CONFIRM_COUNT(normal) ((FORK_TEST_MODE || RACE_FORK_SINGLE_FRAME) ? 1u : (normal))
#define COURSE_START_SEGMENT 1 /* 1..10; for isolated tests only, set start before its end landmark */
#ifndef SEARCH_SWEEP_TEST
#define SEARCH_SWEEP_TEST 0
#endif
#define SEARCH_SWEEP_SPEED_MPS 0.0f
#define SEARCH_SWEEP_W_RAD_S 8.5f
#define SEARCH_SWEEP_DIR 1 /* +1 left, -1 right */
#define COURSE_PULSE_TAPER_RAD 1.659f /* reaches the terminal rate at approximately 95 degrees */
#define COURSE_S1_FIRST_BEND_TAPER_RAD 0.78539816f /* 45 degrees */
#define COURSE_PULSE_OPTICAL_GAIN 10.0f /* extra taper near the centre of one visible line */
#define COURSE_PULSE_OPTICAL_FILTER_MS 10.0f /* first-order filter time constant */
#define COURSE_TOTAL_MS 480000u
#define COURSE_SEGMENT_MS 90000u
#define COURSE_REARM_M 0.02f
#define COURSE_REARM_MS 30u
#define S9_FORK_BLOCK_M 0.0f /* metres from S9 entry; 0 disables */
#define COURSE_GAP_SPEED 0.85f
#define COURSE_GAP_MIN_M 0.40f /* reject an early short blank as successful gap traversal */
#define COURSE_GAP_MAX_M 2.00f /* 0.65 m nominal blank + tolerance; NOT measured wheel travel */
#define COURSE_GAP_MAX_MS 6000u
#define COURSE_GAP_APPROACH_MS 100u
#define S4_GAP_AFTER_SECOND_CORNER_M 0.15f /* from the end of the second confirmed bend */
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
#define COURSE_GAP_TURN_RAD 0.55f  /* target yaw offset, rad (~20 deg) */
#define COURSE_GAP_TURN_DIR 1      /* +1 = right, -1 = left */
#define COURSE_GAP_TURN_W 1.20f    /* max |w| while rotating, rad/s */
/* From the completed finish-line crossing to motor cut-off, in metres. */
#define COURSE_PARK_DISTANCE_M 0.40f
#define COURSE_PARK_WAIT_MS 1000u
#define COURSE_PARK_TURN_WHEEL_MPS 0.20f
#define COURSE_PARK_SPEED_MPS 0.20f
#ifndef CORNER_SLOW_MODE
#define CORNER_SLOW_MODE 0 /* 0: A, cruise through turns; 1: B, apply corner speed cap */
#endif
#define CORNER_A_PULSE_W 15.5f  /* rad/s, corner */
#define CORNER_B_PULSE_W 7.5f /* rad/s, corner */
#define SEARCH_PULSE_W 25.5f /* rad/s before the angle taper, race SEARCH */
#define SEARCH_SPEED_RATIO 0.52f
#define CORNER_MAX_MPS 0.2f /* B only: 0 commands a pivot */
#if CORNER_SLOW_MODE == 1
#define CORNER_PULSE_W CORNER_B_PULSE_W
#elif CORNER_SLOW_MODE == 0
#define CORNER_PULSE_W CORNER_A_PULSE_W
#else
#error Invalid CORNER_SLOW_MODE
#endif
#define S10_ZIGZAG_MAX_MPS 0.75f
#define S10_ZIGZAG_CLEAR_MS 300u
#define S10_ZIGZAG_MIN_M 1.40f /* distance from the second confirmed corner */
#define COURSE_RACE_LAPS 2u
#define COURSE_CORNER_ANGLE_EXIT_ENABLE 1 /* 1 enables mapped corner angle exit */
#define COURSE_CORNER_EXIT_LEAD_DEG 50u /* lead for mapped 90-degree corners */
#define COURSE_RIGHT_ANGLE_BRAKE_MS 25u /* hold zero yaw rate after release */
#ifndef PULSE_EXIT_BRAKE_TEST
#define PULSE_EXIT_BRAKE_TEST 0 /* 0: A; 1: B, reverse pulse braking */
#endif
#define PULSE_EXIT_BRAKE_MS 10u
#define COURSE_PULSE_MIN_W 0.5f /* terminal angular rate for tapered pulses, rad/s */
#define PULSE_EXIT_BRAKE_W 1.0f /* rad/s */
#ifndef COURSE_RIGHT_ANGLE_TEST
#define COURSE_RIGHT_ANGLE_TEST 0 /* 0: A, normal drive; 1: B, slow confirmed right angles */
#endif
#define COURSE_RIGHT_ANGLE_MAX_MPS 0.80f
#define COURSE_RIGHT_ANGLE_PULSE_W 17.0f
/* Optional extra speed caps (m/s). Zero uses the current segment speed. */
#define FORK_BRANCH_MAX_MPS 0.0f   /* fork or visible-line island tracking */
#define ISLAND_FORK_MAX_MPS 0.0f   /* recognized island split, m/s; 0 disables */
#define ORDINARY_ENTRY_PULSE_MS 50u /* S2 and S9 large-ring entry steering assist */
#define BRANCH_TURN_V 0.0f         /* visible-line entry/exit pulse */
/* Effective same-direction yaw after handing a corner back to line PD. */
#ifndef CORNER_COAST_EXIT
#define CORNER_COAST_EXIT 1
#endif
#ifndef TURN_ANGLE_METHOD
#define TURN_ANGLE_METHOD 1 /* 1 optical flow; 2 fitted line; 3 fit with flow fallback */
#endif
#if TURN_ANGLE_METHOD < 1 || TURN_ANGLE_METHOD > 3
#error Invalid TURN_ANGLE_METHOD
#endif
#define TURN_SENSOR_PITCH_M 0.00636f /* provisional: measure sensor 1 to 12, divide by 11 */
#define TURN_SENSOR_FORWARD_M 0.125f /* provisional: axle midpoint to optical row */
#define TURN_FIT_SAMPLES 5
#define TURN_FIT_MIN_SPAN_M 0.020f
#define TURN_FIT_MAX_RMS_M 0.003f
#define CORNER_COAST_TIME_MS 10u
#define CORNER_FLOW_MIN_SHIFT 1.0f /* sensor pitches across the sample window */
#ifndef CORNER_PREDICT_EXIT
#define CORNER_PREDICT_EXIT 1 /* 0: original release; 1: also predict centre crossing */
#endif
#ifndef TURN_PREDICT_SAMPLE_FRAMES
#define TURN_PREDICT_SAMPLE_FRAMES 2
#endif
#ifndef TURN_PREDICT_LEAD_FRAMES
#define TURN_PREDICT_LEAD_FRAMES 3
#endif
#if TURN_PREDICT_SAMPLE_FRAMES < 2 || TURN_PREDICT_SAMPLE_FRAMES > 32 || TURN_PREDICT_LEAD_FRAMES < 1 || TURN_PREDICT_LEAD_FRAMES > 32
#error Invalid turn prediction window
#endif
#ifndef CORNER_CONFIRM_FRAMES
#define CORNER_CONFIRM_FRAMES 1 /* 1: single-frame trial; 2: consecutive same-side hits */
#endif
#if CORNER_CONFIRM_FRAMES != 1 && CORNER_CONFIRM_FRAMES != 2
#error Invalid CORNER_CONFIRM_FRAMES
#endif
#define CORNER_CROSS_BLOCK_MS 0u /* Ignore CROSS for this long after each corner trigger; 0 disables. */
#ifndef ISLAND_LOOP_USE_CORNER
#define ISLAND_LOOP_USE_CORNER 1 /* 0: island pulse, 1: standard corner pulse in LOOP */
#endif
#define ISLAND_PULSE_TRIGGER_E 0.24f
#define ISLAND_PULSE_RETRIGGER_MS 100u /* minimum start-to-start interval */
#define ISLAND_PULSE_W 8.12f /* rad/s */
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
#ifndef SPECIAL_ISLAND_SINGLE_FORK
#define SPECIAL_ISLAND_SINGLE_FORK 1 /* A: phased junction; B: one fork per transition */
#endif
#ifndef SPECIAL_ISLAND_HOLD_MS
#define SPECIAL_ISLAND_HOLD_MS 30u /* heading hold and retrigger lockout */
#endif
#define ORDINARY_EXIT_SIDE_DELAY_MS 0u /* from exit fall-back to exit-side selection */
#define ORDINARY_PULSE_DELAY_MS 0u /* from exit fall-back to exit pulse */
#define ORDINARY_ELEMENT_UNLOCK_RAD 0.17266463f /* 50 degrees */
#define ISLAND_EXIT_PULSE_RAD 0.2617994f /* 15 degrees */
#define ISLAND_EXIT_SINGLE_MS 20u /* single-line dwell for S8 and small-ring entry/exit */
#define ISLAND_SPECIAL_ENTRY_FORK_FRAMES 2u /* S8 and figure-eight small-ring entry */
#define S8_EXIT_FORK_FRAMES 2u /* consecutive valid forks to arm the S8 exit */
#define S2_EXIT_MIN_M 0.70f /* S2 exit fork gate, metres from island entry; 0 disables */
#define S8_EXIT_MIN_YAW_RAD 4.188790f /* 240 degrees right from entry; 0 disables */
#if ISLAND_SPECIAL_ENTRY_FORK_FRAMES < 1 || ISLAND_SPECIAL_ENTRY_FORK_FRAMES > 20
#error Invalid ISLAND_SPECIAL_ENTRY_FORK_FRAMES
#endif
#define EIGHT_FIRST_MIN_M 0.50f
#define EIGHT_FIRST_MIN_YAW 1.0f
/* Signed left yaw from the S9 outer entry fork; distance gate is optional. */
#define EIGHT_FIRST_GATE 0 /* 1 also requires EIGHT_FIRST_MIN_M */
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
    [S1]  = CROSS_SECTION(1.0f, 2),
    [S2]  = ISLAND_SECTION(1.0f, PICK_LEFT, PICK_RIGHT),
    [S3]  = CROSS_SECTION(1.0f, 2), /* octagon: CROSS entry and return */
    [S4]  = GAP_SECTION(1.0f, 2, 0.8f, 3.50f),
    [S5]  = CROSS_SECTION(1.0f, 2),
    [S6]  = CROSS_SECTION(1.0f, 2),
    [S7]  = CROSS_SECTION(1.0f, 2),
    [S8]  = ISLAND_SECTION(1.0f, PICK_RIGHT, PICK_LEFT),
    [S9]  = EIGHT_SECTION(1.0f),
    [S10] = FINISH_SECTION(1.0f, 1, COURSE_PARK_DISTANCE_M)
};
#endif
