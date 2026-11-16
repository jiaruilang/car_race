#ifndef RACE_APP_H
#define RACE_APP_H
/* Include ONLY ONCE, in main.c USER CODE BEGIN 0, after peripheral handles.
   No other .c file should include this implementation header. */
#include "dodo_BMI270.h"
#include "multiplexer.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "course_config.h"

/* 3: fixed-speed test; 4: one-metre straight; 6: stage 2 race. */
#define APP_MODE 6
#if APP_MODE != 3 && APP_MODE != 4 && APP_MODE != 6
#error Invalid APP_MODE: use 3, 4, or 6
#endif
#define TEST_SPEED_MPS 0.40f
#define STOP_AHEAD_M 0.05f
#define DISTANCE_KP 1.5f
#define HEADING_KP 2.0f
#define APP_MEASURED 1     /* set 1 only after ALL measurements below */
#define LEFT_FORWARD_HIGH 1
#define RIGHT_FORWARD_HIGH 0
#define LEFT_ENCODER_SIGN 1
#define RIGHT_ENCODER_SIGN -1
#define GYRO_SIGN 1
#define METRES_PER_COUNT_L 0.00002200f
#define METRES_PER_COUNT_R 0.00002200f
#define EFFECTIVE_TRACK_M 0.18f
#define SPEED_KP_L 0.32f
#define SPEED_KI_L 0.22f
#define SPEED_KP_R 0.35f
#define SPEED_KI_R 0.2f
#define SPEED_FF_L 0.15f    /* feedforward duty per (m/s) of target; 0=off; L/R independent */
#define SPEED_FF_R 0.231f    /* tune = steady_duty/steady_speed, e.g. 0.18/0.30=0.6 */
#define SPEED_FF_TURN_L 0.28f  /* left wheel feedforward in SEARCH and PIVOT */
#define SPEED_FF_TURN_R 0.47f /* right wheel feedforward in SEARCH and PIVOT */
/* Keep tunable speed and output limits in named macros with their units. */
#define MAX_PWM 0.50f
#define COURSE_SPEED_MAX_MPS 0.40f
#define MIN_SPEED_MPS 0.200f  /* lowest sustainable speed (m/s); 0=off; fill after calibration */
#define MIN_WHEEL_TARGET_FRACTION 0.5f /* IDLE: zero wheel targets below this fraction of MIN_SPEED_MPS */
#define LINE_KP 2.5f       /* initial trial only; error range -1..1 */
#define LINE_KD 0.02f
#define LINE_D_MAX 20.0f  /* clamp on de/dt so a stale error cannot kick the D term */
#define APP_SCOPE 1       /* 0 text diagnostics; 1 Seekfree CSV scope */
#define APP_SCOPE_PERIOD_MS 20u /* 50Hz nominal telemetry; control stays 5ms */
/* State-machine telemetry on the Seekfree scope.
   0 = original 8 channels (CH7 = cal)
   1 = CH7 becomes the state code, other 7 channels untouched
   2 = all 8 channels become island state-machine diagnostics (50 Hz)
   3 = corner / steering diagnostics: v cmd, v measured, w cmd, corner,
       w cap, lit count, [gyro rate], state code, R PWM
   4 = course element recognition observer CSV (course_element_test.h)
   5 = COURSE segment diagnostics: segment, pass, element, gap, eight,
       segment mm, fault, finish (default). */
#define APP_SCOPE_STATE 6 /* 6 = right-angle action trace */
#if APP_SCOPE_STATE < 0 || APP_SCOPE_STATE > 6
#error Invalid APP_SCOPE_STATE
#endif
/* ---- Actual angular rate on the STATE 3 scope CSV ----------------------
   Only APP_SCOPE_STATE==3 is affected; STATE 0/1/2/4/5 and text groups stay
   exactly as they were. 0 = original 8 columns (CH7 = state code);
   1 = insert the measured rate as CH7 and shift state code / R PWM to
   CH8 / CH9 (9 columns); 2 = overwrite CH7 with the rate, 8 columns.
   Value = steer_gyro: the 20 ms low-passed gyro rate in rad/s. x1000. */
#define APP_SCOPE_GYRO_CH7 1
#if APP_SCOPE_GYRO_CH7 < 0 || APP_SCOPE_GYRO_CH7 > 2
#error Invalid APP_SCOPE_GYRO_CH7
#endif

#if APP_SCOPE != 0 && APP_SCOPE != 1
#error Invalid APP_SCOPE
#endif
/* Compile only the measurements needed by the selected output. */
#define SCOPE_WHEEL_TARGETS (APP_SCOPE && APP_SCOPE_STATE <= 1)
#define SCOPE_PATTERN (!APP_SCOPE || APP_SCOPE_STATE == 2 || APP_SCOPE_STATE == 3)
#define SCOPE_STEERING (!APP_SCOPE || APP_SCOPE_STATE == 3 || APP_SCOPE_STATE == 6)
#define SCOPE_OBSERVER (APP_SCOPE && APP_SCOPE_STATE == 4)

#define DT 0.005f
#define PI_F 3.14159265358979f /* TRUE pi: physical dps->rad/s and log conversion ONLY */
#define TRACK_HEADING_KP 2.0f /* CROSS and S4 heading hold */

extern struct bmi2_dev bmi270; /* defined in the official dodo_BMI270.c */
static volatile uint32_t app_ms, app_last_completed;
static volatile unsigned app_running, app_fault;
static uint32_t app_last_sample, app_last_log, app_elapsed;
static uint16_t app_prev_l, app_prev_r, app_mask;
static float app_s, app_yaw, app_il, app_ir, app_vl, app_vr;
static float app_error_prev, app_line_prev, app_cmd_l, app_cmd_r;
#if SCOPE_WHEEL_TARGETS
static float app_target_l, app_target_r;
#endif
#if !APP_SCOPE
static int32_t app_counts_l, app_counts_r;
#endif
static float app_bias, app_bias_sum, app_bias_square, app_gz;
static unsigned app_cal_samples, app_cal_ready;
static unsigned app_lost, app_stall;
static int app_imu_init;
static unsigned app_key_prev=1, app_key_stable=1, app_key_ticks;
static char app_tx[320];
static unsigned app_tx_pos, app_tx_len;
/* fault:1 init;2 acquisition;3 scheduling;4 calibration/missing parameters;
   5 lost line;6 timeout;7 encoder jump;8 stall. Reset to clear fault. */
static float app_clip(float x,float lo,float hi){return x<lo?lo:x>hi?hi:x;}
static void App_Stop(void){
    TIM1->CCR1=0;TIM1->CCR2=0;app_running=0;
    app_il=0;app_ir=0;app_cmd_l=0;app_cmd_r=0;
#if SCOPE_WHEEL_TARGETS
    app_target_l=0;app_target_r=0;
#endif
}
static void app_fail(unsigned code){App_Stop();app_fault=code;}
static int32_t app_delta(uint16_t n,uint16_t p){
    uint32_t d=((uint32_t)n+65536u-p)&65535u;
    return d>=32768u?(int32_t)d-65536:(int32_t)d;
}
/* Software deadline guard. ISR never does SPI, printf, delay or control maths. */
static void App_Timer1ms(void){
    ++app_ms;
    if(app_running && (uint32_t)(app_ms-app_last_completed)>20u){
        TIM1->CCR1=0;TIM1->CCR2=0;app_running=0;app_fault=3;
    }
}
static void app_motor(float left,float right){
    float u[2]={left,right};
    static int direction[2]={1,1};
    uint32_t irq=__get_PRIMASK();
    __disable_irq();
    if(!app_running || app_fault){TIM1->CCR1=0;TIM1->CCR2=0;}
    else for(int i=0;i<2;i++){
        float a=app_clip(u[i],-MAX_PWM,MAX_PWM);
        int sign=a>=0?1:-1;
        /* Blank one full control interval before a direction reversal. */
        if(sign!=direction[i]){a=0;direction[i]=sign;}
        if(i==0){
            TIM1->CCR1=0;
            HAL_GPIO_WritePin(L_DIR_GPIO_Port,L_DIR_Pin,
              ((sign>0)==(LEFT_FORWARD_HIGH!=0))?GPIO_PIN_SET:GPIO_PIN_RESET);
            TIM1->CCR1=(uint32_t)(fabsf(a)*7200.0f);
        }else{
            TIM1->CCR2=0;
            HAL_GPIO_WritePin(R_DIR_GPIO_Port,R_DIR_Pin,
              ((sign>0)==(RIGHT_FORWARD_HIGH!=0))?GPIO_PIN_SET:GPIO_PIN_RESET);
            TIM1->CCR2=(uint32_t)(fabsf(a)*7200.0f);
        }
    }
    __set_PRIMASK(irq);
}
static float app_pi(float target,float actual,float *integral,float kp,float ki,float ff){
    float e=target-actual,next=*integral+ki*e*DT;
    float raw=ff*target+kp*e+next,out=app_clip(raw,-MAX_PWM,MAX_PWM);
    if(raw==out || (raw>out && e<0) || (raw<out && e>0))*integral=next;
    return out;
}
static int app_parameters_ok(void){
    return APP_MEASURED && METRES_PER_COUNT_L>0 && METRES_PER_COUNT_R>0 &&
      EFFECTIVE_TRACK_M>0 && SPEED_KP_L>0 && SPEED_KP_R>0 &&
      SPEED_KI_L>=0 && SPEED_KI_R>=0 && SPEED_FF_L>=0 && SPEED_FF_R>=0 &&
      SPEED_FF_TURN_L>=0 && SPEED_FF_TURN_R>=0 &&
      MAX_PWM>0 && MAX_PWM<=0.85f && COURSE_SPEED_MAX_MPS>0 && COURSE_SPEED_MAX_MPS<=0.5f &&
      MIN_SPEED_MPS>=0 && MIN_SPEED_MPS<=COURSE_SPEED_MAX_MPS &&
      MIN_WHEEL_TARGET_FRACTION>=0 && MIN_WHEEL_TARGET_FRACTION<=1.0f &&
      TRACK_HEADING_KP>0 && LINE_D_MAX>0;
}
/* Track sensing and island tuning. Channels 0..11 run left to right. */
#define TRACK_CRUISE course_cruise()
#define ISLAND_SIDE course_side() /* -1 left, +1 right, 0 stop at candidate for inspection */
#define CENTRAL_TOTAL_MAX 9     /* widest single segment still a normal line (was 6: 7..9 grey band) */
#define CENTRAL_WEIGHT_MAX 2.0f /* |weighted sum| below this => line is centred */
#define CORNER_HALF_HITS 6      /* all 6 sensors on one side lit => corner */
#define CORNER_FRAMES 2         /* opposite-sign frames needed to flip a latched direction */
/* A run can cross the middle of the array; the side is set by its outer edge. */
/* Right-angle A/B: 0 = ordinary line control, 1 = moving arc, 2 = pivot. */
#define CORNER_TURN_MODE 1
#if CORNER_TURN_MODE < 0 || CORNER_TURN_MODE > 2
#error Invalid CORNER_TURN_MODE
#endif
#define CORNER_ARC_RADIUS_M 0.10f
#define CORNER_ARC_SPEED_MPS 0.20f
#define CORNER_EXIT_SPEED_MPS 0.20f
#define CORNER_ARC_LINE_MAX_RAD_S 0.35f /* max optical correction during arc */
#define CORNER_PIVOT_ADVANCE_M 0.03f /* calibrate sensor-to-axle approach on track */
#define CORNER_PIVOT_APPROACH_MPS 0.20f
#define CORNER_PIVOT_APPROACH_MAX_RAD_S 0.60f /* heading correction cap before braking */
#define CORNER_PIVOT_MAX_RAD_S 2.0f
#define CORNER_PIVOT_MIN_RAD_S 1.0f /* breakaway command outside yaw tolerance */
#define CORNER_STOP_WHEEL_MPS 0.04f
#define CORNER_TARGET_RAD (PI_F/2.0f)
#define CORNER_TOL_RAD (7.0f*PI_F/180.0f)
#define CORNER_SLOW_RAD (25.0f*PI_F/180.0f)
#define CORNER_MAX_OVERSHOOT_RAD (25.0f*PI_F/180.0f)
#define CORNER_HEADING_KP 5.0f
#define CORNER_HANDOFF_ANGLE_RAD (85.0f*PI_F/180.0f) /* hand back even without a line */
#define CORNER_HANDOFF_LINE_MIN_RAD (70.0f*PI_F/180.0f) /* ignore incoming-line patterns */
#define CORNER_HANDOFF_LINE_MIN_LIT 3 /* minimum width of the contiguous run */
#define CORNER_HANDOFF_LEFT_FIRST_CH 4 /* left turn: run starts at ch4 */
#define CORNER_HANDOFF_RIGHT_LAST_CH 7 /* right turn: mirror run ends at ch7 */
#define CORNER_BRAKE_STOP_FRAMES 3u /* 15 ms below the wheel-speed threshold before pivot */
#define CORNER_TIMEOUT_MS 4000u
#define CORNER_EXIT_MAX_M 0.12f
#define CORNER_REARM_M 0.02f
#define CORNER_CROSS_VETO_M 0.03f
#define CORNER_INCOMING_MAX_AGE_MS 120u
#define CORNER_INCOMING_MAX_RATE 0.35f

#define RIGHT_ANGLE_MIN_LIT 6
#define RIGHT_ANGLE_MAX_LIT 9
#define RIGHT_ANGLE_SHAPE_FRAMES 2u /* 10 ms at the 5 ms control period */
#define RIGHT_ANGLE_BEND_MIN 0.90f /* |bend| + BEND_RATE_K * outward change */
#define RIGHT_ANGLE_BEND_MIN_OFFSET (2.0f/11.0f) /* one sensor pitch */
/* ---- Fork / island entry ---- */
#define FORK_EDGE_MARGIN 2    /* an edge segment starts <=3 or ends >=8 */
#define FORK_GAP_MIN 2        /* min blank channels between the two groups */
#define FORK_SEG_MIN 1        /* min width of each group; 1 allows a single lit sensor */
#define FORK_FRAMES 4         /* consecutive fork frames before entering BRANCH */
/* Pre-island two-segment frames are still in FOLLOW: hold the line-PD input
   on them so a segment jump cannot slam w for one frame. 0 = off. */
/* Max per-tick change of the line error while the island entry segment is
   being acquired (IS_ARMED). Smooths the pick switch when ISLAND_FORCE_SIDE
   kicks in on the first BRANCH frame; 0 = freeze e for the whole phase. */
#define FORK_E_SLEW 0.05f
/* A real island probes in with only 2-3 lit sensors at ONE edge, while a
   sharp corner shows a wide pattern (a whole side / full bar). Gate the
   pre-island steering hold on that narrow probe so corners are never held. */
#define FORK_HOLD_SEG_MIN 2
#define FORK_HOLD_SEG_MAX 3
#define FORK_HOLD_TAIL 20u      /* 100 ms sticky window after the last narrow pair */
#define FORK_HOLD_RELEASE 20u   /* 100 ms smooth release with FORK_E_SLEW */
/* Sharp-corner veto: a corner lights 5+ sensors on ONE HALF of the bar
   (ch0-5 or ch6-11), while an island probe is only 2-3 at one edge.
   When this fires fork is suppressed, so the corner keeps its normal PD
   response and is never steered into the island state machine. 0 = off. */
#define FORK_CORNER_LIT_MIN 6
/* A one-sensor island probe cannot meet FORK_HOLD_SEG_MIN, so the sticky
   window never opens on the very first frame the pattern appears. Hold e
   for THAT frame only (5 ms) instead of opening a 100 ms window. 0 = off. */
/* Bend severity and corner direction hold. */
#define BEND_RATE_K 0.8f
#define BEND_SEARCH_MIN 0.80f
#define BEND_DBEND_MAX 1.0f
#define SEARCH_SPIN_W 3.0f   /* spin rate while finding the line */
#define SEARCH_SPIN_DIR -1    /* +1/-1: flip if SEARCH spins away from the line */
/* ---- Exit from a special state ---- */
#define EXIT_FRAMES 3         /* consecutive valid frames before returning to FOLLOW */
#define CROSS_TIMEOUT_MS 450u
#define BRANCH_TIMEOUT_MS 150000u /* must exceed ARMED+TURN_IN+LOOP+TURN_OUT+LEAVE */
#define SEARCH_TIMEOUT_MS 1100u
/* ---- Island sub-state machine ---- */
#define ISLAND_OUT_MS 6000u     /* hard fallback: leave LOOP after this */
/* Ignore entry forks until this time has elapsed since BRANCH began. */
#define ISLAND_OUT_BLANK_MS 1000u   /* mask entry forks for the first 1 s */
#define ISLAND_LEAVE_MS 1500u   /* straight running before returning to FOLLOW */
#define ISLAND_OUT_DIR -1       /* +1/-1: final turn direction relative to ISLAND_SIDE */
/* ---- Active island turn ---- */
#define BRANCH_FALL_FRAMES 4     /* consecutive single-group frames needed by trigger 1 */
#define BRANCH_WAIT_MAX_MS 2000u /* safety: force the entrance turn if no fall-back */
#define BRANCH_TURN_MS 100u      /* duration of each turn pulse */
#define BRANCH_TURN_W 1.2f       /* extra angular rate added on top of the PD output */
#define BRANCH_TURN_V 0.20f      /* speed cap during a turn */
#define BRANCH_TURN_DIR (-1)     /* entrance turn: w += DIR*ISLAND_SIDE*W */
/* Wait for the split to return after the entrance fall-back. */
#define BRANCH_TWO_FRAMES 2      /* consecutive 2-segment frames to accept the split */
#define BRANCH_TWO_MAX_MS 400u   /* after the fall-back, turn anyway if no split */
/* ---- Exit turn: smaller than the entrance pulse, and follow the RIGHT
       segment of the two-segment pattern while leaving the island. ---- */
#define ISLAND_TURN_OUT_W 0.6f   /* exit pulse rate; entrance uses BRANCH_TURN_W 1.2 */
#define ISLAND_TURN_OUT_MS 100u  /* exit pulse duration, independent of BRANCH_TURN_MS */
/* After the group-count fall-back, the two-segment pattern reappears during
   the final turn. Follow the RIGHT one then; IS_LOOP is never affected. */
#define ISLAND_OUT_FOLLOW_SIDE course_exit_side() /* +1 right, -1 left, 0 = keep ISLAND_SIDE */
enum { FOLLOW, CROSS, BRANCH, SEARCH };
enum { IS_NONE, IS_ARMED, IS_TURN_IN, IS_LOOP, IS_TURN_OUT, IS_LEAVE };
static int track_state, edge_side, fork_frames, clear_frames;
static int side_flip_frames, track_d_ready, track_bend_ready;
enum { CORNER_IDLE, CORNER_ARC, CORNER_APPROACH, CORNER_BRAKE, CORNER_PIVOT };
static int corner_phase, corner_dir, corner_rearm, corner_left_line, corner_near_goal;
enum { CORNER_HANDOFF_NONE, CORNER_HANDOFF_LINE, CORNER_HANDOFF_ANGLE };
static int corner_handoff_reason;
static unsigned corner_started_ms, corner_stop_frames;
static float corner_base_yaw, corner_start_s, corner_goal_s, corner_finished_s;
static float corner_progress, corner_incoming_yaw;
static unsigned corner_incoming_ms;
static int corner_incoming_valid;
static int right_angle_shape_side;
static unsigned right_angle_shape_frames;
static int right_angle_pending_bend_side;
static int island_phase, island_fall_frames, island_out_armed;
static int island_fell, island_two_wait, island_two_frames;
static unsigned island_fall_ms;
static unsigned state_ms, island_phase_start;
static int fork_hold_tail, fork_release;
static float entry_yaw, track_bend_prev;
#if SCOPE_PATTERN
static int dbg_groups, dbg_total, dbg_fork, dbg_corner;
#endif
#if SCOPE_STEERING
static float dbg_v, dbg_w, dbg_wcap;
#endif
#if !APP_SCOPE
static int dbg_island_pair, dbg_one_lit;
static unsigned straight_ms;
#define BEND_STRAIGHT_MS 500u
#define BEND_STRAIGHT_E 0.35f
#endif
#if SCOPE_OBSERVER
#include "course_element_test.h"
#endif

/* Nonlinear line PD and gyro damping. */
#define STEER_W_LIMIT 20.0f
#define STEER2_KP2 1.50f
#define STEER2_GYRO_KD 0.06f
#define RATE_GYRO_TAU 0.020f
static float steer_gyro;

static float steer_line(float e,float dterm){
    return app_clip(-(LINE_KP*e+STEER2_KP2*e*fabsf(e)+LINE_KD*dterm)
                    -STEER2_GYRO_KD*steer_gyro,-STEER_W_LIMIT,STEER_W_LIMIT);
}
static void track_enter(int state){
    track_state=state;
    state_ms=0;
    clear_frames=0;
    right_angle_shape_side=0;right_angle_shape_frames=0;
    right_angle_pending_bend_side=0;
    entry_yaw=app_yaw;
    track_d_ready=0;     /* suppress the D term on the first frame back in FOLLOW */
    track_bend_ready=0;  /* suppress a fake dbend jump on the first frame back */
    island_phase=(state==BRANCH)?IS_ARMED:IS_NONE;
    island_fall_frames=0;island_out_armed=0;island_phase_start=0;
    island_fell=0;island_two_wait=0;island_two_frames=0;
    island_fall_ms=0;
}
static void track_reset(void){
    steer_gyro=0;
    corner_phase=CORNER_IDLE;corner_dir=corner_rearm=0;
    corner_progress=0;corner_incoming_valid=0;
    corner_handoff_reason=CORNER_HANDOFF_NONE;
    track_enter(FOLLOW);
    edge_side=0; fork_frames=0;
    side_flip_frames=0;
    track_bend_prev=0; track_bend_ready=0;
    fork_hold_tail=fork_release=0;
#if !APP_SCOPE
    straight_ms=0;
#endif
}

/* Course state advances ONLY on a confirmed explicit completion event.
   Queue at element exit, commit on the next control tick: the outgoing
   frame must finish selecting its line using the OLD segment's profile. */
static unsigned course_segment=1, course_pass, course_pending;
static unsigned course_armed=1, course_clear_ms, course_enter_ms;
static unsigned course_gap, course_gap_done, course_gap_started, course_gap_seen;
static unsigned course_straight_ms, course_final, course_done;
static unsigned course_start_hold; /* launch heading-lock countdown, frames */
static unsigned course_eight, course_join_armed, course_join_clear, course_join_frames;
static unsigned course_eight_started;
static float course_start_s, course_last_event_s, course_gap_s;
static float course_last_line_yaw, course_eight_s, course_eight_yaw;
/* Heading target after the S4 right turn. */
static float course_gap_target_yaw;
static const CourseSegment *course_spec(void){return &course_map[course_segment-1u];}
static float course_cruise(void){
    return fminf(COURSE_SPEED_MAX_MPS,course_spec()->speed);
}
static int course_side(void){
    return course_eight==EIGHT_SMALL ? EIGHT_SMALL_SIDE:course_spec()->island_side;
}
static int course_exit_side(void){return course_spec()->exit_side;}
static int course_accept(void){
    return course_armed && !course_start_hold;
}
static void course_begin(void){
    course_segment=COURSE_START_SEGMENT;course_pass=course_pending=0;
    course_armed=1;course_clear_ms=0;course_enter_ms=app_elapsed;
    course_start_s=app_s;course_last_event_s=app_s;
    course_gap=course_gap_done=course_gap_seen=course_straight_ms=0;
    course_final=course_done=0;course_eight=EIGHT_OFF;
    course_start_hold=COURSE_START_HOLD_FRAMES;
    course_join_armed=course_join_clear=course_join_frames=0;
    if(course_segment<1 || course_segment>10 || COURSE_GAP_SPEED<=0 ||
       COURSE_GAP_MAX_M<=COURSE_GAP_MIN_M ||
       COURSE_GAP_TURN_RAD<0.0f || COURSE_GAP_TURN_RAD>1.0f ||
       COURSE_GAP_TURN_W<=0.0f ||
       COURSE_FINISH_BAR_SPEED<=0.0f || COURSE_FINISH_BAR_SPEED>COURSE_SPEED_MAX_MPS ||
       (COURSE_GAP_TURN_DIR!=1 && COURSE_GAP_TURN_DIR!=-1)){
        app_fail(13);return;
    }
    for(unsigned i=0;i<10;i++)
        if(course_map[i].speed<=0 || course_map[i].speed>COURSE_SPEED_MAX_MPS ||
           !course_map[i].passes || abs(course_map[i].island_side)!=1 ||
           (course_map[i].checkpoint!=CP_CROSS && course_map[i].checkpoint!=CP_ISLAND)){
            app_fail(13);return;
        }
}
static void course_enter_element(unsigned type){
    if(course_final || type!=course_spec()->checkpoint){app_fail(9);return;}
    if(type==CP_ISLAND && course_spec()->eight){
        course_eight=EIGHT_LARGE_IN;course_eight_s=app_s;course_eight_yaw=app_yaw;
        course_eight_started=app_elapsed;course_join_armed=0;
        course_join_clear=course_join_frames=0;
    }
}
static void course_complete(unsigned type){
    if(type!=course_spec()->checkpoint || !course_armed || course_pending){app_fail(9);return;}
    /* Gap must actually have been crossed, not silently skipped. */
    if(course_spec()->gap && !course_gap_done){app_fail(9);return;}
    if(course_spec()->eight && course_eight!=EIGHT_LARGE_OUT){app_fail(11);return;}
    course_pending=type;
}
static void course_tick(void){
    if(course_start_hold)course_start_hold--;
    if(course_pending){
        course_pending=0;course_pass++;
        course_last_event_s=app_s;course_armed=0;course_clear_ms=0;
        if(course_pass>=course_spec()->passes){
            if(course_segment==S10+1u)course_final=1;
            else{
                course_segment++;course_pass=0;course_start_s=app_s;
                course_enter_ms=app_elapsed;course_gap_done=0;
                course_straight_ms=0;course_eight=EIGHT_OFF;
            }
        }
    }
    if(app_elapsed-course_enter_ms>COURSE_SEGMENT_MS)app_fail(9);
}
/* No global lost-line grace extension. ONLY an armed S4 straight may bridge
   a blank, with a fixed yaw reference and independent distance/time bounds. */
static int course_gap_control(int total,int groups,float bend,float *v,float *w){
    if(!course_gap && track_state==FOLLOW){
        if(total && groups==1 && fabsf(bend)<=COURSE_GAP_E_MAX &&
           fabsf(steer_gyro)<=COURSE_GAP_GYRO_MAX){
            course_straight_ms+=5;course_last_line_yaw=app_yaw;
        }else if(total)course_straight_ms=0;
        float s=app_s-course_start_s;
        if(!total && course_spec()->gap && !course_gap_done &&
           course_straight_ms>=COURSE_GAP_APPROACH_MS &&
           s>=course_spec()->gap_arm_min_m && s<=course_spec()->gap_arm_max_m){
            course_gap=1;course_gap_s=app_s;
            course_gap_started=app_elapsed;course_gap_seen=0;
            /* Yaw grows counter-clockwise (gyro z, GYRO_SIGN 1), so a RIGHT
               turn SUBTRACTS the angle. Flip COURSE_GAP_TURN_DIR to turn left. */
            course_gap_target_yaw=course_last_line_yaw-
                (float)COURSE_GAP_TURN_DIR*COURSE_GAP_TURN_RAD;
        }
    }
    if(!course_gap)return 0;
    float d=app_s-course_gap_s;
    if(d>COURSE_GAP_MAX_M || d< -0.02f || app_elapsed-course_gap_started>COURSE_GAP_MAX_MS){
        app_fail(10);*v=*w=0;return 1;
    }
    /* Early reacquisition means this was not the expected 650 mm gap.
       Stop for inspection rather than driving blindly past visible paint. */
    if(groups==1 && total>0 && total<=CENTRAL_TOTAL_MAX)
        course_gap_seen+=5;
    else course_gap_seen=0;
    app_lost=0;
    if(course_gap_seen>=COURSE_GAP_REACQUIRE_MS){
        if(d<COURSE_GAP_MIN_M){app_fail(10);*v=*w=0;return 1;}
        course_gap=0;course_gap_done=1;course_straight_ms=0;
        track_enter(FOLLOW);edge_side=0;
        app_error_prev=app_line_prev=bend;fork_frames=0;fork_hold_tail=fork_release=0;
        return 0;
    }
    *v=fminf(course_cruise(),COURSE_GAP_SPEED);
    float gap_wcap=COURSE_GAP_TURN_W;
    *w=app_clip(TRACK_HEADING_KP*(course_gap_target_yaw-app_yaw),
                -gap_wcap,gap_wcap);
    track_d_ready=0;return 1;
}
static void course_observe(int groups,int total,int fork,int wide){
    if(!course_armed && !course_final){
        if(track_state==FOLLOW && groups==1 && total>0 && !wide && !fork)
            course_clear_ms+=5;
        else course_clear_ms=0;
        if(course_clear_ms>=COURSE_REARM_MS && app_s-course_last_event_s>=COURSE_REARM_M)
            course_armed=1;
    }
}
/* Called only in the existing island LOOP; optical fork definition unchanged.
   A fresh fork after a released window + leg distance + yaw identifies the
   shared junction. Any guard failure stops instead of silently exiting early. */
static void course_eight_update(int fork){
    if(!course_spec()->eight || track_state!=BRANCH || island_phase!=IS_LOOP)return;
    float d=app_s-course_eight_s, a=fabsf(app_yaw-course_eight_yaw);
    if(d>EIGHT_STAGE_MAX_M || app_elapsed-course_eight_started>EIGHT_STAGE_MAX_MS){app_fail(11);return;}
    if(course_eight==EIGHT_LARGE_OUT)return;
    int ready=course_eight==EIGHT_LARGE_IN ?
        (d>=EIGHT_FIRST_MIN_M && a>=EIGHT_FIRST_MIN_YAW):
        (d>=EIGHT_SMALL_MIN_M && a>=EIGHT_SMALL_MIN_YAW);
    if(!fork){
        course_join_clear+=5;course_join_frames=0;
        if(course_join_clear>=EIGHT_JOIN_CLEAR_MS)course_join_armed=1;
    }else{
        course_join_clear=0;
        course_join_frames=(ready && course_join_armed)?course_join_frames+1:0;
    }
    if(course_join_frames>=FORK_FRAMES){
        course_eight++;
        course_eight_s=app_s;course_eight_yaw=app_yaw;course_eight_started=app_elapsed;
        course_join_frames=course_join_armed=course_join_clear=0;
        island_out_armed=0;track_d_ready=0;
        /* Pick the new side immediately in LOOP; keep optical feedback active.
           No timed blind rotation and no synthetic 'island complete'. */
        app_error_prev=app_line_prev=(float)course_side()*0.5f;
    }
}
static int course_eight_can_exit(void){
    return !course_spec()->eight ||
        (course_eight==EIGHT_LARGE_OUT && app_s-course_eight_s>=EIGHT_RETURN_MIN_M);
}
static void course_limit(float *v,float *w){
    if(*v>0)*v=fminf(*v,course_cruise());
    /* Preserve the direction of line reacquisition but drive an arc. The old
       SEARCH had v=0 and spun in place; all course-mode turns stay forward. */
    if(track_state==SEARCH){*v=course_cruise();}
    /* Slow after the first finish bar; stop after the final bar. */
    if(course_segment==S10+1u){
        if(course_final){course_done=1;App_Stop();*v=*w=0;return;}
        if(course_pass>=1 && *v>0)*v=fminf(*v,COURSE_FINISH_BAR_SPEED);
    }
    /* Launch: the car sits ON the start cross. Hold a straight heading for a
       few frames so the bar pattern cannot kick the steering loop. */
    if(course_start_hold)*w=0.0f;
}

static float corner_speed_limit(float requested){
    float cap=course_cruise();
    if(course_segment==S10+1u && course_pass>=1)
        cap=fminf(cap,COURSE_FINISH_BAR_SPEED);
    return fminf(cap,requested);
}

static void corner_begin(int side,int centered){
    corner_dir=-side; /* left sensor side is negative, left yaw is positive */
    corner_phase=CORNER_TURN_MODE==1?CORNER_ARC:CORNER_APPROACH;
    corner_base_yaw=(corner_incoming_valid &&
        app_elapsed-corner_incoming_ms<=CORNER_INCOMING_MAX_AGE_MS)?
        corner_incoming_yaw:app_yaw;
    corner_incoming_valid=0;
    corner_start_s=app_s;corner_started_ms=app_elapsed;
    corner_stop_frames=0;
    corner_handoff_reason=CORNER_HANDOFF_NONE;
    corner_left_line=!centered;corner_near_goal=0;corner_progress=0;
    edge_side=side;
    app_lost=0;app_il=app_ir=0;app_stall=0;
    track_d_ready=0;fork_frames=0;fork_hold_tail=fork_release=0;
}

/* Return 1 when this manoeuvre owns the current control tick. */
static int corner_run(int centered,int visible,int line_first,int line_last,
                      float line_error,float *v,float *w){
    if(corner_phase==CORNER_IDLE)return 0;
    corner_progress=corner_dir*(app_yaw-corner_base_yaw);
    float remain=CORNER_TARGET_RAD-corner_progress;
    if(app_elapsed-corner_started_ms>=CORNER_TIMEOUT_MS ||
       remain < -CORNER_MAX_OVERSHOOT_RAD){
        app_fail(14);
        return 1;
    }
    if(!centered)corner_left_line=1;
    if(corner_phase==CORNER_APPROACH){
        if(app_s-corner_start_s<CORNER_PIVOT_ADVANCE_M){
            *v=corner_speed_limit(CORNER_PIVOT_APPROACH_MPS);
            *w=app_clip(TRACK_HEADING_KP*(corner_base_yaw-app_yaw),
                        -CORNER_PIVOT_APPROACH_MAX_RAD_S,CORNER_PIVOT_APPROACH_MAX_RAD_S);
            return 1;
        }
        corner_phase=CORNER_BRAKE;
        app_il=app_ir=0;
    }
    if(corner_phase==CORNER_BRAKE){
        if(fabsf(app_vl)<=CORNER_STOP_WHEEL_MPS && fabsf(app_vr)<=CORNER_STOP_WHEEL_MPS)
            corner_stop_frames++;
        else corner_stop_frames=0;
        if(corner_stop_frames<CORNER_BRAKE_STOP_FRAMES)return 1;
        corner_phase=CORNER_PIVOT;
        /* The pivot references the incoming heading captured at detection. */
        app_il=app_ir=0;
    }
    int returning_line=line_last-line_first+1>=CORNER_HANDOFF_LINE_MIN_LIT &&
        (corner_dir>0 ? line_first==CORNER_HANDOFF_LEFT_FIRST_CH :
                        line_last==CORNER_HANDOFF_RIGHT_LAST_CH);
    if(corner_progress>=CORNER_HANDOFF_ANGLE_RAD ||
       (corner_progress>=CORNER_HANDOFF_LINE_MIN_RAD && corner_left_line && returning_line)){
        corner_handoff_reason=corner_progress>=CORNER_HANDOFF_ANGLE_RAD?
            CORNER_HANDOFF_ANGLE:CORNER_HANDOFF_LINE;
        corner_phase=CORNER_IDLE;
        corner_rearm=1;corner_finished_s=app_s;
        track_enter(FOLLOW);
        fork_frames=0;fork_hold_tail=fork_release=0;
        app_error_prev=app_line_prev=line_error;
        app_il=app_ir=0;
        return 0;
    }

    if(corner_phase==CORNER_PIVOT){
        float speed=app_clip(CORNER_HEADING_KP*remain,
                             CORNER_PIVOT_MIN_RAD_S,CORNER_PIVOT_MAX_RAD_S);
        *w=corner_dir*speed;
        return 1;
    }

    if(remain<=CORNER_TOL_RAD && !corner_near_goal){
        corner_near_goal=1;corner_goal_s=app_s;
    }
    if(corner_near_goal && app_s-corner_goal_s>=CORNER_EXIT_MAX_M){
        app_fail(14);
        return 1;
    }
    *v=corner_speed_limit(remain<CORNER_SLOW_RAD?
                          CORNER_EXIT_SPEED_MPS:CORNER_ARC_SPEED_MPS);
    float nominal_w=*v/CORNER_ARC_RADIUS_M;
    float desired_w=corner_dir*app_clip(CORNER_HEADING_KP*remain,-nominal_w,nominal_w);
    if(visible && remain>=CORNER_SLOW_RAD){
        float correction=app_clip(steer_line(line_error,0),
                             -CORNER_ARC_LINE_MAX_RAD_S,CORNER_ARC_LINE_MAX_RAD_S);
        desired_w=corner_dir*fmaxf(0.0f,corner_dir*(desired_w+correction));
    }
    *w=desired_w;
    return 1;
}

static void track_control(float *v,float *w){
    int starts[6],ends[6],groups=0,total=0,previous=0;
    int left_hits=0,right_hits=0;
    float weight=0;
    for(int i=0;i<12;i++){
        int white=MUX_GET_CHANNEL(app_mask,i);
        if(white){
            if(!previous) starts[groups++]=i;
            ends[groups-1]=i;
            total++;
            /* Weighted sum of sensor positions in [-1,+1]: a corner lights one
               whole side -> large |weight|; a cross lights the bar symmetrically
               -> |weight| ~ 0. Continuous, so 7..9-wide segments are covered. */
            weight+=2.0f*i/11.0f-1.0f;
            if(i<6)left_hits++;else right_hits++;
        }
        previous=white;
    }
    /* weight/total == segment centre e for one run; dbend is its per-frame
       change, free of line-width changes. Kept before every early return so
       a state switch cannot fake a jump on the frame we come back. */
    float bend=total?weight/(float)total:0.0f;
    float dbend=track_bend_ready?
        app_clip(bend-track_bend_prev,-BEND_DBEND_MAX,BEND_DBEND_MAX):0.0f;
    track_bend_prev=bend;
    track_bend_ready=1;
    float severity=fabsf(bend)+BEND_RATE_K*fabsf(dbend);
    int wide=(groups==1 && total>=12);
    if(corner_phase==CORNER_IDLE && course_gap_control(total,groups,bend,v,w)){
        right_angle_shape_side=0;right_angle_shape_frames=0;
        right_angle_pending_bend_side=0;
#if SCOPE_PATTERN
        dbg_corner=0;
#endif
        return;
    }
    /* Real island entry: ONE segment probes in from an edge, the other is the
       original straight line near the middle. Requiring BOTH ends to be near
       an edge (the old test) can never match that pattern. */
    int fork=0, island_pair=0, one_lit_probe=0;
    if(groups>=2){
        int last=groups-1;
        int w0=ends[0]-starts[0]+1;
        int wN=ends[last]-starts[last]+1;
        int gap=starts[last]-ends[0]-1;
        int near_left=(starts[0]<=FORK_EDGE_MARGIN);
        int near_right=(ends[last]>=11-FORK_EDGE_MARGIN);
        int shape=(gap>=FORK_GAP_MIN && w0>=FORK_SEG_MIN && wN>=FORK_SEG_MIN);
        /* A fully lit half-bar indicates a corner, not a fork. */
        int corner_lit=0;
        if(FORK_CORNER_LIT_MIN>0){
            if(left_hits>=FORK_CORNER_LIT_MIN ||
               right_hits>=FORK_CORNER_LIT_MIN)corner_lit=1;
        }
        fork=shape && (near_left || near_right);
        if(corner_lit)fork=0;
        /* Narrow edge probe = real island entry. A wide probe, one whole side
           lit, or the full bar means a sharp corner: never open the hold. */
        int probe_narrow=
            (near_left  && w0>=FORK_HOLD_SEG_MIN && w0<=FORK_HOLD_SEG_MAX) ||
            (near_right && wN>=FORK_HOLD_SEG_MIN && wN<=FORK_HOLD_SEG_MAX);
        int corner_wide=wide || corner_lit ||
            (left_hits>=CORNER_HALF_HITS || right_hits>=CORNER_HALF_HITS);
        island_pair=fork && probe_narrow && !corner_wide;
        /* One-lit probe: fork is valid but probe_narrow fails, so no sticky
           window. Flag it for a single-frame hold in the e stage below. */
        one_lit_probe=(fork && !corner_wide &&
            ((near_left && w0==1) || (near_right && wN==1)));
    }
    int central=(groups==1 && total<=CENTRAL_TOTAL_MAX &&
                 fabsf(weight)<CENTRAL_WEIGHT_MAX);
    int corner=0;
    int shape_side=0;
    int pending_bend_side=0;
    if(track_state==FOLLOW && !course_start_hold && groups==1 && total>0){
        if(total>=RIGHT_ANGLE_MIN_LIT && total<=RIGHT_ANGLE_MAX_LIT){
            if(starts[0]==0)shape_side=-1;
            else if(ends[0]==11)shape_side=1;
        }
        /* Confirm a narrow trigger on the next frame before locking direction. */
        if(total<RIGHT_ANGLE_MIN_LIT && fabsf(bend)>=RIGHT_ANGLE_BEND_MIN_OFFSET){
            int bend_side=bend>0.0f?1:-1;
            float outward_step=bend_side*dbend;
            if(right_angle_pending_bend_side==bend_side && outward_step>=0.0f)
                corner=1;
            if(fabsf(bend)+BEND_RATE_K*fmaxf(0.0f,outward_step)>=RIGHT_ANGLE_BEND_MIN)
                pending_bend_side=bend_side;
        }
    }else if(track_state==BRANCH){
        /* Keep the existing turn assistance inside the island. */
        corner=!wide && !fork &&
            (severity>=BEND_SEARCH_MIN || left_hits>=CORNER_HALF_HITS ||
             right_hits>=CORNER_HALF_HITS);
    }
    right_angle_pending_bend_side=pending_bend_side;
    if(!shape_side || shape_side!=right_angle_shape_side)right_angle_shape_frames=0;
    right_angle_shape_side=shape_side;
    if(shape_side){
        if(right_angle_shape_frames<RIGHT_ANGLE_SHAPE_FRAMES)right_angle_shape_frames++;
        if(right_angle_shape_frames>=RIGHT_ANGLE_SHAPE_FRAMES)corner=1;
    }
#if SCOPE_PATTERN
    dbg_groups=groups;dbg_total=total;dbg_fork=fork;dbg_corner=corner;
#endif
#if !APP_SCOPE
    dbg_island_pair=island_pair;
    dbg_one_lit=one_lit_probe;
    if(track_state==FOLLOW && groups==1 && total>0 &&
       total<=CENTRAL_TOTAL_MAX && fabsf(bend)<BEND_STRAIGHT_E){
        if(straight_ms<BEND_STRAIGHT_MS+5u)straight_ms+=5;
    }else straight_ms=0;
#endif
#if SCOPE_OBSERVER
    elem_test_observe(track_state==BRANCH,track_state==CROSS,
        island_phase==IS_LEAVE,wide,fork,groups,total,5u);
#endif
    int corner_side=(fabsf(weight)>0.01f)?(weight>0.0f?1:-1):
                    (right_hits>=left_hits?1:-1);
    int side=0;
    if(groups==1 && total<=6){
        if(ends[0]<=3) side=-1;
        if(starts[0]>=8) side=1;
    }

    int turn_line=groups==1 && total<=6 && total>0;
    int turn_centered=turn_line && fabsf(bend)<=RIGHT_ANGLE_BEND_MIN_OFFSET;
    int line_first=groups==1?starts[0]:-1;
    int line_last=groups==1?ends[0]:-1;
    int corner_handoff=0;
    if(corner_phase!=CORNER_IDLE){
        /* A newly confirmed CROSS can veto an edge-shaped false trigger
           before committed rotation. Later wide masks belong to the turn. */
        if(wide && course_accept() && course_spec()->checkpoint==CP_CROSS &&
           fabsf(app_yaw-corner_base_yaw)<CORNER_TOL_RAD &&
           app_s-corner_start_s<=CORNER_CROSS_VETO_M){
            corner_phase=CORNER_IDLE;
        }else{
            if(corner_run(turn_centered,turn_line,line_first,line_last,bend,v,w))return;
            corner_handoff=1;
        }
    }
    if(corner_rearm && turn_centered && app_s-corner_finished_s>=CORNER_REARM_M)
        corner_rearm=0;
    if(track_state!=FOLLOW || wide || fork || !total)corner_incoming_valid=0;
    else if(turn_centered && fabsf(steer_gyro)<=CORNER_INCOMING_MAX_RATE){
        corner_incoming_yaw=app_yaw;corner_incoming_ms=app_elapsed;
        corner_incoming_valid=1;
    }
    course_observe(groups,total,fork,wide);
    if(track_state==FOLLOW && !corner_handoff){
        if(corner){
            /* One frame is enough to open the window and latch a direction:
               a gradual bend may show an edge pattern for only one frame.
               Once latched, a flip needs CORNER_FRAMES consistent frames. */

            if(!edge_side){
                edge_side=corner_side;side_flip_frames=0;
            }else if(corner_side!=edge_side){
                if(++side_flip_frames>=CORNER_FRAMES){
                    edge_side=corner_side;side_flip_frames=0;
                }
            }else side_flip_frames=0;
        }else side_flip_frames=0;
        /* Use an edge hint only if no corner direction is latched. */
        if(side && !corner && !edge_side){edge_side=side;}
        fork_frames=fork ? fork_frames+1 : 0;
        /* Sticky window keyed on the narrow island probe, NOT on raw fork or
           groups: both flicker as the probe enters the bar, and a per-frame
           hold toggling with them is what shook the head. */
        if(island_pair){fork_hold_tail=FORK_HOLD_TAIL;fork_release=FORK_HOLD_RELEASE;}
        else if(fork_hold_tail>0)fork_hold_tail--;
        if(wide && course_accept()){
            course_enter_element(CP_CROSS);if(!app_running)return;
            track_enter(CROSS);
        }
        else if(fork_frames>=FORK_FRAMES && course_accept()){
            fork_frames=0;
            fork_hold_tail=0;fork_release=0;
            if(!ISLAND_SIDE){app_fail(5);return;}
            course_enter_element(CP_ISLAND);if(!app_running)return;
            track_enter(BRANCH);
        }else if(CORNER_TURN_MODE!=0 && corner && !corner_rearm && !fork &&
                 !wide && !course_gap && !course_final && !course_start_hold){
            corner_begin(corner_side,turn_centered);
            if(corner_run(turn_centered,turn_line,line_first,line_last,bend,v,w))return;
            corner_handoff=1;
        }else if(!total){
            if(!edge_side)edge_side=COURSE_LOSS_SEARCH_DIR;
            track_enter(SEARCH);
        }
    }
    if(corner_handoff){
        if(total)edge_side=0;
        else track_enter(SEARCH); /* retain the turn side for lost-line recovery */
    }
    if(track_state!=FOLLOW)state_ms+=5;
    /* Island sub-state machine: a 2->1 fall-back happens at BOTH the entrance
       and the exit junction, and two-segment patterns reappear during the
       final turn. A pattern-only exit cannot tell those moments apart. */
    if(track_state==BRANCH){
        unsigned ph_ms=state_ms-island_phase_start;
        if(island_phase==IS_ARMED){
            if(groups==1)island_fall_frames++;
            else island_fall_frames=0;
            if(island_fall_frames>=BRANCH_FALL_FRAMES)island_fell=1;
            int trigger=island_fell;
            int hard=0;
            if(state_ms>=BRANCH_WAIT_MAX_MS){
                trigger=1;
                hard=!island_fell;
            }
            /* After the first fall to one group, wait for two groups again. */
            if(trigger && !hard){
                if(groups>=2){
                    if(island_two_frames<BRANCH_TWO_FRAMES)
                        island_two_frames++;
                }else{
                    island_two_frames=0;
                    if(!island_two_wait){
                        island_two_wait=1;island_fall_ms=state_ms;
                    }
                }
                int split_ok=(island_two_frames>=BRANCH_TWO_FRAMES);
                int split_timeout=(island_two_wait &&
                    (int)(state_ms-island_fall_ms)>=(int)BRANCH_TWO_MAX_MS);
                trigger=(split_ok||split_timeout);
            }
            if(trigger){
                island_phase=IS_TURN_IN;island_phase_start=state_ms;
                island_two_wait=0;island_two_frames=0;
            }
        }else if(island_phase==IS_TURN_IN){
            if((int)ph_ms>=(int)BRANCH_TURN_MS){island_phase=IS_LOOP;island_phase_start=state_ms;}
        }else if(island_phase==IS_LOOP){
            course_eight_update(fork);if(!app_running)return;
            int out=0;
            if(fork && state_ms>=ISLAND_OUT_BLANK_MS)
                island_out_armed=1;
            else if(island_out_armed && groups==1)out=1;
            if(state_ms>=ISLAND_OUT_BLANK_MS+ISLAND_OUT_MS && !out &&
               !course_spec()->eight){app_fail(12);return;}
            if(!course_eight_can_exit()){out=0;island_out_armed=0;}
            if(out){island_phase=IS_TURN_OUT;island_phase_start=state_ms;}
        }else if(island_phase==IS_TURN_OUT){
            if((int)ph_ms>=(int)ISLAND_TURN_OUT_MS){island_phase=IS_LEAVE;island_phase_start=state_ms;}
        }else if(island_phase==IS_LEAVE){
            if((int)ph_ms>=(int)ISLAND_LEAVE_MS){
                course_complete(CP_ISLAND);if(!app_running)return;
                track_enter(FOLLOW);
                edge_side=0;
                /* Back in FOLLOW the first pick is driven by app_line_prev.
                   Seed it with the SAME side used during the exit, otherwise
                   the very first frame snaps back to the wrong segment. */
                if(total){
                    int side=(ISLAND_OUT_FOLLOW_SIDE && groups>=2)?
                        ISLAND_OUT_FOLLOW_SIDE:ISLAND_SIDE;
                    int ref=(side>0)?groups-1:0;
                    app_error_prev=app_line_prev=(starts[ref]+ends[ref])/11.0f-1.0f;
                }else app_error_prev=app_line_prev=0;
            }
        }
    }
    if(total || track_state==SEARCH || track_state==CROSS)app_lost=0;
    if(track_state!=FOLLOW){
        /* "Line found" is deliberately looser than "centred": leaving a corner
           the line enters from the edge of the bar, so demanding a centred
           pattern would keep spinning and overshoot. */
        int seen=(groups==1 && total>0 && total<=CENTRAL_TOTAL_MAX);
        int exit_ok;
        if(track_state==SEARCH){
            exit_ok=seen;
        }else if(track_state==BRANCH){
            /* The island exit is owned by island_phase (IS_LEAVE), not by a
               pattern test: two-segment patterns reappear during the final
               turn and would keep resetting a 'seen'-based counter. */
            exit_ok=0;
        }else exit_ok=central; /* CROSS keeps the centred test */
        clear_frames=exit_ok ? clear_frames+1 : 0;
        if(clear_frames>=EXIT_FRAMES){
            if(track_state==CROSS){course_complete(CP_CROSS);if(!app_running)return;}
            track_enter(FOLLOW);
            edge_side=0;
            /* Reset BOTH references: a stale app_line_prev would make the first
               post-exit frame pick a segment near the pre-state line position. */
            app_error_prev=app_line_prev=(starts[0]+ends[0])/11.0f-1.0f;
        }
    }
    if(track_state==SEARCH){
        if(state_ms>SEARCH_TIMEOUT_MS){app_fail(5);return;}
        *v=0; *w=SEARCH_SPIN_DIR*edge_side*SEARCH_SPIN_W; return;
    }
    if(track_state==CROSS){
        if(state_ms>CROSS_TIMEOUT_MS){app_fail(12);return;}
        *v=fminf(TRACK_CRUISE,0.30f);
        *w=app_clip(TRACK_HEADING_KP*(entry_yaw-app_yaw),-0.8f,0.8f);
        return;
    }
    if(track_state==BRANCH && state_ms>BRANCH_TIMEOUT_MS){app_fail(12);return;}
    if(!total){
        track_d_ready=0; /* next line frame must not differentiate across the gap */
        if(track_state==BRANCH){
            /* IS_LEAVE belongs to the exit too: use the smaller exit rate so a
               lost-line nudge cannot undo the gentler final turn. Direction
               expression is unchanged. */
            int d=(island_phase==IS_TURN_OUT)?ISLAND_OUT_DIR:BRANCH_TURN_DIR;
            float nudge_w=(island_phase==IS_TURN_OUT || island_phase==IS_LEAVE)?
                ISLAND_TURN_OUT_W:1.5f;
            *v=0.05f; *w=d*ISLAND_SIDE*nudge_w; return;
        }
        app_motor(0,0);app_cmd_l=app_cmd_r=app_il=app_ir=0;
        app_lost+=5;if(app_lost>=50)app_fail(5);
        return;
    }
    app_lost=0;
    int pick=0;
    float best=10;
    for(int i=0;i<groups;i++){
        float dist=fabsf((starts[i]+ends[i])/11.0f-1.0f-app_line_prev);
        if(dist<best){pick=i;best=dist;}
    }
    /* During ARMED use the nearest segment; then select by entry/exit side. */
    if(track_state==BRANCH && island_phase!=IS_ARMED){
        /* Exiting: the two-segment pattern reappears during the final turn
           and the RIGHT segment is the one leading out. Follow it through
           IS_TURN_OUT + IS_LEAVE instead of the entry-side segment. */
        int side=ISLAND_SIDE;
        /* Only AFTER the fall-back, when two segments reappear, switch to the
           right one. IS_LOOP keeps following ISLAND_SIDE; a single segment
           always picks index 0, so the normal path is unchanged. */
        if(ISLAND_OUT_FOLLOW_SIDE && groups>=2 &&
           (island_phase==IS_TURN_OUT || island_phase==IS_LEAVE))
            side=ISLAND_OUT_FOLLOW_SIDE;
        pick=side<0 ? 0 : groups-1;
    }
    float e=(starts[pick]+ends[pick])/11.0f-1.0f;
    /* Hold and slew the steering error during fork acquisition. */
    int in_window=(track_state==FOLLOW && fork_hold_tail>0);
    int one_lit_hold=(track_state==FOLLOW && one_lit_probe &&
                      !in_window && fork_release==0);
    int entry_slew=(track_state==BRANCH &&
        (island_phase==IS_ARMED || (FORK_E_SLEW>0.0f && island_phase==IS_TURN_IN)));
    if(in_window){e=app_error_prev;fork_release=FORK_HOLD_RELEASE;}
    else if(one_lit_hold)e=app_error_prev;
    else if(track_state==FOLLOW && fork_release>0){
        e=app_error_prev+app_clip(e-app_error_prev,-FORK_E_SLEW,FORK_E_SLEW);
        fork_release--;
    }
    else if(entry_slew)
        e=app_error_prev+app_clip(e-app_error_prev,-FORK_E_SLEW,FORK_E_SLEW);
    /* Differentiate only when the previous frame also followed the line:
       after a lost line / state switch a stale e divided by DT would kick w
       to its limit. LINE_D_MAX caps the remaining single-frame jumps. */
    float de=track_d_ready?app_clip((e-app_error_prev)/DT,-LINE_D_MAX,LINE_D_MAX):0.0f;
    float dterm=(in_window||one_lit_hold||fork_release>0||entry_slew)?0.0f:de;
    *w=steer_line(e,dterm);
#if SCOPE_STEERING
    dbg_wcap=STEER_W_LIMIT;
#endif
    app_error_prev=app_line_prev=e;track_d_ready=1;
    *v=corner_handoff?corner_speed_limit(CORNER_EXIT_SPEED_MPS):TRACK_CRUISE;
    if(fork || track_state==BRANCH)*v=fminf(*v,0.45f);
    /* Active steering only in the two turn phases. Both pulses add to the PD
       output rather than replacing it, so the chosen line is still tracked. */
    if(track_state==BRANCH){
        int dir=0;float turn_w=BRANCH_TURN_W;
        if(island_phase==IS_TURN_IN)dir=BRANCH_TURN_DIR;
        else if(island_phase==IS_TURN_OUT){dir=ISLAND_OUT_DIR;turn_w=ISLAND_TURN_OUT_W;}
        if(dir){
            *w=app_clip(*w+dir*ISLAND_SIDE*turn_w,-STEER_W_LIMIT,STEER_W_LIMIT);
            *v=fminf(*v,BRANCH_TURN_V);
        }
    }
}
static void app_start(void){
    if(app_fault)return;
    if(!app_cal_ready || !app_parameters_ok() ||
       (APP_MODE==6 && CORNER_TURN_MODE==1 &&
        (!(CORNER_ARC_RADIUS_M>0) || CORNER_ARC_SPEED_MPS<=0 ||
         CORNER_EXIT_SPEED_MPS<=0 || CORNER_ARC_LINE_MAX_RAD_S<0)) ||
       (APP_MODE==6 && CORNER_TURN_MODE==2 &&
        (CORNER_PIVOT_ADVANCE_M<0 || CORNER_PIVOT_APPROACH_MPS<=0 ||
         !(CORNER_PIVOT_APPROACH_MAX_RAD_S>=0) ||
         CORNER_PIVOT_MIN_RAD_S<=0 || CORNER_PIVOT_MIN_RAD_S>CORNER_PIVOT_MAX_RAD_S ||
         CORNER_STOP_WHEEL_MPS<=0 || CORNER_BRAKE_STOP_FRAMES==0)) ||
       (APP_MODE==6 && CORNER_TURN_MODE!=0 &&
        (CORNER_TOL_RAD<=0 || CORNER_TOL_RAD>=CORNER_SLOW_RAD ||
         CORNER_SLOW_RAD>=CORNER_TARGET_RAD ||
         !(CORNER_HANDOFF_LINE_MIN_RAD>0 && CORNER_HANDOFF_LINE_MIN_RAD<=CORNER_HANDOFF_ANGLE_RAD &&
           CORNER_HANDOFF_ANGLE_RAD<=CORNER_TARGET_RAD) ||
         CORNER_TIMEOUT_MS==0 || CORNER_HEADING_KP<=0 ||
         CORNER_EXIT_SPEED_MPS<=0 || CORNER_REARM_M<0 || CORNER_EXIT_MAX_M<=0)) ||
       (APP_MODE!=6 && (TEST_SPEED_MPS<=0 || TEST_SPEED_MPS>0.5f ||
                        MIN_SPEED_MPS>TEST_SPEED_MPS)) ||
       (APP_MODE==4 && (STOP_AHEAD_M<0 || STOP_AHEAD_M>=0.995f ||
                        DISTANCE_KP<=0 || HEADING_KP<=0))){
        app_fail(4);return;
    }
    app_s=app_yaw=app_il=app_ir=app_error_prev=app_line_prev=0;
    app_elapsed=app_lost=app_stall=0;
    track_reset();
    app_last_completed=app_ms;app_running=1;
#if SCOPE_OBSERVER
    elem_test_reset();
#endif
    if(APP_MODE==6)course_begin();
}
static void app_control(float gz_rad){
    float v=0,w=0,tl,tr;
#if SCOPE_STEERING
    dbg_v=0;dbg_w=0;dbg_wcap=STEER_W_LIMIT;
#endif
#if SCOPE_WHEEL_TARGETS
    app_target_l=0;app_target_r=0;
#endif
    if(!app_running)return;
    app_elapsed+=5;
    if(app_elapsed>(APP_MODE==6?COURSE_TOTAL_MS:120000u)){app_fail(6);return;}
    app_yaw+=gz_rad*DT;
    steer_gyro += (DT/(RATE_GYRO_TAU+DT))*(gz_rad-steer_gyro);
    if(APP_MODE==3){
        if(app_elapsed>=10000u){App_Stop();return;}
        v=TEST_SPEED_MPS;
    }else if(APP_MODE==4){
        float remaining=1.0f-app_s;
        if(remaining<=0.005f+STOP_AHEAD_M){App_Stop();return;}
        v=fmaxf(MIN_SPEED_MPS,fminf(TEST_SPEED_MPS,DISTANCE_KP*remaining));
        w=app_clip(-HEADING_KP*app_yaw,-0.8f,0.8f);
    }else{
        course_tick();if(!app_running)return;
        track_control(&v,&w);
        if(!app_running || app_lost)return;
        if(corner_phase==CORNER_IDLE)course_limit(&v,&w);
        if(!app_running)return;
        if(corner_phase==CORNER_BRAKE){
            app_motor(0,0);app_cmd_l=app_cmd_r=app_il=app_ir=0;
            return;
        }
    }

#if SCOPE_STEERING
    dbg_v=v;dbg_w=w;
#endif
    tl=v-w*EFFECTIVE_TRACK_M*0.5f;tr=v+w*EFFECTIVE_TRACK_M*0.5f;
    /* Keep the original low-speed compensation for both wheel targets. */
    if(APP_MODE==6 && corner_phase==CORNER_IDLE && MIN_SPEED_MPS>0){
        float outer=fmaxf(fabsf(tl),fabsf(tr));
        if(outer>0 && outer<MIN_SPEED_MPS){
            float scale=MIN_SPEED_MPS/outer;
            tl*=scale; tr*=scale;
        }
        float min_wheel_target=MIN_WHEEL_TARGET_FRACTION*MIN_SPEED_MPS;
        if(fabsf(tl)<min_wheel_target){tl=0;app_il=0;}
        if(fabsf(tr)<min_wheel_target){tr=0;app_ir=0;}
    }
#if SCOPE_WHEEL_TARGETS
    app_target_l=tl;app_target_r=tr;
#endif
    int ff_turn=(APP_MODE==6 && (track_state==SEARCH || corner_phase==CORNER_PIVOT));
    float ffl=ff_turn?SPEED_FF_TURN_L:SPEED_FF_L;
    float ffr=ff_turn?SPEED_FF_TURN_R:SPEED_FF_R;
    app_cmd_l=app_pi(tl,app_vl,&app_il,SPEED_KP_L,SPEED_KI_L,ffl);
    app_cmd_r=app_pi(tr,app_vr,&app_ir,SPEED_KP_R,SPEED_KI_R,ffr);
    /* Sustained high effort with no motion: stop instead of heating a motor. */
    if((fabsf(app_cmd_l)>MAX_PWM*0.9f && fabsf(app_vl)<0.003f)||
       (fabsf(app_cmd_r)>MAX_PWM*0.9f && fabsf(app_vr)<0.003f))app_stall+=5;
    else app_stall=0;
    if(app_stall>=800u){app_fail(8);return;}
    app_motor(app_cmd_l,app_cmd_r);
}
static void App_Init(void){
    App_Stop();
    if(MAX_PWM<=0 || MAX_PWM>0.85f){app_fail(4);return;}
    CoreDebug->DEMCR|=CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL|=DWT_CTRL_CYCCNTENA_Msk;
    app_imu_init=dodo_BMI270_init();
    if(app_imu_init!=0){app_fail(1);return;}
    if(HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL)!=HAL_OK ||
       HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_ALL)!=HAL_OK){app_fail(1);return;}
    app_prev_l=(uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
    app_prev_r=(uint16_t)__HAL_TIM_GET_COUNTER(&htim4);
    if(HAL_TIM_Base_Start_IT(&htim2)!=HAL_OK)app_fail(1);
}
static void app_log(void){
    /* Only this function owns UART3 TX during runtime. No blocking printf. */
    if(app_tx_pos<app_tx_len && __HAL_UART_GET_FLAG(&huart3,UART_FLAG_TXE)){
        huart3.Instance->DR=(uint8_t)app_tx[app_tx_pos++];
    }
    uint32_t period=APP_SCOPE?APP_SCOPE_PERIOD_MS:200u;
    if(app_tx_pos>=app_tx_len && (uint32_t)(HAL_GetTick()-app_last_log)>=period){
        int n;app_last_log=HAL_GetTick();
#if APP_SCOPE
#if APP_SCOPE_STATE == 6
            /* Phase, signed yaw progress (deg x10), v (mm/s), w (mrad/s),
               handoff reason (0 none, 1 line, 2 angle), mask, fault, and mode. */
            n=snprintf(app_tx,sizeof(app_tx),"%d,%ld,%ld,%ld,%d,%u,%u,%d\n",
                corner_phase,(long)(corner_dir*corner_progress*1800.0f/PI_F),
                (long)(dbg_v*1000),(long)(dbg_w*1000),corner_handoff_reason,
                (unsigned)app_mask,app_fault,CORNER_TURN_MODE);
#elif APP_SCOPE_STATE == 5
            /* COURSE segment diagnostics (was STATE 3 in the 11-08 framework).
               CH1 segment; CH2 completed passes; CH3 element state;
               CH4 gap active/done (1/2); CH5 eight phase; CH6 segment mm;
               CH7 fault; CH8 final approach/done (1/2). */
            n=snprintf(app_tx,sizeof(app_tx),"%u,%u,%d,%u,%u,%ld,%u,%u\n",
              course_segment,course_pass,track_state*10+island_phase,
              course_gap?1u:(course_gap_done?2u:0u),course_eight,
              (long)((app_s-course_start_s)*1000),app_fault,course_done?2u:course_final);
#elif APP_SCOPE_STATE == 4
            n=elem_test_csv(app_tx,(unsigned)sizeof(app_tx));
#elif APP_SCOPE_STATE == 3
            /* Corner / steering diagnostics: every value from the SAME
               50 Hz tick. CH1 v cmd, CH2 v measured, CH3 w cmd, CH4 corner,
               CH5 w cap, CH6 lit count. APP_SCOPE_GYRO_CH7 selects the tail. */
#if APP_SCOPE_GYRO_CH7 == 1
            n=snprintf(app_tx,sizeof(app_tx),
              "%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld\n",
              (long)(dbg_v*1000),(long)((app_vl+app_vr)*500.0f),
              (long)(dbg_w*1000),(long)dbg_corner,
              (long)(dbg_wcap*1000),(long)dbg_total,
              (long)(steer_gyro*1000),
              (long)(track_state*10+island_phase),(long)(app_cmd_r*1000));
#elif APP_SCOPE_GYRO_CH7 == 2
            n=snprintf(app_tx,sizeof(app_tx),
              "%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld\n",
              (long)(dbg_v*1000),(long)((app_vl+app_vr)*500.0f),
              (long)(dbg_w*1000),(long)dbg_corner,
              (long)(dbg_wcap*1000),(long)dbg_total,
              (long)(steer_gyro*1000),(long)(app_cmd_r*1000));
#else
            n=snprintf(app_tx,sizeof(app_tx),
              "%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld\n",
              (long)(dbg_v*1000),(long)((app_vl+app_vr)*500.0f),
              (long)(dbg_w*1000),(long)dbg_corner,
              (long)(dbg_wcap*1000),(long)dbg_total,
              (long)(track_state*10+island_phase),(long)(app_cmd_r*1000));
#endif
#elif APP_SCOPE_STATE == 2
            /* All 8 channels dedicated to the island state machine (50 Hz).
               CH1 state code = track_state*10 + island_phase
                    (0 FOLLOW, 10 CROSS, 20..25 BRANCH phases, 30 SEARCH)
               CH2 state_ms, CH3 lit groups, CH4 fork, CH5 corner,
               CH6 fork_frames, CH7 island_out_armed, CH8 RIGHT PWM. */
            n=snprintf(app_tx,sizeof(app_tx),
              "%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld\n",
              (long)(track_state*10+island_phase),(long)state_ms,
              (long)dbg_groups,(long)dbg_fork,(long)dbg_corner,
              (long)fork_frames,(long)island_out_armed,
              (long)(app_cmd_r*1000));
#else
            /* CH1 L target, CH2 L measured, CH3 R target, CH4 R measured (mm/s).
               CH5 run, CH6 fault, CH7 calibrated (or state code when
               APP_SCOPE_STATE==1), CH8 RIGHT PWM in 0.001 duty. */
            float ltarget=(app_running && !app_fault)?app_target_l:0;
            float rtarget=(app_running && !app_fault)?app_target_r:0;
            long ch7;
#if APP_SCOPE_STATE == 1
            ch7=(long)(track_state*10+island_phase);
#else
            ch7=(long)app_cal_ready;
#endif
            n=snprintf(app_tx,sizeof(app_tx),
              "%ld,%ld,%ld,%ld,%u,%u,%ld,%ld\n",
              (long)(ltarget*1000),(long)(app_vl*1000),
              (long)(rtarget*1000),(long)(app_vr*1000),
              app_running,app_fault,ch7,
              (long)(app_cmd_r*1000));
#endif
#else
            /* One short group per 200 ms; groups are sampled separately.
               Each group repeats every 1.2 seconds. UART transmission remains
               nonblocking and APP_SCOPE=1 keeps its original format. */
            static unsigned text_group;
            switch(text_group){
            case 0:
                n=snprintf(app_tx,sizeof(app_tx),
                    "A M%d run%u F%u cal%u key%u\r\n",
                    APP_MODE,app_running,app_fault,app_cal_ready,app_key_stable);
                break;
            case 1:
                /* vc = commanded body speed, w = angular rate, wl = w cap
                   (all x1000), from the same frame as vL/vR/uL/uR. */
                n=snprintf(app_tx,sizeof(app_tx),
                    "B L%ld R%ld vL%ld vR%ld uL%ld uR%ld vc%ld w%ld wl%ld\r\n",
                    (long)app_counts_l,(long)app_counts_r,
                    (long)(app_vl*1000),(long)(app_vr*1000),
                    (long)(app_cmd_l*1000),(long)(app_cmd_r*1000),
                    (long)(dbg_v*1000),(long)(dbg_w*1000),
                    (long)(dbg_wcap*1000));
                break;
            case 2:
                n=snprintf(app_tx,sizeof(app_tx),
                    "C s%ld a%ld z%ld q%03X S%d P%d T%lu\r\n",
                    (long)(app_s*1000),(long)(app_yaw*1800/PI_F),
                    (long)(app_gz*10),(unsigned)app_mask,
                    track_state,island_phase,(unsigned long)state_ms);
                break;
            case 3:
                n=snprintf(app_tx,sizeof(app_tx),
                    "D G%d tot%d fk%d cn%d ff%d oa%d fl%d\r\n",
                    dbg_groups,dbg_total,dbg_fork,dbg_corner,
                    fork_frames,island_out_armed,island_fall_frames);
                break;
            case 4:
                n=snprintf(app_tx,sizeof(app_tx),
                    "E tw%d tf%d cf%d ph%lu ip%d sm%lu kg%ld ol%d\r\n",
                    island_two_wait,island_two_frames,clear_frames,
                    (unsigned long)(state_ms-island_phase_start),
                    dbg_island_pair,(unsigned long)straight_ms,
                    (long)(LINE_KP*100.0f),dbg_one_lit);
                break;
            default:
                n=snprintf(app_tx,sizeof(app_tx),"R S%u pass%u gap%u/%u eight%u end%u F%u\r\n",
                    course_segment,course_pass,course_gap,course_gap_done,course_eight,course_final,app_fault);
                break;
            }
            text_group=(text_group+1u)%6u;
#endif
        app_tx_len=n>0?(unsigned)n:0;
        if(app_tx_len>=sizeof(app_tx))app_tx_len=sizeof(app_tx)-1;
        app_tx_pos=0;
    }
}
static void App_Poll(void){
    uint32_t now=app_ms,gap=now-app_last_sample;
    struct bmi2_sens_data acc,gyr;
    uint16_t l,r;int32_t dl,dr;unsigned key;
    app_log();
    if(app_fault || gap<5u)return;
    app_last_sample=now;
    /* Enforce the sampling deadline while driving. */
    if(gap>7u && app_running){app_fail(3);return;}
    l=(uint16_t)__HAL_TIM_GET_COUNTER(&htim3);r=(uint16_t)__HAL_TIM_GET_COUNTER(&htim4);
    dl=app_delta(l,app_prev_l);dr=app_delta(r,app_prev_r);app_prev_l=l;app_prev_r=r;
#if !APP_SCOPE
    app_counts_l+=dl;app_counts_r+=dr;
#endif
    if(app_running && (dl>5000 || dl< -5000 || dr>5000 || dr< -5000)){app_fail(7);return;}
    app_vl=dl*LEFT_ENCODER_SIGN*METRES_PER_COUNT_L/DT;
    app_vr=dr*RIGHT_ENCODER_SIGN*METRES_PER_COUNT_R/DT;
    if(app_running)app_s+=(app_vl+app_vr)*0.5f*DT;
    MUX_get_value(&app_mask);
    if(bmi270_get_accel_gyro(&bmi270,&acc,&gyr)!=BMI2_OK){app_fail(2);return;}
    app_gz=BMI270_gyro_transition(gyr.gyr.z);
    if(!isfinite(app_gz)){app_fail(2);return;}
    if(!app_cal_ready){
        if(dl || dr || fabsf(app_gz)>5.0f){app_cal_samples=0;app_bias_sum=app_bias_square=0;}
        else{
            app_bias_sum+=app_gz;app_bias_square+=app_gz*app_gz;app_cal_samples++;
            if(app_cal_samples>=400){
                app_bias=app_bias_sum/400;
                if(app_bias_square/400-app_bias*app_bias<0.25f)app_cal_ready=1;
                else{app_cal_samples=0;app_bias_sum=app_bias_square=0;}
            }
        }
    }
    key=HAL_GPIO_ReadPin(GPIO_RUN_GPIO_Port,GPIO_RUN_Pin)==GPIO_PIN_SET;
    if(key==app_key_prev){if(app_key_ticks<4)app_key_ticks++;}
    else{app_key_ticks=0;app_key_prev=key;}
    if(app_key_ticks>=4 && key!=app_key_stable){
        app_key_stable=key;
        if(!key){if(app_running)App_Stop();else app_start();}
    }
    if(app_running && (uint32_t)(app_ms-now)>2u){app_fail(3);return;}
    app_control((app_gz-app_bias)*GYRO_SIGN*PI_F/180.0f);
    app_last_completed=app_ms;
}
#endif
