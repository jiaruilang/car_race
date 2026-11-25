#include <string.h>
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

/* 0 sensors; 1 left pulse; 2 right pulse; 3 speed; 4 one metre;
   5 four left 90-degree turns; 6 line following; 7 static steering bench
   (v=0, spin in place, nudge the car and watch it recover). */
#define APP_MODE 6
/* MODE8: straight loaded-wheel feedforward calibration, one speed per run.
   CSV: target, left/right measured (mm/s), left/right duty (x1000), run, fault. */
#define FF_TEST_SPEED_MPS 0.31f
#define FF_TEST_RUN_MS 3000u
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
#define SPEED_FF_L 0.13f    /* feedforward duty per (m/s) of target; 0=off; L/R independent */
#define SPEED_FF_R 0.107f    /* tune = steady_duty/steady_speed, e.g. 0.18/0.30=0.6 */
#define SPEED_FF_TURN_L 0.13f  /* MODE5 only: spin-in-place load differs from straight running */
#define SPEED_FF_TURN_R 0.107f /* tune = steady turn duty / steady turn wheel speed */
#define MAX_PWM 0.70f
#define TEST_PWM 0.042f
#define TEST_REVERSE 0
#define TEST_SPEED_MPS 1.0f
#define MIN_SPEED_MPS 0.200f  /* lowest sustainable speed (m/s); 0=off; fill after calibration */
#define STOP_AHEAD_M 0.05f   /* coast distance after cut-off (m); stop early by this; 0=off */
#define LINE_KP 2.5f       /* initial trial only; error range -1..1 */
#define LINE_KD 0.02f
#define LINE_D_MAX 20.0f  /* clamp on de/dt so a stale error cannot kick the D term */
#define HEADING_KP 2.0f   /* MODE4 only */
#define TURN_KP 1.0f
#define TURN_MAX_RAD_S 4.0f
#define TURN_TOL_RAD 0.025f  /* angle window for a completed quarter turn; also releases the speed floor */
#define MIN_SPEED_FLOOR_MODE6 1 /* 1: apply the wheel-speed floor in MODE6 too; 0: line mode exempt */
#define MODE7_CRUISE_MPS 0.16f /* 0=off (pure spin in place). Forward bias
                                  during MODE7 recovery, and the hard floor
                                  on BOTH |tl|,|tr|; never below 0.15. */
#define DISTANCE_KP 1.5f
#define APP_SCOPE 1       /* 0 text diagnostics; 1 Seekfree CSV scope */
#define APP_SCOPE_PERIOD_MS 20u /* 50Hz nominal telemetry; control stays 5ms */
/* State-machine telemetry on the Seekfree scope.
   0 = original 8 channels (CH7 = cal)
   1 = CH7 becomes the state code, other 7 channels untouched
   2 = all 8 channels become island state-machine diagnostics (50 Hz)
   3 = corner / feed-forward speed test: v cmd, v measured, w cmd, corner,
       w cap, lit count, [gyro rate], state code, R PWM
   4 = course element recognition observer CSV (course_element_test.h)
   5 = COURSE segment diagnostics: segment, pass, element, gap, eight,
       segment mm, fault, finish  <-- was 3 in the 11-08 framework.
       Renumbered because 3/4 already mean something in the current project. */
#ifndef APP_SCOPE_STATE
#define APP_SCOPE_STATE 5 /* corner calibration, 5 ms */
#endif
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
#define DT 0.005f
#define PI_F 3.14159265358979f /* TRUE pi: physical dps->rad/s and log conversion ONLY */
#define TURN_STEP_RAD (PI_F*0.5f*1.01) /* MODE5: target yaw per quarter; tune here for angle bias */
#define TRACK_HEADING_KP 2.0f /* MODE6 CROSS: own heading-hold gain, independent of MODE4 */

extern struct bmi2_dev bmi270; /* defined in the official dodo_BMI270.c */
static volatile uint32_t app_ms, app_last_completed;
static volatile unsigned app_running, app_fault;
static uint32_t app_last_sample, app_last_log, app_elapsed;
static uint16_t app_prev_l, app_prev_r, app_mask;
static int32_t app_counts_l, app_counts_r;
static float app_s, app_yaw, app_il, app_ir, app_vl, app_vr;
static float app_error_prev, app_line_prev, app_cmd_l, app_cmd_r;
static float app_target_l, app_target_r;
static float app_bias, app_bias_sum, app_bias_square, app_gz;
static unsigned app_cal_samples, app_cal_ready, app_hold, app_quarter;
static unsigned app_lost, app_settled, app_stall;
static int app_imu_init;
static unsigned app_key_prev=1, app_key_stable=1, app_key_ticks;
static char app_tx[320]; /* text mode now carries the state-machine tail */
static unsigned app_tx_pos, app_tx_len;
/* fault:1 init;2 acquisition;3 scheduling;4 calibration/missing parameters;
   5 lost line;6 timeout;7 encoder jump;8 stall. Reset to clear fault. */
static float app_clip(float x,float lo,float hi){return x<lo?lo:x>hi?hi:x;}
static void App_Stop(void){
    TIM1->CCR1=0;TIM1->CCR2=0;app_running=0;
    app_il=0;app_ir=0;app_cmd_l=0;app_cmd_r=0;
    app_target_l=0;app_target_r=0;
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
      MAX_PWM>0 && MAX_PWM<=0.85f && TEST_SPEED_MPS>0 && TEST_SPEED_MPS<=2.0f &&
      MIN_SPEED_MPS>=0 && MIN_SPEED_MPS<=TEST_SPEED_MPS && STOP_AHEAD_M>=0 &&
      TURN_TOL_RAD>0 && TURN_STEP_RAD>0 && TRACK_HEADING_KP>0 && LINE_D_MAX>0 &&
      (MODE7_CRUISE_MPS==0.0f ||
       (MODE7_CRUISE_MPS>=0.15f && MODE7_CRUISE_MPS<=TEST_SPEED_MPS));
}
/* MODE6 trial settings. Channels 0..11 run from left to right. */
#define TRACK_CRUISE course_cruise()
#define ISLAND_SIDE course_side() /* -1 left, +1 right, 0 stop at candidate for inspection */
#define CENTRAL_TOTAL_MAX 9     /* widest single segment still a normal line (was 6: 7..9 grey band) */
#define CENTRAL_WEIGHT_MAX 2.0f /* |weighted sum| below this => line is centred */
#define CORNER_HALF_HITS 6      /* all 6 sensors on one side lit => corner */
#define CORNER_WEIGHT_MIN 2.5f  /* |weighted sum| above this => one side dominates */
#define CORNER_DWEIGHT_MIN 1.5f /* jump in weighted sum from last frame => shape change */
#define TRACK_RECOVER_ON_TIMEOUT 1 /* 1: CROSS/BRANCH timeouts return to FOLLOW instead of fault 5 */
/* ---- Fork / island entry ---- */
#define FORK_MODE 2           /* 0 both ends; 1 one edge; 2 any two or more separate runs */
#define FORK_EDGE_MARGIN 4    /* an edge segment starts <=3 or ends >=8 */
#define FORK_GAP_MIN 1        /* min blank channels between the two groups */
#define FORK_SEG_MIN 2        /* minimum consecutive lit sensors per qualifying run */
#define FORK_FRAMES 1         /* consecutive fork frames before entering BRANCH */
/* Pre-island two-segment frames are still in FOLLOW: hold the line-PD input
   on them so a segment jump cannot slam w for one frame. 0 = off. */
#define FORK_ENTRY_HOLD 1
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
#define FORK_HOLD_ONE_LIT 1
/* ---- Continuous bend handling: weight/total is the line centre, dbend is its
       per-frame change (width-free), severity contributes to corner detection. ---- */
#define BEND_ENABLE 1          /* 0 = fall back to the old weight-only corner test */
#define BEND_RATE_K 0.8f      /* weight of |dbend| in severity */
#define BEND_SEARCH_MIN 0.90f /* severity that counts as a corner (was 0.80) */
#define BEND_DBEND_MAX 1.0f   /* clamp on a single-frame bend change */
/* Centred-line dwell for text telemetry. */
#define BEND_STRAIGHT_MS 500u
#define BEND_STRAIGHT_E 0.35f
/* ---- Direction latching ---- */
#define SIDE_ENABLE 1         /* update search direction from a single edge run */
/* SEARCH now reuses CORNER_PULSE_W, so the lost-line arc and the corner
   pulse can never drift apart. The old SEARCH_SPIN_W is gone: change the
   shared rate in the corner block instead. */
#define SEARCH_SPIN_DIR -1    /* +1/-1: flip if SEARCH spins away from the line */
/* ---- Exit from a special state ---- */
#define EXIT_FRAMES 3         /* consecutive valid frames before returning to FOLLOW */
#define SEARCH_EXIT_FRAMES 1
#define SEARCH_EXIT_MODE 1    /* 0 centred only; 1 centre window (line back); 2 1 + moving inward */
/* Legacy pattern-based BRANCH exit knobs below are no longer referenced at
   runtime: the island exit is now owned by island_phase (IS_LEAVE). */
#define BRANCH_EXIT_MODE 1    /* 0 centred only; 1 any line + dwell; 2 + yaw; 3 + distance */
#define BRANCH_MIN_MS 100u    /* minimum dwell in BRANCH before an exit is allowed */
#define BRANCH_MIN_YAW 0.50f  /* rad of heading change for BRANCH_EXIT_MODE 2 */
#define BRANCH_MIN_DIST 0.15f /* metres travelled for BRANCH_EXIT_MODE 3 */
#define CROSS_TIMEOUT_MS 450u
#define BRANCH_TIMEOUT_MS 150000u /* must exceed ARMED+TURN_IN+LOOP+TURN_OUT+LEAVE */
#define SEARCH_TIMEOUT_MS 1100u
#define BRANCH_NUDGE_MS 100u
/* ---- Island sub-state machine ---- */
#define ISLAND_FORCE_SIDE 1    /* 1: lock onto the ISLAND_SIDE segment for the whole island */
#define ISLAND_OUT_TRIGGER 1   /* 0 time only; 1 second fork->fall; 2 heading; 3 distance */
#define ISLAND_OUT_YAW 3.00f   /* rad for trigger 2 */
#define ISLAND_OUT_DIST 0.60f  /* metres for trigger 3 */
#define ISLAND_OUT_MS 8000u     /* exit deadline from BRANCH for fork trigger */
/* Consecutive fork frames needed to ARM the exit. Was 1 frame, which let a
   single glitch (or the trailing "2" of a 2-1-2 at the same junction) pull
   the exit in early. 3 frames = 15 ms of confirmation, the same order as
   FORK_FRAMES for the island entry. 1 restores the old single-frame arm. */
#define ISLAND_OUT_FORK_FRAMES 3
#if ISLAND_OUT_FORK_FRAMES < 1 || ISLAND_OUT_FORK_FRAMES > 20
#error Invalid ISLAND_OUT_FORK_FRAMES
#endif
#define ISLAND_LEAVE_MS 60u   /* straight running before returning to FOLLOW */
#define ISLAND_OUT_DIR -1       /* +1/-1: final turn direction relative to ISLAND_SIDE */
/* ---- Active island turn ---- */
#define BRANCH_TURN_ENABLE 1     /* 0 = PD only, no turn pulses */
#define BRANCH_TURN_TRIGGER 1    /* 0 immediate; 1 wait for groups to fall to 1; 2 distance */
#define BRANCH_FALL_FRAMES 4     /* consecutive single-group frames needed by trigger 1 */
#define BRANCH_WAIT_MAX_MS 2000u /* safety: force the entrance turn if no fall-back */
#define BRANCH_TURN_DIST 0.08f   /* trigger 2: metres from fork detection to the junction */
#define BRANCH_TURN_MS 100u      /* duration of each turn pulse */
#define BRANCH_TURN_W 1.2f       /* extra angular rate added on top of the PD output */
#define BRANCH_TURN_DIR (-1)     /* entrance turn: w += DIR*ISLAND_SIDE*W */
/* ---- Wait for the two-segment pattern to come back before turning in ----
   The 2->1 fall-back at the entrance is the 0..9 junction fill (ONE wide
   band), not the island branch. IS_TURN_IN re-enables D and follows that
   band's centre, whose edges flicker frame to frame -> the head twitch.
   With this on, IS_ARMED keeps its e slew + dterm=0 while the band is
   present; the entrance turn starts only once groups>=2 reappears, and the
   pick is then locked onto the LEFT (ISLAND_SIDE) segment.
   0 = old behaviour (turn as soon as the fall-back is counted). */
#define BRANCH_ARM_WAIT_TWO 1
#define BRANCH_TWO_FRAMES 2      /* consecutive 2-segment frames to accept the split */
#define BRANCH_TWO_MAX_MS 400u   /* after the fall-back, turn anyway if no split */
/* ---- Exit turn: smaller than the entrance pulse, and follow the RIGHT
       segment of the two-segment pattern while leaving the island. ---- */
#define ISLAND_TURN_OUT_W 2.0f   /* exit pulse rate; entrance uses BRANCH_TURN_W 1.2 */
#define ISLAND_TURN_OUT_MS 200u  /* exit pulse duration, independent of BRANCH_TURN_MS */
/* After the group-count fall-back, the two-segment pattern reappears during
   the final turn. Follow the RIGHT one then; IS_LOOP is never affected. */
#define ISLAND_OUT_FOLLOW_SIDE course_exit_side() /* +1 right, -1 left, 0 = keep ISLAND_SIDE */
/* Shared middle junction: latch heading through the WHOLE 2 -> 1 -> 2.
   Release only after the trailing split clears to a stable narrow line.
   Eight: only LARGE_IN -> SMALL and SMALL -> LARGE_OUT; outer entry/exit
   retain island control. S8 uses this controller only for the straight entry crossing. */
#define EIGHT_CROSS_ENABLE 1
#define CROSS_STRAIGHT_S8 1     /* independent of EIGHT_CROSS_ENABLE */
#define EIGHT_CROSS_W_CAP 1.0f  /* junction angular speed ceiling, rad/s */
#define JUNCTION_PHASE_FRAMES 2u /* consecutive samples for each 2/1/2 phase */
#define JUNCTION_CLEAR_MS 30u   /* narrow single-line release/rearm dwell */
#define JUNCTION_CLEAR_MAX_LIT 6 /* reject the wide middle band as clearance */
#define JUNCTION_MAX_MS 2000u   /* incomplete/stuck junction -> F11 */
#define JUNCTION_LOST_MS 50u    /* maximum blank interval during hold -> F5 */
#define JUNCTION_HEADING_KP 2.0f /* heading error (rad) -> angular speed */
#if JUNCTION_PHASE_FRAMES < 1 || JUNCTION_CLEAR_MS < 5 || JUNCTION_MAX_MS < JUNCTION_CLEAR_MS || JUNCTION_LOST_MS < 5 || JUNCTION_CLEAR_MAX_LIT < 1 || JUNCTION_CLEAR_MAX_LIT > 11
#error Invalid junction configuration
#endif
#if EIGHT_CROSS_ENABLE < 0 || EIGHT_CROSS_ENABLE > 1
#error Invalid EIGHT_CROSS_ENABLE
#endif
#if CROSS_STRAIGHT_S8 < 0 || CROSS_STRAIGHT_S8 > 1
#error Invalid CROSS_STRAIGHT_S8
#endif
enum { FOLLOW, CROSS, BRANCH, SEARCH };
enum { IS_NONE, IS_ARMED, IS_TURN_IN, IS_LOOP, IS_TURN_OUT, IS_LEAVE };
static int track_state, edge_side, edge_age, fork_frames, clear_frames;
static int track_d_ready, track_bend_ready;
static int island_phase, island_fall_frames, island_out_armed;
static int island_out_pending;
static unsigned exit_single_ms;
static float island_exit_pulse_yaw;
static float ordinary_guard_yaw;
static int ordinary_guard_dir, ordinary_search_active;
static unsigned island_out_frames; /* consecutive fork frames for the exit arm */
enum { ENTRY_WAIT_SPLIT, ENTRY_WAIT_SINGLE, ENTRY_CLEAR };
static int island_entry_gate;
static int island_fell, island_two_wait, island_two_frames, island_two_ok;
static unsigned island_fall_ms;
static unsigned state_ms, island_phase_start;
/* app_log() cannot see track_control()'s locals; publish pattern flags
   once per tick so the state trace shows why a transition fired. */
static unsigned segment_fork_hits;
#if FORK_TEST_MODE
static unsigned fork_test_falls;
#endif
static int dbg_groups, dbg_total, dbg_fork, dbg_corner, dbg_island_pair;
static int junction_speed_hold;
static int small_ring_junction;
static int dbg_one_lit; /* single-lit island probe seen this tick */
static int fork_hold_tail, fork_release; /* sticky pre-island steering hold */
static unsigned straight_ms; /* continuous centred-line time (ms) */
static long dbg_kp_gain; /* kp*100, refreshed on line-following frames only */
/* Published for app_log(): commanded body speed, angular rate and the w cap
   actually used this tick (x1000 on the scope / text log). */
static float dbg_v, dbg_w, dbg_wcap;
static float entry_yaw, entry_s, track_weight_prev, track_bend_prev;
/* Corner pulse state (2026-11-17): a constant-w turn released on geometry.
   Declared before track_control()/app_control() so both can see them. */
static int corner_pulse_active, corner_pulse_side;
static float corner_entry_yaw;
static float s8_entry_split_yaw;
static unsigned corner_pulse_cool_ms;
static uint32_t corner_last_trigger_ms;
/* Junction phases: 0 idle, 1 first split, 2 merged band, 3 final split,
   4 waiting for narrow clearance. Heading/reference survive every phase. */
static struct {
    unsigned phase, frames, clear_ms, started, lost_ms;
    float yaw;
    int selected; /* 0 heading hold, 1 first run, 2 second run */
} junction;
#if SPECIAL_ISLAND_SINGLE_FORK
static int special_heading_hold;
#endif

/* Pure observer from the current project: recognises landmark ORDER and the
   two completed crossings of the same cross. Does NOT drive the car.
   ELEM_TEST_ENABLE 0 turns every hook into a no-op (see header). */
#include "course_element_test.h"

/* 1: line PD -> yaw-rate PID -> wheel PI
   2: nonlinear line PD + gyro damping -> wheel PI
   3: existing line PD -> wheel PI */
#ifndef TRACK_STEER_MODE
#define TRACK_STEER_MODE 2
#endif
#if TRACK_STEER_MODE < 1 || TRACK_STEER_MODE > 3
#error Invalid TRACK_STEER_MODE
#endif
/* ---- Steering rate cap + right-angle corner A/B (from current project) --
   steer_line() used to clip w at a hard-coded +/-2.0 rad/s. CORNER_W_TEST
   raises that cap ONLY on FOLLOW frames where the corner test fires.
   0 = bit-for-bit the 11-08 framework behaviour. */
#define STEER_W_LIMIT 20.0f
#define CORNER_W_TEST 1
#define CORNER_TEST_W_LIMIT 20.0f
/* course_limit()'s kinematic clamp: |w| <= K*v/TRACK_M. 1.8 was the old
   hard-coded literal and stays the default, so extracting it changes no
   frame. It caps the corner pulse at 10*v (= 3.0 rad/s at v=0.30); raise
   THIS first if the pulse has to be quicker. */
#define COURSE_W_LIMIT_K 3.6f
/* Corner speed test: disable the SEARCH line-loss fallback. 0 = original. */
#define CORNER_NO_SEARCH 0
#if CORNER_NO_SEARCH < 0 || CORNER_NO_SEARCH > 1
#error Invalid CORNER_NO_SEARCH
#endif
/* ---- Right-angle corner: ENTRY test + CONSTANT-W PULSE (2026-11-17) ----
   ENTRY replaced the old "one side fully lit" test. A right-angle is
   recognised by a CONSECUTIVE run of >= CORNER_ENTER_RUN lit sensors
   counted FROM ONE EDGE: ch0 upwards (ch0..ch6 at 7) or the mirror,
   ch11 downwards (ch5..ch11 at 7). A centred wide band never reaches an
   edge, so it cannot start a pulse; a two-sided split with a gap in the
   middle does not either. severity >= BEND_SEARCH_MIN (now 0.90) stays as
   the one-sided backstop, and the whole test is disabled in an island
   (track_state == BRANCH).
   DRIVING: the old additive WFF term is gone. While the pulse is active the
   PD output is REPLACED by a constant angular rate, and the release is
   geometric instead of pattern-based:
     - turned >= CORNER_PULSE_EXIT_RAD (~147 deg)                  -> release
     - or the line is back in
       the centre window (>= CORNER_PULSE_CENTER_MIN lit among 5..8,
       1-based numbering)                         -> release
   After release the PD takes over again. CORNER_PULSE_COOL_MS blocks a
   new trigger for 200 ms from the last trigger. Once elapsed, ANY raw hit
   starts a fresh pulse, even if the previous pulse is still active.
   This is level-triggered: a continuously present hit also restarts it.
   NOTE: course_limit() then clips |w| at COURSE_W_LIMIT_K*v/0.18, so a
   pulse wider than that is silently reduced.                              */
#define CORNER_ENTER_RUN 6   /* consecutive sensors counted from one edge */
#define SEARCH_EDGE_RUN 5    /* consecutive sensors from one end to update search direction */
#define CORNER_PULSE_EXIT_RAD 1.658f   /* ~147 deg: unconditional release */
#define CORNER_PULSE_MIN_RAD 0.1f /* minimum yaw for predictive release; 0 enables short bends */
#define CORNER_PULSE_CENTER_MIN 1  /* lit sensors required inside the window */
#define CORNER_PULSE_COOL_MS 25u  /* re-arm window measured from the TRIGGER frame */
#if CORNER_ENTER_RUN < 2 || CORNER_ENTER_RUN > 12
#error Invalid CORNER_ENTER_RUN
#endif
#define PREDICT_FIRST_INDEX 2 /* sensor 4 */
#define PREDICT_LAST_INDEX 9  /* sensor 9 */
#define CENTER_FIRST_INDEX 4 /* sensor 5 */
#define CENTER_LAST_INDEX 7  /* sensor 8 */
#if CORNER_PULSE_CENTER_MIN < 1 || CORNER_PULSE_CENTER_MIN > CENTER_LAST_INDEX-CENTER_FIRST_INDEX+1
#error Invalid CORNER_PULSE_CENTER_MIN
#endif
#if CORNER_PULSE_COOL_MS > 2000u
#error Invalid CORNER_PULSE_COOL_MS
#endif
/* ---- Island entry jitter: ignore the OTHER fork segment (A/B only) -----
   The 11-11 project forced pick=0 while BRANCH_ARM_WAIT_TWO is active in
   IS_TURN_IN, i.e. e came from ONE segment only. The merged framework
   replaced it with ISLAND_SIDE-dependent selection. This is NOT a missed
   port: S8 is a RIGHT-hand island and the framework's own regression
   asserts app_line_prev>0 there, so a fixed pick=0 would select the wrong
   side. Keep the merged behaviour by default; set 1 only to A/B whether
   the dynamic pick is what shakes the head on island entry. */
#define ISLAND_ARM_TWO_FIXED_PICK 0
#if ISLAND_ARM_TWO_FIXED_PICK < 0 || ISLAND_ARM_TWO_FIXED_PICK > 1
#error Invalid ISLAND_ARM_TWO_FIXED_PICK
#endif
#define STEER2_KP2 1.50f
#define STEER2_GYRO_KD 0.06f
#define RATE_KP 0.4f
#define RATE_KI 0.0f
#define RATE_KD 0.0f
#define RATE_GYRO_TAU 0.020f
#define RATE_I_LIMIT 0.5f

/* 2026-11-17: the whole WFF family is gone - no additive feed-forward and
   no speed scaling either. steer_line() now runs a plain PD at every
   segment speed; the only corner action left is the constant-w pulse. */
static float steer_gyro, rate_i, rate_prev;
static int rate_ready;
static void steer_reset(void){
    rate_i=0;rate_prev=0;rate_ready=0;steer_gyro=0;
}
static float steer_line(float e,float dterm,float kp,int valid_previous,
                        float wcap){
    float desired=app_clip(-(kp*e+LINE_KD*dterm),-wcap,wcap);
#if TRACK_STEER_MODE == 1
    if(!valid_previous){rate_i=0;rate_ready=0;}
    float error=desired-steer_gyro;
    float derivative=rate_ready?-(steer_gyro-rate_prev)/DT:0;
    rate_prev=steer_gyro;rate_ready=1;
    float next=app_clip(rate_i+RATE_KI*error*DT,-RATE_I_LIMIT,RATE_I_LIMIT);
    /* desired is a nominal kinematic feedforward; PID corrects rate error. */
    float raw=desired+RATE_KP*error+next+RATE_KD*derivative;
    float output=app_clip(raw,-wcap,wcap);
    if(raw==output || (raw>output && error<0) || (raw<output && error>0))rate_i=next;
    return output;
#elif TRACK_STEER_MODE == 2
    /* Plain PD: the P terms were previously multiplied by v/v0 (speed
       scaling). That is gone, so the same gains apply at every segment
       speed. dterm = de/DT already carries a factor of v, and the gyro
       damping settles to ~w, which is also ~v; neither is scaled again. */
    return app_clip(-((kp*e+STEER2_KP2*e*fabsf(e))+LINE_KD*dterm)
                    -STEER2_GYRO_KD*steer_gyro,-wcap,wcap);
#else
    return desired;
#endif
}
/* ---- MODE7: steering test bench ----------------------------------------
   APP_MODE 7 isolates the steering loop: nudge the body off the line and
   watch the recovery. The MODE6 state machine, bend slowdown, island logic,
   SEARCH and the MODE6 wheel speed floor are all bypassed.
   The car sits still with the motors OFF until |e| exceeds TRIGGER_E; then a
   short correction run opens, with a common forward bias (MODE7_CRUISE_MPS)
   keeping both wheels above the speed floor so the differential is a small
   perturbation on already-moving wheels instead of a deadzone start. The run
   ends once the line is centred for SETTLE_MS (or after ACTIVE_MAX_MS), and
   the motors are cut again - so w->0 means STOP, not a minimum turn rate.
   All three TRACK_STEER_MODE variants are reused unchanged; steer_gyro keeps
   being updated every tick in app_control(), so the gyro feedback of modes
   1 and 2 is live.                                                        */
#define MODE7_W_LIMIT 3.0f    /* cap on |w| (rad/s); current-project value */
/* MODE7_CRUISE_MPS is defined near the top of the file so
   app_parameters_ok() can validate it; applied in app_control(). */
#define MODE7_KICK_PWM 0.0f  /* 0=off. Optional start-up duty floor for tiny
                                wheel targets. A PWM floor creates a limit
                                cycle, so leave it off when judging response. */
#define MODE7_NO_TIMEOUT 1   /* 1: skip the 120 s run timeout while debugging */
#define MODE7_NO_STALL 1     /* 1: skip stall fault 8 (car is held by hand) */
#define MODE7_FF_TURN 1      /* 1: SPEED_FF_TURN_L/R (spin load); 0: straight FF */
/* Sit still with the motors off by default; open a short correction run only
   when the line is off centre. Hysteresis keeps it from chattering. */
#define MODE7_TRIGGER_E 0.02f     /* |e| above this starts a correction run */
#define MODE7_SETTLE_E  0.01f     /* |e| below this for SETTLE_MS ends it */
#define MODE7_SETTLE_MS 200u      /* ms of centred line before stopping */
#define MODE7_ACTIVE_MAX_MS 2000u /* hard cap on one run, so the car cannot roll away */
static float steer_test_e_prev;
static int steer_test_d_ready;
static unsigned mode7_active, mode7_steady_ms, mode7_run_ms;
static void steer_test_control(float *w,float *e_out){
    int starts[6],ends[6],groups=0,total=0,previous=0;
    for(int i=0;i<12;i++){
        int white=MUX_GET_CHANNEL(app_mask,i);
        if(white){
            if(!previous)starts[groups++]=i;
            ends[groups-1]=i;
            total++;
        }
        previous=white;
    }
    if(!total){
        /* The line swept out of the bar: freeze, do not keep turning on a
           stale error, and do not open a correction run from a lost line. */
        *w=0;*e_out=0.0f;
        steer_test_e_prev=0;steer_test_d_ready=0;
        rate_i=0;rate_prev=0;rate_ready=0;
        return;
    }
    /* Same normalised centre error as MODE6, so LINE_KP/LINE_KD transfer. */
    int pick=0;float best=10;
    for(int i=0;i<groups;i++){
        float c=(starts[i]+ends[i])/11.0f-1.0f;
        float d=fabsf(c-steer_test_e_prev);
        if(d<best){pick=i;best=d;}
    }
    float e=(starts[pick]+ends[pick])/11.0f-1.0f;
    *e_out=e;
    float de=steer_test_d_ready?
        app_clip((e-steer_test_e_prev)/DT,-LINE_D_MAX,LINE_D_MAX):0.0f;
    *w=app_clip(steer_line(e,de,LINE_KP,steer_test_d_ready,MODE7_W_LIMIT),
                -MODE7_W_LIMIT,MODE7_W_LIMIT);
    steer_test_e_prev=e;steer_test_d_ready=1;
}
static struct {
    int direction, candidate, centered_frames;
    int last_direction, retrigger_angle_seen;
    float start_yaw;
} island_pulse;
static uint32_t island_last_trigger_ms;

static void island_pulse_reset(void){
    memset(&island_pulse,0,sizeof island_pulse);
}

/* Selected run only; sensor numbers 5..8 correspond to indices 4..7. */
static int island_pulse_centered(int start,int end){
    int lo=start>4?start:4,hi=end<7?end:7;
    return hi-lo+1>=3;
}

/* Called once per loop frame, including frames where the line is lost.
   Keep the last trigger heading/direction after an ordinary pulse exit:
   subsequent line following can still earn the 30-degree rearm latch. */
static int island_pulse_update(int visible,float e,int centered,float *w){
    float turn=island_pulse.last_direction*(app_yaw-island_pulse.start_yaw);
    if(island_pulse.last_direction && turn>=ISLAND_PULSE_RETRIGGER_MIN_RAD)
        island_pulse.retrigger_angle_seen=1;
    if(island_pulse.direction){
        island_pulse.centered_frames=centered ? island_pulse.centered_frames+1 : 0;
        if(island_pulse.centered_frames>=2 || turn>=ISLAND_PULSE_MAX_RAD){
            island_pulse.direction=0;
            island_pulse.candidate=0;
            island_pulse.centered_frames=0;
            return 0;
        }
    }else{
        int direction=0;
        if(visible && fabsf(e)>=ISLAND_PULSE_TRIGGER_E)
            direction=e>0 ? -1 : 1;
        if(!direction || direction!=island_pulse.candidate){
            island_pulse.candidate=direction;
            return 0;
        }
        if(island_pulse.last_direction &&
           ((uint32_t)(app_elapsed-island_last_trigger_ms)<ISLAND_PULSE_RETRIGGER_MS ||
            !island_pulse.retrigger_angle_seen))return 0;
        island_last_trigger_ms=app_elapsed;
        island_pulse.direction=island_pulse.last_direction=direction;
        island_pulse.start_yaw=app_yaw;
        island_pulse.retrigger_angle_seen=0;
    }
    *w=island_pulse.direction*ISLAND_PULSE_W;
    return 1;
}
/* Three consecutive single-run frames arm element recognition for this SEARCH. */
static unsigned search_narrow_frames;
/* Evidence from the immediately preceding eligible control frame. */
static int previous_corner_side, previous_cross_hit;
static int search_continues_corner;
static int can_detect_element(void){
    return track_state==FOLLOW || (track_state==SEARCH && search_narrow_frames>=3u);
}
#if CORNER_PREDICT_EXIT
static int turn_samples;
#if CORNER_COAST_EXIT && TURN_ANGLE_METHOD != 1
static unsigned fit_count;
#endif
#endif
static void track_enter(int state){
#if CORNER_PREDICT_EXIT
    turn_samples=0;
#endif
    search_continues_corner=0;
    search_narrow_frames=0;
    previous_corner_side=previous_cross_hit=0;
    if(state==SEARCH)fork_frames=0;
    island_pulse_reset();
    track_state=state;
    state_ms=0;
    clear_frames=0;
    entry_yaw=app_yaw;
    entry_s=app_s;
    track_d_ready=0;     /* suppress the D term on the first frame back in FOLLOW */
    track_bend_ready=0;  /* suppress a fake dbend jump on the first frame back */
    island_phase=(state==BRANCH)?IS_ARMED:IS_NONE;
    island_out_pending=0;exit_single_ms=0;
    island_fall_frames=0;island_out_armed=0;island_out_frames=0;island_phase_start=0;
    island_fell=0;island_two_wait=0;island_two_frames=0;
    ordinary_guard_dir=ordinary_search_active=0;
    island_entry_gate=ENTRY_WAIT_SPLIT;
    island_two_ok=0;island_fall_ms=0;
    /* A corner pulse belongs to the FOLLOW tick that opened it. Clearing it
       here means a state switch (CROSS / BRANCH / SEARCH) can never carry a
       half-finished constant-w turn into another element. */
    corner_pulse_active=0;corner_pulse_side=0;
    corner_entry_yaw=app_yaw;corner_pulse_cool_ms=0;
    /* A state switch ends any junction crossing in progress: drop the main
       line latch so the next 2-phase decides again from continuity. */
    junction.phase=junction.frames=junction.clear_ms=junction.lost_ms=0;
}
static void track_reset(void){ steer_reset();
    corner_last_trigger_ms=app_elapsed-CORNER_CROSS_BLOCK_MS;
    island_last_trigger_ms=app_elapsed-ISLAND_PULSE_RETRIGGER_MS;
    track_enter(FOLLOW);
    edge_side=0; edge_age=100; fork_frames=0;
    track_weight_prev=0;
    track_bend_prev=0; track_bend_ready=0;
    fork_hold_tail=fork_release=0;straight_ms=0;
}

/* Course state advances ONLY on a confirmed explicit completion event.
   Queue at element exit, commit on the next control tick: the outgoing
   frame must finish selecting its line using the OLD segment's profile. */
static unsigned course_segment=1, course_pass, course_pending;
/* 0/1: corners counted, 2: zigzag speed cap active, 3: section finished. */
static unsigned s10_corner_count;
static unsigned s4_corner_count;
static float s4_second_corner_s;
static int s4_last_corner_side;
static int s10_last_corner_side;
static uint32_t s10_last_corner_ms;
static float s10_zigzag_start_s;
static unsigned course_armed=1, course_clear_ms, course_enter_ms;
static unsigned course_gap, course_gap_done, course_gap_started, course_gap_seen;
static unsigned course_straight_ms, course_final, course_done;
static unsigned course_wait_start, course_laps;
static unsigned course_eight, course_join_armed, course_join_clear, course_join_frames;
static unsigned course_eight_started;
static float course_start_s, course_last_event_s, course_gap_s, course_gap_yaw;
static float course_last_line_yaw, course_finish_s, course_eight_s, course_eight_yaw;
/* Heading to hold while crossing the S4 gap. COURSE_GAP_TURN_ENABLE 0 keeps
   the 11-08 behaviour (hold the entry heading = drive straight); 1 turns the
   car RIGHT by COURSE_GAP_TURN_RAD as soon as the line is lost, then holds
   that new heading until the line is reacquired. */
static float course_gap_target_yaw;
static const CourseSegment *course_spec(void){return &course_map[course_segment-1u];}
static float course_cruise(void){
    return COURSE_ENABLE && APP_MODE==6 ? fminf(TEST_SPEED_MPS,course_spec()->speed):TEST_SPEED_MPS;
}
static int course_side(void){
    if(!COURSE_ENABLE || APP_MODE!=6)return -1;
    return course_eight==EIGHT_SMALL ? EIGHT_SMALL_SIDE:course_spec()->island_side;
}
static int course_exit_side(void){return COURSE_ENABLE && APP_MODE==6?course_spec()->exit_side:1;}
static int course_accept(void){
    return !COURSE_ENABLE || (course_armed && !course_final);
}
static void course_begin(void){
    s10_corner_count=0;s10_last_corner_side=0;
    s4_corner_count=0;s4_last_corner_side=0;
#if SPECIAL_ISLAND_SINGLE_FORK
    special_heading_hold=0;
#endif
    course_segment=FORK_TEST_MODE ? FORK_TEST_SEGMENT : COURSE_START_SEGMENT;course_pass=course_pending=0;
    segment_fork_hits=0;dbg_fork=0;
#if FORK_TEST_MODE
    fork_test_falls=0;dbg_groups=0;
#endif
    course_armed=1;course_clear_ms=0;course_enter_ms=app_elapsed;
    course_start_s=app_s;course_last_event_s=app_s;
    course_gap=course_gap_done=course_gap_seen=course_straight_ms=0;
    course_final=course_done=0;course_eight=EIGHT_OFF;
    course_wait_start=(course_segment==1);
    course_laps=0;
    course_join_armed=course_join_clear=course_join_frames=0;
    if(!COURSE_ENABLE || APP_MODE!=6)return;
    if(course_segment<1 || course_segment>10 || COURSE_GAP_SPEED<=0 ||
       COURSE_GAP_MAX_M<=COURSE_GAP_MIN_M || course_map[S10].finish_after_last_bar_m<0.0f ||
       COURSE_GAP_TURN_RAD<0.0f || COURSE_GAP_TURN_RAD>1.0f ||
       COURSE_GAP_TURN_W<=0.0f ||
       COURSE_W_LIMIT_K<=0.0f || CORNER_PULSE_W<=0.0f ||
       JUNCTION_HEADING_KP<=0.0f ||
       EIGHT_CROSS_W_CAP<=0.0f || EIGHT_CROSS_W_CAP>STEER_W_LIMIT ||
       CORNER_PULSE_EXIT_RAD<=0.0f || CORNER_PULSE_MIN_RAD<0.0f ||
       CORNER_PULSE_MIN_RAD>=CORNER_PULSE_EXIT_RAD ||
       COURSE_PARK_SPEED_MPS<=0.0f || COURSE_PARK_SPEED_MPS>TEST_SPEED_MPS ||
       (COURSE_GAP_TURN_DIR!=1 && COURSE_GAP_TURN_DIR!=-1) ||
       ISLAND_OUT_TRIGGER!=1){
        app_fail(13);return;
    }
    for(unsigned i=0;i<10;i++)
        if(course_map[i].speed<=0 || course_map[i].speed>TEST_SPEED_MPS ||
           !course_map[i].passes || abs(course_map[i].island_side)!=1 ||
           (course_map[i].checkpoint!=CP_CROSS && course_map[i].checkpoint!=CP_ISLAND)){
            app_fail(13);return;
        }
}
static void course_enter_element(unsigned type){
    if(!COURSE_ENABLE || APP_MODE!=6)return;
    if(course_final || type!=course_spec()->checkpoint){app_fail(9);return;}
    if(type==CP_ISLAND && course_spec()->eight){
        course_eight=EIGHT_LARGE_IN;course_eight_s=app_s;course_eight_yaw=app_yaw;
        course_eight_started=app_elapsed;course_join_armed=0;
        course_join_clear=course_join_frames=0;
    }
}
static void course_complete(unsigned type){
    if(!COURSE_ENABLE || APP_MODE!=6)return;
    if(type!=course_spec()->checkpoint || !course_armed || course_pending){app_fail(9);return;}
    /* Gap must actually have been crossed, not silently skipped. */
    if(course_spec()->gap && !course_gap_done){app_fail(9);return;}
    if(course_spec()->eight && course_eight!=EIGHT_LARGE_OUT){app_fail(11);return;}
    course_pending=type;
}
static void course_tick(void){
    if(!COURSE_ENABLE)return;
    if(course_pending){
#if FORK_TEST_MODE
        course_pending=0;course_done=1;App_Stop();return;
#endif
        course_pending=0;
        course_last_event_s=app_s;course_armed=0;course_clear_ms=0;
        if(course_wait_start){
            course_wait_start=0;
            course_start_s=app_s;course_enter_ms=app_elapsed;
        }else if(++course_pass>=course_spec()->passes){
            if(course_segment==10 && ++course_laps>=COURSE_RACE_LAPS){
                course_final=1;course_finish_s=app_s;
            }else{
                if(course_segment==10){
                    course_segment=1;
                    /* Entered the zone at the stop line. Consume the exit
                       (start) line before counting any S1 landmarks. */
                    course_wait_start=1;
                }else course_segment++;
                segment_fork_hits=0;dbg_fork=0;
                s10_corner_count=0;s10_last_corner_side=0;
                s4_corner_count=0;s4_last_corner_side=0;
                course_pass=0;course_start_s=app_s;
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
    if(!COURSE_ENABLE)return 0;
    int after_corners=course_segment==4 && s4_corner_count==2 &&
        (track_state==FOLLOW || track_state==SEARCH);
    if(!course_gap && (can_detect_element() || after_corners)){
        if(total && groups==1 && fabsf(bend)<=COURSE_GAP_E_MAX &&
           fabsf(steer_gyro)<=COURSE_GAP_GYRO_MAX){
            course_straight_ms+=5;course_last_line_yaw=app_yaw;
        }else if(total)course_straight_ms=0;
        float s=app_s-course_start_s;
        int straight_entry=can_detect_element() && course_straight_ms>=COURSE_GAP_APPROACH_MS &&
            s>=course_spec()->gap_arm_min_m && s<=course_spec()->gap_arm_max_m;
        int clear_of_corner=s4_corner_count<2 ||
            app_s-s4_second_corner_s>=S4_GAP_AFTER_SECOND_CORNER_M;
        if(!total && course_spec()->gap && !course_gap_done && clear_of_corner &&
           (straight_entry || after_corners)){
            if(track_state==SEARCH)track_enter(FOLLOW);
            course_gap=1;course_gap_s=app_s;
            course_gap_yaw=straight_entry ? course_last_line_yaw : app_yaw;
            course_gap_started=app_elapsed;course_gap_seen=0;
            /* Yaw grows counter-clockwise (gyro z, GYRO_SIGN 1), so a RIGHT
               turn SUBTRACTS the angle. Flip COURSE_GAP_TURN_DIR to turn left. */
            course_gap_target_yaw=course_gap_yaw-
                (float)COURSE_GAP_TURN_DIR*COURSE_GAP_TURN_RAD*
                (float)(COURSE_GAP_TURN_ENABLE);
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
        track_enter(FOLLOW);edge_age=100;edge_side=0;
        app_error_prev=app_line_prev=bend;fork_frames=0;fork_hold_tail=fork_release=0;
        rate_i=0;rate_ready=0;
        return 0;
    }
    *v=fminf(course_cruise(),COURSE_GAP_SPEED);
    /* Heading hold towards course_gap_target_yaw. With TURN_ENABLE 1 the
       target is offset to the RIGHT, so the car immediately rotates by that
       angle and then runs straight along the new heading (the P loop drives
       w -> 0 once the angle is reached). With TURN_ENABLE 0 the target is
       the entry heading, i.e. exactly the 11-08 "hold heading" behaviour.
       A/B is bit-for-bit at TURN_ENABLE 0. */
    float gap_wcap=(float)(COURSE_GAP_TURN_ENABLE?
        COURSE_GAP_TURN_W:COURSE_GAP_W_MAX);
    *w=app_clip(TRACK_HEADING_KP*(course_gap_target_yaw-app_yaw),
                -gap_wcap,gap_wcap);
    track_d_ready=0;return 1;
}
static void course_observe(int groups,int total,int fork,int wide){
    if(!COURSE_ENABLE)return;
    if(!course_armed && !course_final){
        if(can_detect_element() && groups==1 && total>0 && !wide && !fork)
            course_clear_ms+=5;
        else course_clear_ms=0;
        if(course_clear_ms>=COURSE_REARM_MS && app_s-course_last_event_s>=COURSE_REARM_M)
            course_armed=1;
    }
}
/* Called only in the existing island LOOP; optical fork definition unchanged.
   A fresh fork after a released window + leg distance + yaw identifies the
   shared junction. Any guard failure stops instead of silently exiting early. */
static int eight_small_entry_ready(float distance){
    return app_yaw-course_eight_yaw>=EIGHT_FIRST_MIN_YAW &&
           (!EIGHT_FIRST_GATE || distance>=EIGHT_FIRST_MIN_M);
}
static void course_eight_update(int fork){
    if(!COURSE_ENABLE || !course_spec()->eight || track_state!=BRANCH || island_phase!=IS_LOOP)return;
    float d=app_s-course_eight_s, a=fabsf(app_yaw-course_eight_yaw);
    if(d>EIGHT_STAGE_MAX_M || app_elapsed-course_eight_started>EIGHT_STAGE_MAX_MS){app_fail(11);return;}
    if(EIGHT_CROSS_ENABLE || course_eight==EIGHT_LARGE_OUT)return;
    int ready=course_eight==EIGHT_LARGE_IN ?
        eight_small_entry_ready(d):
        (d>=EIGHT_SMALL_MIN_M && a>=EIGHT_SMALL_MIN_YAW);
    if(!fork){
        course_join_clear+=5;course_join_frames=0;
        if(course_join_clear>=EIGHT_JOIN_CLEAR_MS)course_join_armed=1;
    }else{
        course_join_clear=0;
        course_join_frames=(ready && course_join_armed)?course_join_frames+1:0;
    }
    if(course_join_frames>=(course_eight==EIGHT_LARGE_IN?FORK_CONFIRM_COUNT(ISLAND_SPECIAL_ENTRY_FORK_FRAMES):FORK_CONFIRM_COUNT(FORK_FRAMES))){
        course_eight++;
        course_eight_s=app_s;course_eight_yaw=app_yaw;course_eight_started=app_elapsed;
        course_join_frames=course_join_armed=course_join_clear=0;
        island_out_armed=0;island_out_frames=0;track_d_ready=0;rate_i=0;rate_ready=0;
        /* Pick the new side immediately in LOOP; keep optical feedback active.
           No timed blind rotation and no synthetic 'island complete'. */
        app_error_prev=app_line_prev=(float)course_side()*0.5f;
    }
}
static int course_eight_can_exit(void){
    return !COURSE_ENABLE || !course_spec()->eight ||
        (course_eight==EIGHT_LARGE_OUT && app_s-course_eight_s>=EIGHT_RETURN_MIN_M);
}
/* S8 uses phased line selection at entry and exit. */
static int course_s8_straight(void){
    return COURSE_ENABLE && APP_MODE==6 && CROSS_STRAIGHT_S8 && course_segment==8u;
}
/* A split follows only the selected run. Hold the most recent heading
   through the merged band; restart the derivative on a line switch. */
static void junction_steer(int pick,int smooth,const int *starts,const int *ends,float *v,float *w){
    *v=course_cruise();
    if(pick<0){
        if(junction.selected)junction.yaw=app_yaw;
        junction.selected=0;
        track_d_ready=0;
        *w=app_clip(JUNCTION_HEADING_KP*(junction.yaw-app_yaw),
                    -EIGHT_CROSS_W_CAP,EIGHT_CROSS_W_CAP);
    }else{
        float e=(starts[pick]+ends[pick])/11.0f-1.0f;
        if(smooth)e=app_error_prev+app_clip(e-app_error_prev,-FORK_E_SLEW,FORK_E_SLEW);
        int valid=track_d_ready && junction.selected==pick+1;
        float de=valid && !smooth?app_clip((e-app_error_prev)/DT,-LINE_D_MAX,LINE_D_MAX):0.0f;
        *w=steer_line(e,de,LINE_KP,valid,STEER_W_LIMIT);
        junction.selected=pick+1;
        junction.yaw=app_yaw;
        app_error_prev=app_line_prev=e;
        track_d_ready=1;
    }
    track_bend_ready=0;app_lost=0;
}
/* S8 and small-ring entry/exit: any single run counts, regardless of width. */
static int exit_single_confirmed(int groups){
    exit_single_ms=groups==1 ? exit_single_ms+5u : 0u;
    return exit_single_ms>=ISLAND_EXIT_SINGLE_MS;
}
/* Returns 1 when the junction owns this tick, including its release tick.
   Groups means contiguous lit runs, not the number of lit photodiodes.
   Zero/3+ groups cannot advance the sequence. They reset confirmation only. */
static int junction_control(int fresh_fork,int groups,int total,const int *starts,const int *ends,
                            float *v,float *w){
    int s8=course_s8_straight() && track_state!=BRANCH;
    int eight=COURSE_ENABLE && APP_MODE==6 && EIGHT_CROSS_ENABLE &&
        course_spec()->eight && track_state==BRANCH && island_phase==IS_LOOP &&
        (course_eight==EIGHT_LARGE_IN || course_eight==EIGHT_SMALL);
#if !SPECIAL_ISLAND_SINGLE_FORK
    int narrow=groups==1 && total>0 && total<=JUNCTION_CLEAR_MAX_LIT;
#else
    (void)groups;(void)total;
#endif
    int ready=s8 && (track_state==FOLLOW || track_state==SEARCH) && course_accept() && !course_pending;
    if(eight){
        float d=app_s-course_eight_s,a=fabsf(app_yaw-course_eight_yaw);
        if(d>EIGHT_STAGE_MAX_M || app_elapsed-course_eight_started>EIGHT_STAGE_MAX_MS){
            app_fail(11);*v=*w=0;return 1;
        }
        ready=course_eight==EIGHT_LARGE_IN ?
            eight_small_entry_ready(d):
            (d>=EIGHT_SMALL_MIN_M && a>=EIGHT_SMALL_MIN_YAW);
    }
#if SPECIAL_ISLAND_SINGLE_FORK
    int s8_loop=course_s8_straight() && track_state==BRANCH && island_phase==IS_LOOP;
    if(special_heading_hold && app_elapsed-junction.started>=SPECIAL_ISLAND_HOLD_MS){
        special_heading_hold=0;
        if(course_s8_straight() && track_state==BRANCH && island_phase==IS_TURN_OUT){
            course_complete(CP_ISLAND);
            if(app_running)track_enter(FOLLOW);
            *v=course_cruise();*w=0;
            return 1;
        }
    }
    if(s8_loop)ready=island_entry_gate==ENTRY_CLEAR &&
        (S8_EXIT_MIN_YAW_RAD<=0 || s8_entry_split_yaw-app_yaw>=S8_EXIT_MIN_YAW_RAD);
    if(!special_heading_hold && ready && fresh_fork){
        if(s8){
            s8_entry_split_yaw=app_yaw;
            course_enter_element(CP_ISLAND);
            if(!app_running){*v=*w=0;return 1;}
            track_enter(BRANCH);
            island_phase=IS_LOOP;
            island_entry_gate=ENTRY_CLEAR;
        }else if(s8_loop){
            island_phase=IS_TURN_OUT;
            island_phase_start=state_ms;
        }else{
            course_eight++;
            course_eight_s=app_s;course_eight_yaw=app_yaw;course_eight_started=app_elapsed;
            course_join_frames=course_join_armed=course_join_clear=0;
            island_out_armed=island_out_frames=island_out_pending=0;
        }
        corner_pulse_active=0;fork_frames=fork_hold_tail=fork_release=0;
        rate_i=0;rate_ready=0;
        junction.started=app_elapsed;junction.yaw=app_yaw;junction.selected=0;
        special_heading_hold=1;
    }
    if(special_heading_hold){
        small_ring_junction=course_spec()->eight;
        junction_speed_hold=1;
        junction_steer(-1,0,starts,ends,v,w);
        return 1;
    }
    return 0;
#else
    (void)fresh_fork;
    if(!junction.phase){
        if(!s8 && !eight){junction.frames=junction.clear_ms=0;return 0;}
        if(narrow){
            junction.frames=0;
            if(junction.clear_ms<JUNCTION_CLEAR_MS)junction.clear_ms+=5;
            return 0;
        }
        if(!ready || groups!=2 || junction.clear_ms<JUNCTION_CLEAR_MS){
            junction.frames=junction.clear_ms=0;return 0;
        }
        int entering=s8 || course_eight==EIGHT_LARGE_IN;
        small_ring_junction=eight;
        junction_speed_hold=1;
        if(entering && ++junction.frames<FORK_CONFIRM_COUNT(ISLAND_SPECIAL_ENTRY_FORK_FRAMES))return 0;
        if(track_state==SEARCH)track_enter(FOLLOW);
        /* Entry split already confirmed; next wait for the merged line. */
        if(s8)s8_entry_split_yaw=app_yaw;
        junction.phase=entering?2:1;junction.frames=0;junction.clear_ms=0;
        junction.started=app_elapsed;junction.lost_ms=0;exit_single_ms=0;
        junction.yaw=app_yaw;junction.selected=0;
        corner_pulse_active=0;fork_frames=0;fork_hold_tail=fork_release=0;
        edge_side=0;edge_age=100;rate_i=0;rate_ready=0;
    }
    if(app_elapsed-junction.started>=JUNCTION_MAX_MS){
        app_fail(11);*v=*w=0;return 1;
    }
    junction.lost_ms=total?0:junction.lost_ms+5;
    if(junction.lost_ms>=JUNCTION_LOST_MS){app_fail(5);*v=*w=0;return 1;}
    int exiting=eight && course_eight==EIGHT_SMALL;
    if(junction.phase==2 && groups==1)junction.phase=4;
    if(junction.phase==1){
        junction.frames=groups==2?junction.frames+1:0;
        if(junction.frames>=FORK_CONFIRM_COUNT(JUNCTION_PHASE_FRAMES)){
            junction.phase++;
            junction.frames=0;
        }
    }
    int trailing=junction.phase==4;
    small_ring_junction=eight;
    junction_speed_hold=1; /* Hold through merged and missing-line frames. */
    int pick=-1;
    if(groups==2)pick=exiting ? !trailing : trailing;
    else if(groups==1)pick=0;
    junction_steer(pick,groups==1,starts,ends,v,w);
    if(junction.phase==4){
        if(exit_single_confirmed(groups)){
            if(exiting)track_d_ready=0;
            else app_error_prev=app_line_prev=(starts[0]+ends[0])/11.0f-1.0f;
            junction.phase=junction.frames=junction.clear_ms=0;
            if(s8){
                course_enter_element(CP_ISLAND);
                if(!app_running)return 1;
                track_enter(BRANCH);
                island_phase=IS_LOOP;
                island_entry_gate=ENTRY_CLEAR; /* A sustained single run cleared the entry junction. */
            }else{
                course_eight++;
                course_eight_s=app_s;course_eight_yaw=app_yaw;course_eight_started=app_elapsed;
                course_join_frames=course_join_armed=course_join_clear=0;
                island_out_armed=0;island_out_frames=0;
            }
            rate_i=0;rate_ready=0;
        }
    }
    return 1;
#endif
}
static int ordinary_fork_speed_limit(void){
    return course_segment!=8u && course_eight!=EIGHT_SMALL && !small_ring_junction;
}
static int fixed_pulse_output;
static void course_limit(float *v,float *w){
    if(!COURSE_ENABLE)return;
    if(*v>0)*v=fminf(*v,course_cruise());
    /* Preserve the direction of line reacquisition but drive an arc. The old
       SEARCH had v=0 and spun in place; all course-mode turns stay forward. */
    if(track_state==SEARCH){*v=course_cruise();}
    if(course_segment==10 && s10_corner_count==2){
        if(app_s-s10_zigzag_start_s>=S10_ZIGZAG_MIN_M &&
           app_elapsed-s10_last_corner_ms>=S10_ZIGZAG_CLEAR_MS)s10_corner_count=3;
        else *v=fminf(*v,S10_ZIGZAG_MAX_MPS);
    }
    int entry_fork=(track_state==FOLLOW || track_state==SEARCH) &&
        !course_s8_straight() && course_accept() && fork_frames;
    int island_turn=track_state==BRANCH &&
        (island_phase==IS_ARMED || island_phase==IS_TURN_IN ||
         island_phase==IS_TURN_OUT ||
         (island_phase==IS_LOOP && (island_entry_gate==ENTRY_WAIT_SINGLE ||
          island_out_frames || island_out_armed || island_out_pending || course_join_frames)));
    /* Apply on the first recognized split, then honor the junction's hold. */
    if(ordinary_fork_speed_limit() && ISLAND_FORK_MAX_MPS>0 && (dbg_fork || entry_fork || island_turn || junction_speed_hold))
        *v=fminf(*v,ISLAND_FORK_MAX_MPS);
    if(CORNER_SLOW_MODE && (corner_pulse_active || track_state==SEARCH))
        *v=fminf(*v,CORNER_MAX_MPS);
    if(course_final){
        if(app_s-course_finish_s>=course_spec()->finish_after_last_bar_m){
            course_done=1;App_Stop();*v=*w=0;return;
        }
        *v=fminf(*v,COURSE_PARK_SPEED_MPS);
    }
    if(*v>0 && !fixed_pulse_output)*w=app_clip(*w,-COURSE_W_LIMIT_K*(*v)/EFFECTIVE_TRACK_M,
                            COURSE_W_LIMIT_K*(*v)/EFFECTIVE_TRACK_M);

}

/* Shared centre reacquisition: at least CORNER_PULSE_CENTER_MIN lit
   sensors inside 1-based 5..8 (zero-based channels 4..7). */
static int corner_center_seen(const int *starts,const int *ends,int groups){
    int count=0;
    for(int g=0;g<groups;g++)
        for(int i=starts[g];i<=ends[g];i++)
            if(i>=CENTER_FIRST_INDEX && i<=CENTER_LAST_INDEX)count++;
    return count>=CORNER_PULSE_CENTER_MIN;
}
static int scope_search_arc;
static void search_arc(int direction,float *v,float *w){
    scope_search_arc=1;
    *v=course_cruise();
    *w=direction*CORNER_PULSE_W;
    fixed_pulse_output=1;
}
#if CORNER_PREDICT_EXIT
/* Store consecutive single-line samples in control-frame order. */
static int turn_center_approaching(int groups, int start, int end, int direction){
    static signed char first[TURN_PREDICT_SAMPLE_FRAMES], last[TURN_PREDICT_SAMPLE_FRAMES];
    static int previous_direction;
    static uint32_t previous_ms;
    if(app_elapsed-previous_ms!=5u || direction!=previous_direction)
        turn_samples=0;
    previous_ms=app_elapsed;
    previous_direction=direction;
    if(groups!=1 || end-start+1<CORNER_PULSE_CENTER_MIN || steer_gyro*direction<=0){
        turn_samples=0;
        return 0;
    }
    if(turn_samples==TURN_PREDICT_SAMPLE_FRAMES){
        for(int i=1;i<turn_samples;i++){
            first[i-1]=first[i];
            last[i-1]=last[i];
        }
        turn_samples--;
    }
    first[turn_samples]=start;
    last[turn_samples++]=end;
    if(turn_samples<TURN_PREDICT_SAMPLE_FRAMES)return 0;
    int left=start+end<11;
    int gap=left ? PREDICT_FIRST_INDEX+CORNER_PULSE_CENTER_MIN-1-end :
                   start-(PREDICT_LAST_INDEX-CORNER_PULSE_CENTER_MIN+1);
    for(int i=1;i<turn_samples;i++){
        if(left ? (first[i]<first[i-1] || last[i]<last[i-1]) :
                  (first[i]>first[i-1] || last[i]>last[i-1]))return 0;
    }
    int shift=left ? end-last[0] : first[0]-start;
    int other=left ? start-first[0] : last[0]-end;
    return shift>0 && other>0 && gap*(turn_samples-1)<=shift*TURN_PREDICT_LEAD_FRAMES;
}
#endif
static float corner_remaining_rad, corner_coast_rad;
static unsigned corner_release_reason;
#if CORNER_PREDICT_EXIT && CORNER_COAST_EXIT
/* Infer local radians per sensor pitch from the same optical/yaw interval. */
static int flow_coast_ready(int start,int end,int direction){
    static float pos[TURN_PREDICT_SAMPLE_FRAMES], yaw[TURN_PREDICT_SAMPLE_FRAMES];
    static uint32_t last_ms;
    static int last_direction;
    float x=(start+end)*0.5f-5.5f;
    float rate=direction*steer_gyro;
    if(app_elapsed-last_ms!=5u || direction!=last_direction || rate<=0 || end-start+1<CORNER_PULSE_CENTER_MIN)
        turn_samples=0;
    last_direction=direction;
    last_ms=app_elapsed;
    if(rate<=0 || end-start+1<CORNER_PULSE_CENTER_MIN)return 0;
    if(turn_samples && (pos[turn_samples-1]*x<0 || fabsf(x)>fabsf(pos[turn_samples-1])))
        turn_samples=0;
    if(turn_samples==TURN_PREDICT_SAMPLE_FRAMES){
        for(int i=1;i<turn_samples;i++){pos[i-1]=pos[i];yaw[i-1]=yaw[i];}
        turn_samples--;
    }
    pos[turn_samples]=x;yaw[turn_samples++]=app_yaw;
    if(turn_samples<TURN_PREDICT_SAMPLE_FRAMES)return 0;
    float shift=fabsf(pos[0])-fabsf(x);
    float angle=direction*(app_yaw-yaw[0]);
    if(shift<CORNER_FLOW_MIN_SHIFT || shift<=0 || angle<=0)return 0;
    corner_remaining_rad=fabsf(x)*angle/shift;
    corner_coast_rad=rate*(0.001f*CORNER_COAST_TIME_MS);
    return corner_remaining_rad<=corner_coast_rad;
}
#if TURN_ANGLE_METHOD != 1
/* Keep observations in the current axle frame: forward x, left y, positive yaw left. */
static int fitted_heading(float sensor_pos,int direction,float *remaining){
    static float x[TURN_FIT_SAMPLES],y[TURN_FIT_SAMPLES],prev_yaw,prev_s;
    float da=app_yaw-prev_yaw,ds=app_s-prev_s;
    prev_yaw=app_yaw;prev_s=app_s;
    float c=cosf(da),sn=sinf(da);
    float dx=ds*cosf(0.5f*da),dy=ds*sinf(0.5f*da);
    unsigned skip=fit_count==TURN_FIT_SAMPLES;
    for(unsigned j=skip;j<fit_count;j++){
        float a=x[j]-dx,b=y[j]-dy;
        x[j-skip]=c*a+sn*b;y[j-skip]=-sn*a+c*b;
    }
    fit_count-=skip;
    x[fit_count]=TURN_SENSOR_FORWARD_M;
    y[fit_count++]=-sensor_pos*TURN_SENSOR_PITCH_M;
    if(fit_count<TURN_FIT_SAMPLES)return 0;
    float mx=0,my=0,xx=0,yy=0,xy=0;
    for(unsigned j=0;j<fit_count;j++){mx+=x[j];my+=y[j];}
    mx/=fit_count;my/=fit_count;
    for(unsigned j=0;j<fit_count;j++){
        float a=x[j]-mx,b=y[j]-my;
        xx+=a*a;yy+=b*b;xy+=a*b;
    }
    float disc=hypotf(xx-yy,2*xy);
    float minor=0.5f*(xx+yy-disc);
    float span=hypotf(x[0]-x[fit_count-1],y[0]-y[fit_count-1]);
    if(span<TURN_FIT_MIN_SPAN_M || disc<=0 || minor>0.10f*disc ||
       minor>fit_count*TURN_FIT_MAX_RMS_M*TURN_FIT_MAX_RMS_M)return 0;
    *remaining=direction*0.5f*atan2f(2*xy,xx-yy);
    return *remaining>=0;
}
#endif
static int corner_coast_ready(int start,int end,int direction){
#if TURN_ANGLE_METHOD == 1
    return flow_coast_ready(start,end,direction);
#else
    static unsigned last_ms;
    static int last_dir;
    if(!turn_samples || app_elapsed-last_ms!=5u || direction!=last_dir)fit_count=0;
    last_ms=app_elapsed;last_dir=direction;
    int flow=flow_coast_ready(start,end,direction);
    if(direction*steer_gyro<=0 || end-start+1<CORNER_PULSE_CENTER_MIN){fit_count=0;return 0;}
    float angle;
    if(fitted_heading((start+end)*0.5f-5.5f,direction,&angle)){
        corner_remaining_rad=fmaxf(0,angle);
        corner_coast_rad=direction*steer_gyro*(0.001f*CORNER_COAST_TIME_MS);
        return corner_remaining_rad<=corner_coast_rad;
    }
    if(TURN_ANGLE_METHOD==3)return flow;
    corner_remaining_rad=-1;corner_coast_rad=0;return 0;
#endif
}

#endif
static void track_control(float *v,float *w){
    scope_search_arc=0;
    corner_remaining_rad=-1;corner_coast_rad=0;corner_release_reason=0;
    fixed_pulse_output=0;
    int prior_pulse_side=(track_state==FOLLOW && corner_pulse_active) ? corner_pulse_side : 0;
    junction_speed_hold=0;
    small_ring_junction=0;
    int prior_corner=previous_corner_side, prior_cross=previous_cross_hit;
    /* Gap/junction ownership and all other early returns break confirmation. */
    previous_corner_side=previous_cross_hit=0;
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
    float dweight=weight-track_weight_prev;
    float prev_abs=fabsf(track_weight_prev);
    track_weight_prev=weight; /* update before any early return */
    /* weight/total == segment centre e for one run; dbend is its per-frame
       change, free of line-width changes. Kept before every early return so
       a state switch cannot fake a jump on the frame we come back. */
    float bend=total?weight/(float)total:0.0f;
    float dbend=(BEND_ENABLE && track_bend_ready)?
        app_clip(bend-track_bend_prev,-BEND_DBEND_MAX,BEND_DBEND_MAX):0.0f;
    track_bend_prev=bend;
    track_bend_ready=1;
    float severity=BEND_ENABLE?(fabsf(bend)+BEND_RATE_K*fabsf(dbend)):0.0f;
    int wide=(groups==1 && total>=12);
    if(track_state==SEARCH && search_narrow_frames<3u){
        int narrow=(groups==1 && total>=3 && total<=5);
        search_narrow_frames=narrow ? search_narrow_frames+1u : 0u;
    }
    if(course_gap_control(total,groups,bend,v,w)){
        dbg_fork=0;
        return;
    }
    /* Count fork runs only where the course permits island detection. */
    int fork=0, island_pair=0, one_lit_probe=0;
    int allow_fork=!COURSE_ENABLE || course_spec()->checkpoint==CP_ISLAND;
    if(COURSE_ENABLE && course_segment==9u && S9_FORK_BLOCK_M>0.0f &&
       app_s-course_start_s<S9_FORK_BLOCK_M)allow_fork=0;
    if(allow_fork && groups>=2){
        int last=groups-1;
        int w0=ends[0]-starts[0]+1;
        int wN=ends[last]-starts[last]+1;
        int gap=starts[last]-ends[0]-1;
        int near_left=(starts[0]<=FORK_EDGE_MARGIN);
        int near_right=(ends[last]>=11-FORK_EDGE_MARGIN);
        int shape=(gap>=FORK_GAP_MIN && w0>=FORK_SEG_MIN && wN>=FORK_SEG_MIN);
        /* Structural sharp-corner test: 5+ lit sensors on ONE HALF of the
           bar (ch0-5 or ch6-11). An island entry is two narrow runs, so a
           wide CENTRAL straight segment (grey band, up to 9 lit) never
           reaches 5 on one half and is deliberately NOT vetoed. */
        int corner_lit=0;
        if(FORK_CORNER_LIT_MIN>0){
            if(left_hits>=FORK_CORNER_LIT_MIN ||
               right_hits>=FORK_CORNER_LIT_MIN)corner_lit=1;
        }
        if(FORK_MODE==0)
            fork=(starts[0]==0 && ends[0]>=1 &&
                  ends[last]==11 && starts[last]<=10);
        else if(FORK_MODE==1)
            fork=shape && (near_left || near_right);
        else{
            int qualifying=0;
            for(int g=0;g<groups && qualifying<2;g++)
                if(ends[g]-starts[g]+1>=FORK_SEG_MIN)qualifying++;
            fork=qualifying>=2;
        }
        if(FORK_MODE!=2 && corner_lit)fork=0;
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
        one_lit_probe=(FORK_HOLD_ONE_LIT && fork && !corner_wide &&
            ((near_left && w0==1) || (near_right && wN==1)));
    }
    int ordinary_island=track_state==BRANCH && course_segment!=8u &&
        (!course_spec()->eight || course_eight==EIGHT_LARGE_IN || course_eight==EIGHT_LARGE_OUT);
    int ordinary_loop=ordinary_island && island_phase==IS_LOOP;
    if(!ordinary_loop)ordinary_guard_dir=ordinary_search_active=0;
    if(!total && ordinary_loop && !ordinary_search_active && !corner_pulse_active && !island_out_pending){
        if(!ordinary_guard_dir){
            ordinary_guard_dir=BRANCH_TURN_DIR*ISLAND_SIDE;
            ordinary_guard_yaw=app_yaw;
        }
        ordinary_search_active=1;
    }
    if(total && ordinary_search_active){
        ordinary_search_active=0;
        ordinary_guard_dir=0; /* The in-loop lost-line search ends on reacquisition. */
    }
    if(ordinary_guard_dir && (app_yaw-ordinary_guard_yaw)*ordinary_guard_dir>=ORDINARY_ELEMENT_UNLOCK_RAD)
        ordinary_guard_dir=0;
    if(fork){
        ordinary_guard_dir=0;
        if(track_state==SEARCH || track_state==CROSS)track_enter(FOLLOW);
        if(corner_pulse_active){
            corner_pulse_active=0;
            track_d_ready=0;
        }
        island_pulse_reset();
    }
    int new_double=groups==2 && dbg_groups!=2;
    int fresh_fork=fork && !dbg_fork;
    if(fresh_fork)segment_fork_hits++;
#if FORK_TEST_MODE
    if(dbg_groups>=2 && groups==1)fork_test_falls++;
#endif
    course_observe(groups,total,fork,wide);
    dbg_groups=groups;dbg_total=total;dbg_fork=fork;
    /* Current-project element recognition observer (no drive, pure CSV). */
    elem_test_observe(track_state==BRANCH,track_state==CROSS,
        island_phase==IS_LEAVE,wide,fork,groups,total,5u);
    if(!ordinary_guard_dir && junction_control(fresh_fork,groups,total,starts,ends,v,w)){
        island_pulse_reset();
        if(track_state!=FOLLOW)state_ms+=5; /* island clocks keep real elapsed time */
        dbg_corner=0;return;
    }
    dbg_island_pair=island_pair;
    dbg_one_lit=one_lit_probe;
    int central=(groups==1 && total<=CENTRAL_TOTAL_MAX &&
                 fabsf(weight)<CENTRAL_WEIGHT_MAX);
    /* Entry test (2026-11-17): a CONSECUTIVE run of CORNER_ENTER_RUN lit
       sensors measured FROM ONE EDGE - ch0 upwards (ch0..ch6 at 7) or the
       mirror ch11 downwards (ch5..ch11 at 7). A centred band cannot reach
       an edge and a split two-segment pattern has a gap, so neither starts
       a pulse. severity >= BEND_SEARCH_MIN (0.90) stays as the one-sided
       backstop. An island lights its fork pattern for many frames, so the
       whole test is disabled in BRANCH.
       corner_raw is the per-frame pattern; the pulse state machine below
       turns it into the effective `corner`. */
    int edge_run=0;
    for(int g=0;g<groups;g++){
        if(starts[g]==0 && ends[g]>=CORNER_ENTER_RUN-1)edge_run=1;
        if(ends[g]==11 && starts[g]<=12-CORNER_ENTER_RUN)edge_run=1;
    }
    /* S9's outer ring shares ordinary island entry and final exit with S2. */

    if(track_state==BRANCH && (island_phase==IS_TURN_IN || island_phase==IS_LOOP)){
        if(island_entry_gate==ENTRY_WAIT_SPLIT && groups>=2)
            island_entry_gate=ENTRY_WAIT_SINGLE;
        else if(island_entry_gate==ENTRY_WAIT_SINGLE && groups==1)
            island_entry_gate=ENTRY_CLEAR;
    }
    int leave_corner=ordinary_island && island_phase==IS_LEAVE;
    /* An ordinary island's first single run after an armed exit belongs to the exit,
       even when its edge pattern also qualifies as a corner. */
    int loop_corner=ISLAND_LOOP_USE_CORNER && track_state==BRANCH && island_phase==IS_LOOP &&
        (!ordinary_island || island_entry_gate==ENTRY_CLEAR) && !island_out_pending &&
        !(ordinary_island && island_out_armed);
    int branch_corner=leave_corner || loop_corner;
    int loop_fork=loop_corner && (fork || junction_speed_hold);
    if(loop_fork){
        if(corner_pulse_active)junction_speed_hold=1;
        corner_pulse_active=0;
    }
    int corner_raw=!wide && !fork && !loop_fork && (track_state!=BRANCH || branch_corner) && (edge_run ||
        (BEND_ENABLE ?
            (severity>=BEND_SEARCH_MIN) :
            (left_hits>=CORNER_HALF_HITS || right_hits>=CORNER_HALF_HITS ||
             fabsf(weight)>=CORNER_WEIGHT_MIN ||
             fabsf(dweight)>=CORNER_DWEIGHT_MIN)));
    /* Publish before every early return below, so all control paths in
       this tick report the same fresh pattern snapshot to app_log(). */
    dbg_groups=groups;dbg_total=total;dbg_fork=fork;dbg_corner=corner_raw;
    if(track_state==FOLLOW && groups==1 && total>0 &&
       total<=CENTRAL_TOTAL_MAX && fabsf(bend)<BEND_STRAIGHT_E){
        if(straight_ms<BEND_STRAIGHT_MS+5u)straight_ms+=5;
    }else straight_ms=0;
    int corner_side=(fabsf(weight)>0.01f)?(weight>0.0f?1:-1):
                    (right_hits>=left_hits?1:-1);
    /* Optical release takes priority over retriggering an active pulse. */
    if(edge_age<100)edge_age++;
    if(SIDE_ENABLE && track_state==FOLLOW && groups>0 && !fork && !wide){
        int left=starts[0]==0 && ends[0]>=SEARCH_EDGE_RUN-1;
        int right=ends[groups-1]==11 && starts[groups-1]<=12-SEARCH_EDGE_RUN;
        if(left!=right){
            edge_side=left ? -1 : 1;
            edge_age=0;
        }
    }
    int corner=0;
    int turning=corner_pulse_active || track_state==SEARCH;
    int optical_release=corner_pulse_active && corner_center_seen(starts,ends,groups);
    if(!search_continues_corner && !optical_release && corner_raw && !corner_pulse_cool_ms && (can_detect_element() || branch_corner))
        previous_corner_side=corner_side;
    int corner_confirmed=previous_corner_side &&
        (CORNER_CONFIRM_FRAMES==1 || previous_corner_side==prior_corner);
    if(track_state==SEARCH && corner_confirmed)
        track_enter(FOLLOW);
    if(track_state!=FOLLOW && !branch_corner){
        corner_pulse_active=0;
    }else if(corner_confirmed){
        if(course_segment==4 && !course_gap_done && s4_corner_count<2 &&
           corner_side!=s4_last_corner_side){
            s4_corner_count++;
            if(s4_corner_count==2)s4_second_corner_s=app_s;
            s4_last_corner_side=corner_side;
        }
        if(COURSE_ENABLE && course_segment==10 && s10_corner_count<3){
            if(s10_last_corner_side!=corner_side && s10_corner_count<2){
                s10_corner_count++;
                if(s10_corner_count==2)s10_zigzag_start_s=app_s;
            }
            s10_last_corner_side=corner_side;
            s10_last_corner_ms=app_elapsed;
        }
        previous_corner_side=0;
        corner_pulse_active=1;
#if CORNER_PREDICT_EXIT
        turn_samples=0;
#endif
        /* The ordinary-island leave pattern calls for the opposite turn. */
        corner_pulse_side=leave_corner ? -corner_side : corner_side;
        if(ordinary_loop && !ordinary_guard_dir){
            ordinary_guard_dir=-corner_pulse_side;
            ordinary_guard_yaw=app_yaw;
            island_out_armed=island_out_frames=0;
        }
        corner_entry_yaw=app_yaw;
        corner_pulse_cool_ms=CORNER_PULSE_COOL_MS;
        corner_last_trigger_ms=app_elapsed;
        /* A new action must not inherit the old SEARCH direction or PD
           derivative/integrator history. */
        edge_side=corner_side;edge_age=0;
        track_d_ready=0;rate_i=0;rate_ready=0;
        corner=1;
    }else if(corner_pulse_active){
        float turned=fabsf(app_yaw-corner_entry_yaw);
        /* A visible centre line may release even a very short pulse. */
        int release=optical_release;
#if CORNER_PREDICT_EXIT
        if(track_state!=SEARCH){
            if(groups==1 && !edge_run && !wide){
#if CORNER_COAST_EXIT
                int predicted=corner_coast_ready(starts[0],ends[0],-corner_pulse_side);
#else
                int predicted=turn_center_approaching(groups,starts[0],ends[0],-corner_pulse_side);
#endif
                if(predicted && turned>=CORNER_PULSE_MIN_RAD){release=1;corner_release_reason=1;}
            }else turn_samples=0;
        }
#endif
        if(turned>=CORNER_PULSE_EXIT_RAD || release){
            if(ordinary_loop)ordinary_guard_dir=0;
            if(optical_release)corner_release_reason=2;
            else if(!release)corner_release_reason=3;
            corner_pulse_active=0;
        }else corner=1;
    }
    dbg_corner=corner;

    /* SEARCH can recognize a junction before narrow-line reacquisition. */
    if(track_state==FOLLOW || track_state==SEARCH)
        fork_frames=fork ? fork_frames+1 : 0;
    if(can_detect_element() || (track_state==SEARCH && (fork || wide))){
        if(corner)edge_age=0;
        /* Sticky window keyed on the narrow island probe, NOT on raw fork or
           groups: both flicker as the probe enters the bar, and a per-frame
           hold toggling with them is what shook the head. */
        if(FORK_ENTRY_HOLD){
            if(island_pair){fork_hold_tail=FORK_HOLD_TAIL;fork_release=FORK_HOLD_RELEASE;}
            else if(fork_hold_tail>0)fork_hold_tail--;
        }
        previous_cross_hit=wide && course_accept() && !course_s8_straight() &&
            (turning || prior_cross || (uint32_t)(app_elapsed-corner_last_trigger_ms)>=CORNER_CROSS_BLOCK_MS);
        if(previous_cross_hit && prior_cross){
            course_enter_element(CP_CROSS);if(!app_running)return;
            track_enter(CROSS);
        }
        else if(fork_frames>=((COURSE_ENABLE && course_segment==8u)?FORK_CONFIRM_COUNT(ISLAND_SPECIAL_ENTRY_FORK_FRAMES):FORK_CONFIRM_COUNT(FORK_FRAMES)) && course_accept() && !course_s8_straight()){
            fork_frames=0;
            fork_hold_tail=0;fork_release=0;
            if(!ISLAND_SIDE){app_fail(5);return;}
            course_enter_element(CP_ISLAND);if(!app_running)return;
            track_enter(BRANCH);
#if !CORNER_NO_SEARCH
#if COURSE_LOSS_SEARCH_ENABLE
        /* Any lost line outside the S4 gap must keep looking. The old
           condition demanded a fresh latched edge (edge_age<=24 && edge_side),
           which a mid-bar straight or a right-angle overshoot often lacks,
           so the car stopped with F5 instead of searching. */
        }else if(track_state==FOLLOW && !total){
            if(!edge_side)edge_side=COURSE_LOSS_SEARCH_DIR;
            edge_age=0;
            track_enter(SEARCH);
        }
#else
        }else if(track_state==FOLLOW && !total && edge_age<=24 && edge_side) track_enter(SEARCH);
#endif
#else
        }   /* CORNER_NO_SEARCH: no recovery spin; a lost line stops the car */
#endif
    }
    if(prior_pulse_side && track_state==SEARCH){
        if(course_segment==4 && s4_corner_count)s4_last_corner_side=prior_pulse_side;
        if(course_segment==10 && s10_corner_count)s10_last_corner_side=prior_pulse_side;
        search_continues_corner=1;
    }
    if(!corner_raw && !search_continues_corner){
        s10_last_corner_side=0;
        s4_last_corner_side=0;
    }
    if(track_state!=FOLLOW)state_ms+=5;
    /* Island sub-state machine: a 2->1 fall-back happens at BOTH the entrance
       and the exit junction, and two-segment patterns reappear during the
       final turn. A pattern-only exit cannot tell those moments apart. */
    if(track_state==BRANCH){
        unsigned ph_ms=state_ms-island_phase_start;
        if(island_phase==IS_ARMED){
            int trigger=0,hard=0;
            if(ordinary_island){
                if(!island_fell){
                    island_fall_frames=groups==1 ? island_fall_frames+1 : 0;
                    island_fell=island_fall_frames>=2;
                }
                trigger=island_fell && groups==2;
            }
            else if(BRANCH_TURN_TRIGGER==0)trigger=1;
            else if(BRANCH_TURN_TRIGGER==1){
                if(groups==1)island_fall_frames++;
                else island_fall_frames=0;
                int fall_now=(island_fall_frames>=BRANCH_FALL_FRAMES);
                /* Latch the fall-back: while crossing the 0..9 junction
                   fill the group count flickers 1<->2, and the two-segment
                   wait must not restart on every flicker. With the switch
                   off this is bit-for-bit the old per-frame test. */
                if(BRANCH_ARM_WAIT_TWO && fall_now)island_fell=1;
                trigger=BRANCH_ARM_WAIT_TWO?(int)island_fell:fall_now;
            }else if(BRANCH_TURN_TRIGGER==2)
                trigger=(app_s-entry_s>=BRANCH_TURN_DIST);
            if(!ordinary_island && (int)state_ms>=(int)BRANCH_WAIT_MAX_MS){
                trigger=1;
                /* WAIT_MAX is the "no fall-back at all" deadline. Once the
                   fall-back has been seen, BRANCH_TWO_MAX_MS owns the split
                   wait; do not cut that wait short here. */
                hard=!(BRANCH_ARM_WAIT_TWO && BRANCH_TURN_TRIGGER==1 &&
                       island_fell);
            }
            /* New decision: a single wide band is NOT the island branch.
               Stay in IS_ARMED (e slew + dterm=0, so a flickering band
               cannot kick w) until two segments reappear, then start the
               entrance turn with a forced LEFT pick. Two timeouts keep the
               old blind turn available: BRANCH_TWO_MAX_MS is measured from
               the fall-back, BRANCH_WAIT_MAX_MS from track_enter(BRANCH). */
            if(!ordinary_island && BRANCH_ARM_WAIT_TWO && BRANCH_TURN_TRIGGER==1 &&
               trigger && !hard){
                if(groups>=2){
                    if(island_two_frames<FORK_CONFIRM_COUNT(BRANCH_TWO_FRAMES))
                        island_two_frames++;
                }else{
                    island_two_frames=0;
                    if(!island_two_wait){
                        island_two_wait=1;island_fall_ms=state_ms;
                    }
                }
                int split_ok=(island_two_frames>=FORK_CONFIRM_COUNT(BRANCH_TWO_FRAMES));
                int split_timeout=(island_two_wait &&
                    (int)(state_ms-island_fall_ms)>=(int)BRANCH_TWO_MAX_MS);
                if(split_ok){
                    island_two_ok=1;
                    island_entry_gate=ENTRY_WAIT_SINGLE;
                }
                trigger=(split_ok||split_timeout);
            }
            if(trigger){
                if(ordinary_island)island_entry_gate=ENTRY_CLEAR;
                island_phase=IS_TURN_IN;island_phase_start=state_ms;
                island_two_wait=0;island_two_frames=0;
            }
        }else if(island_phase==IS_TURN_IN){
            unsigned turn_ms=ordinary_island ? ORDINARY_ENTRY_PULSE_MS : BRANCH_TURN_MS;
            if(ph_ms>=turn_ms){island_phase=IS_LOOP;island_phase_start=state_ms;}
        }else if(island_phase==IS_LOOP){
            course_eight_update(fork);if(!app_running)return;
            int out=0;
            if(ISLAND_OUT_TRIGGER==1){
                /* Entry must finish before any exit fork frames count. */
                int exit_ready=island_entry_gate==ENTRY_CLEAR;
                if(course_segment==2u && S2_EXIT_MIN_M>0.0f)
                    exit_ready=exit_ready && app_s-entry_s>=S2_EXIT_MIN_M;
                if(course_segment==8u && S8_EXIT_MIN_YAW_RAD>0.0f){
                    float start=course_s8_straight()?s8_entry_split_yaw:entry_yaw;
                    exit_ready=exit_ready && start-app_yaw>=S8_EXIT_MIN_YAW_RAD;
                }
                unsigned required_frames=FORK_CONFIRM_COUNT(course_segment==8u ? S8_EXIT_FORK_FRAMES : ISLAND_OUT_FORK_FRAMES);
                if(fork && exit_ready && !(SPECIAL_ISLAND_SINGLE_FORK && course_s8_straight())){
                    if(island_out_frames<required_frames)
                        island_out_frames++;
                }else island_out_frames=0;
                if(island_out_frames>=required_frames)
                    island_out_armed=1;
                else if(!ordinary_guard_dir && island_out_armed && groups==1){
                    if(!ordinary_island)out=1;
                    else if(!island_out_pending){
                        island_out_pending=1;
                        island_fall_ms=state_ms;
                        if(loop_corner){corner_pulse_active=0;previous_corner_side=0;}
                        island_pulse_reset();
                    }
                }
                if(island_out_pending)
                    out=state_ms-island_fall_ms>=ORDINARY_PULSE_DELAY_MS;
            }else if(ISLAND_OUT_TRIGGER==2)
                out=(fabsf(app_yaw-entry_yaw)>=ISLAND_OUT_YAW);
            else if(ISLAND_OUT_TRIGGER==3)
                out=((app_s-entry_s)>=ISLAND_OUT_DIST);
            unsigned fallback_ms=ISLAND_OUT_TRIGGER==1?state_ms:ph_ms;
            if((int)fallback_ms>=(int)ISLAND_OUT_MS && !out && !island_out_pending){
                if(COURSE_ENABLE){
                    /* Eight uses its per-leg timeout; never turn a timeout
                       into a valid checkpoint completion. */
                    if(!course_spec()->eight){app_fail(12);return;}
                }else out=1;
            }
            if(!course_eight_can_exit()){
                out=0;island_out_armed=0;
                /* A continuous 2 that spans the moment can_exit() turns true
                   must start counting from that frame, not inherit frames
                   accumulated while the exit was still forbidden. */
                island_out_frames=0;
            }
            /* A fallback timer is not a confirmed eight exit landmark. */
            if(COURSE_ENABLE && course_spec()->eight && out &&
               !(island_out_armed && island_out_pending)){app_fail(11);return;}
            if(out){
                island_out_pending=0;
                if(loop_corner){corner_pulse_active=0;previous_corner_side=0;}
                island_pulse_reset();
                if(course_s8_straight()){junction.yaw=app_yaw;exit_single_ms=0;}
                island_phase=IS_TURN_OUT;island_phase_start=state_ms;
                island_exit_pulse_yaw=app_yaw;
            }
        }else if(island_phase==IS_TURN_OUT){
            int finished=ordinary_island
                ? (app_yaw-island_exit_pulse_yaw)*ISLAND_OUT_DIR*ISLAND_SIDE>=ISLAND_EXIT_PULSE_RAD
                : ph_ms>=ISLAND_TURN_OUT_MS;
            if(!course_s8_straight() && finished){island_phase=IS_LEAVE;island_phase_start=state_ms;}
        }else if(island_phase==IS_LEAVE){
            if((int)ph_ms>=(int)ISLAND_LEAVE_MS){
                course_complete(CP_ISLAND);if(!app_running)return;
                track_enter(FOLLOW);
                edge_age=100;edge_side=0;
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
    /* Check after phase advancement so a split on the trigger tick also
       hands steering straight to the selected line. */
    if(ordinary_island && new_double && island_phase==IS_TURN_OUT){
        island_phase=IS_LEAVE;
        island_phase_start=state_ms;
        track_d_ready=0;
    }
    if(course_s8_straight() && track_state==BRANCH){
        int exiting=island_phase==IS_TURN_OUT || island_phase==IS_LEAVE;
        if(island_phase==IS_LOOP && groups==2 && (island_out_frames || island_out_armed)){
            if(loop_corner){corner_pulse_active=0;previous_corner_side=0;}
            island_pulse_reset();
            junction_steer(1,0,starts,ends,v,w);
            return;
        }
        if(exiting){
            junction_steer(groups==1 || groups==2 ? 0 : -1,groups==1,starts,ends,v,w);
            if(exit_single_confirmed(groups)){
                course_complete(CP_ISLAND);if(!app_running)return;
                track_enter(FOLLOW);
                edge_age=100;edge_side=0;
            }else if(state_ms-island_phase_start>=JUNCTION_MAX_MS){
                app_fail(12);
            }
            return;
        }
    }
    if(total || track_state==SEARCH || track_state==CROSS)app_lost=0;
    if(track_state!=FOLLOW){
        /* SEARCH releases on the same centre window as the corner pulse: a
           lost line is often reacquired at the edge of the bar, and accepting
           that would hand a badly-offset line straight to the PD. BRANCH is
           owned by island_phase, CROSS keeps the centred test. */
        int exit_ok;
        int search_release_reason=0;
        if(track_state==SEARCH){
            exit_ok=(SEARCH_EXIT_MODE==0)?central:
                    corner_center_seen(starts,ends,groups);
            if(SEARCH_EXIT_MODE==2)exit_ok=exit_ok && fabsf(weight)<prev_abs;
            search_release_reason=exit_ok ? 5 : 4; /* optical or predicted, published on exit */
#if CORNER_PREDICT_EXIT
            if(groups==1 && !edge_run && !wide){
#if CORNER_COAST_EXIT
                exit_ok |= corner_coast_ready(starts[0],ends[0],SEARCH_SPIN_DIR*edge_side);
#else
                exit_ok |= turn_center_approaching(groups,starts[0],ends[0],SEARCH_SPIN_DIR*edge_side);
#endif
            }else turn_samples=0;
#endif
            if(previous_cross_hit || fork_frames>0)exit_ok=0;
        }else if(track_state==BRANCH){
            /* The island exit is owned by island_phase (IS_LEAVE), not by a
               pattern test: two-segment patterns reappear during the final
               turn and would keep resetting a 'seen'-based counter. */
            exit_ok=0;
        }else exit_ok=central; /* CROSS keeps the centred test */
        clear_frames=exit_ok ? clear_frames+1 : 0;
        if(clear_frames>=(track_state==SEARCH ? SEARCH_EXIT_FRAMES : EXIT_FRAMES)){
            int leaving_search=(track_state==SEARCH);
            if(leaving_search)corner_release_reason=search_release_reason;
            if(track_state==CROSS){course_complete(CP_CROSS);if(!app_running)return;}
            track_enter(FOLLOW);
            edge_age=100;
            /* Keep the search side across a SEARCH exit. Clearing it made the
               NEXT loss fall back to COURSE_LOSS_SEARCH_DIR and spin the
               opposite way, so a corner hunted back and forth. CROSS still
               clears it, because its direction belongs to the crossing. */
            if(!leaving_search)edge_side=0;
            /* Reset BOTH references: a stale app_line_prev would make the first
               post-exit frame pick a segment near the pre-state line position. */
            app_error_prev=app_line_prev=(starts[0]+ends[0])/11.0f-1.0f;
        }
    }
    if(track_state==SEARCH){
        if(state_ms>SEARCH_TIMEOUT_MS){app_fail(5);return;}
        search_arc(SEARCH_SPIN_DIR*edge_side,v,w);
        return;
    }
    if(track_state==CROSS){
        if(state_ms>CROSS_TIMEOUT_MS){if(COURSE_ENABLE){app_fail(12);return;}if(TRACK_RECOVER_ON_TIMEOUT)track_enter(FOLLOW);else app_fail(5);return;}
        *v=fminf(TRACK_CRUISE,0.30f);
        *w=app_clip(TRACK_HEADING_KP*(entry_yaw-app_yaw),-0.8f,0.8f);
        return;
    }
    if(track_state==BRANCH && state_ms>BRANCH_TIMEOUT_MS){
        if(COURSE_ENABLE){app_fail(12);return;}
        if(TRACK_RECOVER_ON_TIMEOUT)track_enter(FOLLOW);else app_fail(5);return;
    }
    if(!total){
        track_d_ready=0; /* next line frame must not differentiate across the gap */
        if(track_state==BRANCH){
            /* IS_LEAVE belongs to the exit too: use the smaller exit rate so a
               lost-line nudge cannot undo the gentler final turn. Direction
               expression is unchanged. */
            int d=(island_phase==IS_TURN_OUT)?ISLAND_OUT_DIR:BRANCH_TURN_DIR;
            float nudge_w=(island_phase==IS_TURN_OUT || island_phase==IS_LEAVE)?
                ISLAND_TURN_OUT_W:1.5f;
            if(branch_corner && corner_pulse_active){
                *v=TRACK_CRUISE;
                *w=-corner_pulse_side*CORNER_PULSE_W;
                fixed_pulse_output=1;
                return;
            }
            if(!ISLAND_LOOP_USE_CORNER && island_phase==IS_LOOP && !island_out_pending && island_pulse_update(0,0,0,w)){
                *v=TRACK_CRUISE;return;
            }
            if(island_phase==IS_LOOP && !island_out_pending){
                search_arc(d*ISLAND_SIDE,v,w);
                return;
            }
            *v=0.05f; *w=d*ISLAND_SIDE*nudge_w; return;
        }
        app_motor(0,0);app_cmd_l=app_cmd_r=app_il=app_ir=0;
        /* The launch hold owns the start bar: a blank frame there must neither
           spin into SEARCH nor trip the 250 ms lost-line fault. */
        app_lost+=5;
        if(app_lost>=50)app_fail(5);
        return;
    }
    app_lost=0;
    int pick=0;
    float best=10;
    for(int i=0;i<groups;i++){
        float dist=fabsf((starts[i]+ends[i])/11.0f-1.0f-app_line_prev);
        if(dist<best){pick=i;best=dist;}
    }
    int exit_side_ready=ordinary_island && island_out_pending &&
                        state_ms-island_fall_ms>=ORDINARY_EXIT_SIDE_DELAY_MS;
    int follow_nearest=ordinary_island && island_phase==IS_LOOP &&
                       island_out_armed && !exit_side_ready;
    if(track_state==BRANCH && ISLAND_FORCE_SIDE && island_phase!=IS_ARMED && !follow_nearest){
        int side=ISLAND_SIDE;
        if(ISLAND_OUT_FOLLOW_SIDE && groups>=2 &&
           (exit_side_ready || island_phase==IS_TURN_OUT || island_phase==IS_LEAVE))
            side=ISLAND_OUT_FOLLOW_SIDE;
        pick=side<0 ? 0 : groups-1;
    }
    /* New entry decision: once the two-segment pattern is confirmed, the
       LEFT segment is the island branch. Force it for the entrance turn
       independently of ISLAND_FORCE_SIDE so this A/B switch stands alone.
       A single segment is already pick=0, and TURN_OUT/LEAVE are untouched. */
    if(BRANCH_ARM_WAIT_TWO && island_two_ok && groups>=2 &&
       track_state==BRANCH && island_phase==IS_TURN_IN){
#if ISLAND_ARM_TWO_FIXED_PICK
        pick=0; /* ignore the other fork segment: 11-11 anti-jitter behaviour */
#else
        pick=ISLAND_SIDE<0?0:groups-1;
#endif
    }
    float e=(starts[pick]+ends[pick])/11.0f-1.0f;
    /* A two-segment pattern shows up a few frames BEFORE FORK_FRAMES confirms
       the island entry. Those frames are still FOLLOW, where ISLAND_FORCE_SIDE
       does not apply and pick can jump to an edge segment: e ~ +/-1, de
       clamped to LINE_D_MAX, and w slams to its +/-2 limit for one frame.
       Hold the previous steering input on those frames; the window is only a
       few ticks and BRANCH takes over as soon as FORK_FRAMES is met. */
    /* Sticky, island-only hold. Sharp-corner patterns have a wide probe and
       never open the window, so they keep their normal PD response. */
    int in_window=(FORK_ENTRY_HOLD && track_state==FOLLOW && fork_hold_tail>0);
    /* A single-lit edge probe never opens the sticky window. Hold e for
       THIS frame only: no 100 ms window (corners stay responsive), no
       release counter, no slew. Suppressed while a window/release is active
       so the existing sticky behaviour is not disturbed. */
    int one_lit_hold=(FORK_ENTRY_HOLD && FORK_HOLD_ONE_LIT &&
                      track_state==FOLLOW && one_lit_probe &&
                      !in_window && fork_release==0);
    /* On the frame track_enter(BRANCH) fires, ISLAND_FORCE_SIDE suddenly
       forces pick onto the ISLAND_SIDE segment and track_d_ready is cleared.
       Slew e through the whole IS_ARMED phase instead of switching abruptly.
       BRANCH_ARM_WAIT_TWO below extends that slew into IS_TURN_IN as well. */
    /* With BRANCH_ARM_WAIT_TWO the wide junction band is still present
       while IS_ARMED waits, and can briefly reappear inside IS_TURN_IN.
       Keep the e slew (and dterm=0 below) through TURN_IN too, so no
       single-frame band edge change can kick w. FORK_E_SLEW==0 freezes e,
       so in that case the TURN_IN extension is disabled. With the switch
       off this collapses to the old IS_ARMED-only condition. */
    int armed_slew=(FORK_ENTRY_HOLD||BRANCH_ARM_WAIT_TWO)&&
                   island_phase==IS_ARMED;
    int turn_in_slew=(BRANCH_ARM_WAIT_TWO && FORK_E_SLEW>0.0f &&
                      island_phase==IS_TURN_IN);
    int entry_slew=(track_state==BRANCH&&(armed_slew||turn_in_slew));
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
    dbg_kp_gain=(long)(LINE_KP*100.0f);
    /* Right-angle corner A/B: widen the w cap ONLY while the corner test
       fires in FOLLOW. Everywhere else the cap stays STEER_W_LIMIT. */
    float wcap=(CORNER_W_TEST && (track_state==FOLLOW || branch_corner) && corner)?
               CORNER_TEST_W_LIMIT:STEER_W_LIMIT;
    dbg_wcap=wcap;
    /* ---- Plain PD, then the corner pulse -------------------------------
       No feed-forward and no speed scaling: the PD is the same at every
       segment speed, and inside a corner the pulse REPLACES its output. */
    *w=steer_line(e,dterm,LINE_KP,track_d_ready,wcap);
    /* Constant-w corner pulse. corner_pulse_side is +1 when the lit pattern
       sits right, and negative w turns right (the PD uses w = -kp*e), so
       the sign matches. */
    if(corner_pulse_active){
        *w=-CORNER_PULSE_W*(float)corner_pulse_side;
        fixed_pulse_output=1;
    }
    if(!fork && !ISLAND_LOOP_USE_CORNER && track_state==BRANCH && island_phase==IS_LOOP &&
       (!ordinary_island || (island_entry_gate==ENTRY_CLEAR && !island_out_pending))){
        int centered=island_pulse_centered(starts[pick],ends[pick]);
        island_pulse_update(1,e,centered,w);
    }
    app_error_prev=app_line_prev=e;track_d_ready=1;
    *v=TRACK_CRUISE;
    if(!FORK_TEST_MODE && ordinary_fork_speed_limit() && FORK_BRANCH_MAX_MPS>0.0f && (fork || track_state==BRANCH))
        *v=fminf(*v,FORK_BRANCH_MAX_MPS);
    /* Active steering only in the two turn phases. The ordinary island exit uses a fixed
       angular rate. Other turn pulses add to the line-following output. */
    if(track_state==BRANCH && BRANCH_TURN_ENABLE){
        int dir=0;float turn_w=BRANCH_TURN_W;
        if(island_phase==IS_TURN_IN)dir=BRANCH_TURN_DIR;
        else if(island_phase==IS_TURN_OUT){dir=ISLAND_OUT_DIR;turn_w=ISLAND_TURN_OUT_W;}
        if(dir){
            float pulse_w=dir*ISLAND_SIDE*turn_w;
            if(!(ordinary_island && island_phase==IS_TURN_OUT))pulse_w+=*w;
            *w=app_clip(pulse_w,-STEER_W_LIMIT,STEER_W_LIMIT);
            if(!FORK_TEST_MODE && BRANCH_TURN_V>0.0f)*v=fminf(*v,BRANCH_TURN_V);
        }
    }
}
static void app_start(void){
    if(APP_MODE==0 || app_fault)return;
    if(!app_cal_ready || (APP_MODE>=3 && !app_parameters_ok())){app_fail(4);return;}
    app_s=app_yaw=app_il=app_ir=app_error_prev=app_line_prev=0;
    app_elapsed=app_hold=app_quarter=app_lost=app_settled=app_stall=0;
    track_reset();steer_test_e_prev=0;steer_test_d_ready=0;
    mode7_active=0;mode7_steady_ms=0;mode7_run_ms=0;
    app_last_completed=app_ms;app_running=1;
    elem_test_reset();course_begin();
}
static void app_control(float gz_rad){
    float v=0,w=0,tl,tr;
    unsigned floor_release=0; /* MODE5: let the wheels stop once the angle is reached */
    dbg_v=0;dbg_w=0;dbg_wcap=STEER_W_LIMIT;
    if(!app_running){app_target_l=0;app_target_r=0;return;}
    app_target_l=0;app_target_r=0;
    app_elapsed+=5;
    /* Corner pulse re-arm delay: ticks down on EVERY frame, so a pulse that
       ended inside an early-return path still re-arms on time. */
    if(corner_pulse_cool_ms>=5u)corner_pulse_cool_ms-=5u;
    else corner_pulse_cool_ms=0;
    if(app_elapsed>((COURSE_ENABLE && APP_MODE==6)?COURSE_TOTAL_MS:120000u) && !(MODE7_NO_TIMEOUT && APP_MODE==7)){
        app_fail(6);return;
    }
    if(APP_MODE==1 || APP_MODE==2){
        float u=TEST_REVERSE?-TEST_PWM:TEST_PWM;
        if(app_elapsed>=10000u){App_Stop();return;}
        app_cmd_l=APP_MODE==1?u:0;app_cmd_r=APP_MODE==2?u:0;
        app_motor(app_cmd_l,app_cmd_r);return;
    }
    app_yaw+=gz_rad*DT;
    steer_gyro += (DT/(RATE_GYRO_TAU+DT))*(gz_rad-steer_gyro);
    if(APP_MODE==8){
        if(app_elapsed>=FF_TEST_RUN_MS){App_Stop();return;}
        v=FF_TEST_SPEED_MPS;
    }else if(APP_MODE==3){
        v=TEST_SPEED_MPS;
        if(app_elapsed>=10000u){App_Stop();return;}
    }else if(APP_MODE==4){
        float remaining=1.0f-app_s;
        if(remaining<=0.005f+STOP_AHEAD_M){App_Stop();return;}
        /* Keep the body speed inside the realisable set; the wheel floor below covers MODE5/6. */
        v=fmaxf(MIN_SPEED_MPS,fminf(TEST_SPEED_MPS,DISTANCE_KP*remaining));
        w=app_clip(-HEADING_KP*app_yaw,-0.8f,0.8f);
    }else if(APP_MODE==5){
        float e=(app_quarter+1)*TURN_STEP_RAD-app_yaw;
        if(fabsf(e)<TURN_TOL_RAD && fabsf(app_vl)<0.015f && fabsf(app_vr)<0.015f){
            app_motor(0,0);app_il=app_ir=0;app_hold+=5;
            if(app_hold>=1000u){app_hold=0;app_quarter++;if(app_quarter>=4)App_Stop();}
            return;
        }
        app_hold=0;w=app_clip(TURN_KP*e,-TURN_MAX_RAD_S,TURN_MAX_RAD_S);
        /* Inside the settle window the floor must be released, otherwise the
           wheels never stop and the settle check above can never pass. */
        floor_release=(fabsf(e)<TURN_TOL_RAD);
    }else if(APP_MODE==6){
        course_tick();if(!app_running)return;
        track_control(&v,&w);
        if(!app_running || app_lost)return;
        course_limit(&v,&w);if(!app_running)return;
    }else if(APP_MODE==7){
        float e7;
        steer_test_control(&w,&e7);
        if(!mode7_active && fabsf(e7)>MODE7_TRIGGER_E){
            mode7_active=1;mode7_steady_ms=0;mode7_run_ms=0;
        }else if(mode7_active){
            mode7_run_ms+=5;
            if(fabsf(e7)<MODE7_SETTLE_E){
                mode7_steady_ms+=5;
                if(mode7_steady_ms>=MODE7_SETTLE_MS){mode7_active=0;mode7_steady_ms=0;}
            }else mode7_steady_ms=0;
            if(mode7_run_ms>=MODE7_ACTIVE_MAX_MS){mode7_active=0;mode7_run_ms=0;}
        }
        if(!mode7_active){
            app_motor(0,0);app_il=app_ir=0;app_cmd_l=app_cmd_r=0;
            app_target_l=app_target_r=0;
            return;
        }
        /* A common forward bias puts both wheels above the speed floor and
           lets the differential ride on already-moving wheels. v grows with
           |w| so the INNER wheel also stays >= MODE7_CRUISE_MPS. */
        v=MODE7_CRUISE_MPS>0.0f?
          MODE7_CRUISE_MPS+fabsf(w)*EFFECTIVE_TRACK_M*0.5f:0.0f;
    }
    dbg_v=v;dbg_w=w;
    tl=v-w*EFFECTIVE_TRACK_M*0.5f;tr=v+w*EFFECTIVE_TRACK_M*0.5f;
    /* Keep each wheel out of the deadzone in turn/line modes. The floor acts
       on |target| and keeps the sign, so MODE5 still spins in place (opposite
       signs) and MODE6 keeps its differential; it must NOT be applied to v,
       which is 0 while spinning. Released at a MODE5 target so the wheels can
       actually stop and the settle check can pass. MODE3/4 stay untouched. */
    if(MIN_SPEED_MPS>0.0f && !floor_release &&
       APP_MODE==5){
        if(tl!=0.0f)tl=copysignf(fmaxf(fabsf(tl),MIN_SPEED_MPS),tl);
        if(tr!=0.0f)tr=copysignf(fmaxf(fabsf(tr),MIN_SPEED_MPS),tr);
    }
    if(APP_MODE==6 && MIN_SPEED_FLOOR_MODE6 && MIN_SPEED_MPS>0){
        float outer=fmaxf(fabsf(tl),fabsf(tr));
        if(outer>0 && outer<MIN_SPEED_MPS){
            float scale=MIN_SPEED_MPS/outer;
            tl*=scale; tr*=scale;
        }
        if(fabsf(tl)<0.5f*MIN_SPEED_MPS){tl=0;app_il=0;}
        if(fabsf(tr)<0.5f*MIN_SPEED_MPS){tr=0;app_ir=0;}
    }
    app_target_l=tl;app_target_r=tr;
    int ff_turn=(APP_MODE==5 || (MODE7_FF_TURN && APP_MODE==7) ||
                 (APP_MODE==6 && track_state==SEARCH));
    float ffl=ff_turn?SPEED_FF_TURN_L:SPEED_FF_L;
    float ffr=ff_turn?SPEED_FF_TURN_R:SPEED_FF_R;
    app_cmd_l=app_pi(tl,app_vl,&app_il,SPEED_KP_L,SPEED_KI_L,ffl);
    app_cmd_r=app_pi(tr,app_vr,&app_ir,SPEED_KP_R,SPEED_KI_R,ffr);
    /* MODE7 only: optional duty floor so a tiny target can break static
       friction. OFF by default - a PWM floor creates a limit cycle that
       hides the real steering response. */
    if(APP_MODE==7 && MODE7_KICK_PWM>0.0f){
        if(app_target_l!=0.0f)
            app_cmd_l=copysignf(fmaxf(fabsf(app_cmd_l),MODE7_KICK_PWM),app_target_l);
        if(app_target_r!=0.0f)
            app_cmd_r=copysignf(fmaxf(fabsf(app_cmd_r),MODE7_KICK_PWM),app_target_r);
    }
    /* Sustained high effort with no motion: stop instead of heating a motor. */
    if((fabsf(app_cmd_l)>MAX_PWM*0.9f && fabsf(app_vl)<0.003f)||
       (fabsf(app_cmd_r)>MAX_PWM*0.9f && fabsf(app_vr)<0.003f))app_stall+=5;
    else app_stall=0;
    if(app_stall>=800u && !(MODE7_NO_STALL && APP_MODE==7)){app_fail(8);return;}
    app_motor(app_cmd_l,app_cmd_r);
}
static void App_Init(void){
    App_Stop();
    if(APP_MODE<0 || APP_MODE>8 || MAX_PWM<=0 || MAX_PWM>0.85f ||
       TEST_PWM<=0 || TEST_PWM>0.15f ||
       (APP_MODE==8 && (!(FF_TEST_SPEED_MPS>0 && FF_TEST_SPEED_MPS<=0.5f) ||
        FF_TEST_RUN_MS<500u || FF_TEST_RUN_MS>10000u))){app_fail(4);return;}
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
    uint32_t period=APP_SCOPE?((FORK_TEST_MODE || APP_SCOPE_STATE==6)?5u:APP_SCOPE_PERIOD_MS):200u;
    if(app_tx_pos>=app_tx_len && (uint32_t)(HAL_GetTick()-app_last_log)>=period){
        int n;app_last_log=HAL_GetTick();
        if(APP_SCOPE){
#if APP_MODE == 8
            n=snprintf(app_tx,sizeof(app_tx),"%ld,%ld,%ld,%ld,%ld,%u,%u\n",
                (long)(app_target_l*1000),(long)(app_vl*1000),(long)(app_vr*1000),
                (long)(app_cmd_l*1000),(long)(app_cmd_r*1000),app_running,app_fault);
#elif FORK_TEST_MODE
            n=snprintf(app_tx,sizeof(app_tx),"%d,%d,%u,%u,%d,%u,%u,%u\n",
                dbg_groups,dbg_fork,segment_fork_hits,fork_test_falls,track_state*10+island_phase,
                (unsigned)junction.phase,course_eight,app_fault?app_fault:(course_done?100u:0u));
#elif APP_SCOPE_STATE == 6
            n=snprintf(app_tx,sizeof(app_tx),"%lu,%d,%ld,%ld,%ld,%ld,%ld,%u\n",
                (unsigned long)app_elapsed,
                track_state==SEARCH ? 2 : scope_search_arc ? 3 : corner_pulse_active ? 1 :
                track_state==CROSS ? 4 : track_state==BRANCH ? 5 : 0,(long)(dbg_w*1000),
                (long)(steer_gyro*1000),(long)(app_yaw*1000),
                (long)(corner_remaining_rad*1000),(long)(corner_coast_rad*1000),corner_release_reason);
#elif APP_SCOPE_STATE == 5
            /* COURSE segment diagnostics (was STATE 3 in the 11-08 framework).
               CH1 segment; CH2 completed passes; CH3 element state;
               CH4 gap active/done (1/2); CH5 eight phase; CH6 segment mm;
               CH7 fault; CH8 fork rising edges in this segment. */
            n=snprintf(app_tx,sizeof(app_tx),"%u,%u,%d,%u,%u,%ld,%u,%u\n",
              course_segment,course_pass,track_state*10+island_phase,
              course_gap?1u:(course_gap_done?2u:0u),course_eight,
              (long)((app_s-course_start_s)*1000),app_fault,segment_fork_hits);
#elif APP_SCOPE_STATE == 4
            n=elem_test_csv(app_tx,(unsigned)sizeof(app_tx));
#elif APP_SCOPE_STATE == 3
            /* Corner / feed-forward speed test: every value from the SAME
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
        }else{
            /* One short group per 200 ms; groups are sampled separately.
               Each group repeats every 1 second. UART transmission remains
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
                    dbg_kp_gain,dbg_one_lit);
                break;
            default:
                n=snprintf(app_tx,sizeof(app_tx),"R S%u pass%u gap%u/%u eight%u end%u F%u\r\n",
                    course_segment,course_pass,course_gap,course_gap_done,course_eight,course_final,app_fault);
                break;
            }
            text_group=(text_group+1u)%6u;
        }
        app_tx_len=n>0?(unsigned)n:0;
        if(app_tx_len>=sizeof(app_tx))app_tx_len=sizeof(app_tx)-1;
        app_tx_pos=0;
    }
}
/* Keep polling locals off the stack during App_Init(). */
__attribute__((noinline)) static void App_Poll(void){
    uint32_t now=app_ms,gap=now-app_last_sample;
    struct bmi2_sens_data acc,gyr;
    uint16_t l,r;int32_t dl,dr;unsigned key;
    app_log();
    if(app_fault || gap<5u)return;
    app_last_sample=now;
    /* A long text-mode log line (rebuilt every 200 ms) can stretch one control
       tick past 7 ms. Restarting the 400-sample gyro average on that tick made
       APP_SCOPE 0 unable to ever finish calibration. Calibration only needs
       "still + stable gz": uneven sampling does not bias the mean. Keep the
       scheduling guard for a running car, but never reset the average. */
    if(gap>7u && app_running){app_fail(3);return;}
    l=(uint16_t)__HAL_TIM_GET_COUNTER(&htim3);r=(uint16_t)__HAL_TIM_GET_COUNTER(&htim4);
    dl=app_delta(l,app_prev_l);dr=app_delta(r,app_prev_r);app_prev_l=l;app_prev_r=r;
    app_counts_l+=dl;app_counts_r+=dr; /* raw signs, also usable before scaling */
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
