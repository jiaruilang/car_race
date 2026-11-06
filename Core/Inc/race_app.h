#ifndef RACE_APP_H
#define RACE_APP_H
/* Include ONLY ONCE, in main.c USER CODE BEGIN 0, after peripheral handles.
   No other .c file should include this implementation header. */
#include "dodo_BMI270.h"
#include "multiplexer.h"
#include <math.h>
#include <stdio.h>

/* 0 sensors; 1 left pulse; 2 right pulse; 3 speed; 4 one metre;
   5 four left 90-degree turns; 6 line following; 7 static steering bench
   (v=0, spin in place, nudge the car and watch it recover). */
#define APP_MODE 6
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
#define SPEED_FF_TURN_L 0.28f  /* MODE5 only: spin-in-place load differs from straight running */
#define SPEED_FF_TURN_R 0.47f /* tune = steady turn duty / steady turn wheel speed */
#define MAX_PWM 0.30f
#define TEST_PWM 0.042f
#define TEST_REVERSE 0
#define TEST_SPEED_MPS 0.30f
#define MIN_SPEED_MPS 0.200f  /* lowest sustainable speed (m/s); 0=off; fill after calibration */
#define STOP_AHEAD_M 0.05f   /* coast distance after cut-off (m); stop early by this; 0=off */
#define LINE_KP 2.0f       /* initial trial only; error range -1..1 */
#define LINE_KD 0.02f
#define LINE_D_MAX 20.0f  /* clamp on de/dt so a stale error cannot kick the D term */
#define HEADING_KP 2.0f   /* MODE4 only */
#define TURN_KP 1.0f
#define TURN_MAX_RAD_S 4.0f
#define TURN_TOL_RAD 0.025f  /* angle window for a completed quarter turn; also releases the speed floor */
#define MIN_SPEED_FLOOR_MODE6 1 /* 1: apply the wheel-speed floor in MODE6 too; 0: line mode exempt */
#define DISTANCE_KP 1.5f
#define APP_SCOPE 1       /* 0 text diagnostics; 1 Seekfree CSV scope */
#define APP_SCOPE_PERIOD_MS 20u /* 50Hz nominal telemetry; control stays 5ms */
#define DT 0.005f
#define PI_F 3.14159265358979f /* TRUE pi: physical dps->rad/s and log conversion ONLY */
#define TURN_STEP_RAD (PI_F*0.5f) /* MODE5: target yaw per quarter; tune here for angle bias */
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
static char app_tx[180];
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
      MAX_PWM>0 && MAX_PWM<=0.85f && TEST_SPEED_MPS>0 && TEST_SPEED_MPS<=0.5f &&
      MIN_SPEED_MPS>=0 && MIN_SPEED_MPS<=TEST_SPEED_MPS && STOP_AHEAD_M>=0 &&
      TURN_TOL_RAD>0 && TURN_STEP_RAD>0 && TRACK_HEADING_KP>0 && LINE_D_MAX>0;
}
/* MODE6 trial settings. Channels 0..11 run from left to right. */
#define TRACK_CRUISE TEST_SPEED_MPS
#define ISLAND_SIDE -1 /* -1 left, +1 right, 0 stop at candidate for inspection */
#define CENTRAL_TOTAL_MAX 9     /* widest single segment still a normal line (was 6: 7..9 grey band) */
#define CENTRAL_WEIGHT_MAX 2.0f /* |weighted sum| below this => line is centred */
#define CORNER_HALF_HITS 6      /* all 6 sensors on one side lit => corner */
#define CORNER_WEIGHT_MIN 2.5f  /* |weighted sum| above this => one side dominates */
#define CORNER_DWEIGHT_MIN 1.5f /* jump in weighted sum from last frame => shape change */
#define CORNER_FRAMES 2         /* opposite-sign frames needed to flip a latched direction */
#define CORNER_SLOW_MPS 0.20f   /* cruise cap while a corner is being approached */
#define TRACK_RECOVER_ON_TIMEOUT 1 /* 1: CROSS/BRANCH timeouts return to FOLLOW instead of fault 5 */
/* ---- Fork / island entry ---- */
#define FORK_MODE 1           /* 0 legacy both ends; 1 one edge segment + another (real car); 2 any two groups */
#define FORK_EDGE_MARGIN 3    /* an edge segment starts <=3 or ends >=8 */
#define FORK_GAP_MIN 1        /* min blank channels between the two groups */
#define FORK_SEG_MIN 1        /* min width of each group; 1 allows a single lit sensor */
#define FORK_FRAMES 2         /* consecutive fork frames before entering BRANCH */
/* ---- Continuous bend handling: weight/total is the line centre, dbend is its
       per-frame change (width-free), severity drives a graded slowdown. ---- */
#define BEND_ENABLE 1          /* 0 = fall back to the old weight-only corner test */
#define BEND_RATE_K 0.8f      /* weight of |dbend| in severity */
#define BEND_SLOW_K 1.0f      /* graded slowdown strength; 0 = only the old binary cap */
#define BEND_SEARCH_MIN 0.80f /* severity above this opens the SEARCH window */
#define BEND_DBEND_MAX 1.0f   /* clamp on a single-frame bend change */
/* B-tier strategies: both OFF by default, switch on independently. */
#define BEND_KP_ENABLE 0      /* B-tier 1: gain scheduling */
#define BEND_KP_K 0.5f        /* strength used when BEND_KP_ENABLE is 1 */
#define BEND_D_USE_DBEND 0    /* B-tier 2: use dbend instead of de as the D input */
#define BEND_D_SCALE 200.0f   /* align dbend (per frame) with de (per second) */
/* ---- Direction latching ---- */
#define SIDE_ENABLE 1         /* fixed: fallback only; modes 0 (off) and 2 (override) removed */
#define EDGE_LATCH 1          /* 1: hold a latched side until the window dies or SEARCH exits */
#define SEARCH_SPIN_W 3.0f   /* spin rate while finding the line */
#define SEARCH_SPIN_DIR -1    /* +1/-1: flip if SEARCH spins away from the line */
/* ---- Exit from a special state ---- */
#define EXIT_FRAMES 3         /* consecutive valid frames before returning to FOLLOW */
#define SEARCH_EXIT_MODE 1    /* 0 centred only; 1 any single line; 2 line moving toward centre */
/* Legacy pattern-based BRANCH exit knobs below are no longer referenced at
   runtime: the island exit is now owned by island_phase (IS_LEAVE). */
#define BRANCH_EXIT_MODE 1    /* 0 centred only; 1 any line + dwell; 2 + yaw; 3 + distance */
#define BRANCH_MIN_MS 100u    /* minimum dwell in BRANCH before an exit is allowed */
#define BRANCH_MIN_YAW 0.50f  /* rad of heading change for BRANCH_EXIT_MODE 2 */
#define BRANCH_MIN_DIST 0.15f /* metres travelled for BRANCH_EXIT_MODE 3 */
#define CROSS_TIMEOUT_MS 450u
#define BRANCH_TIMEOUT_MS 1500u /* must exceed ARMED+TURN_IN+LOOP+TURN_OUT+LEAVE */
#define SEARCH_TIMEOUT_MS 1100u
#define BRANCH_NUDGE_MS 100u
/* ---- Island sub-state machine ---- */
#define ISLAND_FORCE_SIDE 1    /* 1: lock onto the ISLAND_SIDE segment for the whole island */
#define ISLAND_OUT_TRIGGER 1   /* 0 time only; 1 second fork->fall; 2 heading; 3 distance */
#define ISLAND_OUT_YAW 3.00f   /* rad for trigger 2 */
#define ISLAND_OUT_DIST 0.60f  /* metres for trigger 3 */
#define ISLAND_OUT_MS 600u     /* hard fallback: leave LOOP after this */
#define ISLAND_LEAVE_MS 150u   /* straight running before returning to FOLLOW */
#define ISLAND_OUT_DIR 1       /* +1/-1: final turn direction relative to ISLAND_SIDE */
/* ---- Active island turn ---- */
#define BRANCH_TURN_ENABLE 1     /* 0 = PD only, no turn pulses */
#define BRANCH_TURN_TRIGGER 1    /* 0 immediate; 1 wait for groups to fall to 1; 2 distance */
#define BRANCH_FALL_FRAMES 2     /* consecutive single-group frames needed by trigger 1 */
#define BRANCH_WAIT_MAX_MS 250u  /* safety: force the entrance turn if no fall-back */
#define BRANCH_TURN_DIST 0.08f   /* trigger 2: metres from fork detection to the junction */
#define BRANCH_TURN_MS 100u      /* duration of each turn pulse */
#define BRANCH_TURN_W 1.2f       /* extra angular rate added on top of the PD output */
#define BRANCH_TURN_V 0.15f      /* speed cap during a turn */
#define BRANCH_TURN_DIR (-1)     /* entrance turn: w += DIR*ISLAND_SIDE*W */
enum { FOLLOW, CROSS, BRANCH, SEARCH };
enum { IS_NONE, IS_ARMED, IS_TURN_IN, IS_LOOP, IS_TURN_OUT, IS_LEAVE };
static int track_state, edge_side, edge_age, fork_frames, clear_frames;
static int side_flip_frames, track_d_ready, track_bend_ready;
static int island_phase, island_fall_frames, island_out_armed;
static unsigned state_ms, island_phase_start;
static float entry_yaw, entry_s, track_weight_prev, track_bend_prev;

/* 1: line PD -> yaw-rate PID -> wheel PI
   2: nonlinear line PD + gyro damping -> wheel PI
   3: existing line PD -> wheel PI */
#ifndef TRACK_STEER_MODE
#define TRACK_STEER_MODE 3
#endif
#if TRACK_STEER_MODE < 1 || TRACK_STEER_MODE > 3
#error Invalid TRACK_STEER_MODE
#endif
#define STEER2_KP2 0.0f
#define STEER2_GYRO_KD 0.0f
#define RATE_KP 0.30f
#define RATE_KI 0.0f
#define RATE_KD 0.0f
#define RATE_GYRO_TAU 0.020f
#define RATE_I_LIMIT 0.5f
static float steer_gyro, rate_i, rate_prev;
static int rate_ready;
static void steer_reset(void){
    rate_i=0;rate_prev=0;rate_ready=0;steer_gyro=0;
}
static float steer_line(float e,float dterm,float kp,int valid_previous){
    float desired=app_clip(-(kp*e+LINE_KD*dterm),-2.0f,2.0f);
#if TRACK_STEER_MODE == 1
    if(!valid_previous){rate_i=0;rate_ready=0;}
    float error=desired-steer_gyro;
    float derivative=rate_ready?-(steer_gyro-rate_prev)/DT:0;
    rate_prev=steer_gyro;rate_ready=1;
    float next=app_clip(rate_i+RATE_KI*error*DT,-RATE_I_LIMIT,RATE_I_LIMIT);
    /* desired is a nominal kinematic feedforward; PID corrects rate error. */
    float raw=desired+RATE_KP*error+next+RATE_KD*derivative;
    float output=app_clip(raw,-2.0f,2.0f);
    if(raw==output || (raw>output && error<0) || (raw<output && error>0))rate_i=next;
    return output;
#elif TRACK_STEER_MODE == 2
    return app_clip(-(kp*e+LINE_KD*dterm+STEER2_KP2*e*fabsf(e))
                    -STEER2_GYRO_KD*steer_gyro,-2.0f,2.0f);
#else
    return desired;
#endif
}
/* ---- MODE7: static steering test bench ---------------------------------
   APP_MODE 7 keeps the car in place (v=0) and lets steer_line() alone drive
   the differential: nudge the body off the line, watch the recovery. The
   MODE6 state machine, bend slowdown, island logic, SEARCH and the wheel
   speed floor are all bypassed, so only the steering loop is under test.
   All three TRACK_STEER_MODE variants are reused unchanged; steer_gyro keeps
   being updated every tick in app_control(), so the gyro feedback of modes
   1 and 2 is live while spinning in place.                                */
#define MODE7_W_LIMIT 2.0f    /* cap on |w| (rad/s); default = steer_line's own clip */
#define MODE7_KICK_PWM 0.0f  /* 0=off. Optional start-up duty floor for tiny
                                wheel targets. A PWM floor creates a limit
                                cycle, so leave it off when judging response. */
#define MODE7_NO_TIMEOUT 1   /* 1: skip the 120 s run timeout while debugging */
#define MODE7_NO_STALL 1     /* 1: skip stall fault 8 (car is held by hand) */
#define MODE7_FF_TURN 1      /* 1: SPEED_FF_TURN_L/R (spin load); 0: straight FF */
static float steer_test_e_prev;
static int steer_test_d_ready;
static void steer_test_control(float *w){
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
        /* The line swept out of the bar while spinning: freeze, do not
           keep turning on a stale error. Clear the yaw-rate PID memory. */
        *w=0;
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
    float de=steer_test_d_ready?
        app_clip((e-steer_test_e_prev)/DT,-LINE_D_MAX,LINE_D_MAX):0.0f;
    *w=app_clip(steer_line(e,de,LINE_KP,steer_test_d_ready),
                -MODE7_W_LIMIT,MODE7_W_LIMIT);
    steer_test_e_prev=e;steer_test_d_ready=1;
}
static void track_enter(int state){
    track_state=state;
    state_ms=0;
    clear_frames=0;
    entry_yaw=app_yaw;
    entry_s=app_s;
    track_d_ready=0;     /* suppress the D term on the first frame back in FOLLOW */
    track_bend_ready=0;  /* suppress a fake dbend jump on the first frame back */
    island_phase=(state==BRANCH)?IS_ARMED:IS_NONE;
    island_fall_frames=0;island_out_armed=0;island_phase_start=0;
}
static void track_reset(void){ steer_reset();
    track_enter(FOLLOW);
    edge_side=0; edge_age=100; fork_frames=0;
    side_flip_frames=0; track_weight_prev=0;
    track_bend_prev=0; track_bend_ready=0;
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
    /* Real island entry: ONE segment probes in from an edge, the other is the
       original straight line near the middle. Requiring BOTH ends to be near
       an edge (the old test) can never match that pattern. */
    int fork=0;
    if(groups>=2){
        int last=groups-1;
        int w0=ends[0]-starts[0]+1;
        int wN=ends[last]-starts[last]+1;
        int gap=starts[last]-ends[0]-1;
        int near_left=(starts[0]<=FORK_EDGE_MARGIN);
        int near_right=(ends[last]>=11-FORK_EDGE_MARGIN);
        int shape=(gap>=FORK_GAP_MIN && w0>=FORK_SEG_MIN && wN>=FORK_SEG_MIN);
        /* FORK_MODE kept as a 3-way selector: the real-car one-edge test (1)
           has NOT been validated on the car yet, so 0/2 stay available for
           A/B comparison during the island test sequence. */
        if(FORK_MODE==0)
            fork=(starts[0]==0 && ends[0]>=1 &&
                  ends[last]==11 && starts[last]<=10);
        else if(FORK_MODE==1)
            fork=shape && (near_left || near_right);
        else
            fork=shape;
    }
    int central=(groups==1 && total<=CENTRAL_TOTAL_MAX &&
                 fabsf(weight)<CENTRAL_WEIGHT_MAX);
    /* Severity drives the graded slowdown for EVERY bend; this boolean only
       opens the SEARCH window, i.e. the extreme end of the same scale. The
       one-side-all-lit test stays as a hard backstop. */
    int corner=!wide && !fork && (BEND_ENABLE ?
        (severity>=BEND_SEARCH_MIN || left_hits>=CORNER_HALF_HITS ||
         right_hits>=CORNER_HALF_HITS) :
        (left_hits>=CORNER_HALF_HITS || right_hits>=CORNER_HALF_HITS ||
         fabsf(weight)>=CORNER_WEIGHT_MIN || fabsf(dweight)>=CORNER_DWEIGHT_MIN));
    int corner_side=(fabsf(weight)>0.01f)?(weight>0.0f?1:-1):
                    (right_hits>=left_hits?1:-1);
    int side=0;
    if(groups==1 && total<=6){
        if(ends[0]<=3) side=-1;
        if(starts[0]>=8) side=1;
    }
    if(edge_age<100) edge_age++;
    if(track_state==FOLLOW){
        if(corner){
            /* One frame is enough to open the window and latch a direction:
               a gradual bend may show an edge pattern for only one frame.
               Once latched, a flip needs CORNER_FRAMES consistent frames. */
            edge_age=0;
            if(!EDGE_LATCH || !edge_side){
                edge_side=corner_side;side_flip_frames=0;
            }else if(corner_side!=edge_side){
                if(++side_flip_frames>=CORNER_FRAMES){
                    edge_side=corner_side;side_flip_frames=0;
                }
            }else side_flip_frames=0;
        }else side_flip_frames=0;
        /* Legacy single-frame shortcut. Default OFF: it used to overwrite the
           corner direction and could reverse the turn right before line loss. */
        if(SIDE_ENABLE && side && !corner && !edge_side){edge_side=side;edge_age=0;}
        fork_frames=fork ? fork_frames+1 : 0;
        if(wide) track_enter(CROSS);
        else if(fork_frames>=FORK_FRAMES){
            fork_frames=0;
            if(!ISLAND_SIDE){app_fail(5);return;}
            track_enter(BRANCH);
        }else if(!total && edge_age<=24 && edge_side) track_enter(SEARCH);
    }
    if(track_state!=FOLLOW)state_ms+=5;
    /* Island sub-state machine: a 2->1 fall-back happens at BOTH the entrance
       and the exit junction, and two-segment patterns reappear during the
       final turn. A pattern-only exit cannot tell those moments apart. */
    if(track_state==BRANCH){
        unsigned ph_ms=state_ms-island_phase_start;
        if(island_phase==IS_ARMED){
            int trigger=0;
            if(BRANCH_TURN_TRIGGER==0)trigger=1;
            else if(BRANCH_TURN_TRIGGER==1){
                if(groups==1)island_fall_frames++;
                else island_fall_frames=0;
                trigger=(island_fall_frames>=BRANCH_FALL_FRAMES);
            }else if(BRANCH_TURN_TRIGGER==2)
                trigger=(app_s-entry_s>=BRANCH_TURN_DIST);
            if((int)state_ms>=(int)BRANCH_WAIT_MAX_MS)trigger=1;
            if(trigger){island_phase=IS_TURN_IN;island_phase_start=state_ms;}
        }else if(island_phase==IS_TURN_IN){
            if((int)ph_ms>=(int)BRANCH_TURN_MS){island_phase=IS_LOOP;island_phase_start=state_ms;}
        }else if(island_phase==IS_LOOP){
            int out=0;
            if(ISLAND_OUT_TRIGGER==1){
                if(fork)island_out_armed=1;
                else if(island_out_armed && groups==1)out=1;
            }else if(ISLAND_OUT_TRIGGER==2)
                out=(fabsf(app_yaw-entry_yaw)>=ISLAND_OUT_YAW);
            else if(ISLAND_OUT_TRIGGER==3)
                out=((app_s-entry_s)>=ISLAND_OUT_DIST);
            if((int)ph_ms>=(int)ISLAND_OUT_MS)out=1;
            if(out){island_phase=IS_TURN_OUT;island_phase_start=state_ms;}
        }else if(island_phase==IS_TURN_OUT){
            if((int)ph_ms>=(int)BRANCH_TURN_MS){island_phase=IS_LEAVE;island_phase_start=state_ms;}
        }else if(island_phase==IS_LEAVE){
            if((int)ph_ms>=(int)ISLAND_LEAVE_MS){
                track_enter(FOLLOW);
                edge_age=100;edge_side=0;
                if(total)app_error_prev=app_line_prev=(starts[0]+ends[0])/11.0f-1.0f;
                else app_error_prev=app_line_prev=0;
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
            exit_ok=(SEARCH_EXIT_MODE==0)?central:seen;
            if(SEARCH_EXIT_MODE==2)exit_ok=exit_ok && fabsf(weight)<prev_abs;
        }else if(track_state==BRANCH){
            /* The island exit is owned by island_phase (IS_LEAVE), not by a
               pattern test: two-segment patterns reappear during the final
               turn and would keep resetting a 'seen'-based counter. */
            exit_ok=0;
        }else exit_ok=central; /* CROSS keeps the centred test */
        clear_frames=exit_ok ? clear_frames+1 : 0;
        if(clear_frames>=EXIT_FRAMES){
            track_enter(FOLLOW);
            edge_age=100;edge_side=0;
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
        if(state_ms>CROSS_TIMEOUT_MS){if(TRACK_RECOVER_ON_TIMEOUT)track_enter(FOLLOW);else app_fail(5);return;}
        *v=fminf(TRACK_CRUISE,0.18f);
        *w=app_clip(TRACK_HEADING_KP*(entry_yaw-app_yaw),-0.8f,0.8f);
        return;
    }
    if(track_state==BRANCH && state_ms>BRANCH_TIMEOUT_MS){
        if(TRACK_RECOVER_ON_TIMEOUT)track_enter(FOLLOW);else app_fail(5);return;
    }
    if(!total){
        track_d_ready=0; /* next line frame must not differentiate across the gap */
        if(track_state==BRANCH){
            int d=(island_phase==IS_TURN_OUT)?ISLAND_OUT_DIR:BRANCH_TURN_DIR;
            *v=0.05f; *w=d*ISLAND_SIDE*1.5f; return;
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
    if(track_state==BRANCH && ISLAND_FORCE_SIDE) pick=ISLAND_SIDE<0 ? 0 : groups-1;
    float e=(starts[pick]+ends[pick])/11.0f-1.0f;
    /* Differentiate only when the previous frame also followed the line:
       after a lost line / state switch a stale e divided by DT would kick w
       to its limit. LINE_D_MAX caps the remaining single-frame jumps. */
    float de=track_d_ready?app_clip((e-app_error_prev)/DT,-LINE_D_MAX,LINE_D_MAX):0.0f;
    /* B-tier option 2: dbend covers the whole lit pattern, so a segment switch
       cannot kick it. BEND_D_SCALE puts it in the same units as de. */
    float dterm=(BEND_ENABLE && BEND_D_USE_DBEND)?
        app_clip(dbend*BEND_D_SCALE,-LINE_D_MAX,LINE_D_MAX):de;
    float kp=LINE_KP;
    /* B-tier option 1: gain scheduling, inert while BEND_KP_ENABLE is 0. */
    if(BEND_ENABLE && BEND_KP_ENABLE)kp*=1.0f+BEND_KP_K*severity;
    *w=steer_line(e,dterm,kp,track_d_ready);
    app_error_prev=app_line_prev=e;track_d_ready=1;
    *v=TRACK_CRUISE/(1+fabsf(*w));
    /* Graded slowdown: every bend reduces v, not just a right-angle corner.
       The old binary cap remains as the extreme end of the same scale. */
    if(BEND_ENABLE && BEND_SLOW_K>0.0f)
        *v=fminf(*v,TRACK_CRUISE/(1.0f+BEND_SLOW_K*severity));
    /* Corner approach: cap speed by how far the lit pattern has shifted, but
       keep the PD steering instead of overriding w with a fixed bang-bang. */
    if(track_state==FOLLOW && (corner || fabsf(weight)>=CENTRAL_WEIGHT_MAX))
        *v=fminf(*v,CORNER_SLOW_MPS);
    if(fork || track_state==BRANCH)*v=fminf(*v,0.15f);
    /* Active steering only in the two turn phases. Both pulses add to the PD
       output rather than replacing it, so the chosen line is still tracked. */
    if(track_state==BRANCH && BRANCH_TURN_ENABLE){
        int dir=0;
        if(island_phase==IS_TURN_IN)dir=BRANCH_TURN_DIR;
        else if(island_phase==IS_TURN_OUT)dir=ISLAND_OUT_DIR;
        if(dir){
            *w=app_clip(*w+dir*ISLAND_SIDE*BRANCH_TURN_W,-2,2);
            *v=fminf(*v,BRANCH_TURN_V);
        }
    }
}
static void app_start(void){
    if(APP_MODE==0 || app_fault)return;
    if(!app_cal_ready || (APP_MODE>=3 && !app_parameters_ok())){app_fail(4);return;}
    app_s=app_yaw=app_il=app_ir=app_error_prev=app_line_prev=0;
    app_elapsed=app_hold=app_quarter=app_lost=app_settled=app_stall=0;
    track_reset();steer_test_e_prev=0;steer_test_d_ready=0;
    app_last_completed=app_ms;app_running=1;
}
static void app_control(float gz_rad){
    float v=0,w=0,tl,tr;
    unsigned floor_release=0; /* MODE5: let the wheels stop once the angle is reached */
    if(!app_running){app_target_l=0;app_target_r=0;return;}
    app_target_l=0;app_target_r=0;
    app_elapsed+=5;
    if(app_elapsed>120000u && !(MODE7_NO_TIMEOUT && APP_MODE==7)){
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
    if(APP_MODE==3){
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
        track_control(&v,&w);
        if(!app_running || app_lost)return;
    }else if(APP_MODE==7){
        /* Static bench: no forward speed, w comes only from the steering
           loop; tl/tr below become a pure spin-in-place differential. */
        v=0;
        steer_test_control(&w);
    }
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
    }    app_target_l=tl;app_target_r=tr;
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
    if(APP_MODE<0 || APP_MODE>7 || MAX_PWM<=0 || MAX_PWM>0.85f ||
       TEST_PWM<=0 || TEST_PWM>0.15f){app_fail(4);return;}
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
        if(APP_SCOPE){
            /* CH1 L target, CH2 L measured, CH3 R target, CH4 R measured (mm/s).
               CH5 run, CH6 fault, CH7 calibrated,
               CH8 RIGHT PWM in 0.001 duty (saturated at +/- MAX_PWM*1000). */
            float ltarget=(app_running && !app_fault)?app_target_l:0;
            float rtarget=(app_running && !app_fault)?app_target_r:0;
            n=snprintf(app_tx,sizeof(app_tx),
              "%ld,%ld,%ld,%ld,%u,%u,%u,%ld\n",
              (long)(ltarget*1000),(long)(app_vl*1000),
              (long)(rtarget*1000),(long)(app_vr*1000),
              app_running,app_fault,app_cal_ready,
              (long)(app_cmd_r*1000));
        }else{
            n=snprintf(app_tx,sizeof(app_tx),
              "M%d run%u F%u cal%u key%u L%ld R%ld vL%ld vR%ld s%ld a%ld z%ld q%03X uL%ld uR%ld\r\n",
              APP_MODE,app_running,app_fault,app_cal_ready,app_key_stable,
              (long)app_counts_l,(long)app_counts_r,(long)(app_vl*1000),
              (long)(app_vr*1000),(long)(app_s*1000),(long)(app_yaw*1800/PI_F),
              (long)(app_gz*10),(unsigned)app_mask,
              (long)(app_cmd_l*1000),(long)(app_cmd_r*1000));
        }
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
    if(gap>7u){if(app_running){app_fail(3);return;}app_cal_samples=0;app_bias_sum=app_bias_square=0;}
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
