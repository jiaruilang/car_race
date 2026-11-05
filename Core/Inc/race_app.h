#ifndef RACE_APP_H
#define RACE_APP_H
/* Include ONLY ONCE, in main.c USER CODE BEGIN 0, after peripheral handles.
   No other .c file should include this implementation header. */
#include "dodo_BMI270.h"
#include "multiplexer.h"
#include <math.h>
#include <stdio.h>

/* 0 sensors; 1 left pulse; 2 right pulse; 3 speed; 4 one metre;
   5 four left 90-degree turns; 6 basic line following (NO ring routing). */
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
#define LINE_KP 1.0f       /* initial trial only; error range -1..1 */
#define LINE_KD 0.0f
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
#define ISLAND_SIDE 0 /* -1 left, +1 right, 0 stop at candidate for inspection */
#define CENTRAL_TOTAL_MAX 9     /* widest single segment still a normal line (was 6: 7..9 grey band) */
#define CENTRAL_WEIGHT_MAX 2.0f /* |weighted sum| below this => line is centred */
#define CORNER_HALF_HITS 6      /* all 6 sensors on one side lit => corner */
#define CORNER_WEIGHT_MIN 2.5f  /* |weighted sum| above this => one side dominates */
#define CORNER_DWEIGHT_MIN 2.0f /* jump in weighted sum from last frame => shape change */
#define CORNER_FRAMES 2         /* consecutive corner frames before latching a direction */
#define CORNER_SLOW_MPS 0.12f   /* cruise cap while a corner is being approached */
#define TRACK_RECOVER_ON_TIMEOUT 1 /* 1: CROSS/BRANCH timeouts return to FOLLOW instead of fault 5 */
enum { FOLLOW, CROSS, BRANCH, SEARCH };
static int track_state, edge_side, edge_age, fork_frames, clear_frames;
static int corner_frames, track_d_ready;
static unsigned state_ms;
static float entry_yaw, track_weight_prev;

static void track_enter(int state){
    track_state=state;
    state_ms=0;
    clear_frames=0;
    entry_yaw=app_yaw;
    track_d_ready=0; /* suppress the D term on the first frame back in FOLLOW */
}
static void track_reset(void){
    track_enter(FOLLOW);
    edge_side=0; edge_age=100; fork_frames=0;
    corner_frames=0; track_weight_prev=0;
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
    track_weight_prev=weight; /* update before any early return */
    int wide=(groups==1 && total>=10);
    int fork=(groups>=2 && starts[0]==0 && ends[0]>=1 &&
              ends[groups-1]==11 && starts[groups-1]<=10);
    int central=(groups==1 && total<=CENTRAL_TOTAL_MAX &&
                 fabsf(weight)<CENTRAL_WEIGHT_MAX);
    /* Corner = one side dominates, or the overall weight jumped. Exclude the
       symmetric wide/fork patterns so a cross is never latched as a corner. */
    int corner=!wide && !fork && (left_hits>=CORNER_HALF_HITS ||
               right_hits>=CORNER_HALF_HITS || fabsf(weight)>=CORNER_WEIGHT_MIN ||
               fabsf(dweight)>=CORNER_DWEIGHT_MIN);
    int corner_side=(fabsf(weight)>0.01f)?(weight>0.0f?1:-1):
                    (right_hits>=left_hits?1:-1);
    int side=0;
    if(groups==1 && total<=6){
        if(ends[0]<=3) side=-1;
        if(starts[0]>=8) side=1;
    }
    if(edge_age<100) edge_age++;
    if(track_state==FOLLOW){
        if(corner){if(corner_frames<100)corner_frames++;}
        else corner_frames=0;
        if(corner_frames>=CORNER_FRAMES){edge_side=corner_side;edge_age=0;}
        if(side){edge_side=side;edge_age=0;}
        fork_frames=fork ? fork_frames+1 : 0;
        if(wide) track_enter(CROSS);
        else if(fork_frames>=3){
            fork_frames=0;
            if(!ISLAND_SIDE){app_fail(5);return;}
            track_enter(BRANCH);
        }else if(!total && edge_age<=24 && edge_side) track_enter(SEARCH);
    }
    if(track_state!=FOLLOW)state_ms+=5;
    if(total || track_state==SEARCH || track_state==CROSS)app_lost=0;
    if(track_state!=FOLLOW){
        clear_frames=central ? clear_frames+1 : 0;
        if(clear_frames>=3){
            track_enter(FOLLOW);
            edge_age=100;edge_side=0;
            app_error_prev=(starts[0]+ends[0])/11.0f-1.0f;
        }
    }
    if(track_state==SEARCH){
        if(state_ms>1100){app_fail(5);return;}
        *v=0; *w=-edge_side*2.3f; return;
    }
    if(track_state==CROSS){
        if(state_ms>450){if(TRACK_RECOVER_ON_TIMEOUT)track_enter(FOLLOW);else app_fail(5);return;}
        *v=fminf(TRACK_CRUISE,0.18f);
        *w=app_clip(TRACK_HEADING_KP*(entry_yaw-app_yaw),-0.8f,0.8f);
        return;
    }
    if(track_state==BRANCH && state_ms>550){
        if(TRACK_RECOVER_ON_TIMEOUT)track_enter(FOLLOW);else app_fail(5);return;
    }
    if(!total){
        track_d_ready=0; /* next line frame must not differentiate across the gap */
        if(track_state==BRANCH && state_ms<=200){
            *v=0.05f; *w=-ISLAND_SIDE*1.5f; return;
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
    if(track_state==BRANCH) pick=ISLAND_SIDE<0 ? 0 : groups-1;
    float e=(starts[pick]+ends[pick])/11.0f-1.0f;
    /* Differentiate only when the previous frame also followed the line:
       after a lost line / state switch a stale e divided by DT would kick w
       to its limit. LINE_D_MAX caps the remaining single-frame jumps. */
    float de=track_d_ready?app_clip((e-app_error_prev)/DT,-LINE_D_MAX,LINE_D_MAX):0.0f;
    *w=app_clip(-(LINE_KP*e+LINE_KD*de),-2,2);
    app_error_prev=app_line_prev=e;track_d_ready=1;
    *v=TRACK_CRUISE/(1+fabsf(*w));
    /* Corner approach: cap speed by how far the lit pattern has shifted, but
       keep the PD steering instead of overriding w with a fixed bang-bang. */
    if(track_state==FOLLOW && (corner || fabsf(weight)>=CENTRAL_WEIGHT_MAX))
        *v=fminf(*v,CORNER_SLOW_MPS);
    if(fork || track_state==BRANCH)*v=fminf(*v,0.15f);
}
static void app_start(void){
    if(APP_MODE==0 || app_fault)return;
    if(!app_cal_ready || (APP_MODE>=3 && !app_parameters_ok())){app_fail(4);return;}
    app_s=app_yaw=app_il=app_ir=app_error_prev=app_line_prev=0;
    app_elapsed=app_hold=app_quarter=app_lost=app_settled=app_stall=0;
    track_reset();app_last_completed=app_ms;app_running=1;
}
static void app_control(float gz_rad){
    float v=0,w=0,tl,tr;
    unsigned floor_release=0; /* MODE5: let the wheels stop once the angle is reached */
    if(!app_running){app_target_l=0;app_target_r=0;return;}
    app_target_l=0;app_target_r=0;
    app_elapsed+=5;
    if(app_elapsed>120000u){app_fail(6);return;}
    if(APP_MODE==1 || APP_MODE==2){
        float u=TEST_REVERSE?-TEST_PWM:TEST_PWM;
        if(app_elapsed>=10000u){App_Stop();return;}
        app_cmd_l=APP_MODE==1?u:0;app_cmd_r=APP_MODE==2?u:0;
        app_motor(app_cmd_l,app_cmd_r);return;
    }
    app_yaw+=gz_rad*DT;
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
    float ffl=(APP_MODE==5 || (APP_MODE==6 && track_state==SEARCH))?SPEED_FF_TURN_L:SPEED_FF_L;
    float ffr=(APP_MODE==5 || (APP_MODE==6 && track_state==SEARCH))?SPEED_FF_TURN_R:SPEED_FF_R;
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
    if(APP_MODE<0 || APP_MODE>6 || MAX_PWM<=0 || MAX_PWM>0.85f ||
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
