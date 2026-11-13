#ifndef COURSE_ELEMENT_TEST_H
#define COURSE_ELEMENT_TEST_H
/* =====================================================================
   Course element RECOGNITION test -- current track, two landmarks.

   S1 = island (BRANCH enters -> IS_LEAVE finishes): ONE landmark
   S2 = cross  (wide enters CROSS -> CROSS exits): TWO completed
        crossings of the SAME cross. The road between the first and the
        second crossing is NOT given its own segment number -- same idea
        as the island, where the entry and the exit are one landmark.
        et_seg advances only after pass N completes.

   PURPOSE: tomorrow's dry run only has to prove that the segmentation
   state machine can RECOGNISE these two landmarks, in order. This file
   does NOT drive the car: no v / w writes, no app_fail(), no state
   changes, no speed caps, no island/cross handling. It is a pure
   observer plus a one-line CSV trace.

   JUDGEMENT CONSISTENCY: not a single optical classifier is redefined
   here. Every input is an ALREADY-COMPUTED value from track_control():
     in_island     = (track_state==BRANCH)
     in_cross      = (track_state==CROSS)
     island_leaving= (island_phase==IS_LEAVE)
     wide / fork / groups / total
   Element recognition is therefore the EXISTING state machine's own
   conclusion (BRANCH / CROSS entry and exit edges), not a new test.
   Completion is accepted only after the same phase evidence the
   landmark framework uses (IS_LEAVE for the island, a re-acquired
   centred line for the cross), accumulated while the state is active.

   NOTE ON TIMING: the observe call sits at the dbg_* publish line
   (~L553), which runs BEFORE this tick's state transitions. It sees the
   previous tick's track_state/island_phase and this tick's fresh pattern.
   That one-tick (5 ms) lag is harmless: BRANCH / CROSS stay active for
   many ticks. Instantaneous flags such as fork_frames>=FORK_FRAMES or
   clear_frames>=EXIT_FRAMES are deliberately NOT used -- track_enter()
   clears them in the same tick, so they cannot be sampled reliably.

   NOTE ON S1/S2: these are the CURRENT track's segment numbers
   (S1 island, S2 cross). course_config.h of the new-track framework
   defines S1 = CROSS and S2 = ISLAND, i.e. the opposite order. Do not
   mix them up; ELEM_TEST_ORDER selects which landmark each segment
   expects.

   WIRING (done manually tomorrow, NO existing file is edited by this
   deliverable -- see the companion guide for the exact paste-in lines):
     1) copy this file to Core/Inc/
     2) in race_app.h, after the static declarations (~L304):
          #include "course_element_test.h"
     3) widen the scope range check L57:  >3  ->  >4
     4) in track_control(), at the dbg_* publish line (~L553):
          elem_test_observe(track_state==BRANCH,track_state==CROSS,
              island_phase==IS_LEAVE,wide,fork,groups,total,5u);
     5) in app_start(), after track_reset(): elem_test_reset();
     6) in app_log(), add the APP_SCOPE_STATE==4 branch:
          #if APP_SCOPE_STATE == 4
            n=elem_test_csv(app_tx,(unsigned)sizeof(app_tx));
          #elif APP_SCOPE_STATE == 3
            ... existing STATE 3 block (change #if to #elif) ...
        (When ELEM_TEST_ENABLE==0 the csv helper returns 0, so STATE 4
         then emits nothing and no other branch is disturbed.)
     7) set APP_SCOPE_STATE 4 for this test; set it back to 3 for the
        right-angle-corner speed test. Keep TRACK_FLAT_SPEED 1.

   A/B: ELEM_TEST_ENABLE 0 turns every hook into a no-op.
   ===================================================================== */
#include <stdio.h>

#ifndef ELEM_TEST_ENABLE
#define ELEM_TEST_ENABLE 1
#endif
#if ELEM_TEST_ENABLE < 0 || ELEM_TEST_ENABLE > 1
#error Invalid ELEM_TEST_ENABLE
#endif

/* Re-arm only after a short clean straight, mirroring COURSE_REARM_MS. */
#ifndef ELEM_TEST_REARM_MS
#define ELEM_TEST_REARM_MS   60u
#endif
/* Per-segment deadline; a miss is COUNTED, never stops the car. */
#ifndef ELEM_TEST_SEG_MAX_MS
#define ELEM_TEST_SEG_MAX_MS 90000u
#endif
/* Must match CENTRAL_TOTAL_MAX in race_app.h (default 9). */
#ifndef ELEM_TEST_CENTRAL_MAX
#define ELEM_TEST_CENTRAL_MAX 9
#endif
/* 0: S1 expects island, S2 expects cross (CURRENT track).
   1: S1 expects cross,  S2 expects island (NEW track course_map[S1]/[S2]).
   This merged package targets the NEW track, so the default is 1. */
#ifndef ELEM_TEST_ORDER
#define ELEM_TEST_ORDER 1
#endif
#if ELEM_TEST_ORDER < 0 || ELEM_TEST_ORDER > 1
#error Invalid ELEM_TEST_ORDER
#endif
/* How many COMPLETED crossings of the same cross close its segment.
   A/B: 1 = the old one-cross behaviour, bit-for-bit; 2 = the fix.
   The segment count (ET_SEG_TOTAL) never changes: the road between the
   two crossings is deliberately NOT numbered. */
#ifndef ELEM_TEST_CROSS_PASSES
#define ELEM_TEST_CROSS_PASSES 2
#endif
#if ELEM_TEST_CROSS_PASSES < 1 || ELEM_TEST_CROSS_PASSES > 4
#error Invalid ELEM_TEST_CROSS_PASSES
#endif

#define ET_LM_ISLAND 1
#define ET_LM_CROSS  2
#define ET_SEG_TOTAL 2u

/* Event codes published in the CSV trace. */
enum {
    ET_EV_NONE=0, ET_EV_ISLAND_ENTER=1, ET_EV_ISLAND_DONE=2,
    ET_EV_CROSS_ENTER=3, ET_EV_CROSS_DONE=4, ET_EV_ORDER_ERR=5,
    ET_EV_DUP=6, ET_EV_TIMEOUT=7, ET_EV_ABNORMAL=8, ET_EV_FINISH=9,
    ET_EV_CROSS_PASS=10  /* one crossing done, segment still open */
};

#if ELEM_TEST_ENABLE

static unsigned et_seg, et_expect, et_armed;
static unsigned et_seg_ms, et_clean_ms, et_last_ev;
static unsigned et_ok, et_order_err, et_dup_err, et_timeout, et_abnormal;
static unsigned et_island_enter, et_island_done, et_cross_enter, et_cross_done;
static unsigned et_cross_pass; /* completed crossings inside the current seg */
static int et_prev_island, et_prev_cross, et_saw_leave, et_saw_line;
static int et_ii, et_ic, et_grp, et_tot, et_fk, et_wd;

static unsigned et_expect_of(unsigned seg){
    if(seg==1u)return (ELEM_TEST_ORDER?ET_LM_CROSS:ET_LM_ISLAND);
    if(seg==2u)return (ELEM_TEST_ORDER?ET_LM_ISLAND:ET_LM_CROSS);
    return 0u;
}
/* Only the cross segment needs more than one completed landmark. */
static unsigned et_passes_of(unsigned seg){
    return (seg==(ELEM_TEST_ORDER?1u:2u))?(unsigned)ELEM_TEST_CROSS_PASSES:1u;
}

static void elem_test_reset(void){
    et_seg=1u;et_expect=et_expect_of(1u);et_armed=1u;
    et_seg_ms=et_clean_ms=0;et_last_ev=ET_EV_NONE;
    et_ok=et_order_err=et_dup_err=et_timeout=et_abnormal=0;
    et_island_enter=et_island_done=et_cross_enter=et_cross_done=0;
    et_cross_pass=0;
    et_prev_island=et_prev_cross=et_saw_leave=et_saw_line=0;
    et_ii=et_ic=et_grp=et_tot=et_fk=et_wd=0;
}

/* Called once per 5 ms control tick with the CURRENT frame's flags.
   Edge detection only: it never touches the running state machine. */
static void elem_test_observe(int in_island,int in_cross,int island_leaving,
                              int wide,int fork,int groups,int total,
                              unsigned tick_ms){
    if(!et_seg)elem_test_reset();
    et_ii=!!in_island;et_ic=!!in_cross;
    et_grp=groups;et_tot=total;et_fk=!!fork;et_wd=!!wide;
    et_seg_ms+=tick_ms;
    unsigned ev=ET_EV_NONE;

    /* Re-arm gate: a short clean straight (single centred line, no fork,
       no wide, FOLLOW) after the previous landmark. Same idea as the
       framework's course_observe() rearm, but purely observational. */
    if(!et_armed){
        int clean=(!in_island && !in_cross && groups==1 && total>0 &&
                   total<=ELEM_TEST_CENTRAL_MAX && !wide && !fork);
        et_clean_ms=clean?et_clean_ms+tick_ms:0u;
        if(et_clean_ms>=ELEM_TEST_REARM_MS){et_armed=1u;et_clean_ms=0u;}
    }

    /* Phase evidence, accumulated while the element state is active.
       IS_LEAVE lasts ISLAND_LEAVE_MS and a re-acquired centred line is
       present for several ticks before CROSS exits, so a one-tick lag
       cannot miss either of them. */
    if(in_island && island_leaving)et_saw_leave=1;
    if(in_cross && groups==1 && total>0 && total<=ELEM_TEST_CENTRAL_MAX)
        et_saw_line=1;

    /* Island entry / completion edges. */
    if(in_island && !et_prev_island){
        et_island_enter++;et_saw_leave=0;ev=ET_EV_ISLAND_ENTER;
    }
    if(!in_island && et_prev_island){
        if(et_saw_leave){et_island_done++;ev=ET_EV_ISLAND_DONE;}
        else {et_abnormal++;ev=ET_EV_ABNORMAL;}
        et_saw_leave=0;
    }
    /* Cross entry / completion edges. */
    if(in_cross && !et_prev_cross){
        et_cross_enter++;et_saw_line=0;ev=ET_EV_CROSS_ENTER;
    }
    if(!in_cross && et_prev_cross){
        if(et_saw_line){et_cross_done++;ev=ET_EV_CROSS_DONE;}
        else {et_abnormal++;ev=ET_EV_ABNORMAL;}
        et_saw_line=0;
    }

    /* Segment advancement: only a completed landmark can advance.
       The cross needs et_passes_of() COMPLETED crossings. Until the last
       one, the segment number stays put and the road between the two
       crossings is not numbered -- exactly like an island's in/out pair.
       Re-arm after each pass so the second crossing is a fresh edge. */
    if(ev==ET_EV_ISLAND_DONE || ev==ET_EV_CROSS_DONE){
        unsigned lm=(ev==ET_EV_ISLAND_DONE)?ET_LM_ISLAND:ET_LM_CROSS;
        if(!et_armed || et_seg>ET_SEG_TOTAL){et_dup_err++;ev=ET_EV_DUP;}
        else if(lm!=et_expect){et_order_err++;ev=ET_EV_ORDER_ERR;}
        else if(lm==ET_LM_CROSS &&
                ++et_cross_pass<et_passes_of(et_seg)){
            et_armed=0u;et_seg_ms=0u;ev=ET_EV_CROSS_PASS;
        }
        else{
            et_cross_pass=0u;
            et_ok++;et_seg++;et_expect=et_expect_of(et_seg);
            et_armed=0u;et_seg_ms=0u;
            if(et_seg>ET_SEG_TOTAL)ev=ET_EV_FINISH;
        }
    }
    /* A miss is reported, never enforced (no app_fail, no stop). */
    if(et_seg<=ET_SEG_TOTAL && et_seg_ms>=ELEM_TEST_SEG_MAX_MS){
        et_timeout++;ev=ET_EV_TIMEOUT;et_seg_ms=0u;
    }
    et_prev_island=!!in_island;et_prev_cross=!!in_cross;
    et_last_ev=ev;
}

/* One CSV line, all values from the SAME tick. < 100 bytes. 17 columns.
   seg,expect,ev,armed,ii,ic,grp,tot,fk,wd,ok,oe,dup,to,ab,cp,ms
   cp = completed crossings of the current cross (0..PASSES-1). */
static int elem_test_csv(char *buf,unsigned n){
    return snprintf(buf,n,"%u,%u,%u,%u,%d,%d,%d,%d,%d,%d,%u,%u,%u,%u,%u,%u,%lu\n",
        et_seg,et_expect,et_last_ev,et_armed,et_ii,et_ic,
        et_grp,et_tot,et_fk,et_wd,et_ok,et_order_err,et_dup_err,
        et_timeout,et_abnormal,et_cross_pass,(unsigned long)et_seg_ms);
}
/* Pass only when both segments closed in order, the cross was actually
   crossed the required number of times, and no error was recorded. */
static int elem_test_pass(void){
    return (et_seg>ET_SEG_TOTAL && et_ok==ET_SEG_TOTAL &&
            et_cross_done>=ELEM_TEST_CROSS_PASSES &&
            !et_order_err && !et_dup_err && !et_timeout && !et_abnormal);
}
/* Counters for the guide's verdict table. */
static unsigned elem_test_ok(void){return et_ok;}
static unsigned elem_test_seg(void){return et_seg;}

#else /* ELEM_TEST_ENABLE == 0 : every hook becomes a no-op */
#define elem_test_reset() ((void)0)
#define elem_test_observe(a,b,c,d,e,f,g,h) ((void)0)
static int elem_test_csv(char *buf,unsigned n){(void)buf;(void)n;return 0;}
static int elem_test_pass(void){return 1;}
static unsigned elem_test_ok(void){return 0;}
static unsigned elem_test_seg(void){return 0;}
#endif

#endif /* COURSE_ELEMENT_TEST_H */
