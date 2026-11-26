#ifndef COURSE_ELEMENTS_H
#define COURSE_ELEMENTS_H

/* Each row: segment, completed CROSS passes, nominal wheel-centre travel (m),
   target speed (m/s). Replace V on any individual row to tune that element.
   Adjacent tangent lengths are deducted from dimensioned vertex distances.
   Compound lengths are nominal; the next checkpoint re-anchors odometry. */
#ifndef COURSE_ELEMENT_MODE
#define COURSE_ELEMENT_MODE 0
#endif
#define COURSE_ELEMENT_DEFAULT_MPS 0.83f
#define COURSE_START_ZONE_MPS 1.5f /* before the start stripe */
#define COURSE_ELEMENT_LOOKAHEAD_M 0.12f
#define COURSE_ELEMENT_ACCEL_WAIT_MS 50u
#define COURSE_ELEMENT_RADIUS_M (0.90f/12.5f)

#if COURSE_ELEMENT_MODE
enum { ELEMENT_LINE, ELEMENT_GAP, ELEMENT_ISLAND };
typedef struct { unsigned char segment,pass; float metres,speed; unsigned char bends,kind; } CourseElement;
/* bends counts confirmed turns required to finish a bend or compound element. */
#define COURSE_CROSS_ELEMENT_MPS COURSE_ELEMENT_DEFAULT_MPS
#define V COURSE_ELEMENT_DEFAULT_MPS
/* First and return CROSS speeds for each loop, indexed by race segment. */
static const float course_cross_speed[11][2]={
    [1]={V,V}, [3]={V,V}, [4]={V,V}, [5]={V,V}, [6]={V,V}, [7]={V,V}
};
#define R COURSE_ELEMENT_RADIUS_M
#define Q (1.5707963f*R) /* 90-degree arc */
#define T (0.5773503f*R) /* tangent deduction for a 60-degree turn */
static const CourseElement course_elements[]={
    /* S1: start stripe eastbound, lower zigzag, then north to first CROSS. */
    {1,0,.450f-.414214f*R,1.5}, {1,0,.785398f*R,V,1}, /* 135-degree included angle */
    {1,0,1.200f*1.414214f-2*.414214f*R,V,5}, {1,0,.785398f*R,V,1}, /* approach; whole zigzag */
    {1,0,.500f-1.414214f*R,1.2}, {1,0,Q,V,1}, /* north; turn west */
    {1,0,.250f-2*R,V}, {1,0,Q,V,1}, /* west; turn north */
    {1,0,.500f-R,1.2}, 
    /* CROSS to first corner; two inner sides; last corner to CROSS. */
    {1,1,.250f-R,1.2}, {1,1,Q,V,1}, {1,1,.250f-2*R,V}, {1,1,Q,V,1}, {1,1,.250f-2*R,V}, {1,1,Q,V,1}, {1,1,.250f-R,1.2}, /* approach CROSS; 250 mm square */
    /* S2: westbound approach and whole lower hexagonal island. */
    {2,0,.420f,1.2}, {2,0,1.200f,0.9,0,ELEMENT_ISLAND},
    /* S3: west, north, east, north; then whole upper octagon with its stem. */
    {3,0,.330f-R,V}, {3,0,Q,V,1},
    {3,0,.500f-2*R,1.0}, {3,0,Q,V,1},
    {3,0,.650f-2*R,1.2}, {3,0,Q,V,1},
    {3,0,.250f-R,1.1}, {3,1,.200f,1.1}, {3,1,1.650f,1.1,8},
    {3,1,.200f-R,1.3}, {3,1,Q,V,1}, {3,1,.250f-R,1.2},
    /* S4: west, north, west to the gap, blank, west, south to CROSS. */
    {4,0,.400f-R,1.1}, {4,0,Q,V,1},
    {4,0,.250f-2*R,V}, {4,0,Q,V,1},
    {4,0,.500f-R,1.5}, {4,0,.650f,1.5,0,ELEMENT_GAP}, /* approach; gap horizontal projection */
    {4,0,.500f-R,1.5}, {4,0,Q,V,1}, {4,0,.250f-R,1.2},
    
    /* CROSS to first corner; two inner sides; last corner to CROSS. */
    {4,1,.250f-R,1.2}, {4,1,Q,V,1}, {4,1,.250f-2*R,V}, {4,1,Q,V,1}, {4,1,.250f-2*R,V}, {4,1,Q,V,1}, {4,1,.250f-R,1.2}, /* 250 mm square */
    /* S5: short westbound link, then 350 mm square. */
    {5,0,.230f,1.2}, 
    /* CROSS to first corner; two inner sides; last corner to CROSS. */
    {5,1,.350f-R,1.2}, {5,1,Q,V,1}, {5,1,.350f-2*R,V}, {5,1,Q,V,1}, {5,1,.350f-2*R,V}, {5,1,Q,V,1}, {5,1,.350f-R,1.2},
    /* S6: southbound link, then 250 mm square. */
    {6,0,.490f,1.2}, 
    /* CROSS to first corner; two inner sides; last corner to CROSS. */
    {6,1,.250f-R,1.2}, {6,1,Q,V,1}, {6,1,.250f-2*R,V}, {6,1,Q,V,1}, {6,1,.250f-2*R,V}, {6,1,Q,V,1}, {6,1,.250f-R,1.2},
    /* S7: westbound link (scaled from drawing), then 250 mm square. */
    {7,0,.300f,1.2}, 
    /* CROSS to first corner; two inner sides; last corner to CROSS. */
    {7,1,.250f-R,1.2}, {7,1,Q,V,1}, {7,1,.250f-2*R,V}, {7,1,Q,V,1}, {7,1,.250f-2*R,V}, {7,1,Q,V,1}, {7,1,.250f-R,1.2},
    /* S8: south, 30-degree deflection, diagonal approach, whole island. */
    {8,0,.460f-.267949f*R,1.2}, {8,0,.523599f*R,V,1},
    {8,0,.225f-.267949f*R,1.2}, {8,0,1.200f,0.9,0,ELEMENT_ISLAND},
    /* S9: eastbound link, two 60-degree turns, then entire figure eight. */
    {9,0,.225f-T,1.2}, {9,0,1.047198f*R,V,1},
    {9,0,.225f-2*T,1.2}, {9,0,1.047198f*R,V,1},
    {9,0,.590f-T,1.2}, {9,0,3.000f,0.9,0,ELEMENT_ISLAND}, /* 300 mm large + 200 mm small hexagon */
    /* S10: east, south, upper zigzag westbound, south, east to finish stripe. */
    {10,0,.530f-R,1.0}, {10,0,Q,V,1}, {10,0,.550f-2*R,V}, {10,0,Q,V,1},
    {10,0,.240f-R-.414214f*R,V},          /* westbound approach */
    {10,0,.785398f*R,0.75,1},                /* zigzag entry: 135 included, 45 yaw */
    {10,0,6*.21213f-2*.414214f*R,0.75,5},    /* five internal right angles */
    {10,0,.785398f*R,0.75,1},                /* zigzag exit: 135 included, 45 yaw */
    {10,0,.490f-R-.414214f*R,V},          /* southbound exit straight */
    {10,0,Q,V,1}, {10,0,.350f-R,1.5}
};
#undef V
#undef R
#undef Q
#undef T
#endif
#endif
