#include <stdio.h>
#include <float.h>
#include "NuMicro.h"
#include "timer.h"
#include "test.h"
#include "delay.h"

static VECTOR3F test_range[5] = {0};

static DESIGN_RANGE design_range[]=
{
    {"ACT100/CR700", {13.2f, 12.99f, 14.0f}},
    {"IOP100", {20.0f, 18.89f, 20.0f}},
};

static float step_to_mm(int axis, int step){
    switch(axis)
    {
        case 0:
        case 2:
            return (float)step*2.0f/200.0f/16.0f;
        break;
        case 1:
            return (float)step*1.0f/200.0f*TAN_30DEG/16.0f;
        break;
    }
    return 0;
}

void set_driving_current(CURRENT_SETTING cur_set){
    for(int i=0;i<3;i++)
        select_current(cur_set, &(get_motor_ref(i)->activeI0State),
                                &(get_motor_ref(i)->activeI1State));
}

static float current_setting_to_percentage(CURRENT_SETTING current_setting){
    return 12.5*(int)current_setting;
}

#define IN_TOLERANCE(V, D, T) (((D)-(T))<(V)&&(V)<((D)+(T)))

static void print_test_summary()
{
    VECTOR3F average_range = {0, 0, 0},
             max_range     = {0, 0, 0},
             min_range     = {FLT_MAX, FLT_MAX, FLT_MAX};
    
    for(int i=0; i<5; i++)
    {
        // average range
        average_range.x+=test_range[i].x;
        average_range.y+=test_range[i].y;
        average_range.z+=test_range[i].z;
        
        // max range
        if(test_range[i].x > max_range.x) max_range.x=test_range[i].x;
        if(test_range[i].y > max_range.y) max_range.y=test_range[i].y;
        if(test_range[i].z > max_range.z) max_range.z=test_range[i].z;
        
        // min range
        if(test_range[i].x < min_range.x) min_range.x=test_range[i].x;
        if(test_range[i].y < min_range.y) min_range.y=test_range[i].y;
        if(test_range[i].z < min_range.z) min_range.z=test_range[i].z;
    }
    average_range.x/=5.0f;
    average_range.y/=5.0f;
    average_range.z/=5.0f;
    
    printf("======== TEST REPORT =======\n");
    
    printf("Range: [%.2f, %.2f, %.2f]\n", average_range.x,
                                          average_range.y,
                                          average_range.z);
    
    float range_tolerance = 2.0f;
    bool pass_range_tolernace_test=false;
    for(int d=0; d<sizeof(design_range)/sizeof(DESIGN_RANGE); d++)
    {
        if(IN_TOLERANCE(average_range.x, design_range[d].range.x, range_tolerance)
        && IN_TOLERANCE(average_range.y, design_range[d].range.y, range_tolerance)
        && IN_TOLERANCE(average_range.z, design_range[d].range.z, range_tolerance))
        {
            printf("Model: [%s]\n", design_range[d].model);
            printf("Error: [%.2f, %.2f, %.2f]\n",
                                    (design_range[d].range.x-average_range.x),
                                    (design_range[d].range.y-average_range.y),
                                    (design_range[d].range.z-average_range.z));
            printf("[PASSED] tolerance test\n");
            pass_range_tolernace_test=true;
            break;
        }
    }
    if(!pass_range_tolernace_test){
        printf("[FAILED] unknown model by its range.\n");
    }
    
    bool pass_repeatability_test=false;
    float repeat_tolerance = 0.35;
    printf("MAX-min: [%.2f, %.2f, %.2f]\n", (max_range.x-min_range.x),
                                            (max_range.y-min_range.y),
                                            (max_range.z-min_range.z));
    if((max_range.x-min_range.x)<repeat_tolerance
    && (max_range.y-min_range.y)<repeat_tolerance
    && (max_range.z-min_range.z)<repeat_tolerance)
    {
        pass_repeatability_test=true;
        printf("[PASSED] repeatability test\n");
    }
    else
        printf("[FAILED] repeatability test\n");
    
    printf("---------- SUMMARY ---------\n");
    if(pass_range_tolernace_test&&pass_repeatability_test)
        printf("[PASSED]\n");
    else
        printf("[FAILED]\n");
    printf("==== END OF TEST REPORT ====\n");
}

int burn_in_loop(int count, CURRENT_SETTING current_setting){
    
    //set driving current
    set_driving_current(current_setting);
    printf("test loop %d started with %.1f%% torque\n", count, 
                                current_setting_to_percentage(current_setting));
    
    //for each axis
    for(int i=0; i<3; i++)
    {
        //go home
        motor_home(i);
        uDelay(3000);
        while (*(get_motor_ref(i)->busyPin.datAddr)){
            __NOP();
        }
        get_motor_ref(i)->position=0;
        for (int i = 0; i<500; i++)
            uDelay(3000);
        
        //move to far end
        motor_move_abspos(i, get_motor_ref(i)->maxHomeStep);
        uDelay(3000);
        while (*(get_motor_ref(i)->busyPin.datAddr)){
            __NOP();
        }
        
        //calculate length in mm
        float length = step_to_mm(i, get_motor_ref(i)->position);
        
        //an evil pointer hack to an access struct element by its index.
        ((float*)(&(test_range[count%5])))[i] = length;
        
        //print out
        printf("axis(%d): travel steps = %d \n", i, get_motor_ref(i)->position);
        printf("axis(%d): travel length = %f mm\n", i, length);
        
        for (int i = 0; i<500; i++)
            uDelay(3000);
    }
    
    printf("test loop %d finished\n", count);
    
    if((count%5)==4) //QC report
        print_test_summary();
    
    return count+1;
}