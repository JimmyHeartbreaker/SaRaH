#include "arm_math.h"

namespace Samples::Filter
{
    #define BLOCK_SIZE     720  
    #define NUM_STAGE_IIR  3
    #define NUM_ORDER_IIR  (NUM_STAGE_IIR * 2)
    #define NUM_STD_COEFFS 2 // b0, b1, b2, a1, a2

    static float scaler[NUM_STAGE_IIR]=
    {                                                   
 0.079377897083759307861328125  *5.3                                 ,       
    0.064816884696483612060546875       *5.3                         ,
    0.0586096234619617462158203125      *5.3                       
    };

    static float iirState[NUM_ORDER_IIR];

    static float iirCoeffs[NUM_STAGE_IIR * (NUM_STD_COEFFS*2+1)] = 
    {
        scaler[0], 2*scaler[0], scaler[0], -0.4556631743907928466796875,    0.09316779673099517822265625                                       ,          
        scaler[1], 2*scaler[1], scaler[1], -0.587421238422393798828125,     0.1616139113903045654296875                                   ,            
        scaler[2], 2*scaler[2], scaler[2], -0.70819938182830810546875     ,    0.23030115664005279541015625                                                                    
    };

    static arm_biquad_cascade_df2T_instance_f32 S1k;
    bool firstRun = true;
    void FilterSamples( float* inbuf) 
    { 
        //pre warm
        if(firstRun)
        {
            float prewarm[64];
            arm_biquad_cascade_df2T_f32(&S1k, inbuf+BLOCK_SIZE-65,prewarm , 64);    
            firstRun = false;
        }
        
        arm_biquad_cascade_df2T_f32(&S1k, inbuf,inbuf , BLOCK_SIZE);    
       
    }

    void SetupFilter()
    {    
        arm_biquad_cascade_df2T_init_f32(&S1k, NUM_STAGE_IIR, iirCoeffs, iirState);
    }
}