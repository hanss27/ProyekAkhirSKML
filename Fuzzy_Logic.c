#define S_FUNCTION_LEVEL 2
#define S_FUNCTION_NAME Fuzzy_Logic  // s-Function Block Name
#include "simstruc.h"
#include <math.h>

#define U(element) (*uPtrs[element]) /* Pointer to Input Port0 */

static void mdlInitializeSizes(SimStruct *S) {
    ssSetNumDiscStates(S, 3);

    if (!ssSetNumInputPorts(S, 1)) return;
    ssSetInputPortWidth(S, 0, 2);   // Number of Inputs
    ssSetInputPortDirectFeedThrough(S, 0, 1);
    ssSetInputPortOverWritable(S, 0, 1);

    if (!ssSetNumOutputPorts(S, 1)) return;
    ssSetOutputPortWidth(S, 0, 2);  // Number of Outputs
    ssSetNumSampleTimes(S, 1);

    ssSetOptions(S, (SS_OPTION_EXCEPTION_FREE_CODE
    | SS_OPTION_DISCRETE_VALUED_OUTPUT));
} 

static void mdlInitializeSampleTimes(SimStruct *S) {
    ssSetSampleTime(S, 0, 1e-4);    // Continuous: CONTINUOUS_SAMPLE_TIME OR 1e-3|| Discrete: 0.001
    ssSetOffsetTime(S, 0, 0.0);
}

#define MDL_INITIALIZE_CONDITIONS 

static void mdlInitializeConditions(SimStruct *S) {
    real_T *X0 = ssGetRealDiscStates(S);
    int_T nXStates = ssGetNumDiscStates(S);
    InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S,0);
    int_T i;

    /* Initialize the states to 0.0 */
    for (i=0; i < nXStates; i++) X0[i] = 0.0;
}

static void mdlOutputs(SimStruct *S, int_T tid) {
    real_T *Y = ssGetOutputPortRealSignal(S,0);
    real_T *X = ssGetRealDiscStates(S);

    real_T delta_Kp = X[1];
    real_T delta_Ki = X[2];
    Y[0] = delta_Kp;
    Y[1] = delta_Ki;

}

#define MDL_UPDATE

static void mdlUpdate(SimStruct *S, int_T tid) {
    real_T *X = ssGetRealDiscStates(S);
    InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S,0);

    real_T dt = 1e-4;

    // Inputs
    real_T w_ref = U(0);
    real_T theta_m = U(1);
    
    real_T theta_old = X[3];

    // Errors
    real_T w_act = (theta_m - theta_old)/dt;
    real_T error = w_ref - w_act;
    real_T error_prev = X[0];
    
    real_T error_scaling = 10;
    real_T change_error_scaling = 20;

    // Fuzzy Logic Variables
    real_T reg_ned_large, reg_neg_med, reg_neg_short, reg_zero, reg_pos_short, reg_pos_med, reg_pos_large;
    real_T err_pos_large, err_pos_med, err_pos_short, err_zero, err_neg_short, err_neg_med, err_neg_large;
    real_T ce_pos_large, ce_pos_med, ce_pos_short, ce_zero, ce_neg_short, ce_neg_med, ce_neg_large;

    real_T delta_Kp,delta_Ki;
        

    // Change of error
    real_T change_error = (error - error_prev);

    //DEGREE OF MEMBERSHIP 
    
    // Degree Of Membership ERROR
    reg_ned_large 	= 0.90*error_scaling;
    reg_neg_med = 0.60 * error_scaling;
    reg_neg_short =  0.30*error_scaling;
	reg_zero 	= 0.0*error_scaling;
    reg_pos_short = -0.30*error_scaling;
    reg_pos_med = -0.60 * error_scaling;
	 reg_pos_large 	= -0.90*error_scaling;

    if (error <=  reg_pos_large){
        err_neg_large = 1;
        err_neg_med = 0;
        err_neg_short = 0;
        err_zero = 0;
        err_pos_short = 0;
        err_pos_med = 0;
        err_pos_large = 0;
    }
    else if ( reg_pos_large < error && error < reg_zero){
        if ( reg_pos_large < error && error < reg_pos_med){
            err_neg_large = 0;
            err_neg_med =  1;
            err_neg_short = 0;
            err_zero = 0;
            err_pos_short = 0;
            err_pos_med = 0;
            err_pos_large = 0;  
        }
        else if (reg_pos_med <= error && error < reg_pos_short){
            err_neg_large = 0; 
            err_neg_med = 0;
            err_neg_short = 1;
            err_zero = 0;
            err_pos_short = 0;
            err_pos_med = 0;
            err_pos_large = 0;   
        }
        else if (reg_pos_short <= error && error < reg_zero){
            err_neg_large = 0;
            err_neg_med = 0;
            err_neg_short = 0;
            err_zero = 1;
            err_pos_short = 0;
            err_pos_med = 0;
            err_pos_large = 0;  
        }
    }
     else if (reg_zero <= error && error < reg_ned_large){
        if (reg_zero <= error && error < reg_neg_short){
            err_neg_large = 0;
            err_neg_med = 0;
            err_neg_short = 0;
            err_zero = 0;
            err_pos_short = 1;
            err_pos_med = 0;
            err_pos_large = 0;    
        }
        else if (reg_neg_short <= error && error < reg_neg_med){
            err_neg_large = 0;
            err_neg_med = 0;
            err_neg_short = 0;
            err_zero = 0;
            err_pos_short = 0;
            err_pos_med = 1;
            err_pos_large = 0;  
        }
        else if (reg_neg_med <= error && error < reg_ned_large){
            err_neg_large = 0;
            err_neg_med = 0;
            err_neg_short = 0;
            err_zero = 0;
            err_pos_short = 0;
            err_pos_med = 0;
            err_pos_large = 1;  
        }
    }
	else if (error >= reg_ned_large){
        err_neg_large = 0;
        err_neg_med = 0;
        err_neg_short = 0;
        err_zero = 0;
        err_pos_short = 0;
        err_pos_med = 0;
        err_pos_large = 1;  
    }

    // Degree of Membership CHANGE OF ERROR
    reg_ned_large 	= 0.75*change_error_scaling;
    reg_neg_short =  0.35*change_error_scaling;
    reg_zero 	= 0.0*change_error_scaling;
    reg_pos_short = -0.35*change_error_scaling;
    reg_pos_large 	= -0.75*change_error_scaling;

    if (change_error <=  reg_pos_large){
        ce_neg_large = 1;
        ce_neg_med = 0;
        ce_neg_short = 0;
        ce_zero = 0;
        ce_pos_short = 0;
        ce_pos_med = 0;
        ce_pos_large = 0;
    }
    else if ( reg_pos_large < change_error && change_error < reg_zero){
        if ( reg_pos_large < change_error && change_error < reg_pos_med){
            ce_neg_large = 0;
            ce_neg_med = 1;
            ce_neg_short = 0;
            ce_zero = 0;
            ce_pos_short = 0;
            ce_pos_med = 0;
            ce_pos_large = 0;  
        }
        else if (reg_pos_med <= change_error && change_error < reg_pos_short){
            ce_neg_large = 0;
            ce_neg_med = 0;
            ce_neg_short = 1;
            ce_zero = 0;
            ce_pos_short = 0;
            ce_pos_med = 0;
            ce_pos_large = 0;     
        }
        else if (reg_pos_short <= change_error  && change_error < reg_zero){
            ce_neg_large = 0;
            ce_neg_med = 0;
            ce_neg_short = 0;
            ce_zero = 1;
            ce_pos_short = 0;
            ce_pos_med = 0;
            ce_pos_large = 0;   
        }
    }
     else if (reg_zero <= change_error && change_error < reg_ned_large){
        if (reg_zero <= change_error  && change_error < reg_neg_short){
            ce_neg_large = 0;
            ce_neg_med = 0;
            ce_neg_short = 0;
            ce_zero = 1;
            ce_pos_short = 0;
            ce_pos_med = 0;
            ce_pos_large = 0;   
        }
        else if (reg_neg_short <= change_error  && change_error < reg_neg_med){
            ce_neg_large = 0;
            ce_neg_med = 0;
            ce_neg_short = 0;
            ce_zero = 0;
            ce_pos_short = 1;
            ce_pos_med = 0;
            ce_pos_large = 0;  
        }
        else if (reg_neg_med <= change_error  && change_error < reg_ned_large){
            ce_neg_large = 0;
            ce_neg_med = 0;
            ce_neg_short = 0;
            ce_zero = 0;
            ce_pos_short = 0;
            ce_pos_med = 1;
            ce_pos_large = 0;  
        }
    }
	else if (change_error >= reg_ned_large){
        ce_neg_large = 0;
            ce_neg_med = 0;
            ce_neg_short = 0;
            ce_zero = 0;
            ce_pos_short = 0;
            ce_pos_med = 0;
            ce_pos_large = 1;  
    }

    // RULE BASE

    // Kp & Ki Rule Base

    // Check the Index of maximum degree of membership from the error
    int err_arr[7] = {err_neg_large, err_neg_med, err_neg_short, err_zero, err_pos_short, err_pos_med, err_pos_large};
    int i, err_Index = 0;
    for (i = 0; i < 7; i ++){
        if (err_arr[i] > err_arr[err_Index]) {err_Index = i;}
    }

    // Check the Index of maximum degree of membership from the change of error
    int ce_arr[7] = {ce_neg_large, ce_neg_med, ce_neg_short, ce_zero, ce_pos_short, ce_pos_med, ce_pos_large};
    int ce_Index = 0;
    for (i = 0; i < 7; i ++){
        if (ce_arr[i] > ce_arr[ce_Index]) {ce_Index = i;}
    }

    if (err_Index == 0){
        if (ce_Index == 0){delta_Kp = 1.5; delta_Ki = -1.5;}
        else if (ce_Index == 1) {delta_Kp = 1.5; delta_Ki = -1.5;}
        else if (ce_Index == 2) {delta_Kp = 1; delta_Ki = -1.5;}
        else if (ce_Index == 3) {delta_Kp = 1; delta_Ki = -1;}
        else if (ce_Index == 4) {delta_Kp = 0.5; delta_Ki = -1;}
        else if (ce_Index == 5) {delta_Kp = 0.5; delta_Ki = 0;}
        else if (ce_Index == 6) {delta_Kp = 0; delta_Ki = 0;}
    }
    else if (err_Index == 1){
        if (ce_Index == 0){delta_Kp = 1.5; delta_Ki = -1.5;}
        else if (ce_Index == 1) {delta_Kp = 1.5; delta_Ki = -1.5;}
        else if (ce_Index == 2) {delta_Kp = 1; delta_Ki = -1;}
        else if (ce_Index == 3) {delta_Kp = 1; delta_Ki = -1;}
        else if (ce_Index == 4) {delta_Kp = 0.5; delta_Ki = -0.5;}
        else if (ce_Index == 5) {delta_Kp = 0; delta_Ki = 0;}
        else if (ce_Index == 6) {delta_Kp = 0; delta_Ki = 0;}
    }    

    else if (err_Index == 2){
        if (ce_Index == 0){delta_Kp = 1; delta_Ki = -1;}
        else if (ce_Index == 1) {delta_Kp = 1; delta_Ki = -1;}
        else if (ce_Index == 2) {delta_Kp = 1; delta_Ki = -0.5;}
        else if (ce_Index == 3) {delta_Kp = 0.5; delta_Ki = -0.5;}
        else if (ce_Index == 4) {delta_Kp = 0; delta_Ki = 0;}
        else if (ce_Index == 5) {delta_Kp = -0.5; delta_Ki = 0.5;}
        else if (ce_Index == 6) {delta_Kp = -1; delta_Ki = 0.5;}
    }    

    else if (err_Index == 3){
        if (ce_Index == 0){delta_Kp = 1; delta_Ki = -1;}
        else if (ce_Index == 1) {delta_Kp = 0.5; delta_Ki = -0.5;}
        else if (ce_Index == 2) {delta_Kp = 0.5; delta_Ki = -0.5;}
        else if (ce_Index == 3) {delta_Kp = 0; delta_Ki = 0;}
        else if (ce_Index == 4) {delta_Kp = -0.5; delta_Ki = 0.5;}
        else if (ce_Index == 5) {delta_Kp = -1; delta_Ki = 0.5;}
        else if (ce_Index == 6) {delta_Kp = -1; delta_Ki = 1;}
    }  

    else if (err_Index == 4){
        if (ce_Index == 0){delta_Kp = 0.5; delta_Ki = -0.5;}
        else if (ce_Index == 1) {delta_Kp = 0.5; delta_Ki = -0.5;}
        else if (ce_Index == 2) {delta_Kp = 0; delta_Ki = 0;}
        else if (ce_Index == 3) {delta_Kp = -0.5; delta_Ki = 0.5;}
        else if (ce_Index == 4) {delta_Kp = -1; delta_Ki = 0.5;}
        else if (ce_Index == 5) {delta_Kp = -1; delta_Ki = 1;}
        else if (ce_Index == 6) {delta_Kp = -1; delta_Ki = 1;}
    }      

    else if (err_Index == 5){
        if (ce_Index == 0){delta_Kp = 0; delta_Ki = 0;}
        else if (ce_Index == 1) {delta_Kp = 0; delta_Ki = 0;}
        else if (ce_Index == 2) {delta_Kp = -0.5; delta_Ki = 0.5;}
        else if (ce_Index == 3) {delta_Kp = -1; delta_Ki = 1;}
        else if (ce_Index == 4) {delta_Kp = -1; delta_Ki = 1;}
        else if (ce_Index == 5) {delta_Kp = -1; delta_Ki = 3;}
        else if (ce_Index == 6) {delta_Kp = -1.5; delta_Ki = 1.5;}
    }   

    else if (err_Index == 6){
        if (ce_Index == 0){delta_Kp = 0; delta_Ki = 0;}
        else if (ce_Index == 1) {delta_Kp = -0.5; delta_Ki = 0;}
        else if (ce_Index == 2) {delta_Kp = -0.5; delta_Ki = 0.5;}
        else if (ce_Index == 3) {delta_Kp = -1; delta_Ki = 1;}
        else if (ce_Index == 4) {delta_Kp = -1; delta_Ki = 1.5;}
        else if (ce_Index == 5) {delta_Kp = -1.5; delta_Ki = 1.5;}
        else if (ce_Index == 6) {delta_Kp = -1.5; delta_Ki = 1.5;}
    }     

    X[0] = error;
    X[1] = delta_Kp;
    X[2] = delta_Ki;
    X[3] = theta_m;

}


static void mdlTerminate(SimStruct *S) { } /* Keep this function empty since no memory is allocated */ 

#ifdef MATLAB_MEX_FILE
/* Is this file being compiled as a MEX-file? */
#include "simulink.c" /*MEX-file interface mechanism*/
#else
#include "cg_sfun.h" /*Code generation registration function*/
#endif

