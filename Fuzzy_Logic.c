#define S_FUNCTION_LEVEL 2
#define S_FUNCTION_NAME Fuzzy_Logic  // s-Function Block Name
#include "simstruc.h"
#include <math.h>

#define U(element) (*uPtrs[element]) /* Pointer to Input Port0 */

static void mdlInitializeSizes(SimStruct *S) {
    ssSetNumDiscStates(S, 5);

    if (!ssSetNumInputPorts(S, 1)) return;
    ssSetInputPortWidth(S, 0, 2);   // Number of Inputs
    ssSetInputPortDirectFeedThrough(S, 0, 1);
    ssSetInputPortOverWritable(S, 0, 1);

    if (!ssSetNumOutputPorts(S, 1)) return;
    ssSetOutputPortWidth(S, 0, 4);  // Number of Outputs
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
    Y[2] = X[0];
    Y[3] = X[4];

}

#define MDL_UPDATE

static void mdlUpdate(SimStruct *S, int_T tid) {
    real_T *X = ssGetRealDiscStates(S);
    InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S,0);

    real_T dt = 1e-3;

    // Inputs
    real_T w_ref = U(0);
    real_T theta_m = U(1);
    real_T theta_old = X[3];

    real_T w_act = (theta_m - theta_old)/dt;
    real_T error_prev = X[0];
    
    real_T error_scaling = 10;
    real_T change_error_scaling = 10;

    // Fuzzy Logic Variables
    real_T reg_NL, reg_NM, reg_NS, reg_Z, reg_PS, reg_PM, reg_PL;
    
    real_T err_MF1, err_MF1_DOM, err_MF2, err_MF2_DOM; 
    real_T err_MF1_1, err_MF1_2, err_MF2_1, err_MF2_2;
    
    real_T ce_MF1, ce_MF1_DOM, ce_MF2, ce_MF2_DOM; 
    real_T ce_MF1_1, ce_MF1_2, ce_MF2_1, ce_MF2_2;

    real_T delta_Kp ,delta_Ki;

    // Error    
    real_T error = w_ref - w_act;
    real_T error_out = error;

    // Change of error
    real_T change_error = (error - error_prev);
    real_T change_error_out = change_error;

    // Normalization
    error = (error - error_scaling)/error_scaling;
    change_error = (change_error - change_error_scaling)/change_error_scaling;

    //DEGREE OF MEMBERSHIP 
    
    // Degree Of Membership ERROR
    reg_NL 	= -0.75*error_scaling;
    reg_NM = -0.5 * error_scaling;
    reg_NS =  -0.25*error_scaling;
	reg_Z 	= 0.00 *error_scaling;
    reg_PS = 0.25*error_scaling;
    reg_PM = 0.5 * error_scaling;
	reg_PL 	= 0.75*error_scaling;

    // FUZZYFICATION of Errors
    if (error <= reg_NL){
        err_MF1 = reg_NL;  // Lower NL Upper NL
        err_MF2 = reg_NL;  // Lower NL Upper NL
        err_MF1_DOM = 1;
        err_MF2_DOM = 1;

    }
    else if (error >= reg_PL){
        err_MF1 = reg_PL;  // Lower PL Upper PL
        err_MF2 = reg_PL;  // Lower PL Upper PL
        err_MF1_DOM = 1;
        err_MF2_DOM = 1;
    }
    else if (error == reg_NM){
        err_MF1 = reg_NM;  // Lower NM Upper NM
        err_MF2 = reg_NM;  // Lower NM Upper NM
        err_MF1_DOM = 1;
        err_MF2_DOM = 1;
    }
    else if (error == reg_NS){
        err_MF1 = reg_NS;  // Lower NS Upper NS
        err_MF2 = reg_NS;  // Lower NS Upper NS
        err_MF1_DOM = 1;
        err_MF2_DOM = 1;
    }
    else if (error == reg_Z){
        err_MF1 = reg_Z;  // Lower Z Upper Z
        err_MF2 = reg_Z;  // Lower Z Upper Z
        err_MF1_DOM = 1;
        err_MF2_DOM = 1;
    }
    else if (error == reg_PS){
        err_MF1 = reg_PS;  // Lower PS Upper PS
        err_MF2 = reg_PS;  // Lower PS Upper PS
        err_MF1_DOM = 1;
        err_MF2_DOM = 1;
    }
    else if (error == reg_PM){
        err_MF1 = reg_PM;  // Lower PM Upper PM
        err_MF2 = reg_PM;  // Lower PM Upper PM
        err_MF1_DOM = 1;
        err_MF2_DOM = 1;
    }
    else if (reg_NL < error && error < reg_Z){
        if (reg_NL < error && error < reg_NM){
            err_MF1 = reg_NL; // Lower NL Upper NL
            err_MF2 = reg_NM; // Lower NL Upper NS
            err_MF1_DOM = (reg_NM - error)/(reg_NM - reg_NL);            
            
            err_MF2_1 = (error-reg_NL)/(reg_NM - reg_NL);
            err_MF2_2 = (reg_NS-error)/(reg_NS-reg_NM);
            if (err_MF2_1 > err_MF2_2) err_MF2_DOM = err_MF2_2;
            else if (err_MF2_1 < err_MF2_2) err_MF2_DOM = err_MF2_1;
            if (err_MF2_DOM > 0) err_MF2_DOM = err_MF2_DOM;
            else if (err_MF2_DOM < 0) err_MF2_DOM = 0; 
        }
        else if (reg_NM < error && error < reg_NS){
            err_MF1 = reg_NM;  // Lower NL Upper NS
            err_MF2 = reg_NS;  // Lower NM Upper Z

            err_MF1_1 = (error-reg_NL)/(reg_NM - reg_NL);
            err_MF1_2 = (reg_NS-reg_NL)/(reg_NS-reg_NM); 

            err_MF2_1 = (error-reg_NM)/(reg_NS - reg_NM);
            err_MF2_2 = (reg_Z-error)/(reg_Z-reg_NS);

            if (err_MF1_1 > err_MF1_2) err_MF1_DOM = err_MF1_2;
            else if (err_MF1_1 < err_MF1_2) err_MF1_DOM = err_MF1_1;
            if (err_MF1_DOM > 0) err_MF1_DOM = err_MF1_DOM;
            else if (err_MF1_DOM < 0) err_MF1_DOM = 0; 

            if (err_MF2_1 > err_MF2_2) err_MF2_DOM = err_MF2_2;
            else if (err_MF2_1 < err_MF2_2) err_MF2_DOM = err_MF2_1;
            if (err_MF2_DOM > 0) err_MF2_DOM = err_MF2_DOM;
            else if (err_MF2_DOM < 0) err_MF2 = 0; 
        }
        else if (reg_NS < error && error < reg_Z){
            err_MF1 = reg_NS;  // Lower NM Upper Z
            err_MF2 = reg_Z;   // Lower NS Upper PS

            err_MF1_1 = (error-reg_NM)/(reg_NS - reg_NM);
            err_MF1_2 = (reg_Z-error)/(reg_Z-reg_NS);

            err_MF2_1 = (error-reg_NS)/(reg_Z - reg_NS);
            err_MF2_2 = (reg_PS-error)/(reg_PS-reg_Z);

            if (err_MF1_1 > err_MF1_2) err_MF1_DOM = err_MF1_2;
            else if (err_MF1_1 < err_MF1_2) err_MF1_DOM = err_MF1_1;
            if (err_MF1_DOM > 0) err_MF1_DOM = err_MF1_DOM;
            else if (err_MF1_DOM < 0) err_MF1_DOM = 0; 

            if (err_MF2_1 > err_MF2_2) err_MF2_DOM = err_MF2_2;
            else if (err_MF2_1 < err_MF2_2) err_MF2_DOM = err_MF2_1;
            if (err_MF2_DOM > 0) err_MF2_DOM = err_MF2_DOM;
            else if (err_MF2_DOM < 0) err_MF2_DOM = 0; 
        }
    }
    else if (reg_Z < error && error < reg_PL){
        if (reg_Z < error && error < reg_NS){
            err_MF1 = reg_Z;  // Lower NS Upper PS
            err_MF2 = reg_PS;   // Lower Z Upper PM
            
            err_MF1_1 = (error-reg_NS)/(reg_Z - reg_NS);
            err_MF1_2 = (reg_PS-error)/(reg_PS-reg_Z);

            err_MF2_1 = (error-reg_Z)/(reg_PS - reg_Z);
            err_MF2_2 = (reg_PM-error)/(reg_PM-reg_PS); 


            if (err_MF1_1 > err_MF1_2) err_MF1_DOM = err_MF1_2;
            else if (err_MF1_1 < err_MF1_2) err_MF1_DOM = err_MF1_1;
            if (err_MF1_DOM > 0) err_MF1_DOM = err_MF1_DOM;
            else if (err_MF1_DOM < 0) err_MF1_DOM = 0; 

            if (err_MF2_1 > err_MF2_2) err_MF2_DOM = err_MF2_2;
            else if (err_MF2_1 < err_MF2_2) err_MF2_DOM = err_MF2_1;
            if (err_MF2_DOM > 0) err_MF2_DOM = err_MF2_DOM;
            else if (err_MF2_DOM < 0) err_MF2_DOM = 0; 
        }
        else if (reg_NS < error && error < reg_PM){
            err_MF1 = reg_PS;  // Lower Z Upper PM
            err_MF2 = reg_PM;   // Lower PS Upper PL
            
            err_MF1_1 = (error-reg_Z)/(reg_PS - reg_Z);
            err_MF1_2 = (reg_PM-error)/(reg_PM-reg_PS); 

            err_MF2_1 = (error-reg_PS)/(reg_PM - reg_PS);
            err_MF2_2 = (reg_PL-error)/(reg_PL-reg_PM); 


            if (err_MF1_1 > err_MF1_2) err_MF1_DOM = err_MF1_2;
            else if (err_MF1_1 < err_MF1_2) err_MF1_DOM = err_MF1_1;
            if (err_MF1_DOM > 0) err_MF1_DOM = err_MF1_DOM;
            else if (err_MF1_DOM < 0) err_MF1_DOM = 0; 

            if (err_MF2_1 > err_MF2_2) err_MF2_DOM = err_MF2_2;
            else if (err_MF2_1 < err_MF2_2) err_MF2_DOM = err_MF2_1;
            if (err_MF2_DOM > 0) err_MF2_DOM = err_MF2_DOM;
            else if (err_MF2_DOM < 0) err_MF2_DOM = 0; 
        }
        else if (reg_PM < error < reg_PL){
            err_MF1 = reg_PM;  // Lower PS Upper PL
            err_MF2 = reg_PL;  // Lower PL Upper PL
            
            err_MF1_1 = (error-reg_PS)/(reg_PM - reg_PS);
            err_MF1_2 = (reg_PL-error)/(reg_PL-reg_PM); 

            err_MF2_DOM = (reg_PL - error)/(reg_PL - reg_PM);


            if (err_MF1_1 > err_MF1_2) err_MF1_DOM = err_MF1_2;
            else if (err_MF1_1 < err_MF1_2) err_MF1_DOM = err_MF1_1;
            if (err_MF1_DOM > 0) err_MF1_DOM = err_MF1_DOM;
            else if (err_MF1_DOM < 0) err_MF1_DOM = 0; 
            }  
    }


// FUZZYFICATION of Change of Errors

// Degree Of Membership CHANGE OF ERROR
    reg_NL 	= -0.75*change_error_scaling;
    reg_NM = -0.5 * change_error_scaling;
    reg_NS =  -0.25*change_error_scaling;
	reg_Z 	= 0.00 *change_error_scaling;
    reg_PS = 0.25*change_error_scaling;
    reg_PM = 0.5 * change_error_scaling;
	reg_PL 	= 0.75*change_error_scaling;

    if (change_error <= reg_NL){
        ce_MF1 = reg_NL;  // Lower NL Upper NL
        ce_MF2 = reg_NL;  // Lower NL Upper NL
        ce_MF1_DOM = 1;
        ce_MF2_DOM = 1;
    }
    else if (change_error >= reg_PL){
        ce_MF1 = reg_PL;  // Lower PL Upper PL
        ce_MF2 = reg_PL;  // Lower PL Upper PL
        ce_MF1_DOM = 1;
        ce_MF2_DOM = 1;
    }
    else if (change_error == reg_NM){
        ce_MF1 = reg_NM;  // Lower NM Upper NM
        ce_MF2 = reg_NM;  // Lower NM Upper NM
        ce_MF1_DOM = 1;
        ce_MF2_DOM = 1;
    }
    else if (change_error == reg_NS){
        ce_MF1 = reg_NS;  // Lower NS Upper NS
        ce_MF2 = reg_NS;  // Lower NS Upper NS
        ce_MF1_DOM = 1;
        ce_MF2_DOM = 1;
    }
    else if (change_error == reg_Z){
        ce_MF1 = reg_Z;  // Lower Z Upper Z
        ce_MF2 = reg_Z;  // Lower Z Upper Z
        ce_MF1_DOM = 1;
        ce_MF2_DOM = 1;
    }
    else if (change_error == reg_PS){
        ce_MF1 = reg_PS;  // Lower PS Upper PS
        ce_MF2 = reg_PS;  // Lower PS Upper PS
        ce_MF1_DOM = 1;
        ce_MF2_DOM = 1;
    }
    else if (change_error == reg_PM){
        ce_MF1 = reg_PM;  // Lower PM Upper PM
        ce_MF2 = reg_PM;  // Lower PM Upper PM
        ce_MF1_DOM = 1;
        ce_MF2_DOM = 1;
    }
    else if (reg_NL < change_error && change_error < reg_Z){
        if (reg_NL < change_error && change_error < reg_NM){
            ce_MF1 = reg_NL; // Lower NL Upper NL
            ce_MF2 = reg_NM; // Lower NL Upper NS
            ce_MF1_DOM = (reg_NM - change_error)/(reg_NM - reg_NL);            
            
            ce_MF2_1 = (change_error-reg_NL)/(reg_NM - reg_NL);
            ce_MF2_2 = (reg_NS-change_error)/(reg_NS-reg_NM);
            if (ce_MF2_1 > ce_MF2_2) ce_MF2_DOM = ce_MF2_2;
            else if (ce_MF2_1 < ce_MF2_2) ce_MF2_DOM = ce_MF2_1;
            if (ce_MF2_DOM > 0) ce_MF2_DOM = ce_MF2_DOM;
            else if (ce_MF2_DOM < 0) ce_MF2_DOM = 0; 
        }
        else if (reg_NM < change_error && change_error < reg_NS){
            ce_MF1 = reg_NM;  // Lower NL Upper NS
            ce_MF2 = reg_NS;  // Lower NM Upper Z

            ce_MF1_1 = (change_error-reg_NL)/(reg_NM - reg_NL);
            ce_MF1_2 = (reg_NS-reg_NL)/(reg_NS-reg_NM); 

            ce_MF2_1 = (change_error-reg_NM)/(reg_NS - reg_NM);
            ce_MF2_2 = (reg_Z-change_error)/(reg_Z-reg_NS);

            if (ce_MF1_1 > ce_MF1_2) ce_MF1_DOM = ce_MF1_2;
            else if (ce_MF1_1 < ce_MF1_2) ce_MF1_DOM = ce_MF1_1;
            if (ce_MF1_DOM > 0) ce_MF1_DOM = ce_MF1_DOM;
            else if (ce_MF1_DOM < 0) ce_MF1_DOM = 0; 

            if (ce_MF2_1 > ce_MF2_2) ce_MF2_DOM = ce_MF2_2;
            else if (ce_MF2_1 < ce_MF2_2) ce_MF2_DOM = ce_MF2_1;
            if (ce_MF2_DOM > 0) ce_MF2_DOM = ce_MF2_DOM;
            else if (ce_MF2_DOM < 0) ce_MF2 = 0; 
        }
        else if (reg_NS < change_error && change_error < reg_Z){
            ce_MF1 = reg_NS;  // Lower NM Upper Z
            ce_MF2 = reg_Z;   // Lower NS Upper PS

            ce_MF1_1 = (change_error-reg_NM)/(reg_NS - reg_NM);
            ce_MF1_2 = (reg_Z-change_error)/(reg_Z-reg_NS); 

            ce_MF2_1 = (change_error-reg_NS)/(reg_Z - reg_NS);
            ce_MF2_2 = (reg_PS-change_error)/(reg_PS-reg_Z);

            if (ce_MF1_1 > ce_MF1_2) ce_MF1_DOM = ce_MF1_2;
            else if (ce_MF1_1 < ce_MF1_2) ce_MF1_DOM = ce_MF1_1;
            if (ce_MF1_DOM > 0) ce_MF1_DOM = ce_MF1_DOM;
            else if (ce_MF1_DOM < 0) ce_MF1_DOM = 0; 

            if (ce_MF2_1 > ce_MF2_2) ce_MF2_DOM = ce_MF2_2;
            else if (ce_MF2_1 < ce_MF2_2) ce_MF2_DOM = ce_MF2_1;
            if (ce_MF2_DOM > 0) ce_MF2_DOM = ce_MF2_DOM;
            else if (ce_MF2_DOM < 0) ce_MF2_DOM = 0; 
        }

    }
    else if (reg_Z < change_error && change_error < reg_PL){
        if (reg_Z < change_error && change_error < reg_NS){
            ce_MF1 = reg_Z;  // Lower NS Upper PS
            ce_MF2 = reg_PS;   // Lower Z Upper PM
            
            ce_MF1_1 = (change_error-reg_NS)/(reg_Z - reg_NS);
            ce_MF1_2 = (reg_PS-change_error)/(reg_PS-reg_Z);

            ce_MF2_1 = (change_error-reg_Z)/(reg_PS - reg_Z);
            ce_MF2_2 = (reg_PM-change_error)/(reg_PM-reg_PS); 


            if (ce_MF1_1 > ce_MF1_2) ce_MF1_DOM = ce_MF1_2;
            else if (ce_MF1_1 < ce_MF1_2) ce_MF1_DOM = ce_MF1_1;
            if (ce_MF1_DOM > 0) ce_MF1_DOM = ce_MF1_DOM;
            else if (ce_MF1_DOM < 0) ce_MF1_DOM = 0; 

            if (ce_MF2_1 > ce_MF2_2) ce_MF2_DOM = ce_MF2_2;
            else if (ce_MF2_1 < ce_MF2_2) ce_MF2_DOM = ce_MF2_1;
            if (ce_MF2_DOM > 0) ce_MF2_DOM = ce_MF2_DOM;
            else if (ce_MF2_DOM < 0) ce_MF2_DOM = 0; 
        }
        else if (reg_NS < change_error && change_error < reg_PM){
            ce_MF1 = reg_PS;  // Lower Z Upper PM
            ce_MF2 = reg_PM;   // Lower PS Upper PL
            
            ce_MF1_1 = (change_error-reg_Z)/(reg_PS - reg_Z);
            ce_MF1_2 = (reg_PM-change_error)/(reg_PM-reg_PS); 

            ce_MF2_1 = (change_error-reg_PS)/(reg_PM - reg_PS);
            ce_MF2_2 = (reg_PL-change_error)/(reg_PL-reg_PM); 


            if (ce_MF1_1 > ce_MF1_2) ce_MF1_DOM = ce_MF1_2;
            else if (ce_MF1_1 < ce_MF1_2) ce_MF1_DOM = ce_MF1_1;
            if (ce_MF1_DOM > 0) ce_MF1_DOM = ce_MF1_DOM;
            else if (ce_MF1_DOM < 0) ce_MF1_DOM = 0; 

            if (ce_MF2_1 > ce_MF2_2) ce_MF2_DOM = ce_MF2_2;
            else if (ce_MF2_1 < ce_MF2_2) ce_MF2_DOM = ce_MF2_1;
            if (ce_MF2_DOM > 0) ce_MF2_DOM = ce_MF2_DOM;
            else if (ce_MF2_DOM < 0) ce_MF2_DOM = 0; 
        }
        else if (reg_PM < change_error && change_error < reg_PL){
            ce_MF1 = reg_PM;  // Lower PS Upper PL
            ce_MF2 = reg_PL;  // Lower PL Upper PL
            
            ce_MF1_1 = (change_error-reg_PS)/(reg_PM - reg_PS);
            ce_MF1_2 = (reg_PL-change_error)/(reg_PL-reg_PM); 

            ce_MF2_DOM = (reg_PL - change_error)/(reg_PL - reg_PM);

            if (ce_MF1_1 > ce_MF1_2) ce_MF1_DOM = ce_MF1_2;
            else if (ce_MF1_1 < ce_MF1_2) ce_MF1_DOM = ce_MF1_1;
            if (ce_MF1_DOM > 0) ce_MF1_DOM = ce_MF1_DOM;
            else if (ce_MF1_DOM < 0) ce_MF1_DOM = 0; 
            }  
    }

    real_T OMF_KP, OMF_KI, err_OMF_DOM, err_OMF, ce_OMF_DOM, ce_OMF;

    // Inference Engine 
    // Max Algorithm
    if (err_MF1_DOM > err_MF2_DOM){err_OMF = err_MF1; err_OMF_DOM = err_MF1_DOM;}
    else if (err_MF1_DOM < err_MF2_DOM){err_OMF = err_MF2; err_OMF_DOM = err_MF2_DOM;}
    else if (err_MF1_DOM == err_MF2_DOM){err_OMF = err_MF1; err_OMF_DOM = 1;}

    if (ce_MF1_DOM > ce_MF2_DOM){ce_OMF = ce_MF1; ce_OMF_DOM = ce_MF1_DOM;}
    else if (ce_MF1_DOM < ce_MF2_DOM){ce_OMF = ce_MF2; ce_OMF_DOM = ce_MF2_DOM;}
    else if (ce_MF1_DOM == ce_MF2_DOM){ce_OMF = ce_MF1; ce_OMF_DOM = 1;}
    
    // RULE BASE KP KI
    if (ce_OMF == reg_NL){
        if (err_OMF == reg_NL){OMF_KP = reg_PL; OMF_KI = reg_NL;}
        else if (err_OMF == reg_NM){OMF_KP = reg_PL; OMF_KI = reg_NL;}
        else if (err_OMF == reg_NS){OMF_KP = reg_PM; OMF_KI = reg_NL;}
        else if (err_OMF == reg_Z){OMF_KP = reg_PM; OMF_KI = reg_NM;}
        else if (err_OMF == reg_PS){OMF_KP = reg_PS; OMF_KI = reg_NM;}
        else if (err_OMF == reg_PM){OMF_KP = reg_PS; OMF_KI = reg_Z;}
        else if (err_OMF == reg_PL){OMF_KP = reg_Z; OMF_KI = reg_Z;}
    }
    else if (ce_OMF == reg_NM){
        if (err_OMF == reg_NL){OMF_KP = reg_PL; OMF_KI = reg_NL;}
        else if (err_OMF == reg_NM){OMF_KP = reg_PL; OMF_KI = reg_NL;}
        else if (err_OMF == reg_NS){OMF_KP = reg_PM; OMF_KI = reg_NM;}
        else if (err_OMF == reg_Z){OMF_KP = reg_PM; OMF_KI = reg_NM;}
        else if (err_OMF == reg_PS){OMF_KP = reg_PS; OMF_KI = reg_NS;}
        else if (err_OMF == reg_PM){OMF_KP = reg_Z; OMF_KI = reg_Z;}
        else if (err_OMF == reg_PL){OMF_KP = reg_Z; OMF_KI = reg_Z;}
    }
    else if (ce_OMF == reg_NS){
        if (err_OMF == reg_NL){OMF_KP = reg_PM; OMF_KI = reg_NM;}
        else if (err_OMF == reg_NM){OMF_KP = reg_PM; OMF_KI = reg_NM;}
        else if (err_OMF == reg_NS){OMF_KP = reg_PM; OMF_KI = reg_NS;}
        else if (err_OMF == reg_Z){OMF_KP = reg_PS; OMF_KI = reg_NS;}
        else if (err_OMF == reg_PS){OMF_KP = reg_Z; OMF_KI = reg_Z;}
        else if (err_OMF == reg_PM){OMF_KP = reg_NS; OMF_KI = reg_PS;}
        else if (err_OMF == reg_PL){OMF_KP = reg_NM; OMF_KI = reg_PS;}
    }
    else if (ce_OMF == reg_Z){
        if (err_OMF == reg_NL){OMF_KP = reg_PS; OMF_KI = reg_NS;}
        else if (err_OMF == reg_NM){OMF_KP = reg_PS; OMF_KI = reg_NS;}
        else if (err_OMF == reg_NS){OMF_KP = reg_PS; OMF_KI = reg_NS;}
        else if (err_OMF == reg_Z){OMF_KP = reg_Z; OMF_KI = reg_Z;}
        else if (err_OMF == reg_PS){OMF_KP = reg_NS; OMF_KI = reg_PS;}
        else if (err_OMF == reg_PM){OMF_KP = reg_NM; OMF_KI = reg_PS;}
        else if (err_OMF == reg_PL){OMF_KP = reg_NM; OMF_KI = reg_PM;}
    }
    else if (ce_OMF == reg_PS){
        if (err_OMF == reg_NL){OMF_KP = reg_PS; OMF_KI = reg_NS;}
        else if (err_OMF == reg_NM){OMF_KP = reg_PS; OMF_KI = reg_NS;}
        else if (err_OMF == reg_NS){OMF_KP = reg_Z; OMF_KI = reg_Z;}
        else if (err_OMF == reg_Z){OMF_KP = reg_NS; OMF_KI = reg_PS;}
        else if (err_OMF == reg_PS){OMF_KP = reg_NM; OMF_KI = reg_PS;}
        else if (err_OMF == reg_PM){OMF_KP = reg_NM; OMF_KI = reg_PM;}
        else if (err_OMF == reg_PL){OMF_KP = reg_NM; OMF_KI = reg_PM;}
    }
    else if (ce_OMF == reg_PM){
        if (err_OMF == reg_NL){OMF_KP = reg_Z; OMF_KI = reg_Z;}
        else if (err_OMF == reg_NM){OMF_KP = reg_Z; OMF_KI = reg_Z;}
        else if (err_OMF == reg_NS){OMF_KP = reg_NS; OMF_KI = reg_PS;}
        else if (err_OMF == reg_Z){OMF_KP = reg_NM; OMF_KI = reg_PM;}
        else if (err_OMF == reg_PS){OMF_KP = reg_NM; OMF_KI = reg_PM;}
        else if (err_OMF == reg_PM){OMF_KP = reg_NM; OMF_KI = reg_PL;}
        else if (err_OMF == reg_PL){OMF_KP = reg_NL; OMF_KI = reg_PL;}
    }
    else if (ce_OMF == reg_PL){
        if (err_OMF == reg_NL){OMF_KP = reg_Z; OMF_KI = reg_Z;}
        else if (err_OMF == reg_NM){OMF_KP = reg_NS; OMF_KI = reg_Z;}
        else if (err_OMF == reg_NS){OMF_KP = reg_NS; OMF_KI = reg_PS;}
        else if (err_OMF == reg_Z){OMF_KP = reg_NM; OMF_KI = reg_PM;}
        else if (err_OMF == reg_PS){OMF_KP = reg_NM; OMF_KI = reg_PL;}
        else if (err_OMF == reg_PM){OMF_KP = reg_NL; OMF_KI = reg_PL;}
        else if (err_OMF == reg_PL){OMF_KP = reg_NL; OMF_KI = reg_PL;}
    }
    
    //real_T out_scale = err_OMF_DOM * 0.45 + ce_OMF_DOM * 0.55;
    if (OMF_KP == reg_NL){
        delta_Kp = -3;}
    else if (OMF_KP == reg_NM){delta_Kp = -2;}
    else if (OMF_KP == reg_NS){delta_Kp = -1;}
    else if (OMF_KP == reg_Z){delta_Kp = 0;}
    else if (OMF_KP == reg_PS){delta_Kp = 1;}
    else if (OMF_KP == reg_PM){delta_Kp = 2;}
    else if (OMF_KP == reg_NL){delta_Kp = 3;}
    if (OMF_KI == reg_NL){
        delta_Ki = -3;}
    else if (OMF_KI == reg_NM){delta_Ki = -2;}
    else if (OMF_KI == reg_NS){delta_Ki = -1;}
    else if (OMF_KI == reg_Z){delta_Ki = 0;}
    else if (OMF_KI == reg_PS){delta_Ki = 1;}
    else if (OMF_KI == reg_PM){delta_Ki = 2;}
    else if (OMF_KI == reg_NL){delta_Ki = 3;}

    X[0] = error_out;
    X[1] = delta_Kp;
    X[2] = delta_Ki;
    X[3] = theta_m;
    X[4] = change_error_out;

}


static void mdlTerminate(SimStruct *S) { } /* Keep this function empty since no memory is allocated */ 

#ifdef MATLAB_MEX_FILE
/* Is this file being compiled as a MEX-file? */
#include "simulink.c" /*MEX-file interface mechanism*/
#else
#include "cg_sfun.h" /*Code generation registration function*/
#endif

