// TI File $Revision: /main/2 $
// Checkin $Date: July 31, 2009   09:55:57 $
//###########################################################################
//
// FILE:	Example_posspeed.h
//
// TITLE:	Pos/speed measurement using EQEP peripheral
//
// DESCRIPTION:
//
// Header file containing data type and object definitions and
// initializers.
//
//###########################################################################
// Original Author: SD
//
// $TI Release: DSP2834x C/C++ Header Files V1.10 $
// $Release Date: July 27, 2009 $
//###########################################################################

#ifndef __POSSPEED__
#define __POSSPEED__

#include "IQmathLib.h"         // Include header for IQmath library



/*-----------------------------------------------------------------------------
Define the structure of the POSSPEED Object
-----------------------------------------------------------------------------*/

// Smartec code ---------------

// **note: Q24 is Global IQ
// 10000 펄스 * 4채배  엔코더에서는 theta_mech 가 IQ15 에서는 정밀도가 작음 그래서 IQ 24로 한다. 
typedef struct {
				int32 	theta_elec;     		// Output: Motor Electrical angle (Q24)  
				int32 	theta_mech;     		// Output: Motor Mechanical Angle (Q24) 		
				int 	DirectionQep;      		// Output: Motor rotation direction (Q0) 
				int 	QEP_cnt_idx; 	 		// Variable: Encoder counter index (Q0)
				int32 	theta_raw;     			// Variable: Raw angle from Timer 2 (Q0)            
				int32 	mech_scaler;    		// Parameter: (2*PI)/(ENCODER_PULS_4_MULT), (Global IQ24 )                      <----------- IQ24
				int 	pole_pairs;     		// Parameter: Number of pole pairs (Q0) 
				int32 	cal_angle;     			// Parameter: Raw angular offset between encoder and phase a (Q0) 
				int 	index_sync_flag; 		// Output: Index sync status (Q0)                   

				Uint32 	SpeedScaler;     		// Parameter :  Scaler converting 1/N cycles to a GLOBAL_Q speed (Q0) - independently with global Q
				int32 	Speed_pr;          	 	// Output :  speed in per-unit
				Uint32 	BaseRpm;         		// Parameter : Scaler converting GLOBAL_Q speed to rpm (Q0) speed - independently with global Q
				int32 	SpeedRpm_pr;      		// Output : speed in r.p.m. (Q0) - independently with global Q                               

				int32  	oldpos;  				// Input: Electrical angle (pu) 
				int32 	Speed_fr;           	// Output :  speed in per-unit
				int32 	SpeedRpm_fr;     		// Output : Speed in rpm  (Q0) - independently with global Q

				//_iq 	Sys_Angle;				// Output : System Mechanical Angle (Q24) 
				int32 	Sys_Angle;				// Output : System Mechanical Angle (Q24) 
				int32	CurTheta;				// variable: Motor Mechanical Angle (Q24)
				int32	PreTheta;				// variable: Motor Mechanical Angle (Q24)
				int32	Dtheta;					// variable: Motor Mechanical Angle (Q24)
				int32 	Mot_Rev;				// variable: Number of rev Mech Angle(w2*pi rad)  (Q24)
				

				
				void (*init)();     			// Pointer to the init funcion          
				void (*calc)();    				// Pointer to the calc funtion        
                }  POSSPEED;
/*
typedef struct {
				_iq 	theta_elec;     		// Output: Motor Electrical angle (Q24)  
				_iq 	theta_mech;     		// Output: Motor Mechanical Angle (Q24) 		
				int 	DirectionQep;      		// Output: Motor rotation direction (Q0) 
				int 	QEP_cnt_idx; 	 		// Variable: Encoder counter index (Q0)
				int32 	theta_raw;     			// Variable: Raw angle from Timer 2 (Q0)            
				_iq 	mech_scaler;    		// Parameter: (2*PI)/(ENCODER_PULS_4_MULT), (Global IQ24 )                      <----------- IQ24
				int 	pole_pairs;     		// Parameter: Number of pole pairs (Q0) 
				int32 	cal_angle;     			// Parameter: Raw angular offset between encoder and phase a (Q0) 
				int 	index_sync_flag; 		// Output: Index sync status (Q0)                   

				Uint32 	SpeedScaler;     		// Parameter :  Scaler converting 1/N cycles to a GLOBAL_Q speed (Q0) - independently with global Q
				_iq 	Speed_pr;          	 	// Output :  speed in per-unit
				Uint32 	BaseRpm;         		// Parameter : Scaler converting GLOBAL_Q speed to rpm (Q0) speed - independently with global Q
				int32 	SpeedRpm_pr;      		// Output : speed in r.p.m. (Q0) - independently with global Q                               

				_iq  	oldpos;  				// Input: Electrical angle (pu) 
				_iq 	Speed_fr;           	// Output :  speed in per-unit
				int32 	SpeedRpm_fr;     		// Output : Speed in rpm  (Q0) - independently with global Q

				//_iq 	Sys_Angle;				// Output : System Mechanical Angle (Q24) 
				int32 	Sys_Angle;				// Output : System Mechanical Angle (Q24) 
				_iq		CurTheta;				// variable: Motor Mechanical Angle (Q24)
				_iq		PreTheta;				// variable: Motor Mechanical Angle (Q24)
				_iq		Dtheta;					// variable: Motor Mechanical Angle (Q24)
				_iq 	Mot_Rev;				// variable: Number of rev Mech Angle(w2*pi rad)  (Q24)
				

				
				void (*init)();     			// Pointer to the init funcion          
				void (*calc)();    				// Pointer to the calc funtion        
                }  POSSPEED;
*/


/*	Original code -------------
typedef struct {int theta_elec;     	// Output: Motor Electrical angle (Q15)
                int theta_mech;     	// Output: Motor Mechanical Angle (Q15)
                int DirectionQep;      	// Output: Motor rotation direction (Q0)
                int QEP_cnt_idx; 	 	// Variable: Encoder counter index (Q0)
                int theta_raw;     		// Variable: Raw angle from Timer 2 (Q0)
                int mech_scaler;    	// Parameter: 0.9999/total count, total count = 4000 (Q26)
                int pole_pairs;     	// Parameter: Number of pole pairs (Q0)
                int cal_angle;     		// Parameter: Raw angular offset between encoder and phase a (Q0)
                int index_sync_flag; 	// Output: Index sync status (Q0)

                Uint32 SpeedScaler;     // Parameter :  Scaler converting 1/N cycles to a GLOBAL_Q speed (Q0) - independently with global Q
                _iq Speed_pr;           // Output :  speed in per-unit
                Uint32 BaseRpm;         // Parameter : Scaler converting GLOBAL_Q speed to rpm (Q0) speed - independently with global Q
                int32 SpeedRpm_pr;      // Output : speed in r.p.m. (Q0) - independently with global Q

                _iq  oldpos;  			// Input: Electrical angle (pu)
                _iq Speed_fr;           // Output :  speed in per-unit
                int32 SpeedRpm_fr;     	// Output : Speed in rpm  (Q0) - independently with global Q
                void (*init)();     	// Pointer to the init funcion
                void (*calc)();    		// Pointer to the calc funtion
                }  POSSPEED;
*/

/*-----------------------------------------------------------------------------
Define a POSSPEED_handle
-----------------------------------------------------------------------------*/
typedef POSSPEED *POSSPEED_handle;

/*-----------------------------------------------------------------------------
Default initializer for the POSSPEED Object.
-----------------------------------------------------------------------------*/

#if (CPU_FRQ_300MHZ)

// smartec code 
#define POSSPEED_DEFAULTS {0x0, 0x0,0x0,0x0,0x0,0.000025,2,0,0x0,\
	  13,0,3000,0,\
	  0,0,0,\
	  0.0,0.0, 0.0, 0.0, 0,\
	  (void (*)(long))POSSPEED_Init,\
	  (void (*)(long))POSSPEED_Calc }
/*
#define POSSPEED_DEFAULTS {0x0, 0x0,0x0,0x0,0x0,_IQ24(0.000025),2,0,0x0,\
	  13,0,3000,0,\
	  0,0,0,\
	  _IQ24(0.0),_IQ24(0.0), _IQ24(0.0), _IQ24(0.0), 0,\
	  (void (*)(long))POSSPEED_Init,\
	  (void (*)(long))POSSPEED_Calc }
*/


/* original --------------
  #define POSSPEED_DEFAULTS {0x0, 0x0,0x0,0x0,0x0,16776,2,0,0x0,\
        94,0,6000,0,\
        0,0,0,\
        (void (*)(long))POSSPEED_Init,\
        (void (*)(long))POSSPEED_Calc }
*/
#endif
#if (CPU_FRQ_250MHZ)
  #define POSSPEED_DEFAULTS {0x0, 0x0,0x0,0x0,0x0,16776,2,0,0x0,\
        78,0,6000,0,\
        0,0,0,\
        (void (*)(long))POSSPEED_Init,\
        (void (*)(long))POSSPEED_Calc }
#endif

#if (CPU_FRQ_200MHZ)
  #define POSSPEED_DEFAULTS {0x0, 0x0,0x0,0x0,0x0,16776,2,0,0x0,\
        63,0,6000,0,\
        0,0,0,\
        (void (*)(long))POSSPEED_Init,\
        (void (*)(long))POSSPEED_Calc }
#endif


/*-----------------------------------------------------------------------------
Prototypes for the functions in posspeed.c
-----------------------------------------------------------------------------*/
void POSSPEED_Init(void);
void POSSPEED_Calc(POSSPEED_handle);

#endif /*  __POSSPEED__ */




