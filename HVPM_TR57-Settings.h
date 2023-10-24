/* =================================================================================
File name  : HVPM_Sensorless-Settings.H                     
                
Description: Incremental Build Level control file.
=================================================================================  */

#ifndef PROJ_SETTINGS_H

/*------------------------------------------------------------------------------
Following is the list of the Build Level choices.
------------------------------------------------------------------------------*/
#define LEVEL1  1      		// Module check out (do not connect the motors) 
#define LEVEL2  2           // Verify ADC, park/clarke, calibrate the offset 
#define LEVEL3  3           // Verify closed current(torque) loop and PIDs and speed measurement
#define LEVEL4  4           // Verify speed estimation and rotor position est.
#define LEVEL5  5           // Verify close speed loop and speed PID
#define LEVEL6  6           // Verify close speed loop sensorless

/*------------------------------------------------------------------------------
This line sets the BUILDLEVEL to one of the available choices.
------------------------------------------------------------------------------*/
#define   BUILDLEVEL LEVEL3


#ifndef BUILDLEVEL    
#error  Critical: BUILDLEVEL must be defined !!
#endif  // BUILDLEVEL
//------------------------------------------------------------------------------


#ifndef TRUE
#define FALSE 0
#define TRUE  1
#endif

#define PI 3.14159265358979
#define M_2PI   6.283185307179586
#define M_2PIS3 2.094395102393195   // 2PI/3
#define RAC3S2  1.224744871391589   // racine(3/2)
#define RAC2S3  0.816496580927726   // racine(2/3)
#define RAC3    1.732050807568877   // racine(3)
#define RAC2    1.414213562373095   // racine(2)
#define INVRAC2 0.707106781186547   // 1.0/racine(2)
#define INVRAC3 0.577350269189625   // 1.0/racine(3)
#define INVRAC6 0.408248290463863   // 1.0/racine(6)

// Define the system frequency (MHz)
#if (DSP2803x_DEVICE_H==1)
#define SYSTEM_FREQUENCY 60
#elif (DSP2833x_DEVICE_H==1)
#define SYSTEM_FREQUENCY 150
#endif


//Define system Math Type
// Select Floating Math Type for 2833x
// Select IQ Math Type for 2803x 
#if (DSP2803x_DEVICE_H==1)
#define MATH_TYPE 0 
#elif (DSP2833x_DEVICE_H==1)
#define MATH_TYPE 1
#endif


// Define the ISR frequency (kHz)
#define ISR_FREQUENCY 10
#define T_ISR 0.0001

// Define the electrical motor parametes (Estun Servomotor)
#define RS 		2.35		    	    // Stator resistance (ohm) 
#define RR   			               	// Rotor resistance (ohm) 
#define LS   	0.0065					// Stator inductance (H) 
#define LR   			  				// Rotor inductance (H) 	
#define LM   			   				// Magnatizing inductance (H)
#define LD      0.0065                  // Stator inductance Ld (H)
#define LQ      0.0065                  // Stator inductance Lq (H)
#define POLES  	8						// Number of poles
//#define PHIMAX  0.075511  			// Amplitude du flux inducteur Ke = 31.63 V/krpm = 31.63/1000*30/PI V/rd.s-1(m�canique)
                                        // Donc PHIMAX = Ke/P = 0.30204/4 = 0.07551 (datasheet)
#define PHIMAX  0.065                   // Obtenu � partir de la mesure de la FEM
#define PHI_F (PHIMAX*RAC3S2)
//#define JROT    0.000031                // Inertie du rotor (kg.m�)
float JROT=0.00007;
//#define fROT    0.00069                 // Coef. frottement visqueux (Nm.rd/s)
float fROT=0.0015;
//#define Csec    0.046                   // Frottement secs Cs = P*PHIMAX*RACINE(3/2)*Iq_d�collement=4*0.065*rac(3/2)*0.15=0.0478N.m
float Csec=0.035;
#define RES_CODEUR  2500                // R�solution du codeur incr�mental
//#define ACC_MAX 0.1                     // Acc�l�ration maximale en rpm/Te = 1000rpm/s/Fe = 0.1

// Define the base quantites
#define MAX_VOLTAGE     93.5          // Tension nominale de la MSAP (� la vitesse nominale)
#define BASE_VOLTAGE    236.14        // Base peak phase voltage (volt), Vdc/sqrt(3)
#define BASE_CURRENT    10            // Base peak phase current (amp), Max. measurable peak curr.
#define BASE_TORQUE     		      // Base torque (N.m)
#define BASE_FLUX       			  // Base flux linkage (volt.sec/rad)
#define BASE_FREQ      	200           // Base electrical frequency (Hz) 

#endif

