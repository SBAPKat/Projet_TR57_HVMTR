/* ==============================================================================
System Name:  	Composite Controller

File Name:	  	HVPM_TR57.C

Description:	Primary system file for the Real Implementation of
          		Field Orientation Control for Three Phase Permanent-Magnet
          		Synchronous Motor (PMSM) 
 
				Supports F2833x(floating point) and F2803x(fixed point) devices  

La broche GPIO32 (I2C SDA) active pendant l'interruption Main_ISR(). (� configurer dans DeviceInit())


=================================================================================  */

// Include header files used in the main function

#include "PeripheralHeaderIncludes.h"
#include "IQmathLib.h"
#include <math.h>
#include "HVPM_TR57.h"
#include "HVPM_TR57-Settings.h"


#ifdef FLASH
#pragma CODE_SECTION(MainISR,"ramfuncs");
#pragma CODE_SECTION(OffsetISR,"ramfuncs");
#endif

// Prototype statements for functions found within this file.
interrupt void MainISR(void);
//interrupt void OffsetISR(void);
void DeviceInit();
void MemCopy();
void InitFlash();
void HVDMC_Protection(void);
void scib_init(void);
void InitScibGpio(void);
void putchar(int);
void puts(char *);
int getch(void);

#define LSPCLK  37500000L       // Fr�quence LSPCLK
#define BAUDRATE    115200        // Fr�quence de transmission du SCI

#define OPEN_LOOP 1
#define CLOSED_LOOP 2
int loop_status = OPEN_LOOP;

//CONTROLE
float KP_Vitesse = 0.01;
float KI_Vitesse = 1;
float Tau_integrale_Vitesse = 0.003;

float KP_Iq = 10;
float KI_Iq = 1;
float Tau_integrale_Iq = LQ/RS;

float KP_Id = 10;
float KI_Id =1;
float Tau_integrale_Id = LD/RS;
//VARIABLES
float Iq_ref_consigne = 0;
float couple_ref = 0;
float couple_ref_precedent = 0;
float Ud_ref = 0;
float Ud_ref_precedent = 0;
float Uq_ref = 0;
float Uq_ref_precedent = 0;
float Iq_ref = 0;
float Iq_ref_precedent = 0;
float Erreur_vitesse = 0;
float Erreur_vitesse_precedent = 0;
float Erreur_Iq = 0;
float Erreur_Iq_precedent = 0;
float Erreur_Id = 0;
float Erreur_Id_precedent = 0;

float I_alpha = 0;
float I_beta = 0;

#define RESET 1
int CL_reset_request = 0;



// Used for running BackGround in flash, and ISR in RAM
extern Uint16 *RamfuncsLoadStart, *RamfuncsLoadEnd, *RamfuncsRunStart;

// Global variables used in this system
float32 T = 0.001/ISR_FREQUENCY;    // Samping period (sec), see parameter.h 
float Fe=1000.0*ISR_FREQUENCY;      // Fech en Hz

Uint32 IsrTicker = 0;
Uint16 BackTicker = 0;

Uint16 TripFlagDMC=0;				//PWM trip status

// Default ADC initialization 
int ChSel[16]   = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
int	TrigSel[16] = {5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5};
int ACQPS[16]   = {8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8};

volatile Uint16 EnableFlag = FALSE;
// Instance a QEP interface driver 
QEP qep1 = QEP_DEFAULTS; 

// Instance a PWM driver instance
PWMGEN pwm1 = PWMGEN_DEFAULTS;

// Tableaux d�di�s au stockage pour affichage des courbes dans les graphes CCS
#define size_tab 500
struct mesures
{
    int tab1[size_tab];
    int tab2[size_tab];
    int tab3[size_tab];
    int tab4[size_tab];
    int tab5[size_tab];
    int tab6[size_tab];
};

struct mesures mes1;

int tabCount=0, decim_tab_Count=0, decim_tab=20;
float CPU_LOAD=0.0, CPU_LOAD1=0.0;



float Ia, Ib, Ic, Vbus, Va, Vb, Vc, Vq, Vd, Valpha, Vbeta;
float Ialpha, Ibeta, sinTheta, cosTheta;
float Id, Iq, Iqsat=1.0;
int Ia_t, Ib_t, Vbus_t;
int Ta, Tb, Tc, Tmax, Tmin;
float Offset_Ia, Offset_Ib, Offset_Vbus;
float Gain_Ia, Gain_Ib, Gain_Vbus;
int pos_k=0, pos_k1, Calage_Index=600, Nb_paire_poles;
float Res_Mec, Theta_Mec, Theta_Elec=0.0, Theta_Elec0;
float F_Elec_BO=0.0, Theta_Elec_BO=0.0, Vmax_BO=0.0;
float Vitref_rpm=100.0, Vitmax=0.0;

float Isecu=3.0;    // Limite maximale de courant de phase avant d�clenchement des s�curit�s

int index_profil=0, Secu=0, comp=0;
float k_VsF=3.3;
int Sync_ech=0, D_transmit=10, TransmitCount=0;
int x, tmp;

// Filtrage de la vitesse
float F0_vit;   // fr�quence de coupure du filtre passe-bas
float K1_fvit;   // 1er coef de l'�quation r�currence de vitesse en rd/s
float K2_fvit;   // 2�me coef de l'�quation r�currence de vitesse en rd/s
float Vit_rpm=0.0, Vit_rds=0.0, Vit_rds0=0.0, Puls_Elec;

#define TXCOUNT 100
long loopcount=0, i;
void main(void)
{
	
	DeviceInit();	// Device Life support & GPIO		

// Only used if running from FLASH
// Note that the variable FLASH is defined by the compiler
#ifdef FLASH
// Copy time critical code and Flash setup code to RAM
// The  RamfuncsLoadStart, RamfuncsLoadEnd, and RamfuncsRunStart
// symbols are created by the linker. Refer to the linker files. 
	MemCopy(&RamfuncsLoadStart, &RamfuncsLoadEnd, &RamfuncsRunStart);

// Call Flash Initialization to setup flash waitstates
// This function must reside in RAM
	InitFlash();	// Call the flash wrapper init function
#endif //(FLASH)

    Offset_Ia = 2260.0;
    Offset_Ib = 2260.0;
    Offset_Vbus = 0.0;
    Gain_Ia = 3.0/(4096.0*8.29562594*0.02);
    Gain_Ib = 3.0/(4096.0*8.29562594*0.02);
    Gain_Vbus = 3.0*1129.09/(4096.0*9.09);


    //Filtre de calcul de la vitesse
    F0_vit = 100.0; // F0 filtre vitesse : 100Hz
    K1_fvit = 1.0 - (T*M_2PI*F0_vit);
    K2_fvit = M_2PI*M_2PI*F0_vit/((float)RES_CODEUR*4.0);


    Res_Mec = M_2PI/((float)RES_CODEUR*4.0);
    Nb_paire_poles = POLES*0.5;

   // Waiting for enable flag set
   while (EnableFlag==FALSE) 
    { 
      BackTicker++;
    }

// Initialize PWM module	    
    pwm1.PeriodMax = SYSTEM_FREQUENCY*1000000*T/2;  // Prescaler X1 (T1), ISR period = T x 1
    pwm1.HalfPerMax=pwm1.PeriodMax/2;
    pwm1.Deadband  = 2.0*SYSTEM_FREQUENCY;     	    // 120 counts -> 2.0 usec for TBCLK = SYSCLK/1
    PWM_INIT_MACRO(1,2,3,pwm1)
    Tmax = pwm1.HalfPerMax;
    Tmin = -pwm1.HalfPerMax;
    

// Initialize ADC for DMC Kit Rev 1.1 	 
	ChSel[0]=1;		// Dummy meas. avoid 1st sample issue Rev0 Picollo*/
	ChSel[1]=1;		// ChSelect: ADC A1-> Phase A Current 
	ChSel[2]=9;		// ChSelect: ADC B1-> Phase B Current 
	ChSel[3]=3;		// ChSelect: ADC A3-> Phase C Current
	ChSel[4]=15;	// ChSelect: ADC B7-> Phase A Voltage
	ChSel[5]=14;	// ChSelect: ADC B6-> Phase B Voltage
	ChSel[6]=12;	// ChSelect: ADC B4-> Phase C Voltage
	ChSel[7]=7;		// ChSelect: ADC A7-> DC Bus  Voltage

// Initialize ADC module
	ADC_MACRO_INIT(ChSel,TrigSel,ACQPS)

// Initialize QEP module
    qep1.LineEncoder = RES_CODEUR;
    qep1.MechScaler = _IQ30(0.25/qep1.LineEncoder);
    qep1.PolePairs = POLES/2;
    qep1.CalibratedAngle = 0;
	QEP_INIT_MACRO(1,qep1)

// For this example, only init the pins for the SCI-B port.
// This function is found in the DSP2833x_Sci.c file.
	InitScibGpio();
	scib_init();     // Initialize and configure SCI B

//Call HVDMC Protection function
	HVDMC_Protection();

// Reassign ISRs. 

	EALLOW;	// This is needed to write to EALLOW protected registers
	PieVectTable.EPWM1_INT = &MainISR;
	EDIS;

// Enable PIE group 3 interrupt 1 for EPWM1_INT
    PieCtrlRegs.PIEIER3.bit.INTx1 = 1;

// Enable CNT_zero interrupt using EPWM1 Time-base
    EPwm1Regs.ETSEL.bit.INTEN = 1;   // Enable EPWM1INT generation 
    EPwm1Regs.ETSEL.bit.INTSEL = 1;  // Enable interrupt CNT_zero event
    EPwm1Regs.ETPS.bit.INTPRD = 1;   // Generate interrupt on the 1st event
	EPwm1Regs.ETCLR.bit.INT = 1;     // Enable more interrupts

// Enable CPU INT3 for EPWM1_INT:
	IER |= M_INT3;
// Enable global Interrupts and higher priority real-time debug events:
	EINT;   // Enable Global interrupt INTM
	ERTM;	// Enable Global realtime interrupt DBGM

// Exemple d'utilisation du bus CAN en cas de besoin
#if 0
	struct ECAN_REGS ECanaShadow;
	InitECanGpio();
	InitECan();
/* Write to the MSGID field  */

    ECanaMboxes.MBOX25.MSGID.all = 0x95555555; // Extended Identifier

/* Configure Mailbox under test as a Transmit mailbox */

    ECanaShadow.CANMD.all = ECanaRegs.CANMD.all;
    ECanaShadow.CANMD.bit.MD25 = 0;
    ECanaRegs.CANMD.all = ECanaShadow.CANMD.all;

/* Enable Mailbox under test */

    ECanaShadow.CANME.all = ECanaRegs.CANME.all;
    ECanaShadow.CANME.bit.ME25 = 1;
    ECanaRegs.CANME.all = ECanaShadow.CANME.all;

/* Write to DLC field in Master Control reg */

    ECanaMboxes.MBOX25.MSGCTRL.bit.DLC = 8;

/* Write to the mailbox RAM field */

    ECanaMboxes.MBOX25.MDL.all = 0x55555555;
    ECanaMboxes.MBOX25.MDH.all = 0x55555555;

	for(i=0; i < TXCOUNT; i++)
	{
	    ECanaShadow.CANTRS.all = 0;
	    ECanaShadow.CANTRS.bit.TRS25 = 1;             // Set TRS for mailbox under test
	    ECanaRegs.CANTRS.all = ECanaShadow.CANTRS.all;

	    do
	    {
	        ECanaShadow.CANTA.all = ECanaRegs.CANTA.all;
	    } while(ECanaShadow.CANTA.bit.TA25 == 0 );   // Wait for TA5 bit to be set..


	    ECanaShadow.CANTA.all = 0;
	    ECanaShadow.CANTA.bit.TA25 = 1;               // Clear TA5
	    ECanaRegs.CANTA.all = ECanaShadow.CANTA.all;

	    loopcount ++;
	 }
#endif


// IDLE loop. Just sit and loop forever:	
	for(;;)  //infinite loop
	{
	    while(ScibRegs.SCIFFRX.bit.RXFFST == 0) // Test si donn�es re�u dans la FIFO de r�ception SCI
	    {
	       if(Sync_ech)  // Si pas de r�ception alors transmission vers SCI (IHM)
	       {
	          putchar('a');
	          putchar(Ia_t);
	          putchar(Ia_t>>8);
	          putchar(Vbus_t);
	          putchar(Vbus_t>>8);
	          Sync_ech = 0;
	       }
	    }
	    x = getch(); /* traitement des donn�es re�ues (envoy�es par l'IHM si implant�) */
	    switch(x)
	    {
	       case 'a' :    // R�ception du taux de d�cimation D_transmit de la transmission des mesures analogiques
	           tmp = getch();
	           D_transmit = tmp + (getch()<<8);
	           break;
	    }



	}
} //END MAIN CODE


interrupt void MainISR(void)
{
// Set GPIO32 (I2C SDA)
    GpioDataRegs.GPBSET.bit.GPIO32=1;
// Verifying the ISR
    IsrTicker++;

// ------------------------------------------------------------------------------
//  Mesures analogiques
// ------------------------------------------------------------------------------
    Ia = ((AdcMirror.ADCRESULT1) - Offset_Ia)*Gain_Ia;          // Phase A curr.
    Ib = ((AdcMirror.ADCRESULT2) - Offset_Ib)*Gain_Ib;          // Phase B curr.
    Vbus = ((AdcMirror.ADCRESULT7) - Offset_Vbus)*Gain_Vbus;    // DC bus
    Ic = -Ia-Ib;                                                // Phase C curr.

// -------------------------------------------------------------------------------
//  S�curit� en cas de surcourant (d�passement de Isecu)
// -------------------------------------------------------------------------------

    if((Ia>=Isecu) || (Ia<(-Isecu))|| (Ib>Isecu) || (Ib<(-Isecu))|| (Secu==1))
    {
        Secu=2;
        EALLOW; // below registers are "protected", allow access.
        //  GPIO-00 - PIN FUNCTION = --Spare--
        GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 0;     // 0=GPIO,  1=EPWM1A,  2=Resv,  3=Resv
        GpioCtrlRegs.GPADIR.bit.GPIO0 = 1;      // 1=OUTput,  0=INput
        GpioDataRegs.GPACLEAR.bit.GPIO0 = 1;    // uncomment if --> Set Low initially
        //  GpioDataRegs.GPASET.bit.GPIO0 = 1;      // uncomment if --> Set High initially
        //--------------------------------------------------------------------------------------
        //  GPIO-01 - PIN FUNCTION = --Spare--
        GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 0;     // 0=GPIO,  1=EPWM1B,  2=ECAP6,  3=MFSR-B
        GpioCtrlRegs.GPADIR.bit.GPIO1 = 1;      // 1=OUTput,  0=INput
        GpioDataRegs.GPACLEAR.bit.GPIO1 = 1;    // uncomment if --> Set Low initially
        //  GpioDataRegs.GPASET.bit.GPIO1 = 1;      // uncomment if --> Set High initially
        //--------------------------------------------------------------------------------------
        //  GPIO-02 - PIN FUNCTION = --Spare--
        GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 0;     // 0=GPIO,  1=EPWM2A,  2=Resv,  3=Resv
        GpioCtrlRegs.GPADIR.bit.GPIO2 = 1;      // 1=OUTput,  0=INput
        GpioDataRegs.GPACLEAR.bit.GPIO2 = 1;    // uncomment if --> Set Low initially
        //  GpioDataRegs.GPASET.bit.GPIO2 = 1;      // uncomment if --> Set High initially
        //--------------------------------------------------------------------------------------
        //  GPIO-03 - PIN FUNCTION = --Spare--
        GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 0;     // 0=GPIO,  1=EPWM2B,  2=ECAP5,  3=MCLKR-B
        GpioCtrlRegs.GPADIR.bit.GPIO3 = 1;      // 1=OUTput,  0=INput
        GpioDataRegs.GPACLEAR.bit.GPIO3 = 1;    // uncomment if --> Set Low initially
        //  GpioDataRegs.GPASET.bit.GPIO3 = 1;      // uncomment if --> Set High initially
        //--------------------------------------------------------------------------------------
        //  GPIO-04 - PIN FUNCTION = --Spare--
        GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 0;     // 0=GPIO,  1=EPWM3A,  2=Resv,  3=Resv
        GpioCtrlRegs.GPADIR.bit.GPIO4 = 1;      // 1=OUTput,  0=INput
        GpioDataRegs.GPACLEAR.bit.GPIO4 = 1;    // uncomment if --> Set Low initially
        //  GpioDataRegs.GPASET.bit.GPIO4 = 1;      // uncomment if --> Set High initially
        //--------------------------------------------------------------------------------------
        //  GPIO-05 - PIN FUNCTION = --Spare--
        GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 0;     // 0=GPIO,  1=EPWM3B,  2=MFSR-A,  3=ECAP1
        GpioCtrlRegs.GPADIR.bit.GPIO5 = 1;      // 1=OUTput,  0=INput
        GpioDataRegs.GPACLEAR.bit.GPIO5 = 1;    // uncomment if --> Set Low initially
        //  GpioDataRegs.GPASET.bit.GPIO5 = 1;      // uncomment if --> Set High initially
        //--------------------------------------------------------------------------------------
            EDIS;   // Disable register access

    }
    else if(Secu==3) // R�amor�age de la PWM apr�s surcourant par �criture de 3 dans Secu
    {
        Secu=0;
        EALLOW; // below registers are "protected", allow access.
        //  GPIO-00 - PIN FUNCTION = --Spare--
            GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;     // 0=GPIO,  1=EPWM1A,  2=Resv,  3=Resv
        //  GPIO-01 - PIN FUNCTION = --Spare--
            GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1;     // 0=GPIO,  1=EPWM1B,  2=ECAP6,  3=MFSR-B
        //  GPIO-02 - PIN FUNCTION = --Spare--
            GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1;     // 0=GPIO,  1=EPWM2A,  2=Resv,  3=Resv
        //  GPIO-03 - PIN FUNCTION = --Spare--
            GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 1;     // 0=GPIO,  1=EPWM2B,  2=ECAP5,  3=MCLKR-B
        //  GPIO-04 - PIN FUNCTION = --Spare--
            GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 1;     // 0=GPIO,  1=EPWM3A,  2=Resv,  3=Resv
        //  GPIO-05 - PIN FUNCTION = --Spare--
            GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 1;     // 0=GPIO,  1=EPWM3B,  2=MFSR-A,  3=ECAP1
            EDIS;   // Disable register access
    }


// ------------------------------------------------------------------------------
//  Mesure de position. Calcul et filtrage de la vitesse et acc�l�ration. Calcul du Cem par �q. fond. de la dyn.
// ------------------------------------------------------------------------------
    pos_k1 = pos_k;
    pos_k = EQep1Regs.QPOSCNT + Calage_Index;   // position m�canique en pas du codeur par rapport � l'axe d
    if(pos_k>=EQep1Regs.QPOSMAX) pos_k-=EQep1Regs.QPOSMAX;

    if(abs(pos_k-pos_k1)>(EQep1Regs.QPOSMAX/2))
    {
        if(pos_k1>pos_k) pos_k1-=EQep1Regs.QPOSMAX;         //Evite l'erreur Index lors de la d�riv�e
        else pos_k1+=EQep1Regs.QPOSMAX;
    }

    Theta_Mec = Res_Mec*pos_k;                          // angle m�canique en rad
    Theta_Elec0 = Theta_Elec;   // Theta_Elec(n-1)
    Theta_Elec = (Nb_paire_poles*Theta_Mec)-floor((Nb_paire_poles*Theta_Mec)/M_2PI)*M_2PI;    // angle �lectrique en rad
    if(Theta_Elec0>Theta_Elec) Theta_Elec0-=M_2PI;      //Evite l'erreur Index lors de la d�riv�e

    Vit_rds0 = Vit_rds;         // Vit_rds(n-1)
    Vit_rds = K1_fvit*Vit_rds + K2_fvit*(pos_k - pos_k1); // vitesse m�canique filtr�e en rad/s
    Vit_rpm = Vit_rds*(30.0/PI);                        // vitesse m�canique filtr�e en rpm
//    Puls_Elec = (Theta_Elec - Theta_Elec0)*Fe;
    Puls_Elec = Nb_paire_poles*Vit_rds;     // Calcul de la pulsation �lectrique � partir de la vitesse m�canique filtr�e

    if(loop_status == OPEN_LOOP){
        // Boucle ouverte : g�n�ration des tensions Va,b,c pour la vitesse Vitref_rpm
        F_Elec_BO = Nb_paire_poles*Vitref_rpm/60.0;
        Theta_Elec_BO += T*M_2PI*F_Elec_BO;
        if(Theta_Elec_BO>=PI) Theta_Elec_BO-=M_2PI;
        Vmax_BO = k_VsF*PHIMAX*F_Elec_BO*M_2PI;
        Va = Vmax_BO*sin(Theta_Elec_BO);
        Vb = Vmax_BO*sin(Theta_Elec_BO-M_2PIS3);
        Vc = Vmax_BO*sin(Theta_Elec_BO+M_2PIS3);
    }
    else if (loop_status == CLOSED_LOOP){
        // Boucle ferm�e : g�n�ration des tensions Va,b,c pour la vitesse Vit_rpm
        //Controle PID de la vitesse, calcul du couple de référence 
        // Pas d'appel de fonction car Temps réel oblige
        //couple_ref = couple_ref_precedent + KP_Vitesse * (Erreur_vitesse - Erreur_vitesse_precedent + (Periode_echantillonnage / Tau_integrale_Vitesse) * Erreur_vitesse_precedent);
        if(CL_reset_request == RESET){
            Iq_ref = 0;
            Uq_ref = 0;
            Ud_ref = 0;
            Secu = 3;
            
            CL_reset_request = 0;
        }
        Erreur_vitesse = Vitref_rpm - Vit_rpm;

        Iq_ref = Iq_ref + (KP_Vitesse/(Nb_paire_poles*PHI_F))*(Erreur_vitesse - Erreur_vitesse_precedent + T_ISR/(Tau_integrale_Vitesse)*Erreur_vitesse_precedent);

        // anti windup
        if (Iq_ref >Iqsat) Iq_ref = Iqsat;
        if (Iq_ref<-Iqsat) Iq_ref = -Iqsat;
        
        // Calcul de Iq et Id en fonction de Ia Ib et Ic
        // use concordia to get Ialpha and Ibeta
        I_alpha = RAC2S3 * (Ia - 0.5*Ib - 0.5*Ic);
        I_beta = RAC2S3 * (RAC3/2.0*Ib - RAC3/2.0*Ic);
        sinTheta = sin(Theta_Elec);
        cosTheta = cos(Theta_Elec);

        Id = I_alpha * cosTheta + I_beta * sinTheta;
        Iq = -I_alpha * sinTheta + I_beta * cosTheta;
        Erreur_Iq = Iq_ref - Iq;
        Erreur_Id = 0.0 - Id;

        Uq_ref = Uq_ref + KP_Iq*(Erreur_Iq - Erreur_Iq_precedent + T_ISR/Tau_integrale_Iq*Erreur_Iq_precedent);
        Ud_ref = Ud_ref + KP_Id*(Erreur_Id - Erreur_Id_precedent + T_ISR/Tau_integrale_Id*Erreur_Id_precedent);


        if(Uq_ref>Vbus/2.0)Uq_ref=Vbus/2.0;
        if(Uq_ref<-Vbus/2.0)Uq_ref=-Vbus/2.0;
        if(Ud_ref>Vbus/2.0)Ud_ref=Vbus/2.0;
        if(Ud_ref<-Vbus/2.0)Ud_ref=-Vbus/2.0;

        Erreur_Id_precedent = Erreur_Id;
        Erreur_Iq_precedent = Erreur_Iq;
        Erreur_vitesse_precedent = Erreur_vitesse;
        Vq = Uq_ref + Puls_Elec*LD*Id + Puls_Elec * PHI_F;
        Vd = Ud_ref - Puls_Elec*LQ*Iq;
        Valpha = Vd * cosTheta - Vq * sinTheta;
        Vbeta = Vd * sinTheta + Vq * cosTheta;
        Va = INVRAC3 * (Valpha*RAC2);
        Vb = INVRAC3 * (-INVRAC2* Valpha + RAC3S2 * Vbeta);
        Vc = INVRAC3 * (-INVRAC2* Valpha - RAC3S2 * Vbeta);
    }
    else{
        // On ne devrait pas arriver ici,
        Va = 0.0;
        Vb = 0.0;
        Vc = 0.0;
    }
    

// ------------------------------------------------------------------------------
//  Calcul des comparateurs et saturation des tensions � Vbus/2 (pas d'injection d'harmonique 3)
// ------------------------------------------------------------------------------
    if(Vbus<1.0) Vbus = 1.0;    // pour �viter les divisions par 0 si pas d'alimentation
    Ta = (pwm1.PeriodMax)*Va/Vbus;
    Tb = (pwm1.PeriodMax)*Vb/Vbus;
    Tc = (pwm1.PeriodMax)*Vc/Vbus;

    if(Ta > Tmax) Ta = Tmax;
    if(Ta < Tmin) Ta = Tmin;
    if(Tb > Tmax) Tb = Tmax;
    if(Tb < Tmin) Tb = Tmin;
    if(Tc > Tmax) Tc = Tmax;
    if(Tc < Tmin) Tc = Tmin;

// ------------------------------------------------------------------------------
//  Mise � jour des PWM
// ------------------------------------------------------------------------------
    EPwm1Regs.CMPA.half.CMPA = pwm1.HalfPerMax + Ta;
    EPwm2Regs.CMPA.half.CMPA = pwm1.HalfPerMax + Tb;
    EPwm3Regs.CMPA.half.CMPA = pwm1.HalfPerMax + Tc;

// D�cimation des donn�es � transmettre sur le port SCI (dans la fonction main)
        TransmitCount++;
        if(TransmitCount>=D_transmit)
        {
           Ia_t = Ia*1000.0;
           Vbus_t = Vbus;
           Sync_ech = 1; /* indicateur permettant la synchro. du main() avec la d�cimation */
           TransmitCount = 0;
        }


// M�morisation en tableaux (int) � 10kHz d�cim� de decim_tab pour trac�s dans CCS
    decim_tab_Count++;
    if(decim_tab_Count>=decim_tab)
    {
        decim_tab_Count = 0;
        if(tabCount<size_tab)
        {
            mes1.tab1[tabCount] = Ia*10000.0;
            mes1.tab2[tabCount] = Vit_rpm*10.0;
            mes1.tab3[tabCount] = Vbus*100.0;
            mes1.tab4[tabCount] = pos_k;
            mes1.tab5[tabCount] = Iq * 1000;
            mes1.tab6[tabCount] = Iq_ref * 1000;

            //mes1.tab5[tabCount] = Vit_rpm*10.0;
            //mes1.tab6[tabCount] = Tc;
            tabCount++;
        }
        if(tabCount >= size_tab)
        {
            tabCount = 0;    //Buffer circulaire (sinon les donn�es restent m�moris�es)
        }
        if(tabCount==(size_tab*0.1)) Vitref_rpm = Vitmax;
        if(tabCount==(size_tab*0.4)) Vitref_rpm = 0.0;
        if(tabCount==(size_tab*0.6)) Vitref_rpm = -Vitmax;
        if(tabCount==(size_tab*0.9)) Vitref_rpm = 0.0;
        }




// Enable more interrupts from this timer
    EPwm1Regs.ETCLR.bit.INT = 1;

// Acknowledge interrupt to recieve more interrupts from PIE group 3
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;

// Calcul du taux d'occupation CPU : (Tinter/Te)*100.0
    if(EPwm1Regs.ETFLG.bit.INT) // if CPU load is 100% then set CPU_LOAD to 100.0
        CPU_LOAD = 100.0;
    else
    {
        if(EPwm1Regs.TBSTS.bit.CTRDIR) CPU_LOAD = (float)(EPwm1Regs.TBCTR)/((float)EPwm1Regs.TBPRD*2.0)*100.0; 
        else CPU_LOAD = (float)(2*EPwm1Regs.TBPRD - EPwm1Regs.TBCTR)/((float)EPwm1Regs.TBPRD*2.0)*100.0;
    }
// Reset GPIO10 (I2C SDA)
    GpioDataRegs.GPBCLEAR.bit.GPIO32=1;
}// MainISR Ends Here



/**********************************************************/
/***************Protection Configuration*******************/  
/**********************************************************/

void HVDMC_Protection(void)
{
 
      EALLOW;
      
// Configure Trip Mechanism for the Motor control software
// -Cycle by cycle trip on CPU halt
// -One shot IPM trip zone trip 
// These trips need to be repeated for EPWM1 ,2 & 3

//===========================================================================
//Motor Control Trip Config, EPwm1,2,3
//===========================================================================
    
// CPU Halt Trip  
      EPwm1Regs.TZSEL.bit.CBC6=0x1;
      EPwm2Regs.TZSEL.bit.CBC6=0x1;
      EPwm3Regs.TZSEL.bit.CBC6=0x1;

      EPwm1Regs.TZSEL.bit.OSHT1   = 1;  //enable TZ1 for OSHT
      EPwm2Regs.TZSEL.bit.OSHT1   = 1;  //enable TZ1 for OSHT     
      EPwm3Regs.TZSEL.bit.OSHT1   = 1;  //enable TZ1 for OSHT

// What do we want the OST/CBC events to do?
// TZA events can force EPWMxA
// TZB events can force EPWMxB

      EPwm1Regs.TZCTL.bit.TZA = TZ_FORCE_LO; // EPWMxA will go low 
      EPwm1Regs.TZCTL.bit.TZB = TZ_FORCE_LO; // EPWMxB will go low
      
      EPwm2Regs.TZCTL.bit.TZA = TZ_FORCE_LO; // EPWMxA will go low 
      EPwm2Regs.TZCTL.bit.TZB = TZ_FORCE_LO; // EPWMxB will go low
      
      EPwm3Regs.TZCTL.bit.TZA = TZ_FORCE_LO; // EPWMxA will go low 
      EPwm3Regs.TZCTL.bit.TZB = TZ_FORCE_LO; // EPWMxB will go low
      
      
      EDIS;

     // Clear any spurious OV trip
      EPwm1Regs.TZCLR.bit.OST = 1;
      EPwm2Regs.TZCLR.bit.OST = 1;
      EPwm3Regs.TZCLR.bit.OST = 1;  
      
//************************** End of Prot. Conf. ***************************//
}

// Transmit a character from the SCI
void putchar(int a)
{
    while (ScibRegs.SCIFFTX.bit.TXFFST >= 16);
    ScibRegs.SCITXBUF=a;
}

void puts(char * msg)
{
    while(*msg) putchar(*msg++);
}

int getch(void)
{
    while(ScibRegs.SCIFFRX.bit.RXFFST == 0);
    return ScibRegs.SCIRXBUF.all;
}

// Test 1,SCIB  DLB, 8-bit word, baud rate 0x000F, default, 1 STOP bit, no parity
void scib_init()
{
    Uint16 BRR;
    // Note: Clocks were turned on to the SCIB peripheral
    // in the InitSysCtrl() function

// Init. TX FIFO
    ScibRegs.SCIFFTX.bit.SCIFFENA = 1;  // SCI FIFO enhancements are enabled
    ScibRegs.SCIFFTX.bit.SCIRST = 1;    // SCI FIFO can resume transmit or receive
    ScibRegs.SCIFFTX.bit.TXFIFOXRESET = 1;  // Re-enable transmit FIFO operation
    ScibRegs.SCIFFTX.bit.TXFFINTCLR = 1;  // Write 1 to clear TXFFINT flag in bit 7
    ScibRegs.SCIFFTX.bit.TXFFIENA = 0;  // TX FIFO interrupt based on TXFFIVL match (less than or equal to) is disabled
    ScibRegs.SCIFFTX.bit.TXFFIL = 1;  // TXFFIL4�0 Transmit FIFO interrupt level bits. Transmit FIFO will generate interrupt when the FIFO
//    status bits (TXFFST4�0) and FIFO level bits (TXFFIL4�0 ) match (less than or equal to).

// Init. RX FIFO
    ScibRegs.SCIFFRX.bit.RXFIFORESET = 1;   // Re-enable receive FIFO operation
    ScibRegs.SCIFFRX.bit.RXFFINTCLR = 1;   // Write 1 to clear RXFFINT flag in bit 7
    ScibRegs.SCIFFRX.bit.RXFFIENA = 0;   // RX FIFO interrupt based on RXFFIVL match (less than or equal to) will be disabled
    ScibRegs.SCIFFCT.all=0x0;   // no delay

// Init. SCI format
    ScibRegs.SCICCR.all =0x0007;   // 1 stop bit,  No loopback
                                   // No parity,8 char bits,
                                   // async mode, idle-line protocol
    ScibRegs.SCICTL1.all =0x0003;  // enable TX, RX, internal SCICLK,
                                   // Disable RX ERR, SLEEP, TXWAKE
    ScibRegs.SCICTL2.bit.TXINTENA =0;   // Disable TX int
    ScibRegs.SCICTL2.bit.RXBKINTENA =1; // Disable RX int

    BRR = (float)LSPCLK/(8.0*(float)BAUDRATE)-0.5;  // calcul du BRR avec arrondi au plus proche
    ScibRegs.SCIHBAUD    =BRR>>8;  // @LSPCLK = 37.5MHz.
    ScibRegs.SCILBAUD    =BRR;
    ScibRegs.SCICTL1.all =0x0023;  // Relinquish SCI from Reset
}


//===========================================================================
// No more.
//===========================================================================
