#ifndef _QAC_
//******************************************************************************
//
// Company:      Johnson Controls Inc.
//-----------------------------------------------------------------------------
// Copyright:    This software is JCI property.
//               Duplication or disclosure without JCI written authorization
//               is prohibited.
//-----------------------------------------------------------------------------
// Project:      EL_DI_PSA_X81_IC_SOP_2016
// Language:     ANSI-C
//-----------------------------------------------------------------------------
/// \file
/// \brief  Component:    MDL_Telltale - Implementation.
//-----------------------------------------------------------------------------
// $Log:   //EL_DI_PSA_X81_IC_SOP_2016/archives/MC/Development/DEV/AS/CMP/SWC/MDL_Telltale/MDL_Telltale.c-arc  $
// 
//    Rev 1.29   18 Dec 2015 08:57:38   mkolte
// Cm088739: UT done after fix
// 
//    Rev 1.28   16 Dec 2015 12:55:04   mkolte
// Cm088739: On SM_VOM7 safety mechanism fault, SERVICE Telltale is made ON
// 
//    Rev 1.27   18 Nov 2015 08:27:38   mravat
// Cm088114: LOCF justification given
// 
//    Rev 1.26   31 Oct 2015 09:04:42   mravat
// Cm087682: Service telltale to turn ON, On corruption of data related to “Foot on brake” alert which is stored in flash memory
// 
//    Rev 1.25   09 Oct 2015 14:35:46   mravat
// Cm086745: CCR comments fix
// 
//    Rev 1.24   03 Aug 2015 13:10:12   spatil16
// Cm084247: CVA fix
// 
//    Rev 1.23   30 Jul 2015 13:43:22   spatil16
// Cm083854: Safety implementation
// 
//    Rev 1.22   29 Jul 2015 11:37:06   spatil16
// Cm083854: Safety implementation
// 
//    Rev 1.20   06 Jul 2015 16:30:38   apatil19
// Cm082536: Code review comments fix
// 
//    Rev 1.19   06 Jul 2015 15:49:14   apatil19
// Cm082536: L1 and L2 implementation
// 
//    Rev 1.18   11 Jun 2015 16:04:22   apatil19
// Cm082081: Code review comment implemented
// 
//    Rev 1.17   11 Jun 2015 12:57:44   apatil19
// Cm082081: Updated after code review
// 
//    Rev 1.16   09 Jun 2015 14:37:52   apatil19
// Cm082081: Updated Telltale structure no make it configurable, added compiler switch for L1 and L2 variant code, Implemented change for 15v2
// 
//    Rev 1.15   Jun 04 2015 16:59:04   ashaikw
// Cm082817 :: RTE adaption for IT4 RC2
// 
//    Rev 1.14   May 25 2015 17:51:34   apatil19
// Cm081414: Updated the Seatbelt blinking when only one seat belt is present
// 
//    Rev 1.13   Apr 28 2015 14:29:28   apatil19
// Cm080971: Checksum set to Invalid at timeout
// 
//    Rev 1.12   Apr 27 2015 21:46:32   apatil19
// Cm081413: Code review comment fixed
// 
//    Rev 1.11   Apr 27 2015 18:06:54   aravatm
// Cm080971: Validate checksum if the state of process counter is new
// 
//    Rev 1.10   Apr 24 2015 22:08:58   apatil19
// Cm081413: Lane departure TT Not working
// Cm081414:When SeatBelt =0 ,Back passemger TT not managed.
// 
//    Rev 1.9   Apr 24 2015 19:30:50   aravatm
// Cm080971: Checksum validity checked only if process counter is updated
// 
//    Rev 1.8   Apr 22 2015 15:29:58   apatil19
// Cm080969: Code review comment fix
// 
//    Rev 1.7   Apr 17 2015 22:46:06   apatil19
// Cm080969: Timmer DAT_DG_CAN_STATE_ENGINEhas stopped when ETAT_GMP!=0
// Cm081025: Timmer DAT_DG_CAN_POWER_STEERING_TIMEOUT has stopped when ETAT_GMP!=0
// 
//    Rev 1.6   Apr 17 2015 21:55:48   apatil19
// Cm080972: Added Condition ACTIVE_IN_CONTACT for checksum and counter, corrected req SysRS_14_Telltales-3403 and SysRS_14_Telltales-3404
// 
//    Rev 1.5   Apr 03 2015 15:19:42   apatil19
// Cm077190: Initialize the local variable with zero value
// 
//    Rev 1.4   Mar 27 2015 23:03:28   apatil19
// Cm077190: Implementation corrected for SysRS_14_Telltales-5374
// 
//    Rev 1.3   Mar 24 2015 22:58:42   apatil19
// Cm077362: Implemented req SysRS_14_Telltales-2318, SysRS_14_Telltales-2307, SysRS_14_Telltales-2438 and SysRS_14_Telltales-2449
// 
//    Rev 1.2   Mar 13 2015 13:52:38   apatil19
// Cm077029: Implemntation for req SysRS_14_Telltales-5714
// 
//    Rev 1.1   Feb 25 2015 15:11:14   agawadsw
// Cm078905: L2 DEV: Code Migration from Old PVCS structure to new PVCS structure
// 
//    Rev 1.46   Feb 11 2015 15:57:50   apatil19
// Cm077190: Code review comment fix
// 
//    Rev 1.45   Feb 11 2015 14:47:38   apander1
// Cm077190: Coverity re-run.
// 
//    Rev 1.44   Feb 05 2015 23:24:18   apatil19
// Cm077190: update after UT
// 
//    Rev 1.43   Feb 04 2015 21:25:56   apander1
// Cm077190: Coverity done.
// 
//    Rev 1.42   Feb 04 2015 19:15:22   apander1
// Cm077190: Filtering not Followed for Low tire Pressure Telltale .
// 
//    Rev 1.41   Feb 04 2015 17:24:04   apatil19
// Cm077024: Stop and   Service brake TT does not turn ON
// 
//    Rev 1.40   Feb 04 2015 16:49:56   apander1
// Cm077362: SendStatus is updated for sending real status of airbag telltale to the network.
// 
//    Rev 1.39   Feb 02 2015 18:09:28   apander1
// Cm077026: CalculatePowersteerFault is updated for resetting bEtatGmpFlagL to false if engine state is not running.
// 
//    Rev 1.38   Jan 27 2015 23:31:40   amagdut
// Cm077579: Fixed Review comments for Safety requirement.
// 
//    Rev 1.37   Jan 14 2015 18:51:44   apander1
// Cm076850: Review findings are fixed.
// 
//    Rev 1.36   Jan 14 2015 18:41:54   apander1
// Cm076850: Review findings are fixed.
// 
//    Rev 1.35   Jan 14 2015 00:06:40   apander1
// Cm076850: Updated for unit testing.
// 
//    Rev 1.34   Jan 12 2015 15:36:20   amagdut
// Cm076283:  Implement safety mechanism FCEA_WD_SM1
// 
//    Rev 1.33   Jan 09 2015 16:49:58   apander1
// Cm076850: IT3 requirements are implemented.
//
//    Rev 1.32   Dec 26 2014 20:50:02   apander1
// Cm076921: Optimization in UpdateEsptState removed.
//
//    Rev 1.31   Dec 26 2014 14:46:34   apander1
// Cm076921: Updated for MISRA warnings
//
//    Rev 1.30   Dec 24 2014 15:52:16   apander1
// Cm076921: When "DAT_DG_TELLTALES_ELEC_PARK_BRAKE" is 1 service brake TT should not depend on FRPK Signal : UpdateServiceBrakeDefect is updated
//
//    Rev 1.29   Dec 19 2014 21:51:14   apander1
// Cm076463: Updated for seatbelt telltales.
//
//    Rev 1.28   Dec 12 2014 19:56:38   apander1
// Cm076421: Review findings are fixed.
//
//    Rev 1.27   Dec 12 2014 18:26:22   apander1
// Cm076421: Enum value is updated in RI fiule and flag check is implemented in CalculateSteeringfault function
//
//    Rev 1.26   Dec 08 2014 17:00:26   apander1
// Cm074947: MISRA warnings are fixed
//
//    Rev 1.25   Dec 08 2014 14:06:38   apander1
// Cm074947: Code checked in after unit testing
//
//    Rev 1.24   Nov 28 2014 19:40:12   apander1
// Cm074947: Coverity warnings are fixed
//
//    Rev 1.23   Nov 28 2014 19:28:06   apander1
// Cm074947: Function is splitted to reduce cyclomatic complexity.
//
//    Rev 1.22   Nov 27 2014 17:56:52   apander1
// Cm074947: Review findings are fixed
//
//    Rev 1.21   Nov 27 2014 13:43:14   apander1
// Cm074947: Coverity is run.
//
//    Rev 1.20   Nov 26 2014 21:35:16   apander1
// Cm074947: IT2 requirements are implemented
//
//    Rev 1.19   Nov 24 2014 17:21:16   apander1
// Cm074947: Presence of telltales is checked from NvM
//
//    Rev 1.18   Nov 22 2014 20:54:24   apander1
// Cm074947: New requirements for IT2 are implemented
//
//    Rev 1.17   Nov 17 2014 16:40:20   agawadsw
// Cm075300: CVA activity
//
//    Rev 1.16   Nov 08 2014 00:31:36   apander1
// Cm073739: SetCanOutSignal is updated
//
//    Rev 1.15   Oct 29 2014 16:38:48   apander1
// Cm073227: UpdateDriverSeatbeltState and UpdatePassengerSeatbeltState functions are updated for checking OUXX signal before OUXX_CLIG
//
//    Rev 1.14   Oct 09 2014 15:37:28   apander1
// Cm071674: Coverity warnings are fixed.
//
//    Rev 1.13   Oct 08 2014 21:47:38   apander1
// Cm071103: SetCanOutsignal is updated for VOY parameters.
//
//    Rev 1.12   Oct 08 2014 20:09:20   apander1
// Cm071674: UpdateBatteryTelltale function is added for management of battery telltale.
//
//    Rev 1.11   Sep 15 2014 20:22:02   apander1
// Cm072609: Code is updated for Coverity and QAC warnings.
//
//    Rev 1.10   Sep 09 2014 16:35:46   apander1
// Cm071127: Changes are reverted as corresponding changes are taken in BSW.
//
//    Rev 1.9   Aug 27 2014 15:27:58   apander1
// Cm071983:TTProcessCanSigAndClig is renamed as TTProcessCanSigAndCodeFaultdependent
//
//    Rev 1.8   Aug 27 2014 13:52:02   apander1
// Cm071522:SetCanOutSignal is updated for calling RI_Write_ETAT_COMBINE_SIGNAL_DEF_AIRBAG in case of airbad defect
//
//    Rev 1.7   Aug 26 2014 18:12:08   apander1
// Cm071127 :TTProcess1CanSignal, TTProcessCanSigAndClig, UpdateESPTTState and TTProcessCanSigAndCodeFault are  updated for frame loss behaviour
// Cm071359 :UpdatePassengerSeatbeltState and UpdateDriverSeatbeltState are updated for taking TYPE_DIR into consideration
//
//    Rev 1.6   Aug 13 2014 19:02:38   apander1
// Cm070082: updated for euro6 telltale.UpdateEuro6 function is added
//
//    Rev 1.5   Aug 08 2014 20:19:12   apander1
// Cm070082: UpdateServiceBrakeDefect is updated
//
//    Rev 1.4   Aug 08 2014 16:49:12   apander1
// Cm071127: TTProcess1CanSignal is updated for checking the can state returned from SRV_ComExt
//
//    Rev 1.3   Aug 01 2014 22:22:08   apander1
// Cm070082: Updated for RC4 release
//
//    Rev 1.2   Jul 29 2014 23:31:34   apander1
// Cm070082: Updated for addition of telltales in L1T
//
//    Rev 1.1   Jul 17 2014 22:03:12   apander1
// Cm070082: Updated state_Started for bypassing carmodes
//
//    Rev 1.0   Jul 15 2014 22:01:52   apander1
// Initial revision.
//******************************************************************************

#endif  // #ifndef _QAC_

//==============================================================================
// Component trace identification
//==============================================================================
/// Component trace identification
#define MDL_Telltale   "MDL_Telltale"

//==============================================================================
// INCLUDED FILES
//==============================================================================


#ifndef PRIVATE
#define PRIVATE
#endif  // #ifndef PRIVATE

#include "MDL_Telltale_RI.h"
#include "MDL_Telltale.h"
#include "Rte_Adapter_MDL_Telltale.h"
#ifdef UTEST
    #define Static
#else
    #define Static static
#endif
//==============================================================================
// PRIVATE MACROS
//==============================================================================
#define cMdl_telltale_Safety_cFalse              ((uint16)0xD296)
#define cMdl_telltale_Safety_cTrue               ((uint16)0xD269)
#define cTTFaultAbsent  ((uint8)0)

//ESPACT
#define cESPInactive    ((uint8)0)
#define cESPActive      ((uint8)1)

//ESPI
#define cESPOperation   ((uint8)0)
#define cESPInhibited   ((uint8)1)

#define cINHIB_Indefine      ((uint8)0)
#define cINHIB_NoSignaling   ((uint8)2)
#define cINHIB_Signaling     ((uint8)1)
#define cINHIB_Fault         ((uint8)0x03)
#define cSession_Diag          0x03U

//For LowFuelLevelTelltale - MINC_CLIG
#define eCligSignal_Continous   ((uint8)0)
#define eCligSignal_Blinking    ((uint8)1)
#define eCligSignal_OFF         ((uint8)2)

#define SetTTState(TTState,TTIndex) (TTStates[TTIndex] = (TTState))
#define GetTTState(TTState,TTIndex) ((TTState) = TTStates[TTIndex])

//#define TimeoutResolution    100
#define cMOD_HEX                                ((uint8)0x0F)
#define cCHKini                                 ((uint8)0x03)
#define cVALID_CHKSUM                           ((uint8)0x0F)
#define cMAX_NO_INVALID_CHKSUM_FRAME            ((uint8)0x01)
#define cMAX_NO_INVALID_COUNTER_FRAME           ((uint8)0x03)
#define cInValid 0
#define cValid   1
#define cNotUsedValue                           ((TelltaleStatusType) 3)

#define cFrontPlaces  (uint8)2
#define cAllPlaces    (uint8)1

#define cIOCDIDTellTalesDriving        (uint16)0xD700u
#define cIOCDIDOutputDriving           (uint16)0xD707u
#define cRDBIDIDTellTalesDriving        (uint16)0xD700u
#define cTelltaleIOCPositiveRespCode   (uint8)0x00u
#define cDIDAllTellTalesON             (uint8)0xFFu
#define cDIDALLTellTalesOFF            (uint8)0x00u
#define  cRequestedIOCByteOne          (uint8)0x00u
#define  cRequestedIOCByteTwo          (uint8)0x01u
#define  cIOCTelltaleOnMask            (uint8)0x01u
#define  cHwTelltaleCount              (uint8)0x07u
#if(TARGET_VARIANT == TARGET_EP2JO)
  #define  cWitryOutUpperIndex           (uint8)0x19u
#else
  #define  cWitryOutUpperIndex           (uint8)0x18u
#endif


#define  cBitZeroMask                  (uint8)0x01u
#define  cBitOneMask                   (uint8)0x02u
#define  cBitTwoMask                   (uint8)0x04u
#define  cBitThreeMask                 (uint8)0x08u
#define  cBitFourMask                  (uint8)0x10u
#define  cBitFiveMask                  (uint8)0x20u
#define  cBitSixMask                   (uint8)0x40u
#define  cBitSevenMask                 (uint8)0x80u

#define  cOneShift                   (uint8)0x01u
#define  cTwoShift                   (uint8)0x02u
#define  cThreeShift                 (uint8)0x03u
#define  cFourShift                  (uint8)0x04u
#define  cFiveShift                  (uint8)0x05u
#define  cSixShift                   (uint8)0x06u
#define  cSevenShift                 (uint8)0x07u

#define cAllTelltaleOn                                    ((U8)255u)
#define cValueOne                                         ((U8)1u)

//==============================================================================
// PRIVATE TYPES
//==============================================================================
//MDL_Telltale_SD_1: Dynamic safety applied by monitoring task
static TelltaleStatusType TTStates [TelltaleId_Count];
Static uint8 u8IS_DAT_ABRFrameCountCheckSum = 0x0U;
Static uint8 u8IS_DAT_ABRFrameCountProcessCounter = 0x0U;
Static uint8 u8ProcessCounterValue = 0x0U;
#ifdef VIP_SAFETY_ENABLE
Static uint8 Mirror_u8IS_DAT_ABRFrameCountCheckSum = (uint8)(~(0x0U));
Static uint8 Mirror_u8IS_DAT_ABRFrameCountProcessCounter = (uint8)(~(0x0U));
Static uint8 Mirror_u8ProcessCounterValue = (uint8)(~0x0U);
#endif
Static TimerType EngineRunningTimer = 0x0U;
Static TimerType BrakeDistrFltTimer = 0x0U;
static TimerType BrakeDistrFltEngineRunningTimer = 0x0U;
#ifdef VIP_SAFETY_ENABLE
static TimerType Mirror_EngineRunningTimer = (TimerType)(~(0x0U));
Static TimerType Mirror_BrakeDistrFltTimer = (TimerType)(~(0x0U));
Static TimerType Mirror_BrakeDistrFltEngineRunningTimer = (TimerType)(~(0x0U));
#endif
#ifdef INCLUDE_SCOPE_POST_B1
//MDL_Telltale_SD_8: The corruption of this variable will not lead to violation of any safety goals.
static TimerType LaneDepartureTimer;
#endif
//MDL_Telltale_SD_9: The corruption of this variable will not lead to violation of any safety goals.
static TimerType TyrePressTimer;
//MDL_Telltale_SD_10: Dynamic safety applied by monitoring task
static boolean bPowerSteerORGate;
//MDL_Telltale_SD_11: Dynamic safety applied by monitoring task
static boolean bPowerSteerREDGate;
//MDL_Telltale_SD_12: Dynamic safety applied by monitoring task
static boolean bInfoAltFltSecureGate;
Static uint16 u16ChecksumFilter = cMdl_telltale_Safety_cTrue;
Static uint16 u16ProcessCounterFilter = cMdl_telltale_Safety_cTrue;
//MDL_Telltale_SD_15: Dynamic safety applied by monitoring task
static boolean bInfoBrakeDistrFltGate = cValid;
static boolean		bTTIOCInProcess = cFalse;
static boolean		bWiryOutIOCInProcess = cFalse;

uint8 u8ParamPilTemoinCmb = (uint8)0x00;
//==============================================================================
// FORWARD DECLARATIONS OF PRIVATE FUNCTIONS
//==============================================================================
static void state_Init(void);
static void state_Started(void);
static void state_Diag(void);
static eTransitionResult trans_ToInit(void);
static eTransitionResult trans_ToStarted(void);
static eTransitionResult trans_ToStopped(void);
static eTransitionResult trans_ToDiag(void);
static void SendAirBagPassInhibitionCanOut(void);
static TelltaleStatusType CheckTTControl(const uint8 TabelIndexP, TelltaleIdType* const TTIndexP);
static void SetDefaultValue(void);
static void SetCanOutSignal(const TelltaleStatusType TTStatusP, const TelltaleIdType TTIndexP);
static void CalculateCounterAndchecksum(void);
static void CalculatePowersteerFault(void);
static void CalculateBrakeDistrFault(void);
static void SendAirBagDefectTTStatus (boolean bTelltaleStatusL, boolean bTelltalePresentL);
static void GetRearPassengerSeatbeltStatus(TelltaleStatusType * State);
static TelltaleStatusType GetSeatBeltStatus(uint8 u8Continuous, uint8 u8Blinking);
static void IOC_WiryOutputControl(uint8 u8ByteOneP, uint8 u8ByteTwoP);
static void GetSignalID(tTelltaleSafetySignalID * u8SignalID, TelltaleIdType TTidP);
#ifdef VIP_SAFETY_ENABLE
static boolean CheckIfSafetyTelltale(uint8 TelltaleId);
#endif

//==============================================================================
// PRIVATE DATA
//==============================================================================
REGISTER_STATE_FUNCTIONS(MDL_Telltale, NULL_STATE, &state_Init, &state_Started, &state_Diag, NULL_STATE);
REGISTER_TRANS_FUNCTIONS(MDL_Telltale, &trans_ToInit, &trans_ToStarted, &trans_ToDiag, &trans_ToStopped);
DEFINE_CMP_CONTEXT(MDL_Telltale);

//==============================================================================
// CONSTANT PRIVATE DATA
//==============================================================================

//==============================================================================
// PRIVATE FUNCTIONS
//==============================================================================
//###################################################
//# state_Init
//###################################################
static void state_Init(void)
{
    // add code to be executed while Init state is active
	InitTellaleData();
    bTTIOCInProcess = cFalse;
    bWiryOutIOCInProcess = cFalse;
}

//###################################################
//# state_Started
//###################################################

static void state_Started(void)
{
    TelltaleIdType     TTIndexL;
    uint8           TableIndexL;
    boolean         bIsValidCarModeL = cFalse;
    TelltaleStatusType TTStatusL = TelltaleStatus_Off;
    sNonCfgTTControl const * pTTCfgInfo;

    CalculateCounterAndchecksum();
    CalculatePowersteerFault();
    CalculateBrakeDistrFault();
    SendAirBagPassInhibitionCanOut();

    //Check if PHASE_VIE = NORMAL
    (void)RI_Read_CARMODE_IS_Active(&bIsValidCarModeL);
    /*Check is valid Car Mode*/
    if(cFalse == bIsValidCarModeL)
    {
        /*If not valid => turn Off all TLTs */
        SetDefaultValue();
        InitTellaleData();
    }
    else
    {
        //Update the status of all telltales
        for (TableIndexL = (TelltaleIdType)0; TableIndexL < TTCfgInfoLength; TableIndexL++)
        {
              TTStatusL = CheckTTControl(TableIndexL, &TTIndexL);
              //CCOV:apatil19: (TTIndexL > TelltaleId_Count) Is not configured, its defensive case, can't be covered False case with valid configuration
              if(TTIndexL < TelltaleId_Count)
              {
            	  if( ((bTTIOCInProcess != cFalse) && (TableIndexL < cHwTelltaleCount) )
            		  || ((bWiryOutIOCInProcess != cFalse) && (TableIndexL >= cHwTelltaleCount) && (TableIndexL < cWitryOutUpperIndex))
            	     )
            	  {
                      //Do Nothing IOC is in Process
            	  }
            	  else
            	  {
            		  if((bTTIOCInProcess == cTrue) &&
            				  ((TTIndexL == TelltaleId_TELLTALES_TURN_SIGNAL_R) || (TTIndexL == TelltaleId_TELLTALES_TURN_SIGNAL_L)))
            		  {
            			  //When IOC service executed, TURN INDICATOR LED R & L set to ON in function MDL_Telltale_Process_IOC() but when value is overwritten by digital telltale value
            			  //Hence added above condition to by pass the setting the index 3rd & 4th of array TTStates[TTIndex]. TURN indicator HMI telltales will not affected with this condition
            		  }
            		  else
            		  {
            			  SetTTState(TTStatusL, TTIndexL);
            		  }
        			  // Set Configurable telltale on HMI
        			  SetConfigTelltale(TTIndexL, TTStatusL);
            	  }
                  SetCanOutSignal(TTStatusL, TTIndexL);
              }
        }
        // For telltales do not require DAT_DG parameters
        for(TableIndexL = (TelltaleIdType)0; TableIndexL < TTNonCfgInfoLength; TableIndexL++)
        {
        	pTTCfgInfo = GetTTDescriptionNonCfgTT(TableIndexL);

        	if(cDUMMY_ID_NONSAFETY == pTTCfgInfo->TTSwId)
        	{
        		TTStatusL = pTTCfgInfo->pTTProcessControl(TableIndexL);
        	}
        	else
        	{
        		//We will need SwID instead of Table index for safety teltales
        		TTStatusL = pTTCfgInfo->pTTProcessControl(pTTCfgInfo->TTSwId);
        	}

        	// Set Non Configurable telltale on HMI
        	SetNonConfigTelltale(TableIndexL, TTStatusL);
        }
        // Fixed Defect-21236, SysRS_14_Telltales-4945
        TTStatusL = UpdateServiceBrakeDefect(TelltaleId_TELLTALES_ELEC_PARK_BRAKE);
        SetBrakeWarningTelltale(TTStatusL);
        // Fixed Defect-7675 
        (void)UpdateDriverSeatbeltState(TelltaleId_TELLTALES_SEAT_BELT_OUTLINE);
        (void)UpdatePassengerSeatbeltState(TelltaleId_TELLTALES_SEAT_BELT_REM_CFG);
    }
    // Write the telltale Data on Rte.
    SendTelltaleDataOnRte();

    (void)RI_SRVSafety_Trigger_NotifyTrigger(SafetyTriggerId_MDLTelltalesTask);
}

//###################################################
//# state_Diag
//###################################################
static void state_Diag(void)
{
    // add code to be executed while Started diag state is active
}

//###################################################
//# trans_ToInit
//###################################################
static eTransitionResult trans_ToInit(void)
{
    // add code to be executed on transition to Init state
    // NOTE: Perform only simple initialization of your internal RAM data.
    // Reading and writing to RTE/NVM shall be avoided as these subsystems are
    // probably not active in this state. Use trans_ToStarted() for this purpose.

    return etrSuccessful;
}

//###################################################
//# trans_ToStarted
//###################################################
static eTransitionResult trans_ToStarted(void)
{
    // add code to be executed on transition to Started normal state
    // NOTE: You can safely read/write from/to RTE/NVM in this function.

    SetDefaultValue();

    //amagdut: SRV_Safety Implementation
    // Monitoring the MDL_Telltale Task
    /* STSR-13343: The SM FCEA_WD_SM1 shall be extended to detect if the safety related telltales status is not refreshed periodically and if the BSW driver configuration which drives telltales outputs is not checked periodically. Involved components are:
      - the applicative presentation of the telltales status,
      - the transfer of the telltales status to the output of the microcontroller in charge of the physical HW LED. */
    (void)RI_SRVSafety_Trigger_EnableMonitoring(SafetyTriggerId_MDLTelltalesTask, cTrue);

    return etrSuccessful;
}

//###################################################
//# trans_ToDiag
//###################################################
static eTransitionResult trans_ToDiag(void)
{
    // add code to be executed on transition to Started diag state

    return etrSuccessful;
}

//###################################################
//# trans_ToStopped
//###################################################
static eTransitionResult trans_ToStopped(void)
{
    SetDefaultValue();

    //amagdut: SRV_Safety Implementation
    (void)RI_SRVSafety_Trigger_EnableMonitoring(SafetyTriggerId_MDLTelltalesTask, cFalse);

    return etrSuccessful;
}

//###################################################
//# SetDefaultValue
//###################################################
static void SetDefaultValue(void)
{
    uint8 TTIndexL;

    //Loop through all telltales and switch off all
    for (TTIndexL=(uint8)0; TTIndexL<TelltaleId_Count; TTIndexL++)
    {
          //Update the array TTStates with the current state of the telltale
          SetTTState(TelltaleStatus_Off, TTIndexL);
          //Send information on CAN for the presence of the telltale on the cluster
          SetCanOutSignal(TelltaleStatus_Off, TTIndexL);
    }
}

//------------------------------------------------------------------------------
/// DESCRIPTION:         Process telltales
///
/// PARAMETERS:          \param TTidP: telltale id
///
/// RETURN VALUE:        \return TelltaleStatusType - On, Off, InAlarm
//------------------------------------------------------------------------------
static TelltaleStatusType CheckTTControl(const uint8 TabelIndexP, TelltaleIdType* const TTIndexP)
{
    sTTControl const *  pTTCfgInfo;
    uint8 u8PresenceL = 0U;
    uint8 u8PresenceSafetyL = 0U;
    TelltaleStatusType     TTStateL    = TelltaleStatus_Off;
    boolean IsSafety = cFalse;


    //Get the interfaces to be called for updating the status of the telltale
    pTTCfgInfo = GetTTDescription(TabelIndexP);
    *TTIndexP = pTTCfgInfo->TTSwId;

    (void)RI_ReadHwId(*TTIndexP, &u8PresenceL);

    //Check if Telltale is Safety related
    IsSafety = CheckIfSafetyTelltale( *TTIndexP);
    if(IsSafety == cTrue)
    {
    	(void)RI_ReadHwIdSafety(*TTIndexP, &u8PresenceSafetyL);
		if(((cFalse == u8PresenceL) && (u8PresenceSafetyL == 0x5)) ||
				((cTrue == u8PresenceL) && (u8PresenceSafetyL == 0xA)))
		{
			//Do Nothing
		}
		else
		{
			RI_SRV_Safety_ManageSafetyCountRL3A(eRL3A_NVM);
		}

    	if(u8PresenceL != (uint8)cZero)
    	{
    		TTStateL = pTTCfgInfo->pTTProcessControl( *TTIndexP);
    	}

    }
    else
    {

    	if(u8PresenceL != (uint8)cZero)
    	{
    		TTStateL = pTTCfgInfo->pTTProcessControl(TabelIndexP);
    	}
    }
    return TTStateL;
}

//-----------------------------------------------------------------------------------------------------
/// DESCRIPTION:         This function sends output on CAN for the presence of telltales on the cluster
///
/// PARAMETERS:          \param TTidP: telltale id
///
/// RETURN VALUE:        \return TelltaleStatusType - On, Off, InAlarm
//-----------------------------------------------------------------------------------------------------
static void SetCanOutSignal(const TelltaleStatusType TTStatusP, const TelltaleIdType TTIndexP)
{
    uint8 u8PresenceL = (uint8)0U;
    boolean bFlagL = cFalse;
#ifdef VIP_SAFETY_ENABLE
    uint8 u8PresenceSafetyL = (uint8)0U;
    boolean IsSafety = cFalse;
#endif

    const boolean bTelltaleStatusL = (TTStatusP == TelltaleStatus_Off) ? FALSE : TRUE;

    (void)RI_ReadHwId(TTIndexP, &u8PresenceL);
#ifdef VIP_SAFETY_ENABLE

    //Check if Telltale is Safety related
    IsSafety = CheckIfSafetyTelltale( TTIndexP);
    if(IsSafety == cTrue)
    {
    	(void)RI_ReadHwIdSafety(TTIndexP, &u8PresenceSafetyL);
		if(((cFalse == u8PresenceL) && (u8PresenceSafetyL == 0x5)) ||
				((cTrue == u8PresenceL) || (u8PresenceSafetyL == 0xA)))
		{
			//Do Nothing
		}
		else
		{
			RI_SRV_Safety_ManageSafetyCountRL3A(eRL3A_NVM);
		}
    }
#endif

    if(u8PresenceL == (uint8)cZero)
    {
        bFlagL = cFalse;
    }
    else
    {
        bFlagL = cTrue;
    }

    switch (TTIndexP)
    {
		case TelltaleId_TELLTALES_STOP:
			 (void)RI_Write_CMB_STOP_VOY(bFlagL);
			 break;

        case TelltaleId_TELLTALES_SERVICE:
             (void)RI_Write_CMB_SERVICE_VOY(bFlagL);
             break;

        case TelltaleId_TELLTALES_ENGINE_COOLANT_TEMPERATURE_LED:
             (void)RI_Write_CMB_T_EAU_VOY(bFlagL);
             (void)RI_Write_CMB_NIVE_VOY(bFlagL);
             break;

        case TelltaleId_TELLTALES_LOW_FUEL_LEVEL_LED:
             (void)RI_Write_CMB_MINC_VOY(bFlagL);
             break;

        case TelltaleId_TELLTALES_ELEC_PARK_BRAKE_SYST_FLT:
             (void)RI_Write_CMB_FSE_SYST_DEF_VOY(bFlagL);
             break;

        case TelltaleId_TELLTALES_ABS_FLT:
             (void)RI_Write_CMB_ABS_DEF_VOY(bFlagL);
             break;

        case TelltaleId_TELLTALES_ESP:
             (void)RI_Write_CMB_ASR_DEF_VOY(bFlagL);
             break;

        case TelltaleId_TELLTALES_PRESENCE_DOOR_OPEN:
             (void)RI_Write_CMB_PORT_OUV_VOY(bFlagL);
             break;

        case TelltaleId_TELLTALES_ENGINE_COOLANT:
             (void)RI_Write_CMB_T_EAU_VOY(bFlagL);
             (void)RI_Write_CMB_NIVE_VOY(bFlagL);
             break;

        case TelltaleId_TELLTALES_ENGINE_OIL:
              (void)RI_Write_CMB_PHUI_VOY(bFlagL);
              (void)RI_Write_CMB_T_HUIL_VOY(bFlagL);
             break;

        case TelltaleId_TELLTALES_PASS_SAF_FLT:
             (void)RI_Write_CMB_SEC_PASS_DEF_VOY(bFlagL);
             SendAirBagDefectTTStatus(bTelltaleStatusL, bFlagL);
             break;


        case TelltaleId_TELLTALES_BATTERY_FLT:
             (void)RI_Write_CMB_CBAT_DEF_VOY(bFlagL);
             (void)RI_Write_CMB_GENE_DEF_VOY(bFlagL);
             break;

        case TelltaleId_TELLTALES_LOW_FUEL_LEVEL:
             (void)RI_Write_CMB_MINC_VOY(bFlagL);
       break;

        case TelltaleId_TELLTALES_AVAIL_SPACE_MEAS:
             (void)RI_Write_CMB_MPD_VOY(bFlagL);
        break;

        default:
        break;
    }
}

//-----------------------------------------------------------------------------------------------------
/// DESCRIPTION:         This function sends output on CAN for the presence of telltales on the cluster
///
/// PARAMETERS:          \param u8ByteOneP: IOC request data Byte one, u8ByteTwoP: IOC request data Byte two
///
/// RETURN VALUE:        \return None
//-----------------------------------------------------------------------------------------------------
static void IOC_WiryOutputControl(uint8 u8ByteOneP, uint8 u8ByteTwoP)
{
	uint8 u8DataL;

	u8DataL = (uint8)((u8ByteOneP & cBitSevenMask)>> cSevenShift);
	if (u8DataL == 0x00U)
	{
		u8DataL = TelltaleStatus_Off;
	}
	else
	{
		u8DataL = TelltaleStatus_On;
	}
	SetTTState(u8DataL, TelltaleId_WIRE_LANE_POSITIONING_ASSIST_LED);

	u8DataL = (uint8)((u8ByteOneP & cBitSixMask)>> cSixShift);
	if (u8DataL == 0x00U)
	{
		u8DataL = TelltaleStatus_Off;
	}
	else
	{
		u8DataL = TelltaleStatus_On;
	}
	SetTTState(u8DataL, TelltaleId_WIRE_LANE_DEPARTURE_LED);

	u8DataL = (uint8)((u8ByteOneP & cBitFiveMask)>> cFiveShift);
	if (u8DataL == 0x00U)
	{
		u8DataL = TelltaleStatus_Off;
	}
	else
	{
		u8DataL = TelltaleStatus_On;
	}
	SetTTState(u8DataL, TelltaleId_WIRE_ESP_LED);

	u8DataL = (uint8)((u8ByteOneP & cBitFourMask)>> cFourShift);
	if (u8DataL == 0x00U)
	{
		u8DataL = TelltaleStatus_Off;
	}
	else
	{
		u8DataL = TelltaleStatus_On;
	}
	SetTTState(u8DataL, TelltaleId_WIRE_ELEC_PARK_BREAK_LED);

	u8DataL = (uint8)((u8ByteOneP & cBitThreeMask)>> cThreeShift);
	if (u8DataL == 0x00U)
	{
		u8DataL = TelltaleStatus_Off;
	}
	else
	{
		u8DataL = TelltaleStatus_On;
	}
	SetTTState(u8DataL, TelltaleId_WIRE_STOP_AND_START_LED);

	u8DataL = (uint8)((u8ByteOneP & cBitTwoMask)>> cTwoShift);
	if (u8DataL == 0x00U)
	{
		u8DataL = TelltaleStatus_Off;
	}
	else
	{
		u8DataL = TelltaleStatus_On;
	}
	SetTTState(u8DataL, TelltaleId_WIRE_CITY_PARK_LED);

	u8DataL = (uint8)((u8ByteOneP & cBitOneMask)>> cOneShift);
	if (u8DataL == 0x00U)
	{
		u8DataL = TelltaleStatus_Off;
	}
	else
	{
		u8DataL = TelltaleStatus_On;
	}
	SetTTState(u8DataL, TelltaleId_WIRE_XDOME_LED);

	u8DataL = u8ByteOneP & cBitZeroMask;
	if (u8DataL == 0x00U)
	{
		u8DataL = TelltaleStatus_Off;
	}
	else
	{
		u8DataL = TelltaleStatus_On;
	}
	SetTTState(u8DataL, TelltaleId_WIRE_HILL_ASSIST_LED);

	u8DataL = (uint8)((u8ByteTwoP & cBitSevenMask)>> cSevenShift);
	if (u8DataL == 0x00U)
	{
		u8DataL = TelltaleStatus_Off;
	}
	else
	{
		u8DataL = TelltaleStatus_On;
	}
	SetTTState(u8DataL, TelltaleId_WIRE_LANE_KEEPING_ASSIST_LED);

	u8DataL = (uint8)((u8ByteTwoP & cBitSixMask)>> cSixShift);
	if (u8DataL == 0x00U)
	{
		u8DataL = TelltaleStatus_Off;
	}
	else
	{
		u8DataL = TelltaleStatus_On;
	}
	SetTTState(u8DataL, TelltaleId_WIRE_ECO_MODE_LED);

	u8DataL = (uint8)((u8ByteTwoP & cBitFiveMask)>> cFiveShift);
	if (u8DataL == 0x00U)
	{
		u8DataL = TelltaleStatus_Off;
	}
	else
	{
		u8DataL = TelltaleStatus_On;
	}
	SetTTState(u8DataL, TelltaleId_WIRE_DYNAMIC_LED);

#if(TARGET_VARIANT == TARGET_EP2JO)
	u8DataL = (uint8)((u8ByteTwoP & cBitThreeMask)>> cThreeShift);
	if (u8DataL == 0x00U)
	{
		u8DataL = TelltaleStatus_Off;
	}
	else
	{
		u8DataL = TelltaleStatus_On;
	}
	SetTTState(u8DataL, TelltaleId_WIRE_PARKING_ASSISTANCE_LED);
#endif
}

//==============================================================================
// PUBLIC FUNCTIONS
//==============================================================================
//------------------------------------------------------------------------------
/// DESCRIPTION:         Control Telltale/Output from IOC Service
///
/// PARAMETERS:          \param ParamIn: Input Structure
///                      \param DcmParamOutType: Output structure
///
/// RETURN VALUE:        \return standard result codes apply
//------------------------------------------------------------------------------
FUNC(Std_ReturnType, RTE_CODE) MDL_Telltale_Process_IOC(P2CONST(DcmIOCParamInType, AUTOMATIC, RTE_APPL_DATA) ParamIn, P2VAR(DcmParamOutType, AUTOMATIC, RTE_APPL_DATA) ParamOutRef)
{
	Std_ReturnType RetL = S_OK;
	sTTControl const *  pTTCfgInfo;
	ParamOutRef->RequestResult = cTelltaleIOCPositiveRespCode;
	uint8 u8IndexL;
	uint8 u8BufDataL;
	uint8 u8TTIndexL;
	uint8 u8PresenceL = (uint8)cZero;

	 if(ParamIn->IOControlD == cIOCDIDTellTalesDriving)
	 {
			switch(ParamIn->Control)
			{
		    case DcmIOCControl_TemporaryControl:
  	            u8BufDataL = ParamIn->RequestParam[cRequestedIOCByteOne];
		        if((u8BufDataL == cDIDAllTellTalesON) || (u8BufDataL == cDIDALLTellTalesOFF))
		        {
		        	bTTIOCInProcess = cTrue;
		        	if(u8BufDataL == cDIDAllTellTalesON)
		        	{
		        		u8BufDataL = (U8)TelltaleStatus_On;
		        		u8ParamPilTemoinCmb = cDIDAllTellTalesON;
		        	}
		        	else
		        	{
		        		u8BufDataL = (U8)TelltaleStatus_Off;
		        		u8ParamPilTemoinCmb = cDIDALLTellTalesOFF;
		        	}
		        	for(u8IndexL = (uint8)0x0u; u8IndexL < cHwTelltaleCount; u8IndexL++)
		        	{
		        		 pTTCfgInfo = GetTTDescription(u8IndexL);
		        		 u8TTIndexL = pTTCfgInfo->TTSwId;
		        		 (void)RI_ReadHwId(u8TTIndexL, &u8PresenceL);

		        		 if(u8PresenceL == (uint8)cOne)
		        		 {
		        			 SetTTState(u8BufDataL, u8TTIndexL);
		        		 }
		        	}
		        }
		        else
		        {
		        	ParamOutRef->RequestResult = DcmRequestResult_RequestOutOfRange;
		        	RetL = E_FAIL;
		        }
		        break;

		    case DcmIOCControl_Stop:
		    	bTTIOCInProcess = cFalse;
		        break;

		    case DcmIOCControl_Freeze:
		    	bTTIOCInProcess = cTrue;
		        break;
		    default:
		        ParamOutRef->RequestResult = DcmRequestResult_RequestOutOfRange;
		        RetL = E_FAIL;
		        break;
			}

	 }
	 else if(ParamIn->IOControlD == cIOCDIDOutputDriving)
	 {
			switch(ParamIn->Control)
			{
		    case DcmIOCControl_TemporaryControl:
		    	bWiryOutIOCInProcess = cTrue;
		    	u8BufDataL = ParamIn->RequestParam[cRequestedIOCByteOne];
		    	u8TTIndexL = ParamIn->RequestParam[cRequestedIOCByteTwo];
		    	IOC_WiryOutputControl(u8BufDataL, u8TTIndexL);

		        break;

		    case DcmIOCControl_Stop:
		    	bWiryOutIOCInProcess = cFalse;
		        break;

		    case DcmIOCControl_Freeze:
		    	bWiryOutIOCInProcess = cTrue;
		        break;

		    default:
		        ParamOutRef->RequestResult = DcmRequestResult_RequestOutOfRange;
		        RetL = E_FAIL;
		        break;
			}

	 }
	 else
	 {
		 RetL = E_FAIL;
		 ParamOutRef->RequestResult = DcmRequestResult_RequestOutOfRange;
	 }

	return RetL;
}
//------------------------------------------------------------------------------
/// DESCRIPTION:         Control Telltale/Output from IOC Service
///
/// PARAMETERS:          \param ParamIn: Input Structure
///                      \param DcmParamOutType: Output structure
///
/// RETURN VALUE:        \return standard result codes apply
//------------------------------------------------------------------------------
FUNC(Std_ReturnType, RTE_CODE) MDL_Telltale_GetCurrentValue(P2CONST(DcmRDBIParamInType, AUTOMATIC, RTE_APPL_DATA) ParamIn, P2VAR(DcmParamOutType, AUTOMATIC, RTE_APPL_DATA) ValueRef)
{
	Std_ReturnType RetL = S_OK;

	if(ParamIn->RDBIDataId == cRDBIDIDTellTalesDriving)
	{
		ValueRef->RequestResult = cTelltaleIOCPositiveRespCode;
		ValueRef->ResponseParam[0] = u8ParamPilTemoinCmb;

	}
	else
	{
		RetL = E_FAIL;
		ValueRef->RequestResult = DcmRequestResult_RequestOutOfRange;
	}

	return RetL;
}

//------------------------------------------------------------------------------
/// DESCRIPTION:         Requests a system event to be processed by the component.
///
/// PARAMETERS:          \param eventP: unique system event identifier
///                      \param paramP: an eventP-specific parameter (0 - if unused)
///
/// RETURN VALUE:        \return standard result codes apply
//------------------------------------------------------------------------------
Std_ReturnType MDL_Telltale_OnSysMsg(const eSysMsgType eventP,
                                const tSysMsgParamType paramP)
{
    // perform user-events processing...
    //
    // ...finally delegate all other events to BaseCmp library implementation
    return LIB_BaseCmp_OnSysMsg(This(MDL_Telltale), eventP, paramP);
}

//------------------------------------------------------------------------------
/// DESCRIPTION:         Used to execute the main functionality of the component.
///                      The task call is done from the OS/System StateManager.
///                      The scheduling mechanism is defined by the OS
///                      configuration dependent on the needs of
///                      the individual components.
///
/// PARAMETERS:          \param None
///
/// RETURN VALUE:        \return None
//------------------------------------------------------------------------------
void MDL_Telltale_Task( void )
{
    LIB_BaseCmp_Task(This(MDL_Telltale));
}

//------------------------------------------------------------------------------
// UML::<interface name>
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
/// DESCRIPTION:         Used to read the status of a telltale.
///
/// PARAMETERS:          \param None
///
/// RETURN VALUE:        \return None
//------------------------------------------------------------------------------
Std_ReturnType MDL_Telltale_GetTelltaleStatus(const TelltaleIdType TelltaleP, TelltaleStatusType * const TelltaleStatusP)
{
    Std_ReturnType bResultL;
    if ((NULL != TelltaleStatusP) && (TelltaleP < TelltaleId_Count))
    {
        *TelltaleStatusP = TTStates[TelltaleP] ;
        bResultL = S_OK;
    }
    else
    {
        bResultL = E_INVALIDARG;
    }

    return bResultL;
}

//------------------------------------------------------------------------------
/// DESCRIPTION:         Process a CAN signal
///
/// PARAMETERS:          \param TTidP: telltale id
///
/// RETURN VALUE:        \return TelltaleStatusType - On, Off, InAlarm
//------------------------------------------------------------------------------
TelltaleStatusType TTProcess1CanSignal(const TelltaleIdType TTidP)
{
    sTTControl const *      pTTCfgInfo;
    TelltaleStatusType         TTStateL    = TelltaleStatus_Off;
    Std_ReturnType          bResultL = E_FAIL;
    uint8                   u8CANValueL = 0U;
    ComExtSignalStateType  CANStateUnusedL = 0U;

    //Get info about interfaces to be called for updating the status of the telltale
    pTTCfgInfo = GetTTDescription(TTidP);
    // CCOV:MKOLTE: The false case can not be covered as the Callback is assigned for each function call. See sTTControl TTCfgInfo[] cost array for reference.
    if (NULL != pTTCfgInfo->pReadTTControlTelltale)
    {
        bResultL = pTTCfgInfo->pReadTTControlTelltale(&u8CANValueL, &CANStateUnusedL);

        if (S_OK == bResultL)
        {
            if (cInvalidTelltaleStatus > (TelltaleStatusType)u8CANValueL)
            {
            	if(u8CANValueL == 0)
            	{
            		TTStateL = (TelltaleStatusType) TelltaleStatus_Off;
            	}
            	else if(u8CANValueL == 1)
            	{
            		TTStateL = (TelltaleStatusType) TelltaleStatus_On;
            	}
            	else
            	{
            		TTStateL = (TelltaleStatusType) TelltaleStatus_Warning;
            	}
            }
            else
            {
              //Nothing to do
            }
        }
        else
        {
          //Nothing to do
        }
    }
    else
    {
      //Nothing to do
    }

    return TTStateL;
}

//------------------------------------------------------------------------------
/// DESCRIPTION:         Process a CAN signal
///
/// PARAMETERS:          \param TTidP: telltale id
///
/// RETURN VALUE:        \return TelltaleStatusType - On, Off, InAlarm
//------------------------------------------------------------------------------
TelltaleStatusType TTProcess1CanSignalNonCfg(const TelltaleIdType TTidP)
{
	sNonCfgTTControl const *      pTTCfgInfo;
    TelltaleStatusType         TTStateL    = TelltaleStatus_Off;
    Std_ReturnType          bResultL = E_FAIL;
    uint8                   u8CANValueL = 0U;
    ComExtSignalStateType  CANStateUnusedL = 0U;

    //Get info about interfaces to be called for updating the status of the telltale
    pTTCfgInfo = GetTTDescriptionNonCfgTT(TTidP);
    // CCOV:MKOLTE: The false case can not be covered as the Callback is assigned for each function call. See sTTControl TTCfgInfo[] cost array for reference.
    if (NULL != pTTCfgInfo->pReadTTControlTelltale)
    {
        bResultL = pTTCfgInfo->pReadTTControlTelltale(&u8CANValueL, &CANStateUnusedL);

        if (S_OK == bResultL)
        {
            if (cInvalidTelltaleStatus > (TelltaleStatusType)u8CANValueL)
            {
            	if(u8CANValueL == 0)
            	{
            		TTStateL = (TelltaleStatusType) TelltaleStatus_Off;
            	}
            	else if(u8CANValueL == 1)
            	{
            		TTStateL = (TelltaleStatusType) TelltaleStatus_On;
            	}
            	else
            	{
            		TTStateL = (TelltaleStatusType) TelltaleStatus_Warning;
            	}
            }
            else
            {
              //Nothing to do
            }
        }
        else
        {
          //Nothing to do
        }
    }
    else
    {
      //Nothing to do
    }

    return TTStateL;
}


//------------------------------------------------------------------------------
/// DESCRIPTION:         Process a CAN signal for Advance emergency break system Telltale
///
/// PARAMETERS:          \param TTidP: telltale id
///
/// RETURN VALUE:        \return TelltaleStatusType -
//------------------------------------------------------------------------------
TelltaleStatusType TTProcessAvdavceEmergencyBreake(const TelltaleIdType TTidP)
{
    TelltaleStatusType         TTStateL    = TelltaleStatus_Off;
    Std_ReturnType          bResultL = E_FAIL;
    uint8                   u8CANValueL = 0U;
    ComExtSignalStateType  CANStateUnusedL = 0U;

    //Read CAN signal DMD_ALLUMAGE_FA value
    bResultL = RI_Read_CDE_COMBINE_TEMOINS_DMD_ALLUMAGE_FA(&u8CANValueL, &CANStateUnusedL);

    if (S_OK == bResultL)
    {
        if (cNotUsedValue != (TelltaleStatusType)u8CANValueL)
        {
        	if(u8CANValueL == 0)
        	{
        		TTStateL = (TelltaleStatusType) TelltaleStatus_Off;
        	}
        	else if(u8CANValueL == 1)
        	{
        		TTStateL = (TelltaleStatusType) TelltaleStatus_On;
        	}
        	else
        	{
        		TTStateL = (TelltaleStatusType) TelltaleStatus_Warning;
        	}
        }
        else
        {
            //Get present status and return the same when received CAN signal value is Invalid
            GetTTState(TTStateL,TelltaleId_TELLTALES_ADVANCED_EMERGENCY_BRAKE_SYSTEM);
        }
    }
    else
    {
      //Nothing to do
    }

    return TTStateL;
}

//------------------------------------------------------------------------------
/// DESCRIPTION:         Process a CAN signal and def code
///
/// PARAMETERS:          \param TTidP: telltale id
///
/// RETURN VALUE:        \return TelltaleStatusType - On, Off, InAlarm
//------------------------------------------------------------------------------
TelltaleStatusType AUTO_COMMUT_HIGHB_REGLProcess(const TelltaleIdType TTidP)
{
	sTTControl const *      pTTCfgInfo;
    TelltaleStatusType      TTStateL    = TelltalesAutoCommutHighbRegl_OFF;
    Std_ReturnType          bResultL = E_FAIL;
    ComExtSignalStateType   CANStateUnusedL = 0U;
    boolean                   bCANDefCodeL = 0U;
    boolean                   bTTControlCANL = 0U;

    pTTCfgInfo = GetTTDescription(TTidP);
    //CCOV:apatil19: Telltale configuration with (NULL != pTTCfgInfo->pReadTTControlBlinking) and (NULL == pTTCfgInfo->pReadTTControlTelltale)
    //Is not configured, First is true and second condition is false is wrong configuration and its defensive case, can't be covered with valid configuration
    if ((NULL != pTTCfgInfo->pReadTTControlBlinking) && (NULL != pTTCfgInfo->pReadTTControlTelltale))
    {
    	bResultL  = pTTCfgInfo->pReadTTControlBlinking(&bCANDefCodeL, &CANStateUnusedL);
    	bResultL |= pTTCfgInfo->pReadTTControlTelltale(&bTTControlCANL, &CANStateUnusedL);

    	if (S_OK == bResultL)
    	{
    		//Check if defect is not present
    		if (cFalse == bCANDefCodeL)
    		{
    			if (cTrue == bTTControlCANL)
    			{
    				TTStateL = (TelltaleStatusType)TelltalesAutoCommutHighbRegl_ON;
    			}

    		}
    		else
    		{
    			TTStateL = TelltalesAutoCommutHighbRegl_FAULT;
    		}
    	}
    }

    return TTStateL;
}

//------------------------------------------------------------------------------
/// DESCRIPTION:         Process a CAN signal and def code
///
/// PARAMETERS:          \param TTidP: telltale id
///
/// RETURN VALUE:        \return TelltaleStatusType - On, Off, InAlarm
//------------------------------------------------------------------------------
TelltaleStatusType TTProcessCanSigAndCodeFault(const TelltaleIdType TTidP)
{
    sTTControl const *      pTTCfgInfo;
    TelltaleStatusType      TTStateL    = TelltaleStatus_Off;
    Std_ReturnType          bResultL = E_FAIL;
    ComExtSignalStateType   CANStateUnusedL = 0U;
    uint8                   u8CANDefCodeL = 0U;
    uint8                   u8TTControlCANL = 0U;

    pTTCfgInfo = GetTTDescription(TTidP);
    //CCOV:apatil19: Telltale configuration with (NULL != pTTCfgInfo->pReadTTControlBlinking) and (NULL == pTTCfgInfo->pReadTTControlTelltale)
    //Is not configured, First is true and second condition is false is wrong configuration and its defensive case, can't be covered with valid configuration
    if ((NULL != pTTCfgInfo->pReadTTControlBlinking) && (NULL != pTTCfgInfo->pReadTTControlTelltale))
    {
        bResultL  = pTTCfgInfo->pReadTTControlBlinking(&u8CANDefCodeL, &CANStateUnusedL);
        bResultL |= pTTCfgInfo->pReadTTControlTelltale(&u8TTControlCANL, &CANStateUnusedL);

        if (S_OK == bResultL)
        {
            //Check if defect is not present
            if (cTTFaultAbsent == u8CANDefCodeL)
            {
                if (cInvalidTelltaleStatus > (TelltaleStatusType)u8TTControlCANL)
                {
                	if(u8TTControlCANL == 0)
                	{
                		TTStateL = (TelltaleStatusType) TelltaleStatus_Off;
                	}
                	else if(u8TTControlCANL == 1)
                	{
                		TTStateL = (TelltaleStatusType) TelltaleStatus_On;
                	}
                	else
                	{
                		TTStateL = (TelltaleStatusType) TelltaleStatus_Warning;
                	}
                }
                else
                {
                    //Nothing to do
                }
            }
            else
            {
                TTStateL = TelltaleStatus_Warning;
            }
        }
    }
    else
    {
        //Nothing to do
    }

    return TTStateL;
}
//------------------------------------------------------------------------------
/// DESCRIPTION:         Process a CAN signal and def code
///
/// PARAMETERS:          \param TTidP: telltale id
///
/// RETURN VALUE:        \return TelltaleStatusType - On, Off, InAlarm
//------------------------------------------------------------------------------
TelltaleStatusType TTProcessCanSigAndCodeFaultNonCfg(const TelltaleIdType TTidP)
{
	sNonCfgTTControl const *      pTTCfgInfo;
    TelltaleStatusType      TTStateL    = TelltaleStatus_Off;
    Std_ReturnType          bResultL = E_FAIL;
    ComExtSignalStateType   CANStateUnusedL = 0U;
    uint8                   u8CANDefCodeL = 0U;
    uint8                   u8TTControlCANL = 0U;

    pTTCfgInfo = GetTTDescriptionNonCfgTT(TTidP);
    //CCOV:apatil19: Telltale configuration with (NULL != pTTCfgInfo->pReadTTControlBlinking) and (NULL == pTTCfgInfo->pReadTTControlTelltale)
    //Is not configured, First is true and second condition is false is wrong configuration and its defensive case, can't be covered with valid configuration
    if ((NULL != pTTCfgInfo->pReadTTControlBlinking) && (NULL != pTTCfgInfo->pReadTTControlTelltale))
    {
        bResultL  = pTTCfgInfo->pReadTTControlBlinking(&u8CANDefCodeL, &CANStateUnusedL);
        bResultL |= pTTCfgInfo->pReadTTControlTelltale(&u8TTControlCANL, &CANStateUnusedL);

        if (S_OK == bResultL)
        {
            //Check if defect is not present
            if (cTTFaultAbsent == u8CANDefCodeL)
            {
                if (cInvalidTelltaleStatus > (TelltaleStatusType)u8TTControlCANL)
                {
                	if(u8TTControlCANL == 0)
                	{
                		TTStateL = (TelltaleStatusType) TelltaleStatus_Off;
                	}
                	else if(u8TTControlCANL == 1)
                	{
                		TTStateL = (TelltaleStatusType) TelltaleStatus_On;
                	}
                	else
                	{
                		TTStateL = (TelltaleStatusType) TelltaleStatus_Warning;
                	}
                }
                else
                {
                    //Nothing to do
                }
            }
            else
            {
                TTStateL = TelltaleStatus_Warning;
            }
        }
    }
    else
    {
        //Nothing to do
    }

    return TTStateL;
}
//------------------------------------------------------------------------------
/// DESCRIPTION:         Process two CAN signals (clig) where the telltale will
///                      turn on or blink only in case control signal is set
///
/// PARAMETERS:          \param TTidP: telltale id
///
/// RETURN VALUE:        \return TelltaleStatusType - On, Off, InAlarm
//------------------------------------------------------------------------------
TelltaleStatusType TTProcessCanSigAndCodeFaultdependentNonCfg(const TelltaleIdType TTidP)
{
	sNonCfgTTControl const *      pTTCfgInfo;
    uint8                   u8TTOnOffL = 0U;
    uint8                   u8TTBlinkingL = 0U;
    ComExtSignalStateType  CANStateUnusedL = 0U;
    TelltaleStatusType      TTStateL = TelltaleStatus_Off;
    Std_ReturnType          StdResultL = E_FAIL;
    boolean                   bPresenceL = cFalse;

    pTTCfgInfo = GetTTDescriptionNonCfgTT(TTidP);
    //Requirement SysRS_14_Telltales-7693 implemented.
    (void)RI_ReadHwId(TelltaleId_TELLTALES_SEAT_BELT_OUTLINE, &bPresenceL);

    if(cTrue == bPresenceL)
    {
    	//CCOV:apatil19: Telltale configuration with (NULL != pTTCfgInfo->pReadTTControlBlinking) and (NULL == pTTCfgInfo->pReadTTControlTelltale)
    	//Is not configured, First is true and second condition is false is wrong configuration and its defensive case, can't be covered with valid configuration
    	if ((NULL != pTTCfgInfo->pReadTTControlBlinking) && (NULL != pTTCfgInfo->pReadTTControlTelltale))
    	{
    		//Read the CAN signal responsible for blinking of the telltale
    		StdResultL  = pTTCfgInfo->pReadTTControlBlinking(&u8TTBlinkingL, &CANStateUnusedL);
    		//Read the CAN signal responsible for turning ON of the telltale
    		StdResultL |= pTTCfgInfo->pReadTTControlTelltale(&u8TTOnOffL, &CANStateUnusedL);

    		if (S_OK == StdResultL)
    		{
    			if (cTrue == (TelltaleStatusType)u8TTOnOffL)
    			{
    				if(eCligSignal_Blinking == (TelltaleStatusType)u8TTBlinkingL)
    				{
    					TTStateL = TelltaleStatus_Warning;
    				}
    				else if(eCligSignal_Continous == (TelltaleStatusType)u8TTBlinkingL)
    				{
    					TTStateL = TelltaleStatus_On;
    				}
    				else
    				{
    					//Nothing to do
    				}
    			}
    			else
    			{
    				//Nothing to do
    			}
    		}
    	}
    	else
    	{
    		//Nothing to do
    	}
    }
    return TTStateL;
}
//------------------------------------------------------------------------------
/// DESCRIPTION:         Process two CAN signals (clig) where the telltale will
///                      turn on or blink only in case control signal is set
///
/// PARAMETERS:          \param TTidP: telltale id
///
/// RETURN VALUE:        \return TelltaleStatusType - On, Off, InAlarm
//------------------------------------------------------------------------------
TelltaleStatusType TTProcessCanSigAndCodeFaultdependent(const TelltaleIdType TTidP)
{
    sTTControl const *      pTTCfgInfo;
    uint8                   u8TTOnOffL = 0U;
    uint8                   u8TTBlinkingL = 0U;
    ComExtSignalStateType  CANStateUnusedL = 0U;
    TelltaleStatusType      TTStateL = TelltaleStatus_Off;
    Std_ReturnType          StdResultL = E_FAIL;

    pTTCfgInfo = GetTTDescription(TTidP);
    //CCOV:apatil19: Telltale configuration with (NULL != pTTCfgInfo->pReadTTControlBlinking) and (NULL == pTTCfgInfo->pReadTTControlTelltale)
    //Is not configured, First is true and second condition is false is wrong configuration and its defensive case, can't be covered with valid configuration
    if ((NULL != pTTCfgInfo->pReadTTControlBlinking) && (NULL != pTTCfgInfo->pReadTTControlTelltale))
    {
        //Read the CAN signal responsible for blinking of the telltale
        StdResultL  = pTTCfgInfo->pReadTTControlBlinking(&u8TTBlinkingL, &CANStateUnusedL);
        //Read the CAN signal responsible for turning ON of the telltale
        StdResultL |= pTTCfgInfo->pReadTTControlTelltale(&u8TTOnOffL, &CANStateUnusedL);

        if (S_OK == StdResultL)
        {
            if (cTrue == (TelltaleStatusType)u8TTOnOffL)
            {
                if(eCligSignal_Blinking == (TelltaleStatusType)u8TTBlinkingL)
                {
                    TTStateL = TelltaleStatus_Warning;
                }
                else if(eCligSignal_Continous == (TelltaleStatusType)u8TTBlinkingL)
                {
                    TTStateL = TelltaleStatus_On;
                }
                else
                {
                    //Nothing to do
                }
            }
            else
            {
                //Nothing to do
            }
        }
    }
    else
    {
      //Nothing to do
    }

    return TTStateL;
}

//###################################################
//# GetSeatBeltStatus
//###################################################
static TelltaleStatusType GetSeatBeltStatus(uint8 u8Continuous, uint8 u8Blinking)
{
    TelltaleStatusType TTStateL = TelltaleStatus_Off;

    if((eDriverSeatbeltLamp_Reminder == u8Continuous) && (eDriverSeatbeltPattern_Blinking == u8Blinking))
    {
        TTStateL = TelltaleStatus_Warning;
    }
    else if(eDriverSeatbeltLamp_Reminder == u8Continuous)
    {
       TTStateL = TelltaleStatus_On;
    }
    else
    {
      // do nothing here
    }

    return TTStateL;
}
//------------------------------------------------------------------------------
/// DESCRIPTION:         Implementation of ESP telltale control
///
/// PARAMETERS:          \param TTidP: telltale id
///
/// RETURN VALUE:        \return TelltaleStatusType - On, Off, InAlarm
//------------------------------------------------------------------------------
TelltaleStatusType UpdateESPTTState(const TelltaleIdType TTidP)
{
    //variable to store Regulated ESP function
    tCDE_CMB_SIGNALISATION_ESPACT   ESPStateL = 0U;
    //variable to store ESP fault
    tCDE_COMBINE_TEMOINS_ASR_DEF    ESPFaultL = 0U;
    //Variable to get the state of the CAN message. It is unused
    ComExtSignalStateType          CANStateUnusedL = 0U;
    TelltaleStatusType              TTStateL = TelltaleStatus_Off;
    Std_ReturnType                  StdResultL = 0U;
    TelltaleStatusType              ESPOffStateL = 0U;

    (void)Rte_Adapter_Read_GetTelltaleState(TelltaleId_TELLTALES_ESP_OFF,&ESPOffStateL);

    if(TelltaleStatus_Off == ESPOffStateL)
    {
    	StdResultL  = MDL_Telltale_Read_Boolean_SafetyCritical_Signal(&ESPStateL, &CANStateUnusedL, ESP);
    	StdResultL |= MDL_Telltale_Read_Boolean_SafetyCritical_Signal (&ESPFaultL, &CANStateUnusedL, ESP_FLT);

    	if (S_OK == StdResultL)
    	{
    		if (TRUE == ESPFaultL)
    		{
    			TTStateL = TelltaleStatus_On;
    		}
    		else if (cESPActive == ESPStateL)
    		{
    			TTStateL = TelltaleStatus_Warning;
    		}
    		else
    		{
    			//Default TTStateL is set at initialization
    		}
    	}
    	else
    	{
    		//Nothing to do
    	}
    }
    else
    {
    	TTStateL = (TelltaleStatusType)TelltaleStatus_Off;
    }

    return TTStateL;
}

//------------------------------------------------------------------------------
/// DESCRIPTION:         Implementation of Driver Seatbelt telltale control
///
/// PARAMETERS:          \param TTidP: telltale id
///
/// RETURN VALUE:        \return TelltaleStatusType - On, Off, InAlarm
//------------------------------------------------------------------------------
TelltaleStatusType UpdateDriverSeatbeltState(const TelltaleIdType TTidP)
{
    //variable to read the steering direction from CAN
    tDONNEES_BSI_LENTES_TYPE_DIR        CANTYPE_DIRL = 0U;
    //variable to read the driver seatbelt telltale signal
    tCDE_CMB_SIGNALISATION_OUCC         CANOUCCL = 0U;
    //variable to read the CAN signal responsible for blinking of the driver seatbelt telltale
    tCDE_CMB_SIGNALISATION_OUCC_CLIG    CANOUCC_CLIGL = 0U;
    //variable to read the driver seatbelt telltale signal in case steering direction is right
    tCDE_CMB_SIGNALISATION_OUCP         CANOUCPL  = 0U;
    //variable to read the CAN signal responsible for blinking of the driver seatbelt telltale
    //in case steering direction is right
    tCDE_CMB_SIGNALISATION_OUCP_CLIG    CANOUCP_CLIGL  = 0U;
    //unused variable for reading the state of the CAN message
    ComExtSignalStateType              CANStateUnusedL  = 0U;
    Std_ReturnType                      StdResultL  = 0U;
    TelltaleStatusType                  TTStateL = TelltaleStatus_Off;
    //MDL_Telltale_SD_16: The corruption of this variable will not lead to violation of any safety goals.
    static uint8                        u8SteeringType = (uint8)eSteeringDir_Left;
    uint8                               u8PresencePassL  = 0U;
    uint8                               u8OutlineL  = 0U;
    TelltaleStatusType                  VHLStateL = TelltaleStatus_Off;

    //DAT_DG_TELLTALES_SEAT_BELT_REM_CFG  = ‘Front places’ or DAT_DG_TELLTALES_SEAT_BELT_OUTLINE  = ‘Presence’
    (void)RI_ReadHwId(TelltaleId_TELLTALES_SEAT_BELT_REM_CFG,&u8PresencePassL);
    (void)RI_ReadHwId(TelltaleId_TELLTALES_SEAT_BELT_OUTLINE,&u8OutlineL);

    //SysRS_14_Telltales-7817
    Send_Info_Telltales_Seat_Belt_REM_CFG(u8PresencePassL);

    if( cZero != u8OutlineL)
    {
         //SysRS_14_Telltales-7496
         GetRearPassengerSeatbeltStatus(&VHLStateL);
         if( TelltaleStatus_Off != VHLStateL)
         {
        	 VHLStateL = TelltaleStatus_On;
         }
         Send_Telltale_Seat_Belt_REM_VHL(VHLStateL);
    }

    if((cFrontPlaces == u8PresencePassL) || ( cZero != u8OutlineL))
    {
        //Read the actual steering direction received from CAN
        StdResultL = RI_Read_DONNEES_BSI_LENTES_TYPE_DIR(&CANTYPE_DIRL, &CANStateUnusedL);
        //update the variable if a valid steering direction is received

        if((CANTYPE_DIRL == eSteeringDir_Left) ||(CANTYPE_DIRL == eSteeringDir_Right))
        {
            u8SteeringType = CANTYPE_DIRL;
        }
        else
        {
            //Nothing to do
        }

        if(S_OK == StdResultL)
        {
            if(eSteeringDir_Left == u8SteeringType)
            {
                //Read the signal for turning on the driver seatbelt telltale ON
                StdResultL  = RI_Read_CDE_CMB_SIGNALISATION_OUCC(&CANOUCCL, &CANStateUnusedL);
                //Read the signal for blinking of the driver seatbelt telltale
                StdResultL |= RI_Read_CDE_CMB_SIGNALISATION_OUCC_CLIG(&CANOUCC_CLIGL, &CANStateUnusedL);

                if(S_OK == StdResultL)
                {
                    TTStateL = GetSeatBeltStatus(CANOUCCL, CANOUCC_CLIGL);
                    Send_Info_Telltales_Seat_Belt_REM_FRL(TTStateL);
                }
            }
            //CCOV:rpandey2: Since default steering direction is set as left condition below can not be false
            else if(eSteeringDir_Right == u8SteeringType)
            {
                //Read the signal for turning on the driver seatbelt telltale ON
                StdResultL  = RI_Read_CDE_CMB_SIGNALISATION_OUCP(&CANOUCPL, &CANStateUnusedL);
                //Read the signal for blinking of the driver seatbelt telltale
                StdResultL |= RI_Read_CDE_CMB_SIGNALISATION_OUCP_CLIG(&CANOUCP_CLIGL, &CANStateUnusedL);

                if(S_OK == StdResultL)
                {
                    TTStateL = GetSeatBeltStatus(CANOUCPL, CANOUCP_CLIGL);
                    Send_Info_Telltales_Seat_Belt_REM_FRL(TTStateL);
                }
            }
            else
            {
                //Nothing to do
            }
        }
        else
        {
            //Nothing to do
        }
    }
    else
    {
    	Send_Telltale_Seat_Belt_REM_VHL(TelltaleStatus_Off);
    	Send_Info_Telltales_Seat_Belt_REM_FRL(TelltaleStatus_Off);
    }

    if(cAllPlaces == u8PresencePassL)
    {
    	GetRearPassengerSeatbeltStatus(&TTStateL);
    	// SysRS_14_Telltales-7675
    	Send_Info_Telltales_Seat_Belt_REM_ALL(TTStateL);
    }
    else
    {
    	Send_Info_Telltales_Seat_Belt_REM_ALL(TelltaleStatus_Off);
    }
    return TTStateL;
}

//------------------------------------------------------------------------------
/// DESCRIPTION:         Implementation of Passenger Seatbelt telltale control
///
/// PARAMETERS:          \param TTidP: telltale id
///
/// RETURN VALUE:        \return TelltaleStatusType - On, Off, InAlarm
//------------------------------------------------------------------------------
TelltaleStatusType UpdatePassengerSeatbeltState(const TelltaleIdType TTidP)
{
    //variable to read the steering direction from CAN
    tDONNEES_BSI_LENTES_TYPE_DIR        CANTYPE_DIRL  = 0U;
    //variables to read the CAN signals responsible for the front passenger's seatbelt
    uint8         CANOUCPL  = 0U;
    uint8    CANOUCP_CLIGL  = 0U;
    uint8         CANOUCCL  = 0U;
    uint8    CANOUCC_CLIGL  = 0U;
    ComExtSignalStateType               CANStateUnusedL  = 0U;
    Std_ReturnType                      StdResultL  = 0U;
    TelltaleStatusType                  TTStateL = TelltaleStatus_Off;
    //MDL_Telltale_SD_17: The corruption of this variable will not lead to violation of any safety goals.
    static uint8                        u8PSteeringType = (uint8)eSteeringDir_Left;
    uint8                               u8PresenceL  = 0U;
    uint8                               u8OutlineL   = 0U;

    //DAT_DG_TELLTALES_SEAT_BELT_REM_CFG  = ‘Front places’ or DAT_DG_TELLTALES_SEAT_BELT_OUTLINE  = ‘Presence’
    (void)RI_ReadHwId(TelltaleId_TELLTALES_SEAT_BELT_REM_CFG,&u8PresenceL);
    (void)RI_ReadHwId(TelltaleId_TELLTALES_SEAT_BELT_OUTLINE,&u8OutlineL);

    if((cFrontPlaces == u8PresenceL) || ( cZero != u8OutlineL))
    {
        StdResultL = RI_Read_DONNEES_BSI_LENTES_TYPE_DIR(&CANTYPE_DIRL, &CANStateUnusedL);

        if((CANTYPE_DIRL == eSteeringDir_Left) ||(CANTYPE_DIRL == eSteeringDir_Right))
        {
            u8PSteeringType = CANTYPE_DIRL;
        }
        else
        {
            //Nothing to do
        }

        if(S_OK == StdResultL)
        {
            //Read the CAN signal for each individual passenger's seatbelt
            //state of the can message is unused as it has already been taken care of at SRV_ComExt level
            StdResultL  = RI_Read_CDE_CMB_SIGNALISATION_OUCP(&CANOUCPL, &CANStateUnusedL);
            StdResultL |= RI_Read_CDE_CMB_SIGNALISATION_OUCP_CLIG(&CANOUCP_CLIGL, &CANStateUnusedL);
            StdResultL |= RI_Read_CDE_CMB_SIGNALISATION_OUCC(&CANOUCCL, &CANStateUnusedL);
            StdResultL |= RI_Read_CDE_CMB_SIGNALISATION_OUCC_CLIG(&CANOUCC_CLIGL, &CANStateUnusedL);

            if(S_OK == StdResultL)
            {
                if(eSteeringDir_Left == u8PSteeringType)
                {
                    TTStateL = GetSeatBeltStatus(CANOUCPL, CANOUCP_CLIGL);
                    Send_Info_Telltales_Seat_Belt_REM_FRR(TTStateL);
                }
                //CCOV:rpandey2: Since default steering direction is set as left condition below can not be false
                else if(eSteeringDir_Right == u8PSteeringType)
                {
                    TTStateL = GetSeatBeltStatus(CANOUCCL, CANOUCC_CLIGL);
                    Send_Info_Telltales_Seat_Belt_REM_FRR(TTStateL);
                }
                else
                {
                    //Nothing to do
                }
            }
            else
            {
                //Nothing to do
            }
        }
        else
        {
            //Nothing to do
        }

    }
    else
    {
    	Send_Info_Telltales_Seat_Belt_REM_FRR(TelltaleStatus_Off);
    }
    return TTStateL;
}

//------------------------------------------------------------------------------
/// DESCRIPTION:         Implementation of Rear Passenger Seatbelt telltale
///
/// PARAMETERS:          \param TTidP: telltale id
///
/// RETURN VALUE:        \return TelltaleStatusType - On, Off, InAlarm
//------------------------------------------------------------------------------
//CCM 22: rpandey2: CCM is high because of multiple signals required for each passenger resulting in complex conditions.
//Breaking this function will make it difficult to understand.
static void GetRearPassengerSeatbeltStatus(TelltaleStatusType * State)
{
    Std_ReturnType                      StdResultL  = 0U;
    //variables to read the CAN signals responsible for each individual rear passenger
    tCDE_CMB_SIGNALISATION_OUCARG       CANOUCARGL  = 0U;
    tCDE_CMB_SIGNALISATION_OUCARM       CANOUCARML  = 0U;
    tCDE_CMB_SIGNALISATION_OUCARD       CANOUCARDL  = 0U;
    tCDE_COMBINE_TEMOINS_OUCARG2        CANOUCARG2L  = 0U;
    tCDE_COMBINE_TEMOINS_OUCARD2        CANOUCARD2L  = 0U;
    tCDE_CMB_SIGNALISATION_OUCARG_CLIG  CANOUCARG_CLIGL  = 0U;
    tCDE_CMB_SIGNALISATION_OUCARM_CLIG  CANOUCARM_CLIGL  = 0U;
    tCDE_CMB_SIGNALISATION_OUCARD_CLIG  CANOUCARD_CLIGL  = 0U;
    tCDE_COMBINE_TEMOINS_OUCARG2_CLIG   CANOUCARG2_CLIGL  = 0U;
    tCDE_COMBINE_TEMOINS_OUCARD2_CLIG   CANOUCARD2_CLIGL  = 0U;
    tCDE_CMB_SIGNALISATION_OUCP         CANOUCPL  = 0U;
    tCDE_CMB_SIGNALISATION_OUCP_CLIG    CANOUCP_CLIGL  = 0U;
    tCDE_CMB_SIGNALISATION_OUCC         CANOUCCL  = 0U;
    tCDE_CMB_SIGNALISATION_OUCC_CLIG    CANOUCC_CLIGL  = 0U;
    ComExtSignalStateType               CANStateUnusedL  = 0U;


    StdResultL = RI_Read_CDE_CMB_SIGNALISATION_OUCARG(&CANOUCARGL, &CANStateUnusedL);
    StdResultL |= RI_Read_CDE_CMB_SIGNALISATION_OUCARM(&CANOUCARML, &CANStateUnusedL);
    StdResultL |= RI_Read_CDE_CMB_SIGNALISATION_OUCARD(&CANOUCARDL, &CANStateUnusedL);
    StdResultL |= RI_Read_CDE_COMBINE_TEMOINS_OUCARG2(&CANOUCARG2L, &CANStateUnusedL);
    StdResultL |= RI_Read_CDE_COMBINE_TEMOINS_OUCARD2(&CANOUCARD2L, &CANStateUnusedL);
    StdResultL |= RI_Read_CDE_CMB_SIGNALISATION_OUCARG_CLIG(&CANOUCARG_CLIGL, &CANStateUnusedL);
    StdResultL |= RI_Read_CDE_CMB_SIGNALISATION_OUCARM_CLIG(&CANOUCARM_CLIGL, &CANStateUnusedL);
    StdResultL |= RI_Read_CDE_CMB_SIGNALISATION_OUCARD_CLIG(&CANOUCARD_CLIGL, &CANStateUnusedL);
    StdResultL |= RI_Read_CDE_COMBINE_TEMOINS_OUCARG2_CLIG(&CANOUCARG2_CLIGL, &CANStateUnusedL);
    StdResultL |= RI_Read_CDE_COMBINE_TEMOINS_OUCARD2_CLIG(&CANOUCARD2_CLIGL, &CANStateUnusedL);
    StdResultL |= RI_Read_CDE_CMB_SIGNALISATION_OUCP(&CANOUCPL, &CANStateUnusedL);
    StdResultL |= RI_Read_CDE_CMB_SIGNALISATION_OUCP_CLIG(&CANOUCP_CLIGL, &CANStateUnusedL);
    StdResultL |= RI_Read_CDE_CMB_SIGNALISATION_OUCC(&CANOUCCL, &CANStateUnusedL);
    StdResultL |= RI_Read_CDE_CMB_SIGNALISATION_OUCC_CLIG(&CANOUCC_CLIGL, &CANStateUnusedL);

   if((ePassSeatbeltLamp_Reminder == CANOUCARGL)
      || (ePassSeatbeltLamp_Reminder == CANOUCARML) || (ePassSeatbeltLamp_Reminder == CANOUCARDL)
      || (ePassSeatbeltLamp_Reminder == CANOUCARG2L) || (ePassSeatbeltLamp_Reminder == CANOUCARD2L)
      || (ePassSeatbeltLamp_Reminder == CANOUCPL) || (ePassSeatbeltLamp_Reminder == CANOUCCL))
    {

        if((ePassengerSeatbeltPattern_Blinking == CANOUCARG_CLIGL)
            || (ePassengerSeatbeltPattern_Blinking == CANOUCARM_CLIGL) || (ePassengerSeatbeltPattern_Blinking == CANOUCARD_CLIGL)
            || (ePassengerSeatbeltPattern_Blinking == CANOUCARG2_CLIGL) ||(ePassengerSeatbeltPattern_Blinking == CANOUCARD2_CLIGL)
            || (ePassengerSeatbeltPattern_Blinking == CANOUCP_CLIGL) ||   (ePassengerSeatbeltPattern_Blinking == CANOUCC_CLIGL))
        {
            *State = TelltaleStatus_Warning;
        }
        else
        {
            *State = TelltaleStatus_On;
        }
    }
    else
    {
        *State = TelltaleStatus_Off;
    }
}

//------------------------------------------------------------------------------
/// DESCRIPTION:         Implementation of Service Break Defect telltale control
///
/// PARAMETERS:          \param TTidP: telltale id
///
/// RETURN VALUE:        \return TelltaleStatusType - On, Off, InAlarm
//------------------------------------------------------------------------------
TelltaleStatusType UpdateServiceBrakeDefect(const TelltaleIdType TTidP)
{
    Std_ReturnType                      StdResultL  = 0U;
    ComExtSignalStateType               CANStateUnusedL  = 0U;
    //variable to read the State of the parking brake
    tCDE_CMB_SIGNALISATION_FRPK         CANFRPKL  = 0U;
    //variable to read the Brake fluid warning
    tCDE_COMBINE_TEMOINS_NIVL_AL        CANNIVL_ALL  = 0U;
    //variable to read the Braking force distribution system fault
    tCOMBINE_TEMOINS_REF_DEF            CANREF_DEFL  = 0U;
    TelltaleStatusType                  TTStateL = TelltaleStatus_Off;
    uint8                               u8Presence1L = 0U;
    uint8                               u8Presence2L = 0U;

#ifdef VIP_SAFETY_ENABLE
    boolean IsSafety = cFalse;
    uint8 								u8PresenceSafety1L = 0U;
    uint8 								u8PresenceSafety2L = 0U;
#endif

    StdResultL  = MDL_Telltale_Read_Boolean_SafetyCritical_Signal(&CANFRPKL, &CANStateUnusedL, PARK_BRAKE);
    StdResultL |= MDL_Telltale_Read_Boolean_SafetyCritical_Signal(&CANNIVL_ALL, &CANStateUnusedL, BRAKE_FLUID_LEVEL);
    StdResultL |= MDL_Telltale_Read_Boolean_SafetyCritical_Signal(&CANREF_DEFL, &CANStateUnusedL, BRAKE_DISTR_FLT);

    if(S_OK == StdResultL)
    {
        //ajoshi8: Task 478509 : Read explicitly the presence value of both parameters from NvM
    	(void)RI_ReadHwId(TelltaleId_TELLTALES_ELEC_PARK_BRAKE, &u8Presence1L);
        (void)RI_ReadHwId(TelltaleId_TELLTALES_BRAKE_FAILURE, &u8Presence2L);

#ifdef VIP_SAFETY_ENABLE

        //Check if Telltale is Safety related
        IsSafety = CheckIfSafetyTelltale( TelltaleId_TELLTALES_ELEC_PARK_BRAKE);
        if(IsSafety == cTrue)
        {
    		(void)RI_ReadHwIdSafety(TelltaleId_TELLTALES_ELEC_PARK_BRAKE, &u8PresenceSafety1L);
    		(void)RI_ReadHwIdSafety(TelltaleId_TELLTALES_BRAKE_FAILURE, &u8PresenceSafety2L);

    		if( (((cFalse == u8Presence1L) && (u8PresenceSafety1L == 0x5)) || ((cTrue == u8Presence1L) && (u8PresenceSafety1L == 0xA))) &&
    		    (((cFalse == u8Presence2L) && (u8PresenceSafety2L == 0x5)) || ((cTrue == u8Presence2L) && (u8PresenceSafety2L == 0xA))) )
    		{
    			//Do Nothing
    		}
    		else
    		{
    			RI_SRV_Safety_ManageSafetyCountRL3A(eRL3A_NVM);
    		}
        }
#endif
        (void)RI_Write_CMB_NIVL_VOY(cTrue);
        (void)RI_Write_CMB_REF_DEF_VOY(cTrue);
        (void)RI_Write_CMB_FRPK_VOY(cTrue);
        //ajoshi8: Task 478509 : Check explicitly the presence value of both parameters from NvM
        if( ((uint8)0U == u8Presence1L) && ((uint8)1U == u8Presence2L)  )  // From NvM : TELLTALES_ELEC_PARK_BRAKE = 0 && TELLTALES_BRAKE_FAILURE = 1
        {
            if((eParkBrake_Applied == CANFRPKL) || (eBrakeFluidLevel_Warn == CANNIVL_ALL) || (eBrakeDistrFault_Fault == CANREF_DEFL) || (bInfoBrakeDistrFltGate == cTrue))
            {
                TTStateL = TelltaleStatus_On;
            }
            else
            {
                TTStateL = TelltaleStatus_Off;
            }
            (void)RI_Write_CMB_FSE_SER_DEF_VOY(cFalse);
        }
        //ajoshi8: Task 478509 : Check explicitly the presence value of both parameters from NvM
        else if( ((uint8)1U == u8Presence1L) && ((uint8)1U == u8Presence2L) )   // From NvM :  TELLTALES_ELEC_PARK_BRAKE = 1 && TELLTALES_BRAKE_FAILURE = 1
        {
            if((eBrakeFluidLevel_Warn == CANNIVL_ALL) || (eBrakeDistrFault_Fault == CANREF_DEFL) || (bInfoBrakeDistrFltGate == cTrue))
            {
                TTStateL = TelltaleStatus_On;
            }
            else
            {
                TTStateL = TelltaleStatus_Off;
            }
            (void)RI_Write_CMB_FSE_SER_DEF_VOY(cTrue);
        }
        else
        {
        	//no action
        }
    }

    return TTStateL;
}

//------------------------------------------------------------------------------
/// DESCRIPTION:         Implementation of Euro6 telltale control
///
/// PARAMETERS:          \param TTidP: telltale id
///
/// RETURN VALUE:        \return TelltaleStatusType - On, Off, InAlarm
//------------------------------------------------------------------------------
TelltaleStatusType UpdateEuro6(const TelltaleIdType TTidP)
{
    ComExtSignalStateType                CANStateUnusedL  = 0U;
    //variable to read Requests lighting of indicator NOX
    tCDE_CMB_SIGNALISATION_DMD_ALLUM_SCR CANDMD_ALLUM_SCRL  = 0U;
    TelltaleStatusType                   TTStateL = TelltaleStatus_Off;

    (void)RI_Read_CDE_CMB_SIGNALISATION_DMD_ALLUM_SCR(&CANDMD_ALLUM_SCRL, &CANStateUnusedL);

    if(eEuro6_Blinking == CANDMD_ALLUM_SCRL)
    {
         TTStateL = TelltaleStatus_Warning;
    }
    else if(eEuro6_Continuous == CANDMD_ALLUM_SCRL)
    {
        TTStateL = TelltaleStatus_On;
    }
    else
    {
        //Nothing to do
    }

    return TTStateL;
}

//------------------------------------------------------------------------------
/// DESCRIPTION:         Implementation of Battery telltale control
///
/// PARAMETERS:          \param TTidP: telltale id
///
/// RETURN VALUE:        \return TelltaleStatusType - On, Off, InAlarm
//------------------------------------------------------------------------------
TelltaleStatusType UpdateBatteryTelltale(const TelltaleIdType TTidP)
{
    uint8 CANStateL  = 0U;
    uint8 u8VAR_ALT_FAULTL  = 0U;
    boolean CANCBAT_DEFL  = 0U;
    boolean CANGENE_DEFL  = 0U;
    TelltaleStatusType TTStateL = TelltaleStatus_Off;

    //Check for alternator fault
    (void)MDL_Telltale_Read_Boolean_SafetyCritical_Signal(&CANGENE_DEFL, &CANStateL, ALTERNATOR_FLT);

    if((eAltFlt_Fault == CANGENE_DEFL) || (bInfoAltFltSecureGate == cTrue))
    {
        u8VAR_ALT_FAULTL = cTrue;
    }
    else
    {
        u8VAR_ALT_FAULTL = cFalse;
    }

    (void)MDL_Telltale_Read_Boolean_SafetyCritical_Signal(&CANCBAT_DEFL, &CANStateL, BATT_CHARG_FLT);

    if((u8VAR_ALT_FAULTL == cTrue) || (CANCBAT_DEFL == eBattFlt_Fault))
    {
        TTStateL = TelltaleStatus_On;
    }
    else
    {
        TTStateL = TelltaleStatus_Off;
    }

    return TTStateL;
}

//------------------------------------------------------------------------------
/// DESCRIPTION:         function to control the status of service telltale
///
/// PARAMETERS:          \param TTidP: telltale id
///
/// RETURN VALUE:        \return TelltaleStatusType - On, Off, InAlarm
//------------------------------------------------------------------------------
TelltaleStatusType UpdateServiceTelltale(const TelltaleIdType TTidP)
{
    TelltaleStatusType TTStateL = TelltaleStatus_Off;
    uint8 CANSERVICEL  = 0U;
    uint8 CANStateL  = 0U;
    uint8 u8SteerFaultPresence  = 0U;
    uint8 u8PresenceSafetyL  = 0U;
#ifdef VIP_SAFETY_ENABLE
    boolean IsSafety = cFalse;
#endif

    (void)RI_ReadHwId(TelltaleId_TELLTALES_POWER_STEER_FLT, &u8SteerFaultPresence);
#ifdef VIP_SAFETY_ENABLE

        //Check if Telltale is Safety related
        IsSafety = CheckIfSafetyTelltale( TelltaleId_TELLTALES_POWER_STEER_FLT);
        if(IsSafety == cTrue)
        {
    		(void)RI_ReadHwIdSafety(TelltaleId_TELLTALES_POWER_STEER_FLT, &u8PresenceSafetyL);
    		if(((cFalse == u8SteerFaultPresence) && (u8PresenceSafetyL == 0x5)) ||
    				((cTrue == u8SteerFaultPresence) || (u8PresenceSafetyL == 0xA)))
    		{
    			//Do Nothing
    		}
    		else
    		{
    			RI_SRV_Safety_ManageSafetyCountRL3A(eRL3A_NVM);
    		}
        }
#endif
    (void)MDL_Telltale_Read_Boolean_SafetyCritical_Signal(&CANSERVICEL, &CANStateL, SERVICE);

    if((CANSERVICEL == (uint8)cOne)|| ((u8SteerFaultPresence == cZero) && (bPowerSteerORGate == (boolean)cOne)))
    {
        TTStateL = TelltaleStatus_On;
    }
    else
    {
        //Nothing to do
    }

    return TTStateL;
}

//------------------------------------------------------------------------------
/// DESCRIPTION:         function to calculate the checksum and counter validity
///
/// PARAMETERS:          None
///
/// RETURN VALUE:        None
//------------------------------------------------------------------------------
//LOCF 139: mravat: Levels is < 6 .Hence not an issue, and no need to split the function
static void CalculateCounterAndchecksum(void)
{
    uint8 u8ChecksumL  = 0U;
    uint8 u8VarCanReqAltFltL  = cFalse;
    uint8 CANCPT_PROCESS_4B_UC_FREINL  = 0U;
    uint8 CANETAT_PRINCIP_SEVL  = 0U;
    uint8 CANDEFAUT_GEE_UCFREINL  = 0U;
    uint8 CANStateL  = 0U;

    //Read ETAT_PRINCIP_SEV
    (void)MDL_Telltale_Read_uint8_SafetyCritical_Signal(&CANETAT_PRINCIP_SEVL, &CANStateL, VHL_ELEC_SYSTEM);

    if((eStatusSEV_Contact == CANETAT_PRINCIP_SEVL) || (eStatusSEV_Start == CANETAT_PRINCIP_SEVL))
    {

        //Calculate Process counter filter
        (void)RI_Read_IS_DAT_ABR_CPT_PROCESS_4B_UC_FREIN(&CANCPT_PROCESS_4B_UC_FREINL, &CANStateL);


        if(ComExtSignalState_New == CANStateL)
        {
            //calculate and update checksum only if data is refreshed
            RI_Read_ComputedFrameCks_IS_DAT_AB(&u8ChecksumL);


            // SET u8ChecksumL = (u8ChecksumL + cCHKini) MOD cMOD_HEX;
            u8ChecksumL = (u8ChecksumL + cCHKini) & cMOD_HEX;

            //Calculation of checksum filter
            //Check if the calculated filter value is valid
            if(cVALID_CHKSUM == u8ChecksumL)
            {
                u16ChecksumFilter = cMdl_telltale_Safety_cTrue;

                //reset the invalid checksum count
                u8IS_DAT_ABRFrameCountCheckSum = (uint8)cZero;
#ifdef VIP_SAFETY_ENABLE
                Mirror_u8IS_DAT_ABRFrameCountCheckSum = (uint8)(~u8IS_DAT_ABRFrameCountCheckSum);
#endif
            }
             else
             {
#ifdef VIP_SAFETY_ENABLE
                 //CCOV:apatil19: Can't covered the True case as it's Safety implementation and reached only when memory corruption
                 //Memory corruption check for  u8IS_DAT_ABRFrameCountCheckSum
                 if(u8IS_DAT_ABRFrameCountCheckSum != (uint8)(~Mirror_u8IS_DAT_ABRFrameCountCheckSum))
                 {
                	 RI_SRV_Safety_ManageSafetyCountRL3A(eRL3A_NVM);
                 }
#endif

                //if checksum is invalid two consecutive times the set the filtered value to invalid and reset the frame count
                if(cMAX_NO_INVALID_CHKSUM_FRAME == u8IS_DAT_ABRFrameCountCheckSum)
                {
                    u16ChecksumFilter = cMdl_telltale_Safety_cFalse;
                }
                else
                {
                    //increment the checksum counter as received checksum value is invalid
                    u8IS_DAT_ABRFrameCountCheckSum++;
#ifdef VIP_SAFETY_ENABLE
                    Mirror_u8IS_DAT_ABRFrameCountCheckSum = (uint8)(~u8IS_DAT_ABRFrameCountCheckSum);
#endif
                }
             }
#ifdef VIP_SAFETY_ENABLE
                //Memory corruption check for  u8ProcessCounterValue
                //CCOV:apatil19: Can't covered the True case as it's Safety implementation and reached only when memory corruption
             if(u8ProcessCounterValue != (uint8)(~Mirror_u8ProcessCounterValue))
             {
            	 RI_SRV_Safety_ManageSafetyCountRL3A(eRL3A_NVM);
             }
#endif

            if(u8ProcessCounterValue == CANCPT_PROCESS_4B_UC_FREINL)
            {
#ifdef VIP_SAFETY_ENABLE
                //Memory corruption check for  u8IS_DAT_ABRFrameCountProcessCounter
                //CCOV:apatil19: Can't covered the True case as it's Safety implementation and reached only when memory corruption
                if(u8IS_DAT_ABRFrameCountProcessCounter != (uint8)(~Mirror_u8IS_DAT_ABRFrameCountProcessCounter))
                {
                	RI_SRV_Safety_ManageSafetyCountRL3A(eRL3A_NVM);
                }
#endif
                if(cMAX_NO_INVALID_COUNTER_FRAME <= u8IS_DAT_ABRFrameCountProcessCounter)
                {
                    u16ProcessCounterFilter = cMdl_telltale_Safety_cFalse;
                }
                else
                {
                    u8IS_DAT_ABRFrameCountProcessCounter++;
#ifdef VIP_SAFETY_ENABLE
                    Mirror_u8IS_DAT_ABRFrameCountProcessCounter = (uint8)(~u8IS_DAT_ABRFrameCountProcessCounter);
#endif
                }
            }
            else
            {
                u16ProcessCounterFilter = cMdl_telltale_Safety_cTrue;
                u8ProcessCounterValue = CANCPT_PROCESS_4B_UC_FREINL;
                u8IS_DAT_ABRFrameCountProcessCounter = (uint8)cZero;
#ifdef VIP_SAFETY_ENABLE
                Mirror_u8ProcessCounterValue = (uint8)(~u8ProcessCounterValue);
                Mirror_u8IS_DAT_ABRFrameCountProcessCounter = (uint8)(~u8IS_DAT_ABRFrameCountProcessCounter);
#endif
            }
        }
        else if (ComExtSignalState_ConfAbsent == CANStateL)
        {
            u16ProcessCounterFilter = cMdl_telltale_Safety_cFalse;
            u16ChecksumFilter = cMdl_telltale_Safety_cFalse;
        }
        else
        {
            //do nothing
        }

        //Calculate UC Frien failure info
        (void)RI_Read_IS_DAT_ABR_DEFAUT_GEE_UCFREIN(&CANDEFAUT_GEE_UCFREINL, &CANStateL);

        if(eGeeFault_Fault == CANDEFAUT_GEE_UCFREINL)
        {
            u8VarCanReqAltFltL = cTrue;
        }
        else
        {
            //Do nothing, u8VarCanReqAltFltL has already set to false
        }

    }
    else
    {
        u16ChecksumFilter = cMdl_telltale_Safety_cTrue;
        u16ProcessCounterFilter = cMdl_telltale_Safety_cTrue;
    }

    if((u16ChecksumFilter == cMdl_telltale_Safety_cTrue) && (u16ProcessCounterFilter == cMdl_telltale_Safety_cTrue) && (u8VarCanReqAltFltL == cTrue))
    {
        bInfoAltFltSecureGate = cTrue;
    }
    else
    {
        bInfoAltFltSecureGate = cFalse;
    }
#ifdef VIP_SAFETY_ENABLE
    //CCOV:apatil19: Can't covered the True case as it's Safety implementation and reached only when memory corruption
    if((u16ChecksumFilter != cMdl_telltale_Safety_cFalse) && (u16ChecksumFilter != cMdl_telltale_Safety_cTrue))
    {
    	RI_SRV_Safety_ManageSafetyCountRL3A(eRL3A_NVM);
    }

    //CCOV:apatil19: Can't covered the True case as it's Safety implementation and reached only when memory corruption
    if((u16ProcessCounterFilter != cMdl_telltale_Safety_cFalse) && (u16ProcessCounterFilter != cMdl_telltale_Safety_cTrue))
    {
    	RI_SRV_Safety_ManageSafetyCountRL3A(eRL3A_NVM);
    }
#endif
}
//------------------------------------------------------------------------------
/// DESCRIPTION:         function to calculate the power steering fault
///
/// PARAMETERS:          None
///
/// RETURN VALUE:        None
//------------------------------------------------------------------------------
static void CalculatePowersteerFault(void)
{
    uint16 SteeringTimerValL = 0U;
    uint16 SteeringTimerValSafetyL = 0U;
    uint8 CANETAT_PRINCIP_SEVL = 0U;
    uint8 CANStateL = 0U;
    uint8 CANETAT_GMPL = 0U;
    uint8 CANSteeringFlt1 = 0U;
    uint8 CANSteeringFltType = 0U;
    static uint16 u16EtatGmpFlagL = cMdl_telltale_Safety_cFalse;
    boolean bIsTimerStartedL = cFalse;
    boolean bUBPDATE_FLT_1 = cFalse;
    boolean bUBPDATE_FLT = cFalse;
    boolean bIsTimerElapsedL = cFalse;

    (void)RI_Read_CAN_POWER_STEERING_TIMEOUT(&SteeringTimerValL);
    (void)RI_Read_CAN_POWER_STEERING_TIMEOUT_SAFETY(&SteeringTimerValSafetyL);

    if((SteeringTimerValL ^ SteeringTimerValSafetyL) != 0xFFFF)
    {
    	SteeringTimerValL = SteeringTimerValL ^ SteeringTimerValSafetyL;
    	RI_SRV_Safety_ManageSafetyCountRL3A(eRL3A_NVM);
    }

    (void)MDL_Telltale_Read_uint8_SafetyCritical_Signal(&CANETAT_PRINCIP_SEVL, &CANStateL, VHL_ELEC_SYSTEM);
    (void)MDL_Telltale_Read_uint8_SafetyCritical_Signal(&CANETAT_GMPL, &CANStateL, STATE_ENGINE);

    if(CANETAT_GMPL != eEngineNotRunning)
    {
        if(u16EtatGmpFlagL == cMdl_telltale_Safety_cFalse)
        {
#ifdef VIP_SAFETY_ENABLE
        	if(Mirror_EngineRunningTimer != (TimerType)(~EngineRunningTimer))
              {
        			RI_SRV_Safety_ManageSafetyCountRL3A(eRL3A_NVM);
              }
#endif
              (void)OSIsTimerStarted(EngineRunningTimer,&bIsTimerStartedL);
              if(bIsTimerStartedL == cTrue)
              {
                   //Check if the running timer is already elapsed
                  (void)OSIsTimerElapsed(EngineRunningTimer, SteeringTimerValL, &bIsTimerElapsedL);

                  if(bIsTimerElapsedL == cTrue)
                  {
                      u16EtatGmpFlagL = cMdl_telltale_Safety_cTrue;
                      (void)OSTimerStop(&EngineRunningTimer);
#ifdef VIP_SAFETY_ENABLE
                      Mirror_EngineRunningTimer = (TimerType)(~EngineRunningTimer); //Safety implementation
#endif
                  }
                  else
                  {
                       u16EtatGmpFlagL = cMdl_telltale_Safety_cFalse;
                  }
              }
              else
              {
                  (void)OSTimerStart(&EngineRunningTimer);
#ifdef VIP_SAFETY_ENABLE
                  Mirror_EngineRunningTimer = (TimerType)(~EngineRunningTimer); //Safety implementation
#endif
              }
        }
        else if(u16EtatGmpFlagL == cMdl_telltale_Safety_cTrue)
        {
            //do nothing, extra check for safety requirement
        }
        else
        {
#ifdef VIP_SAFETY_ENABLE
        	RI_SRV_Safety_ManageSafetyCountRL3A(eRL3A_NVM);
#endif
        }
    }
    else
    {
        u16EtatGmpFlagL = cMdl_telltale_Safety_cFalse;
        (void)OSTimerStop(&EngineRunningTimer);
#ifdef VIP_SAFETY_ENABLE
        Mirror_EngineRunningTimer = (TimerType)(~EngineRunningTimer); //Safety implementation
#endif
    }

   if(( u16EtatGmpFlagL == cMdl_telltale_Safety_cTrue) && ((eStatusSEV_Contact == CANETAT_PRINCIP_SEVL) || (eStatusSEV_Start == CANETAT_PRINCIP_SEVL)))
   {
       (void)MDL_Telltale_Read_Boolean_SafetyCritical_Signal(&bUBPDATE_FLT_1, &CANStateL, UBPDATE_FLT_1);
       (void)MDL_Telltale_Read_Boolean_SafetyCritical_Signal(&bUBPDATE_FLT, &CANStateL, UBPDATE_FLT_TYPE);

       if((bUBPDATE_FLT == eDataNotRefreshed) || (bUBPDATE_FLT_1 == eNotRefreshed))
       {
           bPowerSteerORGate = cFalse;
           bPowerSteerREDGate = cTrue;
       }
       else
       {
           (void)MDL_Telltale_Read_uint8_SafetyCritical_Signal(&CANSteeringFlt1, &CANStateL, STEERING_FLT_1);
           (void)MDL_Telltale_Read_uint8_SafetyCritical_Signal(&CANSteeringFltType, &CANStateL, STEERING_FLT_TYPE);

           if((CANSteeringFltType == eG4DAFlt) || (CANSteeringFltType == eG3G4DAFlt) ||
              (CANSteeringFlt1 == eRequestRed) || (CANSteeringFlt1 == eReserved))
           {
             bPowerSteerORGate = cFalse;
             bPowerSteerREDGate = cTrue;
           }
           else if(((eNoRequest == CANSteeringFlt1) && (eG3DAFlt == CANSteeringFltType)) ||
                   ((CANSteeringFlt1 == eRequestOrange) && ((CANSteeringFltType == eNoDefault) || (CANSteeringFltType == eG3DAFlt))))
           {
                bPowerSteerORGate = cTrue;
                bPowerSteerREDGate = cFalse;
           }
           else
           {
               bPowerSteerORGate = cFalse;
               bPowerSteerREDGate = cFalse;
           }
       }
   }
   else
   {
      bPowerSteerORGate = cFalse;
      bPowerSteerREDGate = cFalse;
   }
}

//------------------------------------------------------------------------------
/// DESCRIPTION:         function to calculate the brake distr. fault
///
/// PARAMETERS:          \param TTidP: telltale id
///
/// RETURN VALUE:        \return TelltaleStatusType - On, Off, InAlarm
//------------------------------------------------------------------------------
//LOCF 139: mravat: Levels is < 6 .Hence not an issue, and no need to split the function
static void CalculateBrakeDistrFault(void)
{
    uint16 u16BrakeDistrTimeoutL = 0U;
    uint16 u16BrakeDistrTimeoutSafetyL = 0U;
    uint16 u16BrakeDistrEngRunningTimeoutL = 0U;
    uint16 u16BrakeDistrEngRunningTimeoutSafetyL = 0U;
    uint8 u8EtatPrincipSevL = 0U;
    uint8 u8CanStateL = 0U;
    uint8 u8EtatGmpL = 0U;
    boolean bBrakeFltL = cFalse;
    boolean bIsTimerStartedL = cFalse;
    boolean bIsTimerElapsedL = cFalse;
    boolean bvarBrakeDistrFltL = cFalse;
    static uint16 u16BrakeDistrFlagL = cMdl_telltale_Safety_cFalse;
    static uint16 u16EngineStateL = cMdl_telltale_Safety_cFalse;

    (void)RI_Read_IS_DAT_ABR_REQ_LAMPE_DEF_REF(&bBrakeFltL, &u8CanStateL);
    (void)MDL_Telltale_Read_uint8_SafetyCritical_Signal(&u8EtatGmpL, &u8CanStateL, STATE_ENGINE);
    if(bBrakeFltL == cTrue)
    {
        if(u16BrakeDistrFlagL == cMdl_telltale_Safety_cFalse)
        {
#ifdef VIP_SAFETY_ENABLE
            //CCOV:apatil19: Can't covered the True case as it's Safety implementation and reached only when memory corruption
            if(BrakeDistrFltTimer != (TimerType)(~Mirror_BrakeDistrFltTimer))
            {
            	RI_SRV_Safety_ManageSafetyCountRL3A(eRL3A_NVM);
            }
#endif
            (void)OSIsTimerStarted(BrakeDistrFltTimer,&bIsTimerStartedL);
            if(bIsTimerStartedL == cTrue)
            {
                (void)RI_ReadCAN_REQ_BRAKE_DISTR_FLT(&u16BrakeDistrTimeoutL);
                (void)RI_ReadCAN_REQ_BRAKE_DISTR_FLT_SAFETY(&u16BrakeDistrTimeoutSafetyL);

                if((u16BrakeDistrTimeoutL  ^ u16BrakeDistrTimeoutSafetyL) != 0xFFFF)
                {
                	RI_SRV_Safety_ManageSafetyCountRL3A(eRL3A_NVM);
                }

                (void)OSIsTimerElapsed(BrakeDistrFltTimer, u16BrakeDistrTimeoutL, &bIsTimerElapsedL);
                if(bIsTimerElapsedL == cTrue)
                {
                    (void)OSTimerStop(&BrakeDistrFltTimer);
                    u16BrakeDistrFlagL = cMdl_telltale_Safety_cTrue;
#ifdef VIP_SAFETY_ENABLE
                    Mirror_BrakeDistrFltTimer = (TimerType)(~BrakeDistrFltTimer); //safety implementation
#endif
                }
                else
                {
                    u16BrakeDistrFlagL = cMdl_telltale_Safety_cFalse;
                }
            }
            else
            {

                (void)OSTimerStart(&BrakeDistrFltTimer);
#ifdef VIP_SAFETY_ENABLE
                Mirror_BrakeDistrFltTimer = (TimerType)(~BrakeDistrFltTimer); //safety implementation
#endif
            }
        }
        else if (u16BrakeDistrFlagL == cMdl_telltale_Safety_cTrue)
        {
            //do nothing, extra check for safety requirement
        }
        else
        {
#ifdef VIP_SAFETY_ENABLE
        	//CCOV:apatil19: Can't covered this path as it's Safety implementation and reached only when memory corruption
        	(void)RI_SRV_Safety_ManageSafetyCountRL3A(eRL3A_NVM);//MDL_Telltale_SD_19
#endif
        }
    }
    else
    {
        u16BrakeDistrFlagL = cMdl_telltale_Safety_cFalse;
        (void)OSTimerStop(&BrakeDistrFltTimer);
#ifdef VIP_SAFETY_ENABLE
        Mirror_BrakeDistrFltTimer = (TimerType)(~BrakeDistrFltTimer); //safety implementation
#endif
    }
    if((u8EtatGmpL != eEngineNotRunning))
    {
        if((u16EngineStateL == cMdl_telltale_Safety_cFalse))
        {
#ifdef VIP_SAFETY_ENABLE
        	//CCOV:apatil19: Can't covered the True case as it's Safety implementation and reached only when memory corruption
            if(BrakeDistrFltEngineRunningTimer != (TimerType)(~Mirror_BrakeDistrFltEngineRunningTimer))
            {
            	RI_SRV_Safety_ManageSafetyCountRL3A(eRL3A_NVM);
            }
#endif
            (void)OSIsTimerStarted(BrakeDistrFltEngineRunningTimer, &bIsTimerStartedL);
            if(bIsTimerStartedL == cTrue)
            {
                (void)RI_Read_ENGINE_STATE_TIMEOUT(&u16BrakeDistrEngRunningTimeoutL);
                (void)RI_Read_ENGINE_STATE_TIMEOUT_SAFETY(&u16BrakeDistrEngRunningTimeoutSafetyL);

                if((u16BrakeDistrEngRunningTimeoutSafetyL ^ u16BrakeDistrEngRunningTimeoutL) != 0xFFFF)
                {
                	RI_SRV_Safety_ManageSafetyCountRL3A(eRL3A_NVM);
                }

                (void)OSIsTimerElapsed(BrakeDistrFltEngineRunningTimer, u16BrakeDistrEngRunningTimeoutL, &bIsTimerElapsedL);
                if(bIsTimerElapsedL == cTrue)
                {
                    (void)OSTimerStop(&BrakeDistrFltEngineRunningTimer);
#ifdef VIP_SAFETY_ENABLE
                    Mirror_BrakeDistrFltEngineRunningTimer = (TimerType)(~BrakeDistrFltEngineRunningTimer); //safety implementation
#endif
                    u16EngineStateL = cMdl_telltale_Safety_cTrue;
                }
                else
                {
                    u16EngineStateL = cMdl_telltale_Safety_cFalse;
                }
            }
            else
            {
                (void)OSTimerStart(&BrakeDistrFltEngineRunningTimer);
#ifdef VIP_SAFETY_ENABLE
                Mirror_BrakeDistrFltEngineRunningTimer = (TimerType)(~BrakeDistrFltEngineRunningTimer); //safety implementation
#endif
            }
        }
        else if(u16EngineStateL == cMdl_telltale_Safety_cTrue)
        {
            //do nothing, extra check for safety requirement
        }
        else
        {
#ifdef VIP_SAFETY_ENABLE
        	RI_SRV_Safety_ManageSafetyCountRL3A(eRL3A_NVM);
#endif
        }
    }
    else
    {
        u16EngineStateL = cMdl_telltale_Safety_cFalse;
        (void)OSTimerStop(&BrakeDistrFltEngineRunningTimer);
#ifdef VIP_SAFETY_ENABLE
        Mirror_BrakeDistrFltEngineRunningTimer = (TimerType)(~BrakeDistrFltEngineRunningTimer); //safety implementation
#endif
    }

    //Read ETAT_PRINCIP_SEV
    (void)MDL_Telltale_Read_uint8_SafetyCritical_Signal(&u8EtatPrincipSevL, &u8CanStateL, VHL_ELEC_SYSTEM);
    //Implementation of Req SysRS_14_Telltales-3403
    if((eStatusSEV_Contact == u8EtatPrincipSevL) || (eStatusSEV_Start == u8EtatPrincipSevL))
    {
        if((u16BrakeDistrFlagL == cMdl_telltale_Safety_cTrue)  && (u16EngineStateL == cMdl_telltale_Safety_cTrue) )
        {
            bvarBrakeDistrFltL = cTrue;
        }
        else
        {
            //Nothing to do
        }
    }
    else
    {
        //Nothing to do
    }

    //Implementation of Req SysRS_14_Telltales-3404
    if((u16ChecksumFilter == cMdl_telltale_Safety_cTrue) && (u16ProcessCounterFilter == cMdl_telltale_Safety_cTrue) && (bvarBrakeDistrFltL == cFalse))
    {
        bInfoBrakeDistrFltGate = cINVALID;
    }
    else
    {
        bInfoBrakeDistrFltGate = cVALID;
    }
}


//------------------------------------------------------------------------------
/// DESCRIPTION:         function to update stop telltale
///
/// PARAMETERS:          \param TTidP: telltale id
///
/// RETURN VALUE:        \return TelltaleStatusType - On, Off, InAlarm
//------------------------------------------------------------------------------
TelltaleStatusType UpdateStopTelltale(const TelltaleIdType TTidP)
{
    TelltaleStatusType TTStateL = TelltaleStatus_Off;
    uint8 CANSTOPL  = 0U;
    uint8 CANStateL  = 0U;

    (void)MDL_Telltale_Read_Boolean_SafetyCritical_Signal(&CANSTOPL, &CANStateL, STOP);

    if((CANSTOPL == (uint8)cOne) || (bPowerSteerREDGate == cTrue) || (bInfoAltFltSecureGate == cTrue) || (bInfoBrakeDistrFltGate == cTrue))
    {
        TTStateL = TelltaleStatus_On;
    }

    return TTStateL;
}

//------------------------------------------------------------------------------
/// DESCRIPTION:         function to update status of airbag inhibition telltale
///
/// PARAMETERS:          \param TTidP: telltale id
///
/// RETURN VALUE:        \return TelltaleStatusType - On, Off, InAlarm
//------------------------------------------------------------------------------
static void SendAirBagPassInhibitionCanOut (void)
{
	uint8 u8PresenceL = (uint8)0U;

#ifdef VIP_SAFETY_ENABLE
	uint8 u8PresenceSafetyL = (uint8)0U;
    boolean IsSafety = cFalse;
#endif


	(void)RI_ReadHwId(TelltaleId_TELLTALES_AIRBAG_INHIB, &u8PresenceL);
#ifdef VIP_SAFETY_ENABLE

        //Check if Telltale is Safety related
        IsSafety = CheckIfSafetyTelltale( TelltaleId_TELLTALES_AIRBAG_INHIB);
        if(IsSafety == cTrue)
        {
    		(void)RI_ReadHwIdSafety(TelltaleId_TELLTALES_AIRBAG_INHIB, &u8PresenceSafetyL);
    		if(((cFalse == u8PresenceL) && (u8PresenceSafetyL == 0x5)) ||
    				((cTrue == u8PresenceL) || (u8PresenceSafetyL == 0xA)))
    		{
    			//Do Nothing
    		}
    		else
    		{
    			RI_SRV_Safety_ManageSafetyCountRL3A(eRL3A_NVM);
    		}
        }
#endif

	if(u8PresenceL != cFalse)
	{
		//SysRS_14_Telltales-4860
		(void)RI_Write_CMB_ABPI_VOY(cTrue);
		//ajoshi8: Handling this as per feedback from System Engineer for Defect 390386, since this case is not mentioned in SysRS_14 document
		(void)RI_Write_ETAT_COMBINE_IHM_INHIB(cINHIB_NoSignaling);
	}
	else
	{
		//SysRS_14_Telltales-4860
		(void)RI_Write_CMB_ABPI_VOY(cFalse);
		//ajoshi8: Defect 390386 : Need to send the CAN signal IHM_INHIB = 0 when DAT_DG_TELLTALES_AIRBAG_INHIB = 0
		//SysRS_14_Telltales-2414
		(void)RI_Write_ETAT_COMBINE_IHM_INHIB(cINHIB_Indefine);
	}
}

//------------------------------------------------------------------------------
/// DESCRIPTION:         function to update status of airbag telltale
///
/// PARAMETERS:          \param TTidP: telltale id
///
/// RETURN VALUE:        \return TelltaleStatusType - On, Off, InAlarm
//------------------------------------------------------------------------------
static void SendAirBagDefectTTStatus (boolean bTelltaleStatusP, boolean bTelltalePresentP)
{

#if(PLATFORM_VARIANT == VARIANT_L2)
	TelltaleStatusType bOutTTStatusL = bTelltaleStatusP;
#endif
    Dcm_SesCtrlType Session = 0U;
    uint8 u8CanOutL = cINHIB_Indefine;
    if(bTelltalePresentP != cFalse)
    {
        (void)RI_GetSesCtrlType(&Session);
        if(Session != cSession_Diag)
        {
#if(PLATFORM_VARIANT == VARIANT_L2)
            (void)RI_GetTelltaleSoftwareState(TelltaleId_TELLTALES_PASS_SAF_FLT, &bOutTTStatusL);
            //Code for L2, requirement Id SysRS_14_Telltales-2307
            if(bOutTTStatusL != TelltaleStatus_Off)
            {
                u8CanOutL = cINHIB_Signaling;
            }
            else
            {
                u8CanOutL = cINHIB_NoSignaling;
            }
#endif
#if(PLATFORM_VARIANT == VARIANT_L1_LCD_TEXT)
    //For L1 text the functionality will be implemented in L1 specific file
    //The function should implement req. SysRS_14_Telltales-6929 for L1 and return CAN signal SIGNAL_DEF_AIRBAG value as per requirement
                u8CanOutL = RI_L1_SendAirBagDefectTTStatus(bTelltaleStatusP,bTelltalePresentP);
#endif

        }
        else
        {
            //For Requirement SysRS_14_Telltales-2318
            if(bTelltaleStatusP != cFalse)
            {
                u8CanOutL = cINHIB_Signaling;
            }
            else
            {
                u8CanOutL = cINHIB_NoSignaling;
            }
        }
    }
    else
    {
        //Nothing to do
    }

    (void)RI_Write_ETAT_COMBINE_SIGNAL_DEF_AIRBAG(u8CanOutL);
}

//------------------------------------------------------------------------------
/// DESCRIPTION:         function to update lane keeping assist telltale
///
/// PARAMETERS:          \param TTidP: telltale id
///
/// RETURN VALUE:        \return TelltaleStatusType - On, Off, InAlarm
//------------------------------------------------------------------------------
#define cLkaTelltaleState_Off               ((uint8)0x00)
#define cLkaTelltaleState_OnFixedNotReady   ((uint8)0x01)
#define cLkaTelltaleState_OnBlinking        ((uint8)0x02)
#define cLkaTelltaleState_OnFixedReady      ((uint8)0x03)
TelltaleStatusType UpdateLKATelltale(const TelltaleIdType TTidP)
{
	uint8 u8LKAL  = 0U;
	uint8 u8CANState  = 0U;
	TelltaleStatusType TTStateL = LKATelltaleState_Off;

	(void)RI_Read_ETAT_ARTIV_ET_HY_LKA_STATE_INDICATOR(&u8LKAL, &u8CANState);

    switch(u8LKAL)
    {
        case cLkaTelltaleState_OnBlinking:
            TTStateL = LKATelltaleState_Warning;
            break;

        case cLkaTelltaleState_OnFixedReady:
            TTStateL = LKATelltaleState_OnReady;
            break;

        case cLkaTelltaleState_OnFixedNotReady:
            TTStateL = LKATelltaleState_OnNotReady;
            break;

         default: // cLkaTelltaleState_Off
            TTStateL = LKATelltaleState_Off;
            break;
    }

	return TTStateL;
}

//------------------------------------------------------------------------------
/// DESCRIPTION:         function to update Active Blind Spot Detection telltale
///
/// PARAMETERS:          \param TTidP: telltale id
///
/// RETURN VALUE:        \return TelltaleStatusType - On, Off, InAlarm
//------------------------------------------------------------------------------
#define cABSDTelltaleState_Off               ((uint8)0x00)
#define cABSDTelltaleState_OnFixedNotReady   ((uint8)0x01)
#define cABSDTelltaleState_OnBlinking        ((uint8)0x02)
#define cABSDTelltaleState_OnFixedReady      ((uint8)0x03)

TelltaleStatusType UpdateABSDTelltale(const TelltaleIdType TTidP)
{
    uint8 u8CANSigL  = (uint8)0x0U;
    uint8 u8CANStateL  = (uint8)0x0U;
    TelltaleStatusType TTStateL = TelltaleStatus_Off;

    (void)RI_Read_ABSD_STATE_INDICATOR(&u8CANSigL, &u8CANStateL);
    if(u8CANSigL == cABSDTelltaleState_OnBlinking)
    {
        TTStateL = TelltaleStatus_Warning;
    }
    else if ((u8CANSigL == cABSDTelltaleState_OnFixedNotReady) || (u8CANSigL == cABSDTelltaleState_OnFixedReady))
    {
        TTStateL = TelltaleStatus_On;
    }
    else
    {
        //Nothing to do
    }

    return TTStateL;
}

//------------------------------------------------------------------------------
/// DESCRIPTION:         function to update hill assist descent control telltale
///
/// PARAMETERS:          \param TTidP: telltale id
///
/// RETURN VALUE:        \return TelltaleStatusType - On, Off, InAlarm
//------------------------------------------------------------------------------
TelltaleStatusType UpdateHillAssistDescentControl(const TelltaleIdType TTidP)
{
//ajoshi8: SysRS_14_Telltales-6134 is set as NA for D3 variant in 'BL_P2D3_C5.1.2.0' of SysRS14_Manage_Telltales

	TelltaleStatusType TTStateL = TelltaleStatus_Off;

	Boolean LvdsOutPresence = mDATRead(U1Bit, LVDS_OUT_PRESENCE, Default);
	Boolean LvdsInPresence = mDATRead(U1Bit, LVDS_IN_PRESENCE, Default);

	if( (LvdsInPresence == cTrue) && (LvdsOutPresence == cFalse))  // for P2 variant
	{
		uint8 u8HADCFeedback  = 0U;
		uint8 u8HADCPush  = 0U;
		uint8 u8CANState  = 0U;

		(void)RI_Read_CDE_COMBINE_TEMOINS_P_TEM_HADC_FEEDBACK(&u8HADCFeedback , &u8CANState);
		(void)RI_Read_Telltale_CDE_LED_PUSH_P_LED_HADC_PUSH(&u8HADCPush, &u8CANState);

		if(u8HADCFeedback == eHADCFeedbackSlowBlink)
		{
			TTStateL = TelltaleStatus_SlowWarning;
		}
		else if(u8HADCFeedback == eHADCFeedbackBlink)
		{
			TTStateL = TelltaleStatus_Warning;
		}
		else if(u8HADCFeedback == eHADCFeedbackOn)
		{
			TTStateL = TelltaleStatus_On;
		}
		else if(u8HADCFeedback == eHADCFeedbackOff)
		{
			if(u8HADCPush == cTrue)
			{
			    TTStateL = TelltaleStatus_OnSelect;
			}
			else
			{
			    TTStateL = TelltaleStatus_Off;
			}
        }
		else
		{
			/*Do nothing*/
		}
	}
	else
	{
		//D3 variant : Nothing to do
	}

    return TTStateL;
}
//------------------------------------------------------------------------------
/// DESCRIPTION:         function to update hill assist descent control telltale
///
/// PARAMETERS:          \param TTidP: telltale id
///
/// RETURN VALUE:        \return TelltaleStatusType - On, Off, InAlarm
//------------------------------------------------------------------------------
TelltaleStatusType UpdateLaneDepartureWarning(const TelltaleIdType TTidP)
{
//ajoshi8: SysRS_14_Telltales-6200 is set as NA in 'BL_P2D3_C5.1.2.0' of SysRS14_Manage_Telltales
//    uint8 u8CANFeedback  = 0U;
//    uint8 u8CANState  = 0U;
      TelltaleStatusType TTStateL = TelltaleStatus_Off;

    //ajoshi8: SysRS_14_Telltales-6200 is set as NA in 'BL_P2D3_C5.1.2.0' of SysRS14_Manage_Telltales
//    (void)RI_Read_CDE_COMBINE_TEMOINS_DMD_ALLUM_AFIL(&u8CANFeedback , &u8CANState);
//
//    if(u8CANFeedback == eLaneDepartOn)
//    {
//        TTStateL = TelltaleStatus_On;
//    }
//    else if(u8CANFeedback == eLaneDepartSlowBlink)
//    {
//        TTStateL = TelltaleStatus_Fault;
//    }
//    else if(u8CANFeedback == eLaneDepartBlink)
//    {
//        TTStateL = TelltaleStatus_Warning;
//    }
//    else
//    {
//        //Nothing to do
//    }

    return TTStateL;
}
//------------------------------------------------------------------------------
/// DESCRIPTION:         function to update Low Tyre Pressure telltale
///
/// PARAMETERS:          \param TTidP: telltale id
///
/// RETURN VALUE:        \return TelltaleStatusType - On, Off, InAlarm
//------------------------------------------------------------------------------
TelltaleStatusType UpdateLowTyrePress(const TelltaleIdType TTidP)
{
    uint8                   u8SOUG_ALL  = 0U;
    uint8                   u8DSG_DEFL  = 0U;
    uint8                   CANETAT_PRINCIP_SEVL = 0U;
    //MDL_Telltale_SD_22: The corruption of this variable will not lead to violation of any safety goals.
    static uint8            CANETAT_PRINCIP_SEV_OldL = 0U;
    boolean                 bIsTimerStartedL  = 0U;
    boolean                 bIsTimerElapsedL  = 0U;

    //MDL_Telltale_SD_23: The corruption of this variable will not lead to violation of any safety goals.
    static boolean          bIsStateFilteringNotRunL = cFalse;
    ComExtSignalStateType   CANStateL;
    TelltaleStatusType      TTStateL = TelltaleStatus_Off;
    TelltaleStatusType TTOutStatusL[3] = {TelltaleStatus_On, TelltaleStatus_Warning, TelltaleStatus_Off};

    //Read the CAN signal responsible for turning ON of the telltale
    (void)RI_Read_CDE_COMBINE_TEMOINS_SOUG_AL(&u8SOUG_ALL, &CANStateL);

    //Check if frame loss is confirmed
    if(CANStateL == ComExtSignalState_ConfAbsent)
    {
        //SysRS_14_Telltales-5374: On confirmation of frame loss SOUG_AL = warning present
        //SysRS_14_Telltales-5382: On confirmation of frame loss value of DSG_DEF should be 0 after 10 seconds
        u8DSG_DEFL = eCligSignal_Continous;
        (void)MDL_Telltale_Read_uint8_SafetyCritical_Signal(&CANETAT_PRINCIP_SEVL, &CANStateL, VHL_ELEC_SYSTEM);

        if(bIsStateFilteringNotRunL == cFalse )
        {
            (void)OSIsTimerStarted(TyrePressTimer, &bIsTimerStartedL);
            if(bIsTimerStartedL == cTrue)
            {
                (void)OSIsTimerElapsed(TyrePressTimer, (TimerType)cTyrePressTimeout, &bIsTimerElapsedL);
                if(bIsTimerElapsedL == cTrue)
                {
                    //Timer VAR_CAN_TYRE_PRESSURE_FLT is expire (state = STATE_CAN_AFTER_FILTERING_AFTER_TEMPO)
                    bIsStateFilteringNotRunL = cTrue;
                    (void)OSTimerStop(&TyrePressTimer);
                    //u8DSG_DEFL is set to zero already, Low Tyre Pressure Telltale is continuous on
                }
                else
                {
                    //SysRS_14_Telltales-5382: On confirmation of frame loss value of DSG_DEF should be 1 for 10 seconds
                    u8DSG_DEFL = eCligSignal_Blinking; //(state = STATE_CAN_AFTER_FILTERING_TEMPO)
                }
            }
            else
            {
                //Start timer VAR_CAN_TYRE_PRESSURE_FLT (state = STATE_CAN_AFTER_FILTERING_TEMPO)
                (void)OSTimerStart(&TyrePressTimer);
                u8DSG_DEFL = eCligSignal_Blinking; //When timer start Low Tyre Pressure Telltale start blinking
            }
        }
        //Condition Ch(DAT_LI_CAN_STATE_VHL_ELEC_SYSTEM (state = STATE_CAN_AFTER_FILTERING)
        else if(CANETAT_PRINCIP_SEV_OldL != CANETAT_PRINCIP_SEVL)
        {
            bIsStateFilteringNotRunL = cFalse;
        }
        else
        {
            // nothing here
        }
        //Save old state of DAT_LI_CAN_STATE_VHL_ELEC_SYSTEM
        CANETAT_PRINCIP_SEV_OldL = CANETAT_PRINCIP_SEVL;
    }
    else
    {
        //Read the CAN signal responsible for blinking of the telltale
        (void)RI_Read_CDE_COMBINE_TEMOINS_DSG_DEF(&u8DSG_DEFL, &CANStateL);
        bIsStateFilteringNotRunL = cFalse;

        // ajoshi8 : 288994: D3_P2_IC_T&V:  Low tyre pressure telltale not work as expected
        u8DSG_DEFL = (u8SOUG_ALL == (uint8)0x01 ) ? u8DSG_DEFL : eCligSignal_OFF;

        //ajoshi8: Fix for Defect 236089 - In case if CAN frame 'ID_CDE_COMBINE_TEMOINS' is received, check if the timer 'TyrePressTimer 'was already started last time when the frame loss was confirmed.
		//If the timer was started, stop the timer.
		(void)OSTimerStop(&TyrePressTimer);
    }

	//Telltale is on, update it's output
	TTStateL = TTOutStatusL[u8DSG_DEFL];

    return TTStateL;
}


//------------------------------------------------------------------------------
/// DESCRIPTION:         function to update City park warning telltale
///
/// PARAMETERS:          \param TTidP: telltale id
///
/// RETURN VALUE:        \return TelltaleStatusType - On, Off, InAlarm
//------------------------------------------------------------------------------
TelltaleStatusType UpdateCityParkWarningTelltale(const TelltaleIdType TTidP)
{
	sNonCfgTTControl const *      pTTNonCfgInfo;
	TelltaleStatusType      TTStateL    = TelltaleStatus_Off;
	Std_ReturnType          bResultL = E_FAIL;
	uint8                   u8CANValueL = 0U;
	ComExtSignalStateType  CANStateUnusedL = 0U;

	    //Get info about interfaces to be called for updating the status of the telltale
	    pTTNonCfgInfo = GetTTDescriptionNonCfgTT(TTidP);

	    if (NULL != pTTNonCfgInfo->pReadTTControlTelltale)
	    {
	        bResultL = pTTNonCfgInfo->pReadTTControlTelltale(&u8CANValueL, &CANStateUnusedL);

	        if (S_OK == bResultL)
	        {
	            if (cInvalidTelltaleStatus > (TelltaleStatusType)u8CANValueL)
	            {
	            	if(u8CANValueL == 0)
	            	{
	            		TTStateL = (TelltaleStatusType) TelltaleStatus_Off;
	            	}
	            	else if(u8CANValueL == 1)
	            	{
	            		TTStateL = (TelltaleStatusType) TelltaleStatus_On;
	            	}
	            	else if(u8CANValueL == 2)
	            	{
	            		TTStateL = (TelltaleStatusType) TelltaleStatus_Warning;
	            	}
	            	else
	            	{
	            		//no action
	            	}
	            }
	            else
	            {
	              //Nothing to do
	            }
	        }
	        else
	        {
	          //Nothing to do
	        }
	    }
	    else
	    {
	      //Nothing to do
	    }

	    return TTStateL;
}



//------------------------------------------------------------------------------
/// DESCRIPTION:
///
/// PARAMETERS:          \param IsComponentActiveRef:
///
/// RETURN VALUE:        \return Std_ReturnType -
//------------------------------------------------------------------------------
Std_ReturnType Telltales_IsComponentActive(boolean *IsComponentActiveRef)
{
	uint8 Index;
	uint8 TelltaleIsON=cFalse;

	for(Index=(uint8)0x00u;Index<TelltaleId_Count;Index++)
	{
		if(TTStates[Index] != TelltaleStatus_Off)
		{
			TelltaleIsON=cTrue;
			(*IsComponentActiveRef)=cTrue;

			break;
		}
	}

	if(TelltaleIsON==cFalse)
	{
		(*IsComponentActiveRef)=cFalse;
	}

	return S_OK;
}

//#ifdef VIP_SAFETY_ENABLE
//------------------------------------------------------------------------------
/// DESCRIPTION:         Process a CAN safety signal
///
/// PARAMETERS:          \param TTidP: telltale id
///
/// RETURN VALUE:        \return TelltaleStatusType - On, Off, InAlarm
//------------------------------------------------------------------------------
static void GetSignalID(tTelltaleSafetySignalID * u8SignalID, TelltaleIdType TTidP)
{
	switch(TTidP)
	{
        case TelltaleId_TELLTALES_ELEC_PARK_BRAKE_SYST_FLT:
        	*u8SignalID = ELEC_PARK_BRAKE_SYST;
        	break;
        case TelltaleId_TELLTALES_PASS_SAF_FLT:
        	*u8SignalID = PASS_SAF_FLT;
        	break;
        case TelltaleId_TELLTALES_ABS_FLT:
        	*u8SignalID = ABS_FLT;
        	break;
        case TelltaleId_TELLTALES_TURN_SIGNAL_R:
        	*u8SignalID = TURN_SIGNAL_R;
        	break;
        case TelltaleId_TELLTALES_TURN_SIGNAL_L:
        	*u8SignalID = TURN_SIGNAL_L;
        	break;
        case TelltaleId_TELLTALES_ELEC_PARK_BRAKE_INHIB:
        	*u8SignalID = ELEC_PARK_BRAKE_INHIB;
        	break;
        case TelltaleId_TELLTALES_AIRBAG_INHIB:
        	*u8SignalID = AIRBAG_INHIB;
        	break;
        case TelltaleId_TELLTALES_ESP_OFF:
        	*u8SignalID = ESP_INHIB;
        	break;
        case TelltaleId_TELLTALES_PRESENCE_DOOR_OPEN:
        	*u8SignalID = DOOR_OPEN;
        	break;
        case TelltaleId_TELLTALES_ELEC_FAULTS_G4:
        	*u8SignalID = ELEC_FAULTS_G4;
        	break;
        case TelltaleId_TELLTALES_ELEC_PARK_BRAKE:
        	*u8SignalID = PARK_BRAKE;
        	break;
        default:
        	*u8SignalID = INVALID_TELLTALE_SIGNAL_ID;
	}
}


//------------------------------------------------------------------------------
/// DESCRIPTION:         Process a CAN signal
///
/// PARAMETERS:          \param TTidP: telltale id
///
/// RETURN VALUE:        \return TelltaleStatusType - On, Off, InAlarm
//------------------------------------------------------------------------------
TelltaleStatusType TTProcess1SafetyCanSignalCfg(const TelltaleIdType TTidP)
{

	TelltaleStatusType         TTStateL    = TelltaleStatus_Off;
	Std_ReturnType          bResultL = E_FAIL;
	boolean                  bCANValueL = 0U;
	ComExtSignalStateType  CANStateUnusedL = 0U;
	TelltaleStatusType     TTBrakeFailureStateL    = TelltaleStatus_Off;

	(void)Rte_Adapter_Read_GetTelltaleState(TelltaleId_TELLTALES_BRAKE_FAILURE,&TTBrakeFailureStateL);

	if(TelltaleStatus_Off == TTBrakeFailureStateL)
	{
		//specific for HBC warning light
		bResultL = MDL_Telltale_Read_Boolean_SafetyCritical_Signal(&bCANValueL, &CANStateUnusedL, HYDRAU_BOOST_COMPENS);

		if (S_OK == bResultL)
		{
			if (cInvalidTelltaleStatus > (TelltaleStatusType)bCANValueL)
			{
				if(bCANValueL == 0)
				{
					TTStateL = (TelltaleStatusType) TelltaleStatus_Off;
				}
				else if(bCANValueL == 1)
				{
					TTStateL = (TelltaleStatusType) TelltaleStatus_On;
				}
				else
				{
					//no action
				}
			}
			else
			{
				//Nothing to do
			}
		}
		else
		{
			//Nothing to do
		}
	}
	else
	{
		TTStateL = (TelltaleStatusType) TelltaleStatus_Off;
	}

	return TTStateL;
}

//------------------------------------------------------------------------------
/// DESCRIPTION:         Process a CAN signal and def code
///
/// PARAMETERS:          \param TTidP: telltale id
///
/// RETURN VALUE:        \return TelltaleStatusType - On, Off, InAlarm
//------------------------------------------------------------------------------
TelltaleStatusType TTProcessSafetyCanSigAndCodeFault(const TelltaleIdType TTidP)
{
	TelltaleStatusType      TTStateL    = TelltaleStatus_Off;
	Std_ReturnType          bResultL = E_FAIL;
	ComExtSignalStateType   CANStateUnusedL = 0U;
	boolean                 bCANDefCodeL = 0U;
	boolean                 bTTControlCANL = 0U;
	tTelltaleSafetySignalID	u8SignalID;
	uint8                   u8Presence1L = 0U;
	uint8                   u8Presence2L = 0U;

	//ajoshi8 : Check value of presence for TELLTALES_BRAKE_FAILURE & TELLTALES_ELEC_PARK_BRAKE both , as per 18T2 specification
	(void)RI_ReadHwId(TTidP, &u8Presence1L);
	//ajoshi8: Task 478509 : Read explicitly the presence value of TELLTALES_BRAKE_FAILURE from NvM
	(void)RI_ReadHwId(TelltaleId_TELLTALES_BRAKE_FAILURE, &u8Presence2L);

	//ajoshi8: Task 478509 : Check explicitly the presence value of both parameters from NvM
	if( ((uint8)1U == u8Presence1L) && ((uint8)1U == u8Presence2L)  )   // 18T2: Ensure that both - TELLTALES_BRAKE_FAILURE & TELLTALES_ELEC_PARK_BRAKE have value = 1 [Present] in NVM
	{
		GetSignalID(&u8SignalID, TTidP);
		bResultL  = MDL_Telltale_Read_Boolean_SafetyCritical_Signal(&bCANDefCodeL, &CANStateUnusedL, ELEC_PARK_BRAKE_FLT);
		bResultL |= MDL_Telltale_Read_Boolean_SafetyCritical_Signal(&bTTControlCANL, &CANStateUnusedL, u8SignalID);

		if (S_OK == bResultL)
		{
			//Check if defect is not present
			if (cTTFaultAbsent == bCANDefCodeL)
			{
				if (cInvalidTelltaleStatus > (TelltaleStatusType)bTTControlCANL)
				{
					if(bTTControlCANL == 0)
					{
						TTStateL = (TelltaleStatusType) TelltaleStatus_Off;
					}
					else if(bTTControlCANL == 1)
					{
						TTStateL = (TelltaleStatusType) TelltaleStatus_On;
					}
					else
					{
						TTStateL = (TelltaleStatusType) TelltaleStatus_Warning;
					}
				}
				else
				{
					//Nothing to do
				}
			}
			else
			{
				TTStateL = TelltaleStatus_Warning;
			}
		}
	}

	return TTStateL;
}
//------------------------------------------------------------------------------
/// DESCRIPTION:         Process a CAN signal
///
/// PARAMETERS:          \param TTidP: telltale id
///
/// RETURN VALUE:        \return TelltaleStatusType - On, Off, InAlarm
//------------------------------------------------------------------------------
TelltaleStatusType TTProcess1SafetyCanSignal_Indicators(const TelltaleIdType TTidP)
{
    TelltaleStatusType         TTStateL    = TelltaleStatus_Off;
    Std_ReturnType          bResultL = E_FAIL;
    Boolean                 bNVMValueL = cFalse;
    Boolean                 bNVMValueSafetyL = cFalse;
    boolean                  bCANValueL = 0U;
    ComExtSignalStateType  CANStateUnusedL = 0U;
    tTelltaleSafetySignalID	u8SignalID;

    (void)RI_Read_TELLTALES_TURN_SIGNALS_LED(&bNVMValueL);
    (void)RI_Read_TELLTALES_TURN_SIGNALS_LED_Safety(&bNVMValueSafetyL);


    //ajoshi8 : Defect 382555 -> TELLTALES_TURN_SIGNALS_LED & TELLTALES_TURN_SIGNALS_LED_SAFETY need to be in sync as defined in NvM cfg. matrix
    if(((cFalse == bNVMValueL) && (bNVMValueSafetyL == 0x5)) ||
    	    				((cTrue == bNVMValueL) && (bNVMValueSafetyL == 0xA)))
    {
    	//ajoshi8 : Defect 382555 -> additionally check value of NvM parameter & its associated SAFETY mirror value before processing CAN input
    	if((cTrue == bNVMValueL) && (bNVMValueSafetyL == 0xA))
    	{
    		GetSignalID(&u8SignalID, TTidP);
    		bResultL = MDL_Telltale_Read_Boolean_SafetyCritical_Signal(&bCANValueL, &CANStateUnusedL, u8SignalID);

    		if (S_OK == bResultL)
    		{
    			if (cInvalidTelltaleStatus > (TelltaleStatusType)bCANValueL)
    			{
    				if(bCANValueL == 0)
    				{
    					TTStateL = (TelltaleStatusType) TelltaleStatus_Off;
    				}
    				else if(bCANValueL == 1)
    				{
    					TTStateL = (TelltaleStatusType) TelltaleStatus_On;
    				}
    				else
    				{
    					//no action
    				}
    			}
    			else
    			{
    				//Nothing to do
    			}
    		}
    		else
    		{
    			//Nothing to do
    		}
    	}
    }
    else
    {
    	//Notify RAM corruption
    	RI_SRV_Safety_ManageSafetyCountRL3A(eRL3A_NVM);

    }
    return TTStateL;
}



//------------------------------------------------------------------------------
/// DESCRIPTION:
///
/// PARAMETERS:          \param IsComponentActiveRef:
///
/// RETURN VALUE:        \return Std_ReturnType -
//------------------------------------------------------------------------------
TelltaleStatusType TTProcess1SafetyCanSignal(const TelltaleIdType TTidP)
{
	TelltaleStatusType         TTStateL    = TelltaleStatus_Off;
	Std_ReturnType          bResultL = E_FAIL;
	boolean                  bCANValueL = 0U;
	ComExtSignalStateType  CANStateUnusedL = 0U;
	tTelltaleSafetySignalID 	u8SignalID;

	GetSignalID(&u8SignalID, TTidP);
	bResultL = MDL_Telltale_Read_Boolean_SafetyCritical_Signal(&bCANValueL, &CANStateUnusedL, u8SignalID);

	if (S_OK == bResultL)
	{
		if (cInvalidTelltaleStatus > (TelltaleStatusType)bCANValueL)
		{
			if(bCANValueL == 0)
			{
				TTStateL = (TelltaleStatusType) TelltaleStatus_Off;
			}
			else if(bCANValueL == 1)
			{
				TTStateL = (TelltaleStatusType) TelltaleStatus_On;
			}
			else
			{
				TTStateL = (TelltaleStatusType) TelltaleStatus_Warning;
			}
		}
		else
		{
			//Nothing to do
		}
	}
	else
	{
		//Nothing to do
	}

	return TTStateL;
}



//------------------------------------------------------------------------------
/// DESCRIPTION:
///
/// PARAMETERS:          \param IsComponentActiveRef:
///
/// RETURN VALUE:        \return Std_ReturnType -
//------------------------------------------------------------------------------
static boolean CheckIfSafetyTelltale(uint8 TelltaleId)
{
	boolean bRetL = cFalse;

	switch (TelltaleId)
	{
		case TelltaleId_TELLTALES_STOP:
		case TelltaleId_TELLTALES_SERVICE:
		case TelltaleId_TELLTALES_AIRBAG_INHIB:
		case TelltaleId_TELLTALES_TURN_SIGNAL_R:
		case TelltaleId_TELLTALES_TURN_SIGNAL_L:
		case TelltaleId_TELLTALES_POWER_STEER_FLT:
    	case TelltaleId_TELLTALES_ELEC_PARK_BRAKE_INHIB:
    	case TelltaleId_TELLTALES_ESP:
    	case TelltaleId_TELLTALES_PASS_SAF_FLT:
    	case TelltaleId_TELLTALES_ABS_FLT:
    	case TelltaleId_TELLTALES_BATTERY_FLT:
    	case TelltaleId_TELLTALES_ESP_OFF:
    	case TelltaleId_TELLTALES_ELEC_FAULTS_G4:
    	case TelltaleId_TELLTALES_ELEC_PARK_BRAKE_SYST_FLT:
    	case TelltaleId_TELLTALES_ELEC_PARK_BRAKE:
    	case TelltaleId_TELLTALES_PRESENCE_DOOR_OPEN :
			bRetL = cTrue;
			break;
		default:
			bRetL = cFalse;
	}

	return bRetL;

}
//#endif //VIP_SAFETY_ENABLE
