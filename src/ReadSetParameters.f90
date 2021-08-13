! Copyright 2019 NREL

! Licensed under the Apache License, Version 2.0 (the "License"); you may not use
! this file except in compliance with the License. You may obtain a copy of the
! License at http://www.apache.org/licenses/LICENSE-2.0

! Unless required by applicable law or agreed to in writing, software distributed
! under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
! CONDITIONS OF ANY KIND, either express or implied. See the License for the
! specific language governing permissions and limitations under the License.
! -------------------------------------------------------------------------------------------
! Read and set the parameters used by the controller

! Submodules:
!           Assert: Initial condition and input check
!           ComputeVariablesSetpoints: Compute setpoints used by controllers
!           ReadAvrSWAP: Read AvrSWAP array
!           ReadControlParameterFileSub: Read DISCON.IN input file
!           ReadCPFile: Read text file containing Cp Surface
!           SetParameters: Define initial conditions 

MODULE ReadSetParameters

    USE, INTRINSIC :: ISO_C_Binding

USE Constants
USE Functions

    IMPLICIT NONE

CONTAINS
    ! -----------------------------------------------------------------------------------
    ! Read all constant control parameters from DISCON.IN parameter file
    SUBROUTINE ReadControlParameterFileSub(CntrPar, accINFILE, accINFILE_size)!, accINFILE_size)
        USE, INTRINSIC :: ISO_C_Binding
        USE ROSCO_Types, ONLY : ControlParameters

        INTEGER(4)                              :: accINFILE_size               ! size of DISCON input filename
        CHARACTER(accINFILE_size), INTENT(IN)   :: accINFILE(accINFILE_size)    ! DISCON input filename
        INTEGER(4), PARAMETER                   :: UnControllerParameters = 89  ! Unit number to open file
        TYPE(ControlParameters), INTENT(INOUT)  :: CntrPar                      ! Control parameter type
       

        OPEN(unit=UnControllerParameters, file=accINFILE(1), status='old', action='read')
        
        !----------------------- HEADER ------------------------
        READ(UnControllerParameters, *)
        READ(UnControllerParameters, *)
        READ(UnControllerParameters, *)

        !----------------------- DEBUG --------------------------
        READ(UnControllerParameters, *) 
        READ(UnControllerParameters, *) CntrPar%LoggingLevel
        READ(UnControllerParameters, *) 

        !----------------- CONTROLLER FLAGS ---------------------
        READ(UnControllerParameters, *)
        READ(UnControllerParameters, *) CntrPar%F_LPFType
        READ(UnControllerParameters, *) CntrPar%F_NotchType
        READ(UnControllerParameters, *) CntrPar%IPC_ControlMode
        READ(UnControllerParameters, *) CntrPar%VS_ControlMode
        READ(UnControllerParameters, *) CntrPar%PC_ControlMode
        READ(UnControllerParameters, *) CntrPar%Y_ControlMode        
        READ(UnControllerParameters, *) CntrPar%SS_Mode        
        READ(UnControllerParameters, *) CntrPar%WE_Mode        
        READ(UnControllerParameters, *) CntrPar%PS_Mode        
        READ(UnControllerParameters, *) CntrPar%SD_Mode        
        READ(UnControllerParameters, *) CntrPar%FL_Mode        
        READ(UnControllerParameters, *) CntrPar%Flp_Mode        
        READ(UnControllerParameters, *)

        !----------------- FILTER CONSTANTS ---------------------
        READ(UnControllerParameters, *)        
        READ(UnControllerParameters, *) CntrPar%F_LPFCornerFreq
        READ(UnControllerParameters, *) CntrPar%F_LPFDamping
        READ(UnControllerParameters, *) CntrPar%F_NotchCornerFreq
        ALLOCATE(CntrPar%F_NotchBetaNumDen(2))
        READ(UnControllerParameters,*) CntrPar%F_NotchBetaNumDen
        READ(UnControllerParameters,*) CntrPar%F_SSCornerFreq
        READ(UnControllerParameters,*) CntrPar%F_FlCornerFreq, CntrPar%F_FlDamping
        READ(UnControllerParameters,*) CntrPar%F_FlpCornerFreq, CntrPar%F_FlpDamping
        READ(UnControllerParameters, *)

        !----------- BLADE PITCH CONTROLLER CONSTANTS -----------
        READ(UnControllerParameters, *)
        READ(UnControllerParameters, *) CntrPar%PC_GS_n
        ALLOCATE(CntrPar%PC_GS_angles(CntrPar%PC_GS_n))
        READ(UnControllerParameters,*) CntrPar%PC_GS_angles
        ALLOCATE(CntrPar%PC_GS_KP(CntrPar%PC_GS_n))
        READ(UnControllerParameters,*) CntrPar%PC_GS_KP
        ALLOCATE(CntrPar%PC_GS_KI(CntrPar%PC_GS_n))
        READ(UnControllerParameters,*) CntrPar%PC_GS_KI
        ALLOCATE(CntrPar%PC_GS_KD(CntrPar%PC_GS_n))
        READ(UnControllerParameters,*) CntrPar%PC_GS_KD
        ALLOCATE(CntrPar%PC_GS_TF(CntrPar%PC_GS_n))
        READ(UnControllerParameters,*) CntrPar%PC_GS_TF
        READ(UnControllerParameters, *) CntrPar%PC_MaxPit
        READ(UnControllerParameters, *) CntrPar%PC_MinPit
        READ(UnControllerParameters, *) CntrPar%PC_MaxRat
        READ(UnControllerParameters, *) CntrPar%PC_MinRat
        READ(UnControllerParameters, *) CntrPar%PC_RefSpd
        READ(UnControllerParameters, *) CntrPar%PC_FinePit
        READ(UnControllerParameters, *) CntrPar%PC_Switch
        READ(UnControllerParameters, *)

        !------------------- IPC CONSTANTS -----------------------
        READ(UnControllerParameters, *) 
        READ(UnControllerParameters, *) CntrPar%IPC_IntSat
        ALLOCATE(CntrPar%IPC_KI(2))
        READ(UnControllerParameters,*) CntrPar%IPC_KI
        ALLOCATE(CntrPar%IPC_aziOffset(2))
        READ(UnControllerParameters,*) CntrPar%IPC_aziOffset
        READ(UnControllerParameters, *) CntrPar%IPC_CornerFreqAct
        READ(UnControllerParameters, *)

        !------------ VS TORQUE CONTROL CONSTANTS ----------------
        READ(UnControllerParameters, *)
        READ(UnControllerParameters, *) CntrPar%VS_GenEff
        READ(UnControllerParameters, *) CntrPar%VS_ArSatTq
        READ(UnControllerParameters, *) CntrPar%VS_MaxRat
        READ(UnControllerParameters, *) CntrPar%VS_MaxTq
        READ(UnControllerParameters, *) CntrPar%VS_MinTq
        READ(UnControllerParameters, *) CntrPar%VS_MinOMSpd
        READ(UnControllerParameters, *) CntrPar%VS_Rgn2K
        READ(UnControllerParameters, *) CntrPar%VS_RtPwr
        READ(UnControllerParameters, *) CntrPar%VS_RtTq
        READ(UnControllerParameters, *) CntrPar%VS_RefSpd
        READ(UnControllerParameters, *) CntrPar%VS_n
        ALLOCATE(CntrPar%VS_KP(CntrPar%VS_n))
        READ(UnControllerParameters,*) CntrPar%VS_KP
        ALLOCATE(CntrPar%VS_KI(CntrPar%VS_n))
        READ(UnControllerParameters,*) CntrPar%VS_KI
        READ(UnControllerParameters,*) CntrPar%VS_TSRopt
        READ(UnControllerParameters, *)

        !------- Setpoint Smoother --------------------------------
        READ(UnControllerParameters, *)
        READ(UnControllerParameters, *) CntrPar%SS_VSGain
        READ(UnControllerParameters, *) CntrPar%SS_PCGain
        READ(UnControllerParameters, *) 

        !------------ WIND SPEED ESTIMATOR CONTANTS --------------
        READ(UnControllerParameters, *)
        READ(UnControllerParameters, *) CntrPar%WE_BladeRadius
        READ(UnControllerParameters, *) CntrPar%WE_CP_n
        ALLOCATE(CntrPar%WE_CP(CntrPar%WE_CP_n))
        READ(UnControllerParameters, *) CntrPar%WE_CP
        READ(UnControllerParameters, *) CntrPar%WE_Gamma
        READ(UnControllerParameters, *) CntrPar%WE_GearboxRatio
        READ(UnControllerParameters, *) CntrPar%WE_Jtot
        READ(UnControllerParameters, *) CntrPar%WE_RhoAir
        READ(UnControllerParameters, *) CntrPar%PerfFileName
        ALLOCATE(CntrPar%PerfTableSize(2))
        READ(UnControllerParameters, *) CntrPar%PerfTableSize
        READ(UnControllerParameters, *) CntrPar%WE_FOPoles_N
        ALLOCATE(CntrPar%WE_FOPoles_v(CntrPar%WE_FOPoles_n))
        READ(UnControllerParameters, *) CntrPar%WE_FOPoles_v
        ALLOCATE(CntrPar%WE_FOPoles(CntrPar%WE_FOPoles_n))
        READ(UnControllerParameters, *) CntrPar%WE_FOPoles
        READ(UnControllerParameters, *)

        !-------------- YAW CONTROLLER CONSTANTS -----------------
        READ(UnControllerParameters, *)
        READ(UnControllerParameters, *) CntrPar%Y_ErrThresh
        READ(UnControllerParameters, *) CntrPar%Y_IPC_IntSat
        READ(UnControllerParameters, *) CntrPar%Y_IPC_n
        ALLOCATE(CntrPar%Y_IPC_KP(CntrPar%Y_IPC_n))
        READ(UnControllerParameters,*) CntrPar%Y_IPC_KP
        ALLOCATE(CntrPar%Y_IPC_KI(CntrPar%Y_IPC_n))
        READ(UnControllerParameters,*) CntrPar%Y_IPC_KI
        READ(UnControllerParameters, *) CntrPar%Y_IPC_omegaLP
        READ(UnControllerParameters, *) CntrPar%Y_IPC_zetaLP
        READ(UnControllerParameters, *) CntrPar%Y_MErrSet
        READ(UnControllerParameters, *) CntrPar%Y_omegaLPFast
        READ(UnControllerParameters, *) CntrPar%Y_omegaLPSlow
        READ(UnControllerParameters, *) CntrPar%Y_Rate
        READ(UnControllerParameters, *)

        !------------ FORE-AFT TOWER DAMPER CONSTANTS ------------
        READ(UnControllerParameters, *)      
        READ(UnControllerParameters, *) CntrPar%FA_KI  
        READ(UnControllerParameters, *) CntrPar%FA_HPFCornerFreq
        READ(UnControllerParameters, *) CntrPar%FA_IntSat
        READ(UnControllerParameters, *)      

        !------------ PEAK SHAVING ------------
        READ(UnControllerParameters, *)      
        READ(UnControllerParameters, *) CntrPar%PS_BldPitchMin_N  
        ALLOCATE(CntrPar%PS_WindSpeeds(CntrPar%PS_BldPitchMin_N))
        READ(UnControllerParameters, *) CntrPar%PS_WindSpeeds
        ALLOCATE(CntrPar%PS_BldPitchMin(CntrPar%PS_BldPitchMin_N))
        READ(UnControllerParameters, *) CntrPar%PS_BldPitchMin
        READ(UnControllerParameters, *) 

        !------------ SHUTDOWN ------------
        READ(UnControllerParameters, *)      
        READ(UnControllerParameters, *) CntrPar%SD_MaxPit  
        READ(UnControllerParameters, *) CntrPar%SD_CornerFreq
        READ(UnControllerParameters, *)      

        !------------ FLOATING ------------
        READ(UnControllerParameters, *)
        READ(UnControllerParameters, *) CntrPar%Fl_Kp  
        READ(UnControllerParameters, *) 

        !------------ Flaps ------------
        READ(UnControllerParameters, *)      
        READ(UnControllerParameters, *) CntrPar%Flp_Angle  
        READ(UnControllerParameters, *) CntrPar%Flp_Kp  
        READ(UnControllerParameters, *) CntrPar%Flp_Ki  
        READ(UnControllerParameters, *) CntrPar%Flp_MaxPit  
        ! END OF INPUT FILE    
        
        !------------------- CALCULATED CONSTANTS -----------------------
        CntrPar%PC_RtTq99 = CntrPar%VS_RtTq*0.99
        CntrPar%VS_MinOMTq = CntrPar%VS_Rgn2K*CntrPar%VS_MinOMSpd**2
        CntrPar%VS_MaxOMTq = CntrPar%VS_Rgn2K*CntrPar%VS_RefSpd**2
        CntrPar%VS_Rgn3Pitch = CntrPar%PC_FinePit + CntrPar%PC_Switch
        
        CLOSE(UnControllerParameters)
        
        !------------------- HOUSEKEEPING -----------------------
        CntrPar%PerfFileName = TRIM(CntrPar%PerfFileName)

    END SUBROUTINE ReadControlParameterFileSub
    ! -----------------------------------------------------------------------------------
    ! Calculate setpoints for primary control actions    
    SUBROUTINE ComputeVariablesSetpoints(CntrPar, LocalVar, objInst)
        USE ROSCO_Types, ONLY : ControlParameters, LocalVariables, ObjectInstances
        
        ! Allocate variables
        TYPE(ControlParameters), INTENT(INOUT)  :: CntrPar
        TYPE(LocalVariables), INTENT(INOUT)     :: LocalVar
        TYPE(ObjectInstances), INTENT(INOUT)    :: objInst

        REAL(4)                                 :: VS_RefSpd        ! Referece speed for variable speed torque controller, [rad/s] 
        REAL(4)                                 :: PC_RefSpd        ! Referece speed for pitch controller, [rad/s] 
        REAL(4)                                 :: Omega_op         ! Optimal TSR-tracking generator speed, [rad/s]
        REAL(4)                                 :: WE_Vw_f          ! Filtered Wind Speed Estimate
        ! temp
        ! REAL(4)                                 :: VS_TSRop = 7.5

        ! ----- Calculate yaw misalignment error -----
        LocalVar%Y_MErr = LocalVar%Y_M + CntrPar%Y_MErrSet ! Yaw-alignment error
        
        ! ----- Pitch controller speed and power error -----
        ! Implement setpoint smoothing
        IF (LocalVar%SS_DelOmegaF < 0) THEN
            PC_RefSpd = CntrPar%PC_RefSpd - LocalVar%SS_DelOmegaF
        ELSE
            PC_RefSpd = CntrPar%PC_RefSpd
        ENDIF

        LocalVar%PC_SpdErr = PC_RefSpd - LocalVar%GenSpeedF            ! Speed error
        LocalVar%PC_PwrErr = CntrPar%VS_RtPwr - LocalVar%VS_GenPwr             ! Power error
        
        ! ----- Torque controller reference errors -----
        ! Define VS reference generator speed [rad/s]
        IF (CntrPar%VS_ControlMode == 2) THEN
            ! WE_Vw_f = LPFilter(LocalVar%We_Vw, LocalVar%DT, 0.625, LocalVar%iStatus, .FALSE., objInst%instLPF)
            WE_Vw_f = LocalVar%We_Vw
            VS_RefSpd = (CntrPar%VS_TSRopt * WE_Vw_f / CntrPar%WE_BladeRadius) * CntrPar%WE_GearboxRatio
            VS_RefSpd = saturate(VS_RefSpd,CntrPar%VS_MinOMSpd, CntrPar%VS_RefSpd)
        ELSE
            VS_RefSpd = CntrPar%VS_RefSpd
        ENDIF 
        
        ! Implement setpoint smoothing
        IF (LocalVar%SS_DelOmegaF > 0) THEN
            VS_RefSpd = VS_RefSpd - LocalVar%SS_DelOmegaF
        ENDIF

        ! Force zero torque in shutdown mode
        IF (LocalVar%SD) THEN
            VS_RefSpd = CntrPar%VS_MinOMSpd
        ENDIF

        ! Force minimum rotor speed
        VS_RefSpd = max(VS_RefSpd, CntrPar%VS_MinOmSpd)

        ! TSR-tracking reference error
        IF (CntrPar%VS_ControlMode == 2) THEN
            LocalVar%VS_SpdErr = VS_RefSpd - LocalVar%GenSpeedF
        ENDIF

        ! Define transition region setpoint errors
        LocalVar%VS_SpdErrAr = VS_RefSpd - LocalVar%GenSpeedF               ! Current speed error - Region 2.5 PI-control (Above Rated)
        LocalVar%VS_SpdErrBr = CntrPar%VS_MinOMSpd - LocalVar%GenSpeedF     ! Current speed error - Region 1.5 PI-control (Below Rated)
    
    
    END SUBROUTINE ComputeVariablesSetpoints
    ! -----------------------------------------------------------------------------------
    ! Read avrSWAP array passed from ServoDyn    
    SUBROUTINE ReadAvrSWAP(avrSWAP, LocalVar)
        USE ROSCO_Types, ONLY : LocalVariables
    
        REAL(C_FLOAT), INTENT(INOUT) :: avrSWAP(*)   ! The swap array, used to pass data to, and receive data from, the DLL controller.
        TYPE(LocalVariables), INTENT(INOUT) :: LocalVar
        
        ! Load variables from calling program (See Appendix A of Bladed User's Guide):
        LocalVar%iStatus = NINT(avrSWAP(1))
        LocalVar%Time = avrSWAP(2)
        LocalVar%DT = avrSWAP(3)
        LocalVar%VS_MechGenPwr = avrSWAP(14)
        LocalVar%VS_GenPwr = avrSWAP(15)
        LocalVar%GenSpeed = avrSWAP(20)
        LocalVar%RotSpeed = avrSWAP(21)
        LocalVar%GenTqMeas = avrSWAP(23)
        LocalVar%Y_M = avrSWAP(24)
        LocalVar%HorWindV = avrSWAP(27)
        LocalVar%rootMOOP(1) = avrSWAP(30)
        LocalVar%rootMOOP(2) = avrSWAP(31)
        LocalVar%rootMOOP(3) = avrSWAP(32)
        LocalVar%FA_Acc = avrSWAP(53)
        LocalVar%NacIMU_FA_Acc = avrSWAP(83)
        LocalVar%Azimuth = avrSWAP(60)
        LocalVar%NumBl = NINT(avrSWAP(61))

          ! --- NJA: usually feedback back the previous pitch command helps for numerical stability, sometimes it does not...
        IF (LocalVar%iStatus == 0) THEN
            LocalVar%BlPitch(1) = avrSWAP(4)
            LocalVar%BlPitch(2) = avrSWAP(33)
            LocalVar%BlPitch(3) = avrSWAP(34)
        ELSE
            LocalVar%BlPitch(1) = LocalVar%PitCom(1)
            LocalVar%BlPitch(2) = LocalVar%PitCom(2)
            LocalVar%BlPitch(3) = LocalVar%PitCom(3)      
        ENDIF

    END SUBROUTINE ReadAvrSWAP
    ! -----------------------------------------------------------------------------------
    ! Check for errors before any execution
    SUBROUTINE Assert(LocalVar, CntrPar, avrSWAP, aviFAIL, ErrMsg, size_avcMSG)
        USE, INTRINSIC :: ISO_C_Binding
        USE ROSCO_Types, ONLY : LocalVariables, ControlParameters
    
        IMPLICIT NONE
    
        ! Inputs
        TYPE(ControlParameters), INTENT(IN)     :: CntrPar
        TYPE(LocalVariables), INTENT(IN)        :: LocalVar
        INTEGER(4), INTENT(IN)                  :: size_avcMSG
        REAL(C_FLOAT), INTENT(IN)               :: avrSWAP(*)          ! The swap array, used to pass data to, and receive data from, the DLL controller.
        
        ! Outputs
        INTEGER(C_INT), INTENT(OUT)             :: aviFAIL             ! A flag used to indicate the success of this DLL call set as follows: 0 if the DLL call was successful, >0 if the DLL call was successful but cMessage should be issued as a warning messsage, <0 if the DLL call was unsuccessful or for any other reason the simulation is to be stopped at this point with cMessage as the error message.
        CHARACTER(size_avcMSG-1), INTENT(OUT)   :: ErrMsg              ! a Fortran version of the C string argument (not considered an array here) [subtract 1 for the C null-character]
        
        ! Local
        
        !..............................................................................................................................
        ! Check validity of input parameters:
        !..............................................................................................................................
        
        IF ((CntrPar%F_LPFType > 2.0) .OR. (CntrPar%F_LPFType < 1.0)) THEN
            aviFAIL = -1
            ErrMsg  = 'F_LPFType must be 1 or 2.'
        ENDIF
        
        IF ((CntrPar%F_LPFDamping > 1.0) .OR. (CntrPar%F_LPFDamping < 0.0)) THEN
            aviFAIL = -1
            ErrMsg  = 'Filter damping coefficient must be between [0, 1]'
        ENDIF
        
        IF (CntrPar%IPC_CornerFreqAct < 0.0) THEN
            aviFAIL = -1
            ErrMsg  = 'Corner frequency of IPC actuator model must be positive, or set to 0 to disable.'
        ENDIF
        
        IF (CntrPar%F_LPFCornerFreq <= 0.0) THEN
            aviFAIL = -1
            ErrMsg  = 'CornerFreq must be greater than zero.'
        ENDIF
        
        IF ((CntrPar%IPC_ControlMode > 0) .AND. (CntrPar%Y_ControlMode > 1)) THEN
            aviFAIL = -1
            ErrMsg  = 'IPC control for load reductions and yaw-by-IPC cannot be activated simultaneously'
        ENDIF
        
        IF (LocalVar%DT <= 0.0) THEN
            aviFAIL = -1
            ErrMsg  = 'DT must be greater than zero.'
        ENDIF
        
        IF (CntrPar%VS_MaxRat <= 0.0) THEN
            aviFAIL =  -1
            ErrMsg  = 'VS_MaxRat must be greater than zero.'
        ENDIF
        
        IF (CntrPar%VS_RtTq < 0.0) THEN
            aviFAIL = -1
            ErrMsg  = 'VS_RtTq must not be negative.'
        ENDIF
        
        IF (CntrPar%VS_Rgn2K < 0.0) THEN
            aviFAIL = -1
            ErrMsg  = 'VS_Rgn2K must not be negative.'
        ENDIF
        
        IF (CntrPar%VS_MaxTq < CntrPar%VS_RtTq) THEN
            aviFAIL = -1
            ErrMsg  = 'VS_RtTq must not be greater than VS_MaxTq.'
        ENDIF
        
        IF (CntrPar%VS_KP(1) > 0.0) THEN
            aviFAIL = -1
            ErrMsg  = 'VS_KP must be greater than zero.'
        ENDIF
        
        IF (CntrPar%VS_KI(1) > 0.0) THEN
            aviFAIL = -1
            ErrMsg  = 'VS_KI must be greater than zero.'
        ENDIF
        
        IF (CntrPar%PC_RefSpd <= 0.0) THEN
            aviFAIL = -1
            ErrMsg  = 'PC_RefSpd must be greater than zero.'
        ENDIF
        
        IF (CntrPar%PC_MaxRat <= 0.0) THEN
            aviFAIL = -1
            ErrMsg  = 'PC_MaxRat must be greater than zero.'
        ENDIF
        
        IF (CntrPar%PC_MinPit >= CntrPar%PC_MaxPit)  THEN
            aviFAIL = -1
            ErrMsg  = 'PC_MinPit must be less than PC_MaxPit.'
        ENDIF
        
        IF (CntrPar%IPC_KI(1) < 0.0)  THEN
            aviFAIL = -1
            ErrMsg  = 'IPC_KI(1) must be zero or greater than zero.'
        ENDIF
        
        IF (CntrPar%IPC_KI(2) < 0.0)  THEN
            aviFAIL = -1
            ErrMsg  = 'IPC_KI(2) must be zero or greater than zero.'
        ENDIF
        
        ! ---- Yaw Control ----
        IF (CntrPar%Y_ControlMode > 0) THEN
            IF (CntrPar%Y_IPC_omegaLP <= 0.0)  THEN
                aviFAIL = -1
                ErrMsg  = 'Y_IPC_omegaLP must be greater than zero.'
            ENDIF
            
            IF (CntrPar%Y_IPC_zetaLP <= 0.0)  THEN
                aviFAIL = -1
                ErrMsg  = 'Y_IPC_zetaLP must be greater than zero.'
            ENDIF
            
            IF (CntrPar%Y_ErrThresh <= 0.0)  THEN
                aviFAIL = -1
                ErrMsg  = 'Y_ErrThresh must be greater than zero.'
            ENDIF
            
            IF (CntrPar%Y_Rate <= 0.0)  THEN
                aviFAIL = -1
                ErrMsg  = 'CntrPar%Y_Rate must be greater than zero.'
            ENDIF
            
            IF (CntrPar%Y_omegaLPFast <= 0.0)  THEN
                aviFAIL = -1
                ErrMsg  = 'Y_omegaLPFast must be greater than zero.'
            ENDIF
            
            IF (CntrPar%Y_omegaLPSlow <= 0.0)  THEN
                aviFAIL = -1
                ErrMsg  = 'Y_omegaLPSlow must be greater than zero.'
            ENDIF
        ENDIF

        ! --- Floating Control ---
        IF (CntrPar%Fl_Mode > 0) THEN
            IF (CntrPar%F_NotchType <= 1 .OR. CntrPar%F_NotchCornerFreq == 0.0) THEN
                aviFAIL = -1
                ErrMsg = 'F_NotchType and F_NotchCornerFreq must be specified for Fl_Mode greater than zero.'
            ENDIF
        ENDIF
        
        ! Abort if the user has not requested a pitch angle actuator (See Appendix A
        ! of Bladed User's Guide):
        IF (NINT(avrSWAP(10)) /= 0)  THEN ! .TRUE. if a pitch angle actuator hasn't been requested
            aviFAIL = -1
            ErrMsg  = 'Pitch angle actuator not requested.'
        ENDIF
        
        IF (NINT(avrSWAP(28)) == 0 .AND. ((CntrPar%IPC_ControlMode > 0) .OR. (CntrPar%Y_ControlMode > 1))) THEN
            aviFAIL = -1
            ErrMsg  = 'IPC enabled, but Ptch_Cntrl in ServoDyn has a value of 0. Set it to 1.'
        ENDIF

    END SUBROUTINE Assert
    ! -----------------------------------------------------------------------------------
    ! Define parameters for control actions
    SUBROUTINE SetParameters(avrSWAP, aviFAIL, accINFILE, ErrMsg, size_avcMSG, CntrPar, LocalVar, objInst, PerfData)
        USE ROSCO_Types, ONLY : ControlParameters, LocalVariables, ObjectInstances, PerformanceData
        
        INTEGER(4), INTENT(IN) :: size_avcMSG
        TYPE(ControlParameters), INTENT(INOUT) :: CntrPar
        TYPE(LocalVariables), INTENT(INOUT) :: LocalVar
        TYPE(ObjectInstances), INTENT(INOUT) :: objInst
        TYPE(PerformanceData), INTENT(INOUT) :: PerfData

        REAL(C_FLOAT), INTENT(INOUT) :: avrSWAP(*)          ! The swap array, used to pass data to, and receive data from, the DLL controller.
        INTEGER(C_INT), INTENT(OUT) :: aviFAIL              ! A flag used to indicate the success of this DLL call set as follows: 0 if the DLL call was successful, >0 if the DLL call was successful but cMessage should be issued as a warning messsage, <0 if the DLL call was unsuccessful or for any other reason the simulation is to be stopped at this point with cMessage as the error message.
        CHARACTER(KIND=C_CHAR), INTENT(IN)      :: accINFILE(NINT(avrSWAP(50)))     ! The name of the parameter input file
        CHARACTER(size_avcMSG-1), INTENT(OUT) :: ErrMsg     ! a Fortran version of the C string argument (not considered an array here) [subtract 1 for the C null-character]
        INTEGER(4) :: K    ! Index used for looping through blades.
        
        ! Set aviFAIL to 0 in each iteration:
        aviFAIL = 0
        
        ! Initialize all filter instance counters at 1
        objInst%instLPF = 1
        objInst%instSecLPF = 1
        objInst%instHPF = 1
        objInst%instNotchSlopes = 1
        objInst%instNotch = 1
        objInst%instPI = 1
        
        ! Set unused outputs to zero (See Appendix A of Bladed User's Guide):
        avrSWAP(35) = 1.0 ! Generator contactor status: 1=main (high speed) variable-speed generator
        avrSWAP(36) = 0.0 ! Shaft brake status: 0=off
        avrSWAP(41) = 0.0 ! Demanded yaw actuator torque
        avrSWAP(46) = 0.0 ! Demanded pitch rate (Collective pitch)
        avrSWAP(55) = 0.0 ! Pitch override: 0=yes
        avrSWAP(56) = 0.0 ! Torque override: 0=yes
        avrSWAP(65) = 0.0 ! Number of variables returned for logging
        avrSWAP(72) = 0.0 ! Generator start-up resistance
        avrSWAP(79) = 0.0 ! Request for loads: 0=none
        avrSWAP(80) = 0.0 ! Variable slip current status
        avrSWAP(81) = 0.0 ! Variable slip current demand
        
        ! Read any External Controller Parameters specified in the User Interface
        !   and initialize variables:
        IF (LocalVar%iStatus == 0) THEN ! .TRUE. if we're on the first call to the DLL
            
            ! Inform users that we are using this user-defined routine:
            aviFAIL = 1
            ErrMsg = '                                                                              '//NEW_LINE('A')// &
                     '------------------------------------------------------------------------------'//NEW_LINE('A')// &
                     'Running a controller implemented through NREL''s ROSCO Toolbox                    '//NEW_LINE('A')// &
                     'A wind turbine controller framework for public use in the scientific field    '//NEW_LINE('A')// &
                     'Developed in collaboration: National Renewable Energy Laboratory              '//NEW_LINE('A')// &
                     '                            Delft University of Technology, The Netherlands   '//NEW_LINE('A')// &
                     'Primary development by (listed alphabetically): Nikhar J. Abbas               '//NEW_LINE('A')// &
                     '                                                Sebastiaan P. Mulders         '//NEW_LINE('A')// &
                     '                                                Jan-Willem van Wingerden      '//NEW_LINE('A')// &
                     'Visit our GitHub-page to contribute to this project:                          '//NEW_LINE('A')// &
                     'https://github.com/NREL/ROSCO                                                 '//NEW_LINE('A')// &
                     '------------------------------------------------------------------------------'

            LocalVar%ACC_INFILE_SIZE = NINT(avrSWAP(50))
            Allocate(LocalVar%ACC_INFILE(LocalVar%ACC_INFILE_SIZE))
            LocalVar%ACC_INFILE = accINFILE
            
            CALL ReadControlParameterFileSub(CntrPar, accINFILE, NINT(avrSWAP(50)))

            IF (CntrPar%WE_Mode > 0) THEN
                CALL READCpFile(CntrPar,PerfData)
            ENDIF
            ! Initialize testValue (debugging variable)
            LocalVar%TestType = 0
        
            ! Initialize the SAVED variables:

            ! DO K = 1,LocalVar%NumBl
            LocalVar%PitCom = LocalVar%BlPitch ! This will ensure that the variable speed controller picks the correct control region and the pitch controller picks the correct gain on the first call
            ! END DO
            
            LocalVar%Y_AccErr = 0.0  ! This will ensure that the accumulated yaw error starts at zero
            LocalVar%Y_YawEndT = -1.0 ! This will ensure that the initial yaw end time is lower than the actual time to prevent initial yawing
            
            ! Wind speed estimator initialization, we always assume an initial wind speed of 10 m/s
            LocalVar%WE_Vw = 10
            LocalVar%WE_VwI = LocalVar%WE_Vw - CntrPar%WE_Gamma*LocalVar%RotSpeed
            
            ! Setpoint Smoother initialization to zero
            LocalVar%SS_DelOmegaF = 0

            ! Generator Torque at K omega^2 or rated
            IF (LocalVar%GenSpeed > 0.98 * CntrPar%PC_RefSpd) THEN
                LocalVar%GenTq = CntrPar%VS_RtTq
            ELSE
                LocalVar%GenTq = min(CntrPar%VS_RtTq, CntrPar%VS_Rgn2K*LocalVar%GenSpeed*LocalVar%GenSpeed)
            ENDIF            
            LocalVar%VS_LastGenTrq = LocalVar%GenTq       
            
            ! Check validity of input parameters:
            CALL Assert(LocalVar, CntrPar, avrSWAP, aviFAIL, ErrMsg, size_avcMSG)
            

        ENDIF
    END SUBROUTINE SetParameters
    ! -----------------------------------------------------------------------------------
    ! Read all constant control parameters from DISCON.IN parameter file
    SUBROUTINE ReadCpFile(CntrPar,PerfData)
        USE ROSCO_Types, ONLY : PerformanceData, ControlParameters

        INTEGER(4), PARAMETER :: UnPerfParameters = 89
        TYPE(PerformanceData), INTENT(INOUT) :: PerfData
        TYPE(ControlParameters), INTENT(INOUT) :: CntrPar
        ! Local variables
        INTEGER(4)                  :: i ! iteration index
        OPEN(unit=UnPerfParameters, file=TRIM(CntrPar%PerfFileName), status='old', action='read') ! Should put input file into DISCON.IN
        
        ! ----------------------- Axis Definitions ------------------------
        READ(UnPerfParameters, *)
        READ(UnPerfParameters, *)
        READ(UnPerfParameters, *)
        READ(UnPerfParameters, *)
        ALLOCATE(PerfData%Beta_vec(CntrPar%PerfTableSize(1)))
        READ(UnPerfParameters, *) PerfData%Beta_vec
        READ(UnPerfParameters, *) 
        ALLOCATE(PerfData%TSR_vec(CntrPar%PerfTableSize(2)))
        READ(UnPerfParameters, *) PerfData%TSR_vec

        ! ----------------------- Read Cp, Ct, Cq, Tables ------------------------
        READ(UnPerfParameters, *) 
        READ(UnPerfParameters, *) ! Input file should contains wind speed information here - unneeded for now
        READ(UnPerfParameters, *) 
        READ(UnPerfParameters, *) 
        READ(UnPerfParameters, *) 
        ALLOCATE(PerfData%Cp_mat(CntrPar%PerfTableSize(2),CntrPar%PerfTableSize(1)))
        DO i = 1,CntrPar%PerfTableSize(2)
            READ(UnPerfParameters, *) PerfData%Cp_mat(i,:) ! Read Cp table
        END DO
        READ(UnPerfParameters, *) 
        READ(UnPerfParameters, *) 
        READ(UnPerfParameters, *) 
        READ(UnPerfParameters, *) 
        ALLOCATE(PerfData%Ct_mat(CntrPar%PerfTableSize(1),CntrPar%PerfTableSize(2)))
        DO i = 1,CntrPar%PerfTableSize(2)
            READ(UnPerfParameters, *) PerfData%Ct_mat(i,:) ! Read Ct table
        END DO
        READ(UnPerfParameters, *) 
        READ(UnPerfParameters, *) 
        READ(UnPerfParameters, *) 
        READ(UnPerfParameters, *) 
        ALLOCATE(PerfData%Cq_mat(CntrPar%PerfTableSize(1),CntrPar%PerfTableSize(2)))
        DO i = 1,CntrPar%PerfTableSize(2)
            READ(UnPerfParameters, *) PerfData%Ct_mat(i,:) ! Read Cq table
        END DO
    
      END SUBROUTINE ReadCpFile

      SUBROUTINE WriteRestartFile(LocalVar, CntrPar, objInst, accINFILE, accINFILE_size)
        USE ROSCO_Types, ONLY : LocalVariables, ObjectInstances, ControlParameters
      
        TYPE(LocalVariables), INTENT(IN)                 :: LocalVar
        TYPE(ControlParameters), INTENT(INOUT) :: CntrPar
        TYPE(ObjectInstances), INTENT(INOUT) :: objInst
        INTEGER(4), INTENT(IN)                         :: accINFILE_size               ! size of DISCON input filename
        CHARACTER(accINFILE_size),  INTENT(IN   )      :: accINFILE(accINFILE_size)    ! DISCON input filename  
       
        INTEGER(4), PARAMETER        :: Un            = 87                              ! I/O unit for pack/unpack (checkpoint & restart)
        INTEGER(4)                   :: I                                               ! Generic index.
        CHARACTER(SIZE(accINFILE)-1) :: InFile    ! a Fortran version of the input C string (not considered an array here)    [subtract 1 for the C null-character]
        INTEGER(4)                   :: ErrStat
        CHARACTER(128)               :: ErrMsg                                          ! a Fortran version of the C string argument (not considered an array here) [subtract 1 for the C null-character] 
      
        InFile = TRANSFER( accINFILE(1:LEN(InFile)),  InFile )
        I = INDEX(InFile,C_NULL_CHAR) - 1         ! if this has a c null character at the end...
        IF ( I > 0 ) InFile = InFile(1:I)         ! remove it
      
        OPEN( Un, FILE=TRIM( InFile ), STATUS='UNKNOWN', FORM='UNFORMATTED' , ACCESS='STREAM', IOSTAT=ErrStat, ACTION='WRITE' )
      
        IF ( ErrStat /= 0 ) THEN
           ErrMsg  = 'Cannot open file "'//TRIM( InFile )//'". Another program may have locked it for writing.'
           
        ELSE

           WRITE( Un, IOSTAT=ErrStat) LocalVar%FA_AccHPF
           WRITE( Un, IOSTAT=ErrStat) LocalVar%FA_AccHPFI
           WRITE( Un, IOSTAT=ErrStat) LocalVar%FA_PitCom(1)
           WRITE( Un, IOSTAT=ErrStat) LocalVar%FA_PitCom(2)     
           WRITE( Un, IOSTAT=ErrStat) LocalVar%FA_PitCom(3)
           WRITE( Un, IOSTAT=ErrStat) LocalVar%GenSpeedF     
           WRITE( Un, IOSTAT=ErrStat) LocalVar%GenTq
           WRITE( Un, IOSTAT=ErrStat) LocalVar%GenTqMeas
           WRITE( Un, IOSTAT=ErrStat) LocalVar%GenArTq
           WRITE( Un, IOSTAT=ErrStat) LocalVar%GenBrTq
           WRITE( Un, IOSTAT=ErrStat) LocalVar%GlobalState
           WRITE( Un, IOSTAT=ErrStat) LocalVar%IPC_PitComF(1)
           WRITE( Un, IOSTAT=ErrStat) LocalVar%IPC_PitComF(2)     
           WRITE( Un, IOSTAT=ErrStat) LocalVar%IPC_PitComF(3)
           WRITE( Un, IOSTAT=ErrStat) LocalVar%PC_KP
           WRITE( Un, IOSTAT=ErrStat) LocalVar%PC_KI
           WRITE( Un, IOSTAT=ErrStat) LocalVar%PC_KD           
           WRITE( Un, IOSTAT=ErrStat) LocalVar%PC_TF
           WRITE( Un, IOSTAT=ErrStat) LocalVar%PC_MaxPit
           WRITE( Un, IOSTAT=ErrStat) LocalVar%PC_MinPit
           WRITE( Un, IOSTAT=ErrStat) LocalVar%PC_PitComT
           WRITE( Un, IOSTAT=ErrStat) LocalVar%PC_PitComT_IPC(1)
           WRITE( Un, IOSTAT=ErrStat) LocalVar%PC_PitComT_IPC(2)
           WRITE( Un, IOSTAT=ErrStat) LocalVar%PC_PitComT_IPC(3)     
           WRITE( Un, IOSTAT=ErrStat) LocalVar%PC_PwrErr
           WRITE( Un, IOSTAT=ErrStat) LocalVar%PC_SineExcitation
           WRITE( Un, IOSTAT=ErrStat) LocalVar%PC_SpdErr
           WRITE( Un, IOSTAT=ErrStat) LocalVar%PC_State
           WRITE( Un, IOSTAT=ErrStat) LocalVar%PitCom(1)
           WRITE( Un, IOSTAT=ErrStat) LocalVar%PitCom(2)
           WRITE( Un, IOSTAT=ErrStat) LocalVar%PitCom(3)     
           WRITE( Un, IOSTAT=ErrStat) LocalVar%SS_DelOmegaF
           WRITE( Un, IOSTAT=ErrStat) LocalVar%TestType
           WRITE( Un, IOSTAT=ErrStat) LocalVar%VS_LastGenTrq
           WRITE( Un, IOSTAT=ErrStat) LocalVar%VS_MechGenPwr
           WRITE( Un, IOSTAT=ErrStat) LocalVar%VS_SpdErrAr
           WRITE( Un, IOSTAT=ErrStat) LocalVar%VS_SpdErrBr
           WRITE( Un, IOSTAT=ErrStat) LocalVar%VS_SpdErr
           WRITE( Un, IOSTAT=ErrStat) LocalVar%VS_State
           WRITE( Un, IOSTAT=ErrStat) LocalVar%WE_Vw
           WRITE( Un, IOSTAT=ErrStat) LocalVar%WE_VwI
           WRITE( Un, IOSTAT=ErrStat) LocalVar%WE_VwIdot
           WRITE( Un, IOSTAT=ErrStat) LocalVar%Y_AccErr
           WRITE( Un, IOSTAT=ErrStat) LocalVar%Y_ErrLPFFast
           WRITE( Un, IOSTAT=ErrStat) LocalVar%Y_ErrLPFSlow
           WRITE( Un, IOSTAT=ErrStat) LocalVar%Y_MErr
           WRITE( Un, IOSTAT=ErrStat) LocalVar%Y_YawEndT
           WRITE( Un, IOSTAT=ErrStat) LocalVar%SD
           WRITE( Un, IOSTAT=ErrStat) LocalVar%Fl_PitCom
           WRITE( Un, IOSTAT=ErrStat) LocalVar%NACIMU_FA_AccF
           WRITE( Un, IOSTAT=ErrStat) LocalVar%Flp_Angle(1)
           WRITE( Un, IOSTAT=ErrStat) LocalVar%Flp_Angle(2)
           WRITE( Un, IOSTAT=ErrStat) LocalVar%Flp_Angle(3)
           WRITE( Un, IOSTAT=ErrStat) LocalVar%ACC_INFILE_SIZE
           WRITE( Un, IOSTAT=ErrStat) LocalVar%ACC_INFILE(:)

           WRITE( Un, IOSTAT=ErrStat) objInst%instLPF
           WRITE( Un, IOSTAT=ErrStat) objInst%instSecLPF
           WRITE( Un, IOSTAT=ErrStat) objInst%instHPF
           WRITE( Un, IOSTAT=ErrStat) objInst%instNotchSlopes
           WRITE( Un, IOSTAT=ErrStat) objInst%instNotch
           WRITE( Un, IOSTAT=ErrStat) objInst%instPI
           
           CLOSE ( Un )
           
        ENDIF
      
      END SUBROUTINE WriteRestartFile
      
      SUBROUTINE ReadRestartFile(LocalVar, CntrPar, objInst, PerfData, accINFILE, accINFILE_size)
        USE ROSCO_Types, ONLY : LocalVariables, ControlParameters, ObjectInstances, PerformanceData
      
        TYPE(LocalVariables), INTENT(INOUT) :: LocalVar
        TYPE(ControlParameters), INTENT(INOUT) :: CntrPar
        TYPE(ObjectInstances), INTENT(INOUT) :: objInst
        TYPE(PerformanceData), INTENT(INOUT) :: PerfData
        INTEGER(4), INTENT(IN)                         :: accINFILE_size               ! size of DISCON input filename
        CHARACTER(accINFILE_size),  INTENT(IN   )      :: accINFILE(accINFILE_size)    ! DISCON input filename  
      
        INTEGER(4), PARAMETER        :: Un            = 87                              ! I/O unit for pack/unpack (checkpoint & restart)
        INTEGER(4)                   :: I                                               ! Generic index.  
        CHARACTER(SIZE(accINFILE)-1) :: InFile    ! a Fortran version of the input C string (not considered an array here)    [subtract 1 for the C null-character]
        INTEGER(4)                   :: ErrStat  
        CHARACTER(128)               :: ErrMsg                                          ! a Fortran version of the C string argument (not considered an array here) [subtract 1 for the C null-character]
        
        InFile = TRANSFER( accINFILE(1:LEN(InFile)),  InFile )
        I = INDEX(InFile,C_NULL_CHAR) - 1         ! if this has a c null character at the end...
        IF ( I > 0 ) InFile = InFile(1:I)         ! remove it
      
        OPEN( Un, FILE=TRIM( InFile ), STATUS='OLD', FORM='UNFORMATTED', ACCESS='STREAM', IOSTAT=ErrStat, ACTION='READ' )
      
        IF ( ErrStat /= 0 ) THEN
           ErrMsg  = 'Cannot open file "'//TRIM( InFile )//'". Another program may have locked it for writing.'
           
        ELSE

            READ( Un, IOSTAT=ErrStat) LocalVar%FA_AccHPF
           READ( Un, IOSTAT=ErrStat) LocalVar%FA_AccHPFI
           READ( Un, IOSTAT=ErrStat) LocalVar%FA_PitCom(1)
           READ( Un, IOSTAT=ErrStat) LocalVar%FA_PitCom(2)     
           READ( Un, IOSTAT=ErrStat) LocalVar%FA_PitCom(3)
           READ( Un, IOSTAT=ErrStat) LocalVar%GenSpeedF     
           READ( Un, IOSTAT=ErrStat) LocalVar%GenTq
           READ( Un, IOSTAT=ErrStat) LocalVar%GenTqMeas
           READ( Un, IOSTAT=ErrStat) LocalVar%GenArTq
           READ( Un, IOSTAT=ErrStat) LocalVar%GenBrTq
           READ( Un, IOSTAT=ErrStat) LocalVar%GlobalState
           READ( Un, IOSTAT=ErrStat) LocalVar%IPC_PitComF(1)
           READ( Un, IOSTAT=ErrStat) LocalVar%IPC_PitComF(2)     
           READ( Un, IOSTAT=ErrStat) LocalVar%IPC_PitComF(3)
           READ( Un, IOSTAT=ErrStat) LocalVar%PC_KP
           READ( Un, IOSTAT=ErrStat) LocalVar%PC_KI           
           READ( Un, IOSTAT=ErrStat) LocalVar%PC_KD
           READ( Un, IOSTAT=ErrStat) LocalVar%PC_TF
           READ( Un, IOSTAT=ErrStat) LocalVar%PC_MaxPit
           READ( Un, IOSTAT=ErrStat) LocalVar%PC_MinPit
           READ( Un, IOSTAT=ErrStat) LocalVar%PC_PitComT
           READ( Un, IOSTAT=ErrStat) LocalVar%PC_PitComT_IPC(1)
           READ( Un, IOSTAT=ErrStat) LocalVar%PC_PitComT_IPC(2)
           READ( Un, IOSTAT=ErrStat) LocalVar%PC_PitComT_IPC(3)     
           READ( Un, IOSTAT=ErrStat) LocalVar%PC_PwrErr
           READ( Un, IOSTAT=ErrStat) LocalVar%PC_SineExcitation
           READ( Un, IOSTAT=ErrStat) LocalVar%PC_SpdErr
           READ( Un, IOSTAT=ErrStat) LocalVar%PC_State
           READ( Un, IOSTAT=ErrStat) LocalVar%PitCom(1)
           READ( Un, IOSTAT=ErrStat) LocalVar%PitCom(2)
           READ( Un, IOSTAT=ErrStat) LocalVar%PitCom(3)     
           READ( Un, IOSTAT=ErrStat) LocalVar%SS_DelOmegaF
           READ( Un, IOSTAT=ErrStat) LocalVar%TestType
           READ( Un, IOSTAT=ErrStat) LocalVar%VS_LastGenTrq
           READ( Un, IOSTAT=ErrStat) LocalVar%VS_MechGenPwr
           READ( Un, IOSTAT=ErrStat) LocalVar%VS_SpdErrAr
           READ( Un, IOSTAT=ErrStat) LocalVar%VS_SpdErrBr
           READ( Un, IOSTAT=ErrStat) LocalVar%VS_SpdErr
           READ( Un, IOSTAT=ErrStat) LocalVar%VS_State
           READ( Un, IOSTAT=ErrStat) LocalVar%WE_Vw
           READ( Un, IOSTAT=ErrStat) LocalVar%WE_VwI
           READ( Un, IOSTAT=ErrStat) LocalVar%WE_VwIdot
           READ( Un, IOSTAT=ErrStat) LocalVar%Y_AccErr
           READ( Un, IOSTAT=ErrStat) LocalVar%Y_ErrLPFFast
           READ( Un, IOSTAT=ErrStat) LocalVar%Y_ErrLPFSlow
           READ( Un, IOSTAT=ErrStat) LocalVar%Y_MErr
           READ( Un, IOSTAT=ErrStat) LocalVar%Y_YawEndT
           READ( Un, IOSTAT=ErrStat) LocalVar%SD
           READ( Un, IOSTAT=ErrStat) LocalVar%Fl_PitCom
           READ( Un, IOSTAT=ErrStat) LocalVar%NACIMU_FA_AccF
           READ( Un, IOSTAT=ErrStat) LocalVar%Flp_Angle(1)
           READ( Un, IOSTAT=ErrStat) LocalVar%Flp_Angle(2)
           READ( Un, IOSTAT=ErrStat) LocalVar%Flp_Angle(3)
           READ( Un, IOSTAT=ErrStat) LocalVar%ACC_INFILE_SIZE
           Allocate(LocalVar%ACC_INFILE(LocalVar%ACC_INFILE_SIZE))
           READ( Un, IOSTAT=ErrStat) LocalVar%ACC_INFILE

           READ( Un, IOSTAT=ErrStat) objInst%instLPF
           READ( Un, IOSTAT=ErrStat) objInst%instSecLPF
           READ( Un, IOSTAT=ErrStat) objInst%instHPF
           READ( Un, IOSTAT=ErrStat) objInst%instNotchSlopes
           READ( Un, IOSTAT=ErrStat) objInst%instNotch
           READ( Un, IOSTAT=ErrStat) objInst%instPI
           
           CLOSE ( Un )
           
        ENDIF

        CALL ReadControlParameterFileSub(CntrPar, LocalVar%ACC_INFILE, LocalVar%ACC_INFILE_SIZE)

        IF (CntrPar%WE_Mode > 0) THEN
           CALL READCpFile(CntrPar,PerfData)
        ENDIF
        
      
      END SUBROUTINE ReadRestartFile
            
END MODULE ReadSetParameters
