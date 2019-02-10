/*************************************************************************************************************************************************/
/*!     \file TPS23861.h
*
*       \brief Register File containing hardware definitions and other configurations by the TPS23861 software API functions
*
*       \date January 2018
*
*       This file contains the software structures, defines, and prototypes for the TPS23861.c API functions
*/
/**************************************************************************************************************************************************
*       Copyright © 2013-2018 Texas Instruments Incorporated - http://www.ti.com/                                                                      *
***************************************************************************************************************************************************
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*                                                                                                                                                 *
**************************************************************************************************************************************************/
#ifndef __TPS238x_H_
#define __TPS238x_H_


#include "sys_init.h"

#define TPS_SUCCESSFUL                         0x00
#define TPS_ERR_PORT_IN_USE                    0x80
#define TPS_ERR_PORT_NOT_IN_USE                0x81
#define TPS_ERR_NO_PORT_AVAILABLE              0x82
#define TPS_ERR_I2C_ERROR                      0x83
#define TPS_ERR_I2C_ADDRESS_MISMATCH           0x40
#define TPS_ERR_I2C_AUTOBIT_MISMATCH           0x41



#define TPS_GIVE_ME_NEXT_CHANNEL_AVAILABLE        0xff
#define TPS_CHANNEL_NOT_REGISTERED_VALUE          0xff

#define SWIZZLE_BYTES(x)                       {unsigned short y; y = x; x = (((y & 0xff) << 8) | (y >> 8)); }     ///< Used to switch MSB and LSB bytes in an unsigned short variable

#define TPS_MAX_SYSTEM_CHANNELS                48

#define TPS_VOLTAGE_SCALE_FACTOR               3662
#define TPS_CURRENT_SCALE_FACTOR               61039

extern uint8_t firmwareversion;

typedef enum {
  TPS_OFF                                    = 0x0,
  TPS_ON                                     = 0x1
} TPS238x_On_Off_t;

///Used with functions that indicate a single port.
/// @note that the hardware registers use a bit value with a separate bit for each port number. Conversion can be done with the CONVERT_PORT_NUM () macro
typedef enum {
  TPS238X_PORT_1                             = 1,
  TPS238X_PORT_2                             = 2,
  TPS238X_PORT_3                             = 3,
  TPS238X_PORT_4                             = 4
} TPS238x_PortNum_t;

/// This structure is used with many functions where multiple ports can be indicated at the same time
#define PORT_1_VALUE                           0x1            ///< Bit value of the TPS238x_Ports_t Port 1
#define PORT_2_VALUE                           0x2            ///< Bit value of the TPS238x_Ports_t Port 2
#define PORT_3_VALUE                           0x4            ///< Bit value of the TPS238x_Ports_t Port 3
#define PORT_4_VALUE                           0x8            ///< Bit value of the TPS238x_Ports_t Port 4

typedef unsigned char TPS238x_Ports_t;

#define TPS238X_ALL_PORTS                    ((TPS238x_Ports_t)0xf)
#define TPS238X_ODD_PORTS_HIGH_POWER         ((TPS238x_Ports_t)0x5)

typedef struct {
    unsigned int      i2cAddress;
    TPS238x_PortNum_t devicePortNum;
} TPS238x_System_Port_Map_t;


/// Interrupt Register - used in tps_GetInterruptStatus() to return events <br>
///   Hardware Command 00h; 1 data byte; read only                         <br><br>
/// Provides the faults and events that are currently active.
/// @note The interrupt status flag may be active for events or faults that are masked. No interrupt will be generated, but the status
///        may be present along side of another event which is allowed to generate an interrupt.
typedef struct {
    unsigned char PEC_Power_Enable_Change                    : 1;   ///< Indicates a power enable status change occurred on at least one port
    unsigned char PGC_Power_Good_Change                      : 1;   ///< Indicates a power good change occurred on at least one port
    unsigned char DISF_Disconnect_Event                      : 1;   ///< Indicates a disconnect event occurred on at least one port
    unsigned char DETC_Detection_Cycle                       : 1;   ///< Indicates at least one detection cycle occurred on at least one port
    unsigned char CLASC_Classification_Cycle                 : 1;   ///< Indicates at least one classification cycle occurred on at least one port
    unsigned char IFAULT_PCUT_ILIM_Fault                     : 1;   ///< Indicates that an ICUT or ILIM fault occurred on at least one port
    unsigned char INRF_Inrush_Fault                          : 1;   ///< Indicates that an Inrush fault occurred on at least one port
    unsigned char SUPF_Supply_Event_Fault                    : 1;   ///< Indicates that a supply event fault occurred
}TPS238X_Interrupt_Register_t;

#define PEC                                    0x01   ///< Indicates a power enable status change occurred on at least one port
#define PGC                                    0x02   ///< Indicates a power good change occurred on at least one port
#define DISF                                   0x04   ///< Indicates a disconnect event occurred on at least one port
#define DETC                                   0x08   ///< Indicates at least one detection cycle occurred on at least one port
#define CLASC                                  0x10   ///< Indicates at least one classification cycle occurred on at least one port
#define IFAULT                                 0x20   ///< Indicates that an ICUT or ILIM fault occurred on at least one port
#define INRF                                   0x40   ///< Indicates that an Inrush fault occurred on at least one port
#define SUPF                                   0x80   ///< Indicates that a supply event fault occurred

/// Interrupt Mask Register - Used in tps_GetInterruptMask() and tps_SetInterruptMask() <br>
///                Hardware - Command 01h; 1 data byte; read/write                      <br><br>
/// Enables the various events and faults to generate interrupts
/// @note that writing a 0 masks the events. Writing a 1 to the bit unmasks the interrupt
typedef struct {
    unsigned char PEMSK_Power_Enable_Unmask                  : 1;   ///< Enable power enable interrupts
    unsigned char PGMSK_Power_Good_Unmask                    : 1;   ///< Enable power good interrupts
    unsigned char DIMSK_Disconnect_Unmask                    : 1;   ///< Enable disconnect event interrupts
    unsigned char DEMSK_Detection_Cycle_Unmask               : 1;   ///< Enable detection cycle event interrupts
    unsigned char CLMSK_Classificiation_Cycle_Unmask         : 1;   ///< Enable classification cycle event interrupts
    unsigned char IFMSK_IFAULT_Unmask                        : 1;   ///< Enable ICUT or OLIM fault interrupts
    unsigned char INMSK_Inrush_Fault_Unmask                  : 1;   ///< Enable Inrush fault interrupts
    unsigned char SUMSK_Supply_Event_Fault_Unmask            : 1;   ///< Enable supply event fault interrupts
}TPS238X_Interrupt_Mask_Register_t;

#define PEMSK                                  0x01   ///< Enable power enable interrupts
#define PGMSK                                  0x02   ///< Enable power good interrupts
#define DIMSK                                  0x04   ///< Enable disconnect event interrupts
#define DEMSK                                  0x08   ///< Enable detection cycle event interrupts
#define CLMSK                                  0x10   ///< Enable classification cycle event interrupts
#define IFMSK                                  0x20   ///< Enable ICUT or OLIM fault interrupts
#define INMSK                                  0x40   ///< Enable Inrush fault interrupts
#define SUMSK                                  0x80   ///< Enable supply event fault interrupts

/// Power Event Register - Software API uses TPS238x_Ports_t; Software retrieves these events as part of tps_GetAllInterruptEvents() <br>
///             Hardware - Command 02h; 1 data byte; read only     <br>
///                      - Command 03h; 1 data byte; Clear on read <br><br>
/// Changes to the power enable or power good status occurred for at least one port
typedef struct {
    unsigned char PEC1_Power_Enable_Port_1_Event             : 1;   ///< Change to power enable status for port 1
    unsigned char PEC2_Power_Enable_Port_2_Event             : 1;   ///< Change to power enable status for port 2
    unsigned char PEC3_Power_Enable_Port_3_Event             : 1;   ///< Change to power enable status for port 3
    unsigned char PEC4_Power_Enable_Port_4_Event             : 1;   ///< Change to power enable status for port 4
    unsigned char PGC1_Power_Good_Port_1_Event               : 1;   ///< Change to power good status for port 1
    unsigned char PGC2_Power_Good_Port_2_Event               : 1;   ///< Change to power good status for port 2
    unsigned char PGC3_Power_Good_Port_3_Event               : 1;   ///< Change to power good status for port 3
    unsigned char PGC4_Power_Good_Port_4_Event               : 1;   ///< Change to power good status for port 4
}TPS238X_Power_Enable_Register_t;

#define PEC1                                   0x01   ///< Change to power enable status for port 1
#define PEC2                                   0x02   ///< Change to power enable status for port 2
#define PEC3                                   0x04   ///< Change to power enable status for port 3
#define PEC4                                   0x08   ///< Change to power enable status for port 4
#define PGC1                                   0x10   ///< Change to power good status for port 1
#define PGC2                                   0x20   ///< Change to power good status for port 2
#define PGC3                                   0x40   ///< Change to power good status for port 3
#define PGC4                                   0x80   ///< Change to power good status for port 4

#define POWER_GOOD_EVENT_SHIFT                 0
#define POWER_ENABLE_EVENT_SHIFT               4

/// Detection Event Register - Software API uses TPS238x_Ports_t; Software retrieves these events as part of tps_GetAllInterruptEvents() <br>
///                 Hardware - Command 04h; 1 data byte; read only      <br>
///                          - Command 05h; 1 data byte; Clear on read  <br><br>
/// Detection and/or classification cycles occurred on at least one port
typedef struct {
    unsigned char DETC1_Detection_Cycle_Channel_1_Event         : 1;   ///< Detection cycle occurred on port 1
    unsigned char DETC2_Detection_Cycle_Channel_2_Event         : 1;   ///< Detection cycle occurred on port 2
    unsigned char DETC3_Detection_Cycle_Channel_3_Event         : 1;   ///< Detection cycle occurred on port 3
    unsigned char DETC4_Detection_Cycle_Channel_4_Event         : 1;   ///< Detection cycle occurred on port 4
    unsigned char CLSC1_Classification_Cycle_Channel_1_Event    : 1;   ///< Classification cycle occurred on port 1
    unsigned char CLSC2_Classification_Cycle_Channel_2_Event    : 1;   ///< Classification cycle occurred on port 2
    unsigned char CLSC3_Classification_Cycle_Channel_3_Event    : 1;   ///< Classification cycle occurred on port 3
    unsigned char CLSC4_Classification_Cycle_Channel_4_Event    : 1;   ///< Classification cycle occurred on port 4
}TPS238X_Detection_Event_Register_t;

#define DETC1                                  0x01   ///< Detection cycle occurred on port 1
#define DETC2                                  0x02   ///< Detection cycle occurred on port 2
#define DETC3                                  0x04   ///< Detection cycle occurred on port 3
#define DETC4                                  0x08   ///< Detection cycle occurred on port 4
#define CLSC1                                  0x10   ///< Classification cycle occurred on port 1
#define CLSC2                                  0x20   ///< Classification cycle occurred on port 2
#define CLSC3                                  0x40   ///< Classification cycle occurred on port 3
#define CLSC4                                  0x80   ///< Classification cycle occurred on port 4

#define DETECTION_EVENT_SHIFT                  0
#define CLASSIFICATION_EVENT_SHIFT             4

/// Fault Event Register - Software API uses TPS238x_Ports_t; Software retrieves these events as part of tps_GetAllInterruptEvents() <br>
///             Hardware - Command 06h; 1 data byte; read only        <br>
///                      - Command 07h; 1 data byte; Clear on read    <br><br>
/// ICUT fault and disconnect events occurred for at least one port
typedef struct {
    unsigned char PCUT1_PCUT_Fault_Port_1_Event              : 1;    ///< ICUT fault occurred at port 1
    unsigned char PCUT2_PCUT_Fault_Port_2_Event              : 1;    ///< ICUT fault occurred at port 2
    unsigned char PCUT3_PCUT_Fault_Port_3_Event              : 1;    ///< ICUT fault occurred at port 3
    unsigned char PCUT4_PCUT_Fault_Port_4_Event              : 1;    ///< ICUT fault occurred at port 4
    unsigned char DISF1_Disconnect_Port_1_Event              : 1;    ///< Disconnect event occurred at port 1
    unsigned char DISF2_Disconnect_Port_2_Event              : 1;    ///< Disconnect event occurred at port 2
    unsigned char DISF3_Disconnect_Port_3_Event              : 1;    ///< Disconnect event occurred at port 3
    unsigned char DISF4_Disconnect_Port_4_Event              : 1;    ///< Disconnect event occurred at port 4
}TPS238X_Fault_Event_Register_t;

#define PCUT1                                  0x01    ///< ICUT fault occurred at port 1
#define PCUT2                                  0x02    ///< ICUT fault occurred at port 2
#define PCUT3                                  0x04    ///< ICUT fault occurred at port 3
#define PCUT4                                  0x08    ///< ICUT fault occurred at port 4
#define DISF1                                  0x10    ///< Disconnect event occurred at port 1
#define DISF2                                  0x20    ///< Disconnect event occurred at port 2
#define DISF3                                  0x40    ///< Disconnect event occurred at port 3
#define DISF4                                  0x80    ///< Disconnect event occurred at port 4

#define PCUT_EVENT_SHIFT                       0
#define DISCONNECT_EVENT_SHIFT                 4

/// Inrush/ILIM Event Register - Software API uses TPS238x_Ports_t; Software retrieves these events as part of tps_GetAllInterruptEvents() <br>
///                   Hardware - Command 08h; 1 data byte; read only      <br>
///                            - Command 09h; 1 data byte; Clear on read  <br><br>
/// Inrush and ILIM fault events have occurred for at least one port
typedef struct {
    unsigned char INR1_Inrush_Fault_Port_1_Event             : 1;    ///< Inrush fault occurred at port 1
    unsigned char INR2_Inrush_Fault_Port_2_Event             : 1;    ///< Inrush fault occurred at port 2
    unsigned char INR3_Inrush_Fault_Port_3_Event             : 1;    ///< Inrush fault occurred at port 3
    unsigned char INR4_Inrush_Fault_Port_4_Event             : 1;    ///< Inrush fault occurred at port 4
    unsigned char ILIM1_Limit_Output_Current_Port_1_Event    : 1;    ///< ILIM fault occurred at port 1
    unsigned char ILIM2_Limit_Output_Current_Port_2_Event    : 1;    ///< ILIM fault occurred at port 2
    unsigned char ILIM3_Limit_Output_Current_Port_3_Event    : 1;    ///< ILIM fault occurred at port 3
    unsigned char ILIM4_Limit_Output_Current_Port_4_Event    : 1;    ///< ILIM fault occurred at port 4
}TPS238X_Inrush_ILIM_Event_Register_t;

#define INR1                                   0x01    ///< Inrush fault occurred at port 1
#define INR2                                   0x02    ///< Inrush fault occurred at port 2
#define INR3                                   0x04    ///< Inrush fault occurred at port 3
#define INR4                                   0x08    ///< Inrush fault occurred at port 4
#define ILIM1                                  0x10    ///< ILIM fault occurred at port 1
#define ILIM2                                  0x20    ///< ILIM fault occurred at port 2
#define ILIM3                                  0x40    ///< ILIM fault occurred at port 3
#define ILIM4                                  0x80    ///< ILIM fault occurred at port 4

#define INRUSH_EVENT_SHIFT                     0
#define ILIM_EVENT_SHIFT                       4


/// Supply Event Register - Software retrieves these events as part of tps_GetAllInterruptEvents() <br>
///              Hardware - Command 0Ah; 1 data byte; read only      <br>
///                       - Command 0Bh; 1 data byte; Clear on read  <br><br>
/// Identifies events with the system power supply
typedef struct {
    unsigned char RAMFLT_SRAM_Fault_Event                    : 1;  ///<SRAM menmory falut
    unsigned char OSS_Event                                  : 1;  ///<OSS event has happened
    unsigned char Four_P12_PCUT_Event                        : 1;  ///<4P PCUT has happened on channel 12
    unsigned char Four_P34_PCUT_Event                        : 1;  ///<4P PCUT has happened on pair 12
    unsigned char VPUV_VPower_Undervoltage_Event             : 1;  ///< VPWR Undervoltage
    unsigned char VDWRN_Vdd_UVLO_Warining_Event              : 1;  ///< VDD falls below UVLO under the UVLO warning threshold
    unsigned char VDUV_Vdd_UVLO_Event                        : 1;  ///< VDD UVLO Occurred. Power on reset happened
    unsigned char TSD_Thermal_Shutdown_Event                 : 1;  ///< Thermal shutdown occurred
}TPS238X_Supply_Event_4PPCUT_Register_t;

#define RAMFLT                                 0x01
#define OSSE                                   0x02
#define PCUT12                                 0x04
#define PCUT34                                 0x08
#define VPUV                                   0x10
#define VDWRN                                  0x20
#define VDUV                                   0x40
#define TSD                                    0x80



/// Port Status Register - Results returned in tps_GetPortDetectClassStatus(), tps_GetPortDetectionStatus(), and tps_GetPortClassificationtionStatus() <br>
///             Hardware - Command 0Ch; Port 1; 1 data byte; read only  <br>
///                      - Command 0Dh; Port 2; 1 data byte; read only  <br>
///                      - Command 0Eh; Port 3; 1 data byte; read only  <br>
///                      - Command 0Fh; Port 4; 1 data byte; read only  <br><br>
/// Uses TPS238x_Classification_Status_t and TPS238x_Detection_Status_t to provide the most recent classification and detection results for the port
typedef struct {
    unsigned char Detect                                     : 4;
    unsigned char Class                                      : 4;
}TPS238X_Channel_Status_Register_t;

/// The classification status
typedef enum {
  CLASS_UNKNOWN                              = 0x0,   ///< Unknown - invalid
  CLASS_1                                    = 0x1,   ///< Class 1
  CLASS_2                                    = 0x2,   ///< Class 2
  CLASS_3                                    = 0x3,   ///< Class 3
  CLASS_4                                    = 0x4,   ///< Class 4
  CLASS_0                                    = 0x6,   ///< Class 0
  CLASS_OVERCURRENT                          = 0x7,   ///< Overcurrent - invalid
  CLASS_5_4P_SINGLE                          = 0x8,   ///< Class 5
  CLASS_6_4P_SINGLE                          = 0x9,   ///< Class 6
  CLASS_7_4P_SINGLE                          = 0xA,   ///< Class 7
  CLASS_8_4P_SINGLE                          = 0xB,   ///< Class 8
  CLASS_4PLUS_TYPE1                          = 0xC,   ///< Class 4+
  CLASS4_4P_DUAL                             = 0xD,   ///< Class 5 dual
  CLASS_MISMATCH                             = 0xF

} TPS238x_Classification_Status_t;

/// Detection status
typedef enum {
  DETECT_UNKNOWN                             = 0x0,   ///< Unknown - invalid
  DETECT_SHORT_CIRCUIT                       = 0x1,   ///< Short circuit (<1.8 kOhm) - invalid
  DETECT_RESIST_LOW                          = 0x3,   ///< Resistance too low - invalid
  DETECT_RESIST_VALID                        = 0x4,   ///< Resistance valid
  DETECT_RESIST_HIGH                         = 0x5,   ///< Resistance too hig - invalid
  DETECT_OPEN_CIRCUIT                        = 0x6,   ///< Open circuit - invalid
  DETECT_MOSFET_FAULT                        = 0xE   ///< MOSFET Fault - invalid
} TPS238x_Detection_Status_t;


#define DETECT                                 0x0F
#define CLASS                                  0xF0
#define CLASS_SHIFT                            4
#define DETECT_SHIFT                           0
#define GET_DETECT(x)                          (x & DETECT)
#define GET_CLASS(x)                           (x >> CLASS_SHIFT)


/// Power Status Register - Software API uses TPS238x_Ports_t; Status returned in the
///                            tps_GetPowerStatus(), tps_GetPowerEnableStatus(), and tps_GetPowerGoodStatus () <br>
///              Hardware - Command 10h; 1 data byte; read only   <br><br>
/// Provides status for each port about the power enable and power good settings
typedef struct {
    unsigned char PE1_Power_Enable_Port_1_Status             : 1;   ///< Port 1 has power enabled
    unsigned char PE2_Power_Enable_Port_2_Status             : 1;   ///< Port 2 has power enabled
    unsigned char PE3_Power_Enable_Port_3_Status             : 1;   ///< Port 3 has power enabled
    unsigned char PE4_Power_Enable_Port_4_Status             : 1;   ///< Port 4 has power enabled
    unsigned char PG1_Power_Good_Port_1_Status               : 1;   ///< Port 1 is powered on with good voltage levels
    unsigned char PG2_Power_Good_Port_2_Status               : 1;   ///< Port 2 is powered on with good voltage levels
    unsigned char PG3_Power_Good_Port_3_Status               : 1;   ///< Port 3 is powered on with good voltage levels
    unsigned char PG4_Power_Good_Port_4_Status               : 1;   ///< Port 4 is powered on with good voltage levels
}TPS238x_Power_Stauts_Register_t;

#define PE1_STATUS                             0x01   ///< Port 1 has power enabled
#define PE2_STATUS                             0x02   ///< Port 2 has power enabled
#define PE3_STATUS                             0x04   ///< Port 3 has power enabled
#define PE4_STATUS                             0x08   ///< Port 4 has power enabled
#define PG1_STATUS                             0x10   ///< Port 1 is powered on with good voltage levels
#define PG2_STATUS                             0x20   ///< Port 2 is powered on with good voltage levels
#define PG3_STATUS                             0x40   ///< Port 3 is powered on with good voltage levels
#define PG4_STATUS                             0x80   ///< Port 4 is powered on with good voltage levels
#define POWER_ENABLE_STATUS                   (PE1_STATUS + PE2_STATUS + PE3_STATUS + PE4_STATUS)
#define POWER_GOOD_STATUS                     (PG1_STATUS + PG2_STATUS + PG3_STATUS + PG4_STATUS)
#define POWER_GOOD_SHIFT                       4
#define GET_POWER_ENABLE_STATUS(x)            (x & POWER_ENABLE_STATUS)
#define GET_POWER_GOOD_STATUS(x)              (x >> POWER_GOOD_SHIFT)

/// I2C Slave Address Register - Software configurable using tpc_SetI2CAddresses() <br>
///                   Hardware - Command 11h; 1 data byte; read only               <br>
///                            - writable during I2C slave address programming protocol <br><br>
/// A chain of TPS23861 devices can be configured by the processor to provide each device with a unique I2C address
typedef struct {
    unsigned char I2C_slave_address                          : 7;    ///< 7 bit I2C address
    unsigned char Auto                                       : 1;    ///< The part will default into auto mode after power on if this bit is set
}TPS238x_I2C_Slave_Address_Register_t;


#define AUTO_BIT                               0x80    ///< The part will default into auto mode after power on if this bit is set


#define I2C_ADDRESS_MASK                       0x7F    ///< 7 bit I2C address

#define TPS238X_ALERT_RESPONSE_ADDRESS         0x0C


/// Operating Mode Register - Uses TPS238x_Operating_Modes_t; Configured in the tps_ConfigPort() function  <br>
///                Hardware - Command 12h; 1 data byte; read/write                                         <br><br>
/// Configures the operating mode for each port (off, manual, semi-auto, or auto)
typedef struct {
    unsigned char Port_1_Operating_Mode                      : 2;    ///< Operating mode for port 1
    unsigned char Port_2_Operating_Mode                      : 2;    ///< Operating mode for port 2
    unsigned char Port_3_Operating_Mode                      : 2;    ///< Operating mode for port 3
    unsigned char Port_4_Operating_Mode                      : 2;    ///< Operating mode for port 4
}TPS238x_Operating_Mode_Register_t;

/// Operating mode for each of the ports (off, manual, semi-auto, auto)
typedef enum {
  OPERATING_MODE_OFF                         = 0x0,   ///< Off; no detection or classifications
  OPERATING_MODE_DIAGNOSTIC                  = 0x1,   ///<Diagnostic
  OPERATING_MODE_SEMI_AUTO                   = 0x2,   ///< Semi-auto, automatic detection and classification (if enabled), but no automatic power on
  OPERATING_MODE_AUTO                        = 0x3    ///< Auto, automatic detection,. classification, anb power on

} TPS238x_Operating_Modes_t;

#define OPERATING_MODE_MASK                    0x3

#define OPERATING_PORT_1_MODE                  0x03
#define OPERATING_PORT_2_MODE                  0x0C
#define OPERATING_PORT_3_MODE                  0x30
#define OPERATING_PORT_4_MODE                  0xC0

/// Disconnect Enable Register - Software API uses TPS238x_Ports_t. Configured in tps_SetDisconnectEnable() and tps_SetPortDisconnectEnable()   <br>
///                   Hardware - Command 13h; 1 data byte; read/write <br><br>
/// Enable the disconnect detection mechanism for the indicated ports.
typedef struct {
    unsigned char DCDE1_Disconnect_Enable_Channel_1             : 1;   ///< DC disconnect enable for 2 pair operation in port 1
    unsigned char DCDE2_Disconnect_Enable_Channel_2             : 1;   ///< DC disconnect enable for 2 pair operation in port 2
    unsigned char DCDE3_Disconnect_Enable_Channel_3             : 1;   ///< DC disconnect enable for 2 pair operation in port 3
    unsigned char DCDE4_Disconnect_Enable_Channel_4             : 1;   ///< DC disconnect enable for 2 pair operation in port 4
    unsigned char Reserved_1                                    : 4;
} TPS238x_Disconnect_Enable_Register_t;

#define DCDE1                                  0x01     ///< DC disconnect enable for 2 pair operation in port 1
#define DCDE2                                  0x02     ///< DC disconnect enable for 2 pair operation in port 2
#define DCDE3                                  0x04     ///< DC disconnect enable for 2 pair operation in port 3
#define DCDE4                                  0x08     ///< DC disconnect enable for 2 pair operation in port 4

/// Detect/Class Enable Register - Software API uses TPS238x_Ports_t;  Configured in tps_SetDetectClassEnable() and tps_SetPortDetectClassEnable()   <br>
///                     Hardware - Command 14h; 1 data byte; read/write  <br><br>
///  Enables detection and classification for each port. Behave differently in Manual, Semi-auto and auto modes. <br>
///  In manual mode, setting a bit means one cycle of detection or classification is performed for the indicated port. The bit
///   is cleared automatically when the cycle is complete. This is similar to the restart commands<br>
/// In semi-auto mode, detection and classifications are performed continuously as long as the port is not powered up. Classifications will
///  follow valid detections. <br>
/// In auto mode, classifications will follow valid detections, and power up will follow valid classifications. <br>
/// @note During cool down time following a fault, execution of this command will be delayed till the end of the cool down period.
typedef struct {
    unsigned char DETE1_Detection_Enable_Channel_1              : 1;     ///< Enable detections for port 1
    unsigned char DETE2_Detection_Enable_Channel_2              : 1;     ///< Enable detections for port 2
    unsigned char DETE3_Detection_Enable_Channel_3              : 1;     ///< Enable detections for port 3
    unsigned char DETE4_Detection_Enable_Channel_4              : 1;     ///< Enable detections for port 4
    unsigned char CLE1_Classification_Enable_Channel_1          : 1;     ///< Enable classifications for port 1
    unsigned char CLE2_Classification_Enable_Channel_2          : 1;     ///< Enable classifications for port 2
    unsigned char CLE3_Classification_Enable_Channel_3          : 1;     ///< Enable classifications for port 3
    unsigned char CLE4_Classification_Enable_Channel_4          : 1;     ///< Enable classifications for port 4
} TPS238x_Detect_Classification_Enable_Register_t;

#define DETE1                                  0x01     ///< Enable detections for port 1
#define DETE2                                  0x02     ///< Enable detections for port 2
#define DETE3                                  0x04     ///< Enable detections for port 3
#define DETE4                                  0x08     ///< Enable detections for port 4
#define CLE1                                   0x10     ///< Enable classifications for port 1
#define CLE2                                   0x20     ///< Enable classifications for port 2
#define CLE3                                   0x40     ///< Enable classifications for port 3
#define CLE4                                   0x80     ///< Enable classifications for port 4

/// Port Power Priority Register - Software API uses TPS238x_Ports_t; Configured in tps_FastShutdownEnable() and tps_FastPortShutdownEnable   <br>
///                     Hardware - Command 15h; 1 data byte; read/write  <br><br>
/// Port power priority bits used for fast shutdown
typedef struct {

    unsigned char DCUT1_ICUT_Disable_Channel_1                  : 1;  ///< Enable fast shutdown port 1
    unsigned char DCUT2_ICUT_Disable_Channel_2                  : 1;
    unsigned char DCUT3_ICUT_Disable_Channel_3                  : 1;
    unsigned char DCUT4_ICUT_Disable_Channel_4                  : 1;
    unsigned char ONE_BIT_OSS_Enable_Channel_1                  : 1;   ///< Enable fast shutdown port 1
    unsigned char ONE_BIT_OSS_Enable_Channel_2                  : 1;   ///< Enable fast shutdown port 2
    unsigned char ONE_BIT_OSS_Enable_Channel_3                  : 1;   ///< Enable fast shutdown port 3
    unsigned char ONE_BIT_OSS_Enable_Channel_4                  : 1;   ///< Enable fast shutdown port 4
} TPS238x_OSS_PCUTDisable_t;

#define OSS1                                   0x10
#define OSS2                                   0x20
#define OSS3                                   0x40
#define OSS4                                   0x80

#define DCUT1                                  0x01
#define DCUT2                                  0x02
#define DCUT3                                  0x04
#define DCUT4                                  0x08


#define OSS_SHIFT                              4

/// Timing Configuration Register - Software configured in tps_SetTiming() <br>
///                      Hardware - Command 16h; 1 data byte; read/write   <br><br>
/// Set the timing configurations used by all four ports
typedef struct {
    unsigned char TMPDO_Disconnect_Delay                     : 2;    ///< Disconnect delay, which is the time to turn off a port once there is a disconnect condition
    unsigned char TICUT_ICUT_Fault_Timing                    : 2;    ///< ICUT Fault Timing period, which is the overcurrent time duration before port turn off
    unsigned char TSTART_Start_Fault_Timing                  : 2;    ///< TSTART period, which is the maximum allowed overcurrent time during inrush
    unsigned char TLIM_ILIM_Fault_Timing                     : 2;    ///< ILIM fault timing, which is the foldback current time limit duration before port turn off
} TPS238x_Timing_Configuration_Register_t;

/// Set the ILIM fault timing, which is the foldback current time limit duration before port turn off. Used in tps_SetTiming()
typedef enum {
  TLIM_60_MS                                 = 0x0,      ///< 60 ms
  TLIM_15_MS                                 = 0x1,      ///<15ms
  TLIM_10_MS                                 = 0x2,      ///<10ms
  TLIM_6_MS                                  = 0x3       ///< 6 ms
} TPS238x_ILIM_Timing_t;

/// Set the length of the TSTART period, which is the maximum allowed overcurrent time during inrush. Used in tps_SetTiming()
typedef enum {
  TSTART_60_MS                               = 0x0,      ///< 60 ms
  TSTART_30_MS                               = 0x1,      ///< 30 ms
  TSTART_120_MS                              = 0x2       ///< 120 ms
} TPS238x_TStart_Timing_t;

/// Set the length of the ICUT Fault Timing period, which is the overcurrent time duration before port turn off. Used in tps_SetTiming()
typedef enum {
  TICUT_60_MS                                = 0x0,      ///< 60 ms
  TICUT_30_MS                                = 0x1,      ///< 30 ms
  TICUT_120_MS                               = 0x2,      ///< 120 ms
  TICUT_240_MS                               = 0x3       ///< 240 ms
} TPS238x_TICUT_Timing_t;

/// Set the length of the Disconnect delay, which is the time to turn off a port once there is a disconnect condition. Used in tps_SetTiming()
typedef enum {
  TDIS_360_MS                                = 0x0,      ///< 360 ms
  TDIS_90_MS                                 = 0x1,      ///< 90 ms
  TDIS_180_MS                                = 0x2,      ///< 180 ms
  TDIS_720_MS                                = 0x3       ///< 720 ms
} TPS238x_TDIS_Timing_t;


#define TDIS_MASK                              0x3
#define TDIS_SHIFT                             0x0

#define TICUT_MASK                             0xC
#define TICUT_SHIFT                            0x2

#define TSTART_MASK                            0x30
#define TSTART_SHIFT                           0x4

#define TLIM_MASK                              0xC0
#define TLIM_SHIFT                             0x6

/// General Mask 1 Register                                         <br>
///                Hardware - Command 17h; 1 data byte; read/write  <br><br>
///
typedef struct {


    unsigned char Reserved_2                                   : 2;
    unsigned char Detec_Change_Enable                          : 1;     ///< 1 = DETCn bit is set only when a change in detection occurred for the associated port
    unsigned char Class_Change_Enable                          : 1;     ///< 1 = CLSCn bit is set only when a change of class occurred for the associated port.
    unsigned char Multi_Bit_Priority                           : 1;     ///< 1 = 3-bit shutdown priority; 0 = 1-bit shutdown priority
    unsigned char Reserved_3                                   : 1;
    unsigned char Reserved_4                                   : 1;
    unsigned char INTEN_INT_Pin_Mask                           : 1;     ///< 1 = Interrupts will generate \\INT pin output
} TPS238x_General_Mask_1_Register_t;


#define DECHE                                  0x04
#define CLCHE                                  0x08
#define _3_BIT_OSS_PRIORITY                    0x10
#define IRFBEN                                 0x40
#define INTEN                                  0x80      ///< External INT/ enabled by any unmasked bit of interrupt register

/// Detect/Class Restart Register - Software API uses TPS238x_Ports_t; Configured in tps_RestartDetection(), tps_RestartPortDetection(),
///                                                 tps_RestartClassification(), tps_RestartPortClassification(), and tps_RestartDetectClass()
///                      Hardware - Command 18h; 1 data byte; write only
/// In diagnostic mode, the indicated ports will restart detection/classification. In auto mode, action is dependant on the ports operating mode
/// @note During a cool down period following a fault condition, this command will be accepted, but deferrred until the cool down timer expires.

typedef struct {
    unsigned char RDET1_Restart_Detection_Channel_1             : 1;    ///< Restart Detection Channel 1
    unsigned char RDET2_Restart_Detection_Channel_2             : 1;    ///< Restart Detection Channel 2
    unsigned char RDET3_Restart_Detection_Channel_3             : 1;    ///< Restart Detection Channel 3
    unsigned char RDET4_Restart_Detection_Channel_4             : 1;    ///< Restart Detection Channel 4
    unsigned char RCL1_Restart_Classification_Channel_1         : 1;    ///< Restart Classification Channel 1
    unsigned char RCL2_Restart_Classification_Channel_2         : 1;    ///< Restart Classification Channel 2
    unsigned char RCL3_Restart_Classification_Channel_3         : 1;    ///< Restart Classification Channel 3
    unsigned char RCL4_Restart_Classification_Channel_4         : 1;    ///< Restart Classification Channel 4
} TPS238x_Detect_Class_Restart_Register_t;

#define RDET1                                  0x01    ///< Restart Detection Channel 1
#define RDET2                                  0x02    ///< Restart Detection Channel 2
#define RDET3                                  0x04    ///< Restart Detection Channel 3
#define RDET4                                  0x08    ///< Restart Detection Channel 4
#define RCL1                                   0x10    ///< Restart Classification Channel 1
#define RCL2                                   0x20    ///< Restart Classification Channel 2
#define RCL3                                   0x40    ///< Restart Classification Channel 3
#define RCL4                                   0x80    ///< Restart Classification Channel 4

#define RESTART_DETECTION_SHIFT                0
#define RESTART_DETECTION_MASK                 0x0F

#define RESTART_CLASSIFCATION_SHIFT            4
#define RESTART_CLASSIFCATION_MASK             0xF0

/// Power Enable Register - Software API uses TPS238x_Ports_t; Configured in tps_SetPortPower, tps_SetPowerOn(), and tps_SetPowerOff() <br>
///              Hardware - Command 19h; 1 data byte; write only    <br><br>
/// Directs the TPS23861 to power a port on or off.
/// @note Setting PWON and PWOFF for the same register turns the port OFF

typedef struct {
    unsigned char PWON1_Power_On_Channel_1                      : 1;     ///< Power on Channel 1
    unsigned char PWON2_Power_On_Channel_2                      : 1;     ///< Power on Channel 2
    unsigned char PWON3_Power_On_Channel_3                      : 1;     ///< Power on Channel 3
    unsigned char PWON4_Power_On_Channel_4                      : 1;     ///< Power on Channel 4
    unsigned char PWOFF1_Power_On_Channel_1                     : 1;     ///< Power off Channel 1
    unsigned char PWOFF2_Power_On_Channel_2                     : 1;     ///< Power off Channel 2
    unsigned char PWOFF3_Power_On_Channel_3                     : 1;     ///< Power off Channel 3
    unsigned char PWOFF4_Power_On_Channel_4                     : 1;     ///< Power off Channel 4
} TPS238x_Power_Enable_Register_t;

#define PWON1                                  0x01     ///< Power on Channel 1
#define PWON2                                  0x02     ///< Power on Channel 2
#define PWON3                                  0x04     ///< Power on Channel 3
#define PWON4                                  0x08     ///< Power on Channel 4
#define PWOFF1                                 0x10     ///< Power off Channel 1
#define PWOFF2                                 0x20     ///< Power off Channel 2
#define PWOFF3                                 0x40     ///< Power off Channel 3
#define PWOFF4                                 0x80     ///< Power off Channel 4

#define POWER_OFF_SHIFT                        4
#define POWER_OFF_MASK                         0xF0

#define POWER_ON_SHIFT                         0
#define POWER_ON_MASK                          0x0F

/// Reset Register - Software API uses TPS238x_Ports_t in tps_ResetPorts() and tps_ResetPort()<br>
///       Hardware - Command 1Ah; 1 data byte; write only <br><br>
/// Forces reset events to occur.
typedef struct {
    unsigned char RESP1_Reset_Channel_1                         : 1;    ///< Reset Channel 1
    unsigned char RESP1_Reset_Channel_2                         : 1;    ///< Reset Channel 2
    unsigned char RESP1_Reset_Channel_3                         : 1;    ///< Reset Channel 3
    unsigned char RESP1_Reset_Channel_4                         : 1;    ///< Reset Channel 4
    unsigned char RESAL_Reset_Registers                         : 1;    ///< Reset register bits; Equivalent to power up reset
    unsigned char Reserved_5                                    : 1;
    unsigned char CLINP_Clear_Interrupt_Pin                     : 1;     ///< Note this does not affect any of the interrupt status bits
    unsigned char CLRAIN_Clear_Interrupt_Bits                   : 1;     ///< Note this also releases the Interrupt Pin
} TPS238x_Reset_Register_t;

#define RESP1                                  0x01
#define RESP2                                  0x02
#define RESP3                                  0x04
#define RESP4                                  0x08
#define RESAL                                  0x10
#define CLINP                                  0x40
#define CLRAIN                                 0x80

/// ID Register                                          <br>
///    Hardware - Command 1Bh; 1 data byte; read/write   <br>
///
typedef struct {
    unsigned char ICV_IC_Version_Number                      : 3;     ///< IC Version number
    unsigned char MFR_ID_Manufacture_ID_Number               : 5;     ///< Manufacture Identification Number
} TPS238x_ID_Register_t;

#define ICV_MASK                                    0x07
#define MFR_ID_MASK                                 0xF8
#define MFR_ID_SHIFT                           3
#define MFR_ID_DEFAULT                         0x0A  // This is the the results in the MFR_ID section of the register
#define ICV_DEFAULT                            0x05
#define ID_REGISTER_DEFAULT                    ((MFR_ID_DEFAULT<<MFR_ID_SHIFT) | ICV_DEFAULT)


typedef struct {
    unsigned char CC12_1_Connection_Check_Status_Channel_1                         : 1;    ///< Channel 1 connection check
    unsigned char CC12_2_Connection_Check_Status_Channel_2                         : 1;    ///< Channel 2 connection check
    unsigned char CC34_3_Connection_Check_Status_Channel_3                         : 1;    ///< Channel 3 connection check
    unsigned char CC34_4_Connection_Check_Status_Channel_4                         : 1;    ///< Channel 4 connection check
    unsigned char AC1_Autoclass_Status_Channel_1                                   : 1;    ///< Channel 1 autoclass results
    unsigned char AC2_Autoclass_Status_Channel_2                                   : 1;    ///< Channel 2 autoclass results
    unsigned char AC3_Autoclass_Status_Channel_3                                   : 1;    ///< Channel 3 autoclass results
    unsigned char AC4_Autoclass_Status_Channel_4                                   : 1;    ///< Channel 4 autoclass results
} TPS238x_Connect_Check_Auto_Class_Register_t;

#define CC12                                 0x03     ///< Channel 1&2 connection check
#define CC34                                 0x0C     ///< Channel 3&4 connection check
#define AC1                                    0x10     ///< Channel 1 autoclass
#define AC2                                    0x20     ///< Channel 2 autoclass
#define AC3                                    0x40     ///< Channel 3 autoclass
#define AC4                                    0x80     ///< Channel 4 autoclass

#define AUTOCLASS_SHIFT                        4
#define AUTOCLASS_MASK                         0xF0

#define CONNECTIONCHECK_SHIFT                       0
#define CONNECTIONCHECK_MASK                        0x0F

typedef enum {
  CC_UNKNOWN                                 = 0x0,      ///< Unknown or incompatible
  VALID_4P_SINGLE                            = 0x1,      ///< Valid 4P single signature
  VALID_4P_DUAL                              = 0x2,      ///< Valid 4P dual signature
  INVALID_CC_DECTECT                         = 0x3       ///< invalid CC
}TPS238x_Connection_Check_Status_t;


typedef struct {
    unsigned char  _2P_Policing                        : 8;    ///< 2P policing configuration
} TPS238x_2P_Policing_config_Register_t;


typedef struct {
    unsigned char CAPDET1_Capacitance_Detect_Channel_1                         : 2;    ///< Channel 1 connection check
    unsigned char CAPDET2_Capacitance_Detect_Channel_2                         : 2;    ///< Channel 2 connection check
    unsigned char CAPDET3_Capacitance_Detect_Channel_3                         : 2;    ///< Channel 3 connection check
    unsigned char CAPDET3_Capacitance_Detect_Channel_4                         : 2;    ///< Channel 4 connection check
} TPS238x_Capacitance_Detect_Register_t;

typedef enum {
  CAP_DETECT_DISABLED                        = 0x0,      ///< Diabled
  _1_10UF_CAP_MEASUREMENT_ENABLED            = 0x1,      ///< 1-10uF enabled
}TPS238x_Legacy_Detect_t;

#define LEGACY_DETECT_MASK                          0x03

///Power-on Fault Register  -Software configured in???   <br>
///               Hardware - Command 24h/25h; 1 data byte, read only  <br><br>
///Allow user to get ports' power on fault status
typedef struct {
    unsigned char PowerOn_Fault_Channel_1                       : 2;  ///< Port 1 power on fault status
    unsigned char PowerOn_Fault_Channel_2                       : 2;  ///< Port 2 power on fault status
    unsigned char PowerOn_Fault_Channel_3                       : 2;  ///< Port 3 power on fault status
    unsigned char PowerOn_Fault_Channel_4                       : 2;  ///< Port 4 power on fault status
}TPS238x_Power_On_Fault_Register_t;

typedef enum {
    NO_FAULT                                   = 0x00,
    INVALID_DETECTION                          = 0x01,
    CLASSIFICATION_ERROR                       = 0x02,
    INSUFFICIENT_POWER_ALLOCATION              = 0x03
}TPS238x_Power_On_Fault_t;

#define PF1                                    = 0x03;
#define PF2                                    = 0x0C;
#define PF3                                    = 0x30;
#define PF4                                    = 0xC0;


///Re-mapping Register -Software configured in ???  <br>
///           Hardware -Command 26h; 1 data byte, read/write <br><br>
/// Allow user to re-map the ports
typedef struct {
    unsigned char Physical_Remap_Logical_Port_1             : 2;  ///< Physical remap logical port 1
    unsigned char Physical_Remap_Logical_Port_2             : 2;  ///< Physical remap logical port 2
    unsigned char Physical_Remap_Logical_Port_3             : 2;  ///< Physical remap logical port 3
    unsigned char Physical_Remap_Logical_Port_4             : 2;  ///< Physical remap logical port 4
}TPS238x_Remapping_Register_t;

typedef enum {
    PHYSICAL_PORT_1                            = 0x00,
    PHYSICAL_PORT_2                            = 0x01,
    PHYSICAL_PORT_3                            = 0x02,
    PHYSICAL_PORT_4                            = 0x03
}TPS238x_Remapping_t;


///Multi-bit Power Priority21 Register - Software configured in ???  <br>
///                         Hardware -Command 27h 1 data byte, read/write <br><br>
/// Allow user to configure 3 bit port's priority
typedef struct {
    unsigned char Multi_Bit_Power_Priority_Channel_1                    : 3;
    unsigned char Reserved_6                                            : 1;
    unsigned char Multi_Bit_Power_Priority_Channel_2                    : 3;
    unsigned char Reserved_7                                            : 1;
}TPS238x_Multi_Bit_Power_Priority21_Configure_Register_t;

///Multi-bit Power Priority43 Register - Software configured in ???  <br>
///                         Hardware -Command 27h 1 data byte, read/write <br><br>
/// Allow user to configure 3 bit port's priority
typedef struct {
    unsigned char Multi_Bit_Power_Priority_Channel_3                    : 3;
    unsigned char Reserved_8                                            : 1;
    unsigned char Multi_Bit_Power_Priority_Channel_4                    : 3;
    unsigned char Reserved_9                                            : 1;
}TPS238x_Multi_Bit_Power_Priority43_Configure_Register_t;


typedef enum {
    PRIORITY_1                                  = 0x00,
    PRIORITY_2                                  = 0x01,
    PRIORITY_3                                  = 0x02,
    PRIORITY_4                                  = 0x03,
    PRIORITY_5                                  = 0x04,
    PRIORITY_6                                  = 0x05,
    PRIORITY_7                                  = 0x07

}TPS238x_Multi_Bit_Power_Priority_t;

#define MULTI_BIT_PRIORITY_MASK                           0x07
#define MULTI_BIT_PRIORITY_SHIFT                          0x04

typedef struct {
    unsigned char MC12_Power_Allocation                     : 3;  ///< Power allocation for channel 1&2
    unsigned char _4P12_Enabled                             : 1;  ///< 4 pair wired for channel 1&2
    unsigned char MC34_Power_Allocation                     : 3;  ///< Power allocation for channel 3&4
    unsigned char _4P34_Enabled                             : 1 ;  ///< 4 pair wired for channel 3&4
}TPS238x_4P_Power_Allocation_Register_t;

typedef enum {
    _4P_DISABLE                                 = 0x0,
    _4P_ENABLE                                  = 0x1,
}TPS238x_4P_Enable_Disable_t;

typedef enum {
    _4P_15W                                  = 0x00,
    _4P_30W                                  = 0x03,
    _4P_45W                                  = 0x04,
    _4P_60W                                  = 0x05,
    _4P_75W                                  = 0x06,
    _4P_90W                                  = 0x07
}TPS238x_4P_Power_Allocation_t;


typedef enum {
    _2P_15W_15W                              = 0x00,
    _2P_4W_4W                                = 0x01,
    _2P_7W_7W                                = 0x02,
    _2P_30W_30W                              = 0x03,
    _2P_7W_15W                               = 0x04,
    _2P_15W_7W                               = 0x05,
    _2P_15W_30W                              = 0x06,
    _2P_30W_15W                              = 0x07
}TPS238x_2P_Power_Allocation_t;


/// Temperature Register - Software gets this value using tps_GetTemperature() <br>
///             Hardware - Command 2Ch; 1 data byte; read only                 <br><br>
/// Die temperature
typedef struct {
    unsigned char Temp_Value                                : 8;
}TPS238x_Temperature_Register_t;

#define CONVERT_TEMP(x)                        (((x*652)-20000)/1000)       ///< Macro to convert result from tps_GetTemperature() into degrees C (float)


typedef struct {
    unsigned char DCDT12_DC_Disconnect_Control_12           : 1;  ///< DC disconnect threshold control
    unsigned char DCDT34_DC_Disconnect_Control_34           : 1;  ///< DC disconnect threshold control
    unsigned char _4PPCT12_4P_PCUT_ENABLE_12                : 1;  ///< 4 Pair PCUT enable
    unsigned char _4PPCT34_4P_PCUT_ENABLE_34                : 1;  ///< 4 Pair PCUT enable
    unsigned char NCT12_4P_PCUT_FAULT_MANAGE_12             : 1;  ///< 4 Pair PCUT fault management
    unsigned char NCT34_4P_PCUT_FAULT_MANAGE_34             : 1;  ///< 4 Pair PCUT fault management
    unsigned char NLM12_4P_ILIM_FAULT_MANAGE_12             : 1;  ///< 4 Pair ILIM fault management
    unsigned char NLM34_4P_ILIM_FAULT_MANAGE_34             : 1;  ///< 4 Pair ILIM fault management
}TPS238x_Dis_PCUT_ILIM_Register_t;

/// Current levels for disconnect threshold. Used in tps_ConfigPort()
typedef enum {
  DCDT_6_5_MILLIAMP                          = 0x0,      ///< 6.5 mA
  DCDT_4_5_MILLIAMP                          = 0x1      ///< 4.5 mA
} TPS238x_Disconnect_Threshold_t;



/// Input Voltage Register - Software gets this value using tps_GetInputVoltage()     <br>
///               Hardware - Command 2Eh; 2 data byte (LSB followed by MSB); read only   <br><br>
/// The system will measure the input voltage around 1/sec. The returned value has an LSB of 3.662mV

typedef union {
    struct Input_Voltage_Short_t {
        unsigned short Input_Voltage                         : 14;       ///< Voltage with 3.662 mV lsb
        unsigned short Reserved_10                            :  2;
    } Input_Voltage_Short;

    struct Input_Voltage_Char_t {
        unsigned char  Input_Voltage_LSB                     : 8;
        unsigned char  Input_Voltage_MSB                     : 6;
        unsigned char  Reserved_11                           : 2;
    } Input_Voltage_Char;
} TPS238x_Input_Voltage_Register_u;

typedef unsigned short TPS238x_Input_Voltage_t;

#define TPS2368X_INPUT_VOLTAGE_MASK_SHORT                    0x3FFF

/// Port 1 Current Register - Software gets this value using tps_GetPortMeasurements()  <br>
///                Hardware - Command 30h; 2 data byte (LSB followed by MSB); read only <br>
/// Port 2 Current Register                                                             <br>
///                Hardware - Command 34h; 2 data byte (LSB followed by MSB); read only <br>
/// Port 3 Current Register                                                             <br>
///                Hardware - Command 38h; 2 data byte (LSB followed by MSB); read only <br>
/// Port 4 Current Register                                                             <br>
///                Hardware - Command 3Ch; 2 data byte (LSB followed by MSB); read only <br><br>
/// 14 bit data conversion result of the current for the port.  The LSB is 63.360uA

typedef union {
    struct Port_Current_Short_t {
        unsigned short Port_Current                          : 14;    ///< Port current with lsb of 63.360 micro-Amps
        unsigned short Reserved_12                           :  2;
    } Port_Current_Short;

    struct Port_Current_Char_t {
        unsigned char  Port_Current_LSB                      : 8;
        unsigned char  Port_Current_MSB                      : 6;
        unsigned char  Reserved_13                           : 2;
    } Port_Current_Char;
} TPS238x_Port_Current_Register_u;

typedef unsigned short TPS238x_Port_Current_t;

#define TPS238X_CHANNEL_CURRENT_MASK_SHORT                     0x3FFF

/// Port 1 Voltage Register - Software gets this value using tps_GetPortMeasurements()  <br>
///                Hardware - Command 32h; 2 data byte (LSB followed by MSB); read only <br>
/// Port 2 Voltage Register                                                             <br>
///                Hardware - Command 36h; 2 data byte (LSB followed by MSB); read only <br>
/// Port 3 Voltage Register                                                             <br>
///                Hardware - Command 3Ah; 2 data byte (LSB followed by MSB); read only <br>
/// Port 4 Voltage Register                                                             <br>
///                Hardware - Command 3Eh; 2 data byte (LSB followed by MSB); read only <br><br>
///     14 bit data conversion result of the voltage for the port. The LSB is 3.662mV
typedef union {
    struct Port_Voltage_Short_t {
        unsigned short Port_Voltage                          : 14;       ///< Voltage of port with lsb of 3.662 milli-Volts
        unsigned short Reserved_14                           :  2;
    } Port_Voltage_Short;

    struct Port_Voltage_Char_t {
        unsigned char  Port_Voltage_LSB                      : 8;
        unsigned char  Port_Voltage_MSB                      : 6;
        unsigned char  Reserved_15                           : 2;
    } Port_Voltage_Char;
} TPS238x_Port_Voltage_Register_u;

typedef unsigned short TPS238x_Port_Voltage_t;

#define TPS238X_CHANNEL_VOLTAGE_MASK_SHORT                     0x3FFF



/// PoE Plus Register - Software configures this in tps_ConfigPort() <br>
///          Hardware - Command 40h; 1 data byte; read/write <br><br>
///

typedef struct {
    unsigned char Reserved_16                                   : 4;
    unsigned char POEP1_Foldback_Curve_Channel_1                : 1;
    unsigned char POEP1_Foldback_Curve_Channel_2                : 1;
    unsigned char POEP1_Foldback_Curve_Channel_3                : 1;
    unsigned char POEP1_Foldback_Curve_Channel_4                : 1;
} TPS238x_Foldback_Selection_Register_t;

/// Foldback curve applied to a port when powered on. Used in tps_ConfigPort()
typedef enum {
  _1X_ILIM_FOLDBACK_CURVE                    = 0x0,       ///< 1 x Ilim foldback curve applied when port is powered on
  _2X_ILIM_FOLDBACK_CURVE                    = 0x1        ///< 2 x Ilim foldback curve applied when port is powered on
} TPS238x_Foldback_t;

#define POEP1                                  0x10
#define POEP2                                  0x20
#define POEP3                                  0x40
#define POEP4                                  0x80

#define POE_PLUS_SHIFT                         4


/// Firmware revision Register                                          <br>
///              Hardware - Command 41h; 1 data byte; read only    <br><br>
/// Get the Firmware Revision number.
/// After a RESET or POR fault this value will default to 0000, 0000b, but upon a “valid” SRAM load,
/// this value will reflect the corresponding SRAM version of firmware (0x01h – 0xFEh).

typedef struct {
    unsigned char FIRMWARE_REV                               : 8;
}TPS238x_Frimware_Revision_Register_t;



/// I2C Watchdog Register                                          <br>
///              Hardware - Command 42h; 1 data byte; read/write   <br><br>
/// Monitors the I2C clock line in order to detect hung I2C communcations
typedef struct {
    unsigned char WDS_Watchdog_Status                        : 1;     ///< Watchdog timer has expired
    unsigned char IWD_I2C_Watchdog_Disable                   : 4;     ///< Set to IWD_MASK_VALUE to disable I2C watchdog (default)
    unsigned char Reserved_17                                : 3;
} TPS238x_I2C_Watchdog_Register_t;

#define WDS                                    0x01
#define IWD                                    0x1E

#define IWD_SHIFT                              1
#define IWD_MASK_VALUE                         0xB            ///< Value to place in the IWD bits to disable the I2C watchdog (default setting)

/// Device ID Register                                          <br>
///              Hardware - Command 43h; 1 data byte; read only    <br><br>
/// Get the Firmware Revision number.
/// After a RESET or POR fault this value will default to 0000, 0000b, but upon a “valid” SRAM load,
/// this value will reflect the corresponding SRAM version of firmware (0x01h – 0xFEh).

typedef struct {
    unsigned char SR                                         : 5;
    unsigned char DID                                        : 3;
}TPS238x_Device_ID_Register_t;

#define SILICONREV_MASK                      0x1F
#define DEVICEID_MASK                        0xE0
#define DEVICEID_SHIFT                       5

/// Port 1 Detect Resistance Register - Software gets these values using tps_GetDetectResistance() <br>
///                          Hardware - Command 44h; 1 data byte  read only  <br>
/// Port 2 Detect Resistance Register                                                              <br>
///                          Hardware - Command 45h; 1 data byte  read only  <br>
/// Port 3 Detect Resistance Register                                                              <br>
///                          Hardware - Command 46h; 1 data byte  read only  <br>
/// Port 4 Detect Resistance Register                                                              <br>
///                          Hardware - Command 47h; 1 data byte  read only  <br><br>
typedef struct{
    unsigned char Detect_Resistance                           : 8;
}TPS238x_Detect_Resistance_Register_t;


/// Channel 1 Detect Capacitance Register - Software gets these values using tps_GetDetectCapacitance() <br>
///                          Hardware - Command 48h; 1 data byte  read only  <br>
/// Channel 2 Detect Capacitance Register                                                              <br>
///                          Hardware - Command 49h; 1 data byte  read only  <br>
/// Channel 3 Detect Capacitance Register                                                              <br>
///                          Hardware - Command 4Ah; 1 data byte  read only  <br>
/// Channel 4 Detect Capacitance Register                                                              <br>
///                          Hardware - Command 4Bh; 1 data byte  read only  <br><br>

typedef struct{
    unsigned char Detect_Capacitance                           : 8;
}TPS238x_Detect_Capacitance_Register_t;



/// Assigned Status Register - Results returned in tps_GetPortAssignedClassStatus(), tps_GetPortPreviousClassStatus() <br>
///             Hardware - Command 4Ch; Channel 1; 1 data byte; read only  <br>
///                      - Command 4Dh; Channel 2; 1 data byte; read only  <br>
///                      - Command 4Eh; Channel 3; 1 data byte; read only  <br>
///                      - Command 4Fh; Channel 4; 1 data byte; read only  <br><br>

typedef struct {
    unsigned char PCLASS_Previous_Class           : 4;  ///< DC disconnect threshold control
    unsigned char ACLASS_Assigned_Class           : 4;  ///< DC disconnect threshold control
}TPS238x_Assigned_Status_Register_t;


#define PRECLASS_MASK    0x0F
#define ASSCLASS_SHIFT   0x04
/// Autoclass Control Status Register - Results returned in tps_GetPortAssignedClassStatus(), tps_GetPortPreviousClassStatus() <br>
///             Hardware - Command 50h;  1 data byte; read/write  <br>

typedef struct {
    unsigned char AAC1_Auto_AutoClass_Channel_1          : 1;  ///< Auto autoclass 1
    unsigned char AAC2_Auto_AutoClass_Channel_2          : 1;  ///< Auto autoclass 2
    unsigned char AAC2_Auto_AutoClass_Channel_3          : 1;  ///< Auto autoclass 3
    unsigned char AAC2_Auto_AutoClass_Channel_4          : 1;  ///< Auto autoclass 4
    unsigned char MAC1_Manual_AutoClass_Channel_1        : 1;  ///< Manual autoclass 1
    unsigned char MAC2_Manual_AutoClass_Channel_2        : 1;  ///< Manual autoclass 2
    unsigned char MAC2_Manual_AutoClass_Channel_3        : 1;  ///< Manual autoclass 3
    unsigned char MAC2_Manual_AutoClass_Channel_4        : 1;  ///< Manual autoclass 4
}TPS238x_Autoclass_Control_Register_t;

#define AAC1      0x1
#define AAC2      0x2
#define AAC3      0x4
#define AAC4      0x8
#define MAC1      0x10
#define MAC2      0x20
#define MAC3      0x40
#define MAC4      0x80

#define MAC_SHIFT    4
#define AAC_MASK     0x0F

/// Assigned Status Register - Results returned in tps_GetChannelAutoClassPower() <br>
///             Hardware - Command 51h; Channel 1; 1 data byte; read only  <br>
///                      - Command 52h; Channel 2; 1 data byte; read only  <br>
///                      - Command 53h; Channel 3; 1 data byte; read only  <br>
///                      - Command 54h; Channel 4; 1 data byte; read only  <br><br>

typedef struct {
    unsigned char AC_AutoClass_Power           : 7;  ///< DC disconnect threshold control
    unsigned char AC_FLT_AutoClass_Fault       : 1;  ///< DC disconnect threshold control
}TPS238x_Autoclass_Power_Status_Register_t;

#define AUTOCLASS_POWER_SHIFT       7
#define AUTOCLASS_POWER_MASK        0x7F
#define AUTOCLASS_FAULT_MASK        0x80

/// Alternative Foldback Selection Register - Results returned in tps_SetAlternativeFoldback() <br>
///             Hardware - Command 55h; Channel 1-8; 1 data byte; R/W  <br>

typedef struct {
    unsigned char Alt_Inrush_Channel_1_Enabled   :1; ///< Alternative inrush foldback enabled channel 1
    unsigned char Alt_Inrush_Channel_2_Enabled   :1; ///< Alternative inrush foldback enabled channel 2
    unsigned char Alt_Inrush_Channel_3_Enabled   :1; ///< Alternative inrush foldback enabled channel 3
    unsigned char Alt_Inrush_Channel_4_Enabled   :1; ///< Alternative inrush foldback enabled channel 4
    unsigned char Alt_Ilim_Channel_1_Enabled     :1; ///< Alternative operation foldback enabled channel 1
    unsigned char Alt_Ilim_Channel_2_Enabled     :1; ///< Alternative operation foldback enabled channel 2
    unsigned char Alt_Ilim_Channel_3_Enabled     :1; ///< Alternative operation foldback enabled channel 3
    unsigned char Alt_Ilim_Channel_4_Enabled     :1; ///< Alternative operation foldback enabled channel 4
}TPS238x_Alternative_Foldback_Selection_Register_t;

#define ALT_ILIM_SHIFT       4
#define ALT_INRUSH_MASK      0x0F
#define ALT_ILIM_MASK        0xF0


/// SRAM Control Register - Results returned in tps_SetSRAMControl() <br>
///             Hardware - Command 60h; Channel 1-8; 1 data byte; R/W  <br>

typedef struct {
    unsigned char Clear_Pointer               :1; ///< Clear pointer
    unsigned char SRAM_Parity_Select          :1; ///< SRAM parity
    unsigned char Read_Write_Select           :1; ///< Read/write select
    unsigned char Programmed_SRAM_Indicator   :1; ///< Alternative inrush foldback enabled channel 4
    unsigned char Reserved_18                 :1; ///< Reserved
    unsigned char Reserved_19                 :1; ///< Reserved
    unsigned char CPU_Reset                   :1; ///< CPU rest
    unsigned char I2C_Programming_Select      :1;
}TPS238x_SRAM_Control_Register_t;


/// SRAM Data Entry/Read Register - Results returned in tps_ReadSRAMData();tps_WriteSRAMData() <br>
///             Hardware - Command 61h; Channel 1-8; 1 data byte; R/W  <br>

typedef struct {
    unsigned char SRAM_Data               :8; ///< SRAM data
}TPS238x_SRAM_Data_Register_t;

/// SRAM Data Address - Software gets this value using tps_SetSRAMStartAddress()     <br>
///               Hardware - Command 62/63h; 2 data byte (LSB followed by MSB); R/W   <br><br>


typedef union {
    struct SRAM_Start_Address_Short_t {
        unsigned short Input_Voltage                         : 16;       ///< SRAM start address
    } SRAM_Start_Address_Short;

    struct SRAM_Start_Address_Char_t {
        unsigned char  SRAM_Start_Address_LSB                     : 8;
        unsigned char  SRAM_Start_Address_MSB                     : 8;
    } SRAM_Start_Address_Char;
} TPS238x_SRAM_Start_Address_Register_u;

typedef unsigned short TPS238x_SRAM_Start_Address_t;

#define TPS2368X_SRAM_Start_Address_MASK_SHORT                    0xFFFF











/// A macro that converts a port number into a port value
#define CONVERT_PORT_NUM(x)                    (1 << ((uint8_t)x-1))


/**********************************************************************
                 I2C Commands for the TPS23880
***********************************************************************/

#define TPS238X_INTERRUPT_COMMAND                            0x00
#define TPS238X_INTERRUPT_MASK_COMMAND                       0x01
#define TPS238X_POWER_EVENT_COMMAND                          0x02
#define TPS238X_POWER_EVENT_CLEAR_COMMAND                    0x03
#define TPS238X_DETECTION_EVENT_COMMAND                      0x04
#define TPS238X_DETECTION_EVENT_CLEAR_COMMAND                0x05
#define TPS238X_FAULT_EVENT_COMMAND                          0x06
#define TPS238X_FAULT_EVENT_CLEAR_COMMAND                    0x07
#define TPS238X_START_LIMIT_EVENT_COMMAND                    0x08
#define TPS238X_START_LIMIT_EVENT_CLEAR_COMMAND              0x09
#define TPS238X_SUPPLY_EVENT_COMMAND                         0x0A
#define TPS238X_SUPPLY_EVENT_CLEAR_COMMAND                   0x0B
#define TPS238X_CHANNEL_1_STATUS_COMMAND                     0x0C
#define TPS238X_CHANNEL_2_STATUS_COMMAND                     0x0D
#define TPS238X_CHANNEL_3_STATUS_COMMAND                     0x0E
#define TPS238X_CHANNEL_4_STATUS_COMMAND                     0x0F
#define TPS238X_POWER_STATUS_COMMAND                         0x10
#define TPS238X_I2C_SLAVE_ADDRESS_COMMAND                    0x11
#define TPS238X_OPERATING_MODE_COMMAND                       0x12
#define TPS238X_DISCONNECT_ENABLE_COMMAND                    0x13
#define TPS238X_DETECT_CLASS_ENABLE_COMMAND                  0x14
#define TPS238X_PORT_POWER_PRIORITY_COMMAND                  0x15
#define TPS238X_TIMING_CONFIGURATION_COMMAND                 0x16
#define TPS238X_GENERAL_MASK_COMMAND                         0x17
#define TPS238X_DETECT_CLASS_RESTART_COMMAND                 0x18
#define TPS238X_POWER_ENABLE_COMMAND                         0x19
#define TPS238X_RESET_COMMAND                                0x1A
#define TPS238X_ID_COMMAND                                   0x1B
#define TPS238X_CONNECTIONCHECK_AUTOCLASS_COMMAND            0x1C
#define TPS238X_2P_POLICE_1_CONFIG_COMMAND                   0x1E
#define TPS238X_2P_POLICE_2_CONFIG_COMMAND                   0x1F
#define TPS238X_2P_POLICE_3_CONFIG_COMMAND                   0x20
#define TPS238X_2P_POLICE_4_CONFIG_COMMAND                   0x21
#define TPS238X_LEGACY_DETECTION_COMMAND                     0x22
#define TPS238X_POWER_ON_FAULT_EVENT_COMMAND                 0x24
#define TPS238X_POWER_ON_FAULT_CLEAR_EVENT_COMMAND           0x25
#define TPS238X_CHANNEL_REMAPPING_COMMAND                    0x26
#define TPS238X_MULTI_BIT_PORT_PRIORITY_21_COMMAND           0x27
#define TPS238X_MULTI_BIT_PORT_PRIORITY_43_COMMAND           0x28
#define TPS238X_4PWIRED_POWER_ALLOCATION_CONFIG_COMMAND      0x29
#define TPS238X_4P_POLICE_12_CONFIG_COMMAND                  0x2A
#define TPS238X_4P_POLICE_34_CONFIG_COMMAND                  0x2B
#define TPS238X_TEMPERATURE_COMMAND                          0x2C
#define TPS238X_4P_DISCONNECT_PCUT_ILIM_CONFIG_COMMAND       0x2D
#define TPS238X_INPUT_VOLTAGE_COMMAND                        0x2E
#define TPS238X_INPUT_VOLTAGE_LSB_COMMAND                    0x2E
#define TPS238X_INPUT_VOLTAGE_MSB_COMMAND                    0x2F

#define TPS238X_CHANNEL_1_CURRENT_COMMAND                    0x30
#define TPS238X_CHANNEL_1_CURRENT_LSB_COMMAND                0x30
#define TPS238X_CHANNEL_1_CURRENT_MSB_COMMAND                0x31

#define TPS238X_CHANNEL_1_VOLTAGE_COMMAND                    0x32
#define TPS238X_CHANNEL_1_VOLTAGE_LSB_COMMAND                0x32
#define TPS238X_CHANNEL_1_VOLTAGE_MSB_COMMAND                0x33

#define TPS238X_CHANNEL_2_CURRENT_COMMAND                    0x34
#define TPS238X_CHANNEL_2_CURRENT_LSB_COMMAND                0x34
#define TPS238X_CHANNEL_2_CURRENT_MSB_COMMAND                0x35

#define TPS238X_CHANNEL_2_VOLTAGE_COMMAND                    0x36
#define TPS238X_CHANNEL_2_VOLTAGE_LSB_COMMAND                0x36
#define TPS238X_CHANNEL_2_VOLTAGE_MSB_COMMAND                0x37

#define TPS238X_CHANNEL_3_CURRENT_COMMAND                    0x38
#define TPS238X_CHANNEL_3_CURRENT_LSB_COMMAND                0x38
#define TPS238X_CHANNEL_3_CURRENT_MSB_COMMAND                0x39

#define TPS238X_CHANNEL_3_VOLTAGE_COMMAND                    0x3A
#define TPS238X_CHANNEL_3_VOLTAGE_LSB_COMMAND                0x3A
#define TPS238X_CHANNEL_3_VOLTAGE_MSB_COMMAND                0x3B

#define TPS238X_CHANNEL_4_CURRENT_COMMAND                    0x3C
#define TPS238X_CHANNEL_4_CURRENT_LSB_COMMAND                0x3C
#define TPS238X_CHANNEL_4_CURRENT_MSB_COMMAND                0x3D

#define TPS238X_CHANNEL_4_VOLTAGE_COMMAND                    0x3E
#define TPS238X_CHANNEL_4_VOLTAGE_LSB_COMMAND                0x3E
#define TPS238X_CHANNEL_4_VOLTAGE_MSB_COMMAND                0x3F

#define TPS238X_CHANNEL_FOLDBACK                             0x40
#define TPS238X_FIRMWARE_REVISION_COMMAND                    0x41
#define TPS238X_I2C_WATCHDOG_COMMAND                         0x42
#define TPS238X_DEVICE_ID_COMMAND                            0x43


#define TPS238X_CHANNEL_1_DETECT_RESISTANCE_COMMAND          0x44
#define TPS238X_CHANNEL_2_DETECT_RESISTANCE_COMMAND          0x45
#define TPS238X_CHANNEL_3_DETECT_RESISTANCE_COMMAND          0x46
#define TPS238X_CHANNEL_4_DETECT_RESISTANCE_COMMAND          0x47
#define TPS238X_CHANNEL_1_DETECT_CAPACITANCE_COMMAND         0x48
#define TPS238X_CHANNEL_2_DETECT_CAPACITANCE_COMMAND         0x49
#define TPS238X_CHANNEL_3_DETECT_CAPACITANCE_COMMAND         0x4A
#define TPS238X_CHANNEL_4_DETECT_CAPACITANCE_COMMAND         0x4B

#define TPS238X_CHANNEL_1_ASSIGNED_CLASS_COMMAND             0x4C
#define TPS238X_CHANNEL_2_ASSIGNED_CLASS_COMMAND             0x4D
#define TPS238X_CHANNEL_3_ASSIGNED_CLASS_COMMAND             0x4E
#define TPS238X_CHANNEL_4_ASSIGNED_CLASS_COMMAND             0x4F

#define TPS238X_AUTO_CLASS_CONTROL_COMMAND                   0x50
#define TPS238X_CHANNEL_1_AUTO_CLASS_POWER_COMMAND           0x51
#define TPS238X_CHANNEL_2_AUTO_CLASS_POWER_COMMAND           0x52
#define TPS238X_CHANNEL_3_AUTO_CLASS_POWER_COMMAND           0x53
#define TPS238X_CHANNEL_4_AUTO_CLASS_POWER_COMMAND           0x54

#define TPS238X_ALTERNATIVE_FOLDBACK_ENABLE_COMMAND          0x55

#define TPS238X_SRAM_CONTROL_COMMAND                         0x60
#define TPS238X_SRAM_DATA_COMMAND                            0x61

#define TPS238X_SRAM_START_ADDRESS_COMMAND                   0x62
#define TPS238X_SRAM_START_ADDRESS_LSB_COMMAND               0x62
#define TPS238X_SRAM_START_ADDRESS_MSB_COMMAND               0x63






/*************************************************************************************************************/
/*                              PROTOTYPES                                                                   */
/*************************************************************************************************************/

#ifdef __CPLUSPLUS
extern "C" {
#endif

uint8_t tps_RegisterPort (uint16_t device_i2c_address, TPS238x_PortNum_t devicePortNum);

uint16_t tps_GetDeviceI2CAddress (uint8_t systemPortNum);
TPS238x_PortNum_t tps_GetDevicePortNum (uint8_t systemPortNum);
uint8_t tps_GetSystemPortNumber (uint16_t deviceI2CAddress, TPS238x_PortNum_t devicePortNum);
uint8_t tps_SetDeviceInterruptMask (uint8_t device_i2c_address, TPS238X_Interrupt_Mask_Register_t intMask);
uint8_t tps_GetDeviceInterruptMask (uint8_t device_i2c_address, TPS238X_Interrupt_Mask_Register_t *intMask);
uint8_t tps_GetDeviceInterruptStatus (uint8_t device_i2c_address, TPS238X_Interrupt_Register_t *status);
uint8_t tps_GetDeviceAllInterruptEvents (uint8_t device_i2c_address,
                                         TPS238x_On_Off_t clearEvent,
                                         TPS238x_Ports_t *powerEnablePortEvents,
                                         TPS238x_Ports_t *powerGoodPortEvents,
                                         TPS238x_Ports_t *detectionPortEvents,
                                         TPS238x_Ports_t *classificationPortEvents,
                                         TPS238x_Ports_t *icutPortEvents,
                                         TPS238x_Ports_t *disconnectPortEvents,
                                         TPS238x_Ports_t *inrushPortEvents,
                                         TPS238x_Ports_t *ilimPortEvents,
                                         uint8_t *supplyEvents);
uint8_t tps_GetDevicePowerEnableEvent(uint8_t device_i2c_address,TPS238x_On_Off_t clearEvent,TPS238x_Ports_t *powerEnablePortEvents);
uint8_t tps_GetDevicePowerGoodEvent(uint8_t device_i2c_address,TPS238x_On_Off_t clearEvent,TPS238x_Ports_t *powerGoodPortEvents);
uint8_t tps_GetDeviceDetectionEvent(uint8_t device_i2c_address,TPS238x_On_Off_t clearEvent,TPS238x_Ports_t *detectionPortEvents);
uint8_t tps_GetDeviceClassificationEvent(uint8_t device_i2c_address,TPS238x_On_Off_t clearEvent,TPS238x_Ports_t *classificationPortEvents);
uint8_t tps_GetDevicePCUTFaultEvent(uint8_t device_i2c_address,TPS238x_On_Off_t clearEvent,TPS238x_Ports_t *pcutPortEvents);
uint8_t tps_GetDeviceDisconnectEvent(uint8_t device_i2c_address,TPS238x_On_Off_t clearEvent,TPS238x_Ports_t *disconnectPortEvents);
uint8_t tps_GetDeviceStartEvent(uint8_t device_i2c_address,TPS238x_On_Off_t clearEvent,TPS238x_Ports_t *inrushPortEvents);
uint8_t tps_GetDeviceILIMEvent(uint8_t device_i2c_address,TPS238x_On_Off_t clearEvent,TPS238x_Ports_t *ilimPortEvents);
uint8_t tps_GetDeviceSupplyEvent(uint8_t device_i2c_address,TPS238x_On_Off_t clearEvent,uint8_t *supplyEvents);

uint8_t tps_GetPortDetectRequestedClassStatus (uint8_t systemPortNum, TPS238x_Detection_Status_t *detectionStatus,
                                      TPS238x_Classification_Status_t *classificationStatus);
uint8_t tps_GetPortDetectionStatus (uint8_t systemPortNum, TPS238x_Detection_Status_t *detectionStatus);
uint8_t tps_GetPortRequestedClassificationStatus (uint8_t systemPortNum, TPS238x_Classification_Status_t *classificationStatus);
uint8_t tps_GetDevicePowerStatus (uint8_t device_i2c_address, TPS238x_Ports_t *powerEnablePorts, TPS238x_Ports_t *powerGoodPorts);
uint8_t tps_GetDevicePowerEnableStatus (uint8_t device_i2c_address, TPS238x_Ports_t *powerEnablePorts);
uint8_t tps_GetPortPowerEnableStatus (uint8_t systemPortNum);
uint8_t tps_GetDevicePowerGoodStatus (uint8_t device_i2c_address, TPS238x_Ports_t *powerGoodPorts);
uint8_t tps_GetPortPowerGoodStatus (uint8_t systemPortNum);

uint8_t tps_SetDeviceOperatingMode (uint8_t device_i2c_address, TPS238x_Operating_Modes_t operatingMode1, TPS238x_Operating_Modes_t operatingMode2,
                             TPS238x_Operating_Modes_t operatingMode3, TPS238x_Operating_Modes_t operatingMode4);
uint8_t tps_SetPortOperatingMode (uint8_t systemPortNum, TPS238x_Operating_Modes_t operatingMode);

uint8_t tps_SetDeviceDisconnectEnable (uint8_t device_i2c_address, TPS238x_Ports_t disconnectPorts);
uint8_t tps_GetDeviceDisconnectEnable (uint8_t device_i2c_address, TPS238x_Ports_t *disconnectPorts);
uint8_t tps_SetPortDisconnectEnable (uint8_t systemPortNum, TPS238x_On_Off_t on_off);

uint8_t tps_SetDeviceDetectClassEnable (uint8_t device_i2c_address, TPS238x_Ports_t detectPorts, TPS238x_Ports_t classPorts);
uint8_t tps_GetDeviceDetectClassEnable (uint8_t device_i2c_address, TPS238x_Ports_t *detectPorts, TPS238x_Ports_t *classPorts);
uint8_t tps_SetPortDetectClassEnable (uint8_t systemPortNum, TPS238x_On_Off_t on_off_detect, TPS238x_On_Off_t on_off_class);

uint8_t tps_GetDeviceDetectionEnable (uint8_t device_i2c_address, TPS238x_Ports_t *detectPorts);
uint8_t tps_GetPortDetectionEnable (uint8_t systemPortNum);

uint8_t tps_GetDeviceClassificationEnable (uint8_t device_i2c_address, TPS238x_Ports_t *classPorts);
uint8_t tps_GetPortClassificationEnable (uint8_t systemPortNum);

uint8_t tps_SetDevoiceOneBitOSS(uint8_t device_i2c_address,TPS238x_Ports_t portsOSS);
uint8_t tps_SetDevoicePCUTDisable(uint8_t device_i2c_address,TPS238x_Ports_t portsPCUTDisable);

uint8_t tps_SetDeviceTiming (uint8_t device_i2c_address, TPS238x_ILIM_Timing_t ilimTiming, TPS238x_TStart_Timing_t startTiming,
                             TPS238x_TICUT_Timing_t icutTiming, TPS238x_TDIS_Timing_t disconnectTiming);

uint8_t tps_SetDeviceGeneralMask(uint8_t device_i2c_address, TPS238x_General_Mask_1_Register_t genMask);
uint8_t tps_GetDeviceGeneralMask(uint8_t device_i2c_address, uint8_t *genMask);

uint8_t tps_RestartDeviceDetection (uint8_t device_i2c_address, TPS238x_Ports_t detectPorts);
uint8_t tps_RestartPortDetection (uint8_t systemPortNum);
uint8_t tps_RestartDeviceClassification (uint8_t device_i2c_address, TPS238x_Ports_t classPorts);
uint8_t tps_RestartPortClassification (uint8_t systemPortNum);
uint8_t tps_RestartDeviceDetectClass (uint8_t device_i2c_address, TPS238x_Ports_t detectPorts, TPS238x_Ports_t classPorts);

uint8_t tps_SetDevicePowerOn (uint8_t device_i2c_address, TPS238x_Ports_t portsPoweredOn);
uint8_t tps_SetDevicePowerOff (uint8_t device_i2c_address, TPS238x_Ports_t portsPoweredOff);
uint8_t tps_SetPortPower (uint8_t systemPortNum, TPS238x_On_Off_t on_off);

uint8_t tps_ResetDevicePort (uint8_t systemPortNum);
uint8_t tps_ResetPort (uint8_t systemPortNum);

uint8_t tps_GetID(uint8_t device_i2c_address,uint8_t *ICVersion,uint8_t *MFRID);

uint8_t tps_GetDeviceAutoClassStatus(uint8_t device_i2c_address, TPS238x_Ports_t *autoclassports);
uint8_t tps_GetPortAutoClassStatus(uint8_t systemPortNum, TPS238x_On_Off_t *on_off_autoclass);
uint8_t tps_GetDeviceConnectionCheckStatus(uint8_t device_i2c_address, TPS238x_Ports_t *connectioncheck);
uint8_t tps_GetPortConnectionCheckStatus(uint8_t systemPortNum, uint8_t *connectioncheck);

uint8_t tps_SetPort2PPolicing(uint8_t systemPortNum,TPS238x_2P_Policing_config_Register_t icutCurrentThreshold);
uint8_t tps_GetPort2PPolicing(uint8_t systemPortNum, TPS238x_2P_Policing_config_Register_t *icutCurrentThreshold);

uint8_t tps_SetDeviceLegacyDetection(uint8_t device_i2c_address, TPS238x_Legacy_Detect_t legacydetect1, TPS238x_Legacy_Detect_t legacydetect2,TPS238x_Legacy_Detect_t legacydetect3,TPS238x_Legacy_Detect_t legacydetect4);

uint8_t tps_GetDevicePowerOnFault (uint8_t device_i2c_address, TPS238x_On_Off_t clearEvent,TPS238x_Power_On_Fault_t *poweronfaultports);

uint8_t tps_SetDeviceReMapping(uint8_t device_i2c_address,TPS238x_Remapping_Register_t remapping);
uint8_t tps_GetDeviceReMapping(uint8_t device_i2c_address,uint8_t *remapping);
uint8_t tps_SetPortMultiBitPriority(uint8_t systemPortNum,TPS238x_Multi_Bit_Power_Priority_t multiBitPriority);
uint8_t tps_GetPortMultiBitPriority(uint8_t systemPortNum);

uint8_t tps_SetDevice4PPowerAllocation(uint8_t device_i2c_address,TPS238x_4P_Power_Allocation_t allocatepower12,TPS238x_4P_Power_Allocation_t allocatepower34);
uint8_t tps_SetDevice2PPowerAllocation(uint8_t device_i2c_address,TPS238x_2P_Power_Allocation_t allocatepower12,TPS238x_2P_Power_Allocation_t allocatepower34);
uint8_t tps_GetDevice4P2PPowerAllocatoion(uint8_t device_i2c_address, uint8_t *powerallocation);

uint8_t tps_SetPort4PPolicing(uint8_t systemPortNum,uint8_t icutCurrentThreshold);
uint8_t tps_GetPort4PPolicing(uint8_t systemPortNum, uint8_t *icutCurrentThreshold);

uint8_t tps_SetDevice4PDCDTPCUTILIM(uint8_t device_i2c_address,TPS238x_Dis_PCUT_ILIM_Register_t DCDTPCUTILIMConfig);

uint8_t tps_GetDeviceTemperature (uint8_t device_i2c_address, uint8_t *temperature);
uint8_t tps_GetDeviceInputVoltage (uint8_t device_i2c_address, uint16_t *voltage);

uint8_t tps_GetPortMeasurements (uint8_t systemPortNum, uint16_t *voltage, uint16_t *current);

uint8_t tps_SetPortILIM(uint8_t systemPortNum, TPS238x_Foldback_t poepFoldbackCurve);
uint8_t tps_GetPortILIM(uint8_t systemPortNum, TPS238x_Foldback_t *poepFoldbackCurve);
uint8_t tps_GetDeviceFirmwareRev(uint8_t device_i2c_address,uint8_t *firmwareRev);

uint8_t tps_GetPortDetectResistance (uint8_t systemPortNum, uint8_t *detectResistance);
uint8_t tps_GetPortDetectCapacitance (uint8_t systemPortNum, uint8_t *detectCapacitance);

uint8_t tps_GetPortAssignedClass(uint8_t systemPortNum, uint8_t *preclass, uint8_t *assclass);
uint8_t tps_SetDeviceAutoClass(uint8_t device_i2c_address, TPS238x_Ports_t autoACports, TPS238x_Ports_t manualACports);
uint8_t tps_GetDeviceAutoClass(uint8_t device_i2c_address, TPS238x_Ports_t *autoACports, TPS238x_Ports_t *manualACports);
uint8_t tps_GetPortAutoClassPower (uint8_t systemPortNum,TPS238x_On_Off_t *fault,  uint8_t *autoclasspower);
uint8_t tps_SetDeviceAltFoldbackEnable(uint8_t device_i2c_address, TPS238x_Ports_t altinrush, TPS238x_Ports_t altILIM);
uint8_t tps_GetDeviceAutoClass(uint8_t device_i2c_address, TPS238x_Ports_t *altinrush, TPS238x_Ports_t *altILIM);

uint8_t tps_UpdateSRAMCode(void);

uint8_t tps_ReleasePort (uint8_t systemPortNum);
uint8_t tps_ResetInterruptPin (uint8_t device_i2c_address);
uint32_t tps_GetPortPower (uint8_t systemPortNum);

void PSE_TPS238X_init(TPS238x_Operating_Modes_t opMode);





#ifdef __CPLUSPLUS
}
#endif



#endif /* __TPS238X_H_ */



























































































































