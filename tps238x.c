/*************************************************************************************************************************************************/
/*!     \file TPS238x.c
*
*       \brief Functions that configure and control the TPS23861/TPS2388 Power over Ethernet controller
*
*       \date January 2018
*
*       These software application programming interface functions will allow the user to configure and control the TPS23861/TPS2388 PoE controller.
*
*       These functions are written in C programming language. In order to support numerous processors and hardware systems,
*       the hardware interfaces are all abstracted with functions located in the TPS238x_Glue.c file. The TPS2387 functions
*       will reference a generic I2C read and write function, which the user will convert in the glue layer functions into a
*       specific I2C function for the hardware and OS in use by the user solution.
*
*       \note that the functions in this file are not re-entrant. It is the user's responsibility to assure that these functions
*       are not called until the previous function has completed.
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
***************************************************************************************************************************************************
*                                 MODULE CHANGE LOG                                                                                               *
*                                                                                                                                                 *
*       Date Changed:             { Date }                        Developer:       { Name }                                                       *
*       Change Description:       { Description }                                                                                                 *
*                                                                                                                                                 *
**************************************************************************************************************************************************/
/**************************************************************************************************************************************************
*                                 Included Headers                                                                                                *
**************************************************************************************************************************************************/
#include <tps238x.h>
#include "I2C_functions.h"
#include "usci_uart.h"
#include "sram_code.h"


/**************************************************************************************************************************************************
*                                 Definitions                                                                                                     *
**************************************************************************************************************************************************/

/**************************************************************************************************************************************************
*                                 Prototypes                                                                                                      *
**************************************************************************************************************************************************/

/**************************************************************************************************************************************************
*                                 Global Variables                                                                                                *
**************************************************************************************************************************************************/


/*************************************************************************************************************************************************/
/*  This section of code contains public functions that are generally used by standard applications. The parameter sets for public API functions */
/*  provide an abstraction layer that will be maintained throughout updates and changes to underlying processor functions.                       */
/*                                                                                                                                               */
/*! \cond PUBLIC                                                                                                                                 */
/*************************************************************************************************************************************************/



static TPS238x_System_Port_Map_t TPS_PortMap[TPS_MAX_SYSTEM_CHANNELS] = {{TPS_CHANNEL_NOT_REGISTERED_VALUE, (TPS238x_PortNum_t)TPS_CHANNEL_NOT_REGISTERED_VALUE},
                                                                       {TPS_CHANNEL_NOT_REGISTERED_VALUE, (TPS238x_PortNum_t)TPS_CHANNEL_NOT_REGISTERED_VALUE},
                                                                       {TPS_CHANNEL_NOT_REGISTERED_VALUE, (TPS238x_PortNum_t)TPS_CHANNEL_NOT_REGISTERED_VALUE},
                                                                       {TPS_CHANNEL_NOT_REGISTERED_VALUE, (TPS238x_PortNum_t)TPS_CHANNEL_NOT_REGISTERED_VALUE},

                                                                       {TPS_CHANNEL_NOT_REGISTERED_VALUE, (TPS238x_PortNum_t)TPS_CHANNEL_NOT_REGISTERED_VALUE},
                                                                       {TPS_CHANNEL_NOT_REGISTERED_VALUE, (TPS238x_PortNum_t)TPS_CHANNEL_NOT_REGISTERED_VALUE},
                                                                       {TPS_CHANNEL_NOT_REGISTERED_VALUE, (TPS238x_PortNum_t)TPS_CHANNEL_NOT_REGISTERED_VALUE},
                                                                       {TPS_CHANNEL_NOT_REGISTERED_VALUE, (TPS238x_PortNum_t)TPS_CHANNEL_NOT_REGISTERED_VALUE},

                                                                       {TPS_CHANNEL_NOT_REGISTERED_VALUE, (TPS238x_PortNum_t)TPS_CHANNEL_NOT_REGISTERED_VALUE},
                                                                       {TPS_CHANNEL_NOT_REGISTERED_VALUE, (TPS238x_PortNum_t)TPS_CHANNEL_NOT_REGISTERED_VALUE},
                                                                       {TPS_CHANNEL_NOT_REGISTERED_VALUE, (TPS238x_PortNum_t)TPS_CHANNEL_NOT_REGISTERED_VALUE},
                                                                       {TPS_CHANNEL_NOT_REGISTERED_VALUE, (TPS238x_PortNum_t)TPS_CHANNEL_NOT_REGISTERED_VALUE},

                                                                       {TPS_CHANNEL_NOT_REGISTERED_VALUE, (TPS238x_PortNum_t)TPS_CHANNEL_NOT_REGISTERED_VALUE},
                                                                       {TPS_CHANNEL_NOT_REGISTERED_VALUE, (TPS238x_PortNum_t)TPS_CHANNEL_NOT_REGISTERED_VALUE},
                                                                       {TPS_CHANNEL_NOT_REGISTERED_VALUE, (TPS238x_PortNum_t)TPS_CHANNEL_NOT_REGISTERED_VALUE},
                                                                       {TPS_CHANNEL_NOT_REGISTERED_VALUE, (TPS238x_PortNum_t)TPS_CHANNEL_NOT_REGISTERED_VALUE},

                                                                       {TPS_CHANNEL_NOT_REGISTERED_VALUE, (TPS238x_PortNum_t)TPS_CHANNEL_NOT_REGISTERED_VALUE},
                                                                       {TPS_CHANNEL_NOT_REGISTERED_VALUE, (TPS238x_PortNum_t)TPS_CHANNEL_NOT_REGISTERED_VALUE},
                                                                       {TPS_CHANNEL_NOT_REGISTERED_VALUE, (TPS238x_PortNum_t)TPS_CHANNEL_NOT_REGISTERED_VALUE},
                                                                       {TPS_CHANNEL_NOT_REGISTERED_VALUE, (TPS238x_PortNum_t)TPS_CHANNEL_NOT_REGISTERED_VALUE},

                                                                       {TPS_CHANNEL_NOT_REGISTERED_VALUE, (TPS238x_PortNum_t)TPS_CHANNEL_NOT_REGISTERED_VALUE},
                                                                       {TPS_CHANNEL_NOT_REGISTERED_VALUE, (TPS238x_PortNum_t)TPS_CHANNEL_NOT_REGISTERED_VALUE},
                                                                       {TPS_CHANNEL_NOT_REGISTERED_VALUE, (TPS238x_PortNum_t)TPS_CHANNEL_NOT_REGISTERED_VALUE},
                                                                       {TPS_CHANNEL_NOT_REGISTERED_VALUE, (TPS238x_PortNum_t)TPS_CHANNEL_NOT_REGISTERED_VALUE},

                                                                       {TPS_CHANNEL_NOT_REGISTERED_VALUE, (TPS238x_PortNum_t)TPS_CHANNEL_NOT_REGISTERED_VALUE},
                                                                       {TPS_CHANNEL_NOT_REGISTERED_VALUE, (TPS238x_PortNum_t)TPS_CHANNEL_NOT_REGISTERED_VALUE},
                                                                       {TPS_CHANNEL_NOT_REGISTERED_VALUE, (TPS238x_PortNum_t)TPS_CHANNEL_NOT_REGISTERED_VALUE},
                                                                       {TPS_CHANNEL_NOT_REGISTERED_VALUE, (TPS238x_PortNum_t)TPS_CHANNEL_NOT_REGISTERED_VALUE},

                                                                       {TPS_CHANNEL_NOT_REGISTERED_VALUE, (TPS238x_PortNum_t)TPS_CHANNEL_NOT_REGISTERED_VALUE},
                                                                       {TPS_CHANNEL_NOT_REGISTERED_VALUE, (TPS238x_PortNum_t)TPS_CHANNEL_NOT_REGISTERED_VALUE},
                                                                       {TPS_CHANNEL_NOT_REGISTERED_VALUE, (TPS238x_PortNum_t)TPS_CHANNEL_NOT_REGISTERED_VALUE},
                                                                       {TPS_CHANNEL_NOT_REGISTERED_VALUE, (TPS238x_PortNum_t)TPS_CHANNEL_NOT_REGISTERED_VALUE},

                                                                       {TPS_CHANNEL_NOT_REGISTERED_VALUE, (TPS238x_PortNum_t)TPS_CHANNEL_NOT_REGISTERED_VALUE},
                                                                       {TPS_CHANNEL_NOT_REGISTERED_VALUE, (TPS238x_PortNum_t)TPS_CHANNEL_NOT_REGISTERED_VALUE},
                                                                       {TPS_CHANNEL_NOT_REGISTERED_VALUE, (TPS238x_PortNum_t)TPS_CHANNEL_NOT_REGISTERED_VALUE},
                                                                       {TPS_CHANNEL_NOT_REGISTERED_VALUE, (TPS238x_PortNum_t)TPS_CHANNEL_NOT_REGISTERED_VALUE},

                                                                       {TPS_CHANNEL_NOT_REGISTERED_VALUE, (TPS238x_PortNum_t)TPS_CHANNEL_NOT_REGISTERED_VALUE},
                                                                       {TPS_CHANNEL_NOT_REGISTERED_VALUE, (TPS238x_PortNum_t)TPS_CHANNEL_NOT_REGISTERED_VALUE},
                                                                       {TPS_CHANNEL_NOT_REGISTERED_VALUE, (TPS238x_PortNum_t)TPS_CHANNEL_NOT_REGISTERED_VALUE},
                                                                       {TPS_CHANNEL_NOT_REGISTERED_VALUE, (TPS238x_PortNum_t)TPS_CHANNEL_NOT_REGISTERED_VALUE},

                                                                       {TPS_CHANNEL_NOT_REGISTERED_VALUE, (TPS238x_PortNum_t)TPS_CHANNEL_NOT_REGISTERED_VALUE},
                                                                       {TPS_CHANNEL_NOT_REGISTERED_VALUE, (TPS238x_PortNum_t)TPS_CHANNEL_NOT_REGISTERED_VALUE},
                                                                       {TPS_CHANNEL_NOT_REGISTERED_VALUE, (TPS238x_PortNum_t)TPS_CHANNEL_NOT_REGISTERED_VALUE},
                                                                       {TPS_CHANNEL_NOT_REGISTERED_VALUE, (TPS238x_PortNum_t)TPS_CHANNEL_NOT_REGISTERED_VALUE},

                                                                       {TPS_CHANNEL_NOT_REGISTERED_VALUE, (TPS238x_PortNum_t)TPS_CHANNEL_NOT_REGISTERED_VALUE},
                                                                       {TPS_CHANNEL_NOT_REGISTERED_VALUE, (TPS238x_PortNum_t)TPS_CHANNEL_NOT_REGISTERED_VALUE},
                                                                       {TPS_CHANNEL_NOT_REGISTERED_VALUE, (TPS238x_PortNum_t)TPS_CHANNEL_NOT_REGISTERED_VALUE},
                                                                       {TPS_CHANNEL_NOT_REGISTERED_VALUE, (TPS238x_PortNum_t)TPS_CHANNEL_NOT_REGISTERED_VALUE},
};

/*************************************************************************************************************************************************
*  tps_RegisterPort
**************************************************************************************************************************************************/
/*!
* @brief Allocates a system level port number for a given TPS23861/TPS2388 device and it's specific port number.
*
* It is easier for users to maintain a system with 4 TPS23861/TPS2388 with 4 ports each, when each port has a unique number. By registering each of the 16
* individual ports (in this example), the user can work with ports 0-15, rather than have to maintain device 1 - port 0, through device 4 - port 3.
*
* This function must be called prior to any function call that has a systemPortNum as an input. The current logic has room for
* TPS_MAX_SYSTEM_PORTS (64) ports. The TPS_PortMap[] variable can be modified to support more ports (or save some memory by supporting fewer).
*
* @param[in]   device_i2c_address  7 bit I2C address of the TPS238x part being controlled (do not included R/W bit)
* @param[in]   devicePortNum       TPS238X_PORT_1 - TPS238X_PORT_4, The port number ON THE SPECIFIC TPS23861/TPS2388 device
*
* @return  uint8_t  systemPortNum     Unique handle that will be used to refer to this device and port number in future function calls.
* @return  TPS_ERR_NO_PORT_AVAILABLE  No port number available, all are in use
*
* @sa tps_ReleasePort ()
**************************************************************************************************************************************************/
uint8_t tps_RegisterPort (uint16_t device_i2c_address, TPS238x_PortNum_t devicePortNum)
{
uint8_t i;
uint8_t found = 0xff;

    for (i=0; i<TPS_MAX_SYSTEM_CHANNELS; i++)
    {
        if (TPS_PortMap[i].i2cAddress == TPS_CHANNEL_NOT_REGISTERED_VALUE)
        {
            found = i;
            TPS_PortMap[i].i2cAddress = device_i2c_address;
            TPS_PortMap[i].devicePortNum = devicePortNum;
            break;
        }
    }

    if (found == 0xff)
        return (TPS_ERR_NO_PORT_AVAILABLE);
    else
        return (found);
}

/*************************************************************************************************************************************************
*  tps_GetDeviceI2CAddress
**************************************************************************************************************************************************/
/*!
* @brief Get the TPS23861/TPS2388 I2C Address associated with a registered System Port Number
*
* This function returns the I2C address for the TPS23861/TPS2388 device registered for a given System Port Number
*
* @param[in]   systemPortNum          Port Number Value returned from the port registration function, tps_RegisterPort()
*
* @return  uint16_t    I2C Address number associated with this specific port.
* @return  TPS_PORT_NOT_REGISTERED_VALUE - Indicates no port associated.
*
* @sa tps_RegisterPort ()
* @sa tps_GetDevicePortNum ()
**************************************************************************************************************************************************/
uint16_t tps_GetDeviceI2CAddress (uint8_t systemPortNum)
{
    return (TPS_PortMap[systemPortNum].i2cAddress);
}

/*************************************************************************************************************************************************
*  tps_GetDevicePortNum
**************************************************************************************************************************************************/
/*!
* @brief Get the TPS2388 device port number associated with a registered System Port Number
*
* This function returns the device port number for the TPS2388 device registered for a given System Port Number
*
* @param[in]   systemPortNum          Port Number Value returned from the port registration function, tps_RegisterPort()
*
* @return  uint16_t    Device Port Number associated with this specific System Port Number.
* @return  TPS_PORT_NOT_REGISTERED_VALUE - Indicates no port associated.
*
* @sa tps_RegisterPort ()
* @sa tps_GetDeviceI2CAddress ()
**************************************************************************************************************************************************/
TPS238x_PortNum_t tps_GetDevicePortNum (uint8_t systemPortNum)
{
    return (TPS_PortMap[systemPortNum].devicePortNum);
}

/*************************************************************************************************************************************************
*  tps_GetSystemPortNumber
**************************************************************************************************************************************************/
/*!
* @brief Returns the handle already allocated for the given TPS23861/TPS2388 and device port number.
*
* This function can be used if the System Port Number returned in the tps_RegisterPort() function is lost or not maintained.
*
* @param[in]   device_i2c_address   7 bit I2C address of the TPS238x part being controlled (do not included R/W bit)
* @param[in]   devicePortNum        TPS238X_PORT_1 - TPS238X_PORT_4, The port number ON THE SPECIFIC TPS23861/TPS2388 device
*
* @return  uint8_t  systemPortNum         Unique handle that will be used to refer to this device and port number in future function calls.
* @return  TPS_PORT_NOT_REGISTERED_VALUE  This combination of device and port number was not registered
*
* @sa tps_RegisterPort ()
**************************************************************************************************************************************************/
uint8_t tps_GetSystemPortNumber (uint16_t device_i2c_address, TPS238x_PortNum_t devicePortNum)
{
uint8_t i;

    for (i=0; i<TPS_MAX_SYSTEM_CHANNELS; i++)
    {
        if ((TPS_PortMap[i].i2cAddress == device_i2c_address) && (TPS_PortMap[i].devicePortNum == devicePortNum))
        {
            return (i);
        }
    }

    return (TPS_CHANNEL_NOT_REGISTERED_VALUE);
}



/**************************************************************************************************************************************************
*                                 Interrupt Configuration Functions
**************************************************************************************************************************************************/

/*************************************************************************************************************************************************
*  tps_SetDeviceInterruptMask
**************************************************************************************************************************************************/
/*!
* @brief Set the interrupt mask register
*
* The function sets which TPS23861 events/faults are allowed to generate interrupts (unmasked).
*
* @param[in]   device_i2c_address 7 bit I2C address of the TPS238x part being controlled (do not included R/W bit)
* @param[in]   intMask            TPS238X_Interrupt_Mask_Register_t variable that contains the conditions that are allowed to generate interrupts
*                                        PEMSK_Power_Enable_Unmask
*                                        PGMSK_Power_Good_Unmask
*                                        DIMSK_Disconnect_Unmask
*                                        DEMSK_Detection_Cycle_Unmask
*                                        CLMSK_Classificiation_Cycle_Unmask
*                                        IFMSK_IFAULT_Unmask
*                                        INMSK_Inrush_Fault_Unmask
*                                        SUMSK_Supply_Event_Fault_Unmask
*
* @return  uint8_t    (I2C_SUCCESSFUL or I2C error status)
*
* @sa tps_GetDeviceInterruptMask ()
* @sa tps_GetDeviceInterruptStatus ()
* @sa tps_GetDeviceAllInterruptEvents ()
**************************************************************************************************************************************************/
uint8_t tps_SetDeviceInterruptMask (uint8_t device_i2c_address, TPS238X_Interrupt_Mask_Register_t intMask)
{
uint8_t rtn;

    rtn = tps_WriteI2CReg (device_i2c_address, TPS238X_INTERRUPT_MASK_COMMAND, *(uint8_t*)&intMask);
    return (rtn);
}

/*************************************************************************************************************************************************
*  tps_GetDeviceInterruptMask
**************************************************************************************************************************************************/
/*!
* @brief Get the current setting of the interrupt mask register
*
* The function returns the TPS23861/TPS2388 events/faults that are able to generate interrupts (unmasked).
*
* @param[in]   device_i2c_address  7 bit I2C address of the TPS238x part being controlled (do not included R/W bit)
* @param[out]  *intMask            Address of a TPS238X_Interrupt_Mask_Register_t variable that will the current events that can generate interrupts
*                                        PEMSK_Power_Enable_Unmask
*                                        PGMSK_Power_Good_Unmask
*                                        DIMSK_Disconnect_Unmask
*                                        DEMSK_Detection_Cycle_Unmask
*                                        CLMSK_Classificiation_Cycle_Unmask
*                                        IFMSK_IFAULT_Unmask
*                                        INMSK_Inrush_Fault_Unmask
*                                        SUMSK_Supply_Event_Fault_Unmask
* @param[out]  *intDelayTime       Address of a uint8_t variable that will indicate the amount of defer time, with a 10ms lsb, of non-critical interrupts.
*
* @return  uint8_t    (I2C_SUCCESSFUL or I2C error status)
*
* @sa tps_SetDeviceInterruptMask ()
* @sa tps_GetDeviceInterruptStatus ()
* @sa tps_GetDeviceAllInterruptEvents ()
* @sa tps_GetSystemPortNumber ()
**************************************************************************************************************************************************/
uint8_t tps_GetDeviceInterruptMask (uint8_t device_i2c_address, TPS238X_Interrupt_Mask_Register_t *intMask)
{
uint8_t rtn;

    rtn = tps_ReadI2CReg (device_i2c_address, TPS238X_INTERRUPT_MASK_COMMAND, (uint8_t *)intMask);

    return (rtn);
}






/**************************************************************************************************************************************************
*                                 Interrupt Based Event Functions                                                                                                *
**************************************************************************************************************************************************/

/*************************************************************************************************************************************************
*  tps_GetDeviceInterruptStatus
**************************************************************************************************************************************************/
/*!
* @brief Get the current interrupt status for the indicated TPS23861/TPS2388 part.
*
* The function returns a variable that has a one bit indicator for each of the interrupts in the TPS23861/TPS2388.
*
* The interrupt mask register identifies which events/faults will generate an interrupt. The status register will still indicate
* events/faults that are masked and would not generate an interrupt. If the user wants to only process unmasked interrupts, the results
* from this function must be combined with the interrupt mask (tps_GetInterruptMask)
*
* @param[in]   device_i2c_address  7 bit I2C address of the TPS238x part being controlled (do not included R/W bit)
* @param[out]  *status             Address of a TPS238X_Interrupt_Register_t variable that will receive the current interrupt status
*                                       PEC_Power_Enable_Change
*                                       PGC_Power_Good_Change
*                                       DISF_Disconnect_Event
*                                       DETC_Detection_Cycle
*                                       CLASC_Classification_Cycle
*                                       IFAULT_ICUT_ILIM_Fault
*                                       INRF_Inrush_Fault
*                                       SUPF_Supply_Event_Fault
*
* @return  uint8_t    (I2C_SUCCESSFUL or I2C error status)
*
* @sa tps_GetDeviceAllInterruptEvents ()
* @sa tps_SetDeviceInterruptMask ()
* @sa tps_GetDeviceInterruptMask ()
* @sa tps_GetSystemPortNumber ()
**************************************************************************************************************************************************/
uint8_t tps_GetDeviceInterruptStatus (uint8_t device_i2c_address, TPS238X_Interrupt_Register_t *status)
{
uint8_t rtn;
uint8_t value;

    rtn = tps_ReadI2CReg (device_i2c_address, TPS238X_INTERRUPT_COMMAND, &value);

    *status = *(TPS238X_Interrupt_Register_t *)&value;
    return (rtn);
}

/*************************************************************************************************************************************************
*  tps_GetDeviceAllInterruptEvents
**************************************************************************************************************************************************/
/*!
* @brief Returns all of the individual event/fault registers that generated a TPS23861/TPS2388 interrupt
*
* There are a number of events and faults that can generate an interrupt. This function returns the individual indicators for each of the
* interruptable event/fault conditions.
*
* Calling this function will clear the interrupts associated with these events/faults, so the user should process all indicators present.

* There are individual functions that return the event/fault status for an individual register.
*
* @param[in]   device_i2c_address        7 bit I2C address of the TPS238x part being controlled (do not included R/W bit)
* @param[in]   clearEvent                TPS_ON will cause the events to be cleared in the hardware, TPS_OFF will just read the current value of the events
* @param[out]  *powerEnablePortEvents    Address of a TPS238x_Ports_t variable that will receive the ports containing a power enable status change
* @param[out]  *powerGoodPortEvents      Address of a TPS238x_Ports_t variable that will receive the ports containing a power good status change
* @param[out]  *detectionPortEvents      Address of a TPS238x_Ports_t variable that will receive the ports that had a detection cycle
* @param[out]  *classificationPortEvents Address of a TPS238x_Ports_t variable that will receive the ports that had a classification cycle
* @param[out]  *icutPortEvents           Address of a TPS238x_Ports_t variable that will receive the ports that had a ICUT fault
* @param[out]  *disconnectPortEvents     Address of a TPS238x_Ports_t variable that will receive the ports that had a disconnect event
* @param[out]  *inrushPortEvents         Address of a TPS238x_Ports_t variable that will receive the ports that had a inrush fault at port turn on
* @param[out]  *ilimPortEvents           Address of a TPS238x_Ports_t variable that will receive the ports that had a ILIM fault occurred
* @param[out]  *supplyEvents             Address of a TPS238X_Supply_Event_Register_t variable that will receive the supply event faults
*                                               VPUV_VPower_Undervoltage_Event
*                                               VDUV_Vdd_UVLO_Event
*                                               TSD_Thermal_Shutdown_Event
*
* @return  uint8_t    (I2C_SUCCESSFUL or I2C error status)
*
* @note Since the individual hardware registers contain more than one status in them, event status cannot be processed individually.
*
* @sa tps_SetDeviceInterruptMask ()
* @sa tps_GetDeviceInterruptMask ()
* @sa tps_GetDeviceInterruptStatus ()
* @sa tps_GetSystemPortNumber ()
**************************************************************************************************************************************************/
uint8_t tps_GetDeviceAllInterruptEvents (uint8_t device_i2c_address,
                                         TPS238x_On_Off_t clearEvent,
                                         TPS238x_Ports_t *powerEnablePortEvents,
                                         TPS238x_Ports_t *powerGoodPortEvents,
                                         TPS238x_Ports_t *detectionPortEvents,
                                         TPS238x_Ports_t *classificationPortEvents,
                                         TPS238x_Ports_t *pcutPortEvents,
                                         TPS238x_Ports_t *disconnectPortEvents,
                                         TPS238x_Ports_t *inrushPortEvents,
                                         TPS238x_Ports_t *ilimPortEvents,
                                         uint8_t *supplyEvents)
{
uint8_t rtn;
uint8_t value;
uint8_t value_out;

    // Assure that the interrupt pin is released
//    tps_ResetInterruptPin (device_i2c_address);

    rtn = tps_ReadI2CReg (device_i2c_address, TPS238X_POWER_EVENT_COMMAND + clearEvent, &value);
    value_out = value & 0xf;
    *powerEnablePortEvents = value_out;


    value_out = value >> POWER_ENABLE_EVENT_SHIFT;
    *powerGoodPortEvents = value_out;

    rtn += tps_ReadI2CReg (device_i2c_address, TPS238X_DETECTION_EVENT_COMMAND + clearEvent, &value);
    value_out = value & 0xf;
    *detectionPortEvents = value_out;

    value_out = value >> CLASSIFICATION_EVENT_SHIFT;
    *classificationPortEvents = value_out;


    rtn += tps_ReadI2CReg (device_i2c_address, TPS238X_FAULT_EVENT_COMMAND + clearEvent, &value);
    value_out = value & 0xf;
    *pcutPortEvents = value_out;

    value_out = value >> DISCONNECT_EVENT_SHIFT;
    *disconnectPortEvents = value_out;


    rtn += tps_ReadI2CReg (device_i2c_address, TPS238X_START_LIMIT_EVENT_COMMAND + clearEvent, &value);
    value_out = value & 0xf;
    *inrushPortEvents = value_out;

    value_out = value >> ILIM_EVENT_SHIFT;
    *ilimPortEvents = value_out;


    rtn += tps_ReadI2CReg (device_i2c_address, TPS238X_SUPPLY_EVENT_COMMAND + clearEvent, &value);
    *supplyEvents = value;

    return (rtn);
}



uint8_t tps_GetDevicePowerEnableEvent(uint8_t device_i2c_address,TPS238x_On_Off_t clearEvent,TPS238x_Ports_t *powerEnablePortEvents)
{
    uint8_t rtn;
    uint8_t value;
    rtn = tps_ReadI2CReg (device_i2c_address, TPS238X_POWER_EVENT_COMMAND + clearEvent, &value);
    *powerEnablePortEvents = value & 0x0f;

    return(rtn);
}


uint8_t tps_GetDevicePowerGoodEvent(uint8_t device_i2c_address,TPS238x_On_Off_t clearEvent,TPS238x_Ports_t *powerGoodPortEvents)
{
    uint8_t rtn;
    uint8_t value;
    rtn = tps_ReadI2CReg (device_i2c_address, TPS238X_POWER_EVENT_COMMAND + clearEvent, &value);
    *powerGoodPortEvents = value >> POWER_ENABLE_EVENT_SHIFT;

    return(rtn);
}

uint8_t tps_GetDeviceDetectionEvent(uint8_t device_i2c_address,TPS238x_On_Off_t clearEvent,TPS238x_Ports_t *detectionPortEvents)
{
    uint8_t rtn;
    uint8_t value;
    rtn = tps_ReadI2CReg (device_i2c_address, TPS238X_DETECTION_EVENT_COMMAND + clearEvent, &value);
    *detectionPortEvents = value & 0x0f;
    return (rtn);
}

uint8_t tps_GetDeviceClassificationEvent(uint8_t device_i2c_address,TPS238x_On_Off_t clearEvent,TPS238x_Ports_t *classificationPortEvents)
{
    uint8_t rtn;
    uint8_t value;
    rtn = tps_ReadI2CReg (device_i2c_address, TPS238X_DETECTION_EVENT_COMMAND + clearEvent, &value);
    *classificationPortEvents = value >> CLASSIFICATION_EVENT_SHIFT;

    return(rtn);
}

uint8_t tps_GetDevicePCUTFaultEvent(uint8_t device_i2c_address,TPS238x_On_Off_t clearEvent,TPS238x_Ports_t *pcutPortEvents)
{
    uint8_t rtn;
    uint8_t value;
    rtn = tps_ReadI2CReg (device_i2c_address, TPS238X_FAULT_EVENT_COMMAND + clearEvent, &value);
    *pcutPortEvents = value & 0x0f;
    return (rtn);
}

uint8_t tps_GetDeviceDisconnectEvent(uint8_t device_i2c_address,TPS238x_On_Off_t clearEvent,TPS238x_Ports_t *disconnectPortEvents)
{
    uint8_t rtn;
    uint8_t value;
    rtn = tps_ReadI2CReg (device_i2c_address, TPS238X_FAULT_EVENT_COMMAND + clearEvent, &value);
    *disconnectPortEvents = value >> DISCONNECT_EVENT_SHIFT;

    return(rtn);
}

uint8_t tps_GetDeviceStartEvent(uint8_t device_i2c_address,TPS238x_On_Off_t clearEvent,TPS238x_Ports_t *inrushPortEvents)
{
    uint8_t rtn;
    uint8_t value;
    rtn = tps_ReadI2CReg (device_i2c_address, TPS238X_START_LIMIT_EVENT_COMMAND + clearEvent, &value);
    *inrushPortEvents = value & 0x0f;
    return (rtn);
}

uint8_t tps_GetDeviceILIMEvent(uint8_t device_i2c_address,TPS238x_On_Off_t clearEvent,TPS238x_Ports_t *ilimPortEvents)
{
    uint8_t rtn;
    uint8_t value;
    rtn = tps_ReadI2CReg (device_i2c_address, TPS238X_START_LIMIT_EVENT_COMMAND + clearEvent, &value);
    *ilimPortEvents = value >> ILIM_EVENT_SHIFT;

    return(rtn);
}


uint8_t tps_GetDeviceSupplyEvent(uint8_t device_i2c_address,TPS238x_On_Off_t clearEvent,uint8_t *supplyEvents)
{
    uint8_t rtn;
    uint8_t value;
    rtn = tps_ReadI2CReg (device_i2c_address, TPS238X_SUPPLY_EVENT_COMMAND + clearEvent, &value);
    *supplyEvents = value;

    return(rtn);
}



/**************************************************************************************************************************************************
*                                 System Status Functions                                                                                                *
**************************************************************************************************************************************************/

/*************************************************************************************************************************************************
*  tps_GetPortDetectRequestedClassStatus
**************************************************************************************************************************************************/
/*!
* @brief Returns the detection and requested classification status of the specified port
*
* This function will return the most recent detection and classification result for the indicated port on the TPS23861/TPS2388.
*
* The function will return a 0 (CLASS_UNKNOWN) when the port is turned off.
*
* @param[in]   systemPortNum          Port Number Value returned from the port registration function, tps_RegisterPort()
* @param[out]  *detectionStatus       Address of a TPS238x_Detection_Status_t variable that will receive the detecttion status
*                                            for the indicated port (DETECT_UNKNOWN, DETECT_SHORT_CIRCUIT, DETECT_RESIST_LOW,
*                                            DETECT_RESIST_VALID, DETECT_RESIST_HIGH, DETECT_OPEN_CIRCUIT, DETECT_MOSFET_FAULT,
*                                            DETECT_LEGACY, DETECT_CAP_INVALID_CLAMP_VOLTAGE, DETECT_CAP_INVALID_DELTA_V, or
*                                            DETECT_CAP_VALID_LEGACY_RANGE)
* @param[out]  *classificationStatus  Address of a TPS238x_Classification_Status_t variable that will receive the classification status
*                                            for the indicated port (CLASS_UNKNOWN, CLASS_1, CLASS_2, CLASS_3, CLASS_4,
*                                            CLASS_0, CLASS_OVERCURRENT, CLASS_MISMATCH, or CLASS_5)
*
* @return  uint8_t    (I2C_SUCCESSFUL or I2C error status)
*
* @sa tps_GetPortDetectionStatus ()
* @sa tps_GetPortRequestedClassificationStatus ()
**************************************************************************************************************************************************/
uint8_t tps_GetPortDetectRequestedClassStatus (uint8_t systemPortNum, TPS238x_Detection_Status_t *detectionStatus,
                                      TPS238x_Classification_Status_t *classificationStatus)
{
uint8_t rtn;
uint8_t value;
uint8_t portNum = tps_GetDevicePortNum(systemPortNum);
uint8_t command = TPS238X_CHANNEL_1_STATUS_COMMAND + (uint8_t) portNum - 1;   // Determine which Port Status command has been requested

    rtn = tps_ReadI2CReg (tps_GetDeviceI2CAddress(systemPortNum), command, &value);

    *classificationStatus = (TPS238x_Classification_Status_t)GET_CLASS(value);
    *detectionStatus = (TPS238x_Detection_Status_t)GET_DETECT(value);

    return (rtn);
}

/*************************************************************************************************************************************************
*  tps_GetPortDetectionStatus
**************************************************************************************************************************************************/
/*!
* @brief Returns the detection status of the specified port
*
* This function will return the most recent detection result for the indicated port on the TPS23861/TPS2388.
*
* The function will return a 0 (CLASS_UNKNOWN) when the port is turned off.
*
* @param[in]   systemPortNum          Port Number Value returned from the port registration function, tps_RegisterPort()
* @param[out]  *detectionStatus       Address of a TPS238x_Detection_Status_t variable that will receive the detecttion status
*                                            for the indicated port (DETECT_UNKNOWN, DETECT_SHORT_CIRCUIT, DETECT_RESIST_LOW,
*                                            DETECT_RESIST_VALID, DETECT_RESIST_HIGH, DETECT_OPEN_CIRCUIT, DETECT_MOSFET_FAULT,
*                                            DETECT_LEGACY, DETECT_CAP_INVALID_CLAMP_VOLTAGE, DETECT_CAP_INVALID_DELTA_V, or
*                                            DETECT_CAP_INVALID_LEGACY_RANGE)
*
* @return  uint8_t    (I2C_SUCCESSFUL or I2C error status)
*
* @sa tps_GetPortClassificationStatus ()
* @sa tps_GetPortDetectClassStatus ()
**************************************************************************************************************************************************/
uint8_t tps_GetPortDetectionStatus (uint8_t systemPortNum, TPS238x_Detection_Status_t *detectionStatus)
{
uint8_t rtn;
uint8_t value;
uint8_t portNum = tps_GetDevicePortNum(systemPortNum);
uint8_t command = TPS238X_CHANNEL_1_STATUS_COMMAND + (uint8_t) portNum - 1;   // Determine which Port Status command has been requested

    rtn = tps_ReadI2CReg (tps_GetDeviceI2CAddress(systemPortNum), command, &value);

    *detectionStatus = (TPS238x_Detection_Status_t)GET_DETECT(value);

    return (rtn);
}

/*************************************************************************************************************************************************
*  tps_GetPortRequestedClassificationStatus
**************************************************************************************************************************************************/
/*!
* @brief Returns the classification status of the specified port
*
* This function will return the most recent classification result for the indicated port on the TPS23861/TPS2388.
*
* The function will return a 0 (CLASS_UNKNOWN) when the port is turned off.
*
* @param[in]   systemPortNum          Port Number Value returned from the port registration function, tps_RegisterPort()
* @param[out]  *classificationStatus  Address of a TPS238x_Classification_Status_t variable that will receive the classification status
*                                            for the indicated port (CLASS_UNKNOWN, CLASS_1, CLASS_2, CLASS_3, CLASS_4,
*                                            CLASS_0, CLASS_OVERCURRENT, CLASS_MISMATCH, or CLASS_5)
*
* @return  uint8_t    (I2C_SUCCESSFUL or I2C error status)
*
* @sa tps_GetPortDetectionStatus ()
* @sa tps_GetPortDetectClassStatus ()
**************************************************************************************************************************************************/
uint8_t tps_GetPortRequestedClassificationStatus (uint8_t systemPortNum, TPS238x_Classification_Status_t *classificationStatus)
{
uint8_t rtn;
uint8_t value;
uint8_t portNum = tps_GetDevicePortNum(systemPortNum);
uint8_t command = TPS238X_CHANNEL_1_STATUS_COMMAND + (uint8_t) portNum - 1;   // Determine which Port Status command has been requested

    rtn = tps_ReadI2CReg (tps_GetDeviceI2CAddress(systemPortNum), command, &value);

    *classificationStatus = (TPS238x_Classification_Status_t)GET_CLASS(value);

    return (rtn);
}

/*************************************************************************************************************************************************
*  tps_GetDevicePowerStatus
**************************************************************************************************************************************************/
/*!
* @brief Returns the power enable and power good status of the 4 ports in the TPS23861/TPS2388
*
* This function will return both the power enable and the power good state of each port.
*
* @param[in]   device_i2c_address        7 bit I2C address of the TPS238x part being controlled (do not included R/W bit)
* @param[out]  *powerEnablePorts         Address of a TPS238x_Ports_t variable that will receive the power enable status with
*                                            one bit for each port in power enabled state.
* @param[out]  *powerGoodPorts           Address of a TPS238x_Ports_t variable that will receive the power good status with
*                                            one bit for each port in power good state.
*
* @return  uint8_t    (I2C_SUCCESSFUL or I2C error status)
*
* @sa tps_GetDevicePowerEnableStatus ()
* @sa tps_GetPortPowerEnableStatus ()
* @sa tps_GetDevicePowerGoodStatus ()
* @sa tps_GetPortPowerGoodStatus ()
* @sa tps_GetSystemPortNumber ()
**************************************************************************************************************************************************/
uint8_t tps_GetDevicePowerStatus (uint8_t device_i2c_address, TPS238x_Ports_t *powerEnablePorts, TPS238x_Ports_t *powerGoodPorts)
{
uint8_t rtn;
uint8_t value;
uint8_t value_out;

    rtn = tps_ReadI2CReg (device_i2c_address, TPS238X_POWER_STATUS_COMMAND, &value);

    value_out = GET_POWER_ENABLE_STATUS (value);
    *powerEnablePorts = value_out;

    value_out = GET_POWER_GOOD_STATUS(value);
    *powerGoodPorts = value_out;

    return (rtn);
}

/*************************************************************************************************************************************************
*  tps_GetDevicePowerEnableStatus
**************************************************************************************************************************************************/
/*!
* @brief Returns the power enable status of the 4 ports in the TPS23861/TPS2388
*
* This function will return the power good state of each port.
*
* @param[in]   device_i2c_address        7 bit I2C address of the TPS238x part being controlled (do not included R/W bit)
* @param[out]  *powerEnablePorts         Address of a TPS238x_Ports_t variable that will receive the power enable status with
*                                            one bit for each port in power enabled state.
*
* @return  uint8_t    (I2C_SUCCESSFUL or I2C error status)
*
* @sa tps_GetPortPowerEnableStatus ()
* @sa tps_GetDevicePowerGoodStatus ()
* @sa tps_GetPortPowerGoodStatus ()
* @sa tps_GetDevicePowerStatus ()
* @sa tps_GetSystemPortNumber ()
**************************************************************************************************************************************************/
uint8_t tps_GetDevicePowerEnableStatus (uint8_t device_i2c_address, TPS238x_Ports_t *powerEnablePorts)
{
uint8_t rtn;
uint8_t value;

    rtn = tps_ReadI2CReg (device_i2c_address, TPS238X_POWER_STATUS_COMMAND, &value);

    value = GET_POWER_ENABLE_STATUS(value);
    *powerEnablePorts = *(TPS238x_Ports_t*)&value;

    return (rtn);
}

/*************************************************************************************************************************************************
*  tps_GetPortPowerEnableStatus
**************************************************************************************************************************************************/
/*!
* @brief Returns the power enable status of one of the 4 ports in the TPS23861/TPS2388
*
* This function will return the power enable state of the indicated port.
*
* @param[in]   systemPortNum          Port Number Value returned from the port registration function, tps_RegisterPort()
*
* @return  uint8_t     TPS_ON if Power Enabled
*                      TPS_OFF is Power is NOT enabled,
*                      TPS_ERR_I2C_ERROR on I2C error status
*
* @sa tps_GetDevicePowerEnableStatus ()
* @sa tps_GetDevicePowerGoodStatus ()
* @sa tps_GetPortPowerGoodStatus ()
* @sa tps_GetDevicePowerStatus ()
* @sa tps_GetSystemPortNumber ()
**************************************************************************************************************************************************/
uint8_t tps_GetPortPowerEnableStatus (uint8_t systemPortNum)
{

uint8_t value;
uint8_t portNum = tps_GetDevicePortNum(systemPortNum);

    tps_ReadI2CReg (tps_GetDeviceI2CAddress(systemPortNum), TPS238X_POWER_STATUS_COMMAND, &value);
    value = GET_POWER_ENABLE_STATUS(value);
    if (value & CONVERT_PORT_NUM(portNum))
        return (TPS_ON);
    else
        return (TPS_OFF);

}

/*************************************************************************************************************************************************
*  tps_GetDevicePowerGoodStatus
**************************************************************************************************************************************************/
/*!
* @brief Returns the power good status of the 4 ports in the TPS23861/TPS2388
*
* This function will return the power good state of each port.
*
* @param[in]   device_i2c_address        7 bit I2C address of the TPS238x part being controlled (do not included R/W bit)
* @param[out]  *powerGoodPorts           Address of a TPS238x_Ports_t variable that will receive the power good status with
*                                            one bit for each port in power good state.
*
* @return  uint8_t    (I2C_SUCCESSFUL or I2C error status)
*
* @sa tps_GetPortPowerGoodStatus ()
* @sa tps_GetDevicePowerEnableStatus ()
* @sa tps_GetPortPowerEnableStatus ()
* @sa tps_GetDevicePowerStatus ()
* @sa tps_GetSystemPortNumber ()
**************************************************************************************************************************************************/
uint8_t tps_GetDevicePowerGoodStatus (uint8_t device_i2c_address, TPS238x_Ports_t *powerGoodPorts)
{
uint8_t rtn;
uint8_t value;

    rtn = tps_ReadI2CReg (device_i2c_address, TPS238X_POWER_STATUS_COMMAND, &value);

    value = GET_POWER_GOOD_STATUS(value);
    *powerGoodPorts = value;

    return (rtn);
}

/*************************************************************************************************************************************************
*  tps_GetPortPowerGoodStatus
**************************************************************************************************************************************************/
/*!
* @brief Returns the power good status of one of the 4 ports in the TPS23861/TPS2388
*
* This function will return the power good state of the indicated port.
*
* @param[in]   systemPortNum          Port Number Value returned from the port registration function, tps_RegisterPort()
*
* @return  uint8_t     TPS_ON if Power Good
*                      TPS_OFF is Power is NOT good,
*                      TPS_ERR_I2C_ERROR on I2C error status
*
* @sa tps_GetDevicePowerGoodStatus ()
* @sa tps_GetDevicePowerEnableStatus ()
* @sa tps_GetPortPowerEnableStatus ()
* @sa tps_GetDevicePowerStatus ()
* @sa tps_GetSystemPortNumber ()
**************************************************************************************************************************************************/
uint8_t tps_GetPortPowerGoodStatus (uint8_t systemPortNum)
{
uint8_t rtn;
uint8_t value;
uint8_t portNum = tps_GetDevicePortNum(systemPortNum);

    rtn = tps_ReadI2CReg (tps_GetDeviceI2CAddress(systemPortNum), TPS238X_POWER_STATUS_COMMAND, &value);
    if (rtn != I2C_SUCCESSFUL)
        return (TPS_ERR_I2C_ERROR);

    value = GET_POWER_GOOD_STATUS(value);
    if (value & CONVERT_PORT_NUM(portNum))
        return (TPS_ON);
    else
        return (TPS_OFF);

}

/**************************************************************************************************************************************************
*                                 System Configuration Functions                                                                                                *
**************************************************************************************************************************************************/
/*************************************************************************************************************************************************
*  tps_SetDeviceOpMode
**************************************************************************************************************************************************/
/*!
* @brief Set the Operating Mode for all of the ports on the TPS23861/TPS2388
*
* The TPS23861/TPS2388 has a number of operating modes that determine the characteristic capabilities of a given port. This function allows the user to configure the
* operating for all of the ports.
*
* @param[in]   device_i2c_address        7 bit I2C address of the TPS238x part being controlled (do not included R/W bit)
* @param[in]   operatingMode1            Define the operating mode for port 1
*                                            [OPERATING_MODE_OFF, OPERATING_MODE_MANUAL, OPERATING_MODE_SEMI_AUTO, or OPERATING_MODE_AUTO]
* @param[in]   operatingMode2            Define the operating mode for port 2
*                                            [OPERATING_MODE_OFF, OPERATING_MODE_MANUAL, OPERATING_MODE_SEMI_AUTO, or OPERATING_MODE_AUTO]
* @param[in]   operatingMode3            Define the operating mode for port 3
*                                            [OPERATING_MODE_OFF, OPERATING_MODE_MANUAL, OPERATING_MODE_SEMI_AUTO, or OPERATING_MODE_AUTO]
* @param[in]   operatingMode4            Define the operating mode for port 4
*                                            [OPERATING_MODE_OFF, OPERATING_MODE_MANUAL, OPERATING_MODE_SEMI_AUTO, or OPERATING_MODE_AUTO]
*
* @return  uint8_t    (I2C_SUCCESSFUL or I2C error status)
*
* @sa tps_SetPortOpMode ()
* @sa tps_ConfigPort ()
**************************************************************************************************************************************************/
uint8_t tps_SetDeviceOperatingMode (uint8_t device_i2c_address, TPS238x_Operating_Modes_t operatingMode1, TPS238x_Operating_Modes_t operatingMode2,
                             TPS238x_Operating_Modes_t operatingMode3, TPS238x_Operating_Modes_t operatingMode4)
{
uint8_t rtn;
uint8_t value;

    // Load the Operating Mode
    value = (((uint8_t)operatingMode4 << 6) | ((uint8_t)operatingMode3 << 4) | ((uint8_t)operatingMode2 << 2) | ((uint8_t)operatingMode1));             // Each port gets 2 bits
    rtn = tps_WriteI2CReg (device_i2c_address, TPS238X_OPERATING_MODE_COMMAND, value);

    return (rtn);
}


/*************************************************************************************************************************************************
*  tps_SetPortOpMode
**************************************************************************************************************************************************/
/*!
* @brief Set the Operating Mode of a single port
*
* The TPS23861/TPS2388 has a number of operating modes that determine the characteristic capabilities of a given port. This function allows the user to configure the
* operating for a given port.
*
* @param[in]   systemPortNum             Port Number Value returned from the port registration function, tps_RegisterPort()
* @param[in]   operatingMode             Define the operating mode for the port
*                                            [OPERATING_MODE_OFF, OPERATING_MODE_MANUAL, OPERATING_MODE_SEMI_AUTO, or OPERATING_MODE_AUTO]
*
* @return  uint8_t    (I2C_SUCCESSFUL or I2C error status)
*
* @sa tps_SetDeviceOpMode ()
* @sa tps_ConfigPort ()
**************************************************************************************************************************************************/
uint8_t tps_SetPortOperatingMode (uint8_t systemPortNum, TPS238x_Operating_Modes_t operatingMode)
{
uint8_t rtn;
uint8_t value;
uint8_t portNum = tps_GetDevicePortNum(systemPortNum);

    // Load the Operating Mode
    rtn = tps_ReadI2CReg (tps_GetDeviceI2CAddress(systemPortNum), TPS238X_OPERATING_MODE_COMMAND, &value);
    value &= ~(OPERATING_MODE_MASK << (2 * ((uint8_t) portNum - 1)));               // Clear old value for this port number
    value |= ((uint8_t)operatingMode << (2 * ((uint8_t) portNum - 1)));             // Each port gets 2 bits
    rtn += tps_WriteI2CReg (tps_GetDeviceI2CAddress(systemPortNum), TPS238X_OPERATING_MODE_COMMAND, value);

    return (rtn);
}


/*************************************************************************************************************************************************
*  tps_SetDeviceDisconnectEnable
**************************************************************************************************************************************************/
/*!
* @brief Configures the ports to configure those that are enabled for 2 pair disconnection operation
*
* This function will configure all four ports to identify which ones support disconnection operations.
* If the user wishes to configure a single port, the tps_SetPortDisconnectEnable() is used.
*
* Disconnect operation consists of measuring the port current at the SENn pin, starting the TDIS timer when the current drops below the threshold,
* and turning the port off if the timer times out.
*
* @param[in]   device_i2c_address        7 bit I2C address of the TPS238x part being controlled (do not included R/W bit)
* @param[in]   disconnectPorts           A TPS238x_Ports_t variable that contains the ports that will be enabled for 2 pair
*                                            disconnection with one bit for each port.
*
* @return  uint8_t    (I2C_SUCCESSFUL or I2C error status)
*
* @sa tps_SetPortDisconnectEnable ()
* @sa tps_GetDeviceDisconnectEnable ()
**************************************************************************************************************************************************/
uint8_t tps_SetDeviceDisconnectEnable (uint8_t device_i2c_address, TPS238x_Ports_t disconnectPorts)
{
uint8_t rtn;

    rtn = tps_WriteI2CReg (device_i2c_address, TPS238X_DISCONNECT_ENABLE_COMMAND, *(uint8_t*)&disconnectPorts);

    return (rtn);
}

/*************************************************************************************************************************************************
*  tps_GetDeviceDisconnectEnable
**************************************************************************************************************************************************/
/*!
* @brief Returns the ports that are enabled for 2 pair disconnection operation
*
* This function will return the ports that are configured for 2 pair disconnect operations. Disconnect operation consists of measuring the port
* current at the SENn pin, starting the TDIS timer when the current drops below the threshold, and turning the port off if the timer times out.
*
* @param[in]   device_i2c_address        7 bit I2C address of the TPS238x part being controlled (do not included R/W bit)
* @param[out]  *disconnectPorts          Address of a TPS238x_Ports_t variable that will receive the ports that are enabled for 2 pair
*                                            disconnection with one bit for each port in disconnect enabled state.
*
* @return  uint8_t    (I2C_SUCCESSFUL or I2C error status)
*
* @sa tps_SetPortDisconnectEnable ()
* @sa tps_SetDeviceDisconnectEnable ()
* @sa tps_GetSystemPortNumber ()
**************************************************************************************************************************************************/
uint8_t tps_GetDeviceDisconnectEnable (uint8_t device_i2c_address, TPS238x_Ports_t *disconnectPorts)
{
uint8_t rtn;
uint8_t value;

    rtn = tps_ReadI2CReg (device_i2c_address, TPS238X_DISCONNECT_ENABLE_COMMAND, &value);

    *disconnectPorts = value;

    return (rtn);
}

/*************************************************************************************************************************************************
*  tps_SetPortDisconnectEnable
**************************************************************************************************************************************************/
/*!
* @brief Configures a single port to enable or disable the 2 pair disconnection operation
*
* This function will configure one of the four ports to enable or disable disconnection operations.
* If the user wishes to configure a all ports at once, the tps_SetDisconnectEnable() is used.
*
* Disconnect operation consists of measuring the port current at the SENn pin, starting the TDIS timer when the current drops below the threshold,
* and turning the port off if the timer times out.
*
* @param[in]   systemPortNum             Port Number Value returned from the port registration function, tps_RegisterPort()
* @param[in]   on_off                    TPS_ON to enable disconnection operations for the indicated port, TPS_OFF otherwise
* @param[in]   disconnectThreshold       Disconnection threshold for indicated port [DCTH_7_5_MILLIAMP, DCTH_15_MILLIAMP,
*                                                                                    DCTH_30_MILLIAMP or DCTH_50_MILLIAMP]
*
* @return  uint8_t    (I2C_SUCCESSFUL or I2C error status)
*
* @sa tps_SetDeviceDisconnectEnable ()
* @sa tps_GetDeviceDisconnectEnable ()
**************************************************************************************************************************************************/
uint8_t tps_SetPortDisconnectEnable (uint8_t systemPortNum, TPS238x_On_Off_t on_off)
{
uint8_t rtn;
uint8_t disconnectValue;
uint8_t portNum = tps_GetDevicePortNum(systemPortNum);
uint16_t i2cAddress = tps_GetDeviceI2CAddress(systemPortNum);

    rtn = tps_ReadI2CReg (i2cAddress, TPS238X_DISCONNECT_ENABLE_COMMAND, &disconnectValue);

    if (on_off == TPS_ON)
    {
        disconnectValue |= CONVERT_PORT_NUM(portNum);              // Set the disconnect enable bit
    }
    else
        disconnectValue &= ~(CONVERT_PORT_NUM(portNum));           // Clear the disconnect enable bit

    rtn += tps_WriteI2CReg (i2cAddress, TPS238X_DISCONNECT_ENABLE_COMMAND, disconnectValue);

    return (rtn);
}

/*************************************************************************************************************************************************
*  tps_SetDeviceDetectClassEnable
**************************************************************************************************************************************************/
/*!
* @brief Configures the device ports that are enabled for detection and classification operation for a specified TPS23861/TPS2388
*
* This function will set the detection and classification operations for all of the ports on a specific TPS23861/TPS2388.
*
* @param[in]   device_i2c_address        7 bit I2C address of the TPS238x part being controlled (do not included R/W bit)
* @param[in]   detectPorts               A TPS238x_Ports_t variable that identifies the ports that will be enabled for detection
*                                            with one bit for each port in detection enabled state.
* @param[in]   classPorts                A TPS238x_Ports_t variable that identifies the ports that will be enabled for classification
*                                            with one bit for each port in classification enabled state.
*
* @return  uint8_t    (I2C_SUCCESSFUL or I2C error status)
*
* @sa tps_SetPortDetectClassEnable ()
* @sa tps_GetDeviceClassificationEnable ()
* @sa tps_GetDeviceDetectionEnable ()
* @sa tps_GetDeviceDetectClassEnable ()
**************************************************************************************************************************************************/
uint8_t tps_SetDeviceDetectClassEnable (uint8_t device_i2c_address, TPS238x_Ports_t detectPorts, TPS238x_Ports_t classPorts)
{
uint8_t rtn;
uint8_t value;

    value = (*(uint8_t*)&classPorts << CLASS_SHIFT) | *(uint8_t*)&detectPorts;
    rtn = tps_WriteI2CReg (device_i2c_address, TPS238X_DETECT_CLASS_ENABLE_COMMAND, value);

    return (rtn);
}

/*************************************************************************************************************************************************
*  tps_GetDeviceDetectClassEnable
**************************************************************************************************************************************************/
/*!
* @brief Returns the device ports that are enabled for detection and classification operation on the specified TPS23861/TPS2388.
*
* This function will return the device ports that are configured for detection and classification operations.
*
* @param[in]   device_i2c_address        7 bit I2C address of the TPS238x part being controlled (do not included R/W bit)
* @param[out]  *detectPorts              Address of a TPS238x_Ports_t variable that will receive the ports that are enabled for detection
*                                            with one bit for each port in detection enabled state.
* @param[out]  *classPorts               Address of a TPS238x_Ports_t variable that will receive the ports that are enabled for classification
*                                            with one bit for each port in classification enabled state.
*
* @return  uint8_t    (I2C_SUCCESSFUL or I2C error status)
*
* @sa tps_GetDeviceClassificationEnable ()
* @sa tps_GetDeviceDetectionEnable ()
* @sa tps_SetDeviceDetectClassEnable ()
* @sa tps_GetSystemPortNumber ()
**************************************************************************************************************************************************/
uint8_t tps_GetDeviceDetectClassEnable (uint8_t device_i2c_address, TPS238x_Ports_t *detectPorts, TPS238x_Ports_t *classPorts)
{
uint8_t rtn;
uint8_t value;

    rtn = tps_ReadI2CReg (device_i2c_address, TPS238X_DETECT_CLASS_ENABLE_COMMAND, &value);

    *(uint8_t *)detectPorts = GET_DETECT(value);
    *(uint8_t *)classPorts  = GET_CLASS(value);

    return (rtn);
}

/*************************************************************************************************************************************************
*  tps_SetPortDetectClassEnable
**************************************************************************************************************************************************/
/*!
* @brief Configures a single port to enable or disable the detection and classification operation
*
* This function will set the detection and classification operations for the selected port of the TPS23861/TPS2388.
*
* @param[in]   systemPortNum             Port Number Value returned from the port registration function, tps_RegisterPort()
* @param[in]   on_off_detect             TPS_ON to enable detection operations for the indicated port, TPS_OFF otherwise
* @param[in]   on_off_class              TPS_ON to enable classification operations for the indicated port, TPS_OFF otherwise
*
* @return  uint8_t    (I2C_SUCCESSFUL or I2C error status)
*
* @sa tps_SetDeviceDetectClassEnable ()
* @sa tps_SetPortDetectClassEnable ()
* @sa tps_GetPortClassificationEnable ()
* @sa tps_GetPortDetectionEnable ()
**************************************************************************************************************************************************/
uint8_t tps_SetPortDetectClassEnable (uint8_t systemPortNum, TPS238x_On_Off_t on_off_detect, TPS238x_On_Off_t on_off_class)
{
uint8_t rtn;
uint8_t value;
uint8_t portNum = tps_GetDevicePortNum(systemPortNum);

    rtn = tps_ReadI2CReg (tps_GetDeviceI2CAddress(systemPortNum), TPS238X_DETECT_CLASS_ENABLE_COMMAND, &value);

    if (on_off_class == TPS_ON)
        value |= (CONVERT_PORT_NUM(portNum) << CLASS_SHIFT);       // Set the classification enable bit
    else
        value &= ~(CONVERT_PORT_NUM(portNum) << CLASS_SHIFT);      // Clear the classification enable bit

    if (on_off_detect == TPS_ON)
        value |= (CONVERT_PORT_NUM(portNum));                      // Set the detection enable bit
    else
        value &= ~(CONVERT_PORT_NUM(portNum));                     // Clear the detection enable bit

    rtn += tps_WriteI2CReg (tps_GetDeviceI2CAddress(systemPortNum), TPS238X_DETECT_CLASS_ENABLE_COMMAND, value);

    return (rtn);
}


/*************************************************************************************************************************************************
*  tps_GetDeviceDetectionEnable
**************************************************************************************************************************************************/
/*!
* @brief Returns the device ports that are enabled for detection operation on the specific TPS23861/TPS2388
*
* This function will return the ports that are configured for detection operations.
*
* @param[in]   device_i2c_address        7 bit I2C address of the TPS238x part being controlled (do not included R/W bit)
* @param[out]  *detectPorts              Address of a TPS238x_Ports_t variable that will receive the device ports that are enabled for detection
*                                            with one bit for each port in detection enabled state.
*
* @return  uint8_t    (I2C_SUCCESSFUL or I2C error status)
*
* @sa tps_GetPortDetectionEnable ()
* @sa tps_GetDeviceClassificationEnable ()
* @sa tps_GetDeviceDetectClassEnable ()
* @sa tps_SetDeviceDetectClassEnable ()
* @sa tps_GetSystemPortNumber ()
**************************************************************************************************************************************************/
uint8_t tps_GetDeviceDetectionEnable (uint8_t device_i2c_address, TPS238x_Ports_t *detectPorts)
{
uint8_t rtn;
uint8_t value;

    rtn = tps_ReadI2CReg (device_i2c_address, TPS238X_DETECT_CLASS_ENABLE_COMMAND, &value);

    *(uint8_t *)detectPorts = GET_DETECT(value);

    return (rtn);
}

/*************************************************************************************************************************************************
*  tps_GetPortDetectionEnable
**************************************************************************************************************************************************/
/*!
* @brief Returns whether the indicated system port is enabled for detection operation
*
* This function will return a TPS_ON or TPS_OFF to indicate that the specific system port is enabled for detection or not
*
* @param[in]   systemPortNum             Port Number Value returned from the port registration function, tps_RegisterPort()
*
* @return  uint8_t    TPS_ON  - Port is enabled for detection
*                     TPS_OFF - Port is NOT enabled for detection
*
* @sa tps_GetDeviceDetectionEnable ()
* @sa tps_GetPortClassificationEnable ()
* @sa tps_SetPortDetectClassEnable ()
**************************************************************************************************************************************************/
uint8_t tps_GetPortDetectionEnable (uint8_t systemPortNum)
{
uint8_t value;
uint8_t portNum = tps_GetDevicePortNum(systemPortNum);

    tps_ReadI2CReg (tps_GetDeviceI2CAddress(systemPortNum), TPS238X_DETECT_CLASS_ENABLE_COMMAND, &value);

    value = GET_DETECT(value);

    if (value & CONVERT_PORT_NUM(portNum))
        return (TPS_ON);
    else
        return (TPS_OFF);

}


/*************************************************************************************************************************************************
*  tps_GetDeviceClassificationEnable
**************************************************************************************************************************************************/
/*!
* @brief Returns the device ports that are enabled for classification operation for a specific TPS23861/TPS2388 device
*
* This function will return the ports that are configured for classification operations.
*
* @param[in]   device_i2c_address        7 bit I2C address of the TPS238x part being controlled (do not included R/W bit)
* @param[out]  *classPorts               Address of a TPS238x_Ports_t variable that will receive the ports that are enabled for classification
*                                            with one bit for each port in classification enabled state.
*
* @return  uint8_t    (I2C_SUCCESSFUL or I2C error status)
*
* @sa tps_GetPortClassificationEnable ()
* @sa tps_GetDeviceDetectionEnable ()
* @sa tps_GetDeviceDetectClassEnable ()
* @sa tps_SetDeviceDetectClassEnable ()
* @sa tps_GetSystemPortNumber ()
**************************************************************************************************************************************************/
uint8_t tps_GetDeviceClassificationEnable (uint8_t device_i2c_address, TPS238x_Ports_t *classPorts)
{
uint8_t rtn;
uint8_t value;

    rtn = tps_ReadI2CReg (device_i2c_address, TPS238X_DETECT_CLASS_ENABLE_COMMAND, &value);

    *(uint8_t *)classPorts  = GET_CLASS(value);

    return (rtn);
}


/*************************************************************************************************************************************************
*  tps_GetPortClassificationEnable
**************************************************************************************************************************************************/
/*!
* @brief Returns whether the indicated system port is enabled for classification operation
*
* This function will return whether the indicated system port is configured for classification operations.
*
* @param[in]   systemPortNum             Port Number Value returned from the port registration function, tps_RegisterPort()
*
* @return  uint8_t    TPS_ON  - Port is enabled for classification
*                     TPS_OFF - Port is NOT enabled for classification
*
* @sa tps_GetDeviceClassificationEnable ()
* @sa tps_GetPortDetectionEnable ()
* @sa tps_SetPortDetectClassEnable ()
**************************************************************************************************************************************************/
uint8_t tps_GetPortClassificationEnable (uint8_t systemPortNum)
{
uint8_t value;
uint8_t portNum = tps_GetDevicePortNum(systemPortNum);

    tps_ReadI2CReg (tps_GetDeviceI2CAddress(systemPortNum), TPS238X_DETECT_CLASS_ENABLE_COMMAND, &value);

    value = GET_CLASS(value);

    if (value & CONVERT_PORT_NUM(portNum))
        return (TPS_ON);
    else
        return (TPS_OFF);
}



/*************************************************************************************************************************************************
*  tps_SetDevoiceOneBitOSS
**************************************************************************************************************************************************/
/*!
* @brief Configures the 1 bit OSS  on a specific TPS23880
*
* This function will enable/disable 1 bit OSS for TPS23880
*
* @param[in]   device_i2c_address        7 bit I2C address of the TPS238x part being controlled (do not included R/W bit)
* @param[in]   portsOSS            A TPS238x_Ports_t variable that identifies the ports that will be configured for 1 bit OSS.
*
* @return  uint8_t    (I2C_SUCCESSFUL or I2C error status)
*
**************************************************************************************************************************************************/
uint8_t tps_SetDevoiceOneBitOSS (uint8_t device_i2c_address, TPS238x_Ports_t portsOSS)
{
uint8_t rtn;
uint8_t value;

    value = *(uint8_t*)&portsOSS << OSS_SHIFT;
    rtn = tps_WriteI2CReg (device_i2c_address, TPS238X_PORT_POWER_PRIORITY_COMMAND, value);

    return (rtn);
}


/*************************************************************************************************************************************************
*  tps_SetDevoicePCUTDisable
**************************************************************************************************************************************************/
/*!
* @brief Disable PCUT for TPS23880
*
* This function will diable the PCUT for TPS23880
* @param[in]   device_i2c_address        7 bit I2C address of the TPS238x part being controlled (do not included R/W bit)
* @param[in]   portsPCUTDisable            A TPS238x_Ports_t variable that identifies the ports that will be powered on.
*
* @return  uint8_t    (I2C_SUCCESSFUL or I2C error status)
*

**************************************************************************************************************************************************/
uint8_t tps_SetDevoicePCUTDisable(uint8_t device_i2c_address,TPS238x_Ports_t portsPCUTDisable)
{
uint8_t rtn;

    rtn = tps_WriteI2CReg (device_i2c_address, TPS238X_PORT_POWER_PRIORITY_COMMAND, *(uint8_t*)&portsPCUTDisable);

    return (rtn);
}


/*************************************************************************************************************************************************
*  tps_SetDeviceTiming
**************************************************************************************************************************************************/
/*!
* @brief Configures the timing of the various current limits and disconnection determinations for a given device in the system.
*
* The TPS23861 has three levels of overcurrent protections, disconnection detection, and time out after fault conditions. In order to prevent
* glitches or transients from causing unnecessary shutdowns or faults, the system uses various timers to define the amount of time required
* over the threshold level before a fault or shutdown is declared.

* This function sets up the various timing parameters used in the system. These timings are applicable on all active ports.
*
* @param[in]   device_i2c_address        7 bit I2C address of the TPS238x part being controlled (do not included R/W bit)
* @param[in]   ilimTiming                The ILIM Fault Timing which is the foldback current time limit before port turn off
*                                             [TLIM_60_MS, TLIM_15_MS, TLIM_12_MS, or TLIM_10_MS]
* @param[in]   startTiming               The length of the TSTART period, which is the maximum amount of allowed overcurrent time during inrush
*                                             [TSTART_30_MS, TSTART_60_MS, or TSTART_120_MS]
* @param[in]   icutTiming                ICUT Fault timing, which is the overcurrent time duration before port turn off
*                                             [TICUT_30_MS, TICUT_60_MS, TICUT_120_MS, or TICUT_240_MS]
* @param[in]   disconnectTiming          Disconnect delay, which is the time to turn off a port once there is a disconnect condition
*                                             [TDIS_90_MS, TDIS_180_MS, TDIS_360_MS, or TDIS_720_MS]

*
* @return  uint8_t    (I2C_SUCCESSFUL or I2C error status)
*
* @sa tps_ConfigPort ()
* @sa tps_ConfigDevice4Pair ()
**************************************************************************************************************************************************/
uint8_t tps_SetDeviceTiming (uint8_t device_i2c_address, TPS238x_ILIM_Timing_t ilimTiming, TPS238x_TStart_Timing_t startTiming,
                             TPS238x_TICUT_Timing_t icutTiming, TPS238x_TDIS_Timing_t disconnectTiming)
{
uint8_t rtn;
uint8_t value;

    value = (ilimTiming << TLIM_SHIFT) | ( startTiming << TSTART_SHIFT) |
            (icutTiming << TICUT_SHIFT) | (disconnectTiming << TDIS_SHIFT);

    rtn = tps_WriteI2CReg (device_i2c_address, TPS238X_TIMING_CONFIGURATION_COMMAND, value);

    return (rtn);
}



/*************************************************************************************************************************************************
*  tps_SetDeviceGeneralMask
**************************************************************************************************************************************************/
/*!
* @brief Set the general interrupt mask register
*
* The function sets which TPS238x events/faults are allowed to generate interrupts (unmasked).
*
* @param[in]   device_i2c_address 7 bit I2C address of the TPS238x part being controlled (do not included R/W bit)
* @param[in]   genMask            TPS238x_General_Mask_1_Register_t variable that contains the conditions that are allowed to generate interrupts
*
*                                  INTEN
*                                  nbitACC
*                                  MbitPrty
*                                  CLCHE
*                                  DECHE
*
*
*
* @return  uint8_t    (I2C_SUCCESSFUL or I2C error status)
*
* @sa tps_GetDeviceGeneralMask ()
**************************************************************************************************************************************************/
uint8_t tps_SetDeviceGeneralMask(uint8_t device_i2c_address,TPS238x_General_Mask_1_Register_t genMask)
{

    uint8_t rtn;

    rtn = tps_WriteI2CReg (device_i2c_address, TPS238X_GENERAL_MASK_COMMAND, *(uint8_t*)&genMask);
    return (rtn);
}

/*************************************************************************************************************************************************
*  tps_GetDeviceGeneralMask
**************************************************************************************************************************************************/
/*!
* @brief Get the general interrupt mask register
*
*
* @param[in]   device_i2c_address 7 bit I2C address of the TPS238x part being controlled (do not included R/W bit)
* @param[out]   genMask            TPS238x_General_Mask_1_Register_t variable that contains the conditions that are allowed to generate interrupts
*
*                                  INTEN
*                                  nbitACC
*                                  MbitPrty
*                                  CLCHE
*                                  DECHE
*
*
*
* @return  uint8_t    (I2C_SUCCESSFUL or I2C error status)
*
* @sa tps_GetDeviceGeneralMask ()
**************************************************************************************************************************************************/

uint8_t tps_GetDeviceGeneralMask(uint8_t device_i2c_address,uint8_t *genMask)
{
     uint8_t rtn;

     rtn = tps_ReadI2CReg (device_i2c_address, TPS238X_GENERAL_MASK_COMMAND, genMask);

     return (rtn);
}


/*************************************************************************************************************************************************
*  tps_RestartDeviceDetection
**************************************************************************************************************************************************/
/*!
* @brief Forces a restart of the detection process on the indicated ports on a given TPS23861/TPS2388 in the system
*
* This function will restart the detection process on the indicated ports on the specified TPS23861/TPS2388.
*
* @param[in]   device_i2c_address        7 bit I2C address of the TPS238x part being controlled (do not included R/W bit)
* @param[in]   detectPorts               A TPS238x_Ports_t variable that identifies the ports that will be restarted for detection
*                                            with one bit for each port restarted.
*
* @return  uint8_t    (I2C_SUCCESSFUL or I2C error status)
*
* @sa tps_RestartPortDetection ()
* @sa tps_RestartDeviceClassification ()
* @sa tps_RestartDeviceDetectClass ()
* @sa tps_GetSystemPortNumber ()
**************************************************************************************************************************************************/
uint8_t tps_RestartDeviceDetection (uint8_t device_i2c_address, TPS238x_Ports_t detectPorts)
{
uint8_t rtn;
uint8_t value;

    value = *(uint8_t*)&detectPorts;
    rtn = tps_WriteI2CReg (device_i2c_address, TPS238X_DETECT_CLASS_RESTART_COMMAND, value);

    return (rtn);
}

/*************************************************************************************************************************************************
*  tps_RestartPortDetection
**************************************************************************************************************************************************/
/*!
* @brief Forces a restart of the detection process on the indicated device ports for a given TPS23861/TPS2388
*
* This function will restart the detection process on the indicated device ports on the given TPS23861/TPS2388.
*
* @param[in]   systemPortNum             Port Number Value returned from the port registration function, tps_RegisterPort()
*
* @return  uint8_t    (I2C_SUCCESSFUL or I2C error status)
*
* @sa tps_RestartPortClassification ()
* @sa tps_RestartDeviceDetection ()
* @sa tps_RestartDeviceDetection ()
**************************************************************************************************************************************************/
uint8_t tps_RestartPortDetection (uint8_t systemPortNum)
{
uint8_t rtn;
uint8_t value;
uint8_t portNum = tps_GetDevicePortNum(systemPortNum);

    value = CONVERT_PORT_NUM(portNum);

    // Load the Operating Mode
    rtn = tps_WriteI2CReg (tps_GetDeviceI2CAddress(systemPortNum), TPS238X_DETECT_CLASS_RESTART_COMMAND, value);

    return (rtn);
}


/*************************************************************************************************************************************************
*  tps_RestartDeviceClassification
**************************************************************************************************************************************************/
/*!
* @brief Forces a restart of the classification process on the indicated ports
*
* This function will restart the classification process on the indicated ports on the TPS23861/TPS2388.
*
* @param[in]   device_i2c_address        7 bit I2C address of the TPS238x part being controlled (do not included R/W bit)
* @param[in]   classPorts                A TPS238x_Ports_t variable that identifies the ports that will be restarted for classification
*                                            with one bit for each port restarted.
*
* @return  uint8_t    (I2C_SUCCESSFUL or I2C error status)
*
* @sa tps_RestartPortClassification ()
* @sa tps_RestartDeviceDetection ()
* @sa tps_RestartDeviceDetectClass ()
**************************************************************************************************************************************************/
uint8_t tps_RestartDeviceClassification (uint8_t device_i2c_address, TPS238x_Ports_t classPorts)
{
uint8_t rtn;
uint8_t value;

    value = (*(uint8_t*)&classPorts << CLASS_SHIFT);
    rtn = tps_WriteI2CReg (device_i2c_address, TPS238X_DETECT_CLASS_RESTART_COMMAND, value);

    return (rtn);
}

/*************************************************************************************************************************************************
*  tps_RestartPortClassification
**************************************************************************************************************************************************/
/*!
* @brief Forces a restart of the classification process on the single indicated port
*
* This function will restart the classification process on the specified port on the TPS23861/TPS2388.
*
* @param[in]   systemPortNum             Port Number Value returned from the port registration function, tps_RegisterPort()
*
* @return  uint8_t    (I2C_SUCCESSFUL or I2C error status)
*
* @sa tps_RestartDeviceClassification ()
* @sa tps_RestartPortDetection ()
**************************************************************************************************************************************************/
uint8_t tps_RestartPortClassification (uint8_t systemPortNum)
{
uint8_t rtn;
uint8_t value;
uint8_t portNum = tps_GetDevicePortNum(systemPortNum);

    value = (CONVERT_PORT_NUM(portNum) << CLASS_SHIFT);

    // Load the Operating Mode
    rtn = tps_WriteI2CReg (tps_GetDeviceI2CAddress(systemPortNum), TPS238X_DETECT_CLASS_RESTART_COMMAND, value);

    return (rtn);
}

/*************************************************************************************************************************************************
*  tps_RestartDeviceDetectClass
**************************************************************************************************************************************************/
/*!
* @brief Forces a restart both the classification and detection processes on the indicated ports
*
* This function will restart both the classification and detection process on the indicated ports on the TPS23861/TPS2388.
*
* @param[in]   device_i2c_address        7 bit I2C address of the TPS238x part being controlled (do not included R/W bit)
* @param[in]   detectPorts               A TPS238x_Ports_t variable that identifies the ports that will be restarted for detection
*                                            with one bit for each port restarted.
* @param[in]   classPorts                A TPS238x_Ports_t variable that identifies the ports that will be restarted for classification
*                                            with one bit for each port restarted.
*
* @return  uint8_t    (I2C_SUCCESSFUL or I2C error status)
*
* @sa tps_RestartDeviceClassification ()
* @sa tps_RestartDeviceDetection ()
**************************************************************************************************************************************************/
uint8_t tps_RestartDeviceDetectClass (uint8_t device_i2c_address, TPS238x_Ports_t detectPorts, TPS238x_Ports_t classPorts)
{
uint8_t rtn;
uint8_t value;

    value = (*(uint8_t *)&classPorts << CLASS_SHIFT) | *(uint8_t *)&detectPorts;
    rtn = tps_WriteI2CReg (device_i2c_address, TPS238X_DETECT_CLASS_RESTART_COMMAND, value);

    return (rtn);
}

/*************************************************************************************************************************************************
*  tps_SetDevicePowerOn
**************************************************************************************************************************************************/
/*!
* @brief Configures the power on state for each of the device ports on a specific TPS23861/TPS2388
*
* This function will power on multiple ports at the same time. Each of the ports indicated in the TPS23861/TPS2388 will be powered on.
*
* @param[in]   device_i2c_address        7 bit I2C address of the TPS238x part being controlled (do not included R/W bit)
* @param[in]   portsPoweredOn            A TPS238x_Ports_t variable that identifies the ports that will be powered on.
*
* @return  uint8_t    (I2C_SUCCESSFUL or I2C error status)
*
* @sa tps_SetDevicePowerOff ()
* @sa tps_SetPortPower ()
* @sa tps_GetSystemPortNumber ()
**************************************************************************************************************************************************/
uint8_t tps_SetDevicePowerOn (uint8_t device_i2c_address, TPS238x_Ports_t portsPoweredOn)
{
uint8_t rtn;

    rtn = tps_WriteI2CReg (device_i2c_address, TPS238X_POWER_ENABLE_COMMAND, *(uint8_t*)&portsPoweredOn);

    return (rtn);
}

/*************************************************************************************************************************************************
*  tps_SetDevicePowerOff
**************************************************************************************************************************************************/
/*!
* @brief Configures the power down (off) state for each of the device ports on a specific TPS23861/TPS2388
*
* This function will power down (off) multiple ports at the same time. Each of the ports indicated in the TPS23861/TPS2388 will be powered off.
*
* @param[in]   device_i2c_address        7 bit I2C address of the TPS238x part being controlled (do not included R/W bit)
* @param[in]   portsPoweredOff           A TPS238x_Ports_t variable that identifies the ports that will be powered off.
*
* @return  uint8_t    (I2C_SUCCESSFUL or I2C error status)
*
* @sa tps_SetDevicePowerOn ()
* @sa tps_SetPortPower ()
* @sa tps_GetSystemPortNumber ()
**************************************************************************************************************************************************/
uint8_t tps_SetDevicePowerOff (uint8_t device_i2c_address, TPS238x_Ports_t portsPoweredOff)
{
uint8_t rtn;
uint8_t value;

//    value = *(uint8_t*)&portsPoweredOff << POWER_OFF_SHIFT;
    value = *(uint8_t *)&portsPoweredOff;
    value = (value << POWER_OFF_SHIFT) & 0xF0;
    rtn = tps_WriteI2CReg (device_i2c_address, TPS238X_POWER_ENABLE_COMMAND, value);

    return (rtn);
}

/*************************************************************************************************************************************************
*  tps_SetPortPower
**************************************************************************************************************************************************/
/*!
* @brief Power on or off a single specified system port
*
* This function will either power on or power down (off) a single port on one of the registered TPS23861/TPS2388 ports in the system.
*
* @param[in]   systemPortNum          Port Number Value returned from the port registration function, tps_RegisterPort()
* @param[in]   on_off                 Indicates whether to power on (TPS_ON) or power off (TPS_OFF) the specified port
*
* @return  uint8_t    (I2C_SUCCESSFUL or I2C error status)
*
* @sa tps_SetDevicePowerOn ()
* @sa tps_SetDevicePowerOff ()
**************************************************************************************************************************************************/
uint8_t tps_SetPortPower (uint8_t systemPortNum, TPS238x_On_Off_t on_off)
{
uint8_t rtn;
uint8_t value;
uint8_t portBit = tps_GetDevicePortNum(systemPortNum);

    portBit = CONVERT_PORT_NUM(portBit);

    // This TPS register is Write Only with bits for forcing a Power Off and separate bits for Power On
    if (on_off == TPS_OFF)
        value = (portBit << POWER_OFF_SHIFT);
    else
        value = (portBit << POWER_ON_SHIFT);

    rtn = tps_WriteI2CReg (tps_GetDeviceI2CAddress(systemPortNum), TPS238X_POWER_ENABLE_COMMAND, value);

    return (rtn);
}

/*************************************************************************************************************************************************
*  tps_ResetDevicePort
**************************************************************************************************************************************************/
/*!
* @brief Forces a reset of the indicated device ports on the indicated TPS23861/TPS2388 device.
*
* This function will reset the indicated device ports on the identified TPS23861/TPS2388.
*
* @param[in]   systemPortNum             Port Number Value returned from the port registration function, tps_RegisterPort()
*
* @return  uint8_t    (I2C_SUCCESSFUL or I2C error status)
*
* @sa tps_ResetPort ()
**************************************************************************************************************************************************/
uint8_t tps_ResetDevicePort (uint8_t systemPortNum)
{
uint8_t rtn;
uint8_t value;
uint8_t portNum = tps_GetDevicePortNum(systemPortNum);

    value = CONVERT_PORT_NUM(portNum);

    rtn = tps_WriteI2CReg (tps_GetDeviceI2CAddress(systemPortNum), TPS238X_RESET_COMMAND, value);

    return (rtn);
}

/*************************************************************************************************************************************************
*  tps_ResetPort
**************************************************************************************************************************************************/
/*!
* @brief Forces a reset of the single specified system port number identifying a single port on a specific registered TPS23861/TPS2388.
*
* This function will reset the single specified port registered with the system for a given TPS23861/TPS2388.
*
* @param[in]   systemPortNum             Port Number Value returned from the port registration function, tps_RegisterPort()
*
* @return  uint8_t    (I2C_SUCCESSFUL or I2C error status)
*
* @sa tps_ResetDevicePort ()
**************************************************************************************************************************************************/
uint8_t tps_ResetPort (uint8_t systemPortNum)
{
uint8_t rtn;
uint8_t value;
uint8_t portNum = tps_GetDevicePortNum(systemPortNum);

    value = CONVERT_PORT_NUM(portNum);

    // Load the Operating Mode
    rtn = tps_WriteI2CReg (tps_GetDeviceI2CAddress(systemPortNum), TPS238X_RESET_COMMAND, value);

    return (rtn);
}



/*************************************************************************************************************************************************
*  tps_GetID
**************************************************************************************************************************************************/
/*!
* @brief Get manufacture ID and IC version
*
* This function will get manufacture ID and IC version for a given TPS238x.
*
* @param[in]   device_i2c_address             7 bit I2C address of the TPS238x part being controlled (do not included R/W bit)
* @param[in]   TPS238x_ID_Register_t variable that contains the IC versiona and MFR ID
*
* @return  uint8_t    (I2C_SUCCESSFUL or I2C error status)
*

**************************************************************************************************************************************************/

uint8_t tps_GetID(uint8_t device_i2c_address,uint8_t *ICVersion,uint8_t *MFRID)
{
uint8_t rtn;
uint8_t value;

    rtn = tps_ReadI2CReg (device_i2c_address, TPS238X_ID_COMMAND, &value);
    *ICVersion = value & ICV_MASK;
    *MFRID = value >>MFR_ID_SHIFT;

    return (rtn);

}

/*************************************************************************************************************************************************
*  tps_GetDeviceAutoClassStatus
**************************************************************************************************************************************************/
/*!
* @brief Get device autoclass status
*
* This function will get device autoclass status TPS238x.
*
* @param[in]   device_i2c_address             7 bit I2C address of the TPS238x part being controlled (do not included R/W bit)
* @param[in]   TPS238x_Ports_t variable that contains the autoclass status
*
* @return  uint8_t    (I2C_SUCCESSFUL or I2C error status)
*

**************************************************************************************************************************************************/


uint8_t tps_GetDeviceAutoClassStatus(uint8_t device_i2c_address, TPS238x_Ports_t *autoclassports)
{
uint8_t rtn;
uint8_t value;

    rtn = tps_ReadI2CReg (device_i2c_address, TPS238X_CONNECTIONCHECK_AUTOCLASS_COMMAND, &value);
    *autoclassports = value >> AUTOCLASS_SHIFT;

    return (rtn);
}


/*************************************************************************************************************************************************
*  tps_GetPortAutoClassStatus
**************************************************************************************************************************************************/
/*!
* @brief Get device autoclass status
*
* This function will get device autoclass status TPS238x.
*
* @param[in]   systemPortNum             Port Number Value returned from the port registration function, tps_RegisterPort()

*
* @return  uint8_t    TPS_ON   - Autoclass supported
*                     TPS_OFF  - Autoclass not supported

**************************************************************************************************************************************************/

uint8_t tps_GetPortAutoClassStatus(uint8_t systemPortNum, TPS238x_On_Off_t *on_off_autoclass)
{
uint8_t rtn;
uint8_t value;
uint8_t portNum = tps_GetDevicePortNum(systemPortNum);

    rtn = tps_ReadI2CReg (tps_GetDeviceI2CAddress(systemPortNum), TPS238X_CONNECTIONCHECK_AUTOCLASS_COMMAND, &value);
    value = value >> AUTOCLASS_SHIFT;

    if (value & CONVERT_PORT_NUM(portNum))
        *on_off_autoclass = TPS_ON;
    else
        *on_off_autoclass = TPS_OFF;
    return(rtn);
}


/*************************************************************************************************************************************************
*  tps_GetDeviceAutoClassStatus
**************************************************************************************************************************************************/
/*!
* @brief Get device autoclass status
*
* This function will get device autoclass status TPS238x.
*
* @param[in]   device_i2c_address             7 bit I2C address of the TPS238x part being controlled (do not included R/W bit)
* @param[in]   TPS238x_Ports_t variable that contains the autoclass status
*
* @return  uint8_t    (I2C_SUCCESSFUL or I2C error status)
*

**************************************************************************************************************************************************/
uint8_t tps_GetDeviceConnectionCheckStatus(uint8_t device_i2c_address, TPS238x_Ports_t *connectioncheck)
{
uint8_t rtn;
uint8_t value;

    rtn = tps_ReadI2CReg (device_i2c_address, TPS238X_CONNECTIONCHECK_AUTOCLASS_COMMAND, &value);
    *connectioncheck = value & CONNECTIONCHECK_MASK;
    return (rtn);
}


/*************************************************************************************************************************************************
*  tps_GetPortConnectionCheckStatus
**************************************************************************************************************************************************/
/*!
* @brief Get device autoclass status
*
* This function will get device autoclass status TPS238x.
*
* @param[in]   systemPortNum             Port Number Value returned from the port registration function, tps_RegisterPort()
*  @param[in]  TPS238x_Connection_Check_Status_t           variable that contains the connection check status
*
* @return   uint8_t    (I2C_SUCCESSFUL or I2C error status)

**************************************************************************************************************************************************/


uint8_t tps_GetPortConnectionCheckStatus(uint8_t systemPortNum, uint8_t *connectioncheck)
{
uint8_t rtn;
uint8_t value;
uint8_t portNum = tps_GetDevicePortNum(systemPortNum);

    rtn = tps_ReadI2CReg (tps_GetDeviceI2CAddress(systemPortNum), TPS238X_CONNECTIONCHECK_AUTOCLASS_COMMAND, &value);
    if(portNum == TPS238X_PORT_1 | portNum == TPS238X_PORT_2)
    {
        *connectioncheck = *(TPS238x_Connection_Check_Status_t*)(value & CC12);
    }

    else
    {
        *connectioncheck = *(TPS238x_Connection_Check_Status_t*)(value & CC34);
    }

    return (rtn);
}

/*************************************************************************************************************************************************
*  tps_SetPort2PPolicing
**************************************************************************************************************************************************/
/*!
* @brief Set port 2P policing threshold
*
* This function will set port 2P policing threshold of TPS238x.
*
* @param[in]   systemPortNum             Port Number Value returned from the port registration function, tps_RegisterPort()
* @param[in]  TPS238x_2P_Policing_config_Register_t           variable that contains the 2P policing threshold
*
* @return   uint8_t    (I2C_SUCCESSFUL or I2C error status)

**************************************************************************************************************************************************/


uint8_t tps_SetPort2PPolicing(uint8_t systemPortNum,TPS238x_2P_Policing_config_Register_t icutCurrentThreshold)
{
uint8_t rtn;
uint8_t value;
uint8_t portNum = tps_GetDevicePortNum(systemPortNum);

    value = *(uint8_t*)&icutCurrentThreshold;
    rtn = tps_WriteI2CReg (tps_GetDeviceI2CAddress(systemPortNum), TPS238X_2P_POLICE_1_CONFIG_COMMAND+((uint8_t)portNum - 1), value);
    return (rtn);
}

/*************************************************************************************************************************************************
*  tps_GetPort2PPolicing
**************************************************************************************************************************************************/
/*!
* @brief Get port 2P policing threshold
*
* This function will get port 2P policing threshold of TPS238x.
*
* @param[in]  systemPortNum             Port Number Value returned from the port registration function, tps_RegisterPort()
* @param[in]  TPS238x_2P_Policing_config_Register_t           variable that contains the 2P policing threshold
*
* @return   uint8_t    (I2C_SUCCESSFUL or I2C error status)

**************************************************************************************************************************************************/


uint8_t tps_GetPort2PPolicing(uint8_t systemPortNum,TPS238x_2P_Policing_config_Register_t *icutCurrentThreshold)
{
uint8_t rtn;
uint8_t value;
uint8_t portNum = tps_GetDevicePortNum(systemPortNum);

    rtn = tps_ReadI2CReg (tps_GetDeviceI2CAddress(systemPortNum), TPS238X_2P_POLICE_1_CONFIG_COMMAND+((uint8_t)portNum - 1), &value);
    *icutCurrentThreshold = *(TPS238x_2P_Policing_config_Register_t*)value;
    return (rtn);
}


/*************************************************************************************************************************************************
*  tps_SetDeviceLegacyDetection
**************************************************************************************************************************************************/
/*!
* @brief Set the legacy detection for all of the channel on the TPS238x
*
*  This function allows the user to configure the legacy detection for all of the channels.
*
* @param[in]   device_i2c_address        7 bit I2C address of the TPS238x part being controlled (do not included R/W bit)
* @param[in]   legacydetect1            Define the legacy detection for channel 1
*
* @param[in]   legacydetect2            Define the legacy detection for channel 2
*
* @param[in]   legacydetect3            Define the legacy detection for channel 3
*
* @param[in]   legacydetect4            Define the legacy detection for channel 4
*
*
* @return  uint8_t    (I2C_SUCCESSFUL or I2C error status)
*
**************************************************************************************************************************************************/

uint8_t tps_SetDeviceLegacyDetection(uint8_t device_i2c_address, TPS238x_Legacy_Detect_t legacydetect1, TPS238x_Legacy_Detect_t legacydetect2,TPS238x_Legacy_Detect_t legacydetect3,TPS238x_Legacy_Detect_t legacydetect4)
{
uint8_t rtn;
uint8_t value;

    value = (((uint8_t)legacydetect4 << 6) | ((uint8_t)legacydetect3 << 4) | ((uint8_t)legacydetect2 << 2) | ((uint8_t)legacydetect1));
    rtn = tps_WriteI2CReg (device_i2c_address,TPS238X_LEGACY_DETECTION_COMMAND , value);
    return (rtn);
}

/*************************************************************************************************************************************************
*  tps_SetPortLegacyDetection
**************************************************************************************************************************************************/
/*!
* @brief Set the tps_SetPortLegacyDetection of a single port
*
*This function allows the user to configure the legacy detection for a given channel.
*
* @param[in]   systemPortNum             Port Number Value returned from the port registration function, tps_RegisterPort()
* @param[in]   legacydetect               Define the legacy detection for the channel
*
*
* @return  uint8_t    (I2C_SUCCESSFUL or I2C error status)
*
**************************************************************************************************************************************************/
uint8_t tps_SetPortLegacyDetection (uint8_t systemPortNum, TPS238x_Legacy_Detect_t legacydetect)
{
uint8_t rtn;
uint8_t value;
uint8_t portNum = tps_GetDevicePortNum(systemPortNum);

    // Load the Operating Mode
    rtn = tps_ReadI2CReg (tps_GetDeviceI2CAddress(systemPortNum), TPS238X_LEGACY_DETECTION_COMMAND, &value);
    value &= ~(LEGACY_DETECT_MASK << (2 * ((uint8_t) portNum - 1)));               // Clear old value for this port number
    value |= ((uint8_t)legacydetect << (2 * ((uint8_t) portNum - 1)));             // Each port gets 2 bits
    rtn += tps_WriteI2CReg (tps_GetDeviceI2CAddress(systemPortNum), TPS238X_LEGACY_DETECTION_COMMAND, value);

    return (rtn);
}



/*************************************************************************************************************************************************
*  tps_GetDevicePowerOnFault
**************************************************************************************************************************************************/
/*!
* @brief Get power on faluts of all channels
*
*  This function allows the user to get power on faluts of all channels.
*
* @param[in]   device_i2c_address        7 bit I2C address of the TPS238x part being controlled (do not included R/W bit)
* @param[in]   clearEvent                CoR register
* @param[in]   poweronfaultports         power on fault status

*
*
* @return  uint8_t    (I2C_SUCCESSFUL or I2C error status)
*
**************************************************************************************************************************************************/


uint8_t tps_GetDevicePowerOnFault (uint8_t device_i2c_address, TPS238x_On_Off_t clearEvent,TPS238x_Power_On_Fault_t *poweronfaultports)
{
uint8_t rtn;
uint8_t value;

    rtn = tps_ReadI2CReg (device_i2c_address, TPS238X_POWER_ON_FAULT_EVENT_COMMAND + clearEvent, &value);
    *poweronfaultports = (TPS238x_Power_On_Fault_t)value;
    return (rtn);
}

/*************************************************************************************************************************************************
*  tps_SetDeviceReMapping
**************************************************************************************************************************************************/
/*!
* @brief Set port remapping over 4 channels
*
*  This function allows the user to set port remapping over 4 channels.
*
* @param[in]   device_i2c_address        7 bit I2C address of the TPS238x part being controlled (do not included R/W bit)
* @param[in]   remapping                 value to write to remapping register
*
* @return  uint8_t    (I2C_SUCCESSFUL or I2C error status)
*
**************************************************************************************************************************************************/


uint8_t tps_SetDeviceReMapping(uint8_t device_i2c_address,TPS238x_Remapping_Register_t remapping)
{
    uint8_t rtn;

    rtn = tps_WriteI2CReg (device_i2c_address, TPS238X_CHANNEL_REMAPPING_COMMAND, *(uint8_t*)&remapping);

    return (rtn);
}


/*************************************************************************************************************************************************
*  tps_GetDeviceReMapping
**************************************************************************************************************************************************/
/*!
* @brief get port remapping over 4 channels
*
*  This function allows the user to get port remapping over 4 channels.
*
* @param[in]   device_i2c_address        7 bit I2C address of the TPS238x part being controlled (do not included R/W bit)
* @param[in]   remapping                 value to store  remapping information
*
* @return  uint8_t    (I2C_SUCCESSFUL or I2C error status)
*
**************************************************************************************************************************************************/


uint8_t tps_GetDeviceReMapping(uint8_t device_i2c_address,uint8_t *remapping)
{
    uint8_t rtn;

    rtn = tps_ReadI2CReg (device_i2c_address, TPS238X_CHANNEL_REMAPPING_COMMAND, (uint8_t *)remapping);

    return (rtn);
}

/*************************************************************************************************************************************************
*  tps_SetPortMultiBitPriority
**************************************************************************************************************************************************/
/*!
* @brief Set the 3 bit oss for a single channel
*
*This function allows the user to configure 3 bit oss priority for a given channel.
*
* @param[in]   systemPortNum             Port Number Value returned from the port registration function, tps_RegisterPort()
* @param[in]   multiBitPriority          3 bit code for priority
*
*
* @return  uint8_t    (I2C_SUCCESSFUL or I2C error status)
*
**************************************************************************************************************************************************/


uint8_t tps_SetPortMultiBitPriority(uint8_t systemPortNum,TPS238x_Multi_Bit_Power_Priority_t multiBitPriority)
{
uint8_t rtn;
uint8_t value;
uint8_t portNum = tps_GetDevicePortNum(systemPortNum);


    if ((uint8_t) portNum <= 2)
    {
        rtn = tps_ReadI2CReg (tps_GetDeviceI2CAddress(systemPortNum), TPS238X_MULTI_BIT_PORT_PRIORITY_21_COMMAND, &value);

        if ((uint8_t) portNum == 2)
        {
            value &= (MULTI_BIT_PRIORITY_MASK);                                           // Clear old value for this port number
            value |= ((uint8_t)multiBitPriority << MULTI_BIT_PRIORITY_SHIFT);             // Each port gets 3 bits
        }
        else
        {
            value &= (MULTI_BIT_PRIORITY_MASK << MULTI_BIT_PRIORITY_SHIFT);                      // Clear old value for this port number
            value |= (uint8_t)multiBitPriority;                                          // Each port gets 3 bits
        }

        rtn += tps_WriteI2CReg (tps_GetDeviceI2CAddress(systemPortNum), TPS238X_MULTI_BIT_PORT_PRIORITY_21_COMMAND, value);
        rtn += tps_ReadI2CReg (tps_GetDeviceI2CAddress(systemPortNum), TPS238X_MULTI_BIT_PORT_PRIORITY_21_COMMAND, &value);

    }
    else
    {
        rtn = tps_ReadI2CReg (tps_GetDeviceI2CAddress(systemPortNum), TPS238X_MULTI_BIT_PORT_PRIORITY_43_COMMAND, &value);

        if ((uint8_t) portNum == 4)
        {
            value &= (MULTI_BIT_PRIORITY_MASK);                                           // Clear old value for this port number
            value |= ((uint8_t)multiBitPriority << MULTI_BIT_PRIORITY_SHIFT);             // Each port gets 3 bits
        }
        else
        {
            value &= (MULTI_BIT_PRIORITY_MASK << MULTI_BIT_PRIORITY_SHIFT);                      // Clear old value for this port number
            value |= (uint8_t)multiBitPriority;                                          // Each port gets 3 bits
        }
        rtn += tps_WriteI2CReg (tps_GetDeviceI2CAddress(systemPortNum), TPS238X_MULTI_BIT_PORT_PRIORITY_43_COMMAND, value);
        rtn += tps_ReadI2CReg (tps_GetDeviceI2CAddress(systemPortNum), TPS238X_MULTI_BIT_PORT_PRIORITY_43_COMMAND, &value);

    }
    return (rtn);
}

/*************************************************************************************************************************************************
*  tps_GetPortMultiBitPriority
**************************************************************************************************************************************************/
/*!
* @brief Get the 3 bit oss for a single channel
*
*This function allows the user to configure 3 bit oss priority for a given channel.
*
* @param[in]   systemPortNum             Port Number Value returned from the port registration function, tps_RegisterPort()

*
*
* @return  uint8_t    3 bit oss priority
*
**************************************************************************************************************************************************/


uint8_t tps_GetPortMultiBitPriority(uint8_t systemPortNum)
{
uint8_t value;
uint8_t portNum = tps_GetDevicePortNum(systemPortNum);

    if ((uint8_t) portNum <= 2)
    {
        tps_ReadI2CReg (tps_GetDeviceI2CAddress(systemPortNum), TPS238X_MULTI_BIT_PORT_PRIORITY_21_COMMAND, &value);

        if(portNum == 1)
        {
            value &= MULTI_BIT_PRIORITY_MASK;
        }
        else
        {
            value = value >> MULTI_BIT_PRIORITY_SHIFT;
        }

    }
    else
    {
        tps_ReadI2CReg (tps_GetDeviceI2CAddress(systemPortNum), TPS238X_MULTI_BIT_PORT_PRIORITY_43_COMMAND, &value);

        if(portNum == 3)
        {
            value &= MULTI_BIT_PRIORITY_MASK;
        }
        else
        {
            value = value >> MULTI_BIT_PRIORITY_SHIFT;
        }

    }

    return(value);

}


/*************************************************************************************************************************************************
*  tps_SetDevice4PPowerAllocation
**************************************************************************************************************************************************/
/*!
* @brief Set power allocation for 4 pair ports
*
*  This function set power allocation for 4 pair ports
*
* @param[in]   device_i2c_address        7 bit I2C address of the TPS238x part being controlled (do not included R/W bit)
* @param[in]   allocatepower12, allocatepower34                allocated power for each port
*
* @return  uint8_t    (I2C_SUCCESSFUL or I2C error status)
*
**************************************************************************************************************************************************/


uint8_t tps_SetDevice4PPowerAllocation(uint8_t device_i2c_address,TPS238x_4P_Power_Allocation_t allocatepower12,TPS238x_4P_Power_Allocation_t allocatepower34)
{
uint8_t value;
uint8_t rtn;

    value = 0x88 | (allocatepower34 << 4) |(allocatepower12);
    rtn = tps_WriteI2CReg (device_i2c_address,TPS238X_4PWIRED_POWER_ALLOCATION_CONFIG_COMMAND, value);
    return(rtn);
}

/*************************************************************************************************************************************************
*  tps_SetDevice2PPowerAllocation
**************************************************************************************************************************************************/
/*!
* @brief Set power allocation for 2 pair ports
*
*  This function set power allocation for 2 pair ports
*
* @param[in]   device_i2c_address        7 bit I2C address of the TPS238x part being controlled (do not included R/W bit)
* @param[in]   allocatepower12, allocatepower34                allocated power for each port
*
* @return  uint8_t    (I2C_SUCCESSFUL or I2C error status)
*
**************************************************************************************************************************************************/


uint8_t tps_SetDevice2PPowerAllocation(uint8_t device_i2c_address,TPS238x_2P_Power_Allocation_t allocatepower12,TPS238x_2P_Power_Allocation_t allocatepower34)
{
uint8_t value;
uint8_t rtn;
    value = (allocatepower34 << 4) | (allocatepower12);
    rtn = tps_WriteI2CReg (device_i2c_address,TPS238X_4PWIRED_POWER_ALLOCATION_CONFIG_COMMAND, value);
    return(rtn);
}

/*************************************************************************************************************************************************
*  tps_GetDevice4P2PPowerAllocatoion
**************************************************************************************************************************************************/
/*!
* @brief Get power allocation for all ports
*
*  This function set power allocation for all ports
*
* @param[in]   device_i2c_address        7 bit I2C address of the TPS238x part being controlled (do not included R/W bit)
* @param[in]   powerallocation           allocated power for each port
*
* @return  uint8_t    (I2C_SUCCESSFUL or I2C error status)
*
**************************************************************************************************************************************************/


uint8_t tps_GetDevice4P2PPowerAllocatoion(uint8_t device_i2c_address, uint8_t *powerallocation)
{
uint8_t rtn;
uint8_t value;
    rtn = tps_ReadI2CReg (device_i2c_address, TPS238X_4PWIRED_POWER_ALLOCATION_CONFIG_COMMAND, &value);
    *powerallocation = value;
    return(rtn);
}


/*************************************************************************************************************************************************
*  tps_SetPort4PPolicing
**************************************************************************************************************************************************/
/*!
* @brief Set port 4P policing threshold
*
* This function will set port 2P policing threshold of TPS238x.
*
* @param[in]   systemPortNum             Port Number Value returned from the port registration function, tps_RegisterPort()
* @param[in]  icutCurrentThreshold           variable that contains the 2P policing threshold
*
* @return   uint8_t    (I2C_SUCCESSFUL or I2C error status)

**************************************************************************************************************************************************/


uint8_t tps_SetPort4PPolicing(uint8_t systemPortNum,uint8_t icutCurrentThreshold)
{
uint8_t rtn;
uint8_t value;
uint8_t portNum = tps_GetDevicePortNum(systemPortNum);

    if (portNum <= 2)
    {
        value = icutCurrentThreshold;
        rtn = tps_WriteI2CReg (tps_GetDeviceI2CAddress(systemPortNum), TPS238X_4P_POLICE_12_CONFIG_COMMAND, value);
    }
    else
    {
        rtn = tps_WriteI2CReg (tps_GetDeviceI2CAddress(systemPortNum), TPS238X_4P_POLICE_34_CONFIG_COMMAND, value);
    }
    return (rtn);
}

/*************************************************************************************************************************************************
*  tps_GetPort4PPolicing
**************************************************************************************************************************************************/
/*!
* @brief Get port 4P policing threshold
*
* This function will get port 4P policing threshold of TPS238x.
*
* @param[in]  systemPortNum             Port Number Value returned from the port registration function, tps_RegisterPort()
* @param[in]  *icutCurrentThreshold           variable that stores the 4P policing threshold
*
* @return   uint8_t    (I2C_SUCCESSFUL or I2C error status)

**************************************************************************************************************************************************/


uint8_t tps_GetPort4PPolicing(uint8_t systemPortNum,uint8_t *icutCurrentThreshold)
{
uint8_t rtn;
uint8_t value;
uint8_t portNum = tps_GetDevicePortNum(systemPortNum);

    if (portNum <= 2)
    {
        rtn = tps_ReadI2CReg (tps_GetDeviceI2CAddress(systemPortNum), TPS238X_4P_POLICE_12_CONFIG_COMMAND, &value);
        *icutCurrentThreshold = value;
    }
    else
    {
        rtn = tps_ReadI2CReg (tps_GetDeviceI2CAddress(systemPortNum), TPS238X_4P_POLICE_34_CONFIG_COMMAND, &value);
        *icutCurrentThreshold = value;
    }

    return (rtn);
}



/*************************************************************************************************************************************************
*  tps_SetDevice4PDCDTPCUTILIM
**************************************************************************************************************************************************/
/*!
* @brief Set port DC disconnect threshold, 4P PCUT enable, 4P PCUT management, 4P ILIM fault management
*
* This function will set  port DC disconnect threshold, 4P PCUT enable, 4P PCUT management, 4P ILIM fault management
*
* @param[in]   device_i2c_address        7 bit I2C address of the TPS238x part being controlled (do not included R/W bit)
* @param[in]   DCDTPCUTILIMConfig        variable that contains the DCDT,PCUT and ILIM configurations
*
* @return   uint8_t    (I2C_SUCCESSFUL or I2C error status)

**************************************************************************************************************************************************/


uint8_t tps_SetDevice4PDCDTPCUTILIM(uint8_t device_i2c_address,TPS238x_Dis_PCUT_ILIM_Register_t DCDTPCUTILIMConfig)
{
uint8_t rtn;

    rtn = tps_WriteI2CReg (device_i2c_address, TPS238X_INTERRUPT_MASK_COMMAND, *(uint8_t*)&DCDTPCUTILIMConfig);
    return (rtn);
}


/*************************************************************************************************************************************************
*  tps_SetPortILIM
**************************************************************************************************************************************************/
/*!
* @brief Set channel foldback curve
*
* This function will set channel foldback curve of TPS238x.
*
* @param[in]  systemPortNum             Port Number Value returned from the port registration function, tps_RegisterPort()
* @param[in]  poepFoldbackCurve         variable that contains the foldback curve
*
* @return   uint8_t    (I2C_SUCCESSFUL or I2C error status)

**************************************************************************************************************************************************/


uint8_t tps_SetPortILIM(uint8_t systemPortNum, TPS238x_Foldback_t poepFoldbackCurve)
{
uint8_t rtn;
uint8_t value;
uint8_t portNum = tps_GetDevicePortNum(systemPortNum);

    // Load the PoE Plus Setting
    rtn = tps_ReadI2CReg (tps_GetDeviceI2CAddress(systemPortNum), TPS238X_CHANNEL_FOLDBACK, &value);
    if (poepFoldbackCurve == _2X_ILIM_FOLDBACK_CURVE)
        value |= (CONVERT_PORT_NUM(portNum) << POE_PLUS_SHIFT);                      // Set the PoE Plus setting, so we use the 2x Foldback Curve
    else
        value &= ~(CONVERT_PORT_NUM(portNum) << POE_PLUS_SHIFT);                     // Clear the PoE Plus setting, so we use the 1x Foldback Curve
    rtn += tps_WriteI2CReg (tps_GetDeviceI2CAddress(systemPortNum), TPS238X_CHANNEL_FOLDBACK, value);

    return (rtn);

}


/*************************************************************************************************************************************************
*  tps_GetPortILIM
**************************************************************************************************************************************************/
/*!
* @brief Get channel foldback curve
*
* This function will get channel foldback curve of TPS238x.
*
* @param[in]  systemPortNum             Port Number Value returned from the port registration function, tps_RegisterPort()
* @param[in]  poepFoldbackCurve         variable that stores the foldback curve
*
* @return   uint8_t    (I2C_SUCCESSFUL or I2C error status)

**************************************************************************************************************************************************/


uint8_t tps_GetPortILIM(uint8_t systemPortNum, TPS238x_Foldback_t *poepFoldbackCurve)
{

uint8_t value;
uint8_t rtn;
uint8_t portBit = tps_GetDevicePortNum(systemPortNum);

    rtn = tps_ReadI2CReg (tps_GetDeviceI2CAddress(systemPortNum), TPS238X_CHANNEL_FOLDBACK, &value);
    *poepFoldbackCurve = (TPS238x_Foldback_t)((value>>(3+portBit)) & 0x01);


    return(rtn);
}

/*************************************************************************************************************************************************
*  tps_GetDeviceFirmwareRev
**************************************************************************************************************************************************/
/*!
* @brief Get device firmware revision
*
* This function will get device firmware revision of TPS238x.
*
* @param[in]  device_i2c_address             7 bit I2C address of the TPS238x part being controlled (do not included R/W bit)
* @param[in]  *firmwareRev                   variable that stores the device firmware revision
*
* @return   uint8_t    (I2C_SUCCESSFUL or I2C error status)

**************************************************************************************************************************************************/


uint8_t tps_GetDeviceFirmwareRev(uint8_t device_i2c_address,uint8_t *firmwareRev)
{
uint8_t rtn;
uint8_t value;

    rtn = tps_ReadI2CReg (device_i2c_address, TPS238X_FIRMWARE_REVISION_COMMAND, &value);
    *firmwareRev = value;
    return (rtn);

}


/*************************************************************************************************************************************************
*  tps_GetDeviceID
**************************************************************************************************************************************************/
/*!
* @brief Get device silicon revision and device ID
*
* This function will get device silicon revision and device ID of TPS238x.
*
* @param[in]  device_i2c_address             7 bit I2C address of the TPS238x part being controlled (do not included R/W bit)
* @param[in]  *siliconRev                   variable that stores the device silicon revision
* @param[in]  *deviceID                     variable that stores the device ID
*
* @return   uint8_t    (I2C_SUCCESSFUL or I2C error status)

**************************************************************************************************************************************************/

uint8_t tps_GetDeviceID(uint8_t device_i2c_address,uint8_t *siliconRev, uint8_t *deviceID)
{
uint8_t rtn;
uint8_t value;

    rtn = tps_ReadI2CReg (device_i2c_address, TPS238X_FIRMWARE_REVISION_COMMAND, &value);
    *siliconRev = value & SILICONREV_MASK;
    *deviceID = (value & DEVICEID_MASK) >> DEVICEID_SHIFT;
    return (rtn);

}




/**************************************************************************************************************************************************
*                                 System Measurements
**************************************************************************************************************************************************/

/*************************************************************************************************************************************************
*  tps_GetDeviceTemperature
**************************************************************************************************************************************************/
/*!
* @brief Returns the temperature of the specified TPS23861/TPS2388
*
* This function will return the temperature of the specified TPS23861/TPS2388.
*
* @param[in]   device_i2c_address        7 bit I2C address of the TPS238x part being controlled (do not included R/W bit)
* @param[out]  *temperature              The address of a uint8_t variable where the temperature of the TPS23861/TPS2388. The temperature
*                                           will be a scaled integer with an LSB of 0.652 degrees Celsius
*
* @return  uint8_t    (I2C_SUCCESSFUL or I2C error status)
*
**************************************************************************************************************************************************/
uint8_t tps_GetDeviceTemperature (uint8_t device_i2c_address, uint8_t *temperature)
{
uint8_t rtn;

    rtn = tps_ReadI2CReg (device_i2c_address, TPS238X_TEMPERATURE_COMMAND, (uint8_t *)temperature);

    return (rtn);
}



/*************************************************************************************************************************************************
*  tps_GetDeviceInputVoltage
**************************************************************************************************************************************************/
/*!
* @brief Returns the input voltage of the specified TPS23861/TPS2388
*
* This function will return the input voltage of the specified TPS23861/TPS2388.
*
* @param[in]   device_i2c_address        7 bit I2C address of the TPS238x part being controlled (do not included R/W bit)
* @param[out]  *voltage                  The address of a uint16_t variable where the input voltage of the TPS23861/TPS2388. The voltage
*                                           will be a scaled integer with an LSB of 3.662 mVolts
*
* @return  uint8_t    (I2C_SUCCESSFUL or I2C error status)
*
* @sa tps_GetPortMeasurements ()
**************************************************************************************************************************************************/
uint8_t tps_GetDeviceInputVoltage (uint8_t device_i2c_address, uint16_t *voltage)
{
uint8_t rtn;
uint16_t value;

    rtn = tps_ReadI2CMultiple (device_i2c_address, TPS238X_INPUT_VOLTAGE_COMMAND, (uint8_t *)&value, 2);

    *voltage = value & TPS2368X_INPUT_VOLTAGE_MASK_SHORT;

    return (rtn);
}


/*************************************************************************************************************************************************
*  tps_GetPortMeasurements
**************************************************************************************************************************************************/
/*!
* @brief Returns the voltage and current of the registered system port.
*
* This function will return the voltage and current on the registered port on a specific TPS23861/TPS2388.
*
* @param[in]   systemPortNum             Port Number Value returned from the port registration function, tps_RegisterPort()
* @param[out]  *voltage                  The address of a uint16_t variable where the voltage of the indicated port will be placed. The voltage
*                                           will be a scaled integer with an LSB of 3.662 mVolts
* @param[out]  *current                  The address of a uint16_t variable where the current of the indicated port will be placed. The current
*                                           will be a scaled integer with an LSB that is dependent on the current-sense resistor.
*                                           For a 250 mOhm resistor, the LSB will be 62.260 micro-Amps.
*                                           For a 255 mOhm resistor, the LSB will be 61.039 micro-Amps.
*
* @return  uint8_t    (I2C_SUCCESSFUL or I2C error status)
*
* @sa tps_GetDeviceInputVoltage ()
**************************************************************************************************************************************************/
uint8_t tps_GetPortMeasurements (uint8_t systemPortNum, uint16_t *voltage, uint16_t *current)
{
uint8_t rtn;
uint8_t command;
uint16_t value;
uint8_t portNum = tps_GetDevicePortNum(systemPortNum);

    command = TPS238X_CHANNEL_1_CURRENT_COMMAND + (4 * ((uint8_t) portNum - 1));
    rtn = tps_ReadI2CMultiple (tps_GetDeviceI2CAddress(systemPortNum), command, (uint8_t *)&value, 2);
    *current = value & TPS238X_CHANNEL_CURRENT_MASK_SHORT;

    command = TPS238X_CHANNEL_1_VOLTAGE_COMMAND + (4 * ((uint8_t) portNum - 1));
    rtn += tps_ReadI2CMultiple (tps_GetDeviceI2CAddress(systemPortNum), command, (uint8_t *)&value, 2);
    *voltage = value & TPS238X_CHANNEL_VOLTAGE_MASK_SHORT;

    return (rtn);
}

/*************************************************************************************************************************************************
*  tps_GetPortDetectResistance
**************************************************************************************************************************************************/
/*!
* @brief Returns the detection resistance value for the registered system port number on a specific TPS2388x
*
* This function will return the detection resistance for the registered system port on a TPS2388x.
*
* @param[in]   systemPortNum             Port Number Value returned from the port registration function, tps_RegisterPort()
* @param[out]  *detectResistance         The address of a uint8_t variable where the detection resistance of the port.
*
* @return  uint8_t    (I2C_SUCCESSFUL or I2C error status)
*
**************************************************************************************************************************************************/
uint8_t tps_GetPortDetectResistance (uint8_t systemPortNum, uint8_t *detectResistance)
{
uint8_t rtn;
uint8_t value;
uint8_t portNum = tps_GetDevicePortNum(systemPortNum);
uint8_t command = TPS238X_CHANNEL_1_DETECT_RESISTANCE_COMMAND + ((uint8_t)portNum - 1);

    rtn = tps_ReadI2CReg (tps_GetDeviceI2CAddress(systemPortNum), command, (uint8_t *)&value);

    *detectResistance = value;
    return (rtn);
}


/*************************************************************************************************************************************************
*  tps_GetPortDetectCapacitance
**************************************************************************************************************************************************/
/*!
* @brief Returns the detection capacitance value for the registered system port number on a specific TPS2388x
*
* This function will return the detection capacitance for the registered system port on a TPS2388x.
*
* @param[in]   systemPortNum             Port Number Value returned from the port registration function, tps_RegisterPort()
* @param[out]  *detectResistance         The address of a uint8_t variable where the detection capacitance of the port.
*
* @return  uint8_t    (I2C_SUCCESSFUL or I2C error status)
*
*
**************************************************************************************************************************************************/
uint8_t tps_GetPortDetectCapacitance (uint8_t systemPortNum, uint8_t *detectCapacitance)
{
uint8_t rtn;
uint8_t value;
uint8_t portNum = tps_GetDevicePortNum(systemPortNum);
uint8_t command = TPS238X_CHANNEL_1_DETECT_CAPACITANCE_COMMAND + ((uint8_t)portNum - 1);

    rtn = tps_ReadI2CReg (tps_GetDeviceI2CAddress(systemPortNum), command, (uint8_t *)&value);

    *detectCapacitance = value;
    return (rtn);
}

/*************************************************************************************************************************************************
*  tps_GetPortAssignedClass
**************************************************************************************************************************************************/
/*!
* @brief Returns previous and assigned class on each channel of a TPS2388x
*
* This function will return the previous and assigned class on each channel of a TPS2388x.
*
* @param[in]   systemPortNum             Port Number Value returned from the port registration function, tps_RegisterPort()
* @param[in]   *preclass                 Variable stores the previous class status
* @param[in]   *assclass                 Variable stores the assigned class status
*
* @return  uint8_t    (I2C_SUCCESSFUL or I2C error status)
*
*
**************************************************************************************************************************************************/


uint8_t tps_GetPortAssignedClass(uint8_t systemPortNum, uint8_t *preclass, uint8_t *assclass)
{
uint8_t rtn;
uint8_t value;
uint8_t portNum = tps_GetDevicePortNum(systemPortNum);
uint8_t command = TPS238X_CHANNEL_1_ASSIGNED_CLASS_COMMAND + ((uint8_t)portNum - 1);

    rtn = tps_ReadI2CReg (tps_GetDeviceI2CAddress(systemPortNum), command, (uint8_t *)&value);

    *preclass = value & PRECLASS_MASK;
    *assclass = value >> ASSCLASS_SHIFT;

    return(rtn);
}


/*************************************************************************************************************************************************
*  tps_SetDeviceAutoClass
**************************************************************************************************************************************************/
/*!
* @brief Set auto class on each channel of a TPS2388x
*
* This function will Set auto class on each channel of a TPS2388x.
*
* @param[in]   device_i2c_address        7 bit I2C address of the TPS238x part being controlled (do not included R/W bit)
* @param[in]   autoACports               Variable contains the auto auto class ports
* @param[in]   manualACports             Variable contains the manual auto class ports
*
* @return  uint8_t    (I2C_SUCCESSFUL or I2C error status)
*
*
**************************************************************************************************************************************************/

uint8_t tps_SetDeviceAutoClass(uint8_t device_i2c_address, TPS238x_Ports_t autoACports, TPS238x_Ports_t manualACports)
{
uint8_t rtn;
uint8_t value;
    value = autoACports | (manualACports << MAC_SHIFT);
    rtn = tps_WriteI2CReg (device_i2c_address, TPS238X_AUTO_CLASS_CONTROL_COMMAND, value);

    return(rtn);
}


/*************************************************************************************************************************************************
*  tps_GetDeviceAutoClass
**************************************************************************************************************************************************/
/*!
* @brief Get auto class on each channel of a TPS2388x
*
* This function will get auto class on each channel of a TPS2388x.
*
* @param[in]   device_i2c_address        7 bit I2C address of the TPS238x part being controlled (do not included R/W bit)
* @param[in]   *autoACports              Variable stores the auto auto class ports
* @param[in]   *manualACports            Variable stores the manual auto class ports
*
* @return  uint8_t    (I2C_SUCCESSFUL or I2C error status)
*
*
**************************************************************************************************************************************************/


uint8_t tps_GetDeviceAutoClass(uint8_t device_i2c_address, TPS238x_Ports_t *autoACports, TPS238x_Ports_t *manualACports)
{

uint8_t rtn;
uint8_t value;

    rtn = tps_ReadI2CReg (device_i2c_address, TPS238X_AUTO_CLASS_CONTROL_COMMAND, (uint8_t *)&value);
    *autoACports = value & AAC_MASK;
    *manualACports = value >> MAC_SHIFT;

    return(rtn);
}


/*************************************************************************************************************************************************
*  tps_GetPortAutoClassPower
**************************************************************************************************************************************************/
/*!
* @brief Returns the detection resistance value for the registered system port number on a specific TPS2388x
*
* This function will return the detection resistance for the registered system port on a TPS2388x.
*
* @param[in]   systemPortNum             Port Number Value returned from the port registration function, tps_RegisterPort()
* @param[out]  *detectResistance         The address of a uint8_t variable where the detection resistance of the port.
*
* @return  uint8_t    (I2C_SUCCESSFUL or I2C error status)
*
**************************************************************************************************************************************************/
uint8_t tps_GetPortAutoClassPower (uint8_t systemPortNum,TPS238x_On_Off_t *fault,  uint8_t *autoclasspower)
{
uint8_t rtn;
uint8_t value;
uint8_t portNum = tps_GetDevicePortNum(systemPortNum);
uint8_t command = TPS238X_CHANNEL_1_AUTO_CLASS_POWER_COMMAND + ((uint8_t)portNum - 1);

    rtn = tps_ReadI2CReg (tps_GetDeviceI2CAddress(systemPortNum), command, (uint8_t *)&value);

    *fault = *(TPS238x_On_Off_t*) (value >> AUTOCLASS_POWER_SHIFT);
    *autoclasspower = value & AUTOCLASS_POWER_MASK;
    return (rtn);
}

/*************************************************************************************************************************************************
*  tps_SetDeviceAltFoldbackEnable
**************************************************************************************************************************************************/
/*!
* @brief Set alternative inrush and ILIM foldback on each channel of a TPS2388x
*
* This function will set alternative inrush and ILIM foldbacks on each channel of a TPS2388x.
*
* @param[in]   device_i2c_address        7 bit I2C address of the TPS238x part being controlled (do not included R/W bit)
* @param[in]   altinrush                 Variable contains enabling alternative inrush ports
* @param[in]   manualACports             Variable contains enabling alternative ILIM ports
*
* @return  uint8_t    (I2C_SUCCESSFUL or I2C error status)
*
*
**************************************************************************************************************************************************/


uint8_t tps_SetDeviceAltFoldbackEnable(uint8_t device_i2c_address, TPS238x_Ports_t altinrush, TPS238x_Ports_t altILIM)
{
uint8_t rtn;
uint8_t value;
    value = altinrush | (altILIM << MAC_SHIFT);
    rtn = tps_WriteI2CReg (device_i2c_address, TPS238X_ALTERNATIVE_FOLDBACK_ENABLE_COMMAND, value);

    return(rtn);
}




/*************************************************************************************************************************************************
*  tps_GetDeviceAltFoldbackEnable
**************************************************************************************************************************************************/
/*!
* @brief Get alternative inrush and ILIM foldback on each channel of a TPS2388x
*
* This function will get alternative inrush and ILIM foldbacks on each channel of a TPS2388x.
*
* @param[in]   device_i2c_address        7 bit I2C address of the TPS238x part being controlled (do not included R/W bit)
* @param[in]   *altinrush                 Variable contains enabling alternative inrush ports
* @param[in]   *manualACports             Variable contains enabling alternative ILIM ports
*
* @return  uint8_t    (I2C_SUCCESSFUL or I2C error status)
*
*
**************************************************************************************************************************************************/


uint8_t tps_GetDeviceAltFoldbackEnable(uint8_t device_i2c_address, TPS238x_Ports_t *altinrush, TPS238x_Ports_t *altILIM)
{

uint8_t rtn;
uint8_t value;

    rtn = tps_ReadI2CReg (device_i2c_address, TPS238X_ALTERNATIVE_FOLDBACK_ENABLE_COMMAND, (uint8_t *)&value);
    *altinrush = value & ALT_INRUSH_MASK;
    *altILIM = value >> ALT_ILIM_SHIFT;

    return(rtn);
}







/*************************************************************************************************************************************************
*  tps_ReleasePort
**************************************************************************************************************************************************/
/*!
* @brief Releases a register system port number from the port mapping table
*
* This function would undo the registration performed in tps_RegisterPort().
*
* @param[in]   systemPortNum          Port Number Value returned from the port registration function, tps_RegisterPort()
*
* @return  uint8_t    TPS_SUCCESSFUL or TPS_ERR_PORT_NOT_IN_USE
*
* @sa tps_RegisterPort ()
**************************************************************************************************************************************************/
uint8_t tps_ReleasePort (uint8_t systemPortNum)
{
    if (TPS_PortMap[systemPortNum].i2cAddress == TPS_CHANNEL_NOT_REGISTERED_VALUE)
    {
        return (TPS_ERR_PORT_NOT_IN_USE);
    }
    else
    {
        TPS_PortMap[systemPortNum].i2cAddress   = TPS_CHANNEL_NOT_REGISTERED_VALUE;
        TPS_PortMap[systemPortNum].devicePortNum = (TPS238x_PortNum_t)TPS_CHANNEL_NOT_REGISTERED_VALUE;
        return (TPS_SUCCESSFUL);
    }
}


uint8_t tps_ResetInterruptPin (uint8_t device_i2c_address)
{
uint8_t rtn;
const uint8_t value = CLINP;

    // Reset the interrupt pin
    rtn = tps_WriteI2CReg (device_i2c_address, TPS238X_RESET_COMMAND, value);

    return (rtn);


}





/*************************************************************************************************************************************************
*  tps_GetPortPower
**************************************************************************************************************************************************/
/*!
* @brief Get the port power consumption based on a system port number
*
* This function would get the port power consumption based on a system port number
*
* @param[in]   systemPortNum          Port Number Value returned from the port registration function, tps_RegisterPort()
*
* @return  uint32_t    port power
*
**************************************************************************************************************************************************/


uint32_t tps_GetPortPower (uint8_t systemPortNum)
{
    uint32_t V_temp = 0,
                  I_temp = 0;
    uint16_t voltage = 0,
             current = 0;
    uint32_t portPower = 0;

    tps_GetPortMeasurements (systemPortNum, &voltage, &current);
    V_temp = ((uint32_t)voltage * TPS_VOLTAGE_SCALE_FACTOR) / (uint32_t)1000; // (unit:mV)
    I_temp = ((uint32_t)current * TPS_CURRENT_SCALE_FACTOR) / (uint32_t)1000000;
    portPower = ((uint32_t)(V_temp * I_temp) / (uint32_t)1000);  // (unit:mW)

    return(portPower);
}


/*************************************************************************************************************************************************
*  tps_UpdateSRAMCode
**************************************************************************************************************************************************/
/*!
* @brief Get the port power consumption based on a system port number
*
* This function would get the port power consumption based on a system port number

*
* @return  uint8_t    success or fail
*
**************************************************************************************************************************************************/
uint8_t firmwareversion;
uint16_t SRAM_index = 0;

uint8_t tps_UpdateSRAMCode(void)
{

uint8_t rtn = 0;

    tps_WriteI2CReg (tps2388x_i2cAddList[0], TPS238X_SRAM_CONTROL_COMMAND, 0x01);

    tps_WriteI2CReg (tps2388x_i2cAddList[0], TPS238X_SRAM_START_ADDRESS_LSB_COMMAND, 0x00);

    tps_WriteI2CReg (tps2388x_i2cAddList[0], TPS238X_SRAM_START_ADDRESS_MSB_COMMAND, 0x80);

    if(PARITY_EN == 1)
    {
        tps_WriteI2CReg (tps2388x_i2cAddList[0],TPS238X_SRAM_CONTROL_COMMAND, 0xC4);

        for(SRAM_index = 0; SRAM_index< NUM_PARITY_BYTES; SRAM_index++)
        {
            tps_WriteI2CReg (tps2388x_i2cAddList[0], TPS238X_SRAM_DATA_COMMAND, ParityCode[SRAM_index]);
        }

        tps_WriteI2CReg (tps2388x_i2cAddList[0],TPS238X_SRAM_CONTROL_COMMAND, 0xC5);

        tps_WriteI2CReg (tps2388x_i2cAddList[0], TPS238X_SRAM_START_ADDRESS_LSB_COMMAND, 0x00);

        tps_WriteI2CReg (tps2388x_i2cAddList[0], TPS238X_SRAM_START_ADDRESS_MSB_COMMAND, 0x80);
    }

    tps_WriteI2CReg (tps2388x_i2cAddList[0], TPS238X_SRAM_CONTROL_COMMAND, 0xC0);

    for(SRAM_index = 0; SRAM_index< NUM_SRAM_BYTES; SRAM_index++)
    {
        tps_WriteI2CReg (tps2388x_i2cAddList[0], TPS238X_SRAM_DATA_COMMAND, SRAMCode[SRAM_index]);

    }


    if(PARITY_EN == 1)
    {
        tps_WriteI2CReg (tps2388x_i2cAddList[0], TPS238X_SRAM_CONTROL_COMMAND, 0x18);
    }

    else
    {
        tps_WriteI2CReg (tps2388x_i2cAddList[0], TPS238X_SRAM_CONTROL_COMMAND, 0x08);
    }



    __delay_cycles(100000);

    //Check the firmware version to make sure the SRAM code has been loaded properly
    tps_ReadI2CReg (tps2388x_i2cAddList[0], TPS238X_FIRMWARE_REVISION_COMMAND, &firmwareversion);

#if(1)
    if (firmwareversion == SRAM_VERSION)
    {
        uart_puts("SRAM Code Load Complete!\r\n");
        rtn = 1;
    }
    else
    {
        uart_puts("SRAM Code Load Error!\r\n");
    }

#endif

    return(rtn);
}

void PSE_TPS238X_init(TPS238x_Operating_Modes_t opMode)
{

TPS238X_Interrupt_Mask_Register_t intMask;

//tps_ReadI2CReg (tps2388x_i2cAddList[0], TPS238X_FIRMWARE_REVISION_COMMAND, &firmwareversion);
tps_GetDeviceFirmwareRev(tps2388x_i2cAddList[0],&firmwareversion);

if(firmwareversion != SRAM_VERSION)
{
    //Load SRAM code
    tps_UpdateSRAMCode();
}

        __delay_cycles(184000);

        intMask.CLMSK_Classificiation_Cycle_Unmask = 1;
        intMask.DEMSK_Detection_Cycle_Unmask = 0;
        intMask.DIMSK_Disconnect_Unmask = 1;
        intMask.PGMSK_Power_Good_Unmask = 1;
        intMask.PEMSK_Power_Enable_Unmask = 1;
        intMask.IFMSK_IFAULT_Unmask = 1;
        intMask.INMSK_Inrush_Fault_Unmask = 1;
        intMask.SUMSK_Supply_Event_Fault_Unmask = 1;

        //Configure device's interrupt
        tps_SetDeviceInterruptMask (tps2388x_i2cAddList[0], intMask);
        tps_SetDeviceInterruptMask (tps2388x_i2cAddList[1], intMask);

        //Set all channels in semi-auto mode

        tps_SetDeviceOperatingMode (tps2388x_i2cAddList[0], opMode, opMode,
                                    opMode, opMode);
        tps_SetDeviceOperatingMode (tps2388x_i2cAddList[1], opMode, opMode,
                                    opMode, opMode);

        //Enable all channel's DC disconnect
        tps_SetDeviceDisconnectEnable(tps2388x_i2cAddList[0], TPS238X_ALL_PORTS);
        tps_SetDeviceDisconnectEnable(tps2388x_i2cAddList[1], TPS238X_ALL_PORTS);


        //Set 2pair ports in 2 pair 30W mode
        tps_SetDevice2PPowerAllocation(tps2388x_i2cAddList[0], _2P_30W_30W,_2P_30W_30W);

        //Set  4pair ports in 4 pair 90W mode
        tps_SetDevice4PPowerAllocation(tps2388x_i2cAddList[1],_4P_90W,_4P_90W);


        // Power off all ports in case we are re-running this application without physically shutting down ports from previous run
        tps_SetDevicePowerOff (tps2388x_i2cAddList[0], TPS238X_ALL_PORTS);
        tps_SetDevicePowerOff (tps2388x_i2cAddList[1], TPS238X_ALL_PORTS);

        //Enable all channels' detection and classification
        tps_SetDeviceDetectClassEnable(tps2388x_i2cAddList[0],TPS238X_ALL_PORTS,TPS238X_ALL_PORTS);
        tps_SetDeviceDetectClassEnable(tps2388x_i2cAddList[1],TPS238X_ALL_PORTS,TPS238X_ALL_PORTS);
}


/*************************************************************************************************************************************************/
/* End of PUBLIC section                                                                                                                        */
/*                                                                                                                                               */
/*! \endcond                                                                                                                                     */
/*************************************************************************************************************************************************/












































#if(0)

#include "I2C_functions.h"
#include <tps238x.h>
#include "sys_init.h"




//uint8_t tps_GetI2CAddress(void)
//{
//    uint8_t rtn,i,value;
//    uint8_t j = 0;
//
//    for (i = 0; i < 16; i++)
//    {
//        rtn = tps_GetI2CAddresses(tps2388_binaryAddrList[i], &value);
//        if(rtn && 0x1)
//        {
//            value = (value >> 2) | 0x20;
//
//            if((value == tps2388_binaryAddrList[i]) || (value == tps2388_binaryAddrList[i] + 1))
//            {
//
//                tps2388x_i2cAddList[j] = tps2388_binaryAddrList[i];
//                j = j + 1;
//                tps2388x_i2cAddList[j] = tps2388_binaryAddrList[i] + 1;
//                j = j + 1;
//            }
//
//            /* Make sure buffer is large enough */
//            if(j >= (NUM_OF_TPS2388x * NUM_OF_QUARD))
//            {
//                // Error, allocate more data
//                break;
//            }
//
//        }
//    }
//
//    return (rtn);
//}


static TPS238x_System_Port_Map_t TPS_PortMap[TPS_MAX_SYSTEM_CHANNELS] = {{TPS_CHANNEL_NOT_REGISTERED_VALUE, (TPS238x_PortNum_t)TPS_CHANNEL_NOT_REGISTERED_VALUE},
                                                                       {TPS_CHANNEL_NOT_REGISTERED_VALUE, (TPS238x_PortNum_t)TPS_CHANNEL_NOT_REGISTERED_VALUE},
                                                                       {TPS_CHANNEL_NOT_REGISTERED_VALUE, (TPS238x_PortNum_t)TPS_CHANNEL_NOT_REGISTERED_VALUE},
                                                                       {TPS_CHANNEL_NOT_REGISTERED_VALUE, (TPS238x_PortNum_t)TPS_CHANNEL_NOT_REGISTERED_VALUE},

                                                                       {TPS_CHANNEL_NOT_REGISTERED_VALUE, (TPS238x_PortNum_t)TPS_CHANNEL_NOT_REGISTERED_VALUE},
                                                                       {TPS_CHANNEL_NOT_REGISTERED_VALUE, (TPS238x_PortNum_t)TPS_CHANNEL_NOT_REGISTERED_VALUE},
                                                                       {TPS_CHANNEL_NOT_REGISTERED_VALUE, (TPS238x_PortNum_t)TPS_CHANNEL_NOT_REGISTERED_VALUE},
                                                                       {TPS_CHANNEL_NOT_REGISTERED_VALUE, (TPS238x_PortNum_t)TPS_CHANNEL_NOT_REGISTERED_VALUE},

                                                                       {TPS_CHANNEL_NOT_REGISTERED_VALUE, (TPS238x_PortNum_t)TPS_CHANNEL_NOT_REGISTERED_VALUE},
                                                                       {TPS_CHANNEL_NOT_REGISTERED_VALUE, (TPS238x_PortNum_t)TPS_CHANNEL_NOT_REGISTERED_VALUE},
                                                                       {TPS_CHANNEL_NOT_REGISTERED_VALUE, (TPS238x_PortNum_t)TPS_CHANNEL_NOT_REGISTERED_VALUE},
                                                                       {TPS_CHANNEL_NOT_REGISTERED_VALUE, (TPS238x_PortNum_t)TPS_CHANNEL_NOT_REGISTERED_VALUE},

                                                                       {TPS_CHANNEL_NOT_REGISTERED_VALUE, (TPS238x_PortNum_t)TPS_CHANNEL_NOT_REGISTERED_VALUE},
                                                                       {TPS_CHANNEL_NOT_REGISTERED_VALUE, (TPS238x_PortNum_t)TPS_CHANNEL_NOT_REGISTERED_VALUE},
                                                                       {TPS_CHANNEL_NOT_REGISTERED_VALUE, (TPS238x_PortNum_t)TPS_CHANNEL_NOT_REGISTERED_VALUE},
                                                                       {TPS_CHANNEL_NOT_REGISTERED_VALUE, (TPS238x_PortNum_t)TPS_CHANNEL_NOT_REGISTERED_VALUE},

                                                                       {TPS_CHANNEL_NOT_REGISTERED_VALUE, (TPS238x_PortNum_t)TPS_CHANNEL_NOT_REGISTERED_VALUE},
                                                                       {TPS_CHANNEL_NOT_REGISTERED_VALUE, (TPS238x_PortNum_t)TPS_CHANNEL_NOT_REGISTERED_VALUE},
                                                                       {TPS_CHANNEL_NOT_REGISTERED_VALUE, (TPS238x_PortNum_t)TPS_CHANNEL_NOT_REGISTERED_VALUE},
                                                                       {TPS_CHANNEL_NOT_REGISTERED_VALUE, (TPS238x_PortNum_t)TPS_CHANNEL_NOT_REGISTERED_VALUE},

                                                                       {TPS_CHANNEL_NOT_REGISTERED_VALUE, (TPS238x_PortNum_t)TPS_CHANNEL_NOT_REGISTERED_VALUE},
                                                                       {TPS_CHANNEL_NOT_REGISTERED_VALUE, (TPS238x_PortNum_t)TPS_CHANNEL_NOT_REGISTERED_VALUE},
                                                                       {TPS_CHANNEL_NOT_REGISTERED_VALUE, (TPS238x_PortNum_t)TPS_CHANNEL_NOT_REGISTERED_VALUE},
                                                                       {TPS_CHANNEL_NOT_REGISTERED_VALUE, (TPS238x_PortNum_t)TPS_CHANNEL_NOT_REGISTERED_VALUE},

                                                                       {TPS_CHANNEL_NOT_REGISTERED_VALUE, (TPS238x_PortNum_t)TPS_CHANNEL_NOT_REGISTERED_VALUE},
                                                                       {TPS_CHANNEL_NOT_REGISTERED_VALUE, (TPS238x_PortNum_t)TPS_CHANNEL_NOT_REGISTERED_VALUE},
                                                                       {TPS_CHANNEL_NOT_REGISTERED_VALUE, (TPS238x_PortNum_t)TPS_CHANNEL_NOT_REGISTERED_VALUE},
                                                                       {TPS_CHANNEL_NOT_REGISTERED_VALUE, (TPS238x_PortNum_t)TPS_CHANNEL_NOT_REGISTERED_VALUE},

                                                                       {TPS_CHANNEL_NOT_REGISTERED_VALUE, (TPS238x_PortNum_t)TPS_CHANNEL_NOT_REGISTERED_VALUE},
                                                                       {TPS_CHANNEL_NOT_REGISTERED_VALUE, (TPS238x_PortNum_t)TPS_CHANNEL_NOT_REGISTERED_VALUE},
                                                                       {TPS_CHANNEL_NOT_REGISTERED_VALUE, (TPS238x_PortNum_t)TPS_CHANNEL_NOT_REGISTERED_VALUE},
                                                                       {TPS_CHANNEL_NOT_REGISTERED_VALUE, (TPS238x_PortNum_t)TPS_CHANNEL_NOT_REGISTERED_VALUE},

};


/*************************************************************************************************************************************************
*  tps_RegisterPort
**************************************************************************************************************************************************/
/*!
* @brief Allocates a system level port number for a given TPS23861 device and it's specific port number.
*
* It is easier for users to maintain a system with 4 TPS23861 with 4 ports each, when each port has a unique number. By registering each of the 16
* individual ports (in this example), the user can work with ports 0-15, rather than have to maintain device 1 - port 0, through device 4 - port 3.
*
* This function must be called prior to any function call that has a systemPortNum as an input. The current logic has room for
* TPS_MAX_SYSTEM_PORTS (64) ports. The TPS_PortMap[] variable can be modified to support more ports (or save some memory by supporting fewer).
*
* @param[in]   device_i2c_address  7 bit I2C address of the TPS238x part being controlled (do not included R/W bit)
* @param[in]   devicePortNum       TPS238X_PORT_1 - TPS238X_PORT_4, The port number ON THE SPECIFIC TPS23861 device
*
* @return  uint8_t  systemPortNum     Unique handle that will be used to refer to this device and port number in future function calls.
* @return  TPS_ERR_NO_PORT_AVAILABLE  No port number available, all are in use
*
* @sa tps_ReleasePort ()
**************************************************************************************************************************************************/
uint8_t tps_RegisterPort (uint16_t device_i2c_address, TPS238x_PortNum_t devicePortNum)
{
uint8_t i;
uint8_t found = 0xff;

    for (i=0; i<TPS_MAX_SYSTEM_CHANNELS; i++)
    {
        if (TPS_PortMap[i].i2cAddress == TPS_CHANNEL_NOT_REGISTERED_VALUE)
        {
            found = i;
            TPS_PortMap[i].i2cAddress = device_i2c_address;
            TPS_PortMap[i].devicePortNum = devicePortNum;
            break;
        }
    }

    if (found == 0xff)
        return (TPS_ERR_NO_PORT_AVAILABLE);
    else
        return (found);
}

/*************************************************************************************************************************************************
*  tps_GetDeviceI2CAddress
**************************************************************************************************************************************************/
/*!
* @brief Get the TPS23861 I2C Address associated with a registered System Port Number
*
* This function returns the I2C address for the TPS23861 device registered for a given System Port Number
*
* @param[in]   systemPortNum          Port Number Value returned from the port registration function, tps_RegisterPort()
*
* @return  uint16_t    I2C Address number associated with this specific port.
* @return  TPS_PORT_NOT_REGISTERED_VALUE - Indicates no port associated.
*
* @sa tps_RegisterPort ()
* @sa tps_GetDevicePortNum ()
**************************************************************************************************************************************************/
uint16_t tps_GetDeviceI2CAddress (uint8_t systemPortNum)
{
    return (TPS_PortMap[systemPortNum].i2cAddress);
}

/*************************************************************************************************************************************************
*  tps_GetDevicePortNum()
**************************************************************************************************************************************************/
/*!
* @brief Get the TPS23861 device port number with a registered System Port Number
*
* This function returns the device port number for the TPS23861 device registered for a given System Port Number
*
* @param[in]   systemPortNum          Port Number Value returned from the port registration function, tps_RegisterPort()
*
* @return  uint16_t    Device Port Number associated with this specific System Port Number.
* @return  TPS_PORT_NOT_REGISTERED_VALUE - Indicates no port associated.
*
* @sa tps_RegisterPort ()
* @sa tps_GetDeviceI2CAddress ()
**************************************************************************************************************************************************/
TPS238x_PortNum_t tps_GetDevicePortNum (uint8_t systemPortNum)
{
   return (TPS_PortMap[systemPortNum].devicePortNum);

}

/*************************************************************************************************************************************************
*  tps_GetDeviceInterruptStatus
**************************************************************************************************************************************************/
/*!
* @brief Get the current interrupt status for the indicated TPS238X part.
*
* The function returns a variable that has a one bit indicator for each of the interrupts in the TPS238X.
*
* The interrupt mask register identifies which events/faults will generate an interrupt. The status register will still indicate
* events/faults that are masked and would not generate an interrupt. If the user wants to only process unmasked interrupts, the results
* from this function must be combined with the interrupt mask (tps_GetInterruptMask)
*
* @param[in]   device_i2c_address  7 bit I2C address of the TPS238x part being controlled (do not included R/W bit)
* @param[out]  *status             Address of a TPS238X_Interrupt_Register_t variable that will receive the current interrupt status
*	                                    PEC_Power_Enable_Change
*	                                    PGC_Power_Good_Change
*	                                    DISF_Disconnect_Event
*	                                    DETC_Detection_Cycle
*	                                    CLASC_Classification_Cycle
*	                                    IFAULT_ICUT_ILIM_Fault
*	                                    INRF_Inrush_Fault
*	                                    SUPF_Supply_Event_Fault
*
* @return  uint8_t    (I2C_SUCCESSFUL or I2C error status)
*
* @sa tps_GetDeviceAllInterruptEvents ()
* @sa tps_SetDeviceInterruptMask ()
* @sa tps_GetDeviceInterruptMask ()
* @sa tps_GetSystemPortNumber ()
**************************************************************************************************************************************************/
uint8_t tps_GetDeviceInterruptStatus (uint8_t device_i2c_address, uint8_t *status)
{
uint8_t rtn=0;
uint8_t value[2];
#if SPI_TO_I2C
#else
    rtn = tps_ReadI2CReg (device_i2c_address, TPS238X_INTERRUPT_COMMAND, value);
    memcpy(status,value, sizeof(value));
#endif

    return (rtn);
}

/**************************************************************************************************************************************************
*                                 Interrupt Configuration Functions
**************************************************************************************************************************************************/

/*************************************************************************************************************************************************
*  tps_SetDeviceInterruptMask
**************************************************************************************************************************************************/
/*!
* @brief Set the interrupt mask register
*
* The function sets which TPS238X events/faults are allowed to generate interrupts (unmasked).
*
* @param[in]   device_i2c_address 7 bit I2C address of the TPS238x part being controlled (do not included R/W bit)
* @param[in]   intMask            TPS238X_Interrupt_Mask_Register_t variable that contains the conditions that are allowed to generate interrupts
*                                        PEMSK_Power_Enable_Unmask
*                                        PGMSK_Power_Good_Unmask
*                                        DIMSK_Disconnect_Unmask
*                                        DEMSK_Detection_Cycle_Unmask
*                                        CLMSK_Classificiation_Cycle_Unmask
*                                        IFMSK_IFAULT_Unmask
*                                        INMSK_Inrush_Fault_Unmask
*                                        SUMSK_Supply_Event_Fault_Unmask
*
* @return  uint8_t    (I2C_SUCCESSFUL or I2C error status)
*
* @sa tps_GetDeviceInterruptMask ()
* @sa tps_GetDeviceInterruptStatus ()
* @sa tps_GetDeviceAllInterruptEvents ()
**************************************************************************************************************************************************/
uint8_t tps_SetDeviceInterruptMask (uint8_t device_i2c_address, uint8_t *intMask)
{
uint8_t rtn;
#if SPI_TO_I2C
#else
    rtn = tps_WriteI2CMultiple (device_i2c_address,TPS238X_INTERRUPT_MASK_COMMAND,intMask,2);
 #endif
    return (rtn);
}

/*************************************************************************************************************************************************
*  tps_GetDeviceInterruptMask
**************************************************************************************************************************************************/
/*!
* @brief Get the current setting of the interrupt mask register
*
* The function returns the TPS238C events/faults that are able to generate interrupts (unmasked).
*
* @param[in]   device_i2c_address  7 bit I2C address of the TPS238x part being controlled (do not included R/W bit)
* @param[out]  *intMask            Address of a TPS238X_Interrupt_Mask_Register_t variable that will the current events that can generate interrupts
*                                        PEMSK_Power_Enable_Unmask
*                                        PGMSK_Power_Good_Unmask
*                                        DIMSK_Disconnect_Unmask
*                                        DEMSK_Detection_Cycle_Unmask
*                                        CLMSK_Classificiation_Cycle_Unmask
*                                        IFMSK_IFAULT_Unmask
*                                        INMSK_Inrush_Fault_Unmask
*                                        SUMSK_Supply_Event_Fault_Unmask
* @return  uint8_t    (I2C_SUCCESSFUL or I2C error status)
*
* @sa tps_SetDeviceInterruptMask ()
* @sa tps_GetDeviceInterruptStatus ()
* @sa tps_GetDeviceAllInterruptEvents ()
* @sa tps_GetSystemPortNumber ()
**************************************************************************************************************************************************/
uint8_t tps_GetDeviceInterruptMask (uint8_t device_i2c_address, uint8_t *intMask)
{
uint8_t rtn;
uint8_t value;
#if SPI_TO_I2C
#else
    rtn = tps_ReadI2CReg (device_i2c_address, TPS238X_INTERRUPT_MASK_COMMAND,&value);
#endif
    memcpy(intMask,value, sizeof(value));
   // intMask= *(TPS238X_Interrupt_Mask_Register_t *)&value;

    return (rtn);
}

/*************************************************************************************************************************************************
*  tps_GetI2CAddresses
**************************************************************************************************************************************************/
/*!
* @brief Allows a TPS238X unique I2C addresses
*
* The system is designed so a number of TPS238X can be placed on the same I2C bus. For this to work, each device must have a unique I2C address to
* allow them to be individually commanded. .
*
* @param[in]   numDevices                Number of TPS23861 devices on the I2C bus
* @param[in]   list_ofAddresses          A pointer to a list of uint8_t 7 bit I2C addresses for each of the TPS23861 parts on the I2C bus
*                                             (do not included R/W bit)
* @param[in]   list_ofAutoMode           A pointer to a list of TPS_On_Off_t values that contain TPS_ON for the devices that operate in AUTO mode
*                                              and TPS_OFF for those that do not operate in AUTO mode
*
* @return  uint8_t    (I2C_SUCCESSFUL or I2C error status)
*
**************************************************************************************************************************************************/
uint8_t tps_GetI2CAddresses (uint8_t temp_i2cAddress, uint8_t *current_address)
{
	uint8_t rtn = 0x0;
	uint8_t value=0;
    rtn = tps_ReadI2CReg (temp_i2cAddress, TPS238X_I2C_SLAVE_ADDRESS_COMMAND, &value);

    *current_address=value;
    return rtn;

}



/*************************************************************************************************************************************************
*  tps_GetDeviceGeneralMask
**************************************************************************************************************************************************/
/*!
* @brief Get the General  mask register
*
* The function gets General Mask register updates
*
* @param[in]   device_i2c_address 7 bit I2C address of the TPS238x part being controlled (do not included R/W bit)
* *
* @return  uint8_t    (I2C_SUCCESSFUL or I2C error status)
*
**************************************************************************************************************************************************/
uint8_t tps_GetDeviceGeneralMask (uint8_t device_i2c_address, TPS238x_General_Mask_1_Register_t *status)
{
	uint8_t rtn;
	uint8_t value[2];
#if SPI_TO_I2C
#else
    rtn = tps_ReadI2CMultiple (device_i2c_address, TPS238X_GENERAL_MASK_COMMAND, value,2);
#endif
    *status++ = *(TPS238x_General_Mask_1_Register_t *)&value[0];
    *status = *(TPS238x_General_Mask_1_Register_t *)&value[1];
    return (rtn);
}

uint8_t tps_SetDeviceGeneralMask (uint8_t device_i2c_address, uint8_t * intMask)
{
	uint8_t rtn;
#if SPI_TO_I2C
#else
    rtn = tps_WriteI2CMultiple (device_i2c_address, TPS238X_GENERAL_MASK_COMMAND, intMask,2);
#endif
    return (rtn);
}

/*************************************************************************************************************************************************
*  tps_SetDevicePowerOn
**************************************************************************************************************************************************/
/*!
* @brief Configures the power on state for each of the device ports on a specific TPS23861
*
* This function will power on multiple ports at the same time. Each of the ports indicated in the TPS23861 will be powered on.
*
* @param[in]   device_i2c_address        7 bit I2C address of the TPS238x part being controlled (do not included R/W bit)
* @param[in]   portsPoweredOn            A TPS238x_Ports_t variable that identifies the ports that will be powered on.
*
* @return  uint8_t    (I2C_SUCCESSFUL or I2C error status)
*
* @sa tps_SetDevicePowerOff ()
* @sa tps_SetPortPower ()
* @sa tps_GetSystemPortNumber ()
**************************************************************************************************************************************************/
uint8_t tps_SetDevicePowerOn (uint8_t device_i2c_address, TPS238x_Ports_t portsPoweredOn)
{
uint8_t rtn;
uint8_t value[2];

    value[0]=0x1;
    value[1]=0x0;
    rtn = tps_WriteI2CMultiple (device_i2c_address, TPS238X_POWER_ENABLE_COMMAND, value,sizeof(value));

    return (rtn);
}

/*************************************************************************************************************************************************
*  tps_SetDevicePowerOff
**************************************************************************************************************************************************/
/*!
* @brief Configures the power down (off) state for each of the device ports on a specific TPS23861
*
* This function will power down (off) multiple ports at the same time. Each of the ports indicated in the TPS23861 will be powered off.
*
* @param[in]   device_i2c_address        7 bit I2C address of the TPS238x part being controlled (do not included R/W bit)
* @param[in]   portsPoweredOff           A TPS238x_Ports_t variable that identifies the ports that will be powered off.
*
* @return  uint8_t    (I2C_SUCCESSFUL or I2C error status)
*
* @sa tps_SetDevicePowerOn ()
* @sa tps_SetPortPower ()
* @sa tps_GetSystemPortNumber ()
**************************************************************************************************************************************************/
uint8_t tps_SetDevicePowerOff (uint8_t device_i2c_address, TPS238x_Ports_t portsPoweredOff)
{
uint8_t rtn;
uint8_t value[2];

    value[0] = (*(uint8_t*)&portsPoweredOff & 0x0F) << POWER_OFF_SHIFT;
    value[1] = ((*(uint8_t*)&portsPoweredOff & 0xF0) >> 4) << POWER_OFF_SHIFT;
    rtn = tps_WriteI2CMultiple (device_i2c_address, TPS238X_POWER_ENABLE_COMMAND, value,sizeof(value));

    return (rtn);
}

/*************************************************************************************************************************************************
*  tps_SetPortPower
**************************************************************************************************************************************************/
/*!
* @brief Power on or off a single specified system port
*
* This function will either power on or power down (off) a single port on one of the registered TPS23861 ports in the system.
*
* @param[in]   systemPortNum          Port Number Value returned from the port registration function, tps_RegisterPort()
* @param[in]   on_off                 Indicates whether to power on (TPS_ON) or power off (TPS_OFF) the specified port
*
* @return  uint8_t    (I2C_SUCCESSFUL or I2C error status)
*
* @sa tps_SetDevicePowerOn ()
* @sa tps_SetDevicePowerOff ()
**************************************************************************************************************************************************/
uint8_t tps_SetPortPower (uint8_t systemPortNum, TPS238x_On_Off_t on_off)
{
uint8_t rtn;
uint8_t value[2];
uint8_t portBit = tps_GetDevicePortNum(systemPortNum);

  //  portBit = CONVERT_PORT_NUM(portBit);

    // This TPS register is Write Only with bits for forcing a Power Off and separate bits for Power On
    if (on_off == TPS_OFF)
        value[0] = (portBit << POWER_OFF_SHIFT);
    else
        value[0] = (portBit << POWER_ON_SHIFT);

  //  rtn = tps_WriteI2CReg (tps_GetDeviceI2CAddress(systemPortNum), TPS238X_POWER_ENABLE_COMMAND, value[0],value[1]);

    return (rtn);
}

/*************************************************************************************************************************************************
*  tps_SetPortOpMode
**************************************************************************************************************************************************/
/*!
* @brief Set the Operating Mode of a single port
*
* The TPS23861 has a number of operating modes that determine the characteristic capabilities of a given port. This function allows the user to configure the
* operating for a given port.
*
* @param[in]   systemPortNum             Port Number Value returned from the port registration function, tps_RegisterPort()
* @param[in]   operatingMode             Define the operating mode for the port
*                                            [OPERATING_MODE_OFF, OPERATING_MODE_MANUAL, OPERATING_MODE_SEMI_AUTO, or OPERATING_MODE_AUTO]
*
* @return  uint8_t    (I2C_SUCCESSFUL or I2C error status)
*
* @sa tps_SetDeviceOpMode ()
* @sa tps_ConfigPort ()
**************************************************************************************************************************************************/
uint8_t tps_SetPortOpMode (uint8_t systemPortNum, TPS238x_Operating_Modes_t operatingMode)
{
uint8_t rtn;
uint8_t value;
uint8_t portNum = tps_GetDevicePortNum(systemPortNum);

    // Load the Operating Mode
 //   rtn = tps_ReadI2CReg (tps_GetDeviceI2CAddress(systemPortNum), TPS238X_OPERATING_MODE_COMMAND, &value);
    value &= ~(OPERATING_MODE_MASK << (2 * ((uint8_t) portNum - 1)));               // Clear old value for this port number
    value |= ((uint8_t)operatingMode << (2 * ((uint8_t) portNum - 1)));             // Each port gets 2 bits
   // rtn += tps_WriteI2CReg (tps_GetDeviceI2CAddress(systemPortNum), TPS238X_OPERATING_MODE_COMMAND, value,value);

    return (rtn);
}

/*************************************************************************************************************************************************
*  tps_SetDeviceOpMode
**************************************************************************************************************************************************/
/*!
* @brief Set the Operating Mode for all of the ports on the TPS238
*
* The TPS23861 has a number of operating modes that determine the characteristic capabilities of a given port. This function allows the user to configure the
* operating for all of the ports.
*
* @param[in]   device_i2c_address        7 bit I2C address of the TPS238x part being controlled (do not included R/W bit)
* @param[in]   operatingMode1            Define the operating mode for port 1
*                                            [OPERATING_MODE_OFF, OPERATING_MODE_MANUAL, OPERATING_MODE_SEMI_AUTO, or OPERATING_MODE_AUTO]
* @param[in]   operatingMode2            Define the operating mode for port 2
*                                            [OPERATING_MODE_OFF, OPERATING_MODE_MANUAL, OPERATING_MODE_SEMI_AUTO, or OPERATING_MODE_AUTO]
* @param[in]   operatingMode3            Define the operating mode for port 3
*                                            [OPERATING_MODE_OFF, OPERATING_MODE_MANUAL, OPERATING_MODE_SEMI_AUTO, or OPERATING_MODE_AUTO]
* @param[in]   operatingMode4            Define the operating mode for port 4
*                                            [OPERATING_MODE_OFF, OPERATING_MODE_MANUAL, OPERATING_MODE_SEMI_AUTO, or OPERATING_MODE_AUTO]
*
* @return  uint8_t    (I2C_SUCCESSFUL or I2C error status)
*
* @sa tps_SetPortOpMode ()
* @sa tps_ConfigPort ()
**************************************************************************************************************************************************/
uint8_t tps_SetDeviceOpMode (uint8_t device_i2c_address, TPS238x_Operating_Modes_t operatingMode)
{
uint8_t rtn;
uint8_t value[2];

    // Load the Operating Mode
    value[0] = (((uint8_t)operatingMode << 6) | ((uint8_t)operatingMode << 4) | ((uint8_t)operatingMode << 2) | ((uint8_t)operatingMode));  // Each port gets 2 bits
    value[1] = (((uint8_t)operatingMode << 6) | ((uint8_t)operatingMode << 4) | ((uint8_t)operatingMode << 2) | ((uint8_t)operatingMode));
    rtn = tps_WriteI2CMultiple(device_i2c_address, TPS238X_OPERATING_MODE_COMMAND, value,sizeof(value));

    return (rtn);
}

/**************************************************************************************************************************************************/
uint8_t tps_GetDeviceOpMode (uint8_t device_i2c_address, TPS238x_Operating_Modes_t *operatingMode)
{
uint8_t rtn;
uint8_t value[2];

    // Load the Operating Mode
    rtn = tps_ReadI2CMultiple (device_i2c_address,TPS238X_OPERATING_MODE_COMMAND, value,0x2);
    *operatingMode= *(TPS238x_Operating_Modes_t *)&value;

    return (rtn);
}

/*************************************************************************************************************************************************
*  tps_GetPortDetectionStatus
**************************************************************************************************************************************************/
/*!
* @brief Returns the detection status of the specified port
*
* This function will return the most recent detection result for the indicated port on the TPS23861.
*
* The function will return a 0 (CLASS_UNKNOWN) when the port is turned off.
*
* @param[in]   systemPortNum          Port Number Value returned from the port registration function, tps_RegisterPort()
* @param[out]  *detectionStatus       Address of a TPS238x_Detection_Status_t variable that will receive the detecttion status
*                                            for the indicated port (DETECT_UNKNOWN, DETECT_SHORT_CIRCUIT, DETECT_RESIST_LOW,
*                                            DETECT_RESIST_VALID, DETECT_RESIST_HIGH, DETECT_OPEN_CIRCUIT, DETECT_MOSFET_FAULT,
*                                            DETECT_LEGACY, DETECT_CAP_INVALID_CLAMP_VOLTAGE, DETECT_CAP_INVALID_DELTA_V, or
*                                            DETECT_CAP_INVALID_LEGACY_RANGE)
*
* @return  uint8_t    (I2C_SUCCESSFUL or I2C error status)
*
* @sa tps_GetPortClassificationStatus ()
* @sa tps_GetPortDetectClassStatus ()
**************************************************************************************************************************************************/
uint8_t tps_GetPortDetectionStatus (uint8_t systemPortNum, TPS238x_Detection_Status_t *detectionStatus)
{
uint8_t rtn;
uint8_t value;
uint8_t portNum = tps_GetDevicePortNum(systemPortNum);
uint8_t command = TPS238X_CHANNEL_1_STATUS_COMMAND + (uint8_t) portNum - 1;   // Determine which Port Status command has been requested

    rtn = tps_ReadI2CReg (tps_GetDeviceI2CAddress(systemPortNum), command, &value);

    *detectionStatus = (TPS238x_Detection_Status_t)GET_DETECT(value);

    return (rtn);
}

/*************************************************************************************************************************************************
*  tps_GetPortClassificationStatus
**************************************************************************************************************************************************/
/*!
* @brief Returns the classification status of the specified port
*
* This function will return the most recent classification result for the indicated port on the TPS23861.
*
* The function will return a 0 (CLASS_UNKNOWN) when the port is turned off.
*
* @param[in]   systemPortNum          Port Number Value returned from the port registration function, tps_RegisterPort()
* @param[out]  *classificationStatus  Address of a TPS238x_Classification_Status_t variable that will receive the classification status
*                                            for the indicated port (CLASS_UNKNOWN, CLASS_1, CLASS_2, CLASS_3, CLASS_4,
*                                            CLASS_0, CLASS_OVERCURRENT, CLASS_MISMATCH, or CLASS_5)
*
* @return  uint8_t    (I2C_SUCCESSFUL or I2C error status)
*
* @sa tps_GetPortDetectionStatus ()
* @sa tps_GetPortDetectClassStatus ()
**************************************************************************************************************************************************/
uint8_t tps_GetPortClassificationStatus (uint8_t systemPortNum, TPS238x_Classification_Status_t *classificationStatus)
{
uint8_t rtn;
uint8_t value;
uint8_t portNum = tps_GetDevicePortNum(systemPortNum);
uint8_t command = TPS238X_CHANNEL_1_STATUS_COMMAND + (uint8_t) portNum - 1;   // Determine which Port Status command has been requested

    rtn = tps_ReadI2CReg (tps_GetDeviceI2CAddress(systemPortNum), command, &value);

    *classificationStatus = (TPS238x_Classification_Status_t)GET_CLASS(value);

    return (rtn);
}

/*************************************************************************************************************************************************
*  tps_GetDevicePowerStatus
**************************************************************************************************************************************************/
/*!
* @brief Returns the power enable and power good status of the 4 ports in the TPS23861
*
* This function will return both the power enable and the power good state of each port.
*
* @param[in]   device_i2c_address        7 bit I2C address of the TPS238x part being controlled (do not included R/W bit)
* @param[out]  *powerEnablePorts         Address of a TPS238x_Ports_t variable that will receive the power enable status with
*                                            one bit for each port in power enabled state.
* @param[out]  *powerGoodPorts           Address of a TPS238x_Ports_t variable that will receive the power good status with
*                                            one bit for each port in power good state.
*
* @return  uint8_t    (I2C_SUCCESSFUL or I2C error status)
*
* @sa tps_GetDevicePowerEnableStatus ()
* @sa tps_GetPortPowerEnableStatus ()
* @sa tps_GetDevicePowerGoodStatus ()
* @sa tps_GetPortPowerGoodStatus ()
* @sa tps_GetSystemPortNumber ()
**************************************************************************************************************************************************/
uint8_t tps_GetDevicePowerStatus (uint8_t device_i2c_address, TPS238x_Ports_t *powerEnablePorts, TPS238x_Ports_t *powerGoodPorts)
{
uint8_t rtn;
uint8_t value;
uint8_t value_out;

    rtn = tps_ReadI2CReg (device_i2c_address, TPS238X_POWER_STATUS_COMMAND, &value);

    value_out = GET_POWER_ENABLE_STATUS (value);
    *powerEnablePorts = value_out;

    value_out = GET_POWER_GOOD_STATUS(value);
    *powerGoodPorts = value_out;

    return (rtn);
}

/*************************************************************************************************************************************************
*  tps_GetDevicePowerEnableStatus
**************************************************************************************************************************************************/
/*!
* @brief Returns the power enable status of the 4 ports in the TPS23861
*
* This function will return the power good state of each port.
*
* @param[in]   device_i2c_address        7 bit I2C address of the TPS238x part being controlled (do not included R/W bit)
* @param[out]  *powerEnablePorts         Address of a TPS238x_Ports_t variable that will receive the power enable status with
*                                            one bit for each port in power enabled state.
*
* @return  uint8_t    (I2C_SUCCESSFUL or I2C error status)
*
* @sa tps_GetPortPowerEnableStatus ()
* @sa tps_GetDevicePowerGoodStatus ()
* @sa tps_GetPortPowerGoodStatus ()
* @sa tps_GetDevicePowerStatus ()
* @sa tps_GetSystemPortNumber ()
**************************************************************************************************************************************************/
uint8_t tps_GetDevicePowerEnableStatus (uint8_t device_i2c_address, TPS238x_Ports_t *powerEnablePorts)
{
uint8_t rtn;
uint8_t value;

    rtn = tps_ReadI2CReg (device_i2c_address, TPS238X_POWER_STATUS_COMMAND, &value);

    value = GET_POWER_ENABLE_STATUS(value);
    *powerEnablePorts = *(TPS238x_Ports_t*)&value;

    return (rtn);
}


/*************************************************************************************************************************************************
*  tps_GetDevicePowerGoodStatus
**************************************************************************************************************************************************/
/*!
* @brief Returns the power good status of the 4 ports in the TPS23861
*
* This function will return the power good state of each port.
*
* @param[in]   device_i2c_address        7 bit I2C address of the TPS238x part being controlled (do not included R/W bit)
* @param[out]  *powerGoodPorts           Address of a TPS238x_Ports_t variable that will receive the power good status with
*                                            one bit for each port in power good state.
*
* @return  uint8_t    (I2C_SUCCESSFUL or I2C error status)
*
* @sa tps_GetPortPowerGoodStatus ()
* @sa tps_GetDevicePowerEnableStatus ()
* @sa tps_GetPortPowerEnableStatus ()
* @sa tps_GetDevicePowerStatus ()
* @sa tps_GetSystemPortNumber ()
**************************************************************************************************************************************************/
uint8_t tps_GetDevicePowerGoodStatus (uint8_t device_i2c_address, TPS238x_Ports_t *powerGoodPorts)
{
uint8_t rtn;
uint8_t value;

    rtn = tps_ReadI2CReg (device_i2c_address, TPS238X_POWER_STATUS_COMMAND, &value);

    value = GET_POWER_GOOD_STATUS(value);
    *powerGoodPorts = value;

    return (rtn);
}

/*************************************************************************************************************************************************
*  tps_GetPortPowerGoodStatus
**************************************************************************************************************************************************/
/*!
* @brief Returns the power good status of one of the 4 ports in the TPS23861
*
* This function will return the power good state of the indicated port.
*
* @param[in]   systemPortNum          Port Number Value returned from the port registration function, tps_RegisterPort()
*
* @return  uint8_t     TPS_ON if Power Good
*                      TPS_OFF is Power is NOT good,
*                      TPS_ERR_I2C_ERROR on I2C error status
*
* @sa tps_GetDevicePowerGoodStatus ()
* @sa tps_GetDevicePowerEnableStatus ()
* @sa tps_GetPortPowerEnableStatus ()
* @sa tps_GetDevicePowerStatus ()
* @sa tps_GetSystemPortNumber ()
**************************************************************************************************************************************************/
uint8_t tps_GetPortPowerGoodStatus (uint8_t systemPortNum)
{
uint8_t rtn;
uint8_t value;
uint8_t portNum = tps_GetDevicePortNum(systemPortNum);

    rtn = tps_ReadI2CReg (tps_GetDeviceI2CAddress(systemPortNum), TPS238X_POWER_STATUS_COMMAND, &value);


    value = GET_POWER_GOOD_STATUS(value);
    if (value & CONVERT_PORT_NUM(portNum))
        return (TPS_ON);
    else
        return (TPS_OFF);

}
/*************************************************************************************************************************************************
*  tps_GetdevicePowerEventStatus
**************************************************************************************************************************************************/
/*!
* @brief Returns the power Event status of  TPS238x
*
* This function will return the Events for  power status.
*
* @param[in] device_i2c_address        7 bit I2C address of the TPS238x part being controlled (do not included R/W bit)
*
* @return  uint8_t    Power good status change event
                      Power enable status change event

*
* @sa tps_GetDevicePowerGoodStatus ()
* @sa tps_GetDevicePowerEnableStatus ()
* @sa tps_GetPortPowerEnableStatus ()
* @sa tps_GetDevicePowerStatus ()
* @sa tps_GetSystemPortNumber ()
**************************************************************************************************************************************************/

uint8_t tps_GetdevicePowerEventStatus (uint8_t device_i2c_address,uint8_t ClearOnRead,uint8_t *Value)
{
uint8_t rtn;
uint8_t powerevent[2];

    rtn = tps_ReadI2CReg (device_i2c_address, TPS238X_POWER_EVENT_COMMAND+ClearOnRead, powerevent);
    memcpy(Value,powerevent, sizeof(powerevent));
    return rtn;
}

/*************************************************************************************************************************************************
*  tps_GetPowerSupplystatus
**************************************************************************************************************************************************/
/*!
* @brief Returns the power supply Event status of  TPS238x
*
* This function will return the Events for  power supply status.
*
* @param[in] device_i2c_address        7 bit I2C address of the TPS238x part being controlled (do not included R/W bit)
*
* @return  uint8_t     VDD UnderVoltage event
                       VPWR Undervoltage event 
                       Thermal shutdown Event
                       VDD UnderVoltage Threshold Warning
*
* @sa tps_GetDevicePowerGoodStatus ()
* @sa tps_GetDevicePowerEnableStatus ()
* @sa tps_GetPortPowerEnableStatus ()
* @sa tps_GetDevicePowerStatus ()
* @sa tps_GetSystemPortNumber ()
**************************************************************************************************************************************************/
uint8_t tps_GetdevicePowerSupplyStatus (uint8_t device_i2c_address,uint8_t ClearOnRead,uint8_t *Value)
{
uint8_t rtn;
uint8_t event[2];

    rtn = tps_ReadI2CReg (device_i2c_address, TPS238X_SUPPLY_EVENT_COMMAND+ClearOnRead, event);
    memcpy(Value,event, sizeof(event));
    return rtn;
}

/**************************************************************************************************************************************************/
uint8_t tps_GetdevicePoweronFaultStatus (uint8_t device_i2c_address,uint8_t ClearOnRead,uint8_t *Value)
{
uint8_t rtn;
uint8_t event[2];

    rtn = tps_ReadI2CReg (device_i2c_address, TPS238X_POWER_ON_FAULT_EVENT_COMMAND+ClearOnRead, event);
    memcpy(Value,event, sizeof(event));
    return rtn;
}
/*************************************************************************************************************************************************
*  tps_GetdeviceClassandDetectStatus
**************************************************************************************************************************************************/
/*!
* @brief Returns the Device Classification and Detection status of  TPS238x
*
* This function will return the Events for  power supply status.
*
* @param[in] device_i2c_address        7 bit I2C address of the TPS238x part being controlled (do not included R/W bit)
*
* @return  uint8_t     Classification cycle occurred
* 		   uint8_t	   Detection cycle occurred
*
* @sa tps_GetDevicePowerGoodStatus ()
* @sa tps_GetDevicePowerEnableStatus ()
* @sa tps_GetPortPowerEnableStatus ()
* @sa tps_GetDevicePowerStatus ()
* @sa tps_GetSystemPortNumber ()
**************************************************************************************************************************************************/

uint8_t tps_GetdeviceClassandDetectStatus (uint8_t device_i2c_address,uint8_t ClearOnRead,uint8_t *class_event, uint8_t *detect_event)
{
uint8_t rtn;
uint8_t event[2];

    rtn = tps_ReadI2CReg (device_i2c_address, TPS238X_DETECTION_EVENT_COMMAND+ClearOnRead, event);
    *class_event=(( event[0] >> 4) & 0xF) + (event[1] & 0xF0);
    *detect_event= (event[0] & 0xF) + ((event[1] & 0xF) <<4);
    return rtn;
}
/*************************************************************************************************************************************************
*  tps_GetdeviceFaultStatus
**************************************************************************************************************************************************/
/*!
* @brief Returns the Fault Event status of  TPS238x
*
* This function will return the Events for  Fault.
*
* @param[in] device_i2c_address        7 bit I2C address of the TPS238x part being controlled (do not included R/W bit)
*
* @return  uint8_t     Disconnect event
* 					   tOVLD Fault event
*
* @sa tps_GetDevicePowerGoodStatus ()
* @sa tps_GetDevicePowerEnableStatus ()
* @sa tps_GetPortPowerEnableStatus ()
* @sa tps_GetDevicePowerStatus ()
* @sa tps_GetSystemPortNumber ()
**************************************************************************************************************************************************/

uint8_t tps_Getdevice_disconnect_faultStatus (uint8_t device_i2c_address,uint8_t ClearOnRead,uint8_t *disconnect_event)
{
uint8_t rtn;
uint8_t event[2];

    rtn = tps_ReadI2CReg (device_i2c_address, TPS238X_FAULT_EVENT_COMMAND+ClearOnRead, event);
    *disconnect_event= ((event[0] & 0xF0) >>4 )+ (event [1] & 0xF0);
    return rtn;
}

uint8_t tps_Getdevice_overcurrenttimeout_faultStatus (uint8_t device_i2c_address,uint8_t ClearOnRead,uint8_t *tovld_event)
{
uint8_t rtn;
uint8_t event[2];

    rtn = tps_ReadI2CReg (device_i2c_address, TPS238X_FAULT_EVENT_COMMAND+ClearOnRead, event);
    *tovld_event= (event[0] & 0xF)+ ((event [1] & 0xF) <<4 );
    return rtn;
}
/*************************************************************************************************************************************************
*  tps_GetdeviceStartandLimitStatus
**************************************************************************************************************************************************/
/*!
* @brief Returns the Tstart and TLim fault status of  TPS238x
*
* This function will return the Events for Tstart and TLim fault .
*
* @param[in] device_i2c_address        7 bit I2C address of the TPS238x part being controlled (do not included R/W bit)
*
* @return  uint8_t     tLIM fault occurred or Not
* 					   tSTART fault or class/detect error occurred
*
* @sa tps_GetDevicePowerGoodStatus ()
* @sa tps_GetDevicePowerEnableStatus ()
* @sa tps_GetPortPowerEnableStatus ()
* @sa tps_GetDevicePowerStatus ()
* @sa tps_GetSystemPortNumber ()
**************************************************************************************************************************************************/
uint8_t tps_GetdeviceStartandLimitStatus (uint8_t device_i2c_address,uint8_t ClearOnRead,uint8_t *limitevent)
{
uint8_t rtn;
uint8_t event[2];

    rtn = tps_ReadI2CReg (device_i2c_address, TPS238X_START_LIMIT_EVENT_COMMAND+ClearOnRead, event);
    *limitevent= (event[0] & 0xF) +(event[1] & 0xF << 4);
    return rtn;
}

uint8_t tps_SetDeviceDetandClassEnable (uint8_t device_i2c_address, uint8_t det_enable,uint8_t class_enable)
{
uint8_t rtn;
uint8_t value[2];

    value[0]=(det_enable & 0x0F) + ((class_enable & 0x0F) << 4);
    value[1]=((det_enable & 0xF0)>>4) + (class_enable & 0xF0);
    rtn = tps_WriteI2CMultiple (device_i2c_address, TPS238X_DETECT_CLASS_ENABLE_COMMAND, value,sizeof(value));

    return (rtn);
}

uint8_t tps_SetDeviceDisconnectEnable (uint8_t device_i2c_address, uint8_t enable)
{
uint8_t rtn;
uint8_t value[2];

    value[0]=enable & 0x0F;
    value[1]=(enable & 0xF0) >> 4;
    rtn = tps_WriteI2CMultiple (device_i2c_address, TPS238X_DISCONNECT_ENABLE_COMMAND, value,sizeof(value));

    return (rtn);
}

/*************************************************************************************************************************************************
*  tps_GetPortDetectClassStatus
**************************************************************************************************************************************************/
/*!
* @brief Returns the detection and classification status of the specified port
*
* This function will return the most recent detection and classification result for the indicated port on the TPS23861.
*
* The function will return a 0 (CLASS_UNKNOWN) when the port is turned off.
*
* @param[in]   device_i2c_address          I2C address of the TPS2388
* @param[in]   port_num          		   Associated port number of the TPS2388
* @param[out]  *detectionStatus       Address of a TPS238x_Detection_Status_t variable that will receive the detection status
*                                            for the indicated port (DETECT_UNKNOWN, DETECT_SHORT_CIRCUIT, DETECT_RESIST_LOW,
*                                            DETECT_RESIST_VALID, DETECT_RESIST_HIGH, DETECT_OPEN_CIRCUIT, DETECT_MOSFET_FAULT,
*                                            DETECT_LEGACY, DETECT_CAP_INVALID_CLAMP_VOLTAGE, DETECT_CAP_INVALID_DELTA_V, or
*                                            DETECT_CAP_VALID_LEGACY_RANGE)
* @param[out]  *classificationStatus  Address of a TPS238x_Classification_Status_t variable that will receive the classification status
*                                            for the indicated port (CLASS_UNKNOWN, CLASS_1, CLASS_2, CLASS_3, CLASS_4,
*                                            CLASS_0, CLASS_OVERCURRENT, CLASS_MISMATCH, or CLASS_5)
*
* @return  uint8_t    (I2C_SUCCESSFUL or I2C error status)
*
* @sa tps_GetPortDetectionStatus ()
* @sa tps_GetPortClassificationStatus ()
**************************************************************************************************************************************************/
uint8_t tps_GetPortDetectClassStatus (uint8_t device_i2c_address, TPS238x_PortNum_t port_num,TPS238x_Detection_Status_t *detectionStatus,
                                      TPS238x_Classification_Status_t *classificationStatus)
{
uint8_t rtn;
uint8_t value[2];
//uint8_t portNum = tps_GetDevicePortNum(systemPortNum);
uint8_t command = TPS238X_CHANNEL_1_STATUS_COMMAND + (port_num%5) - 1;   // Determine which Port Status command has been requested

    rtn = tps_ReadI2CReg (device_i2c_address, command, value);

    *classificationStatus = (TPS238x_Classification_Status_t)GET_CLASS(value[0])+ ((TPS238x_Classification_Status_t)GET_CLASS(value[1]) << 4);
    *detectionStatus = (TPS238x_Detection_Status_t)GET_DETECT(value[0])+ (TPS238x_Detection_Status_t)GET_DETECT(value[1] << 4);

    return (rtn);
}
/*************************************************************************************************************************************************
*  tps_GetPortPowerEnableStatus
**************************************************************************************************************************************************/
/*!
* @brief Returns the power enable status of one of the 4 ports in the TPS23861
*
* This function will return the power enable state of the indicated port.
*
* @param[in]   device_i2c_address
*
* @return  uint8_t     TPS_ON if Power Enabled
*                      TPS_OFF is Power is NOT enabled,
*                      TPS_ERR_I2C_ERROR on I2C error status
*
* @sa tps_GetDevicePowerEnableStatus ()
* @sa tps_GetDevicePowerGoodStatus ()
* @sa tps_GetPortPowerGoodStatus ()
* @sa tps_GetDevicePowerStatus ()
* @sa tps_GetSystemPortNumber ()
**************************************************************************************************************************************************/
uint8_t tps_GetPortPowerEnableStatus (uint8_t device_i2c_address, uint8_t *port_state, uint8_t *port_power)
{
uint8_t rtn;
uint8_t value[2];

    rtn = tps_ReadI2CReg (device_i2c_address, TPS238X_POWER_STATUS_COMMAND, value);
    *port_state= value[0] & 0x0F + ((value[1] & 0x0F) << 4);
    *port_power= ((value[0] & 0xF0) >>4) + (value[1] & 0xF0);
    return rtn;

}

//uint8_t tps_SetPortIEEEPowerEnable_Type2 (uint8_t device_i2c_address,uint8_t port_num)
//{
//uint8_t rtn;
//uint8_t value[2];
//
//	if (port_num <=4)
//	{
//		value[0] = 1 << (port_num-1+4);
//		value[1]= 0x0;
//	}
//	else
//	{
//		value[0]=0x0;
//		value[1]=(1 << (port_num-5+4));
//	}
//
//    rtn = tps_WriteI2CMultiple (device_i2c_address, TPS238X_IEEE_POWER_ENABLE_COMMAND, value,sizeof(value));
//
//    return (rtn);
//}

uint8_t tps_SetPortIEEEPowerEnable_Type1 (uint8_t device_i2c_address,uint8_t port_num)
//{
//	uint8_t rtn;
//	uint8_t value[2];
//	if (port_num <=4)
//		{
//			value[0] = 1 << (port_num-1);
//			value[1]= 0x0;
//		}
//		else
//		{
//			value[0]=0x0;
//			value[1]=(1 << (port_num-5));
//		}
//
//	 rtn = tps_WriteI2CMultiple (device_i2c_address, TPS238X_IEEE_POWER_ENABLE_COMMAND, value,sizeof(value));
//
//	return (rtn);
//}


/**************************************************************************************************************************************************
*                                 System Measurements
**************************************************************************************************************************************************/

/*************************************************************************************************************************************************
*  tps_GetPortMeasurements
**************************************************************************************************************************************************/
/*!
* @brief Returns the voltage and current of the registered system port.
*
* This function will return the voltage and current on the registered port on a specific TPS23861.
*
* @param[in]   systemPortNum             Port Number Value returned from the port registration function, tps_RegisterPort()
* @param[out]  *voltage                  The address of a uint16_t variable where the voltage of the indicated port will be placed. The voltage
*                                           will be a scaled integer with an LSB of 3.662 mVolts
* @param[out]  *current                  The address of a uint16_t variable where the current of the indicated port will be placed. The current
*                                           will be a scaled integer with an LSB that is dependent on the current-sense resistor.
*                                           For a 250 mOhm resistor, the LSB will be 62.260 micro-Amps.
*                                           For a 255 mOhm resistor, the LSB will be 61.039 micro-Amps.
*
* @return  uint8_t    (I2C_SUCCESSFUL or I2C error status)
*
* @sa tps_GetDeviceInputVoltage ()
**************************************************************************************************************************************************/
uint8_t tps_GetPortMeasurements (uint8_t systemPortNum, uint16_t *voltage, uint16_t *current)
{
uint8_t rtn;
uint8_t command;
uint16_t value;
uint8_t portNum = tps_GetDevicePortNum(systemPortNum);

    command = TPS238X_CHANNEL_1_CURRENT_COMMAND + (4 * ((uint8_t) portNum - 1));
    rtn = tps_ReadI2CMultiple (tps_GetDeviceI2CAddress(systemPortNum), command, (uint8_t *)&value, 2);
    *current = value & TPS2368X_PORT_CURRENT_MASK_SHORT;

    command = TPS238X_CHANNEL_1_VOLTAGE_COMMAND + (4 * ((uint8_t) portNum - 1));
    rtn += tps_ReadI2CMultiple (tps_GetDeviceI2CAddress(systemPortNum), command, (uint8_t *)&value, 2);
    *voltage = value & TPS2368X_CHANNEL_VOLTAGE_MASK_SHORT;

    return (rtn);
}


uint8_t valueTest[2] = {0,0};
void tps_Init(uint8_t NUM_OF_TPS2388,unsigned char *tps2388_i2cAddList)
{

      uint8_t i;
//    uint8_t i,j,rtn,data;
//    uint8_t current_address,addr_bits;
//    uint8_t value[2];
    // Check the i2c address to discover the PSE part.
	for (i=0; i < NUM_OF_TPS2388; i++)
	{
//		//Need to recheck this function
//		{
//		rtn=tps_GetI2CAddresses(0x7F,&current_address);
//		}
//
//		// if We get a ACK from TPS2388 ,continue configure the I2C address of the PSE
//		if (rtn && 0x1)
//		{
//			if(current_address & 0x4)
//			{
//				for (j=0;j <=15;j++)
//				{
//					tps2388_binaryAddrList[j]|=0x1;
//				}
//			}
//			addr_bits=(current_address >> 3) & 0xF;
//			tps2388_i2cAddList[i]=tps2388_binaryAddrList[addr_bits];
//		}
//		else
//		{
//			if (rtn && 0x20)
//			{
//				//I2C NACK STATUS
//			}
//
//		}
		//tps_GetDeviceGeneralMask(tps2388_i2cAddList[i],&value);
		value[0]= DEMSK_Detection_Cycle_Unmask|CLMSK_Classificiation_Cycle_Unmask|PEMSK_Power_Enable_Unmask|PGMSK_Power_Good_Unmask|SUMSK_Supply_Event_Fault_Unmask;
		value[1]= DEMSK_Detection_Cycle_Unmask|CLMSK_Classificiation_Cycle_Unmask|PEMSK_Power_Enable_Unmask|PGMSK_Power_Good_Unmask|SUMSK_Supply_Event_Fault_Unmask;
		tps_SetDeviceInterruptMask(tps2388_i2cAddList[i],value);
		tps_GetDeviceInterruptMask(tps2388_i2cAddList[i],value);
		value[0] =General_Mask_Access_Configuration|INTEN_INT_Pin_Mask;
		value[1] =General_Mask_Access_Configuration|INTEN_INT_Pin_Mask;
		tps_SetDeviceGeneralMask(tps2388_i2cAddList[i],value);

		// tsai
//		tps_GetDeviceGeneralMask(tps2388_i2cAddList[i],value);
		tps_GetDeviceGeneralMask(tps2388_i2cAddList[i], (TPS238x_General_Mask_1_Register_t *)&valueTest[0]);

		value[0]=TPS238X_ALL_PORTS;
        tps_SetDevicePowerOff(tps2388_i2cAddList[i], value[0]);

        // tsai
        tps_GetDeviceOpMode(tps2388_i2cAddList[i], (TPS238x_Operating_Modes_t *)&value[0]);

        tps_SetDeviceOpMode(tps2388_i2cAddList[i],OPERATING_MODE_SEMI_AUTO);
        value[0]=TPS238X_ALL_PORTS;
        value[1]=TPS238X_ALL_PORTS;
        tps_SetDeviceDetandClassEnable(tps2388_i2cAddList[i],TPS238X_ALL_PORTS,TPS238X_ALL_PORTS);
        tps_SetDeviceDisconnectEnable(tps2388_i2cAddList[i],TPS238X_ALL_PORTS);
        tps_GetdevicePowerSupplyStatus(tps2388_i2cAddList[i],0x1,value);
        tps_GetdevicePowerEventStatus(tps2388_i2cAddList[i],0x1,value);
        tps_GetdeviceClassandDetectStatus(tps2388_i2cAddList[i],0x1,value[0],value[1]);
        tps_Getdevice_disconnect_faultStatus(tps2388_i2cAddList[i],0x1,value);
        tps_GetdeviceStartandLimitStatus(tps2388_i2cAddList[i],0x1,value);

	}
}
#endif
