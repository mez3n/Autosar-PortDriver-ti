/******************************************************************************
 *
 * Module: Port
 *
 * File Name: Port.c
 *
 * Description: Source file for TM4C123GH6PM Microcontroller - Port Driver.
 *
 * Author: Mazen Adel
 ******************************************************************************/

#include "Port.h"
#include "tm4c123gh6pm_registers.h"
#include "Port_Cfg.h"



#if (PORT_DEV_ERROR_DETECT == STD_ON)

#include "Det.h"
/* AUTOSAR Version checking between Det and Port Modules */
#if ((DET_AR_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION)\
        || (DET_AR_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION)\
        || (DET_AR_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
#error "The AR version of Det.h does not match the expected version"
#endif

#endif


/************************************************************************************
                                STATIC VARIABLES
 ************************************************************************************/

static const Port_ConfigType * Port_Configurations =NULL_PTR;
static uint8 Port_Status = PORT_NOT_INITIALIZED;
/************************************************************************************
                                HELPER FUNCTIONS
 ************************************************************************************/

static void GetPort(const Port_PinConfigType* PinConfigPtr,volatile uint32 ** PortGpio_Ptr){
    switch(PinConfigPtr->port_num)
    {
    case  0: *PortGpio_Ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS; /* PORTA Base Address */
    break;
    case  1: *PortGpio_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS; /* PORTB Base Address */
    break;
    case  2: *PortGpio_Ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */
    break;
    case  3: *PortGpio_Ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS; /* PORTD Base Address */
    break;
    case  4: *PortGpio_Ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */
    break;
    case  5: *PortGpio_Ptr = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS; /* PORTF Base Address */
    break;
    }

}


static void UnlockLockedPins(const Port_PinConfigType* PinConfigPtr,const volatile uint32 * PortGpio_Ptr){

    if( ((PinConfigPtr->port_num == 3) && (PinConfigPtr->pin_id == 7)) || ((PinConfigPtr->port_num == 5) && (PinConfigPtr->pin_id == 0)) ) /* PD7 or PF0 */
    {
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_LOCK_REG_OFFSET) = 0x4C4F434B;                     /* Unlock the GPIOCR register */   
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_COMMIT_REG_OFFSET) , PinConfigPtr->pin_id);  /* Set the corresponding bit in GPIOCR register to allow changes on this pin */
    }
    else if( (PinConfigPtr->port_num == 2) && (PinConfigPtr->pin_id <= 3) ) /* PC0 to PC3 */
    {
        /* Do Nothing ...  this is the JTAG pins */
    }
    else
    {
        /* Do Nothing ... No need to unlock the commit register for this pin */
    }
}


static void InputResistorSetup(const Port_PinConfigType* PinConfigPtr,const volatile uint32 * PortGpio_Ptr){
    if(PinConfigPtr->pin_resistor == PULL_UP)
    {
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_UP_REG_OFFSET) , PinConfigPtr->pin_id);       /* Set the corresponding bit in the GPIOPUR register to enable the internal pull up pin */
    }
    else if(PinConfigPtr->pin_resistor == PULL_DOWN)
    {
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_DOWN_REG_OFFSET) , PinConfigPtr->pin_id);     /* Set the corresponding bit in the GPIOPDR register to enable the internal pull down pin */
    }
    else
    {
        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_UP_REG_OFFSET) , PinConfigPtr->pin_id);     /* Clear the corresponding bit in the GPIOPUR register to disable the internal pull up pin */
        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_DOWN_REG_OFFSET) , PinConfigPtr->pin_id);   /* Clear the corresponding bit in the GPIOPDR register to disable the internal pull down pin */
    }
}




static void PinDirectionSetup(const Port_PinConfigType* PinConfigPtr,const volatile uint32 * PortGpio_Ptr){
    if(PinConfigPtr->pin_direction== OUTPUT)
    {
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , PinConfigPtr->pin_id);               /* Set the corresponding bit in the GPIODIR register to configure it as output pin */

        if(PinConfigPtr->pin_initial_value == STD_HIGH)
        {
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DATA_REG_OFFSET) , PinConfigPtr->pin_id);          /* Set the corresponding bit in the GPIODATA register to provide initial value 1 */
        }
        else
        {
            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DATA_REG_OFFSET) , PinConfigPtr->pin_id);        /* Clear the corresponding bit in the GPIODATA register to provide initial value 0 */
        }
    }
    else if(PinConfigPtr->pin_direction== INPUT)
    {
        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , PinConfigPtr->pin_id);             /* Clear the corresponding bit in the GPIODIR register to configure it as input pin */
        InputResistorSetup(PinConfigPtr,PortGpio_Ptr);
    }
    else
    {
        /* Do Nothing */
    }

}





static void GpioModeSetup(const Port_PinConfigType* PinConfigPtr,const volatile uint32 * PortGpio_Ptr){
    /* Setup the pin mode as GPIO */
    CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , PinConfigPtr->pin_id);      /* Clear the corresponding bit in the GPIOAMSEL register to disable analog functionality on this pin */
    CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , PinConfigPtr->pin_id);             /* Disable Alternative function for this pin by clear the corresponding bit in GPIOAFSEL register */
    *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (PinConfigPtr->pin_id * 4));     /* Clear the PMCx bits for this pin */
    SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , PinConfigPtr->pin_id);         /* Set the corresponding bit in the GPIODEN register to enable digital functionality on this pin */
}

static void AdcModeSetup(const Port_PinConfigType* PinConfigPtr,const volatile uint32 * PortGpio_Ptr){
    /* Setup the pin mode as ADC pin*/
    SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , PinConfigPtr->pin_id);        /* Clear the corresponding bit in the GPIOAMSEL register to disable analog functionality on this pin */
    CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , PinConfigPtr->pin_id);             /* Disable Alternative function for this pin by clear the corresponding bit in GPIOAFSEL register */
    *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (PinConfigPtr->pin_id * 4));     /* Clear the PMCx bits for this pin */
    CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , PinConfigPtr->pin_id);       /* Set the corresponding bit in the GPIODEN register to enable digital functionality on this pin */
}



static void AlternativeModeSetup(const Port_PinConfigType* PinConfigPtr,const volatile uint32 * PortGpio_Ptr,Port_PinModeType mode){
    /* Setup the pin mode as Alternative Pin */
    CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , PinConfigPtr->pin_id);  /* Clear the corresponding bit in the GPIOAMSEL register to disable analog functionality on this pin */
    SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , PinConfigPtr->pin_id);           /* Enable Alternative function for this pin by setting the corresponding bit in GPIOAFSEL register */
    *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET)|=((mode & 0x0000000F) << (PinConfigPtr->pin_id * 4));   /* Set the PMCx bits for this pin */
    SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , PinConfigPtr->pin_id);     /* Set the corresponding bit in the GPIODEN register to enable digital functionality on this pin */
}


static void PinModeSetup(const Port_PinConfigType* PinConfigPtr,const volatile uint32 * PortGpio_Ptr,Port_PinModeType mode){
    if(mode==PORT_PIN_MODE_DIO){
        GpioModeSetup(PinConfigPtr,PortGpio_Ptr);
    }else if(mode==PORT_PIN_MODE_ADC){
        AdcModeSetup(PinConfigPtr,PortGpio_Ptr);
    }else{
        AlternativeModeSetup(PinConfigPtr, PortGpio_Ptr,mode);
    }

}



static Std_ReturnType PortInitErrorCheck(const Port_ConfigType* ConfigPtr){
#if (PORT_DEV_ERROR_DETECT == STD_ON)
    /* check if the input configuration pointer is not a NULL_PTR */
    if(ConfigPtr == NULL_PTR)
    {
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_INIT_SID, PORT_E_PARAM_CONFIG);
        return E_NOT_OK;
    }
    else
    {
        /* Do Nothing */
    }
#endif
    return E_OK;
}


static Std_ReturnType PinDirectionErrorCheck(Port_PinType Pin){
#if (PORT_DEV_ERROR_DETECT == STD_ON)
    /* Check if the Driver is initialized before using this function */
    if(Port_Status == PORT_NOT_INITIALIZED)
    {
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_DIR_SID, PORT_E_UNINIT);
        return E_NOT_OK;
    }
    else
    {   /* Do Nothing */    }

    /* check if incorrect Port Pin ID passed */
    if(Pin >= PORT_NUMBER_OF_PORT_PINS)
    {
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_DIR_SID, PORT_E_PARAM_PIN);
        return E_NOT_OK;
    }
    else
    {   /* Do Nothing */    }

    /* check if the API called when the direction is unchangeable */
    if(Port_PinConfigurationSet.Port_ConfigPtr[Pin].pin_direction_changeable == STD_OFF)
    {
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_DIR_SID, PORT_E_DIRECTION_UNCHANGEABLE);
        return E_NOT_OK;
    }
    else
    {   /* Do Nothing */    }
#endif
    return E_OK;
}



static Std_ReturnType PinRefreshDirectionErrorCheck(){
#if (PORT_DEV_ERROR_DETECT == STD_ON)
    /* Check if the Driver is initialized before using this function */
    if(Port_Status == PORT_NOT_INITIALIZED)
    {
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_REFRESH_PORT_DIR_SID, PORT_E_UNINIT);
        return E_NOT_OK;
    }
    else
    {   /* Do Nothing */    }
#endif
    return E_OK;
}


static Std_ReturnType GetVersionInfoErrorCheck( Std_VersionInfoType* versioninfo){
#if (PORT_DEV_ERROR_DETECT == STD_ON)
    /* check if the input configuration pointer is not a NULL_PTR */
    if(versioninfo == NULL_PTR)
    {
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_GET_VERSION_INFO_SID, PORT_E_PARAM_POINTER);
        return E_NOT_OK;
    }
    else
    {   /* Do Nothing */    }

    /* Check if the Driver is initialized before using this function */
    if(Port_Status == PORT_NOT_INITIALIZED)
    {
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_GET_VERSION_INFO_SID, PORT_E_UNINIT);
        return E_NOT_OK;
    }
    else
    {   /* Do Nothing */    }
#endif

    return E_OK;
}


static Std_ReturnType PinModeErrorCheck(Port_PinType Pin, Port_PinModeType Mode){
#if (PORT_DEV_ERROR_DETECT == STD_ON)
    /* Check if the Driver is initialized before using this function */
    if(Port_Status == PORT_NOT_INITIALIZED)
    {
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_MODE_SID, PORT_E_UNINIT);
        return E_NOT_OK;
    }
    else
    {   /* Do Nothing */    }

    /* check if incorrect Port Pin ID passed */
    if(Pin >= PORT_NUMBER_OF_PORT_PINS)
    {
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_MODE_SID, PORT_E_PARAM_PIN);
        return E_NOT_OK;
    }
    else
    {   /* Do Nothing */    }

    /* check if the Port Pin Mode passed not valid */
    if(Mode > MAX_PIN_MODE_NUMBER)
    {
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_MODE_SID, PORT_E_PARAM_INVALID_MODE);
        return E_NOT_OK;
    }
    else
    {   /* Do Nothing */     }

    /* check if the API called when the mode is unchangeable */
    if(Port_PinConfigurationSet.Port_ConfigPtr[Pin].pin_mode_changeable == STD_OFF)
    {
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_MODE_SID, PORT_E_MODE_UNCHANGEABLE);
        return E_NOT_OK;
    }
    else
    {   /* Do Nothing */    }
#endif
    return E_OK;
}

/************************************************************************************
 * Service Name: Port_SetupGpioPin
 * Sync/Async: Synchronous
 * Reentrancy: reentrant
 * Parameters (in): ConfigPtr - Pointer to post-build configuration data
 * Parameters (inout): None
 * Parameters (out): None
 * Return value: None
 * Description: Function to Setup the pin configuration:
 *              - Setup the pin as Digital GPIO pin
 *              - Setup the pin_directionof the GPIO pin
 *              - Provide initial value for o/p pin
 *              - Setup the internal resistor for i/p pin
 ************************************************************************************/



static void Port_SetupGpioPin(const Port_PinConfigType* PinConfigPtr  )
{
    if(PinConfigPtr->pin_initial_mode!=PORT_PIN_MODE_NULL){
        volatile uint32 * PortGpio_Ptr = NULL_PTR; /* pointer to the required Port Registers base address */
        GetPort(PinConfigPtr,&PortGpio_Ptr);
        UnlockLockedPins(PinConfigPtr,PortGpio_Ptr);
        PinDirectionSetup(PinConfigPtr, PortGpio_Ptr);
        PinModeSetup(PinConfigPtr, PortGpio_Ptr,PinConfigPtr->pin_initial_mode);
    }

}


/************************************************************************************
                        PORT FUNCTIONS DECLARED IN THE .h FILE
 ************************************************************************************/



/************************************************************************************
 * Service Name: Port_Init
 * Sync/Async: Synchronous
 * Reentrancy: Non-Reentrant
 * Parameters (in): ConfigPtr - Pointer to post-build configuration data
 * Parameters (inout): None
 * Parameters (out): None
 * Return value: None
 * Description: Function to initialize the Port Driver module:
 *              - Set up each configured port pin
 *              - Set the status of the Port module to initialized
 ************************************************************************************/



void Port_Init(const Port_ConfigType* ConfigPtr){

    if(PortInitErrorCheck(ConfigPtr)!=E_OK){
        return;
    }else{
        /* do nothing  */
    }

    //initialize the module and set the status to initialized
    Port_Configurations = &ConfigPtr[0];
    uint8 index;
    for(index=0;index<PORT_NUMBER_OF_PORT_PINS;index++){
        Port_SetupGpioPin(&(Port_Configurations->Port_ConfigPtr[index]));
    }
    Port_Status=PORT_INITIALIZED;


}


/************************************************************************************
 * Service Name: Port_SetPinDirection
 * Sync/Async: Synchronous
 * Reentrancy: Reentrant
 * Parameters (in): Pin - Port pin ID number
 *                  Direction - Port pin direction (INPUT/OUTPUT)
 * Parameters (inout): None
 * Parameters (out): None
 * Return value: None
 * Description: Function to set the direction of a specified pin during runtime:
 *              - Check if direction change is allowed for the specified pin
 *              - Set the pin direction and update the configuration
 ************************************************************************************/



#if (PORT_SET_PIN_DIRECTION_API == STD_ON)
void Port_SetPinDirection( Port_PinType Pin, Port_PinDirectionType Direction ){

    if(PinDirectionErrorCheck(Pin)!=E_OK){
        return;
    }else{
        /* do nothing */
    }


    Port_PinConfigType* PinConfigPtr=&Port_PinConfigurationSet.Port_ConfigPtr[Pin];
    if(PinConfigPtr->pin_initial_mode!=PORT_PIN_MODE_NULL){
        PinConfigPtr->pin_direction =(Port_PinDirection)Direction;
        volatile uint32 * PortGpio_Ptr = NULL_PTR; /* pointer to the required Port Registers base address */
        GetPort(PinConfigPtr,&PortGpio_Ptr);
        PinDirectionSetup(PinConfigPtr,PortGpio_Ptr);
    }

}


#endif



/************************************************************************************
 * Service Name: Port_RefreshPortDirection
 * Sync/Async: Synchronous
 * Reentrancy: Non-Reentrant
 * Parameters (in): None
 * Parameters (inout): None
 * Parameters (out): None
 * Return value: None
 * Description: Function to refresh the direction of all configured pins:
 *              - Reapply the direction settings for all pins with non-changeable direction
 ************************************************************************************/



void Port_RefreshPortDirection( void ){

    if(PinRefreshDirectionErrorCheck()!=E_OK){
        return;
    }else{
        /* do nothing */
    }

    uint8 index;
    for(index=0;index<=PORT_NUMBER_OF_PORT_PINS;index++){
        Port_PinConfigType* PinConfigPtr=&Port_PinConfigurationSet.Port_ConfigPtr[index];
        if((PinConfigPtr->pin_initial_mode!=PORT_PIN_MODE_NULL)&&(PinConfigPtr->pin_direction_changeable==STD_OFF)){
            volatile uint32 * PortGpio_Ptr = NULL_PTR; /* pointer to the required Port Registers base address */
            GetPort(PinConfigPtr,&PortGpio_Ptr);
            PinDirectionSetup(PinConfigPtr,PortGpio_Ptr);
        }
    }
}


/************************************************************************************
 * Service Name: Port_GetVersionInfo
 * Sync/Async: Synchronous
 * Reentrancy: Reentrant
 * Parameters (in): versioninfo - Pointer to where to store the version information of this module
 * Parameters (inout): None
 * Parameters (out): None
 * Return value: None
 * Description: Function to retrieve the version information of this module:
 *              - Provide the module's ID, vendor ID, and software version information
 ************************************************************************************/


#if (PORT_VERSION_INFO_API == STD_ON)
void Port_GetVersionInfo(Std_VersionInfoType* versioninfo)
{

    if(GetVersionInfoErrorCheck(versioninfo)!=E_OK){
        return;
    }else{
        /* do nothing */
    }

    /* Copy the module Id */
    versioninfo->moduleID = (uint16)PORT_MODULE_ID;
    /* Copy the vendor Id */
    versioninfo->vendorID = (uint16)PORT_VENDOR_ID;
    /* Copy Software Major Version */
    versioninfo->sw_major_version = (uint8)PORT_SW_MAJOR_VERSION;
    /* Copy Software Minor Version */
    versioninfo->sw_minor_version = (uint8)PORT_SW_MINOR_VERSION;
    /* Copy Software Patch Version */
    versioninfo->sw_patch_version = (uint8)PORT_SW_PATCH_VERSION;
}
#endif






/************************************************************************************
 * Service Name: Port_SetPinMode
 * Sync/Async: Synchronous
 * Reentrancy: Reentrant
 * Parameters (in): Pin - Port pin ID number
 *                  Mode - New Port pin mode to be set
 * Parameters (inout): None
 * Parameters (out): None
 * Return value: None
 * Description: Function to set the mode of a specified pin during runtime:
 *              - Check if mode change is allowed for the specified pin
 *              - Set the pin mode and update the configuration
 ************************************************************************************/


#if (PORT_SET_PIN_MODE_API == STD_ON)

void Port_SetPinMode( Port_PinType Pin, Port_PinModeType Mode){

    if(PinModeErrorCheck(Pin,Mode)!=E_OK){
        return;
    }else{
        /* do nothing */
    }

    Port_PinConfigType* PinConfigPtr=&Port_PinConfigurationSet.Port_ConfigPtr[Pin];
    if(PinConfigPtr->pin_initial_mode!=PORT_PIN_MODE_NULL){
        PinConfigPtr->pin_mode =(Port_PinMode)Mode;
        volatile uint32 * PortGpio_Ptr = NULL_PTR; /* pointer to the required Port Registers base address */
        GetPort(PinConfigPtr,&PortGpio_Ptr);
        PinModeSetup(PinConfigPtr, PortGpio_Ptr,PinConfigPtr->pin_mode);
    }

}

#endif

