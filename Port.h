 /******************************************************************************
 *
 * Module: Port
 *
 * File Name: Port.h
 *
 * Description: Header file for TM4C123GH6PM Microcontroller - Port Driver.
 *
 * Author: Mazen Adel
 ******************************************************************************/

#ifndef PORT_H
#define PORT_H






/* Id for the company in the AUTOSAR
 * for example Mazen Adel's ID = 1256 :) */
#define PORT_VENDOR_ID    (1256U)

/* Port Module Id */
#define PORT_MODULE_ID    (124U)

/* Port Instance Id */
#define PORT_INSTANCE_ID  (0U)






/*
 * Module Version 1.0.0
 */
#define PORT_SW_MAJOR_VERSION           (1U)
#define PORT_SW_MINOR_VERSION           (0U)
#define PORT_SW_PATCH_VERSION           (0U)

/*
 * AUTOSAR Version 4.0.3
 */
#define PORT_AR_RELEASE_MAJOR_VERSION   (4U)
#define PORT_AR_RELEASE_MINOR_VERSION   (0U)
#define PORT_AR_RELEASE_PATCH_VERSION   (3U)

/*
 * Macros for Port Status
 */
#define PORT_INITIALIZED                (1U)
#define PORT_NOT_INITIALIZED            (0U)




#include "Common_Macros.h"
#include "Std_Types.h"
#include "Port_Cfg.h"


/* AUTOSAR checking between Std Types and Port Modules */
#if ((STD_TYPES_AR_RELEASE_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION)\
 ||  (STD_TYPES_AR_RELEASE_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION)\
 ||  (STD_TYPES_AR_RELEASE_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
  #error "The AR version of Std_Types.h does not match the expected version"
#endif


/* AUTOSAR Version checking between Port_Cfg.h and Port.h files */
#if ((PORT_CFG_AR_RELEASE_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION)\
 ||  (PORT_CFG_AR_RELEASE_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION)\
 ||  (PORT_CFG_AR_RELEASE_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
  #error "The AR version of Port_Cfg.h does not match the expected version"
#endif

/* Software Version checking between Port_Cfg.h and Port.h files */
#if ((PORT_CFG_SW_MAJOR_VERSION != PORT_SW_MAJOR_VERSION)\
 ||  (PORT_CFG_SW_MINOR_VERSION != PORT_SW_MINOR_VERSION)\
 ||  (PORT_CFG_SW_PATCH_VERSION != PORT_SW_PATCH_VERSION))
  #error "The SW version of Port_Cfg.h does not match the expected version"
#endif




/******************************************************************************
 *                      API Service Id Macros                                 *
 ******************************************************************************/
/* Service ID for Port_Init */
#define PORT_INIT_SID                       (uint8)0x00

/* Service ID for Port_SetPinDirection */
#define PORT_SET_PIN_DIR_SID                (uint8)0x01

/* Service ID for Port_RefreshPortDirection */
#define PORT_REFRESH_PORT_DIR_SID           (uint8)0x02

/* Service ID for Port_GetVersionInfo */
#define PORT_GET_VERSION_INFO_SID           (uint8)0x03

/* Service ID for Port_SetPinMode */
#define PORT_SET_PIN_MODE_SID               (uint8)0x04



/*******************************************************************************
 *                      DET Error Codes                                        *
 *******************************************************************************/
/* Invalid Port Pin ID requested */
#define PORT_E_PARAM_PIN                    (uint8)0x0A

/* Port Pin not configured as changeable */
#define PORT_E_DIRECTION_UNCHANGEABLE       (uint8)0x0B

/* API Port_Init service called with wrong parameter */
#define PORT_E_PARAM_CONFIG                 (uint8)0x0C

/* API Port_SetPinMode service called when Port Pin Mode passed not valid */
#define PORT_E_PARAM_INVALID_MODE           (uint8)0x0D

/* API Port_SetPinMode service called when mode is unchangeable */
#define PORT_E_MODE_UNCHANGEABLE            (uint8)0x0E

/* API service called without module initialization */
#define PORT_E_UNINIT                       (uint8)0x0F

/* APIs called with a Null Pointer */
#define PORT_E_PARAM_POINTER                (uint8)0x10





/*******************************************************************************
 *                              Module Definitions                             *
 *******************************************************************************/

/* GPIO Registers base addresses */
#define GPIO_PORTA_BASE_ADDRESS           0x40004000
#define GPIO_PORTB_BASE_ADDRESS           0x40005000
#define GPIO_PORTC_BASE_ADDRESS           0x40006000
#define GPIO_PORTD_BASE_ADDRESS           0x40007000
#define GPIO_PORTE_BASE_ADDRESS           0x40024000
#define GPIO_PORTF_BASE_ADDRESS           0x40025000

/* GPIO Registers offset addresses */
#define PORT_DATA_REG_OFFSET              0x3FC
#define PORT_DIR_REG_OFFSET               0x400
#define PORT_ALT_FUNC_REG_OFFSET          0x420
#define PORT_PULL_UP_REG_OFFSET           0x510
#define PORT_PULL_DOWN_REG_OFFSET         0x514
#define PORT_DIGITAL_ENABLE_REG_OFFSET    0x51C
#define PORT_LOCK_REG_OFFSET              0x520
#define PORT_COMMIT_REG_OFFSET            0x524
#define PORT_ANALOG_MODE_SEL_REG_OFFSET   0x528
#define PORT_CTL_REG_OFFSET               0x52C





/*******************************************************************************
 *                              Module Data Types                              *
 *******************************************************************************/


typedef uint8 Port_PinType;
typedef uint8 Port_PinDirectionType;
typedef uint8 Port_PinModeType;


/* Description: Enum to hold PIN level */
typedef enum
{
    PORT_PIN_LEVEL_LOW= STD_LOW,
    PORT_PIN_LEVEL_HIGH= STD_HIGH
}Port_PinLevelValue;


/* Description: Enum to hold PIN direction */
typedef enum
{
    INPUT,OUTPUT
}Port_PinDirection;

/* Description: Enum to hold internal resistor type for PIN */
typedef enum
{
    OFF,PULL_UP,PULL_DOWN,RESISTOR_OFF
}Port_InternalResistor;


/* Description: Enum to hold PIN level */
typedef enum {
    PORT_PIN_MODE_DIO=0,
    PORT_PIN_MODE_ALT1=1,
    PORT_PIN_MODE_ALT2=2,
    PORT_PIN_MODE_ALT3=3,
    PORT_PIN_MODE_ALT4=4,
    PORT_PIN_MODE_ALT5=5,
    PORT_PIN_MODE_ALT6=6,
    PORT_PIN_MODE_ALT7=7,
    PORT_PIN_MODE_ALT8=8,
    PORT_PIN_MODE_ALT9=9,
    PORT_PIN_MODE_ADC=10,         /* Port Pin used by ADC */
    PORT_PIN_MODE_NULL=11,
} Port_PinMode;

#define MAX_PIN_MODE_NUMBER   (11U)




/* Description: Structure to configure each individual PIN:
 *	1. the PORT Which the pin belongs to. 0, 1, 2, 3, 4 or 5
 *	2. the number of the pin in the PORT.
 *      3. the direction of pin --> INPUT or OUTPUT
 *      4. the internal resistor --> Disable, Pull up or Pull down
 */
typedef struct 
{
    uint8 pin_id;
    uint8 port_num; 
    Port_PinDirection pin_direction;
    Port_InternalResistor pin_resistor;
    Port_PinLevelValue pin_initial_value;
    Port_PinMode pin_initial_mode;
    Port_PinMode pin_mode;
    uint8 pin_direction_changeable;
    uint8 pin_mode_changeable;
}Port_PinConfigType;


typedef struct{
  Port_PinConfigType Port_ConfigPtr[PORT_NUMBER_OF_PORT_PINS] ;
}Port_ConfigType;




/*******************************************************************************
 *                      Function Prototypes                                    *
 *******************************************************************************/

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
*              - Setup the direction of the GPIO pin
*              - Setup the internal resistor for i/p pin
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


void Port_Init(const Port_ConfigType* ConfigPtr);  // shall have a pointer to the array of pins initialization





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


void Port_SetPinDirection( Port_PinType Pin, Port_PinDirectionType Direction );



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


void Port_RefreshPortDirection( void );




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

void Port_GetVersionInfo( Std_VersionInfoType* versioninfo );



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



void Port_SetPinMode( Port_PinType Pin, Port_PinModeType Mode );



extern  Port_ConfigType Port_PinConfigurationSet;


#endif /* PORT_H */
