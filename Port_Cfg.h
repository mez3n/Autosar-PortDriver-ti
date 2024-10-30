/*
 * Port_Cfg.h
 *
 *  Created on: Aug 25, 2024
 *      Author: Mazen
 */

#ifndef PORT_CFG_H_
#define PORT_CFG_H_

/*
 * Module Version 1.0.0
 */
#define PORT_CFG_SW_MAJOR_VERSION              (1U)
#define PORT_CFG_SW_MINOR_VERSION              (0U)
#define PORT_CFG_SW_PATCH_VERSION              (0U)

/*
 * AUTOSAR Version 4.0.3
 */
#define PORT_CFG_AR_RELEASE_MAJOR_VERSION     (4U)
#define PORT_CFG_AR_RELEASE_MINOR_VERSION     (0U)
#define PORT_CFG_AR_RELEASE_PATCH_VERSION     (3U)

/* Pre-compile option for Development Error Detect */
#define PORT_DEV_ERROR_DETECT                (STD_ON)

/* Pre-compile option for Version Info API */
#define PORT_VERSION_INFO_API                (STD_ON)


#define PORT_NUMBER_OF_PORT_PINS              (43U)

#define PORT_SET_PIN_DIRECTION_API            (STD_ON)

#define PORT_SET_PIN_MODE_API                 (STD_ON)



/*******************************************************************************
 *                         PINS AND PORTS INDECES                              *
 *******************************************************************************/
#define PIN0 (0U)
#define PIN1 (1U)
#define PIN2 (2U)
#define PIN3 (3U)
#define PIN4 (4U)
#define PIN5 (5U)
#define PIN6 (6U)
#define PIN7 (7U)


#define PORTA  (0U) // Base address for Port A
#define PORTB  (1U) // Base address for Port B
#define PORTC  (2U) // Base address for Port C
#define PORTD  (3U) // Base address for Port D
#define PORTE  (4U) // Base address for Port E
#define PORTF  (5U) // Base address for Port F


#endif /* PORT_CFG_H_ */
