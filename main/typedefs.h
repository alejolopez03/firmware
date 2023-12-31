/**
  ******************************************************************************
  * @file           : typedefs.h
  * @author         : Mauricio Barroso Benavides
  * @date           : Jun 21, 2023
  * @brief          : todo: write brief 
  ******************************************************************************
  * @attention
  *
  * MIT License
  *
  * Copyright (c) 2023 Mauricio Barroso Benavides
  *
  * Permission is hereby granted, free of charge, to any person obtaining a copy
  * of this software and associated documentation files (the "Software"), to
  * deal in the Software without restriction, including without limitation the
  * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
  * sell copies of the Software, and to permit persons to whom the Software is
  * furnished to do so, subject to the following conditions:
  *
  * The above copyright notice and this permission notice shall be included in
  * all copies or substantial portions of the Software.
  * 
  * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
  * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
  * IN THE SOFTWARE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef TYPEDEFS_H_
#define TYPEDEFS_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <stdlib.h>

/* Exported Macros -----------------------------------------------------------*/

/* Exported typedef ----------------------------------------------------------*/
/* Events to drive the FSM */
typedef enum {
	FSM_INIT_SYSTEM_EVENT = 0,
	FSM_PENDING_DATA_EVENT,
	FSM_MQTT_OPEN_EVENT,
	FSM_MQTT_CLOSE_EVENT,
	FSM_BUTTON_EVENT,
	FSM_MAX_EVENT
} fsm_events_e;

/* States of the FSM */
typedef enum {
	FSM_IDLE_STATE = 0,
	FSM_SEND_DATA_STATE,
	FSM_SET_ACTUATORS_STATE,
	FSM_MAX_STATE
} fsm_states_e;

/* Exported variables --------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* TYPEDEFS_H_ */

/***************************** END OF FILE ************************************/
