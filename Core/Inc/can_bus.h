#ifndef CAN_BUS_H
#define CAN_BUS_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

// =============================================================================
// CAN ID
// =============================================================================
#define CAN_ID_BOARD_A_CMD     0x201U  // Board A → Board B (command)
#define CAN_ID_BOARD_B_STATUS  0x181U  // Board B → Board A (data sensor)

// =============================================================================
// RELAY COMMAND (Board A → Board B)
// =============================================================================
#define RELAY_CMD_OFF          0x00U
#define RELAY_CMD_ON           0x01U
#define RELAY_CMD_TOGGLE       0x02U
#define RELAY_CMD_STATUS_REQ   0x03U

// =============================================================================
// STRUCT FRAME DATA
// =============================================================================
typedef struct {
    uint8_t  relay_state;
    uint8_t  temperature;
    uint16_t voltage;
    uint16_t uptime;
    uint8_t  fault_flags;
    uint8_t  counter;
} CanBoardBData_t;

// =============================================================================
// VARIABEL GLOBAL
// =============================================================================
extern CanBoardBData_t g_can_b_data;
extern uint8_t         g_can_rx_flag;
extern uint32_t        g_can_rx_count;
extern uint32_t        g_can_tx_count;
extern uint8_t         g_can_err_count;

// =============================================================================
// FUNGSI PUBLIK — tanpa CAN_HandleTypeDef
// =============================================================================
void    CAN_Bus_Init(void);
uint8_t CAN_Bus_SendCmd(uint8_t cmd);
void    CAN_Bus_RxFifo0Callback(void);
void    CAN_Bus_RxFifo1Callback(void);

#endif /* CAN_BUS_H */
