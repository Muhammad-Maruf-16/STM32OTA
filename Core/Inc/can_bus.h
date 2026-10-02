#ifndef CAN_BUS_H
#define CAN_BUS_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

// =============================================================================
// CAN ID
// =============================================================================
#define CAN_ID_BOARD_A_CMD     0x201   // Board A → Board B (command)
#define CAN_ID_BOARD_B_STATUS  0x181   // Board B → Board A (data sensor dummy)

// =============================================================================
// RELAY COMMAND (Board A → Board B)
// =============================================================================
#define RELAY_CMD_OFF          0x00
#define RELAY_CMD_ON           0x01
#define RELAY_CMD_TOGGLE       0x02
#define RELAY_CMD_STATUS_REQ   0x03

// =============================================================================
// STRUCT FRAME DATA
// =============================================================================

// Frame Board B → Board A (8 byte)
// [0] relay_state  [1] temperature  [2-3] voltage×10
// [4-5] uptime     [6] fault_flags  [7] counter
typedef struct {
    uint8_t  relay_state;   // 0=OFF 1=ON
    uint8_t  temperature;   // °C
    uint16_t voltage;       // ×10, misal 330 = 33.0V
    uint16_t uptime;        // detik
    uint8_t  fault_flags;   // bit0=overtemp, bit1=undervolt
    uint8_t  counter;       // increment tiap kirim
} CanBoardBData_t;

// Frame Board A → Board B (command, 8 byte)
typedef struct {
    uint8_t cmd;            // RELAY_CMD_xxx
    uint8_t reserved[7];
} CanBoardACmd_t;

// =============================================================================
// VARIABEL GLOBAL
// =============================================================================
extern CanBoardBData_t g_can_b_data;
extern uint8_t         g_can_rx_flag;
extern uint32_t        g_can_rx_count;
extern uint32_t        g_can_tx_count;
extern uint8_t         g_can_err_count;

// =============================================================================
// FUNGSI PUBLIK
// =============================================================================
void    CAN_Bus_Init(CAN_HandleTypeDef *hcan);
uint8_t CAN_Bus_SendCmd(CAN_HandleTypeDef *hcan, uint8_t cmd);
void    CAN_Bus_RxFifo0Callback(CAN_HandleTypeDef *hcan);
void    CAN_Bus_RxFifo1Callback(CAN_HandleTypeDef *hcan);

#endif
