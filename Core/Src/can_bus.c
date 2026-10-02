#include "can_bus.h"
#include <string.h>

// =============================================================================
// VARIABEL GLOBAL
// =============================================================================
CanBoardBData_t g_can_b_data;
uint8_t         g_can_rx_flag;
uint32_t        g_can_rx_count;
uint32_t        g_can_tx_count;
uint8_t         g_can_err_count;

// =============================================================================
// CAN_Bus_Init — setup filter, aktifkan interrupt, start CAN
// =============================================================================
void CAN_Bus_Init(CAN_HandleTypeDef *hcan)
{
    // Zero semua variabel
    memset(&g_can_b_data, 0, sizeof(g_can_b_data));
    g_can_rx_flag   = 0;
    g_can_rx_count  = 0;
    g_can_tx_count  = 0;
    g_can_err_count = 0;

    // Filter — terima CAN_ID_BOARD_B_STATUS (0x181) saja
    // Mode: ID Mask, 16-bit scale, FIFO 0
    // Format 16-bit: StdId di bit[15:5], RTR di bit[4], IDE di bit[3]
    CAN_FilterTypeDef filter;
    filter.FilterBank           = 0;
    filter.FilterMode           = CAN_FILTERMODE_IDMASK;
    filter.FilterScale          = CAN_FILTERSCALE_16BIT;
    filter.FilterIdHigh         = (CAN_ID_BOARD_B_STATUS << 5) & 0xFFFF;
    filter.FilterIdLow          = 0x0000;
    filter.FilterMaskIdHigh     = (0x7FF << 5) & 0xFFFF;  // exact match
    filter.FilterMaskIdLow      = 0x0000;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterActivation     = ENABLE;
    HAL_CAN_ConfigFilter(hcan, &filter);

    // Aktifkan interrupt RX FIFO0 dan error
    HAL_CAN_ActivateNotification(hcan,
        CAN_IT_RX_FIFO0_MSG_PENDING |
        CAN_IT_ERROR                |
        CAN_IT_BUSOFF               |
        CAN_IT_LAST_ERROR_CODE);

    // Start CAN
    HAL_CAN_Start(hcan);
}

// =============================================================================
// CAN_Bus_SendCmd — kirim command relay ke Board B
// Return: 1=sukses, 0=gagal
// =============================================================================
uint8_t CAN_Bus_SendCmd(CAN_HandleTypeDef *hcan, uint8_t cmd)
{
    CAN_TxHeaderTypeDef tx_hdr;
    uint8_t  tx_data[8] = {0};
    uint32_t tx_mailbox;

    tx_hdr.StdId              = CAN_ID_BOARD_A_CMD;
    tx_hdr.ExtId              = 0;
    tx_hdr.IDE                = CAN_ID_STD;
    tx_hdr.RTR                = CAN_RTR_DATA;
    tx_hdr.DLC                = 8;
    tx_hdr.TransmitGlobalTime = DISABLE;

    tx_data[0] = cmd;

    if (HAL_CAN_AddTxMessage(hcan, &tx_hdr, tx_data, &tx_mailbox) == HAL_OK) {
        g_can_tx_count++;
        return 1;
    }

    g_can_err_count++;
    return 0;
}

// =============================================================================
// CAN_Bus_RxFifo0Callback — dipanggil dari HAL_CAN_RxFifo0MsgPendingCallback
// =============================================================================
void CAN_Bus_RxFifo0Callback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_hdr;
    uint8_t rx_data[8];

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_hdr, rx_data) != HAL_OK)
        return;

    if (rx_hdr.StdId != CAN_ID_BOARD_B_STATUS) return;
    if (rx_hdr.DLC   != 8) return;

    // Parse frame ke struct
    g_can_b_data.relay_state = rx_data[0];
    g_can_b_data.temperature = rx_data[1];
    g_can_b_data.voltage     = ((uint16_t)rx_data[2] << 8) | rx_data[3];
    g_can_b_data.uptime      = ((uint16_t)rx_data[4] << 8) | rx_data[5];
    g_can_b_data.fault_flags = rx_data[6];
    g_can_b_data.counter     = rx_data[7];

    g_can_rx_flag = 1;
    g_can_rx_count++;
}

// =============================================================================
// CAN_Bus_RxFifo1Callback — tidak dipakai, disediakan untuk ekspansi
// =============================================================================
void CAN_Bus_RxFifo1Callback(CAN_HandleTypeDef *hcan)
{
    (void)hcan;
}
