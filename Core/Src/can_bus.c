#include "can_bus.h"
#include "stm32f1xx.h"
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
// PRIVATE: Masuk / keluar initialization mode
// =============================================================================
static uint8_t can_enter_init_mode(void)
{
    CAN1->MCR |= CAN_MCR_INRQ;
    uint32_t t = 0;
    while ((CAN1->MSR & CAN_MSR_INAK) == 0) {
        if (++t > 1000000U) return 0;
    }
    return 1;
}

static uint8_t can_leave_init_mode(void)
{
    CAN1->MCR &= ~CAN_MCR_INRQ;
    uint32_t t = 0;
    while ((CAN1->MSR & CAN_MSR_INAK) != 0) {
        if (++t > 1000000U) return 0;
    }
    return 1;
}

// =============================================================================
// CAN_Bus_Init
// Dipanggil setelah MX_CAN_Init() dari CubeMX (clock & GPIO sudah siap)
// Timing: Prescaler=4, BS1=13TQ, BS2=4TQ, SJW=1TQ -> 1Mbps @72MHz
// Sama persis dengan konfigurasi HAL sebelumnya
// =============================================================================
void CAN_Bus_Init(void)
{
    // Zero semua variabel
    memset(&g_can_b_data, 0, sizeof(g_can_b_data));
    g_can_rx_flag   = 0;
    g_can_rx_count  = 0;
    g_can_tx_count  = 0;
    g_can_err_count = 0;

    // 1. Keluar dari sleep mode
    CAN1->MCR &= ~CAN_MCR_SLEEP;
    uint32_t t = 0;
    while ((CAN1->MSR & CAN_MSR_SLAK) != 0) {
        if (++t > 1000000U) break;
    }

    // 2. Masuk initialization mode
    can_enter_init_mode();

    // 3. Set timing (BTR) sesuai RM0008 Section 24.9.5:
    //    bit[25:24] = SJW[1:0]-1,  bit[22:20] = TS2[2:0]-1
    //    bit[19:16] = TS1[3:0]-1,  bit[9:0]   = BRP[9:0]-1
    //    Mode bit[31:30] = 0 (normal, bukan loopback/silent)
    CAN1->BTR = ((0U  & 0x3U)  << 24) |   // SJW-1 = 0  (1TQ)
                ((3U  & 0x7U)  << 20) |   // TS2-1 = 3  (BS2 = 4TQ)
                ((12U & 0xFU)  << 16) |   // TS1-1 = 12 (BS1 = 13TQ)
                ((3U  & 0x3FFU));          // BRP-1 = 3  (Prescaler = 4)

    // 4. Set MCR — sama dengan HAL: ABOM=1, AWUM=0, NART=0, RFLM=0, TXFP=0
    CAN1->MCR = (CAN1->MCR
                  & ~(CAN_MCR_AWUM | CAN_MCR_NART | CAN_MCR_RFLM | CAN_MCR_TXFP))
                | CAN_MCR_ABOM;

    // 5. Setup filter bank 0: ID mask 16-bit, FIFO0, terima 0x181 saja
    CAN1->FMR  |=  CAN_FMR_FINIT;    // masuk filter init mode

    CAN1->FA1R &= ~(1U << 0);         // nonaktifkan filter 0 sebelum config
    CAN1->FS1R &= ~(1U << 0);         // scale 16-bit
    CAN1->FM1R &= ~(1U << 0);         // mode ID/Mask
    CAN1->FFA1R &= ~(1U << 0);        // FIFO0

    // FR1[15:0]  = FilterId   (StdId << 5)
    // FR1[31:16] = FilterMask (0x7FF << 5 = exact match semua 11 bit)
    CAN1->sFilterRegister[0].FR1 =
        ((uint32_t)(CAN_ID_BOARD_B_STATUS << 5) & 0xFFFFU) |
        ((uint32_t)((0x7FFU << 5) & 0xFFFFU) << 16);
    CAN1->sFilterRegister[0].FR2 = 0U;

    CAN1->FA1R |=  (1U << 0);         // aktifkan filter 0
    CAN1->FMR  &= ~CAN_FMR_FINIT;    // keluar filter init mode

    // 6. Keluar initialization mode -> CAN normal
    can_leave_init_mode();

    // 7. Aktifkan interrupt: RX FIFO0 pending + error + busoff + last error code
    CAN1->IER |= CAN_IER_FMPIE0 |
                 CAN_IER_ERRIE  |
                 CAN_IER_BOFIE  |
                 CAN_IER_LECIE;

    // 8. Enable NVIC
    NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn,
        NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 0));
    NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);

    NVIC_SetPriority(CAN1_SCE_IRQn,
        NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 0));
    NVIC_EnableIRQ(CAN1_SCE_IRQn);
}

// =============================================================================
// CAN_Bus_SendCmd — kirim command ke Board B
// Cari TX mailbox kosong via TSR[TME0/TME1/TME2]
// Return: 1=sukses, 0=semua mailbox penuh
// =============================================================================
uint8_t CAN_Bus_SendCmd(uint8_t cmd)
{
    // Pilih mailbox kosong
    uint8_t mb;
    if      (CAN1->TSR & CAN_TSR_TME0) mb = 0;
    else if (CAN1->TSR & CAN_TSR_TME1) mb = 1;
    else if (CAN1->TSR & CAN_TSR_TME2) mb = 2;
    else {
        g_can_err_count++;
        return 0;
    }

    // TIR: StdId di bit[31:21], IDE=0, RTR=0, TXRQ diset terakhir
    CAN1->sTxMailBox[mb].TIR  = (uint32_t)(CAN_ID_BOARD_A_CMD << 21);
    CAN1->sTxMailBox[mb].TDTR = 8U;       // DLC = 8
    CAN1->sTxMailBox[mb].TDLR = (uint32_t)cmd;  // byte0=cmd, byte1-3=0
    CAN1->sTxMailBox[mb].TDHR = 0U;       // byte4-7=0

    // Set TXRQ — mulai transmit
    CAN1->sTxMailBox[mb].TIR |= CAN_TI0R_TXRQ;

    g_can_tx_count++;
    return 1;
}

// =============================================================================
// CAN_Bus_RxFifo0Callback — baca dari FIFO0, parse ke g_can_b_data
// Dipanggil langsung dari USB_LP_CAN1_RX0_IRQHandler
// =============================================================================
void CAN_Bus_RxFifo0Callback(void)
{
    // Proses semua pesan di FIFO0
    while ((CAN1->RF0R & CAN_RF0R_FMP0) != 0) {

        uint32_t rir  = CAN1->sFIFOMailBox[0].RIR;
        uint8_t  dlc  = (uint8_t)(CAN1->sFIFOMailBox[0].RDTR & 0x0FU);
        uint32_t rdlr = CAN1->sFIFOMailBox[0].RDLR;
        uint32_t rdhr = CAN1->sFIFOMailBox[0].RDHR;

        // Release FIFO slot sebelum proses — cegah overflow
        CAN1->RF0R |= CAN_RF0R_RFOM0;

        uint16_t std_id = (uint16_t)((rir >> 21) & 0x7FFU);

        if (std_id != CAN_ID_BOARD_B_STATUS) continue;
        if (dlc    != 8U)                    continue;

        // Parse data — byte order: RDLR[7:0]=byte0, [15:8]=byte1, dst
        g_can_b_data.relay_state = (uint8_t)( rdlr        & 0xFFU);
        g_can_b_data.temperature = (uint8_t)((rdlr >>  8) & 0xFFU);
        g_can_b_data.voltage     = (uint16_t)(((rdlr >> 16) & 0xFFU) << 8) |
                                   (uint16_t)( (rdlr >> 24) & 0xFFU);
        g_can_b_data.uptime      = (uint16_t)(( rdhr        & 0xFFU) << 8) |
                                   (uint16_t)(( rdhr >>  8) & 0xFFU);
        g_can_b_data.fault_flags = (uint8_t)((rdhr >> 16) & 0xFFU);
        g_can_b_data.counter     = (uint8_t)((rdhr >> 24) & 0xFFU);

        g_can_rx_flag = 1;
        g_can_rx_count++;
    }
}

// =============================================================================
// CAN_Bus_RxFifo1Callback — tidak dipakai, kosongkan saja jika ada
// =============================================================================
void CAN_Bus_RxFifo1Callback(void)
{
    while ((CAN1->RF1R & CAN_RF1R_FMP1) != 0) {
        CAN1->RF1R |= CAN_RF1R_RFOM1;
    }
}
