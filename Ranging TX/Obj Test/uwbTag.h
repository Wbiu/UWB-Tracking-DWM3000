#pragma once
#include "dw3000.h"
#include "kalmanFilter.h"

#ifndef UWB_TAG_H
#define UWB_TAG_H
class UWB_TAG 
{

public:
        enum dw_state
        {
                RDY = 1,
                N_RDY = -1
        };
        
        dw_state state = RDY;
        Kalman* kalman;
        void initSPI();
        void initSPI(uint8_t PIN_RST,uint8_t PIN_IRQ,uint8_t PIN_SS);
        void configure();
        void configure(dwt_config_t config);
        void initDWT();
        void raging(float* distance);
        UWB_TAG(uint8_t unique_ID)
        {       
                kalman = new Kalman(mea_e,est_e,q);
              //  tx_poll_msg = new uint8_t[12] {0x41, 0x88, 0, 0xCA, unique_ID, 'W', 'A', 'V', 'E', 0xE0, 0, 0};
               // rx_resp_msg = new uint8_t[20] {0x41, 0x88, 0, 0xCA, unique_ID, 'V', 'E', 'W', 'A', 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        }
        
private : 
        float tof;
        float mea_e = 0.2; 
        float est_e = 0.2; 
        float q = 0.1;

        const int RNG_DELAY_MS =  1000;
        const int TX_ANT_DLY = 16385;
        const int RX_ANT_DLY = 16385;
        const int ALL_MSG_COMMON_LEN = 10;
        const int ALL_MSG_SN_IDX = 2;
        const int RESP_MSG_POLL_RX_TS_IDX = 10;
        const int RESP_MSG_RESP_TX_TS_IDX = 14;
       // const uint8_t RESP_MSG_TS_LEN  = 4;
        const int POLL_TX_TO_RESP_RX_DLY_UUS  = 240; // 140
        const int RESP_RX_TIMEOUT_UUS = 400;
        /* Buffer to store received response message.
        * Its size is adjusted to longest frame that this example code is supposed to handle. */
        const int RX_BUF_LEN = 20;
        
        /* Default communication configuration. Non-STS DW mode. */
        dwt_config_t dwt_config;

        uint8_t tx_poll_msg[12] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0xE0, 0, 0};
        uint8_t rx_resp_msg[20] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};


        dwt_txconfig_t txconfig_options;
        uint8_t frame_seq_nb = 0;

        uint8_t rx_buffer[20];

        /* Hold copy of status register state here for reference so that it can be examined at a debug breakpoint. */
        uint32_t status_reg = 0;

        UWB_TAG(){};
};

#endif /* end UWB_TAG_H */