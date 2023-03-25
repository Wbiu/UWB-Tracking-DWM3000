#include "uwbTag.h"


void UWB_TAG::initSPI()
{
    const uint8_t PIN_RST = 27; // reset pin
    const uint8_t PIN_IRQ = 34; // irq pin
    const uint8_t PIN_SS = 4;   // spi select pin
    spiBegin(PIN_IRQ, PIN_RST);
    spiSelect(PIN_SS);
}

void UWB_TAG::initSPI(uint8_t PIN_RST,uint8_t PIN_IRQ,uint8_t PIN_SS)
{
    spiBegin(PIN_IRQ, PIN_RST); 
    spiSelect(PIN_SS);
}

void UWB_TAG::configure()
{
    dwt_config = {
            5,                /* Channel number. */
            DWT_PLEN_128,     /* Preamble length. Used in TX only. */
            DWT_PAC8,         /* Preamble acquisition chunk size. Used in RX only. */
            9,                /* TX preamble code. Used in TX only. */
            9,                /* RX preamble code. Used in RX only. */
            1,                /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
            DWT_BR_6M8,       /* Data rate. */
            DWT_PHRMODE_STD,  /* PHY header mode. */
            DWT_PHRRATE_STD,  /* PHY header rate. */
            (129 + 8 - 8),    /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
            DWT_STS_MODE_OFF, /* STS disabled */
            DWT_STS_LEN_64,   /* STS length see allowed values in Enum dwt_sts_lengths_e */
            DWT_PDOA_M0       /* PDOA mode off */
        };
}

void UWB_TAG::configure(dwt_config_t config)
{
    dwt_config = config;
}

void UWB_TAG::initDWT()
{
    while (!dwt_checkidlerc()) // Need to make sure DW IC is in IDLE_RC before proceeding
    {
        state = dw_state::N_RDY;
    }

    if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR)
    {
        state = dw_state::N_RDY;
    }

    // Enabling LEDs here for debug so that for each TX the D1 LED will flash on DW3000 red eval-shield boards.
    dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);

    /* Configure DW IC. See NOTE 6 below. */
    if (dwt_configure(&dwt_config)) // if the dwt_configure returns DWT_ERROR either the PLL or RX calibration has failed the host should reset the device
    {
        state = dw_state::N_RDY;
    }

    dwt_configuretxrf(&txconfig_options);

    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);

    dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
    dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);
    // dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);
    delay(2000);
}

void UWB_TAG::raging(float* distance)
{   
    /* Write frame data to DW IC and prepare transmission. See NOTE 7 below. */
        tx_poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
        dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0); /* Zero offset in TX buffer. */
        dwt_writetxfctrl(sizeof(tx_poll_msg), 0, 1); /* Zero offset in TX buffer, ranging. */

        /* Start transmission, indicating that a response is expected so that reception is enabled automatically after the frame is sent and the delay
         * set by dwt_setrxaftertxdelay() has elapsed. */
        dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
        /* We assume that the transmission is achieved correctly, poll for reception of a frame or error/timeout. See NOTE 8 below. */
        while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
        { };

        /* Increment frame sequence number after transmission of the poll message (modulo 256). */
        frame_seq_nb++;

        if (status_reg & SYS_STATUS_RXFCG_BIT_MASK)
        {
            uint32_t frame_len;
            /* Clear good RX frame event in the DW IC status register. */
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);

            /* A frame has been received, read it into the local buffer. */
            frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
            if (frame_len <= sizeof(rx_buffer))
            {
                dwt_readrxdata(rx_buffer, frame_len, 0);

                /* Check that the frame is the expected response from the companion "SS TWR responder" example.
                 * As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame. */
                rx_buffer[ALL_MSG_SN_IDX] = 0;
                if (memcmp(rx_buffer, rx_resp_msg, ALL_MSG_COMMON_LEN) == 0)
                {
                    uint32_t poll_tx_ts, resp_rx_ts, poll_rx_ts, resp_tx_ts;
                    int32_t rtd_init, rtd_resp;
                    float clockOffsetRatio ;

                    /* Retrieve poll transmission and response reception timestamps. See NOTE 9 below. */
                    poll_tx_ts = dwt_readtxtimestamplo32();
                    resp_rx_ts = dwt_readrxtimestamplo32();

                    /* Read carrier integrator value and calculate clock offset ratio. See NOTE 11 below. */
                    clockOffsetRatio = ((float)dwt_readclockoffset()) / (uint32_t)(1<<26);

                    /* Get timestamps embedded in response message. */
                    resp_msg_get_ts(&rx_buffer[RESP_MSG_POLL_RX_TS_IDX], &poll_rx_ts);
                    resp_msg_get_ts(&rx_buffer[RESP_MSG_RESP_TX_TS_IDX], &resp_tx_ts);

                    /* Compute time of flight and distance, using clock offset ratio to correct for differing local and remote clock rates */
                    rtd_init = resp_rx_ts - poll_tx_ts;
                    rtd_resp = resp_tx_ts - poll_rx_ts;

                    tof = ((rtd_init - rtd_resp * (1 - clockOffsetRatio)) / 2.0) * DWT_TIME_UNITS;
                    *distance = tof * SPEED_OF_LIGHT;

                    /* Display computed distance on LCD. */
                    /*
                                        snprintf(dist_str, sizeof(dist_str), "DIST: %3.2f m", distance);
                    test_run_info((unsigned char *)dist_str);
                    */

                    // Serial.printf("\n{%3.2f}\0",distance);
                    //
                   // Serial.printf("\n{%3.2f}\0", kalman->updateFilter(*distance));
                   *distance =  kalman->updateFilter(*distance);
                }
            }
        }
        else
        {
            /* Clear RX error/timeout events in the DW IC status register. */
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
        }
        /* Execute a delay between ranging exchanges. */
      //  Sleep(RNG_DELAY_MS);
}
