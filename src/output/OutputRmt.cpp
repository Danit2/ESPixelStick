/*
* OutputRmt.cpp - driver code for ESPixelStick RMT Channel
*
* Migrated to support ESP-IDF v5 RMT API (rmt_tx_channel_handle_t, rmt_new_tx_channel, rmt_tx_write_items, ...)
* while keeping backward compatibility with legacy IDF < 5 (direct RMT register access).
*
* Notes:
*  - For IDF5, this implementation writes prepared rmt_item32_t chunks to the new driver using rmt_tx_write_items()
*    and waits for completion with rmt_tx_wait_all_done(). It avoids direct access to RMT.* registers which
*    are not present in IDF5.
*  - For very large or continuous streams a callback/encoder approach (like esp-idf led_strip sample) is more efficient.
*
*/
#include "ESPixelStick.h"
#ifdef ARDUINO_ARCH_ESP32
#include "output/OutputRmt.hpp"

#include <driver/rmt.h>
#if defined(RMT_API_V5) && RMT_API_V5
    #include <driver/rmt_tx.h>
#endif

// forward declaration for the isr handler (legacy IDF)
#if !defined(RMT_API_V5) || (RMT_API_V5 == 0)
static void IRAM_ATTR   rmt_intr_handler (void* param);
static rmt_isr_handle_t RMT_intr_handle = NULL;
static c_OutputRmt *    rmt_isr_ThisPtrs[MAX_NUM_RMT_CHANNELS];
static bool             InIsr = false;
#endif

#ifdef USE_RMT_DEBUG_COUNTERS
static uint32_t RawIsrCounter = 0;
#endif // def USE_RMT_DEBUG_COUNTERS

static TaskHandle_t SendFrameTaskHandle = NULL;
static BaseType_t xHigherPriorityTaskWoken = pdTRUE;
static uint32_t FrameCompletes = 0;
static uint32_t FrameTimeouts = 0;

//----------------------------------------------------------------------------
// Background task: If you're using legacy interrupt-driven flow this will still be used.
// For IDF5 the StartNewFrame() performs a chunked blocking transmit, the task loop still paces frames.
void RMT_Task (void *arg)
{
    while(1)
    {
        delay(1);
#if !defined(RMT_API_V5) || (RMT_API_V5 == 0)
        // process all possible channels (legacy interrupt-driven)
        for (c_OutputRmt * pRmt : rmt_isr_ThisPtrs)
        {
            if(nullptr != pRmt)
            {
                if (pRmt->StartNextFrame())
                {
                    uint32_t NotificationValue = ulTaskNotifyTake( pdTRUE, pdMS_TO_TICKS(100) );
                    if(1 == NotificationValue)
                    {
                        ++FrameCompletes;
                    }
                    else
                    {
                        ++FrameTimeouts;
                    }
                }
            }
        }
#else
        // IDF5: still allow StartNextFrame hooks if driver uses the task model; noop otherwise.
        // Nothing specific here; sleep and loop.
#endif
    }
} // RMT_Task

//----------------------------------------------------------------------------
// Constructor
c_OutputRmt::c_OutputRmt()
{
    memset((void *)&Intensity2Rmt[0], 0x00, sizeof(Intensity2Rmt));
    memset((void *)&SendBuffer[0],    0x00, sizeof(SendBuffer));
#ifdef USE_RMT_DEBUG_COUNTERS
    memset((void *)&BitTypeCounters[0], 0x00, sizeof(BitTypeCounters));
#endif
}

//----------------------------------------------------------------------------
// Destructor
c_OutputRmt::~c_OutputRmt ()
{
    if (HasBeenInitialized)
    {
        String Reason = (F("Shutting down an RMT channel requires a reboot"));
        RequestReboot(Reason, 100000);

#if defined(RMT_API_V5) && RMT_API_V5
        // release new API channel if allocated
        if (tx_channel_handle)
        {
            rmt_del_tx_channel(tx_channel_handle);
            tx_channel_handle = nullptr;
        }
#else
        DisableRmtInterrupts;
        ClearRmtInterrupts;
        rmt_isr_ThisPtrs[OutputRmtConfig.RmtChannelId] = (c_OutputRmt*)nullptr;
        RMT.data_ch[OutputRmtConfig.RmtChannelId] = 0x00;
#endif
    }
}

//----------------------------------------------------------------------------
// Legacy ISR (only compiled for old API)
#if !defined(RMT_API_V5) || (RMT_API_V5 == 0)
static void IRAM_ATTR rmt_intr_handler (void* param)
{
    RMT_DEBUG_COUNTER(RawIsrCounter++);
    if(!InIsr)
    {
        InIsr = true;

        uint32_t isrFlags = RMT.int_st.val;
        RMT.int_clr.val = uint32_t(-1);

        while(0 != isrFlags)
        {
            for (auto CurrentRmtChanThisPtr : rmt_isr_ThisPtrs)
            {
                if(nullptr != CurrentRmtChanThisPtr)
                {
                    CurrentRmtChanThisPtr->ISR_Handler(isrFlags);
                }
            }

            isrFlags = RMT.int_st.val;
            RMT.int_clr.val = uint32_t(-1);
        }
        InIsr = false;
    }
}
#endif // legacy ISR

//----------------------------------------------------------------------------
// Begin - initialize channel
void c_OutputRmt::Begin (OutputRmtConfig_t config, c_OutputCommon * _pParent )
{
    do // once
    {
        if(HasBeenInitialized)
        {
            ResetGpio(OutputRmtConfig.DataPin);
        }

        OutputRmtConfig = config;

#if defined(SUPPORT_OutputType_DMX) || defined(SUPPORT_OutputType_Serial) || defined(SUPPORT_OutputType_Renard)
        if ((nullptr == OutputRmtConfig.pPixelDataSource) && (nullptr == OutputRmtConfig.pSerialDataSource))
#else
        if (nullptr == OutputRmtConfig.pPixelDataSource)
#endif
        {
            String Reason = (F("Invalid RMT configuration parameters. Rebooting"));
            RequestReboot(Reason, 10000);
            break;
        }

        NumRmtSlotsPerIntensityValue = OutputRmtConfig.IntensityDataWidth + ((OutputRmtConfig.SendInterIntensityBits) ? 1 : 0);
        if(OutputRmtConfig_t::DataDirection_t::MSB2LSB == OutputRmtConfig.DataDirection)
        {
            TxIntensityDataStartingMask = 1 << (OutputRmtConfig.IntensityDataWidth - 1);
        }
        else
        {
            TxIntensityDataStartingMask = 1;
        }

#if defined(RMT_API_V5) && RMT_API_V5
        // -----------------------
        // IDF5+: use rmt_tx_channel API
        // -----------------------
        rmt_tx_channel_config_t tx_cfg = {};
        tx_cfg.gpio_num = OutputRmtConfig.DataPin;
        tx_cfg.clk_src = RMT_CLK_SRC_DEFAULT;
        // resolution_hz controls the timing resolution: compute from desired tick length
        // The header defines RMT_TickLengthNS using clk divisor 2 assuming 80MHz/2 -> 40MHz ticks.
        // We'll set a resolution close to previous behaviour: 80MHz / RMT_Clock_Divisor = 40MHz -> resolution_hz = 40e6
        tx_cfg.resolution_hz = uint32_t(RMT_ClockRate / RMT_Clock_Divisor); // e.g., 80e6/2 = 40e6
        // allocate at least enough mem symbols for our expected chunk size
        tx_cfg.mem_block_symbols = (NUM_RMT_SLOTS);
        tx_cfg.trans_queue_depth = 4; // allow queueing multiple transactions
        tx_cfg.intr_flags = 0;
        tx_cfg.invert_out = false;
        tx_cfg.with_dma = false; // keep false by default; optionally true on DMA-capable chips

        esp_err_t err = rmt_new_tx_channel(&tx_cfg, &tx_channel_handle);
        if (err != ESP_OK)
        {
            String Reason = (F("rmt_new_tx_channel failed - no free RMT channels or unsupported features. Rebooting"));
            RequestReboot(Reason, 10000);
            break;
        }
        tx_channel_numeric = OutputRmtConfig.RmtChannelIdNumeric;

        // Optionally apply carrier (if needed) or other tx properties via rmt_apply_carrier, rmt_enable etc.
        // Store MaxNumRmtSlotsPerInterrupt (we will chunk per mem size)
        MaxNumRmtSlotsPerInterrupt = (NUM_RMT_SLOTS / 2);
#else
        // -----------------------
        // Legacy IDF: old rmt_config & isr code
        // -----------------------
        rmt_config_t RmtConfig;
        RmtConfig.rmt_mode = rmt_mode_t::RMT_MODE_TX;
        RmtConfig.channel = OutputRmtConfig.RmtChannelId;
        RmtConfig.gpio_num = OutputRmtConfig.DataPin;
        RmtConfig.clk_div = RMT_Clock_Divisor;
        RmtConfig.mem_block_num = rmt_reserve_memsize_t::RMT_MEM_64;

        RmtConfig.tx_config.carrier_freq_hz = uint32_t(10); // cannot be zero due to a driver bug
        RmtConfig.tx_config.carrier_level = rmt_carrier_level_t::RMT_CARRIER_LEVEL_LOW;
        RmtConfig.tx_config.carrier_duty_percent = 50;
        RmtConfig.tx_config.idle_level = OutputRmtConfig.idle_level;
        RmtConfig.tx_config.carrier_en = false;
        RmtConfig.tx_config.loop_en = true;
        RmtConfig.tx_config.idle_output_en = true;

        ResetGpio(OutputRmtConfig.DataPin);
        ESP_ERROR_CHECK(rmt_config(&RmtConfig));

        if(NULL == RMT_intr_handle)
        {
            for(auto & currentThisPtr : rmt_isr_ThisPtrs) currentThisPtr = nullptr;
            ESP_ERROR_CHECK(rmt_isr_register(rmt_intr_handler, this, ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_LEVEL1 | ESP_INTR_FLAG_SHARED, &RMT_intr_handle));
        }

        RMT.conf_ch[OutputRmtConfig.RmtChannelId].conf1.tx_conti_mode = 0;
        RMT.apb_conf.mem_tx_wrap_en = 1;

        ISR_ResetRmtBlockPointers ();
        RMT.conf_ch[OutputRmtConfig.RmtChannelId].conf1.apb_mem_rst = 0;
        RMT.conf_ch[OutputRmtConfig.RmtChannelId].conf1.mem_owner = RMT_MEM_OWNER_TX;

        RMT.apb_conf.fifo_mask = RMT_DATA_MODE_MEM;
        memset((void*)&RMTMEM.chan[OutputRmtConfig.RmtChannelId].data32[0], 0x0, sizeof(RMTMEM.chan[0].data32));
#endif

        UpdateBitXlatTable(OutputRmtConfig.CitrdsArray);

        if(!SendFrameTaskHandle)
        {
            xTaskCreatePinnedToCore(RMT_Task, "RMT_Task", 4096, NULL, 5, &SendFrameTaskHandle, 1);
            vTaskPrioritySet(SendFrameTaskHandle, 5);
        }

#if defined(RMT_API_V5) && RMT_API_V5
        pParent = _pParent;
        HasBeenInitialized = true;
#else
        pParent = _pParent;
        rmt_isr_ThisPtrs[OutputRmtConfig.RmtChannelId] = this;
        HasBeenInitialized = true;
#endif

    } while (false);
} // Begin

//----------------------------------------------------------------------------
// Update translation table (unchanged)
void c_OutputRmt::UpdateBitXlatTable(const CitrdsArray_t * CitrdsArray)
{
    if (nullptr != OutputRmtConfig.CitrdsArray)
    {
        const ConvertIntensityToRmtDataStreamEntry_t *CurrentTranslation = CitrdsArray;
        while (CurrentTranslation->Id != RmtDataBitIdType_t::RMT_LIST_END)
        {
            SetIntensity2Rmt(CurrentTranslation->Translation, CurrentTranslation->Id);
            CurrentTranslation++;
        }
    }
    else
    {
        logcon(String(CN_stars) + F(" ERROR: Missing pointer to RMT bit translation values (1) ") + CN_stars);
    }
}

//----------------------------------------------------------------------------
// Validate bit table (unchanged)
bool c_OutputRmt::ValidateBitXlatTable(const CitrdsArray_t * CitrdsArray)
{
    bool Response = false;
    if (nullptr != OutputRmtConfig.CitrdsArray)
    {
        const ConvertIntensityToRmtDataStreamEntry_t *CurrentTranslation = CitrdsArray;
        while (CurrentTranslation->Id != RmtDataBitIdType_t::RMT_LIST_END)
        {
            SetIntensity2Rmt(CurrentTranslation->Translation, CurrentTranslation->Id);

            if(Intensity2Rmt[CurrentTranslation->Id].val != CurrentTranslation->Translation.val)
            {
                logcon(String(CN_stars) + F("ERROR: incorrect bit translation deteced. Chan: ") + String(OutputRmtConfig.RmtChannelId) +
                        F(" Slot: ") + String(CurrentTranslation->Id) +
                        F(" Got: 0x") + String(Intensity2Rmt[CurrentTranslation->Id].val, HEX) +
                        F(" Expected: 0x") + String(CurrentTranslation->Translation.val));
            }

            CurrentTranslation++;
        }
    }
    else
    {
        logcon(String(CN_stars) + F("ERROR: Missing pointer to RMT bit translation values (2)") + CN_stars);
    }
    return Response;
}

//----------------------------------------------------------------------------
// GetStatus (limited debug for IDF5 - many RMT registers not available)
void c_OutputRmt::GetStatus (ArduinoJson::JsonObject& jsonStatus)
{
    jsonStatus[F("NumRmtSlotOverruns")] = NumRmtSlotOverruns;
#ifdef USE_RMT_DEBUG_COUNTERS
    jsonStatus[F("OutputIsPaused")] = OutputIsPaused;
    JsonObject debugStatus = jsonStatus["RMT Debug"].to<JsonObject>();
#if defined(RMT_API_V5) && RMT_API_V5
    debugStatus["Note"] = F("Using IDF5 RMT API - raw RMT registers are not readable");
    debugStatus["tx_channel_handle"] = (nullptr != tx_channel_handle) ? 1 : 0;
#else
    debugStatus["RmtChannelId"]                 = OutputRmtConfig.RmtChannelId;
    debugStatus["GPIO"]                         = OutputRmtConfig.DataPin;
    debugStatus["conf0"]                        = String(RMT.conf_ch[OutputRmtConfig.RmtChannelId].conf0.val, HEX);
    debugStatus["conf1"]                        = String(RMT.conf_ch[OutputRmtConfig.RmtChannelId].conf1.val, HEX);
    debugStatus["tx_lim_ch"]                    = String(RMT.tx_lim_ch[OutputRmtConfig.RmtChannelId].limit);
    uint32_t temp = RMT.int_ena.val;
    debugStatus["Raw int_ena"]                  = "0x" + String (temp, HEX);
    temp = RMT.int_st.val;
    debugStatus["Raw int_st"]                   = "0x" + String (temp, HEX);
#endif
    debugStatus["FrameCompletes"]               = String (FrameCompletes);
    debugStatus["FrameTimeouts"]                = String (FrameTimeouts);
    debugStatus["RmtEntriesTransfered"]         = RmtEntriesTransfered;
    debugStatus["RmtXmtFills"]                  = RmtXmtFills;
    debugStatus["RanOutOfData"]                 = RanOutOfData;
#endif
}

//----------------------------------------------------------------------------
// ISR_CreateIntensityData unchanged - builds SendBuffer entries
void IRAM_ATTR c_OutputRmt::ISR_CreateIntensityData ()
{
    register uint32_t OneBitValue  = Intensity2Rmt[RmtDataBitIdType_t::RMT_DATA_BIT_ONE_ID].val;
    register uint32_t ZeroBitValue = Intensity2Rmt[RmtDataBitIdType_t::RMT_DATA_BIT_ZERO_ID].val;

    uint32_t IntensityValue;
    uint32_t NumAvailableBufferSlotsToFill = NUM_RMT_SLOTS - NumUsedEntriesInSendBuffer;
    while ((NumAvailableBufferSlotsToFill > NumRmtSlotsPerIntensityValue) && ThereIsDataToSend)
    {
        ThereIsDataToSend = ISR_GetNextIntensityToSend(IntensityValue);
        RMT_DEBUG_COUNTER(IntensityValuesSent++);
#ifdef USE_RMT_DEBUG_COUNTERS
        if(200 < IntensityValue)
        {
            ++RmtWhiteDetected;
        }
#endif

        uint32_t bitmask = TxIntensityDataStartingMask;
        for (uint32_t BitCount = OutputRmtConfig.IntensityDataWidth; 0 < BitCount; --BitCount)
        {
            RMT_DEBUG_COUNTER(IntensityBitsSent++);
            ISR_WriteToBuffer((IntensityValue & bitmask) ? OneBitValue : ZeroBitValue);
            if(OutputRmtConfig_t::DataDirection_t::MSB2LSB == OutputRmtConfig.DataDirection)
            {
                bitmask >>= 1;
            }
            else
            {
                bitmask <<= 1;
            }
#ifdef USE_RMT_DEBUG_COUNTERS
            if (IntensityValue & bitmask)
            {
                BitTypeCounters[int(RmtDataBitIdType_t::RMT_DATA_BIT_ONE_ID)]++;
            }
            else
            {
                BitTypeCounters[int(RmtDataBitIdType_t::RMT_DATA_BIT_ZERO_ID)]++;
            }
#endif
        }

        if (OutputRmtConfig.SendEndOfFrameBits && !ThereIsDataToSend)
        {
            ISR_WriteToBuffer(Intensity2Rmt[RmtDataBitIdType_t::RMT_END_OF_FRAME].val);
            RMT_DEBUG_COUNTER(BitTypeCounters[int(RmtDataBitIdType_t::RMT_END_OF_FRAME)]++);
        }
        else if (OutputRmtConfig.SendInterIntensityBits)
        {
            ISR_WriteToBuffer(Intensity2Rmt[RmtDataBitIdType_t::RMT_STOP_START_BIT_ID].val);
            RMT_DEBUG_COUNTER(BitTypeCounters[int(RmtDataBitIdType_t::RMT_STOP_START_BIT_ID)]++);
        }

        NumAvailableBufferSlotsToFill = (NUM_RMT_SLOTS - 1) - NumUsedEntriesInSendBuffer;
    }
}

//----------------------------------------------------------------------------
// ISR_GetNextIntensityToSend unchanged
inline bool IRAM_ATTR c_OutputRmt::ISR_GetNextIntensityToSend(uint32_t &DataToSend)
{
    if (nullptr != OutputRmtConfig.pPixelDataSource)
    {
        return OutputRmtConfig.pPixelDataSource->ISR_GetNextIntensityToSend(DataToSend);
    }
#if defined(SUPPORT_OutputType_DMX) || defined(SUPPORT_OutputType_Serial) || defined(SUPPORT_OutputType_Renard)
    else
    {
        return OutputRmtConfig.pSerialDataSource->ISR_GetNextIntensityToSend(DataToSend);
    }
#else
    return false;
#endif
}

//----------------------------------------------------------------------------
// ISR_Handler: legacy only (kept for IDF <5)
void IRAM_ATTR c_OutputRmt::ISR_Handler (uint32_t isrFlags)
{
#if !defined(RMT_API_V5) || (RMT_API_V5 == 0)
    RMT_DEBUG_COUNTER(++ISRcounter);
    if(OutputIsPaused)
    {
        DisableRmtInterrupts;
    }
    else if (isrFlags & RMT_INT_TX_END_BIT )
    {
        RMT_DEBUG_COUNTER(++IntTxEndIsrCounter);
        DisableRmtInterrupts;

        if(NumUsedEntriesInSendBuffer)
        {
            RMT_DEBUG_COUNTER(++FailedToSendAllData);
        }

        vTaskNotifyGiveFromISR( SendFrameTaskHandle, &xHigherPriorityTaskWoken );
    }
    else if (isrFlags & RMT_INT_THR_EVNT_BIT )
    {
        RMT_DEBUG_COUNTER(++IntTxThrIsrCounter);

        if(NumUsedEntriesInSendBuffer)
        {
            RMT_DEBUG_COUNTER(++SendBlockIsrCounter);
            ISR_TransferIntensityDataToRMT( MaxNumRmtSlotsPerInterrupt );
            ISR_CreateIntensityData();

            if (!ThereIsDataToSend && 0 == NumUsedEntriesInSendBuffer)
            {
                RMT_DEBUG_COUNTER(++RanOutOfData);
                DisableRmtInterrupts;
                vTaskNotifyGiveFromISR( SendFrameTaskHandle, &xHigherPriorityTaskWoken );
            }
        }
    }
#ifdef USE_RMT_DEBUG_COUNTERS
    else
    {
        RMT_DEBUG_COUNTER(++UnknownISRcounter);
        if (isrFlags & RMT_INT_ERROR_BIT)
        {
            ErrorIsr++;
        }
        if (isrFlags & RMT_INT_RX_END_BIT)
        {
            RxIsr++;
        }
    }
#endif
#endif
} // ISR_Handler

//----------------------------------------------------------------------------
// ISR_MoreDataToSend unchanged
inline bool IRAM_ATTR c_OutputRmt::ISR_MoreDataToSend()
{
#if defined(SUPPORT_OutputType_DMX) || defined(SUPPORT_OutputType_Serial) || defined(SUPPORT_OutputType_Renard)
    if (nullptr != OutputRmtConfig.pPixelDataSource)
    {
        return OutputRmtConfig.pPixelDataSource->ISR_MoreDataToSend();
    }
    else
    {
        return OutputRmtConfig.pSerialDataSource->ISR_MoreDataToSend();
    }
#else
    return OutputRmtConfig.pPixelDataSource->ISR_MoreDataToSend();
#endif
}

//----------------------------------------------------------------------------
// ISR_ResetRmtBlockPointers (legacy registers) or reset buffer indices for IDF5
inline void IRAM_ATTR c_OutputRmt::ISR_ResetRmtBlockPointers()
{
#if defined(RMT_API_V5) && RMT_API_V5
    // IDF5: nothing to poke in hw registers; just reset local buffer pointers
    RmtBufferWriteIndex = 0;
    SendBufferWriteIndex = 0;
    SendBufferReadIndex  = 0;
    NumUsedEntriesInSendBuffer = 0;
#else
    RMT.conf_ch[OutputRmtConfig.RmtChannelId].conf1.mem_rd_rst = 1;
    RMT.conf_ch[OutputRmtConfig.RmtChannelId].conf1.mem_rd_rst = 0;
    RmtBufferWriteIndex = 0;
    SendBufferWriteIndex = 0;
    SendBufferReadIndex  = 0;
    NumUsedEntriesInSendBuffer = 0;
#endif
}

//----------------------------------------------------------------------------
// ISR_StartNewDataFrame unchanged
inline void IRAM_ATTR c_OutputRmt::ISR_StartNewDataFrame()
{
    if (nullptr != OutputRmtConfig.pPixelDataSource)
    {
        OutputRmtConfig.pPixelDataSource->StartNewFrame();
    }
#if defined(SUPPORT_OutputType_DMX) || defined(SUPPORT_OutputType_Serial) || defined(SUPPORT_OutputType_Renard)
    else
    {
        OutputRmtConfig.pSerialDataSource->StartNewFrame();
    }
#endif
}

//----------------------------------------------------------------------------
// ISR_TransferIntensityDataToRMT: legacy direct-write or transfer into local ring buffer (IDF5 uses rmt_tx_write_items instead)
void IRAM_ATTR c_OutputRmt::ISR_TransferIntensityDataToRMT (uint32_t MaxNumEntriesToTransfer)
{
#if defined(RMT_API_V5) && RMT_API_V5
    // For IDF5 we don't write directly to hardware memory here.
    // This function will simply ensure the ring buffer indices are consistent.
    uint32_t NumEntriesToTransfer = min(NumUsedEntriesInSendBuffer, MaxNumEntriesToTransfer);
    // No direct hardware transfer; the StartNewFrame will call rmt_tx_write_items().
    (void)NumEntriesToTransfer;
#else
    uint32_t NumEntriesToTransfer = min(NumUsedEntriesInSendBuffer, MaxNumEntriesToTransfer);

    while(NumEntriesToTransfer)
    {
        RMTMEM.chan[OutputRmtConfig.RmtChannelId].data32[RmtBufferWriteIndex++].val = SendBuffer[SendBufferReadIndex++].val;
        RmtBufferWriteIndex &= uint32_t(NUM_RMT_SLOTS - 1);
        SendBufferReadIndex &= uint32_t(NUM_RMT_SLOTS - 1);
        --NumEntriesToTransfer;
        --NumUsedEntriesInSendBuffer;
    }
    RMTMEM.chan[OutputRmtConfig.RmtChannelId].data32[RmtBufferWriteIndex].val = uint32_t(0);
#endif
}

//----------------------------------------------------------------------------
// ISR_WriteToBuffer unchanged
inline void IRAM_ATTR c_OutputRmt::ISR_WriteToBuffer(uint32_t value)
{
    SendBuffer[SendBufferWriteIndex++].val = value;
    SendBufferWriteIndex &= uint32_t(NUM_RMT_SLOTS - 1);
    ++NumUsedEntriesInSendBuffer;
}

//----------------------------------------------------------------------------
// PauseOutput unchanged
void c_OutputRmt::PauseOutput(bool PauseOutput)
{
    if (OutputIsPaused == PauseOutput)
    {
        // no change
    }
    else if (PauseOutput)
    {
#if !defined(RMT_API_V5) || (RMT_API_V5 == 0)
        DisableRmtInterrupts;
        ClearRmtInterrupts;
#else
        // IDF5: we can stop by disabling channel if desired
        if (tx_channel_handle)
        {
            rmt_disable(tx_channel_handle);
        }
#endif
    }
    OutputIsPaused = PauseOutput;
}

//----------------------------------------------------------------------------
// StartNewFrame - IDF5: chunked blocking write to rmt driver; legacy: original code
bool c_OutputRmt::StartNewFrame ()
{
    bool Response = false;
    do
    {
        if(OutputIsPaused) { break; }

#if defined(RMT_API_V5) && RMT_API_V5
        // -----------------------
        // IDF5 path: build the send buffer (same as legacy) then write in chunks to the driver
        // -----------------------
        ISR_ResetRmtBlockPointers();

        uint32_t NumInterFrameRmtSlotsCount = 0;
        while (NumInterFrameRmtSlotsCount < OutputRmtConfig.NumIdleBits)
        {
            ISR_WriteToBuffer (Intensity2Rmt[RmtDataBitIdType_t::RMT_INTERFRAME_GAP_ID].val);
            ++NumInterFrameRmtSlotsCount;
        }
        uint32_t NumFrameStartRmtSlotsCount = 0;
        while (NumFrameStartRmtSlotsCount++ < OutputRmtConfig.NumFrameStartBits)
        {
            ISR_WriteToBuffer (Intensity2Rmt[RmtDataBitIdType_t::RMT_STARTBIT_ID].val);
        }

        ISR_StartNewDataFrame();
        ThereIsDataToSend = ISR_MoreDataToSend();
        ISR_CreateIntensityData();

        // now send the buffer contents in chunks via rmt_tx_write_items
        // we will send until NumUsedEntriesInSendBuffer == 0 and no more data
        const TickType_t tx_wait_ticks = pdMS_TO_TICKS(500); // wait 500ms per chunk
        while( NumUsedEntriesInSendBuffer > 0 || ThereIsDataToSend )
        {
            // Prepare a local contiguous array of items to send (because SendBuffer is ring buffer)
            uint32_t available = NumUsedEntriesInSendBuffer;
            if (available == 0 && ThereIsDataToSend)
            {
                ISR_CreateIntensityData();
                available = NumUsedEntriesInSendBuffer;
                if (available == 0) break; // nothing to send
            }

            // compute chunk size (max safe chunk: NUM_RMT_SLOTS - 1)
            size_t chunk = (available > (NUM_RMT_SLOTS - 1)) ? (NUM_RMT_SLOTS - 1) : available;

            // assemble contiguous array `chunkItems` from ring buffer
            // use stack-located temporary buffer up to reasonable size
            rmt_item32_t chunkItems[NUM_RMT_SLOTS];
            for (size_t i = 0; i < chunk; ++i)
            {
                chunkItems[i] = SendBuffer[ (SendBufferReadIndex + i) & (NUM_RMT_SLOTS - 1) ];
            }

            // issue transmit
            rmt_tx_buffer_handle_t buf_handle = nullptr;
            esp_err_t err = rmt_tx_write_items(tx_channel_handle, chunkItems, chunk, tx_wait_ticks, &buf_handle);
            if (err != ESP_OK)
            {
                // transmit failed; abort
                break;
            }
            // wait for this transaction to complete
            err = rmt_tx_wait_all_done(tx_channel_handle, tx_wait_ticks);
            if (buf_handle) rmt_tx_buffer_free(buf_handle);

            // advance ring buffer read pointer & counters
            SendBufferReadIndex = (SendBufferReadIndex + chunk) & (NUM_RMT_SLOTS - 1);
            NumUsedEntriesInSendBuffer -= chunk;

            // refill buffer if needed
            ISR_CreateIntensityData();
        } // while sending chunks

        // optionally send trailing end-of-frame bits if configured (already added by ISR_CreateIntensityData)
        Response = true;
#else
        // -----------------------
        // Legacy path (original code that manipulates RMT registers)
        // -----------------------
        if(InterrupsAreEnabled)
        {
            RMT_DEBUG_COUNTER(IncompleteFrame++);
        }

        DisableRmtInterrupts;
        RMT.conf_ch[OutputRmtConfig.RmtChannelId].conf1.tx_start = 0;

        ISR_ResetRmtBlockPointers ();

        uint32_t NumInterFrameRmtSlotsCount = 0;
        while (NumInterFrameRmtSlotsCount < OutputRmtConfig.NumIdleBits)
        {
            ISR_WriteToBuffer (Intensity2Rmt[RmtDataBitIdType_t::RMT_INTERFRAME_GAP_ID].val);
            ++NumInterFrameRmtSlotsCount;
            RMT_DEBUG_COUNTER(BitTypeCounters[int(RmtDataBitIdType_t::RMT_INTERFRAME_GAP_ID)]++);
        }

        uint32_t NumFrameStartRmtSlotsCount = 0;
        while (NumFrameStartRmtSlotsCount++ < OutputRmtConfig.NumFrameStartBits)
        {
            ISR_WriteToBuffer (Intensity2Rmt[RmtDataBitIdType_t::RMT_STARTBIT_ID].val);
            RMT_DEBUG_COUNTER(BitTypeCounters[int(RmtDataBitIdType_t::RMT_STARTBIT_ID)]++);
        }

#ifdef USE_RMT_DEBUG_COUNTERS
        FrameStartCounter++;
        IntensityValuesSentLastFrame = IntensityValuesSent;
        IntensityValuesSent          = 0;
        IntensityBitsSentLastFrame   = IntensityBitsSent;
        IntensityBitsSent            = 0;
#endif

        ISR_StartNewDataFrame ();
        ThereIsDataToSend = ISR_MoreDataToSend();
        ISR_CreateIntensityData ();

        ISR_TransferIntensityDataToRMT(NUM_RMT_SLOTS - 1);
        ISR_CreateIntensityData ();

        RMT.tx_lim_ch[OutputRmtConfig.RmtChannelId].limit = MaxNumRmtSlotsPerInterrupt;

        ClearRmtInterrupts;
        EnableRmtInterrupts;
        vPortYieldOtherCore(0);

        RMT.conf_ch[OutputRmtConfig.RmtChannelId].conf1.tx_start = 1;
        delay(1);

        Response = true;
#endif

    } while(false);

    return Response;
} // StartNewFrame

#endif // ARDUINO_ARCH_ESP32
