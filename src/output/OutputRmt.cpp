/*
* OutputRmt.cpp - RMT driver code for ESPixelStick RMT Channel
*
* Compatible with:
*   - ESP32 (classic)
*   - ESP32-S2 / S3
*   - ESP32-C2 / C3 / C6
*
* Skipped automatically for ESP8266 builds.
*
* Project: ESPixelStick
*/

#include "ESPixelStick.h"

#ifdef ARDUINO_ARCH_ESP32   // ----- only compile this file for ESP32 targets -----

#include "output/OutputRmt.hpp"

#include <driver/rmt.h>
#if ESP_IDF_VERSION_MAJOR >= 5
  #include <driver/rmt_tx.h>
  #include <esp_idf_version.h>
#endif

#ifndef RMT_DEBUG_COUNTER
#define RMT_DEBUG_COUNTER(p)
#endif

//--------------------------------------------------------------------------------
// Constructor / Destructor
//--------------------------------------------------------------------------------

c_OutputRmt::c_OutputRmt() {}
c_OutputRmt::~c_OutputRmt()
{
#if ESP_IDF_VERSION_MAJOR < 5
    // Legacy direct register reset
    if (OutputRmtConfig.RmtChannelId != rmt_channel_t(-1))
    {
        rmt_driver_uninstall(OutputRmtConfig.RmtChannelId);
        RMT.apb_conf.fifo_mask = 0;
    }
#else
    // IDF v5 uses handle-based API
    if (rmt_tx_channel_handle != nullptr)
    {
        rmt_del_channel(rmt_tx_channel_handle);
        rmt_tx_channel_handle = nullptr;
    }
#endif
}

//--------------------------------------------------------------------------------
// Begin
//--------------------------------------------------------------------------------

void c_OutputRmt::Begin(OutputRmtConfig_t config, c_OutputCommon* pParentPtr)
{
    OutputRmtConfig = config;
    pParent = pParentPtr;

#if ESP_IDF_VERSION_MAJOR < 5
    // ---------------- OLD ESP32 (IDF 4.x) ----------------
    rmt_config_t rmt_tx;
    rmt_tx.channel    = config.RmtChannelId;
    rmt_tx.gpio_num   = config.DataPin;
    rmt_tx.mem_block_num = 1;
    rmt_tx.clk_div    = 2;
    rmt_tx.tx_config.loop_en = false;
    rmt_tx.tx_config.carrier_en = false;
    rmt_tx.tx_config.idle_output_en = true;
    rmt_tx.tx_config.idle_level = config.idle_level;
    rmt_tx.rmt_mode   = RMT_MODE_TX;

    ESP_ERROR_CHECK(rmt_config(&rmt_tx));
    ESP_ERROR_CHECK(rmt_driver_install(rmt_tx.channel, 0, 0));

#else
    // ---------------- NEW ESP32-Sx/Cx (IDF 5.x+) ----------------
    rmt_tx_channel_config_t tx_chan_config = {
        .gpio_num = config.DataPin,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 40 * 1000 * 1000, // 40 MHz
        .mem_block_symbols = 64,
        .trans_queue_depth = 4,
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &rmt_tx_channel_handle));

    rmt_translator_config_t translator_config = {};
    ESP_ERROR_CHECK(rmt_new_translator(&translator_config, &rmt_translator_handle));

    ESP_ERROR_CHECK(rmt_enable(rmt_tx_channel_handle));
#endif
}

//--------------------------------------------------------------------------------
// Start a new frame
//--------------------------------------------------------------------------------

bool c_OutputRmt::StartNewFrame()
{
#if ESP_IDF_VERSION_MAJOR < 5
    rmt_tx_start(OutputRmtConfig.RmtChannelId, true);
#else
    if (rmt_tx_channel_handle)
    {
        // Use translator-less raw send
        rmt_transmit_config_t tx_cfg = {};
        ESP_ERROR_CHECK(rmt_transmit(rmt_tx_channel_handle, SendBuffer, sizeof(SendBuffer), &tx_cfg));
    }
#endif
    return true;
}

//--------------------------------------------------------------------------------
// Pause output
//--------------------------------------------------------------------------------

void c_OutputRmt::PauseOutput(bool state)
{
    OutputIsPaused = state;
#if ESP_IDF_VERSION_MAJOR < 5
    if (state)
        rmt_tx_stop(OutputRmtConfig.RmtChannelId);
#else
    if (state && rmt_tx_channel_handle)
        ESP_ERROR_CHECK(rmt_disable(rmt_tx_channel_handle));
#endif
}

//--------------------------------------------------------------------------------
// Status
//--------------------------------------------------------------------------------

void c_OutputRmt::GetStatus(ArduinoJson::JsonObject& jsonStatus)
{
    jsonStatus["Driver"] = "RMT";
    jsonStatus["Paused"] = OutputIsPaused;
}

//--------------------------------------------------------------------------------
// Utility helpers
//--------------------------------------------------------------------------------

void c_OutputRmt::UpdateBitXlatTable(const CitrdsArray_t* CitrdsArray)
{
    if (!CitrdsArray) return;
    for (int i = 0; i < RmtDataBitIdType_t::RMT_LIST_END; i++)
    {
        Intensity2Rmt[i] = CitrdsArray[i].Translation;
    }
}

bool c_OutputRmt::ValidateBitXlatTable(const CitrdsArray_t* CitrdsArray)
{
    return (CitrdsArray != nullptr);
}

#endif // ARDUINO_ARCH_ESP32
