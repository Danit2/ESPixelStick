/**
 * @file OutputRmt.cpp
 * @brief Unified RMT output for ESP32 (classic) and ESP32-S2/S3/C2/C3/C6 with ESP-IDF 5+ API
 */

#include "OutputRmt.hpp"
#include "esp_idf_version.h"
#include "driver/rmt_tx.h"
#include "esp_log.h"

static const char* TAG = "OutputRmt";

c_OutputRmt::~c_OutputRmt()
{
#if ESP_IDF_VERSION_MAJOR < 5
    // Legacy ESP32 cleanup
    if (OutputRmtConfig.RmtChannelId < RMT_CHANNEL_MAX)
    {
        RMT.conf_ch[OutputRmtConfig.RmtChannelId].conf1.tx_start = 0;
        RMT.conf_ch[OutputRmtConfig.RmtChannelId].conf1.mem_rd_rst = 1;
        RMT.conf_ch[OutputRmtConfig.RmtChannelId].conf1.mem_rd_rst = 0;
        RMT.conf_ch[OutputRmtConfig.RmtChannelId].conf1.mem_owner = RMT_MEM_OWNER_TX;
        RMT.conf_ch[OutputRmtConfig.RmtChannelId].conf1.apb_mem_rst = 0;
        RMT.conf_ch[OutputRmtConfig.RmtChannelId].conf1.tx_conti_mode = 0;
        RMT.apb_conf.mem_tx_wrap_en = 1;
    }
#else
    if (tx_channel)
    {
        rmt_disable(tx_channel);
        rmt_del_channel(tx_channel);
        tx_channel = nullptr;
    }
#endif
}

// ---------------------------------------------------------------------------

void c_OutputRmt::Begin(OutputRmtConfig_t config, c_OutputCommon* pOutputCommon)
{
    OutputRmtConfig = config;
    pOutput = pOutputCommon;

#if ESP_IDF_VERSION_MAJOR < 5
    // Old style setup for ESP32 (classic)
    RMT.conf_ch[config.RmtChannelId].conf1.tx_conti_mode = 0;
    RMT.apb_conf.mem_tx_wrap_en = 1;
    RMT.conf_ch[config.RmtChannelId].conf1.apb_mem_rst = 0;
    RMT.conf_ch[config.RmtChannelId].conf1.mem_owner = RMT_MEM_OWNER_TX;
    RMT.apb_conf.fifo_mask = RMT_DATA_MODE_MEM;
#else
    // New style setup for ESP-IDF 5.x and newer
    rmt_tx_channel_config_t tx_chan_config = {
        .gpio_num = (gpio_num_t)config.OutputGpioNum,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .mem_block_symbols = 64,
        .resolution_hz = 10'000'000, // 10MHz
        .trans_queue_depth = 4,
    };

    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &tx_channel));

    rmt_bytes_encoder_config_t bytes_encoder_config = {
        .bit0 = {400, 850},
        .bit1 = {850, 400},
        .flags = {.msb_first = 1},
    };

    ESP_ERROR_CHECK(rmt_new_bytes_encoder(&bytes_encoder_config, &bytes_encoder));

    ESP_ERROR_CHECK(rmt_enable(tx_channel));
    ESP_LOGI(TAG, "RMT initialized on GPIO %d (new API)", config.OutputGpioNum);
#endif
}

// ---------------------------------------------------------------------------

void c_OutputRmt::ISR_ResetRmtBlockPointers()
{
#if ESP_IDF_VERSION_MAJOR < 5
    RMT.conf_ch[OutputRmtConfig.RmtChannelId].conf1.mem_rd_rst = 1;
    RMT.conf_ch[OutputRmtConfig.RmtChannelId].conf1.mem_rd_rst = 0;
#else
    // No manual reset needed with new driver
#endif
}

// ---------------------------------------------------------------------------

bool c_OutputRmt::StartNewFrame()
{
#if ESP_IDF_VERSION_MAJOR < 5
    RMT.conf_ch[OutputRmtConfig.RmtChannelId].conf1.tx_start = 0;
    RMT.conf_ch[OutputRmtConfig.RmtChannelId].conf1.tx_start = 1;
#else
    if (!tx_channel || !bytes_encoder)
        return false;

    const uint8_t* data = pOutput->GetBuffer();
    size_t len = pOutput->GetBufferLength();

    rmt_transmit_config_t tx_config = {
        .loop_count = 0,
    };

    esp_err_t err = rmt_transmit(tx_channel, bytes_encoder, data, len, &tx_config);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "RMT transmit failed: %s", esp_err_to_name(err));
        return false;
    }
#endif
    return true;
}
