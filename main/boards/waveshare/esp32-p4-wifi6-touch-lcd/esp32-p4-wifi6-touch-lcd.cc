#include "wifi_board.h"
#include "codecs/box_audio_codec.h"
#include "application.h"
#include "display/lcd_display.h"
#include "button.h"
#include "power_save_timer.h"

#include "esp_video.h"
#include "esp_video_init.h"
#include "esp_cam_sensor_xclk.h"

#include "esp_lcd_panel_ops.h"
#include "esp_lcd_mipi_dsi.h"
#include "esp_ldo_regulator.h"

#if CONFIG_BOARD_TYPE_WAVESHARE_ESP32_P4_WIFI6_TOUCH_LCD_4B
#include "esp_lcd_st7703.h"
#elif CONFIG_BOARD_TYPE_WAVESHARE_ESP32_P4_WIFI6_TOUCH_LCD_4_3
#include "esp_lcd_st7701.h"
#elif CONFIG_BOARD_TYPE_WAVESHARE_ESP32_P4_WIFI6_TOUCH_LCD_5
#include "esp_lcd_hx8394.h"
#elif CONFIG_BOARD_TYPE_WAVESHARE_ESP32_P4_WIFI6_TOUCH_LCD_7B
#include "esp_lcd_ek79007.h"
#elif CONFIG_BOARD_TYPE_WAVESHARE_ESP32_P4_WIFI6_TOUCH_LCD_3_4C || CONFIG_BOARD_TYPE_WAVESHARE_ESP32_P4_WIFI6_TOUCH_LCD_4C \
    || CONFIG_BOARD_TYPE_WAVESHARE_ESP32_P4_WIFI6_TOUCH_LCD_8   || CONFIG_BOARD_TYPE_WAVESHARE_ESP32_P4_WIFI6_TOUCH_LCD_10_1
#include "esp_lcd_jd9365.h"
#elif CONFIG_BOARD_TYPE_WAVESHARE_ESP32_P4_WIFI6_TOUCH_LCD_7
#include "esp_lcd_ili9881c.h"
#endif

#include "config.h"
#include "lcd_init_cmds.h"

#include <esp_log.h>
#include <driver/i2c_master.h>
#include <esp_lvgl_port.h>
#include "esp_lcd_touch_gt911.h"
#define TAG "WaveshareEsp32p4"

#if CUSTOM_LCDDISPLAY
class CustomLcdDisplay : public LcdDisplay {
public:
    CustomLcdDisplay(esp_lcd_panel_io_handle_t io, esp_lcd_panel_handle_t panel)
        : LcdDisplay(io, panel, DISPLAY_WIDTH, DISPLAY_HEIGHT) {
        lv_init();
        lvgl_port_cfg_t port_cfg = ESP_LVGL_PORT_INIT_CONFIG();
        ESP_ERROR_CHECK(lvgl_port_init(&port_cfg));
        const lvgl_port_display_cfg_t disp_cfg = {
            .io_handle = io,
            .panel_handle = panel,
            .buffer_size = static_cast<uint32_t>(DISPLAY_WIDTH * 50),
            .double_buffer = false,
            .hres = DISPLAY_WIDTH,
            .vres = DISPLAY_HEIGHT,
            .rotation = {
                .swap_xy = DISPLAY_SWAP_XY,
                .mirror_x = DISPLAY_MIRROR_X,
                .mirror_y = DISPLAY_MIRROR_Y,
            },
            .flags = {
                .buff_dma = true,
                .buff_spiram = true,
                .sw_rotate = true,
            },
        };
        const lvgl_port_display_dsi_cfg_t dsi_cfg = {};
        display_ = lvgl_port_add_disp_dsi(&disp_cfg, &dsi_cfg);
        ESP_ERROR_CHECK(display_ != nullptr ? ESP_OK : ESP_ERR_NO_MEM);
#if CUSTOM_DISPLAY_ROTATION == 90
        constexpr auto rotation = LV_DISPLAY_ROTATION_90;
#elif CUSTOM_DISPLAY_ROTATION == 180
        constexpr auto rotation = LV_DISPLAY_ROTATION_180;
#elif CUSTOM_DISPLAY_ROTATION == 270
        constexpr auto rotation = LV_DISPLAY_ROTATION_270;
#else
        constexpr auto rotation = LV_DISPLAY_ROTATION_0;
#endif
        lvgl_port_lock(0);
        lv_display_set_rotation(display_, rotation);
        if (DISPLAY_OFFSET_X != 0 || DISPLAY_OFFSET_Y != 0) {
            lv_display_set_offset(display_, DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y);
        }
        lvgl_port_unlock();
    }
};
#endif

class WaveshareEsp32p4 : public WifiBoard {
private:
    i2c_master_bus_handle_t i2c_bus_;
    Button boot_button_;
    LcdDisplay *display_;
    EspVideo* camera_ = nullptr;
    PowerSaveTimer* power_save_timer_ = nullptr;

    esp_err_t i2c_device_probe(uint8_t addr) {
        return i2c_master_probe(i2c_bus_, addr, 100);
    }

    void InitializeCodecI2c() {
        i2c_master_bus_config_t i2c_bus_cfg = {
            .i2c_port = I2C_NUM_1,
            .sda_io_num = AUDIO_CODEC_I2C_SDA_PIN,
            .scl_io_num = AUDIO_CODEC_I2C_SCL_PIN,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = 0,
            .trans_queue_depth = 0,
            .flags = { .enable_internal_pullup = 1 },
        };
        ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_cfg, &i2c_bus_));
    }

    static esp_err_t bsp_enable_dsi_phy_power(void) {
#if MIPI_DSI_PHY_PWR_LDO_CHAN > 0
        static esp_ldo_channel_handle_t phy_pwr_chan = NULL;
        esp_ldo_channel_config_t ldo_cfg = {
            .chan_id = MIPI_DSI_PHY_PWR_LDO_CHAN,
            .voltage_mv = MIPI_DSI_PHY_PWR_LDO_VOLTAGE_MV,
        };
        esp_ldo_acquire_channel(&ldo_cfg, &phy_pwr_chan);
        ESP_LOGI(TAG, "MIPI DSI PHY Powered on");
#endif
        return ESP_OK;
    }

    void InitializeLCD() {
        bsp_enable_dsi_phy_power();
        esp_lcd_panel_io_handle_t io = NULL;
        esp_lcd_panel_handle_t disp_panel = NULL;
        esp_lcd_dsi_bus_handle_t mipi_dsi_bus = NULL;

        esp_lcd_dsi_bus_config_t bus_config = {
            .bus_id = 0,
            .num_data_lanes = 2,
            .lane_bit_rate_mbps = LCD_MIPI_DSI_LANE_BITRATE_MBPS,
        };
        esp_lcd_new_dsi_bus(&bus_config, &mipi_dsi_bus);

        ESP_LOGI(TAG, "Install MIPI DSI LCD control panel");
        esp_lcd_dbi_io_config_t dbi_config = {
            .virtual_channel = 0,
            .lcd_cmd_bits = 8,
            .lcd_param_bits = 8,
        };
        esp_lcd_new_panel_io_dbi(mipi_dsi_bus, &dbi_config, &io);

#if CONFIG_BOARD_TYPE_WAVESHARE_ESP32_P4_WIFI6_TOUCH_LCD_7B
        esp_lcd_dpi_panel_config_t dpi_config = {
            .dpi_clk_src = MIPI_DSI_DPI_CLK_SRC_DEFAULT,
            .dpi_clock_freq_mhz = 52,
            .in_color_format = LCD_COLOR_FMT_RGB565,
            .out_color_format = LCD_COLOR_FMT_RGB565,
            .num_fbs = 1,
            .video_timing = {
                .h_size = 1024, .v_size = 600,
                .hsync_pulse_width = 10, .hsync_back_porch = 160, .hsync_front_porch = 160,
                .vsync_pulse_width = 1, .vsync_back_porch = 23, .vsync_front_porch = 12,
            },
        };
        ek79007_vendor_config_t vendor_config = {
            .mipi_config = { .dsi_bus = mipi_dsi_bus, .dpi_config = &dpi_config },
        };
        const esp_lcd_panel_dev_config_t lcd_dev_config = {
            .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
            .bits_per_pixel = 16,
            .reset_gpio_num = PIN_NUM_LCD_RST,
            .vendor_config = &vendor_config,
        };
        esp_lcd_new_panel_ek79007(io, &lcd_dev_config, &disp_panel);
#endif
        esp_lcd_panel_reset(disp_panel);
        esp_lcd_panel_init(disp_panel);

#if CUSTOM_LCDDISPLAY
        display_ = new CustomLcdDisplay(io, disp_panel);
#else
        display_ = new MipiLcdDisplay(io, disp_panel, DISPLAY_WIDTH, DISPLAY_HEIGHT,
                                       DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y,
                                       DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y, DISPLAY_SWAP_XY);
#endif
    }

    void InitializeTouch() {
        esp_lcd_touch_handle_t tp;
        esp_lcd_touch_config_t tp_cfg = {
            .x_max = DISPLAY_WIDTH,
            .y_max = DISPLAY_HEIGHT,
            .rst_gpio_num = GPIO_NUM_NC,
            .int_gpio_num = GPIO_NUM_NC,
            .levels = { .reset = 0, .interrupt = 0 },
            .flags = { .swap_xy = 0, .mirror_x = 0, .mirror_y = 0 },
        };
        esp_lcd_panel_io_handle_t tp_io_handle = NULL;
        esp_lcd_panel_io_i2c_config_t tp_io_config = {
            .dev_addr = ESP_LCD_TOUCH_IO_I2C_GT911_ADDRESS,
            .control_phase_bytes = 1,
            .dc_bit_offset = 0,
            .lcd_cmd_bits = 16,
            .flags = { .disable_control_phase = 1 },
        };

        if (ESP_OK == i2c_device_probe(ESP_LCD_TOUCH_IO_I2C_GT911_ADDRESS)) {
            ESP_LOGI(TAG, "Touch panel found at 0x%02X", ESP_LCD_TOUCH_IO_I2C_GT911_ADDRESS);
        } else if (ESP_OK == i2c_device_probe(ESP_LCD_TOUCH_IO_I2C_GT911_ADDRESS_BACKUP)) {
            ESP_LOGI(TAG, "Touch panel found at backup 0x%02X", ESP_LCD_TOUCH_IO_I2C_GT911_ADDRESS_BACKUP);
            tp_io_config.dev_addr = ESP_LCD_TOUCH_IO_I2C_GT911_ADDRESS_BACKUP;
        } else {
            ESP_LOGE(TAG, "Touch panel NOT found!");
            return;
        }

        tp_io_config.scl_speed_hz = 400 * 1000;
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(i2c_bus_, &tp_io_config, &tp_io_handle));
        ESP_ERROR_CHECK(esp_lcd_touch_new_i2c_gt911(tp_io_handle, &tp_cfg, &tp));

        const lvgl_port_touch_cfg_t touch_cfg = {
            .disp = lv_display_get_default(),
            .handle = tp,
        };
        lvgl_port_add_touch(&touch_cfg);
        ESP_LOGI(TAG, "Touch initialized");

        // --- ИСПРАВЛЕННЫЙ БЛОК: добавляем кликабельный слой и обработчик ---
        lv_obj_t* screen = lv_scr_act();
        if (screen) {
            // Делаем экран кликабельным, чтобы он принимал события
            lv_obj_add_flag(screen, LV_OBJ_FLAG_CLICKABLE);
            
            // Добавляем обработчик нажатия (используем лямбду с захватом this)
            lv_obj_add_event_cb(screen, [](lv_event_t* e) {
                // Получаем указатель на наш класс из user_data
                auto* self = static_cast<WaveshareEsp32p4*>(lv_event_get_user_data(e));
                if (!self) return;

                // 1. Будим таймер сна (включает подсветку, если экран погас)
                if (self->power_save_timer_) {
                    self->power_save_timer_->WakeUp();
                }

                // 2. Запускаем чат, если в режиме ожидания
                auto& app = Application::GetInstance();
                if (app.GetDeviceState() == kDeviceStateIdle || 
                    app.GetDeviceState() == kDeviceStateStarting) {
                    ESP_LOGI(TAG, "Starting chat from touch");
                    app.ToggleChatState();
                }
            }, LV_EVENT_CLICKED, this);
            
            ESP_LOGI(TAG, "Touch event handler registered");
        }
        // --- КОНЕЦ ИСПРАВЛЕННОГО БЛОКА ---
    }

    void InitializeCamera() {
        esp_video_init_csi_config_t base_csi_config = {
            .sccb_config = { .init_sccb = false, .i2c_handle = i2c_bus_, .freq = 400000 },
            .reset_pin = GPIO_NUM_NC,
            .pwdn_pin = GPIO_NUM_NC,
        };
        esp_video_init_config_t cam_config = { .csi = &base_csi_config };
        camera_ = new EspVideo(cam_config);
    }

    void InitializeButtons() {
        boot_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting) {
                EnterWifiConfigMode();
                return;
            }
            app.ToggleChatState();
        });
    }

public:
    WaveshareEsp32p4() : boot_button_(BOOT_BUTTON_GPIO) {
        InitializeCodecI2c();
        InitializeLCD();
        InitializeTouch();
        InitializeCamera();
        InitializeButtons();
        GetBacklight()->RestoreBrightness();

        // Таймер сна: 600 секунд (10 минут) до гашения экрана
        power_save_timer_ = new PowerSaveTimer(-1, 600, -1);
        power_save_timer_->OnEnterSleepMode([this]() {
            ESP_LOGI(TAG, "Entering sleep mode");
            GetBacklight()->SetBrightness(1);
        });
        power_save_timer_->OnExitSleepMode([this]() {
            ESP_LOGI(TAG, "Exiting sleep mode");
            GetBacklight()->RestoreBrightness();
        });
        power_save_timer_->SetEnabled(true);
        ESP_LOGI(TAG, "Power save timer enabled (600s)");
    }

    virtual AudioCodec* GetAudioCodec() override {
        static BoxAudioCodec audio_codec(
            i2c_bus_, AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_GPIO_MCLK, AUDIO_I2S_GPIO_BCLK, AUDIO_I2S_GPIO_WS,
            AUDIO_I2S_GPIO_DOUT, AUDIO_I2S_GPIO_DIN, AUDIO_CODEC_PA_PIN,
            AUDIO_CODEC_ES8311_ADDR, AUDIO_CODEC_ES7210_ADDR, AUDIO_INPUT_REFERENCE,
            30.0f, 2
        );
        return &audio_codec;
    }

    virtual Display* GetDisplay() override { return display_; }
    virtual Camera* GetCamera() override { return camera_; }
    virtual Backlight* GetBacklight() override {
        static PwmBacklight backlight(DISPLAY_BACKLIGHT_PIN, DISPLAY_BACKLIGHT_OUTPUT_INVERT);
        return &backlight;
    }
};

DECLARE_BOARD(WaveshareEsp32p4);
