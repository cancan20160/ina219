#ifndef INA219_H
#define INA219_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/*===========================================================================*/
/* REGISTER DEFINITIONS                                                      */
/*===========================================================================*/
#define INA219_REG_CONFIG           (0x00U)
#define INA219_REG_SHUNT_VOLTAGE    (0x01U)
#define INA219_REG_BUS_VOLTAGE      (0x02U)
#define INA219_REG_POWER            (0x03U)
#define INA219_REG_CURRENT          (0x04U)
#define INA219_REG_CALIBRATION      (0x05U)

/*===========================================================================*/
/* CONFIG REGISTER BIT DEFINITIONS                                           */
/*===========================================================================*/
/* Reset bit */
#define INA219_CONFIG_RESET             (0x8000U)

/* Bus voltage range */
#define INA219_CONFIG_BRNG_16V          (0x0000U)
#define INA219_CONFIG_BRNG_32V          (0x2000U)
#define INA219_CONFIG_BRNG_MASK         (0x2000U)

/* PGA (Shunt voltage only) */
#define INA219_CONFIG_PGA_40MV          (0x0000U)
#define INA219_CONFIG_PGA_80MV          (0x0800U)
#define INA219_CONFIG_PGA_160MV         (0x1000U)
#define INA219_CONFIG_PGA_320MV         (0x1800U)
#define INA219_CONFIG_PGA_MASK          (0x1800U)

/* Bus ADC resolution/averaging */
#define INA219_CONFIG_BADC_9BIT         (0x0000U)
#define INA219_CONFIG_BADC_10BIT        (0x0080U)
#define INA219_CONFIG_BADC_11BIT        (0x0100U)
#define INA219_CONFIG_BADC_12BIT        (0x0180U)
#define INA219_CONFIG_BADC_12BIT_2S     (0x0480U)
#define INA219_CONFIG_BADC_12BIT_4S     (0x0500U)
#define INA219_CONFIG_BADC_12BIT_8S     (0x0580U)
#define INA219_CONFIG_BADC_12BIT_16S    (0x0600U)
#define INA219_CONFIG_BADC_12BIT_32S    (0x0680U)
#define INA219_CONFIG_BADC_12BIT_64S    (0x0700U)
#define INA219_CONFIG_BADC_12BIT_128S   (0x0780U)
#define INA219_CONFIG_BADC_MASK         (0x0780U)

/* Shunt ADC resolution/averaging */
#define INA219_CONFIG_SADC_9BIT         (0x0000U)
#define INA219_CONFIG_SADC_10BIT        (0x0008U)
#define INA219_CONFIG_SADC_11BIT        (0x0010U)
#define INA219_CONFIG_SADC_12BIT        (0x0018U)
#define INA219_CONFIG_SADC_12BIT_2S     (0x0048U)
#define INA219_CONFIG_SADC_12BIT_4S     (0x0050U)
#define INA219_CONFIG_SADC_12BIT_8S     (0x0058U)
#define INA219_CONFIG_SADC_12BIT_16S    (0x0060U)
#define INA219_CONFIG_SADC_12BIT_32S    (0x0068U)
#define INA219_CONFIG_SADC_12BIT_64S    (0x0070U)
#define INA219_CONFIG_SADC_12BIT_128S   (0x0078U)
#define INA219_CONFIG_SADC_MASK         (0x0078U)

/* Operating mode */
#define INA219_CONFIG_MODE_POWERDOWN            (0x0000U)
#define INA219_CONFIG_MODE_SHUNT_TRIGGERED      (0x0001U)
#define INA219_CONFIG_MODE_BUS_TRIGGERED        (0x0002U)
#define INA219_CONFIG_MODE_SHUNT_BUS_TRIGGERED  (0x0003U)
#define INA219_CONFIG_MODE_ADC_OFF              (0x0004U)
#define INA219_CONFIG_MODE_SHUNT_CONTINUOUS     (0x0005U)
#define INA219_CONFIG_MODE_BUS_CONTINUOUS       (0x0006U)
#define INA219_CONFIG_MODE_SHUNT_BUS_CONTINUOUS (0x0007U)
#define INA219_CONFIG_MODE_MASK                 (0x0007U)

/* Bus voltage register bit definitions */
#define INA219_BUS_VOLTAGE_OVF          (0x0001U)
#define INA219_BUS_VOLTAGE_CNVR         (0x0002U)

/*===========================================================================*/
/* DEFAULT VALUES                                                            */
/*===========================================================================*/
#define INA219_DEFAULT_SHUNT_RESISTOR_MOHM  (100U)  /* 0.1 ohm = 100 milliohm */
#define INA219_DEFAULT_MAX_CURRENT_MA       (3200U) /* 3.2A */
#define INA219_DEFAULT_CONFIG               (INA219_CONFIG_BRNG_32V | INA219_CONFIG_PGA_320MV | INA219_CONFIG_BADC_12BIT | INA219_CONFIG_SADC_12BIT | INA219_CONFIG_MODE_SHUNT_BUS_CONTINUOUS)

/*===========================================================================*/
/* TYPE DEFINITIONS                                                          */
/*===========================================================================*/

/**
 * @brief Error codes
 */
typedef enum {
    INA219_OK = 0,
    INA219_ERROR_INVALID_PARAM,
    INA219_ERROR_I2C_WRITE,
    INA219_ERROR_I2C_READ,
    INA219_ERROR_TIMEOUT,
    INA219_ERROR_NOT_INITIALIZED,
    INA219_ERROR_CALIBRATION
} ina219_error_t;

/**
 * @brief Bus voltage range
 */
typedef enum {
    INA219_BRNG_16V = 0,
    INA219_BRNG_32V = 1
} ina219_bus_voltage_range_t;

/**
 * @brief PGA gain and range
 */
typedef enum {
    INA219_PGA_40MV  = 0,
    INA219_PGA_80MV  = 1,
    INA219_PGA_160MV = 2,
    INA219_PGA_320MV = 3,
} ina219_pga_gain_t;

/**
 * @brief ADC resolution/averaging
 */
typedef enum {
    INA219_ADC_9BIT       = 0x0,
    INA219_ADC_10BIT      = 0x1,
    INA219_ADC_11BIT      = 0x2,
    INA219_ADC_12BIT      = 0x3,
    INA219_ADC_12BIT_2S   = 0x9,
    INA219_ADC_12BIT_4S   = 0xA,    
    INA219_ADC_12BIT_8S   = 0xB,
    INA219_ADC_12BIT_16S  = 0xC,
    INA219_ADC_12BIT_32S  = 0xD,
    INA219_ADC_12BIT_64S  = 0xE,
    INA219_ADC_12BIT_128S = 0xF,
} ina219_adc_mode_t;

/**
 * @brief Platform I2C write function pointer
 */
typedef ina219_error_t (*ina219_i2c_write_fn_t)(uint8_t device_addr, uint8_t reg_addr, const uint8_t *data, uint16_t len);

/**
 * @brief Platform I2C read function pointer
 */
typedef ina219_error_t (*ina219_i2c_read_fn_t)(uint8_t device_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);

/**
 * @brief Platform delay function pointer (milliseconds)
 */
typedef void (*ina219_delay_ms_fn_t)(uint32_t ms);

/**
 * @brief Platform interface structure
 */
typedef struct {
    ina219_i2c_write_fn_t i2c_write;
    ina219_i2c_read_fn_t i2c_read;
    ina219_delay_ms_fn_t delay_ms;
} ina219_platform_t;

/**
 * @brief Operating mode
 */
typedef enum {
    INA219_MODE_POWERDOWN = 0,
    INA219_MODE_SHUNT_TRIGGERED = 1,
    INA219_MODE_BUS_TRIGGERED = 2,
    INA219_MODE_SHUNT_BUS_TRIGGERED = 3,
    INA219_MODE_ADC_OFF = 4,
    INA219_MODE_SHUNT_CONTINUOUS = 5,
    INA219_MODE_BUS_CONTINUOUS = 6,
    INA219_MODE_SHUNT_BUS_CONTINUOUS = 7
} ina219_mode_t;

/**
 * @brief Configuration structure
 */
typedef struct {
    ina219_bus_voltage_range_t bus_voltage_range;
    ina219_pga_gain_t pga_gain;
    ina219_adc_mode_t bus_adc_mode;
    ina219_adc_mode_t shunt_adc_mode;
    ina219_mode_t mode;
} ina219_config_t;

/**
 * @brief Calibration structure
 */
typedef struct {
    uint32_t shunt_resistor_mohm;
    uint32_t max_current_ma;
    uint16_t calibration_value;
    uint32_t current_lsb_ua;
    uint32_t power_lsb_uw;
} ina219_calibration_t;

/**
 * @brief Device handle structure
 */
typedef struct {
    uint8_t i2c_address;
    ina219_platform_t platform;
    ina219_config_t config;
    ina219_calibration_t calibration;
    bool is_initialized;
} ina219_handle_t;

/*===========================================================================*/
/* FUNCTION PROTOTYPES - Initialization & Configuration                      */
/*===========================================================================*/

/**
 * @brief Initialize INA219 device handle
 */
ina219_error_t ina219_init(ina219_handle_t *handle, uint8_t i2c_address, const ina219_platform_t *platform);

/**
 * @brief De-initialize INA219 device
 */
ina219_error_t ina219_deinit(ina219_handle_t *handle);

/**
 * @brief Reset INA219 device
 */
ina219_error_t ina219_reset(ina219_handle_t *handle);

/**
 * @brief Configure INA219 device
 */
ina219_error_t ina219_configure(ina219_handle_t *handle, const ina219_config_t *config);

/**
 * @brief Get current configuration
 */
ina219_error_t ina219_get_config(ina219_handle_t *handle, ina219_config_t *config);

/*===========================================================================*/
/* FUNCTION PROTOTYPES - Calibration                                         */
/*===========================================================================*/

/**
 * @brief Calibrate INA219 device
 */
ina219_error_t ina219_calibrate(ina219_handle_t *handle, uint32_t shunt_resistor_mohm, uint32_t max_current_ma);

/**
 * @brief Get current calibration parameters
 */
ina219_error_t ina219_get_calibration(const ina219_handle_t *handle, ina219_calibration_t *calibration);

/*===========================================================================*/
/* FUNCTION PROTOTYPES - Measurements                                        */
/*===========================================================================*/

/**
 * @brief Read shunt voltage in microvolts
 */
ina219_error_t ina219_read_shunt_voltage(ina219_handle_t *handle, int32_t *voltage_uv);

/**
 * @brief Read bus voltage in millivolts
 */
ina219_error_t ina219_read_bus_voltage(ina219_handle_t *handle, uint32_t *voltage_mv);

/**
 * @brief Read current in milliamps
 */
ina219_error_t ina219_read_current(ina219_handle_t *handle, int32_t *current_ma);

/**
 * @brief Read power in milliwatts
 */
ina219_error_t ina219_read_power(ina219_handle_t *handle, uint32_t *power_mw);

/**
 * @brief Read all measurements at once
 */
ina219_error_t ina219_read_all(ina219_handle_t *handle,
                               int32_t *voltage_uv,
                               uint32_t *voltage_mv,
                               int32_t *current_ma,
                               uint32_t *power_mw);

/*===========================================================================*/
/* FUNCTION PROTOTYPES - Status & Flags                                      */
/*===========================================================================*/

/**
 * @brief Check if conversion is ready
 */
ina219_error_t ina219_is_conversion_ready(ina219_handle_t *handle, bool *ready);

/**
 * @brief Check if math overflow occured
 */
ina219_error_t ina219_is_overflow(ina219_handle_t *handle, bool *overflow);

/*===========================================================================*/
/* FUNCTION PROTOTYPES - Register Access                                     */
/*===========================================================================*/

/**
 * @brief Write to configuration register
 */
ina219_error_t ina219_write_config_register(ina219_handle_t *handle, uint16_t config);

/**
 * @brief Read configuration register
 */
ina219_error_t ina219_read_config_register(ina219_handle_t *handle, uint16_t *config);

/**
 * @brief Write to calibration register
 */
ina219_error_t ina219_write_calibration_register(ina219_handle_t *handle, uint16_t calibration);

/**
 * @brief Read calibration register
 */
ina219_error_t ina219_read_calibration_register(ina219_handle_t *handle, uint16_t *calibration);

/**
 * @brief Write raw register value (16-bit)
 */
ina219_error_t ina219_write_register(ina219_handle_t *handle, uint8_t reg_addr, uint16_t value);

/**
 * @brief Read raw register value (16-bit)
 */
ina219_error_t ina219_read_register(ina219_handle_t *handle, uint8_t reg_addr, uint16_t *value);

/*===========================================================================*/
/* FUNCTION PROTOTYPES - Operating Mode Control                              */
/*===========================================================================*/

/**
 * @brief Set operating mode
 */
ina219_error_t ina219_set_mode(ina219_handle_t *handle, ina219_mode_t mode);

/**
 * @brief Get current operating mode
 */
ina219_error_t ina219_get_mode(ina219_handle_t *handle, ina219_mode_t *mode);

/**
 * @brief Enter power-down mode
 */
ina219_error_t ina219_powerdown(ina219_handle_t *handle);

/**
 * @brief Trigger a single conversion
 */
ina219_error_t ina219_trigger_conversion(ina219_handle_t *handle);

#ifdef __cplusplus
}
#endif

#endif /* INA219_H */