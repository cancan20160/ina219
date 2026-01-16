#include "ina219.h"
#include <stddef.h>

/*===========================================================================*/
/* PRIVATE CONSTANTS                                                         */
/*===========================================================================*/
#define INA219_SHUNT_VOLTAGE_LSB_UV (10U) /* 10 µV per bit */
#define INA219_BUS_VOLTAGE_LSB_MV (4U)    /* 4 mV per bit */
#define INA219_CALIBRATION_FACTOR (0.04096f)
#define INA219_MAX_CALIBRATION_VALUE (0xFFFEU)

/*===========================================================================*/
/* PRIVATE FUNCTION PROTOTYPES                                               */
/*===========================================================================*/
static ina219_error_t ina219_validate_handle(const ina219_handle_t *handle);
static ina219_error_t ina219_write_register_internal(ina219_handle_t *handle, uint8_t reg_addr, uint16_t value);
static ina219_error_t ina219_read_register_internal(ina219_handle_t *handle, uint8_t reg_addr, uint16_t *value);
static uint16_t ina219_config_to_register(const ina219_config_t *config);
static void ina219_register_to_config(uint16_t reg_value, ina219_config_t *config);
static ina219_error_t ina219_calculate_calibration(ina219_handle_t *handle);

/*===========================================================================*/
/* PRIVATE FUNCTIONS - Validation & Internal Operations                      */
/*===========================================================================*/

/**
 * @brief Validate device handle
 */
static ina219_error_t ina219_validate_handle(const ina219_handle_t *handle)
{
    ina219_error_t result = INA219_ERROR_INVALID_PARAM;

    if (handle != NULL)
    {
        if ((handle->platform.i2c_write != NULL) && (handle->platform.i2c_read != NULL) && (handle->platform.delay_ms != NULL))
        {
            if (handle->is_initialized)
            {
                result = INA219_OK;
            }
            else
            {
                result = INA219_ERROR_NOT_INITIALIZED;
            }
        }
    }

    return result;
}

/**
 * @brief Internal register write function
 */
static ina219_error_t ina219_write_register_internal(ina219_handle_t *handle, uint8_t reg_addr, uint16_t value)
{
    ina219_error_t result;
    uint8_t data[2];

    data[0] = (uint8_t)((value >> 8) & 0xFFU);
    data[1] = (uint8_t)(value & 0xFFU);

    result = handle->platform.i2c_write(handle->i2c_address, reg_addr, data, 2U);

    return INA219_OK;
}

/**
 * @brief Internal register read function
 */
static ina219_error_t ina219_read_register_internal(ina219_handle_t *handle, uint8_t reg_addr, uint16_t *value)
{
    ina219_error_t result;
    uint8_t data[2];

    result = handle->platform.i2c_read(handle->i2c_address, reg_addr, data, 2U);

    if (result == INA219_OK)
    {
        *value = ((uint16_t)data[0] << 8) | (uint16_t)data[1];
    }

    return INA219_OK;
}

/**
 * @brief Convert configuration structure to register value
 */
static uint16_t ina219_config_to_register(const ina219_config_t *config)
{
    uint16_t reg_value = 0U;

    /* Bus voltage range */
    if (config->bus_voltage_range == INA219_BRNG_32V)
    {
        reg_value |= INA219_CONFIG_BRNG_32V;
    }

    /* PGA gain */
    switch (config->pga_gain)
    {
    case INA219_PGA_40MV:
        reg_value |= INA219_CONFIG_PGA_40MV;
        break;
    case INA219_PGA_80MV:
        reg_value |= INA219_CONFIG_PGA_80MV;
        break;
    case INA219_PGA_160MV:
        reg_value |= INA219_CONFIG_PGA_160MV;
        break;
    case INA219_PGA_320MV:
        reg_value |= INA219_CONFIG_PGA_320MV;
        break;
    default:
        reg_value |= INA219_CONFIG_PGA_320MV;
        break;
    }

    /* Bus ADC mode */
    switch (config->bus_adc_mode)
    {
    case INA219_ADC_9BIT:
        reg_value |= INA219_CONFIG_BADC_9BIT;
        break;
    case INA219_ADC_10BIT:
        reg_value |= INA219_CONFIG_BADC_10BIT;
        break;
    case INA219_ADC_11BIT:
        reg_value |= INA219_CONFIG_BADC_11BIT;
        break;
    case INA219_ADC_12BIT:
        reg_value |= INA219_CONFIG_BADC_12BIT;
        break;
    case INA219_ADC_12BIT_2S:
        reg_value |= INA219_CONFIG_BADC_12BIT_2S;
        break;
    case INA219_ADC_12BIT_4S:
        reg_value |= INA219_CONFIG_BADC_12BIT_4S;
        break;
    case INA219_ADC_12BIT_8S:
        reg_value |= INA219_CONFIG_BADC_12BIT_8S;
        break;
    case INA219_ADC_12BIT_16S:
        reg_value |= INA219_CONFIG_BADC_12BIT_16S;
        break;
    case INA219_ADC_12BIT_32S:
        reg_value |= INA219_CONFIG_BADC_12BIT_32S;
        break;
    case INA219_ADC_12BIT_64S:
        reg_value |= INA219_CONFIG_BADC_12BIT_64S;
        break;
    case INA219_ADC_12BIT_128S:
        reg_value |= INA219_CONFIG_BADC_12BIT_128S;
        break;
    default:
        reg_value |= INA219_CONFIG_BADC_12BIT;
        break;
    }

    /* Shunt ADC mode */
    switch (config->shunt_adc_mode)
    {
    case INA219_ADC_9BIT:
        reg_value |= INA219_CONFIG_SADC_9BIT;
        break;
    case INA219_ADC_10BIT:
        reg_value |= INA219_CONFIG_SADC_10BIT;
        break;
    case INA219_ADC_11BIT:
        reg_value |= INA219_CONFIG_SADC_11BIT;
        break;
    case INA219_ADC_12BIT:
        reg_value |= INA219_CONFIG_SADC_12BIT;
        break;
    case INA219_ADC_12BIT_2S:
        reg_value |= INA219_CONFIG_SADC_12BIT_2S;
        break;
    case INA219_ADC_12BIT_4S:
        reg_value |= INA219_CONFIG_SADC_12BIT_4S;
        break;
    case INA219_ADC_12BIT_8S:
        reg_value |= INA219_CONFIG_SADC_12BIT_8S;
        break;
    case INA219_ADC_12BIT_16S:
        reg_value |= INA219_CONFIG_SADC_12BIT_16S;
        break;
    case INA219_ADC_12BIT_32S:
        reg_value |= INA219_CONFIG_SADC_12BIT_32S;
        break;
    case INA219_ADC_12BIT_64S:
        reg_value |= INA219_CONFIG_SADC_12BIT_64S;
        break;
    case INA219_ADC_12BIT_128S:
        reg_value |= INA219_CONFIG_SADC_12BIT_128S;
        break;
    default:
        reg_value |= INA219_CONFIG_SADC_12BIT;
        break;
    }

    /* Operating mode */
    switch (config->mode)
    {
    case INA219_MODE_POWERDOWN:
        reg_value |= INA219_CONFIG_MODE_POWERDOWN;
        break;
    case INA219_MODE_SHUNT_TRIGGERED:
        reg_value |= INA219_CONFIG_MODE_SHUNT_TRIGGERED;
        break;
    case INA219_MODE_BUS_TRIGGERED:
        reg_value |= INA219_CONFIG_MODE_BUS_TRIGGERED;
        break;
    case INA219_MODE_SHUNT_BUS_TRIGGERED:
        reg_value |= INA219_CONFIG_MODE_SHUNT_BUS_TRIGGERED;
        break;
    case INA219_MODE_ADC_OFF:
        reg_value |= INA219_CONFIG_MODE_ADC_OFF;
        break;
    case INA219_MODE_SHUNT_CONTINUOUS:
        reg_value |= INA219_CONFIG_MODE_SHUNT_CONTINUOUS;
        break;
    case INA219_MODE_BUS_CONTINUOUS:
        reg_value |= INA219_CONFIG_MODE_BUS_CONTINUOUS;
        break;
    case INA219_MODE_SHUNT_BUS_CONTINUOUS:
        reg_value |= INA219_CONFIG_MODE_SHUNT_BUS_CONTINUOUS;
        break;
    default:
        reg_value |= INA219_CONFIG_MODE_SHUNT_BUS_CONTINUOUS;
        break;
    }

    return reg_value;
}

/**
 * @brief Convert register value to configuration structure
 */
static void ina219_register_to_config(uint16_t reg_value, ina219_config_t *config)
{
    /* Bus voltage range */
    if ((reg_value & INA219_CONFIG_BRNG_MASK) == INA219_CONFIG_BRNG_32V)
    {
        config->bus_voltage_range = INA219_BRNG_32V;
    }
    else
    {
        config->bus_voltage_range = INA219_BRNG_16V;
    }

    /* PGA gain */
    switch (reg_value & INA219_CONFIG_PGA_MASK)
    {
    case INA219_CONFIG_PGA_40MV:
        config->pga_gain = INA219_PGA_40MV;
        break;
    case INA219_CONFIG_PGA_80MV:
        config->pga_gain = INA219_PGA_80MV;
        break;
    case INA219_CONFIG_PGA_160MV:
        config->pga_gain = INA219_PGA_160MV;
        break;
    case INA219_CONFIG_PGA_320MV:
        config->pga_gain = INA219_PGA_320MV;
        break;
    default:
        config->pga_gain = INA219_PGA_320MV;
        break;
    }

    /* Bus ADC mode */
    switch (reg_value & INA219_CONFIG_BADC_MASK)
    {
    case INA219_CONFIG_BADC_9BIT:
        config->bus_adc_mode = INA219_ADC_9BIT;
        break;
    case INA219_CONFIG_BADC_10BIT:
        config->bus_adc_mode = INA219_ADC_10BIT;
        break;
    case INA219_CONFIG_BADC_11BIT:
        config->bus_adc_mode = INA219_ADC_11BIT;
        break;
    case INA219_CONFIG_BADC_12BIT:
        config->bus_adc_mode = INA219_ADC_12BIT;
        break;
    case INA219_CONFIG_BADC_12BIT_2S:
        config->bus_adc_mode = INA219_ADC_12BIT_2S;
        break;
    case INA219_CONFIG_BADC_12BIT_4S:
        config->bus_adc_mode = INA219_ADC_12BIT_4S;
        break;
    case INA219_CONFIG_BADC_12BIT_8S:
        config->bus_adc_mode = INA219_ADC_12BIT_8S;
        break;
    case INA219_CONFIG_BADC_12BIT_16S:
        config->bus_adc_mode = INA219_ADC_12BIT_16S;
        break;
    case INA219_CONFIG_BADC_12BIT_32S:
        config->bus_adc_mode = INA219_ADC_12BIT_32S;
        break;
    case INA219_CONFIG_BADC_12BIT_64S:
        config->bus_adc_mode = INA219_ADC_12BIT_64S;
        break;
    case INA219_CONFIG_BADC_12BIT_128S:
        config->bus_adc_mode = INA219_ADC_12BIT_128S;
        break;
    default:
        config->bus_adc_mode = INA219_ADC_12BIT;
        break;
    }

    /* Shunt ADC mode */
    switch (reg_value & INA219_CONFIG_SADC_MASK)
    {
    case INA219_CONFIG_SADC_9BIT:
        config->shunt_adc_mode = INA219_ADC_9BIT;
        break;
    case INA219_CONFIG_SADC_10BIT:
        config->shunt_adc_mode = INA219_ADC_10BIT;
        break;
    case INA219_CONFIG_SADC_11BIT:
        config->shunt_adc_mode = INA219_ADC_11BIT;
        break;
    case INA219_CONFIG_SADC_12BIT:
        config->shunt_adc_mode = INA219_ADC_12BIT;
        break;
    case INA219_CONFIG_SADC_12BIT_2S:
        config->shunt_adc_mode = INA219_ADC_12BIT_2S;
        break;
    case INA219_CONFIG_SADC_12BIT_4S:
        config->shunt_adc_mode = INA219_ADC_12BIT_4S;
        break;
    case INA219_CONFIG_SADC_12BIT_8S:
        config->shunt_adc_mode = INA219_ADC_12BIT_8S;
        break;
    case INA219_CONFIG_SADC_12BIT_16S:
        config->shunt_adc_mode = INA219_ADC_12BIT_16S;
        break;
    case INA219_CONFIG_SADC_12BIT_32S:
        config->shunt_adc_mode = INA219_ADC_12BIT_32S;
        break;
    case INA219_CONFIG_SADC_12BIT_64S:
        config->shunt_adc_mode = INA219_ADC_12BIT_64S;
        break;
    case INA219_CONFIG_SADC_12BIT_128S:
        config->shunt_adc_mode = INA219_ADC_12BIT_128S;
        break;
    default:
        config->shunt_adc_mode = INA219_ADC_12BIT;
        break;
    }

    /* Operating mode */
    switch (reg_value & INA219_CONFIG_MODE_MASK)
    {
    case INA219_CONFIG_MODE_POWERDOWN:
        config->mode = INA219_MODE_POWERDOWN;
        break;
    case INA219_CONFIG_MODE_SHUNT_TRIGGERED:
        config->mode = INA219_MODE_SHUNT_TRIGGERED;
        break;
    case INA219_CONFIG_MODE_BUS_TRIGGERED:
        config->mode = INA219_MODE_BUS_TRIGGERED;
        break;
    case INA219_CONFIG_MODE_SHUNT_BUS_TRIGGERED:
        config->mode = INA219_MODE_SHUNT_BUS_TRIGGERED;
        break;
    case INA219_CONFIG_MODE_ADC_OFF:
        config->mode = INA219_MODE_ADC_OFF;
        break;
    case INA219_CONFIG_MODE_SHUNT_CONTINUOUS:
        config->mode = INA219_MODE_SHUNT_CONTINUOUS;
        break;
    case INA219_CONFIG_MODE_BUS_CONTINUOUS:
        config->mode = INA219_MODE_BUS_CONTINUOUS;
        break;
    case INA219_CONFIG_MODE_SHUNT_BUS_CONTINUOUS:
        config->mode = INA219_MODE_SHUNT_BUS_CONTINUOUS;
        break;
    default:
        config->mode = INA219_MODE_SHUNT_BUS_CONTINUOUS;
        break;
    }
}

/**
 * @brief Calculate calibration register value and LSB values
 */
static ina219_error_t ina219_calculate_calibration(ina219_handle_t *handle)
{
    /* TODO: Implement calibration calculation */
    return INA219_OK;
}

/*===========================================================================*/
/* PUBLIC FUNCTIONS - Initialization & Configuration                         */
/*===========================================================================*/

ina219_error_t ina219_init(ina219_handle_t *handle, uint8_t i2c_address, const ina219_platform_t *platform)
{
    ina219_error_t result = INA219_ERROR_INVALID_PARAM;

    if ((handle != NULL) && (platform != NULL))
    {
        if ((platform->i2c_write != NULL) && (platform->i2c_read != NULL) && (platform->delay_ms != NULL))
        {
            /* Initialize handle structure */
            handle->i2c_address = i2c_address;
            handle->platform = *platform;
            handle->is_initialized = false;

            /* Set default configuration */
            handle->config.bus_voltage_range = INA219_BRNG_32V;
            handle->config.pga_gain = INA219_PGA_320MV;
            handle->config.bus_adc_mode = INA219_ADC_12BIT;
            handle->config.shunt_adc_mode = INA219_ADC_12BIT;
            handle->config.mode = INA219_MODE_SHUNT_BUS_CONTINUOUS;

            /* Set default calibration parameters */
            handle->calibration.shunt_resistor_mohm = INA219_DEFAULT_SHUNT_RESISTOR_MOHM;
            handle->calibration.max_current_ma = INA219_DEFAULT_MAX_CURRENT_MA;
            handle->calibration.calibration_value = 0U;
            handle->calibration.current_lsb_ua = 0U;
            handle->calibration.power_lsb_uw = 0U;

            /* Calculate default calibration */
            result = ina219_calculate_calibration(handle);

            if (result == INA219_OK)
            {
                /* Write calibration to device */
                result = ina219_write_register_internal(handle, INA219_REG_CALIBRATION, handle->calibration.calibration_value);

                if (result = INA219_OK)
                {
                    /* Write default configuration */
                    uint16_t config_reg = ina219_config_to_register(&handle->config);
                    result = ina219_write_register_internal(handle, INA219_REG_CONFIG, config_reg);

                    if (result = INA219_OK)
                    {
                        handle->is_initialized = true;
                    }
                }
            }
        }
    }

    return result;
}

ina219_error_t ina219_deinit(ina219_handle_t *handle)
{
}

ina219_error_t ina219_reset(ina219_handle_t *handle)
{
    ina219_error_t result;

    result = ina219_validate_handle(handle);

    if (result == INA219_OK)
    {
        /* Write reset bit to configuration register */
        result = ina219_write_register_internal(handle, INA219_REG_CONFIG, INA219_CONFIG_RESET);

        if (result == INA219_OK)
        {
            /* Wait for reset to complete (datasheet specifies 40us)*/
            handle->platform.delay_ms(1U);

            /* Reconfigure device with stored settings*/
            uint16_t config_reg = ina219_config_to_register(&handle->config);
            result = ina219_write_register_internal(handle, INA219_REG_CONFIG, config_reg);

            if (result = INA219_OK)
            {
                /* Restore calibration*/
                result = ina219_write_register_internal(handle, INA219_REG_CALIBRATION, handle->calibration.calibration_value);
            }
        }
    }

    return result;
}

ina219_error_t ina219_configure(ina219_handle_t *handle, const ina219_config_t *config)
{
    ina219_error_t result = INA219_ERROR_INVALID_PARAM;

    if (config != NULL)
    {
        result = ina219_validate_handle(handle);

        if (result == INA219_OK)
        {
            uint16_t config_reg = ina219_config_to_register(config);
            result = ina219_write_register_internal(handle, INA219_REG_CONFIG, config_reg);

            if (result == INA219_OK)
            {
                /* Update stored configuration */
                handle->config = *config;
            }
        }
    }

    return result;
}

ina219_error_t ina219_get_config(ina219_handle_t *handle, ina219_config_t *config)
{
    ina219_error_t result = INA219_ERROR_INVALID_PARAM;

    if (config != NULL)
    {
        result = ina219_validate_handle(handle);

        if (result == INA219_OK)
        {
            uint16_t config_reg;
            result = ina219_read_register_internal(handle, INA219_REG_CONFIG, &config_reg);

            if (result == INA219_OK)
            {
                ina219_register_to_config(config_reg, config);
                handle->config = *config;
            }
        }
    }

    return result;
}

/*===========================================================================*/
/* PUBLIC FUNCTIONS - Calibration                                            */
/*===========================================================================*/

ina219_error_t ina219_calibrate(ina219_handle_t *handle, uint32_t shunt_resistor_mohm, uint32_t max_current_ma)
{
    /* TODO: Implement calibration */
    return INA219_OK;
}

ina219_error_t ina219_get_calibration(const ina219_handle_t *handle, ina219_calibration_t *calibration)
{
    /* TODO: Implement get calibration */
    return INA219_OK;
}

/*===========================================================================*/
/* PUBLIC FUNCTIONS - Measurements                                          */
/*===========================================================================*/

ina219_error_t ina219_read_shunt_voltage(ina219_handle_t *handle, int32_t *voltage_uv)
{
    ina219_error_t result = INA219_ERROR_INVALID_PARAM;

    if (voltage_uv != NULL)
    {
        result = ina219_validate_handle(handle);

        if (result == INA219_OK)
        {
            uint16_t raw_value;
            result = ina219_read_register_internal(handle, INA219_REG_SHUNT_VOLTAGE, &raw_value);

            if (result == INA219_OK)
            {
                /* Convert to microvolts */
                *voltage_uv = (int32_t)((int16_t)raw_value) * (int32_t)INA219_SHUNT_VOLTAGE_LSB_UV;
            }
        }
    }

    return result;
}

ina219_error_t ina219_read_bus_voltage(ina219_handle_t *handle, uint32_t *voltage_mv)
{
    ina219_error_t result = INA219_ERROR_INVALID_PARAM;

    if (voltage_mv != NULL)
    {
        result = ina219_validate_handle(handle);

        if (result == INA219_OK)
        {
            uint16_t raw_value;
            result = ina219_read_register_internal(handle, INA219_REG_BUS_VOLTAGE, &raw_value);

            if (result == INA219_OK)
            {
                /* Bits 15-3: Bus voltage value (13 bits) */
                *voltage_mv = ((uint32_t)raw_value >> 3) * INA219_BUS_VOLTAGE_LSB_MV;
            }
        }
    }

    return result;
}

ina219_error_t ina219_read_current(ina219_handle_t *handle, int32_t *current_ma)
{
    ina219_error_t result = INA219_ERROR_INVALID_PARAM;

    if (current_ma != NULL)
    {
        result = ina219_validate_handle(handle);

        if (result == INA219_OK)
        {
            uint16_t raw_value;
            result = ina219_read_register_internal(handle, INA219_REG_CURRENT, &raw_value);

            if (result == INA219_OK)
            {
                /* Current register is a signed 16-bit value, convert to milliamps by diving by 1000*/
                int32_t current_ua = (int32_t)(int16_t)raw_value * (int32_t)handle->calibration.current_lsb_ua;
                *current_ma = current_ua / 1000;
            }
        }
    }

    return result;
}

ina219_error_t ina219_read_power(ina219_handle_t *handle, uint32_t *power_mw)
{
    ina219_error_t result = INA219_ERROR_INVALID_PARAM;

    if (power_mw != NULL)
    {
        result = ina219_validate_handle(handle);

        if (result == INA219_OK)
        {
            uint16_t raw_value;
            result = ina219_read_register_internal(handle, INA219_REG_POWER, &raw_value);

            if (result == INA219_OK)
            {
                /* Power register is an unsigned 16-bit value, convert to milliwatts by dividing by 1000 */
                uint32_t power_uw = (uint32_t)raw_value * handle->calibration.power_lsb_uw;
                *power_mw = power_uw / 1000U;
            }
        }
    }

    return result;
}

ina219_error_t ina219_read_all(ina219_handle_t *handle, int32_t *shunt_voltage_uv, uint32_t *bus_voltage_mv, int32_t *current_ma, uint32_t *power_mw)
{
    ina219_error_t result = INA219_ERROR_INVALID_PARAM;

    if ((shunt_voltage_uv != NULL) && (bus_voltage_mv != NULL) && (current_ma != NULL) && (power_mw != NULL))
    {
        result = ina219_validate_handle(handle);

        if (result == INA219_OK)
        {
            result = ina219_read_shunt_voltage(handle, shunt_voltage_uv);

            if (result == INA219_OK)
            {
                result = ina219_read_bus_voltage(handle, bus_voltage_mv);

                if (result == INA219_OK)
                {
                    result == ina219_read_current(handle, current_ma);

                    if (result == INA219_OK)
                    {
                        result == ina219_read_power(handle, power_mw);
                    }
                }
            }
        }
    }

    return result;
}

/*===========================================================================*/
/* PUBLIC FUNCTIONS - Status & Flags                                        */
/*===========================================================================*/

ina219_error_t ina219_is_conversion_ready(ina219_handle_t *handle, bool *ready)
{
    ina219_error_t result = INA219_ERROR_INVALID_PARAM;

    if (ready != NULL)
    {
        result = ina219_validate_handle(handle);

        if (result == INA219_OK)
        {
            uint16_t bus_voltage_reg;
            result = ina219_read_register_internal(handle, INA219_REG_BUS_VOLTAGE, &bus_voltage_reg);

            if (result == INA219_OK)
            {
                /* Check CNVR bit (bit-1) */
                *ready = ((bus_voltage_reg & INA219_BUS_VOLTAGE_CNVR) != 0U);
            }
        }
    }

    return result;
}

ina219_error_t ina219_is_overflow(ina219_handle_t *handle, bool *overflow)
{
    ina219_error_t result = INA219_ERROR_INVALID_PARAM;

    if (overflow != NULL)
    {
        result = ina219_validate_handle(handle);

        if (result == INA219_OK)
        {
            uint16_t bus_voltage_reg;
            result = ina219_read_register_internal(handle, INA219_REG_BUS_VOLTAGE, &bus_voltage_reg);

            if (result == INA219_OK)
            {
                /* Check OVF bit (bit-0) */
                *overflow = ((bus_voltage_reg & INA219_BUS_VOLTAGE_OVF) != 0U);
            }
        }
    }

    return result;
}

/*===========================================================================*/
/* PUBLIC FUNCTIONS - Register Access                                       */
/*===========================================================================*/

ina219_error_t ina219_write_config_register(ina219_handle_t *handle,
                                            uint16_t config)
{
    /* TODO: Implement config register write */
    return INA219_OK;
}

ina219_error_t ina219_read_config_register(ina219_handle_t *handle,
                                           uint16_t *config)
{
    /* TODO: Implement config register read */
    return INA219_OK;
}

ina219_error_t ina219_write_calibration_register(ina219_handle_t *handle,
                                                 uint16_t calibration)
{
    /* TODO: Implement calibration register write */
    return INA219_OK;
}

ina219_error_t ina219_read_calibration_register(ina219_handle_t *handle,
                                                uint16_t *calibration)
{
    /* TODO: Implement calibration register read */
    return INA219_OK;
}

ina219_error_t ina219_read_register(ina219_handle_t *handle, uint8_t reg_addr, uint16_t *value)
{
    /* TODO: Implement register read */
    return INA219_OK;
}

ina219_error_t ina219_write_register(ina219_handle_t *handle, uint8_t reg_addr, uint16_t value)
{
    /* TODO: Implement register write */
    return INA219_OK;
}

/*===========================================================================*/
/* PUBLIC FUNCTIONS - Operating Mode Control                                */
/*===========================================================================*/

ina219_error_t ina219_set_mode(ina219_handle_t *handle, ina219_mode_t mode)
{
    ina219_error_t result;
    uint16_t config_reg;

    result = ina219_validate_handle(handle);

    if (result == INA219_OK)
    {
        /* Read current configuration */
        result = ina219_read_register_internal(handle, INA219_REG_CONFIG, &config_reg);

        if (result == INA219_OK)
        {
            /* Clear mode bits */
            config_reg &= ~INA219_CONFIG_MODE_MASK;

            /* Set new mode */
            switch (mode)
            {
            case INA219_MODE_POWERDOWN:
                config_reg |= INA219_CONFIG_MODE_POWERDOWN;
                break;
            case INA219_MODE_SHUNT_TRIGGERED:
                config_reg |= INA219_CONFIG_MODE_SHUNT_TRIGGERED;
                break;
            case INA219_MODE_BUS_TRIGGERED:
                config_reg |= INA219_CONFIG_MODE_BUS_TRIGGERED;
                break;
            case INA219_MODE_SHUNT_BUS_TRIGGERED:
                config_reg |= INA219_CONFIG_MODE_SHUNT_BUS_TRIGGERED;
                break;
            case INA219_MODE_ADC_OFF:
                config_reg |= INA219_CONFIG_MODE_ADC_OFF;
                break;
            case INA219_MODE_SHUNT_CONTINUOUS:
                config_reg |= INA219_CONFIG_MODE_SHUNT_CONTINUOUS;
                break;
            case INA219_MODE_BUS_CONTINUOUS:
                config_reg |= INA219_CONFIG_MODE_BUS_CONTINUOUS;
                break;
            case INA219_MODE_SHUNT_BUS_CONTINUOUS:
                config_reg |= INA219_CONFIG_MODE_SHUNT_BUS_CONTINUOUS;
                break;
            default:
                result = INA219_ERROR_INVALID_PARAM;
                break;
            }

            if (result == INA219_OK)
            {
                /* Write updated configuration */
                result = ina219_write_register_internal(handle, INA219_REG_CONFIG, config_reg);

                if (result == INA219_OK)
                {
                    /* Update stored mode */
                    handle->config.mode = mode;
                }
            }
        }
    }

    return result;
}

ina219_error_t ina219_get_mode(ina219_handle_t *handle, ina219_mode_t *mode)
{
    ina219_error_t result;

    if (mode != NULL)
    {
        result = ina219_validate_handle(handle);

        if (result == INA219_OK)
        {
            uint16_t config_reg;
            result = ina219_read_register_internal(handle, INA219_REG_CONFIG, &config_reg);

            if (result == INA219_OK)
            {
                /* Extract mode bits */
                switch (config_reg & INA219_CONFIG_MODE_MASK)
                {
                case INA219_CONFIG_MODE_POWERDOWN:
                    *mode = INA219_MODE_POWERDOWN;
                    break;
                case INA219_CONFIG_MODE_SHUNT_TRIGGERED:
                    *mode = INA219_MODE_SHUNT_TRIGGERED;
                    break;
                case INA219_CONFIG_MODE_BUS_TRIGGERED:
                    *mode = INA219_MODE_BUS_TRIGGERED;
                    break;
                case INA219_CONFIG_MODE_SHUNT_BUS_TRIGGERED:
                    *mode = INA219_MODE_SHUNT_BUS_TRIGGERED;
                    break;
                case INA219_CONFIG_MODE_ADC_OFF:
                    *mode = INA219_MODE_ADC_OFF;
                    break;
                case INA219_CONFIG_MODE_SHUNT_CONTINUOUS:
                    *mode = INA219_MODE_SHUNT_CONTINUOUS;
                    break;
                case INA219_CONFIG_MODE_BUS_CONTINUOUS:
                    *mode = INA219_MODE_BUS_CONTINUOUS;
                    break;
                case INA219_CONFIG_MODE_SHUNT_BUS_CONTINUOUS:
                    *mode = INA219_MODE_SHUNT_BUS_CONTINUOUS;
                    break;
                default:
                    *mode = INA219_MODE_POWERDOWN;
                    break;
                }

                /* Update stored mode */
                handle->config.mode = mode;
            }
        }
    }

    return result;
}

ina219_error_t ina219_powerdown(ina219_handle_t *handle)
{
    ina219_error_t result;

    result = ina219_validate_handle(handle);

    if (result == INA219_OK)
    {
        result = ina219_set_mode(handle, INA219_MODE_POWERDOWN);
    }

    return result;
}

ina219_error_t ina219_trigger_conversion(ina219_handle_t *handle)
{
    ina219_error_t result;

    result = ina219_validate_handle(handle);

    if (result = INA219_OK)
    {
        result = ina219_set_mode(handle, INA219_MODE_SHUNT_BUS_TRIGGERED);
    }

    return result;
}
