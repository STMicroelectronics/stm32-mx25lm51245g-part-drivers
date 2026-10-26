/**
  ******************************************************************************
  * @file    mx25lm51245g.c
  * @brief   This file provides the MX25LM51245G OSPI drivers.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  * MX25LM51245G action :
  *   STR Octal IO protocol (SOPI) and DTR Octal IO protocol (DOPI) bits of
  *   Configuration Register 2 :
  *     DOPI = 1 and SOPI = 0: Operates in DTR Octal IO protocol (accepts 8-8-8 commands)
  *     DOPI = 0 and SOPI = 1: Operates in STR Octal IO protocol (accepts 8-8-8 commands)
  *     DOPI = 0 and SOPI = 0: Operates in Single IO protocol (accepts 1-1-1 commands)
  *   Enter SOPI mode by configuring DOPI = 0 and SOPI = 1 in CR2-Addr0
  *   Exit SOPI mode by configuring DOPI = 0 and SOPI = 0 in CR2-Addr0
  *   Enter DOPI mode by configuring DOPI = 1 and SOPI = 0 in CR2-Addr0
  *   Exit DOPI mode by configuring DOPI = 0 and SOPI = 0 in CR2-Addr0
  *
  *   Memory commands support STR(Single Transfer Rate) &
  *   DTR(Double Transfer Rate) modes in OPI
  *
  *   Memory commands support STR(Single Transfer Rate) mode in SPI
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "mx25lm51245g.h"

/* Define the dummy cycle frequency table globally for reuse */
static const struct
{
  uint32_t freq;        /*!< Frequency in Hz */
  uint8_t dc_cmd_value; /*!< Corresponding dummy cycles xspi command value */
  uint8_t dc_reg_value; /*!< Corresponding dummy cycles register value */
} mx25lm51245g_dc_freq_lut[] =
{
  {133000000, 20, MX25LM51245G_CR2_DC_20_CYCLES}, /*!< 133 MHz: dummy cycles can be 14, 16, 18, or 20 (max chosen) */
  {104000000, 12, MX25LM51245G_CR2_DC_12_CYCLES}, /*!< 104 MHz: dummy cycles can be 10 or 12 (max chosen) */
  {84000000,  8, MX25LM51245G_CR2_DC_8_CYCLES},
  {66000000, 6, MX25LM51245G_CR2_DC_6_CYCLES},
};

/* Private Macro */
#define MX25LM51245G_DC_TABLE_SIZE   (sizeof(mx25lm51245g_dc_freq_lut) / sizeof(mx25lm51245g_dc_freq_lut[0]))

#if defined (MX25LM_CALLBACKS) && (MX25LM_CALLBACKS == 1)
#if defined (USE_HAL_XSPI_DMA) && (USE_HAL_XSPI_DMA == 1)
/* Cache-aligned bounce buffer for asynchronous DMA transfers. */
static __ALIGNED(MX25LM51245G_ALIGNMENT_SIZE) uint8_t mx25lm51245g_async_page_buffer[MX25LM51245G_PAGE_SIZE];
#endif /* USE_HAL_XSPI_DMA */
#endif /* MX25LM_CALLBACKS */

static mx25lm51245g_status_t mx25lm51245g_write_page(mx25lm51245g_obj_t *pobj, mx25lm51245g_addr_size_t addr_size,
                                                     uint32_t addr, uint8_t *p_data, uint32_t size_byte);
static mx25lm51245g_status_t mx25lm51245g_write_enable(mx25lm51245g_obj_t *pobj);
static mx25lm51245g_status_t mx25lm51245g_wip_autopolling(mx25lm51245g_obj_t *pobj);
static mx25lm51245g_status_t mx25lm51245g_wel_autopolling(mx25lm51245g_obj_t *pobj);
static mx25lm51245g_status_t mx25lm51245g_enter_dopi_mode(mx25lm51245g_obj_t *pobj);
static mx25lm51245g_status_t mx25lm51245g_enter_sopi_mode(mx25lm51245g_obj_t *pobj);
static mx25lm51245g_status_t mx25lm51245g_write_status_register(mx25lm51245g_obj_t *pobj, uint8_t value);
static mx25lm51245g_status_t mx25lm51245g_write_cfg_register(mx25lm51245g_obj_t *pobj, uint8_t value);
static mx25lm51245g_status_t mx25lm51245g_write_cfg2_register(mx25lm51245g_obj_t *pobj, uint32_t write_addr,
                                                              uint8_t value);
static mx25lm51245g_status_t mx25lm51245g_read_cfg_register(mx25lm51245g_obj_t *pobj, uint8_t *pvalue);
static mx25lm51245g_status_t mx25lm51245g_read_cfg2_register(mx25lm51245g_obj_t *pobj, uint32_t read_addr,
                                                             uint8_t *p_value);
static mx25lm51245g_status_t mx25lm51245g_send_regular_cmd(mx25lm51245g_obj_t *pobj, hal_xspi_addr_mode_t addr_mode,
                                                           hal_xspi_regular_data_mode_t data_mode, uint32_t addr,
                                                           uint32_t cmd, uint32_t dummy_cycle);
static mx25lm51245g_status_t mx25lm51245g_read_cmd(mx25lm51245g_obj_t *pobj, mx25lm51245g_addr_size_t addr_size,
                                                   uint32_t read_addr, uint32_t size_byte);
static mx25lm51245g_status_t mx25lm51245g_write_cmd(mx25lm51245g_obj_t *pobj, mx25lm51245g_addr_size_t addr_size,
                                                    uint32_t write_addr, uint32_t size_byte);

static mx25lm51245g_status_t mx25lm51245g_send_sector_erase_cmd(mx25lm51245g_obj_t *pobj, uint32_t sector_addr);
static mx25lm51245g_status_t mx25lm51245g_send_block_erase_cmd(mx25lm51245g_obj_t *pobj, uint32_t block_addr);
static mx25lm51245g_status_t mx25lm51245g_send_read_status_reg_cmd(mx25lm51245g_obj_t *pobj);
/* dummy cycle management apis */
static mx25lm51245g_status_t mx25lm51245g_update_dc_reg(mx25lm51245g_obj_t *pobj, uint8_t lut_index);
static uint32_t mx25lm51245g_get_lookup_dummy_cycles_index(mx25lm51245g_obj_t *pobj);
#if defined MX25LM_CALLBACKS && (MX25LM_CALLBACKS == 1)
static void xspi_erase_cplt_cb(hal_xspi_handle_t *hxspi);
static void xspi_ce_cplt_cb(hal_xspi_handle_t *hxspi);
#if defined (USE_HAL_XSPI_DMA) && (USE_HAL_XSPI_DMA == 1)
/* Asynchronous command functions */
static void mx25lm51245g_write_enable_it(mx25lm51245g_obj_t *pobj);
static void mx25lm51245g_wip_autopolling_it(mx25lm51245g_obj_t *pobj);
static void mx25lm51245g_wel_autopolling_it(mx25lm51245g_obj_t *pobj);
/* register callback functions for asynchronous operations */
static void mx25lm51245g_register_status_match_callback(mx25lm51245g_obj_t *pobj, hal_xspi_cb_t p_cb);
static void mx25lm51245g_register_cmd_cplt_callback(mx25lm51245g_obj_t *pobj, hal_xspi_cb_t p_cb);
/* Asynchronous transfer setup functions */
static void mx25lm51245g_async_rx_setup(mx25lm51245g_obj_t *pobj);
static void mx25lm51245g_async_tx_setup(mx25lm51245g_obj_t *pobj);
static void mx25lm51245g_async_write(mx25lm51245g_obj_t *pobj, mx25lm51245g_addr_size_t addr_size,
                                     uint32_t write_addr, uint8_t *pbuffer, uint32_t size_byte);
/* Asynchronous buffer management */
static uint8_t mx25lm51245g_read_uses_direct_dma(const mx25lm51245g_obj_t *pobj);
/* Asynchronous write callback functions */
static void mx25lm51245g_wip_check_cb(hal_xspi_handle_t *phxspi);
static void mx25lm51245g_wel_check_cb(hal_xspi_handle_t *phxspi);
static void mx25lm51245g_write_enable_cb(hal_xspi_handle_t *phxspi);
static void mx25lm51245g_clean_up_cb(hal_xspi_handle_t *phxspi);
static void xspi_wr_cplt_cb(hal_xspi_handle_t *phxspi);
/* Asynchronous read callback functions */
static void xspi_rx_cplt_cb(hal_xspi_handle_t *phxspi);
static void xspi_rd_cplt_cb(hal_xspi_handle_t *phxspi);
#endif /* USE_HAL_XSPI_DMA */
#endif /* MX25LM_CALLBACKS */

/*  Memory initialization and configuration --------------------------------------------------*/

/**
  * @brief Initializes the IO layer. This specific implementation does nothing.
  * @retval 1 (always fail)
  * @note This function must be overridden (using codegen or otherwise with a proper implementation).
  */
__WEAK int32_t mx25lm51245g_io_init(mx25lm51245g_io_t *pio)
{
  (void) pio;
  /* generated by CubeMX2 */
  return 1;
}

/**
  * @brief  Initializes MX25LM51245G.
  * @param  pobj   : Pointer to the part object structure
  * @param  dev_id : Instance ID
  * @retval error status
  */
mx25lm51245g_status_t mx25lm51245g_init(mx25lm51245g_obj_t *pobj, uint32_t dev_id)
{
  mx25lm51245g_status_t ret = MX25LM51245G_OK;

  /* Check if the instance is already initialized */
  if (pobj->is_initialized == MX25LM51245G_INIT_NONE)
  {
    pobj->io.id = dev_id;
    /* Set the default power-on config */
    pobj->access_mode   = MX25LM51245G_ACCESS_INDIRECT;
    pobj->interface     = MX25LM51245G_SPI_MODE;
    pobj->transfer_rate = MX25LM51245G_STR_TRANSFER;

    if (mx25lm51245g_io_init(&(pobj->io)) != 0)
    {
      ret = MX25LM51245G_ERROR;
    }
    else if (mx25lm51245g_reset(pobj) != MX25LM51245G_OK)
    {
      ret = MX25LM51245G_ERROR;
    }
    else if (mx25lm51245g_wip_autopolling(pobj) != MX25LM51245G_OK)
    {
      ret = MX25LM51245G_ERROR;
    }
    else
    {
#if defined (MX25LM_CALLBACKS) && (MX25LM_CALLBACKS == 1)
      pobj->async_context.async_state    = MX25LM51245G_ASYNC_OPERATION_NONE;
      pobj->async_context.dma_buffer      = mx25lm51245g_async_page_buffer;
      pobj->async_context.dma_buffer_size = MX25LM51245G_PAGE_SIZE;
#endif /* MX25LM_CALLBACKS */
      pobj->dc_lut_index = MX25LM51245G_DC_LUT_INIT_INDEX;
      pobj->is_initialized = MX25LM51245G_INIT_OK;
    }
  }

  return ret;
}

/**
  * @brief  De-Initializes the Flash memory.
  * @param  pobj : Pointer to the part object structure
  * @retval error status
  */
mx25lm51245g_status_t mx25lm51245g_deinit(mx25lm51245g_obj_t *pobj)
{
  mx25lm51245g_status_t ret = MX25LM51245G_OK;
  /* Check if the instance is already de-initialized */
  if (pobj->is_initialized != MX25LM51245G_INIT_NONE)
  {
    /* Disable Memory mapped mode */
    if (pobj->access_mode == MX25LM51245G_ACCESS_MMP)
    {
      if (mx25lm51245g_disable_memory_mapped(pobj) != MX25LM51245G_OK)
      {
        ret = MX25LM51245G_ERROR;
      }
    }
    if (ret == MX25LM51245G_OK)
    {
      /* Perform a sw reset to return memory to its default configuration */
      if (mx25lm51245g_reset(pobj) != MX25LM51245G_OK)
      {
        ret = MX25LM51245G_ERROR;
      }
      else
      {
        /* Set default pboj values */
        pobj->is_initialized = MX25LM51245G_INIT_NONE;
        pobj->access_mode    = MX25LM51245G_ACCESS_NONE;
        pobj->interface      = MX25LM51245G_SPI_MODE;
        pobj->transfer_rate  = MX25LM51245G_STR_TRANSFER;
#if defined (MX25LM_CALLBACKS) && (MX25LM_CALLBACKS == 1)
        pobj->async_context.async_state    = MX25LM51245G_ASYNC_OPERATION_NONE;
#endif /* MX25LM_CALLBACKS */
      }
    }
  }

  /* Return error status */
  return ret;
}

/**
  * @brief  Configures MX25LM51245G's interface and transfer rate.
  * @param  pobj      : Pointer to the part object structure.
  * @param  interface : Interface Mode SPI or OPI
  * @param  rate      : Transfer rate STR or DTR
  * @retval error status
  */
mx25lm51245g_status_t mx25lm51245g_set_memconfig(mx25lm51245g_obj_t *pobj, mx25lm51245g_interface_t interface,
                                                 mx25lm51245g_transfer_rate_t rate)
{
  mx25lm51245g_status_t ret = MX25LM51245G_OK;

  if ((pobj->access_mode != MX25LM51245G_ACCESS_INDIRECT) || ((interface == MX25LM51245G_SPI_MODE)
                                                              && (rate == MX25LM51245G_DTR_TRANSFER)))
  {
    /* unsupported configuration */
    ret = MX25LM51245G_ERROR;
  }
  else
  {
    if ((pobj->interface == interface) && (pobj->transfer_rate == rate))
    {
      /* Nothing to configure */
    }
    else
    {
      /* reset to default config */
      if (mx25lm51245g_reset(pobj) != MX25LM51245G_OK)
      {
        ret = MX25LM51245G_ERROR;
      }
      else
      {
        switch (interface)
        {
          case MX25LM51245G_SPI_MODE : /* 1-1-1 commands (SPI), Power on H/W default setting */
            break; /* Do nothing - default configuration after reset SPI-STR */
          case MX25LM51245G_OPI_MODE : /* 8-8-8 commands (OPI) */
            if (rate == MX25LM51245G_STR_TRANSFER)
            {
              /* Enter STR OPI interface */
              if (mx25lm51245g_enter_sopi_mode(pobj) != MX25LM51245G_OK)
              {
                ret = MX25LM51245G_ERROR;
              }
            }
            else
            {
              /* Enter DTR OPI interface */
              if (mx25lm51245g_enter_dopi_mode(pobj) != MX25LM51245G_OK)
              {
                ret = MX25LM51245G_ERROR;
              }
            }
            break;
          default :
            ret = MX25LM51245G_ERROR;
            break;
        }
      }
    }
  }
  return ret;
}

/**
  * @brief  This function enables the octal DTR mode of the memory.
  * @param  pobj : Pointer to the part object structure
  * @retval error status
  */
static mx25lm51245g_status_t mx25lm51245g_enter_dopi_mode(mx25lm51245g_obj_t *pobj)
{
  mx25lm51245g_status_t ret = MX25LM51245G_OK;
  uint8_t reg[2];
  hal_xspi_config_t regular_config;

  /* Enable write operations then Write Configuration register 2 (with new dummy cycles)*/
  /* set the fixed 20 DC required for DOPI */
  if (mx25lm51245g_update_dc_reg(pobj, MX25LM51245G_DC_20_LUT_INDEX_0) != MX25LM51245G_OK)
  {
    ret = MX25LM51245G_ERROR;
  }
  /* Enable write operations */
  else if (mx25lm51245g_write_enable(pobj) != MX25LM51245G_OK)
  {
    ret = MX25LM51245G_ERROR;
  }
  /* Write Configuration register 2 (with Octal I/O SPI protocol) */
  else if (mx25lm51245g_write_cfg2_register(pobj, MX25LM51245G_CR2_REG1_ADDR,
                                            MX25LM51245G_CR2_DOPI) != MX25LM51245G_OK)
  {
    ret = MX25LM51245G_ERROR;
  }
  else
  {
    /* Wait that the configuration is effective and check that memory is ready */
    HAL_Delay(MX25LM51245G_WRITE_REG_MAX_TIME);
    /* Get the memory configuration */
    HAL_XSPI_GetConfig(pobj->io.phxspi, &regular_config);
    /* Reconfigure the memory timing of the peripheral  and enable delayblock */
    regular_config.timing.hold       = HAL_XSPI_DELAY_HOLD_QUARTCYCLE;
    regular_config.timing.dlyb_state = HAL_XSPI_DLYB_ON;
    /**
      * Set the new XSPI configuration. Then Tune the delay line length to span one input clock period.
      * The delay unit cell UNIT is determined once the sampling over one input Tclk is completed,
      * retrieve the number of delay units from LNG[11:0] MSB.
      * Then, enable the output clock on the selected phase number 2 (2 unit delays).
      */
    if (HAL_XSPI_SetConfig(pobj->io.phxspi, &regular_config) != HAL_OK)
    {
      ret = MX25LM51245G_ERROR;
    }
    else if (HAL_XSPI_EnableFreeRunningClock(pobj->io.phxspi) != HAL_OK)
    {
      ret = MX25LM51245G_ERROR;
    }
    else if (HAL_XSPI_DLYB_SetConfigDelay(pobj->io.phxspi, 2) != HAL_OK)
    {
      ret = MX25LM51245G_ERROR;
    }
    else if (HAL_XSPI_DLYB_Enable(pobj->io.phxspi) != HAL_OK)
    {
      ret = MX25LM51245G_ERROR;
    }
    else
    {
      /* Update current status parameter */
      pobj->interface      = MX25LM51245G_OPI_MODE;
      pobj->transfer_rate  = MX25LM51245G_DTR_TRANSFER;
      /* Check if the Flash is busy */
      if (mx25lm51245g_wip_autopolling(pobj) != MX25LM51245G_OK)
      {
        ret = MX25LM51245G_ERROR;
      }
      /* Check the configuration has been correctly done */
      else if (mx25lm51245g_read_cfg2_register(pobj, MX25LM51245G_CR2_REG1_ADDR, reg) != MX25LM51245G_OK)
      {
        ret = MX25LM51245G_ERROR;
      }
      else
      {
        if (reg[0] != MX25LM51245G_CR2_DOPI)
        {
          ret = MX25LM51245G_ERROR;
        }
      }
    }
  }

  return ret;
}

/**
  * @brief  This function enables the octal STR mode of the memory.
  * @param  pobj : Pointer to the part object structure
  * @retval error status
  */
static mx25lm51245g_status_t mx25lm51245g_enter_sopi_mode(mx25lm51245g_obj_t *pobj)
{
  mx25lm51245g_status_t ret = MX25LM51245G_OK;
  uint8_t reg[2];

  /* Enable write operations then Write Configuration register 2 (with new dummy cycles)  */
  /* set the fixed 6 DC required for SOPI */
  if (mx25lm51245g_update_dc_reg(pobj, MX25LM51245G_DC_6_LUT_INDEX_3) != MX25LM51245G_OK)
  {
    ret = MX25LM51245G_ERROR;
  }
  /* Enable write operations */
  else if (mx25lm51245g_write_enable(pobj) != MX25LM51245G_OK)
  {
    ret = MX25LM51245G_ERROR;
  }
  /* Write Configuration register 2 */
  else if (mx25lm51245g_write_cfg2_register(pobj, MX25LM51245G_CR2_REG1_ADDR,
                                            MX25LM51245G_CR2_SOPI) != MX25LM51245G_OK)
  {
    ret = MX25LM51245G_ERROR;
  }
  else
  {
    /* Wait that the configuration is effective and check that memory is ready */
    HAL_Delay(MX25LM51245G_WRITE_REG_MAX_TIME);
    /* Update current status parameter *****************************************/
    pobj->interface      = MX25LM51245G_OPI_MODE;
    pobj->transfer_rate  = MX25LM51245G_STR_TRANSFER;
    /* Check Flash busy ? */
    if (mx25lm51245g_wip_autopolling(pobj) != MX25LM51245G_OK)
    {
      ret = MX25LM51245G_ERROR;
    }
    else
    {
      /* Check the configuration has been correctly done */
      if (mx25lm51245g_read_cfg2_register(pobj, MX25LM51245G_CR2_REG1_ADDR, reg) != MX25LM51245G_OK)
      {
        ret = MX25LM51245G_ERROR;
      }
      else
      {
        if (reg[0] != MX25LM51245G_CR2_SOPI)
        {
          ret = MX25LM51245G_ERROR;
        }
      }
    }
  }

  return ret;
}

/**
  * @brief  Write Flash Status register.
  * @param  pobj         : Part object pointer.
  * @param  value        : Value to write to the status register.
  * @retval error status.
  */
static mx25lm51245g_status_t mx25lm51245g_write_status_register(mx25lm51245g_obj_t *pobj, uint8_t value)
{
  hal_xspi_regular_cmd_t s_command;
  uint8_t reg[2] = {0U};

  /* The new status register value to be set */
  reg[0] = value;
  /* Write enable is required for writing to the status register */
  if (mx25lm51245g_write_enable(pobj) != MX25LM51245G_OK)
  {
    return MX25LM51245G_ERROR;
  }

  /* Initialize the command */
  /* Common configuration for all modes */
  /* Set memory operation type */
  s_command.operation_type                  = HAL_XSPI_OPERATION_COMMON_CFG;
  /* Select IOs to be used */
  s_command.io_select                       = HAL_XSPI_IO_7_0;
  /* Address mode to be used */
  s_command.addr_mode                       = HAL_XSPI_ADDR_NONE;
  /* Set memory address width  */
  s_command.addr_width                      = HAL_XSPI_ADDR_8BIT;
  /* Set read/write address */
  s_command.addr                            = 0U;
  /* Set memory address DTR mode  */
  s_command.addr_dtr_mode_status            = HAL_XSPI_ADDR_DTR_DISABLED;
  /* Set memory alternate bytes       */
  s_command.alternate_bytes                 = 0U;
  /* Set memory DQS mode  */
  s_command.dqs_mode_status                 = HAL_XSPI_DQS_DISABLED;
  /* Set memory dummy cycles  */
  s_command.dummy_cycle                     = MX25LM51245G_WRITE_STATUS_DUMMY_CYCLES;

  switch (pobj->interface)
  {
    case MX25LM51245G_SPI_MODE:
      /* Specific configuration for SPI */
      /* Set memory alternate bytes width  */
      s_command.alternate_bytes_width           = HAL_XSPI_ALTERNATE_BYTES_8BIT;
      /* Set memory alternate bytes DTR mode  */
      s_command.alternate_bytes_dtr_mode_status = HAL_XSPI_ALTERNATE_BYTES_DTR_DISABLED;
      /* Set memory alternate bytes mode  */
      s_command.alternate_bytes_mode            = HAL_XSPI_ALTERNATE_BYTES_NONE;
      /* Set memory instruction mode */
      s_command.instruction_mode                = HAL_XSPI_INSTRUCTION_1LINE;
      /* Set memory Double Transfer Rate mode */
      s_command.instruction_dtr_mode_status     = HAL_XSPI_INSTRUCTION_DTR_DISABLED;
      /* Set memory instruction width */
      s_command.instruction_width               = HAL_XSPI_INSTRUCTION_8BIT;
      /* Set memory instruction  */
      s_command.instruction                     = MX25LM51245G_WRITE_STATUS_REG_CMD;
      /* Set memory data mode  */
      s_command.data_mode                       = HAL_XSPI_REGULAR_DATA_1LINE;
      /* Set memory data DTR mode  */
      s_command.data_dtr_mode_status            = HAL_XSPI_DATA_DTR_DISABLED;
      /* Set memory size byte  */
      s_command.size_byte                       = 1U;
      break;

    case MX25LM51245G_OPI_MODE:
      /* Common configuration for OPI */
      /* Set memory instruction mode */
      s_command.instruction_mode      = HAL_XSPI_INSTRUCTION_8LINES;
      /* Set memory instruction width */
      s_command.instruction_width     = HAL_XSPI_INSTRUCTION_16BIT;
      /* Set memory instruction  */
      s_command.instruction           = MX25LM51245G_OCTA_WRITE_STATUS_REG_CMD;
      /* Set memory addressing mode  */
      s_command.addr_mode             = HAL_XSPI_ADDR_NONE;
      /* Set memory data mode  */
      s_command.data_mode             = HAL_XSPI_REGULAR_DATA_8LINES;
      /* Set memory alternate bytes mode  */
      s_command.alternate_bytes_mode  = HAL_XSPI_ALTERNATE_BYTES_8LINES;

      if (pobj->transfer_rate == MX25LM51245G_DTR_TRANSFER)
      {
        /* Specific configuration for DOPI */
        /* In DTR mode, the minimum size byte value is 2 */
        s_command.size_byte                       = 2U;
        /* In DTR mode, the alternate bytes width is set to 24 bits.
         * Since the minimum size byte is 2, the last alternate byte
         * is transmitted as the first byte in the data phase.
         */
        s_command.alternate_bytes_width           = HAL_XSPI_ALTERNATE_BYTES_24BIT;

        /* The last alternate byte is now sent as the first byte in the data phase */
        reg[0] = 0U;
        /* The last byte to be sent contains the new status register value */
        reg[1] = value;

        /* Set memory alternate bytes DTR mode  */
        s_command.alternate_bytes_dtr_mode_status = HAL_XSPI_ALTERNATE_BYTES_DTR_ENABLED;
        /* Set memory Double Transfer Rate mode */
        s_command.instruction_dtr_mode_status     = HAL_XSPI_INSTRUCTION_DTR_ENABLED;
        /* Set memory data DTR mode  */
        s_command.data_dtr_mode_status            = HAL_XSPI_DATA_DTR_ENABLED;
      }
      else
      {
        /* Specific configuration for SOPI */
        /* Set memory alternate bytes width  */
        s_command.alternate_bytes_width           = HAL_XSPI_ALTERNATE_BYTES_32BIT;
        /* Set memory alternate bytes DTR mode  */
        s_command.alternate_bytes_dtr_mode_status = HAL_XSPI_ALTERNATE_BYTES_DTR_DISABLED;
        /* Set memory Double Transfer Rate mode */
        s_command.instruction_dtr_mode_status = HAL_XSPI_INSTRUCTION_DTR_DISABLED;
        /* Set memory data DTR mode  */
        s_command.data_dtr_mode_status        = HAL_XSPI_DATA_DTR_DISABLED;
        /* Set memory size byte  */
        s_command.size_byte                   = 1U;
      }
      break;

    default:
      /* Unsupported interface */
      return MX25LM51245G_ERROR;
      break;
  }

  /* Send the command */
  if (HAL_XSPI_SendRegularCmd(pobj->io.phxspi, &s_command, MX25LM51245G_CMD_TIMEOUT_VALUE) != HAL_OK)
  {
    return MX25LM51245G_ERROR;
  }

  /* Send data */
  if (HAL_XSPI_Transmit(pobj->io.phxspi, reg, MX25LM51245G_CMD_TIMEOUT_VALUE) != HAL_OK)
  {
    return MX25LM51245G_ERROR;
  }

  /* Write Status Register Cycle Time (tWmax = ~40ms) */
  if (mx25lm51245g_wip_autopolling(pobj) != MX25LM51245G_OK)
  {
    return MX25LM51245G_ERROR;
  }

  return MX25LM51245G_OK;
}

/*  Memory information and status --------------------------------------------------*/

/**
  * @brief  Gets the current memory configuration.
  * @param  pobj     : Pointer to the part object structure
  * @param  p_memcfg : Pointer store the memory configuration structure
  * @retval error status.
  */
mx25lm51245g_status_t mx25lm51245g_get_memconfig(mx25lm51245g_obj_t *pobj, mx25lm51245g_memcfg_t *p_memcfg)
{
  if (pobj->is_initialized != MX25LM51245G_INIT_OK)
  {
    return MX25LM51245G_ERROR;
  }
  p_memcfg->interface     = pobj->interface;
  p_memcfg->transfer_rate = pobj->transfer_rate;
  /* return the transfer rate configuration */
  return MX25LM51245G_OK;
}

/**
  * @brief  Gets the current transfer rate configuration.
  * @param  pobj : Pointer to the part object structure
  * @retval transfer rate configuration.
  */
mx25lm51245g_transfer_rate_t mx25lm51245g_get_transfer_rate_cfg(mx25lm51245g_obj_t *pobj)
{
  /* return the transfer rate configuration */
  return pobj->transfer_rate;
}

/**
  * @brief  Gets the current interface configuration.
  * @param  pobj : Pointer to the part object structure
  * @retval interface configuration.
  */
mx25lm51245g_interface_t mx25lm51245g_get_interface_cfg(mx25lm51245g_obj_t *pobj)
{
  /* return the interface configuration */
  return pobj->interface;
}

/**
  * @brief  Reads Flash 3 Byte IDs.
  *         Manufacturer ID, Memory type, Memory density
  * @param  pobj : Pointer to the part object structure
  * @param  id   : Pointer to a 3 bytes buffer
  * @retval error status
  */
mx25lm51245g_status_t mx25lm51245g_read_id(mx25lm51245g_obj_t *pobj, uint8_t *id)
{
  mx25lm51245g_status_t ret = MX25LM51245G_OK;
  hal_xspi_regular_cmd_t s_command;

  /* Initialize the read ID command */
  /* Set memory operation type */
  s_command.operation_type        = HAL_XSPI_OPERATION_COMMON_CFG;
  /* Select IOs to be used */
  s_command.io_select             = HAL_XSPI_IO_7_0;
  /* Set memory address width  */
  s_command.addr_width            = HAL_XSPI_ADDR_32BIT;
  /* Set memory alternate bytes mode  */
  s_command.alternate_bytes_mode  = HAL_XSPI_ALTERNATE_BYTES_NONE;
  /* Set memory alternate bytes       */
  s_command.alternate_bytes       = 0U;
  /* Set memory alternate bytes width */
  s_command.alternate_bytes_width = HAL_XSPI_ALTERNATE_BYTES_8BIT;
  /* Set memory alternate bytes DTR mode  */
  s_command.alternate_bytes_dtr_mode_status =  HAL_XSPI_ALTERNATE_BYTES_DTR_DISABLED;
  /* Set read/write address */
  s_command.addr              = 0U;
  /* Set memory size byte  */
  s_command.size_byte         = 3U;
  /* Set memory data DTR mode  */
  s_command.data_dtr_mode_status     = HAL_XSPI_DATA_DTR_DISABLED;
  switch (pobj->interface)
  {
    case MX25LM51245G_SPI_MODE:
      /* Set memory instruction mode */
      s_command.instruction_mode     = HAL_XSPI_INSTRUCTION_1LINE;
      /* Set memory Double Transfer Rate mode */
      s_command.instruction_dtr_mode_status = HAL_XSPI_INSTRUCTION_DTR_DISABLED;
      /* Set memory instruction width */
      s_command.instruction_width    = HAL_XSPI_INSTRUCTION_8BIT;
      /* Set memory instruction  */
      s_command.instruction          = MX25LM51245G_READ_ID_CMD;
      /* Set memory addressing mode  */
      s_command.addr_mode            = HAL_XSPI_ADDR_NONE;
      /* Set memory address DTR mode  */
      s_command.addr_dtr_mode_status        = HAL_XSPI_ADDR_DTR_DISABLED;
      /* Set memory data mode  */
      s_command.data_mode            = HAL_XSPI_REGULAR_DATA_1LINE;
      /* Set memory dummy cycles  */
      s_command.dummy_cycle          = MX25LM51245G_RDID_DUMMY_CYCLES;
      /* Set memory DQS mode  */
      s_command.dqs_mode_status             = HAL_XSPI_DQS_DISABLED;
      break;
    case MX25LM51245G_OPI_MODE:
      /* Common config for OPI mode */
      /* Set memory instruction mode */
      s_command.instruction_mode    = HAL_XSPI_INSTRUCTION_8LINES;
      /* Set memory data mode  */
      s_command.data_mode           = HAL_XSPI_REGULAR_DATA_8LINES;
      /* Set memory instruction width */
      s_command.instruction_width    = HAL_XSPI_INSTRUCTION_16BIT;
      /* Set memory instruction  */
      s_command.instruction        = MX25LM51245G_OCTA_READ_ID_CMD;
      /* Set memory addressing mode  */
      s_command.addr_mode        = HAL_XSPI_ADDR_8LINES;
      /* Set memory dummy cycles  */
      s_command.dummy_cycle        = MX25LM51245G_OPI_RDID_DUMMY_CYCLES;

      if (pobj->transfer_rate == MX25LM51245G_DTR_TRANSFER)
      {
        /* Set memory Double Transfer Rate mode */
        s_command.instruction_dtr_mode_status = HAL_XSPI_INSTRUCTION_DTR_ENABLED;
        /* Set memory address DTR mode  */
        s_command.addr_dtr_mode_status     = HAL_XSPI_ADDR_DTR_ENABLED;
        /* Set memory DQS mode  */
        s_command.dqs_mode_status            = HAL_XSPI_DQS_ENABLED;
      }
      else
      {
        /* Set memory Double Transfer Rate mode */
        s_command.instruction_dtr_mode_status = HAL_XSPI_INSTRUCTION_DTR_DISABLED;
        /* Set memory address DTR mode  */
        s_command.addr_dtr_mode_status     = HAL_XSPI_ADDR_DTR_DISABLED;
        /* Set memory DQS mode  */
        s_command.dqs_mode_status            = HAL_XSPI_DQS_DISABLED;
      }
      break;
    default:
      ret = MX25LM51245G_ERROR;
      break;
  }
  if (ret == MX25LM51245G_OK)
  {
    /* Configure the command */
    if (HAL_XSPI_SendRegularCmd(pobj->io.phxspi, &s_command, MX25LM51245G_CMD_TIMEOUT_VALUE) != HAL_OK)
    {
      ret = MX25LM51245G_ERROR;
    }
    else
    {
      /* Reception of the data */
      if (HAL_XSPI_Receive(pobj->io.phxspi, id, MX25LM51245G_CMD_TIMEOUT_VALUE) != HAL_OK)
      {
        ret = MX25LM51245G_ERROR;
      }
    }
  }

  return ret;
}

/**
  * @brief  Gets Flash information.
  * @param  pobj   : Pointer to the part object structure.
  * @param  p_info : Pointer to information structure
  */
void mx25lm51245g_get_info(mx25lm51245g_obj_t const *pobj, mx25lm51245g_info_t *p_info)
{
  STM32_UNUSED(pobj);
  /* Configure the structure with the memory configuration */
  p_info->flash_size             = MX25LM51245G_FLASH_SIZE;
  p_info->erase_block_size       = MX25LM51245G_BLOCK_64K;
  p_info->erase_block_number     = (MX25LM51245G_FLASH_SIZE / MX25LM51245G_BLOCK_64K);
  p_info->erase_sector_size      = MX25LM51245G_SECTOR_4K;
  p_info->erase_sector_number    = (MX25LM51245G_FLASH_SIZE / MX25LM51245G_SECTOR_4K);
  p_info->prog_page_size         = MX25LM51245G_PAGE_SIZE;
  p_info->prog_pages_number      = (MX25LM51245G_FLASH_SIZE / MX25LM51245G_PAGE_SIZE);
}

/**
  * @brief  Reads current status of the OSPI memory.
  * @param  pobj     : Pointer to the part object structure.
  * @param  p_status : Pointer to store current memory status
  * @retval error status
  */
mx25lm51245g_status_t mx25lm51245g_get_status(mx25lm51245g_obj_t *pobj, mx25lm51245g_status_t *p_status)
{
  mx25lm51245g_status_t ret = MX25LM51245G_OK;
  static uint8_t reg[2];

  if (mx25lm51245g_read_security_register(pobj, reg) != MX25LM51245G_OK)
  {
    ret = MX25LM51245G_ERROR;
  }
  else
  {
    /* Check for error flag */
    if ((reg[0] & (MX25LM51245G_SECR_P_FAIL | MX25LM51245G_SECR_E_FAIL)) != 0U)
    {
      *p_status = MX25LM51245G_ERROR;
    }
    /* Check for suspend flag */
    else if ((reg[0] & (MX25LM51245G_SECR_PSB | MX25LM51245G_SECR_ESB)) != 0U)
    {
      *p_status = MX25LM51245G_SUSPENDED;
    }
    else
    {
      if (mx25lm51245g_read_status_register(pobj, reg) != MX25LM51245G_OK)
      {
        ret = MX25LM51245G_ERROR;
      }
      /* Check the write in progress bit */
      else if ((reg[0] & MX25LM51245G_SR_WIP) != 0U)
      {
        *p_status = MX25LM51245G_BUSY;
      }
      else
      {
        /* Memory is ready */
        *p_status = MX25LM51245G_OK;
      }
    }
  }

  return ret;
}

/* Memory erase functions --------------------------------------------------*/

/**
  * @brief  Erases the specified block of flash memory.
  *         MX25LM51245G supports 4K sector or 64K block erase commands.
  * @param  pobj       : Part object pointer.
  * @param  addr       : Sector/block Address to erase
  * @param  size       : size configuration MX25LM51245G_ERASE_SECTOR (4K) or MX25LM51245G_ERASE_BLOCK (64K)
  * @retval error status.
  */
mx25lm51245g_status_t mx25lm51245g_erase(mx25lm51245g_obj_t *pobj, uint32_t addr, mx25lm51245g_erase_t size)
{
  /* Check param */
  if (pobj == NULL)
  {
    return MX25LM51245G_ERROR;
  }

  /* Check the memory access mode */
  if (pobj->access_mode == MX25LM51245G_ACCESS_MMP)
  {
    return MX25LM51245G_ERROR;
  }

  /* Check Flash busy state */
  if (mx25lm51245g_wip_autopolling(pobj) != MX25LM51245G_OK)
  {
    return MX25LM51245G_ERROR;
  }

  /* Enable write operations */
  if (mx25lm51245g_write_enable(pobj) != MX25LM51245G_OK)
  {
    return MX25LM51245G_ERROR;
  }

  /* Send the Sector/Block erase command */
  if (size == MX25LM51245G_ERASE_BLOCK)
  {
    if (mx25lm51245g_send_block_erase_cmd(pobj, addr) != MX25LM51245G_OK)
    {
      return MX25LM51245G_ERROR;
    }
  }
  else
  {
    if (mx25lm51245g_send_sector_erase_cmd(pobj, addr) != MX25LM51245G_OK)
    {
      return MX25LM51245G_ERROR;
    }
  }

  /* Check Flash busy state */
  if (mx25lm51245g_wip_autopolling(pobj) != MX25LM51245G_OK)
  {
    return MX25LM51245G_ERROR;
  }

  return MX25LM51245G_OK;
}

/**
  * @brief  Erases all sectors from the sector containing the specified start address
  * up to and including the sector containing (start address + buffer size).
  *
  * All sectors that overlap with the address range [start_address, start_address + buffer_size)
  * will be erased.
  *
  * @param  pobj       : Part object pointer
  * @param  erase_addr : Start address of the data to be erased
  * @param  size_byte  : Size of data to erased in bytes (Range 1 ~ MX25LM51245G_FLASH_SIZE).
  * @retval error status
  */
mx25lm51245g_status_t mx25lm51245g_sectors_erase(mx25lm51245g_obj_t *pobj, uint32_t erase_addr, uint32_t size_byte)
{
  uint32_t sectors_to_erase;
  uint32_t sector_index;

  /* Check param */
  if (pobj == NULL)
  {
    return MX25LM51245G_ERROR;
  }

  /* Check the memory access mode */
  if (pobj->access_mode == MX25LM51245G_ACCESS_MMP)
  {
    return MX25LM51245G_ERROR;
  }

  /* Calculate number of sectors to erase */
  sectors_to_erase = (uint32_t)(size_byte / MX25LM51245G_SECTOR_4K);

  /* Check if there is remaining data that requires an additional sector */
  if (((erase_addr + size_byte) % MX25LM51245G_SECTOR_4K) != 0U)
  {
    sectors_to_erase += 1U;
  }

  for (sector_index = 0; sector_index < sectors_to_erase ; sector_index++)
  {
    /* Calculate next address */
    uint32_t tmp_address = erase_addr + (sector_index * MX25LM51245G_SECTOR_4K);

    /* Check Flash busy state */
    if (mx25lm51245g_wip_autopolling(pobj) != MX25LM51245G_OK)
    {
      return MX25LM51245G_ERROR;
    }

    /* Enable write operations */
    if (mx25lm51245g_write_enable(pobj) != MX25LM51245G_OK)
    {
      return MX25LM51245G_ERROR;
    }

    if (mx25lm51245g_send_sector_erase_cmd(pobj, tmp_address) != MX25LM51245G_OK)
    {
      return MX25LM51245G_ERROR;
    }
  }

  /* Check Flash busy state */
  if (mx25lm51245g_wip_autopolling(pobj) != MX25LM51245G_OK)
  {
    return MX25LM51245G_ERROR;
  }

  return MX25LM51245G_OK;
}

/**
  * @brief  Erases the whole chip.
  * @param  pobj : Pointer to the part object structure
  * @retval error status
  */
mx25lm51245g_status_t mx25lm51245g_mass_erase(mx25lm51245g_obj_t *pobj)
{
  hal_xspi_auto_polling_config_t s_config;
  /* Check param */
  if (pobj == NULL)
  {
    return MX25LM51245G_ERROR;
  }

  /* Check the memory access mode */
  if (pobj->access_mode == MX25LM51245G_ACCESS_MMP)
  {
    return MX25LM51245G_ERROR;
  }

  /* Check Flash busy state */
  if (mx25lm51245g_wip_autopolling(pobj) != MX25LM51245G_OK)
  {
    return MX25LM51245G_ERROR;
  }

  /* Enable write operations */
  if (mx25lm51245g_write_enable(pobj) != MX25LM51245G_OK)
  {
    return MX25LM51245G_ERROR;
  }

  uint32_t cmd;

  /* configure the bulk erase cmd */
  if (pobj->interface == MX25LM51245G_SPI_MODE)
  {
    cmd = MX25LM51245G_BULK_ERASE_CMD;
  }
  else
  {
    cmd = MX25LM51245G_OCTA_BULK_ERASE_CMD;
  }

  /* send the bulk erase cmd */
  if (mx25lm51245g_send_regular_cmd(pobj, HAL_XSPI_ADDR_NONE, HAL_XSPI_REGULAR_DATA_NONE, 0U, cmd,
                                    MX25LM51245G_CE_DUMMY_CYCLES) != MX25LM51245G_OK)
  {
    return MX25LM51245G_ERROR;
  }

  /* Send Command */
  if (mx25lm51245g_send_read_status_reg_cmd(pobj) != MX25LM51245G_OK)
  {
    return MX25LM51245G_ERROR;
  }

  /* Configure and start WIP autopolling - start data phase */
  s_config.match_value           = 0U;
  s_config.match_mask            = MX25LM51245G_SR_WIP | MX25LM51245G_SR_WEL;
  s_config.match_mode            = HAL_XSPI_MATCH_MODE_AND;
  s_config.interval_cycle        = MX25LM51245G_AUTOPOLLING_INTERVAL_TIME;
  s_config.automatic_stop_status = HAL_XSPI_AUTOMATIC_STOP_ENABLED;

  if (HAL_XSPI_ExecRegularAutoPoll(pobj->io.phxspi, &s_config, MX25LM51245G_CHIP_ERASE_TIMEOUT) != HAL_OK)
  {
    return MX25LM51245G_ERROR;
  }

  return MX25LM51245G_OK;
}

/**
  * @brief  Send a (sector/block) erase command and executes async autopoll on WIP & WEL bits.
  * @param  pobj       : Pointer to the part object structure
  * @param  addr       : Sector/block Address to erase
  * @param  size       : size configuration MX25LM51245G_ERASE_SECTOR (4K) or MX25LM51245G_ERASE_BLOCK (64K)
  * @retval error status
  */
mx25lm51245g_status_t mx25lm51245g_erase_it(mx25lm51245g_obj_t *pobj, uint32_t addr, mx25lm51245g_erase_t size)
{
  hal_xspi_auto_polling_config_t s_config;

  /* Check param */
  if (pobj == NULL)
  {
    return MX25LM51245G_ERROR;
  }

  /* Check the memory access mode */
  if (pobj->access_mode == MX25LM51245G_ACCESS_MMP)
  {
    return MX25LM51245G_ERROR;
  }

  /* Check Flash busy state */
  if (mx25lm51245g_wip_autopolling(pobj) != MX25LM51245G_OK)
  {
    return MX25LM51245G_ERROR;
  }

  /* Enable write operations */
  if (mx25lm51245g_write_enable(pobj) != MX25LM51245G_OK)
  {
    return MX25LM51245G_ERROR;
  }

  /* Send the Sector/Block erase command */
  if (size == MX25LM51245G_ERASE_BLOCK)
  {
    if (mx25lm51245g_send_block_erase_cmd(pobj, addr) != MX25LM51245G_OK)
    {
      return MX25LM51245G_ERROR;
    }
  }
  else
  {
    if (mx25lm51245g_send_sector_erase_cmd(pobj, addr) != MX25LM51245G_OK)
    {
      return MX25LM51245G_ERROR;
    }
  }

  /* Send the Read status command */
  if (mx25lm51245g_send_read_status_reg_cmd(pobj) != MX25LM51245G_OK)
  {
    return MX25LM51245G_ERROR;
  }

  /* Configure and start WIP & WEL autopolling */
  s_config.match_value           = 0U;
  s_config.match_mask            = MX25LM51245G_SR_WIP | MX25LM51245G_SR_WEL;
  s_config.match_mode            = HAL_XSPI_MATCH_MODE_AND;
  s_config.interval_cycle        = MX25LM51245G_AUTOPOLLING_INTERVAL_TIME;
  s_config.automatic_stop_status = HAL_XSPI_AUTOMATIC_STOP_ENABLED;
#if defined (MX25LM_CALLBACKS) && (MX25LM_CALLBACKS == 1)
  pobj->async_context.async_state    = MX25LM51245G_ASYNC_OPERATION_ERASE;
#endif /* MX25LM_CALLBACKS */
  if (HAL_XSPI_ExecRegularAutoPoll_IT(pobj->io.phxspi, &s_config) != HAL_OK)
  {
#if defined (MX25LM_CALLBACKS) && (MX25LM_CALLBACKS == 1)
    pobj->async_context.async_state    = MX25LM51245G_ASYNC_OPERATION_NONE;
#endif /* MX25LM_CALLBACKS */
    return MX25LM51245G_ERROR;
  }

  return MX25LM51245G_OK;
}

/**
  * @brief  Send a Chip erase command and executes async autopoll on WIP & WEL bits.
  * @param  pobj : Pointer to the part object structure
  * @retval error status
  */
mx25lm51245g_status_t mx25lm51245g_mass_erase_it(mx25lm51245g_obj_t *pobj)
{
  hal_xspi_auto_polling_config_t s_config;

  /* Check param */
  if (pobj == NULL)
  {
    return MX25LM51245G_ERROR;
  }

  /* Check the memory access mode */
  if (pobj->access_mode == MX25LM51245G_ACCESS_MMP)
  {
    return MX25LM51245G_ERROR;
  }

  /* Check Flash busy state */
  if (mx25lm51245g_wip_autopolling(pobj) != MX25LM51245G_OK)
  {
    return MX25LM51245G_ERROR;
  }

  /* Enable write operations */
  if (mx25lm51245g_write_enable(pobj) != MX25LM51245G_OK)
  {
    return MX25LM51245G_ERROR;
  }
  uint32_t cmd;

  /* configure the bulk erase cmd */
  if (pobj->interface == MX25LM51245G_SPI_MODE)
  {
    cmd = MX25LM51245G_BULK_ERASE_CMD;
  }
  else
  {
    cmd = MX25LM51245G_OCTA_BULK_ERASE_CMD;
  }

  /* send the bulk erase cmd */
  if (mx25lm51245g_send_regular_cmd(pobj, HAL_XSPI_ADDR_NONE, HAL_XSPI_REGULAR_DATA_NONE, 0U, cmd,
                                    MX25LM51245G_RSTEN_DUMMY_CYCLES) != MX25LM51245G_OK)
  {
    return MX25LM51245G_ERROR;
  }

  /* Send the Read status command */
  if (mx25lm51245g_send_read_status_reg_cmd(pobj) != MX25LM51245G_OK)
  {
    return MX25LM51245G_ERROR;
  }

  /* Configure and start WIP & WEL autopolling */
  s_config.match_value           = 0U;
  s_config.match_mask            = MX25LM51245G_SR_WIP | MX25LM51245G_SR_WEL;
  s_config.match_mode            = HAL_XSPI_MATCH_MODE_AND;
  s_config.interval_cycle        = MX25LM51245G_AUTOPOLLING_INTERVAL_TIME;
  s_config.automatic_stop_status = HAL_XSPI_AUTOMATIC_STOP_ENABLED;
#if defined (MX25LM_CALLBACKS) && (MX25LM_CALLBACKS == 1)
  pobj->async_context.async_state    = MX25LM51245G_ASYNC_OPERATION_MASS_ERASE;
#endif /* MX25LM_CALLBACKS */
  if (HAL_XSPI_ExecRegularAutoPoll_IT(pobj->io.phxspi, &s_config) != HAL_OK)
  {
#if defined (MX25LM_CALLBACKS) && (MX25LM_CALLBACKS == 1)
    pobj->async_context.async_state   = MX25LM51245G_ASYNC_OPERATION_NONE;
#endif /* MX25LM_CALLBACKS */
    return MX25LM51245G_ERROR;
  }

  return MX25LM51245G_OK;
}

/* Memory block protection functions ---------------------------------------------*/

/**
  * @brief  Sets the block protection configuration in the flash memory.
  * @param  pobj     : Part object pointer.
  * @param  bp_cfg   : Block protection configuration to set.
  * @retval error status.
  * @note   The block protection configuration is non-volatile.
  *         Only read operations are allowed for protected blocks, program/erase commands are rejected.
  */
mx25lm51245g_status_t mx25lm51245g_set_block_protection_cfg(mx25lm51245g_obj_t *pobj, mx25lm51245g_bp_cfg_t bp_cfg)
{
  /* Write the new BP configuration to the status register */
  if (mx25lm51245g_write_status_register(pobj, (uint8_t)bp_cfg) != MX25LM51245G_OK)
  {
    return MX25LM51245G_ERROR;
  }

  return MX25LM51245G_OK;
}

/**
  * @brief  Reads the block protection configuration from the flash memory.
  * @param  pobj     : Part object pointer.
  * @param  p_bp_cfg : Pointer to store block protection configuration.
  * @retval error status.
  */
mx25lm51245g_status_t mx25lm51245g_get_block_protection_cfg(mx25lm51245g_obj_t *pobj, mx25lm51245g_bp_cfg_t *p_bp_cfg)
{
  uint8_t reg = 0U;

  /* read the BP configuration from the status register */
  if (mx25lm51245g_read_status_register(pobj, &reg) != MX25LM51245G_OK)
  {
    return MX25LM51245G_ERROR;
  }
  if (reg >= (uint8_t)MX25LM51245G_BP_ALL_BLOCKS)
  {
    *p_bp_cfg = MX25LM51245G_BP_ALL_BLOCKS;
  }
  else
  {
    switch (reg)
    {
      case (uint8_t)MX25LM51245G_BP_NONE:
        *p_bp_cfg = MX25LM51245G_BP_NONE;
        break;
      case (uint8_t)MX25LM51245G_BP_1_BLOCK:
        *p_bp_cfg = MX25LM51245G_BP_1_BLOCK;
        break;
      case (uint8_t)MX25LM51245G_BP_2_BLOCKS:
        *p_bp_cfg = MX25LM51245G_BP_2_BLOCKS;
        break;
      case (uint8_t)MX25LM51245G_BP_3_BLOCKS:
        *p_bp_cfg = MX25LM51245G_BP_3_BLOCKS;
        break;
      case (uint8_t)MX25LM51245G_BP_4_BLOCKS:
        *p_bp_cfg = MX25LM51245G_BP_4_BLOCKS;
        break;
      case (uint8_t)MX25LM51245G_BP_5_BLOCKS:
        *p_bp_cfg = MX25LM51245G_BP_5_BLOCKS;
        break;
      case (uint8_t)MX25LM51245G_BP_6_BLOCKS:
        *p_bp_cfg = MX25LM51245G_BP_6_BLOCKS;
        break;
      case (uint8_t)MX25LM51245G_BP_7_BLOCKS:
        *p_bp_cfg = MX25LM51245G_BP_7_BLOCKS;
        break;
      case (uint8_t)MX25LM51245G_BP_8_BLOCKS:
        *p_bp_cfg = MX25LM51245G_BP_8_BLOCKS;
        break;
      case (uint8_t)MX25LM51245G_BP_9_BLOCKS:
        *p_bp_cfg = MX25LM51245G_BP_9_BLOCKS;
        break;
      case (uint8_t)MX25LM51245G_BP_10_BLOCKS:
        *p_bp_cfg = MX25LM51245G_BP_10_BLOCKS;
        break;
      case (uint8_t)MX25LM51245G_BP_11_BLOCKS:
        *p_bp_cfg = MX25LM51245G_BP_11_BLOCKS;
        break;
      default:
        return MX25LM51245G_ERROR;
        break;
    }
  }

  return MX25LM51245G_OK;
}

/**
  * @brief  Sets the top/bottom OTP bit configuration in the flash memory.
  * @param  pobj     : Part object pointer.
  * @param  tb_cfg   : top/bottom bit configuration to set.
  * @retval error status.
  * @note   The Top/Bottom bit (TB) is one time programmable (OTP).
  *         By default the bit is set to 0 (TOP), if changed to 1, it can't be reverted back as the bit is OTP.
  */
mx25lm51245g_status_t mx25lm51245g_set_top_bottom_cfg(mx25lm51245g_obj_t *pobj, mx25lm51245g_bp_otp_tb_cfg_t tb_cfg)
{
  uint8_t reg = 0U;

  /* Read the current cfg register value */
  if (mx25lm51245g_read_cfg_register(pobj, &reg) != MX25LM51245G_OK)
  {
    return MX25LM51245G_ERROR;
  }
  /* Clear the TB bit */
  reg &= ~(MX25LM51245G_CR1_TB);

  if (tb_cfg == MX25LM51245G_BP_OTP_TB_BOTTOM)
  {
    reg |= MX25LM51245G_CR1_TB;
  }
  /* Write back to cfg register */
  if (mx25lm51245g_write_cfg_register(pobj, reg) != MX25LM51245G_OK)
  {
    return MX25LM51245G_ERROR;
  }

  return MX25LM51245G_OK;
}

/**
  * @brief  Reads the top/bottom configuration from the flash memory.
  * @param  pobj     : Part object pointer.
  * @param  p_tb_cfg : Pointer to store top/bottom configuration.
  * @retval error status.
  */
mx25lm51245g_status_t mx25lm51245g_get_top_bottom_cfg(mx25lm51245g_obj_t *pobj, mx25lm51245g_bp_otp_tb_cfg_t *p_tb_cfg)
{
  uint8_t reg = 0U;
  uint8_t tb_value = 0U;

  /* read the BP configuration from the status register */
  if (mx25lm51245g_read_cfg_register(pobj, &reg) != MX25LM51245G_OK)
  {
    return MX25LM51245G_ERROR;
  }
  tb_value = (reg & (uint8_t)0x08) >> 3;
  /* TB is bit3 in the configuration register */
  if (tb_value == 1U)
  {
    /* TB bit is set */
    *p_tb_cfg = MX25LM51245G_BP_OTP_TB_BOTTOM;
  }
  else
  {
    /* TB bit is reset */
    *p_tb_cfg = MX25LM51245G_BP_OTP_TB_TOP;
  }

  return MX25LM51245G_OK;
}

/* Memory Read/Write functions --------------------------------------------------*/

/**
  * @brief  Reads an amount of data from the OSPI memory in polling mode.
  * @param  pobj       : Pointer to the part object structure.
  * @param  addr  : Read start address
  * @param  addr_size  : Address size
  * @param  p_data     : Pointer to data to be read
  * @param  size_bytes : Size of data to read in bytes
  * @retval error status
  */
mx25lm51245g_status_t mx25lm51245g_read(mx25lm51245g_obj_t *pobj, uint32_t addr, mx25lm51245g_addr_size_t addr_size,
                                        uint8_t *p_data, uint32_t size_bytes)
{
  mx25lm51245g_status_t ret = MX25LM51245G_OK;

  if (mx25lm51245g_read_cmd(pobj, addr_size, addr, size_bytes) != MX25LM51245G_OK)
  {
    ret = MX25LM51245G_ERROR;
  }
  else
  {
    /* Reception of the data */
    if (HAL_XSPI_Receive(pobj->io.phxspi, p_data, MX25LM51245G_CMD_TIMEOUT_VALUE) != HAL_OK)
    {
      ret = MX25LM51245G_ERROR;
    }
  }

  return ret;
}

/**
  * @brief  Writes an amount of data to the OSPI memory in polling mode.
  * @param  pobj       : Pointer to the part object structure
  * @param  addr       : Write start address.
  * @param  addr_size  : Address size.
  * @param  p_data     : Pointer to data to be written.
  * @param  size_byte  : Size of data to write in bytes.
  * @retval error status
  */
mx25lm51245g_status_t mx25lm51245g_write(mx25lm51245g_obj_t *pobj, uint32_t addr, mx25lm51245g_addr_size_t addr_size,
                                         uint8_t *p_data,  uint32_t size_byte)
{
  mx25lm51245g_status_t ret = MX25LM51245G_OK;
  uint32_t write_addr = addr;
  uint32_t write_index;

  if (size_byte == 0U)
  {
    /* nothing to write */
  }
  else
  {
    const uint32_t max_first_page = MX25LM51245G_PAGE_SIZE - (addr % MX25LM51245G_PAGE_SIZE);
    /* Amount of data in the first page */
    const uint32_t first_page = (max_first_page <= size_byte) ? max_first_page : size_byte;
    /* Number of full pages to write */
    const uint32_t full_pages = (size_byte - first_page) / MX25LM51245G_PAGE_SIZE;
    /* Size after the last full page */
    const uint32_t last_page = (size_byte - first_page) % MX25LM51245G_PAGE_SIZE;

    if (mx25lm51245g_write_page(pobj, addr_size, write_addr, p_data, first_page) != MX25LM51245G_OK)
    {
      ret = MX25LM51245G_ERROR;
    }
    else
    {
      write_index = first_page;
      write_addr += first_page;

      for (uint32_t ipage = 0; ipage < full_pages; ipage++)
      {
        /* Check if an error has occurred before writing to the next page */
        if (ret == MX25LM51245G_OK)
        {
          if (mx25lm51245g_write_page(pobj, addr_size, write_addr, &p_data[write_index],
                                      MX25LM51245G_PAGE_SIZE) != MX25LM51245G_OK)
          {
            ret = MX25LM51245G_ERROR;
          }
          write_index += MX25LM51245G_PAGE_SIZE;
          write_addr += MX25LM51245G_PAGE_SIZE;
        }
      }
      if (ret == MX25LM51245G_OK)
      {
        if (last_page != 0U)
        {
          if (mx25lm51245g_write_page(pobj, addr_size, write_addr, &p_data[write_index], last_page) != MX25LM51245G_OK)
          {
            ret = MX25LM51245G_ERROR;
          }
        }
      }
    }
  }
  return ret;
}

/**
  * @brief Writes a block of data to the memory in polling mode (max 1 page).
  * @param  pobj       : Pointer to the part object structure.
  * @param  addr_size  : Address size.
  * @param  addr       : Write address.
  * @param  p_data     : Pointer to data to be written.
  * @param  size_byte  : Size of data to write in bytes (=< MX25LM51245G_PAGE_SIZE).
  * @retval error status
  */
static mx25lm51245g_status_t mx25lm51245g_write_page(mx25lm51245g_obj_t *pobj, mx25lm51245g_addr_size_t addr_size,
                                                     uint32_t addr, uint8_t *p_data, uint32_t size_byte)
{
  mx25lm51245g_status_t ret;

  if (mx25lm51245g_wip_autopolling(pobj) != MX25LM51245G_OK)
  {
    ret = MX25LM51245G_ERROR;
  }
  else if (mx25lm51245g_write_enable(pobj) != MX25LM51245G_OK)
  {
    ret = MX25LM51245G_ERROR;
  }
  else if (mx25lm51245g_write_cmd(pobj, addr_size, addr, size_byte) != MX25LM51245G_OK)
  {
    ret = MX25LM51245G_ERROR;
  }
  else if (HAL_XSPI_Transmit(pobj->io.phxspi, p_data, MX25LM51245G_CMD_TIMEOUT_VALUE) != HAL_OK)
  {
    ret = MX25LM51245G_ERROR;
  }
  else if (mx25lm51245g_wip_autopolling(pobj) != MX25LM51245G_OK)
  {
    ret = MX25LM51245G_ERROR;
  }
  else
  {
    ret = MX25LM51245G_OK;
  }

  return ret;
}

#if defined (USE_HAL_XSPI_DMA) && (USE_HAL_XSPI_DMA == 1)
#if defined (MX25LM_CALLBACKS) && (MX25LM_CALLBACKS == 1)

/**
  * @brief  Registers a user-provided aligned buffer used for asynchronous DMA transfers.
  * @param  pobj   : Pointer to the part object structure.
  * @param  buffer : Pointer to the aligned buffer (must be aligned to MX25LM51245G_ALIGNMENT_SIZE).
  *                  Pass NULL to clear the registration and fall back to the internal static buffer.
  * @param  size   : Size of the buffer in bytes (must be >= MX25LM51245G_PAGE_SIZE when buffer is not NULL).
  * @retval error status
  */
mx25lm51245g_status_t mx25lm51245g_set_dma_buffer(mx25lm51245g_obj_t *pobj, uint8_t *buffer, uint32_t size)
{
  /* Check param */
  if (pobj == NULL)
  {
    return MX25LM51245G_ERROR;
  }

  /* Do not change the buffer while a transfer is ongoing */
  if (pobj->async_context.async_state != MX25LM51245G_ASYNC_OPERATION_NONE)
  {
    return MX25LM51245G_ERROR;
  }

  if (buffer == NULL)
  {
    /* Restore the internal static bounce buffer */
    pobj->async_context.dma_buffer      = mx25lm51245g_async_page_buffer;
    pobj->async_context.dma_buffer_size = MX25LM51245G_PAGE_SIZE;
    return MX25LM51245G_OK;
  }

  /* The user buffer must be able to hold at least one full page */
  if (size < MX25LM51245G_PAGE_SIZE)
  {
    return MX25LM51245G_ERROR;
  }

  pobj->async_context.dma_buffer      = buffer;
  pobj->async_context.dma_buffer_size = size;

  return MX25LM51245G_OK;
}

/**
  * @brief  Sends the read command and reads asynchronously an amount of data from the flash memory.
  * @param  pobj        : Part object pointer.
  * @param  addr_size   : Address size.
  * @param  pdata       : Pointer to data to be read.
  * @param  read_addr   : Read start address.
  * @param  size_byte   : Size of data to read in bytes.
  * @retval error status.
  */
static void mx25lm51245g_async_read(mx25lm51245g_obj_t *pobj, mx25lm51245g_addr_size_t addr_size,
                                    uint32_t read_addr, uint32_t size_byte, uint8_t *pdata)
{
  hal_xspi_regular_cmd_t s_command;

  /* Initialize the read command */
  /* Set memory operation type */
  s_command.operation_type                    = HAL_XSPI_OPERATION_COMMON_CFG;
  /* Select IOs to be used */
  s_command.io_select                         = HAL_XSPI_IO_7_0;
  /* Set read/write address */
  s_command.addr                              = read_addr;
  /* Set memory alternate bytes mode  */
  s_command.alternate_bytes_mode              = HAL_XSPI_ALTERNATE_BYTES_NONE;
  /* Set memory alternate bytes       */
  s_command.alternate_bytes                   = 0U;
  /* Set memory alternate bytes width  */
  s_command.alternate_bytes_width             = HAL_XSPI_ALTERNATE_BYTES_8BIT;
  /* Set memory alternate bytes DTR mode  */
  s_command.alternate_bytes_dtr_mode_status   = HAL_XSPI_ALTERNATE_BYTES_DTR_DISABLED;
  /* Set memory size byte  */
  s_command.size_byte                         = size_byte;

  switch (pobj->interface)
  {
    case MX25LM51245G_SPI_MODE:
      /* Set memory instruction mode */
      s_command.instruction_mode            = HAL_XSPI_INSTRUCTION_1LINE;
      /* Set memory Double Transfer Rate mode */
      s_command.instruction_dtr_mode_status = HAL_XSPI_INSTRUCTION_DTR_DISABLED;
      /* Set memory instruction width */
      s_command.instruction_width           = HAL_XSPI_INSTRUCTION_8BIT;
      /* Set memory instruction  */
      s_command.instruction                 = (addr_size == MX25LM51245G_3BYTES_SIZE)
                                              ? MX25LM51245G_FAST_READ_CMD
                                              : MX25LM51245G_4_BYTE_ADDR_FAST_READ_CMD;
      /* Set memory addressing mode  */
      s_command.addr_mode                   = HAL_XSPI_ADDR_1LINE;
      /* Set memory address DTR mode  */
      s_command.addr_dtr_mode_status        = HAL_XSPI_ADDR_DTR_DISABLED;
      /* Set memory address width  */
      s_command.addr_width                  = (addr_size == MX25LM51245G_3BYTES_SIZE)
                                              ? HAL_XSPI_ADDR_24BIT
                                              : HAL_XSPI_ADDR_32BIT;
      /* Set memory data mode  */
      s_command.data_mode                   = HAL_XSPI_REGULAR_DATA_1LINE;
      /* Set memory data DTR mode  */
      s_command.data_dtr_mode_status        = HAL_XSPI_DATA_DTR_DISABLED;
      /* Set memory dummy cycles  */
      s_command.dummy_cycle                 = MX25LM51245G_FAST_READ_DUMMY_CYCLES;
      /* Set memory DQS mode  */
      s_command.dqs_mode_status             = HAL_XSPI_DQS_DISABLED;
      break;

    case MX25LM51245G_OPI_MODE:
      /* Set memory instruction mode */
      s_command.instruction_mode     = HAL_XSPI_INSTRUCTION_8LINES;
      /* Set memory instruction width */
      s_command.instruction_width    = HAL_XSPI_INSTRUCTION_16BIT;
      /* Set memory addressing mode  */
      s_command.addr_mode            = HAL_XSPI_ADDR_8LINES;
      /* Set memory data mode  */
      s_command.data_mode            = HAL_XSPI_REGULAR_DATA_8LINES;
      /* Set the dummy cycle parameter based on the XSPI frequency
         by referring to the mx25lm51245g_dc_freq_lut table */
      /* This value corresponds to the XSPI register dummy cycles */
      s_command.dummy_cycle          = mx25lm51245g_dc_freq_lut[pobj->async_context.lut_index].dc_cmd_value;

      if (pobj->transfer_rate == MX25LM51245G_DTR_TRANSFER)
      {

        /* Set memory Double Transfer Rate mode */
        s_command.instruction_dtr_mode_status = HAL_XSPI_INSTRUCTION_DTR_ENABLED;
        /* Set memory instruction  */
        s_command.instruction                 = MX25LM51245G_OCTA_READ_DTR_CMD;
        /* Set memory address DTR mode  */
        s_command.addr_dtr_mode_status        = HAL_XSPI_ADDR_DTR_ENABLED;
        /* Set memory address width  */
        s_command.addr_width                  = HAL_XSPI_ADDR_32BIT;
        /* Set memory data DTR mode  */
        s_command.data_dtr_mode_status        = HAL_XSPI_DATA_DTR_ENABLED;
        /* Set memory DQS mode  */
        s_command.dqs_mode_status             = HAL_XSPI_DQS_ENABLED;
      }
      else
      {
        /* Set memory Double Transfer Rate mode */
        s_command.instruction_dtr_mode_status = HAL_XSPI_INSTRUCTION_DTR_DISABLED;
        /* Set memory instruction  */
        s_command.instruction                 = MX25LM51245G_OCTA_READ_CMD;
        /* Set memory address DTR mode  */
        s_command.addr_dtr_mode_status        = HAL_XSPI_ADDR_DTR_DISABLED;
        /* Set memory address width  */
        s_command.addr_width                  = HAL_XSPI_ADDR_32BIT;
        /* Set memory data DTR mode  */
        s_command.data_dtr_mode_status        = HAL_XSPI_DATA_DTR_DISABLED;
        /* Set memory DQS mode  */
        s_command.dqs_mode_status             = HAL_XSPI_DQS_DISABLED;
      }
      break;

    default:
      break;
  }

  /* Send the command */
  (void)HAL_XSPI_SendRegularCmd(pobj->io.phxspi, &s_command, MX25LM51245G_CMD_TIMEOUT_VALUE);
#if defined (__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
  SCB_CleanDCache_by_Addr(pdata, (int32_t)size_byte);
#endif /* __DCACHE_PRESENT */
  (void)HAL_XSPI_Receive_DMA(pobj->io.phxspi, pdata);
}

/**
  * @brief  Reads an amount of data from the flash memory in DMA mode.
  * @param  pobj       : Part object pointer.
  * @param  addr       : Read start address.
  * @param  addr_size  : Address size.
  * @param  pdata      : Pointer to data to be read.
  * @param  size_byte  : Size of data to read in bytes.
  * @note   In DTR mode, read_addr and size_byte must both be even,
  *         otherwise MX25LM51245G_ERROR is returned.
  * @retval error status.
  */
mx25lm51245g_status_t mx25lm51245g_read_dma_async(mx25lm51245g_obj_t *pobj, uint32_t addr,
                                                  mx25lm51245g_addr_size_t addr_size,
                                                  uint8_t *pdata, uint32_t size_byte)
{
  /* Check param */
  if ((pobj == NULL) || (pdata == NULL))
  {
    return MX25LM51245G_ERROR;
  }
  /* Unsupported configuration check */
  if ((pobj->access_mode == MX25LM51245G_ACCESS_MMP)
      || ((pobj->interface == MX25LM51245G_OPI_MODE) && (addr_size == MX25LM51245G_3BYTES_SIZE)))
  {
    return MX25LM51245G_ERROR;
  }
  if ((pobj->transfer_rate == MX25LM51245G_DTR_TRANSFER)
      && (((size_byte & 1U) != 0U) || ((addr & 1U) != 0U)))
  {
    /* DTR mode transfers data by pairs: read_addr and size_byte must both be even */
    return MX25LM51245G_ERROR;
  }
  if (size_byte == 0U)
  {
    /* nothing to read */
    return MX25LM51245G_OK;
  }
  /* make sure no transfer is ongoing */
  if (pobj->async_context.async_state != MX25LM51245G_ASYNC_OPERATION_NONE)
  {
    return MX25LM51245G_ERROR;
  }
  /* Check Flash busy state */
  if (mx25lm51245g_wip_autopolling(pobj) != MX25LM51245G_OK)
  {
    return MX25LM51245G_ERROR;
  }
  if (pobj->interface == MX25LM51245G_OPI_MODE)
  {
    /* Retrieve the dummy cycles lookup table index based on the XSPI frequency */
    /* This index is used to reference the corresponding command parameter and dummy cycles register values */
    pobj->async_context.lut_index = mx25lm51245g_get_lookup_dummy_cycles_index(pobj);

    /* Update the dummy cycles register only if the frequency index has changed */
    if (pobj->async_context.lut_index != pobj->dc_lut_index)
    {
      if (mx25lm51245g_update_dc_reg(pobj, (uint8_t)pobj->async_context.lut_index) != MX25LM51245G_OK)
      {
        return MX25LM51245G_ERROR;
      }
    }
  }
  pobj->async_context.addr_size            = addr_size;
  pobj->async_context.current_addr         = addr;
  pobj->async_context.transfer_buffer      = pdata;
  pobj->async_context.current_index        = 0U;
  pobj->async_context.total_size           = size_byte;
  pobj->async_context.current_transfer_len = 0U;
  pobj->async_context.async_state          = MX25LM51245G_ASYNC_OPERATION_READ;
  pobj->async_context.async_read_phase     = MX25LM51245G_ASYNC_READING;
  /* An aligned user buffer is used directly for DMA (no bounce buffer / no cpy) */
  if (((uintptr_t)pdata % MX25LM51245G_ALIGNMENT_SIZE) == 0U)
  {
    pobj->async_context.is_aligned = 1U;
  }
  else
  {
    pobj->async_context.is_aligned = 0U;
  }
  HAL_XSPI_SetUserData(pobj->io.phxspi, pobj);
  mx25lm51245g_async_rx_setup(pobj);
  return MX25LM51245G_OK;
}

/**
  * @brief  Determines whether the current read chunk can be received directly into
  *         the user buffer via DMA (no bounce buffer / no cpy).
  * @param  pobj : Part object pointer.
  * @retval 1 if the chunk uses direct DMA, 0 if it must go through the bounce buffer.
  * @note   Direct DMA requires an aligned user buffer with at least one full
  *         alignment unit remaining; any smaller tail goes through the bounce buffer.
  */
static uint8_t mx25lm51245g_read_uses_direct_dma(const mx25lm51245g_obj_t *pobj)
{
  uint32_t remaining = pobj->async_context.total_size - pobj->async_context.current_index;
  uint8_t  direct;

  if ((pobj->async_context.is_aligned != 0U) && (remaining >= MX25LM51245G_ALIGNMENT_SIZE))
  {
    direct = 1U;
  }
  else
  {
    direct = 0U;
  }

  return direct;
}

/**
  * @brief  Sets up the next read transfer chunk into the aligned page buffer.
  * @param  pobj       : Part object pointer.
  * @retval None.
  */
static void mx25lm51245g_async_rx_setup(mx25lm51245g_obj_t *pobj)
{
  uint32_t remaining = pobj->async_context.total_size - pobj->async_context.current_index;

  if (remaining == 0U)
  {
    /* Nothing left to transfer */
    xspi_rd_cplt_cb(pobj->io.phxspi);
  }
  else
  {
    uint8_t *buffer;
    uint32_t chunk;

    if (mx25lm51245g_read_uses_direct_dma(pobj) != 0U)
    {
      /* Aligned user buffer: receive directly into it, no bounce buffer / no cpy. */
      buffer = &pobj->async_context.transfer_buffer[pobj->async_context.current_index];
      chunk  = remaining - (remaining % MX25LM51245G_ALIGNMENT_SIZE);
    }
    else
    {
      buffer = pobj->async_context.dma_buffer;
      /* Limit each transfer to the size of the active bounce buffer */
      chunk  = (remaining > pobj->async_context.dma_buffer_size) ? pobj->async_context.dma_buffer_size : remaining;
    }
    /* DTR mode transfers data by pairs: keep the chunk size even */
    if (pobj->transfer_rate == MX25LM51245G_DTR_TRANSFER)
    {
      chunk &= ~1U;
    }
    pobj->async_context.current_transfer_len = chunk;

    (void)HAL_XSPI_RegisterRxCpltCallback(pobj->io.phxspi, xspi_rx_cplt_cb);
    mx25lm51245g_async_read(pobj, pobj->async_context.addr_size, pobj->async_context.current_addr,
                            chunk, buffer);
  }
}

/**
  * @brief  RX chunk completion callback: flags the chunk for deferred handling.
  * @param  phxpi : pointer to xspi handle.
  */
static void xspi_rx_cplt_cb(hal_xspi_handle_t *phxspi)
{
  mx25lm51245g_obj_t *pobj = (mx25lm51245g_obj_t *)HAL_XSPI_GetUserData(phxspi);

  if (pobj->async_context.async_read_phase == MX25LM51245G_ASYNC_READING)
  {
    /* Defer the copy to thread mode (mx25lm51245g_exec_data_handler). */
    pobj->async_context.async_read_phase = MX25LM51245G_ASYNC_READ_CPY_PENDING;
  }
}

/**
  * @brief  Thread-mode data handler for the asynchronous DMA state machine.
  *         Performs the deferred bounce-buffer copies outside of interrupt context.
  * @param  pobj  : Part object pointer.
  * @note   Must be called periodically (e.g. in the application main loop).
  *         Only performs work when a copy is pending, otherwise returns immediately.
  */
mx25lm51245g_status_t mx25lm51245g_exec_data_handler(mx25lm51245g_obj_t *pobj)
{
  mx25lm51245g_status_t ret = MX25LM51245G_ERROR;
  uint8_t *buffer;

  /* Check param */
  if (pobj == NULL)
  {
    return ret;
  }

  switch (pobj->async_context.async_state)
  {
    case MX25LM51245G_ASYNC_OPERATION_WRITE:
      if (pobj->async_context.async_write_phase == MX25LM51245G_ASYNC_WRITE_CPY_PENDING)
      {
        if (pobj->async_context.is_aligned == 0U)
        {
          /* Unaligned user buffer: copy the next chunk into the bounce buffer. */
          buffer = pobj->async_context.dma_buffer;
          for (uint32_t i = 0U; i < pobj->async_context.current_transfer_len; i++)
          {
            buffer[i] = pobj->async_context.transfer_buffer[pobj->async_context.current_index + i];
          }
        }

        /* Kick off the write sequence: WIP check -> write enable -> WEL check -> data phase */
        pobj->async_context.async_write_phase = MX25LM51245G_ASYNC_WIP_CHECK;
        mx25lm51245g_register_status_match_callback(pobj, mx25lm51245g_wip_check_cb);
        mx25lm51245g_wip_autopolling_it(pobj);
      }
      ret = MX25LM51245G_BUSY;
      break;

    case MX25LM51245G_ASYNC_OPERATION_READ:
      if (pobj->async_context.async_read_phase == MX25LM51245G_ASYNC_READ_CPY_PENDING)
      {
        if (mx25lm51245g_read_uses_direct_dma(pobj) != 0U)
        {
          /* Aligned user buffer: chunk was received directly into it. */
#if defined (__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
          SCB_CleanInvalidateDCache_by_Addr(
            &pobj->async_context.transfer_buffer[pobj->async_context.current_index],
            (int32_t)pobj->async_context.current_transfer_len);
#endif /* __DCACHE_PRESENT */
        }
        else
        {
          buffer = pobj->async_context.dma_buffer;
#if defined (__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
          SCB_CleanInvalidateDCache_by_Addr(buffer, (int32_t)pobj->async_context.current_transfer_len);
#endif /* __DCACHE_PRESENT */
          /* Copy the received data from the bounce buffer to the user buffer. */
          for (uint32_t i = 0U; i < pobj->async_context.current_transfer_len; i++)
          {
            pobj->async_context.transfer_buffer[pobj->async_context.current_index + i] = buffer[i];
          }
        }
        pobj->async_context.current_addr  += pobj->async_context.current_transfer_len;
        pobj->async_context.current_index += pobj->async_context.current_transfer_len;

        /* Set up the next chunk (or complete the read) */
        pobj->async_context.async_read_phase = MX25LM51245G_ASYNC_READING;
        mx25lm51245g_async_rx_setup(pobj);
      }
      ret = MX25LM51245G_BUSY;
      break;
    case MX25LM51245G_ASYNC_OPERATION_NONE:
      /* No async operation in progress - memory is in a ready state */
      ret = MX25LM51245G_OK;
      break;
    default:
      /* Not a read/write async operation */
      ret = MX25LM51245G_ERROR;
      break;
  }
  return ret;
}


/**
  * @brief  Writes an amount of data to the OSPI memory in DMA mode.
  * @param  pobj       : Component object pointer
  * @param  addr       : Write start address
  * @param  addr_size  : Address size
  * @param  pdata      : Pointer to data to be written
  * @param  size_byte  : Size of data to write in bytes.
  * @note   - Address size is forced to 3 Bytes when the 4 Bytes address size
  *         command is not available for the specified interface mode.
  *         - In DTR mode, addr and size_byte must both be even,
  *           otherwise MX25LM51245G_ERROR is returned
  * @retval error status
  */
mx25lm51245g_status_t mx25lm51245g_write_dma_async(mx25lm51245g_obj_t *pobj, uint32_t addr,
                                                   mx25lm51245g_addr_size_t addr_size,
                                                   uint8_t *pdata, uint32_t size_byte)
{
  mx25lm51245g_status_t ret = MX25LM51245G_OK;

  if ((pobj == NULL) || (pdata == NULL))
  {
    return MX25LM51245G_ERROR;
  }
  if ((pobj->transfer_rate == MX25LM51245G_DTR_TRANSFER)
      && (((size_byte & 1U) != 0U) || ((addr & 1U) != 0U)))
  {
    /* DTR mode transfers data by pairs: addr and size_byte must both be even */
    return MX25LM51245G_ERROR;
  }

  if (size_byte == 0U)
  {
    /* nothing to write */
  }
  else
  {
    if ((pobj->access_mode == MX25LM51245G_ACCESS_MMP)
        || (pobj->async_context.async_state != MX25LM51245G_ASYNC_OPERATION_NONE))
    {
      ret = MX25LM51245G_ERROR;
    }
    else
    {
      pobj->async_context.current_addr          = addr;
      pobj->async_context.transfer_buffer       = pdata;
      pobj->async_context.current_index         = 0U;
      pobj->async_context.total_size            = size_byte;
      pobj->async_context.current_transfer_len  = 0U;
      pobj->async_context.addr_size             = addr_size;
      pobj->async_context.async_state           = MX25LM51245G_ASYNC_OPERATION_WRITE;
      pobj->async_context.async_write_phase     = MX25LM51245G_ASYNC_WRITE_IDLE;
      HAL_XSPI_SetUserData(pobj->io.phxspi, pobj);
      /* Start async transfer */
      mx25lm51245g_async_tx_setup(pobj);
    }
  }
  return ret;
}

#endif /* MX25LM_CALLBACKS */
#endif /* USE_HAL_XSPI_DMA */

/* Memory mapped functions --------------------------------------------------*/

/**
  * @brief  Enables memory mapped mode for read operations from the OSPI memory.
  * @param  pobj      : Pointer to the part object structure
  * @param  addr_size : Address size
  * @retval error status
  */
mx25lm51245g_status_t mx25lm51245g_enable_memory_mapped(mx25lm51245g_obj_t *pobj, mx25lm51245g_addr_size_t addr_size)
{
  hal_xspi_memory_mapped_config_t s_mem_mapped_cfg;

  /* Check param */
  if (pobj == NULL)
  {
    return MX25LM51245G_ERROR;
  }

  /* OPI mode and 3-bytes address size not supported by memory */
  if ((pobj->interface == MX25LM51245G_OPI_MODE) && (addr_size == MX25LM51245G_3BYTES_SIZE))
  {
    return MX25LM51245G_ERROR;
  }

  /* Check Flash busy ? */
  if (mx25lm51245g_wip_autopolling(pobj) != MX25LM51245G_OK)
  {
    return MX25LM51245G_ERROR;
  }

  /* Write Operation must be Enabled before enabling memory mapped to be able to write data */
  if (mx25lm51245g_write_enable(pobj) != MX25LM51245G_OK)
  {
    return MX25LM51245G_ERROR;
  }

  if (mx25lm51245g_read_cmd(pobj, addr_size, 0U, 1U) != MX25LM51245G_OK)
  {
    return MX25LM51245G_ERROR;
  }

  /* Configure the memory mapped mode */
  s_mem_mapped_cfg.timeout_activation    = HAL_XSPI_TIMEOUT_DISABLE;
  s_mem_mapped_cfg.timeout_period_cycle  = 0x20;
  if (HAL_XSPI_StartMemoryMappedMode(pobj->io.phxspi, &s_mem_mapped_cfg) != HAL_OK)
  {
    return MX25LM51245G_ERROR;
  }
  pobj->access_mode = MX25LM51245G_ACCESS_MMP;

  return MX25LM51245G_OK;
}

/**
  * @brief  Disables memory mapped mode for the OSPI memory.
  * @param  pobj : Pointer to the part object structure
  * @retval error status
  */
mx25lm51245g_status_t mx25lm51245g_disable_memory_mapped(mx25lm51245g_obj_t *pobj)
{
  mx25lm51245g_status_t ret = MX25LM51245G_OK;

  /* Ensure that all the current transactions are finished using the synchronization barrier */
  __DMB();

  if (HAL_XSPI_StopMemoryMappedMode(pobj->io.phxspi) != HAL_OK)
  {
    ret = MX25LM51245G_ERROR;
  }
  else
  {
    /* Update object fields if all operations are well done */
    pobj->access_mode = MX25LM51245G_ACCESS_INDIRECT;
  }

  return ret;
}

/**
  * @brief  Gets the base address of the XSPI instance used.
  * @param  pobj : Pointer to the part object structure
  * @retval memory base address
  */
uint32_t mx25lm51245g_get_memory_mapped_base_address(mx25lm51245g_obj_t *pobj)
{
  return pobj->io.memory_base_address;
}

/* Memory power down functions --------------------------------------------------- */

/**
  * @brief  Enters the deep power-down mode.
  * @param  pobj : Pointer to the part object structure
  * @retval error status
  */
mx25lm51245g_status_t mx25lm51245g_enter_power_down(mx25lm51245g_obj_t *pobj)
{
  /* Check param */
  if (pobj == NULL)
  {
    return MX25LM51245G_ERROR;
  }

  /* Check the memory access mode */
  if (pobj->access_mode == MX25LM51245G_ACCESS_MMP)
  {
    return MX25LM51245G_ERROR;
  }

  uint32_t cmd;

  /* configure the enter deep power down cmd */
  if (pobj->interface == MX25LM51245G_SPI_MODE)
  {
    cmd = MX25LM51245G_ENTER_DEEP_POWER_DOWN_CMD;
  }
  else
  {
    cmd = MX25LM51245G_OCTA_ENTER_DEEP_POWER_DOWN_CMD;
  }

  /* send the enter deep power down cmd */
  if (mx25lm51245g_send_regular_cmd(pobj, HAL_XSPI_ADDR_NONE, HAL_XSPI_REGULAR_DATA_NONE, 0U, cmd,
                                    MX25LM51245G_DP_DUMMY_CYCLES) != MX25LM51245G_OK)
  {
    return MX25LM51245G_ERROR;
  }

  return MX25LM51245G_OK;
}

/**
  * @brief  Exits the deep power-down mode.
  * @param  pobj    : Pointer to the part object structure
  * @retval error status
  */
mx25lm51245g_status_t mx25lm51245g_exit_power_down(mx25lm51245g_obj_t *pobj)
{
  mx25lm51245g_status_t ret = MX25LM51245G_OK;
  /* Save the current configuration */
  mx25lm51245g_obj_t temp_pobj = *pobj;

  /* Wake-up the memory with a SW reset */
  if (mx25lm51245g_reset(pobj) != MX25LM51245G_OK)
  {
    ret = MX25LM51245G_ERROR;
  }
  else
  {
    /* Re-configure with the saved configuration*/
    if (mx25lm51245g_set_memconfig(pobj, temp_pobj.interface, temp_pobj.transfer_rate) != MX25LM51245G_OK)
    {
      ret = MX25LM51245G_ERROR;
    }
  }

  return ret;
}

/* Memory register read/write and control functions -------------------------------------- */

/**
  * @brief  Sends a Write Enable command and waits for it to take effect.
  * @param  pobj : Pointer to the part object structure
  * @retval error status
  */
static mx25lm51245g_status_t mx25lm51245g_write_enable(mx25lm51245g_obj_t *pobj)
{
  uint32_t cmd;

  /* configure the write enable cmd */
  if (pobj->interface == MX25LM51245G_SPI_MODE)
  {
    cmd = MX25LM51245G_WRITE_ENABLE_CMD;
  }
  else
  {
    cmd = MX25LM51245G_OCTA_WRITE_ENABLE_CMD;
  }

  /* send the write enable cmd */
  if (mx25lm51245g_send_regular_cmd(pobj, HAL_XSPI_ADDR_NONE, HAL_XSPI_REGULAR_DATA_NONE, 0U,
                                    cmd, MX25LM51245G_WREN_DUMMY_CYCLES) != MX25LM51245G_OK)
  {
    return MX25LM51245G_ERROR;
  }
  /* Check write enable bit */
  if (mx25lm51245g_wel_autopolling(pobj) != MX25LM51245G_OK)
  {
    return MX25LM51245G_ERROR;
  }

  return MX25LM51245G_OK;
}

/**
  * @brief  Reads the memory's status register value.
  * @param  pobj  : Pointer to the part object structure
  * @param  p_value : Status register value pointer
  * @retval error status
  */
mx25lm51245g_status_t mx25lm51245g_read_status_register(mx25lm51245g_obj_t *pobj, uint8_t *p_value)
{
  uint8_t reg_read[2];

  /* Check param */
  if ((pobj == NULL) || (p_value == NULL))
  {
    return MX25LM51245G_ERROR;
  }

  /* Check the memory access mode */
  if (pobj->access_mode == MX25LM51245G_ACCESS_MMP)
  {
    return MX25LM51245G_ERROR;
  }

  /* Send the Read status command */
  if (mx25lm51245g_send_read_status_reg_cmd(pobj) != MX25LM51245G_OK)
  {
    return MX25LM51245G_ERROR;
  }

  /* Reception data */
  if (HAL_XSPI_Receive(pobj->io.phxspi, reg_read, MX25LM51245G_CMD_TIMEOUT_VALUE) != HAL_OK)
  {
    return MX25LM51245G_ERROR;
  }

  *p_value = reg_read[0];

  return MX25LM51245G_OK;
}

/**
  * @brief  Read Flash Security register value.
  * @param  pobj    : Pointer to the part object structure
  * @param  p_value : Security register value pointer
  * @retval error status
  */
mx25lm51245g_status_t mx25lm51245g_read_security_register(mx25lm51245g_obj_t *pobj, uint8_t *p_value)
{
  uint8_t reg_read[2];

  /* Check param */
  if ((pobj == NULL) || (p_value == NULL))
  {
    return MX25LM51245G_ERROR;
  }

  /* Check the memory access mode */
  if (pobj->access_mode == MX25LM51245G_ACCESS_MMP)
  {
    return MX25LM51245G_ERROR;
  }
  uint32_t cmd;
  hal_xspi_regular_data_mode_t data_mode;
  hal_xspi_addr_mode_t addr_mode;
  uint32_t dummy_cycle;

  /* configure the params for the read security register cmd */
  if (pobj->interface == MX25LM51245G_SPI_MODE)
  {
    cmd         = MX25LM51245G_READ_SECURITY_REG_CMD;
    data_mode   = HAL_XSPI_REGULAR_DATA_1LINE;
    dummy_cycle = MX25LM51245G_RDSCUR_DUMMY_CYCLES;
    addr_mode   = HAL_XSPI_ADDR_NONE;
  }
  else
  {
    cmd         = MX25LM51245G_OCTA_READ_SECURITY_REG_CMD;
    data_mode   = HAL_XSPI_REGULAR_DATA_8LINES;
    dummy_cycle = MX25LM51245G_OPI_RDSCUR_DUMMY_CYCLES;
    addr_mode   = HAL_XSPI_ADDR_8LINES;
  }

  /* send the read security register cmd */
  if (mx25lm51245g_send_regular_cmd(pobj, addr_mode, data_mode, 0U, cmd, dummy_cycle) != MX25LM51245G_OK)
  {
    return MX25LM51245G_ERROR;
  }

  /* Reception data */
  if (HAL_XSPI_Receive(pobj->io.phxspi, reg_read, MX25LM51245G_CMD_TIMEOUT_VALUE) != HAL_OK)
  {
    return MX25LM51245G_ERROR;
  }

  *p_value = reg_read[0];

  return MX25LM51245G_OK;
}

/**
  * @brief  Read Flash configuration register value.
  * @param  pobj         : Part object pointer.
  * @param  pvalue       : Configuration register value pointer.
  * @retval error status.
  */
static mx25lm51245g_status_t mx25lm51245g_read_cfg_register(mx25lm51245g_obj_t *pobj, uint8_t *pvalue)
{
  uint8_t reg_read[2];
  hal_xspi_addr_mode_t addr_mode;
  hal_xspi_regular_data_mode_t data_mode;
  uint32_t cmd;
  uint32_t read_addr;
  uint32_t dummy_cycle;

  /* configure the params for the read configuration register cmd */
  if (pobj->interface == MX25LM51245G_SPI_MODE)
  {
    cmd         = MX25LM51245G_READ_CFG_REG_CMD;
    addr_mode   = HAL_XSPI_ADDR_NONE;
    data_mode   = HAL_XSPI_REGULAR_DATA_1LINE;
    read_addr   = 0U;
    dummy_cycle = MX25LM51245G_RDCR_DUMMY_CYCLES;
  }
  else
  {
    cmd         = MX25LM51245G_OCTA_READ_CFG_REG_CMD;
    addr_mode   = HAL_XSPI_ADDR_8LINES;
    data_mode   = HAL_XSPI_REGULAR_DATA_8LINES;
    read_addr   = 1U;
    dummy_cycle = MX25LM51245G_OPI_RDCR_DUMMY_CYCLES;
  }

  /* send the read configuration register cmd */
  if (mx25lm51245g_send_regular_cmd(pobj, addr_mode, data_mode, read_addr, cmd,
                                    dummy_cycle) != MX25LM51245G_OK)
  {
    return MX25LM51245G_ERROR;
  }

  /* Reception data */
  if (HAL_XSPI_Receive(pobj->io.phxspi, reg_read, MX25LM51245G_CMD_TIMEOUT_VALUE) != HAL_OK)
  {
    return MX25LM51245G_ERROR;
  }

  *pvalue = reg_read[0];

  return MX25LM51245G_OK;
}

/**
  * @brief  Read Flash configuration register 2 value.
  * @param  pobj      : Pointer to the part object structure
  * @param  read_addr : Register address to read (REG1 to REG7)
  * @param  p_value   : Configuration register 2 value pointer
  * @retval error status
  */
static mx25lm51245g_status_t mx25lm51245g_read_cfg2_register(mx25lm51245g_obj_t *pobj, uint32_t read_addr,
                                                             uint8_t *p_value)
{
  uint8_t reg_read[2];
  hal_xspi_addr_mode_t addr_mode;
  hal_xspi_regular_data_mode_t data_mode;
  uint32_t cmd;
  uint32_t dummy_cycle;

  /* configure the params for the read configuration register 2 cmd */
  if (pobj->interface == MX25LM51245G_SPI_MODE)
  {
    cmd         = MX25LM51245G_READ_CFG_REG2_CMD;
    addr_mode   = HAL_XSPI_ADDR_1LINE;
    data_mode   = HAL_XSPI_REGULAR_DATA_1LINE;
    dummy_cycle = MX25LM51245G_RDCR2_DUMMY_CYCLES;
  }
  else
  {
    cmd         = MX25LM51245G_OCTA_READ_CFG_REG2_CMD;
    addr_mode   = HAL_XSPI_ADDR_8LINES;
    data_mode   = HAL_XSPI_REGULAR_DATA_8LINES;
    dummy_cycle = MX25LM51245G_OPI_RDCR2_DUMMY_CYCLES;
  }

  /* send the read configuration register 2 cmd */
  if (mx25lm51245g_send_regular_cmd(pobj, addr_mode, data_mode, read_addr, cmd, dummy_cycle) != MX25LM51245G_OK)
  {
    return MX25LM51245G_ERROR;
  }

  /* Reception data */
  if (HAL_XSPI_Receive(pobj->io.phxspi, reg_read, MX25LM51245G_CMD_TIMEOUT_VALUE) != HAL_OK)
  {
    return MX25LM51245G_ERROR;
  }

  *p_value = reg_read[0];

  return MX25LM51245G_OK;
}

/**
  * @brief  Write Flash configuration register.
  * @param  pobj         : Part object pointer.
  * @param  value        : Value to write to the status register.
  * @retval error status.
  */
static mx25lm51245g_status_t mx25lm51245g_write_cfg_register(mx25lm51245g_obj_t *pobj, uint8_t value)
{
  hal_xspi_regular_cmd_t s_command;
  uint8_t reg[2] = {0U};

  /* In SPI mode,  the status register is read first so its value can be preserved
   * when programming the configuration register.
   */
  if (pobj->interface == MX25LM51245G_SPI_MODE)
  {
    if (mx25lm51245g_read_status_register(pobj, reg) != MX25LM51245G_OK)
    {
      return MX25LM51245G_ERROR;
    }
    /* The new configuration register value to be set - 2nd byte to be sent */
    reg[1] = value;
  }


  /* The new configuration register value to be set */
  reg[1] = value;
  /* Write enable is required for writing to the configuration register */
  if (mx25lm51245g_write_enable(pobj) != MX25LM51245G_OK)
  {
    return MX25LM51245G_ERROR;
  }

  /* Initialize the command */
  /* Common configuration for all modes */
  /* Set memory operation type */
  s_command.operation_type                  = HAL_XSPI_OPERATION_COMMON_CFG;
  /* Select IOs to be used */
  s_command.io_select                       = HAL_XSPI_IO_7_0;
  /* Address mode to be used */
  s_command.addr_mode                       = HAL_XSPI_ADDR_NONE;
  /* Set memory address width  */
  s_command.addr_width                      = HAL_XSPI_ADDR_8BIT;
  /* Set read/write address */
  s_command.addr                            = 0U;
  /* Set memory address DTR mode  */
  s_command.addr_dtr_mode_status            = HAL_XSPI_ADDR_DTR_DISABLED;
  /* Set memory alternate bytes       */
  s_command.alternate_bytes                 = 1U;
  /* Set memory DQS mode  */
  s_command.dqs_mode_status                 = HAL_XSPI_DQS_DISABLED;
  /* Set memory dummy cycles  */
  s_command.dummy_cycle                     = MX25LM51245G_WRITE_STATUS_DUMMY_CYCLES;

  switch (pobj->interface)
  {
    case MX25LM51245G_SPI_MODE:
      /* Specific configuration for SPI */
      /* Set memory alternate bytes width  */
      s_command.alternate_bytes_width           = HAL_XSPI_ALTERNATE_BYTES_8BIT;
      /* Set memory alternate bytes DTR mode  */
      s_command.alternate_bytes_dtr_mode_status = HAL_XSPI_ALTERNATE_BYTES_DTR_DISABLED;
      /* Set memory alternate bytes mode  */
      s_command.alternate_bytes_mode            = HAL_XSPI_ALTERNATE_BYTES_NONE;
      /* Set memory instruction mode */
      s_command.instruction_mode                = HAL_XSPI_INSTRUCTION_1LINE;
      /* Set memory Double Transfer Rate mode */
      s_command.instruction_dtr_mode_status     = HAL_XSPI_INSTRUCTION_DTR_DISABLED;
      /* Set memory instruction width */
      s_command.instruction_width               = HAL_XSPI_INSTRUCTION_8BIT;
      /* Set memory instruction  */
      s_command.instruction                     = MX25LM51245G_WRITE_CFG_REG_CMD;
      /* Set memory data mode  */
      s_command.data_mode                       = HAL_XSPI_REGULAR_DATA_1LINE;
      /* Set memory data DTR mode  */
      s_command.data_dtr_mode_status            = HAL_XSPI_DATA_DTR_DISABLED;
      /* Set memory size byte  */
      s_command.size_byte                       = 2U;
      break;

    case MX25LM51245G_OPI_MODE:
      /* Common configuration for OPI */
      /* Set memory instruction mode */
      s_command.instruction_mode      = HAL_XSPI_INSTRUCTION_8LINES;
      /* Set memory instruction width */
      s_command.instruction_width     = HAL_XSPI_INSTRUCTION_16BIT;
      /* Set memory instruction  */
      s_command.instruction           = MX25LM51245G_OCTA_WRITE_CFG_REG_CMD;
      /* Set memory addressing mode  */
      s_command.addr_mode             = HAL_XSPI_ADDR_NONE;
      /* Set memory data mode  */
      s_command.data_mode             = HAL_XSPI_REGULAR_DATA_8LINES;
      /* Set memory alternate bytes mode  */
      s_command.alternate_bytes_mode  = HAL_XSPI_ALTERNATE_BYTES_8LINES;

      if (pobj->transfer_rate == MX25LM51245G_DTR_TRANSFER)
      {
        /* Specific configuration for DOPI */
        /* In DTR mode, the minimum size byte value is 2 */
        s_command.size_byte                       = 2U;
        /* In DTR mode, the alternate bytes width is set to 24 bits.
         * Since the minimum size byte is 2, the last alternate byte
         * is transmitted as the first byte in the data phase.
         */
        s_command.alternate_bytes_width           = HAL_XSPI_ALTERNATE_BYTES_24BIT;

        /* The last alternate byte is now sent as the first byte in the data phase */
        reg[0] = 0U;
        /* The last byte to be sent contains the new configuration register value */
        reg[1] = value;

        /* Set memory alternate bytes DTR mode  */
        s_command.alternate_bytes_dtr_mode_status = HAL_XSPI_ALTERNATE_BYTES_DTR_ENABLED;
        /* Set memory Double Transfer Rate mode */
        s_command.instruction_dtr_mode_status     = HAL_XSPI_INSTRUCTION_DTR_ENABLED;
        /* Set memory data DTR mode  */
        s_command.data_dtr_mode_status            = HAL_XSPI_DATA_DTR_ENABLED;
      }
      else
      {
        /* Specific configuration for SOPI */
        /* In OPI-STR, only a single byte is sent in the data phase. */
        reg[0] = value;
        /* Set memory alternate bytes width to 32 bits to cover 4 bytes in OPI-STR */
        s_command.alternate_bytes_width           = HAL_XSPI_ALTERNATE_BYTES_32BIT;
        /* Set memory alternate bytes DTR mode  */
        s_command.alternate_bytes_dtr_mode_status = HAL_XSPI_ALTERNATE_BYTES_DTR_DISABLED;
        /* Set memory Double Transfer Rate mode */
        s_command.instruction_dtr_mode_status = HAL_XSPI_INSTRUCTION_DTR_DISABLED;
        /* Set memory data DTR mode  */
        s_command.data_dtr_mode_status        = HAL_XSPI_DATA_DTR_DISABLED;
        /* Set memory size byte  */
        s_command.size_byte                   = 1U;
      }
      break;

    default:
      /* Unsupported interface */
      return MX25LM51245G_ERROR;
      break;
  }

  /* Send the command */
  if (HAL_XSPI_SendRegularCmd(pobj->io.phxspi, &s_command, MX25LM51245G_CMD_TIMEOUT_VALUE) != HAL_OK)
  {
    return MX25LM51245G_ERROR;
  }

  /* Send the configuration data */
  if (HAL_XSPI_Transmit(pobj->io.phxspi, reg, MX25LM51245G_CMD_TIMEOUT_VALUE) != HAL_OK)
  {
    return MX25LM51245G_ERROR;
  }

  /* Write Status Register Cycle Time (tWmax = ~40ms) */
  if (mx25lm51245g_wip_autopolling(pobj) != MX25LM51245G_OK)
  {
    return MX25LM51245G_ERROR;
  }

  return MX25LM51245G_OK;
}

/**
  * @brief  Write Flash configuration register 2.
  * @param  pobj       : Pointer to the part object structure
  * @param  write_addr : Address to write to
  * @param  value      : Value to write to configuration register
  * @retval error status
  */
static mx25lm51245g_status_t mx25lm51245g_write_cfg2_register(mx25lm51245g_obj_t *pobj, uint32_t write_addr,
                                                              uint8_t value)
{
  hal_xspi_addr_mode_t addr_mode;
  hal_xspi_regular_data_mode_t data_mode;
  uint32_t cmd;

  /* configure the params for the write configuration register 2 cmd */
  if (pobj->interface == MX25LM51245G_SPI_MODE)
  {
    cmd       = MX25LM51245G_WRITE_CFG_REG2_CMD;
    addr_mode = HAL_XSPI_ADDR_1LINE;
    data_mode = HAL_XSPI_REGULAR_DATA_1LINE;
  }
  else
  {
    cmd       = MX25LM51245G_OCTA_WRITE_CFG_REG2_CMD;
    addr_mode = HAL_XSPI_ADDR_8LINES;
    data_mode = HAL_XSPI_REGULAR_DATA_8LINES;
  }

  /* send the write configuration register 2 cmd */
  if (mx25lm51245g_send_regular_cmd(pobj, addr_mode, data_mode, write_addr, cmd,
                                    MX25LM51245G_WRCR2_DUMMY_CYCLES) != MX25LM51245G_OK)
  {
    return MX25LM51245G_ERROR;
  }

  /* Transmit data */
  if (HAL_XSPI_Transmit(pobj->io.phxspi, &value, MX25LM51245G_CMD_TIMEOUT_VALUE) != HAL_OK)
  {
    return MX25LM51245G_ERROR;
  }

  return MX25LM51245G_OK;
}

/**
  * @brief  Polling WIP(Write In Progress) bit become to 0.
  * @param  pobj : Pointer to the part object structure
  * @retval error status
  */
static mx25lm51245g_status_t mx25lm51245g_wip_autopolling(mx25lm51245g_obj_t *pobj)
{
  hal_xspi_auto_polling_config_t s_config;

  /* Send Command */
  if (mx25lm51245g_send_read_status_reg_cmd(pobj) != MX25LM51245G_OK)
  {
    return MX25LM51245G_ERROR;
  }

  /* Configure and start WIP autopolling - start data phase */
  s_config.match_value           = 0U;
  s_config.match_mask            = MX25LM51245G_SR_WIP;
  s_config.match_mode            = HAL_XSPI_MATCH_MODE_AND;
  s_config.interval_cycle        = MX25LM51245G_AUTOPOLLING_INTERVAL_TIME;
  s_config.automatic_stop_status = HAL_XSPI_AUTOMATIC_STOP_ENABLED;

  if (HAL_XSPI_ExecRegularAutoPoll(pobj->io.phxspi, &s_config, MX25LM51245G_CMD_TIMEOUT_VALUE) != HAL_OK)
  {
    return MX25LM51245G_ERROR;
  }

  return MX25LM51245G_OK;
}

/**
  * @brief  Check write enable latch (WEL) status.
  * @param  phxspi   xspi handle
  */
static mx25lm51245g_status_t mx25lm51245g_wel_autopolling(mx25lm51245g_obj_t *pobj)
{
  hal_xspi_auto_polling_config_t s_config;

  /* Send Command */
  if (mx25lm51245g_send_read_status_reg_cmd(pobj) != MX25LM51245G_OK)
  {
    return MX25LM51245G_ERROR;
  }
  s_config.match_value           = 2U;
  s_config.match_mask            = MX25LM51245G_SR_WEL;
  s_config.match_mode            = HAL_XSPI_MATCH_MODE_AND;
  s_config.interval_cycle        = MX25LM51245G_AUTOPOLLING_INTERVAL_TIME;
  s_config.automatic_stop_status = HAL_XSPI_AUTOMATIC_STOP_ENABLED;

  if (HAL_XSPI_ExecRegularAutoPoll(pobj->io.phxspi, &s_config, MX25LM51245G_CMD_TIMEOUT_VALUE) != HAL_OK)
  {
    return MX25LM51245G_ERROR;
  }

  return MX25LM51245G_OK;
}

/* Memory send command functions ------------------------------------------- */

/**
  * @brief  Writes an amount of data to the OSPI memory using the Page Program command.
  * @param  pobj       : Pointer to the part object structure
  * @param  addr_size  : Address size
  * @param  write_addr : Write start address
  * @param  size_byte  : Size of data to write in bytes (Range 1 ~ MX25LM51245G_PAGE_SIZE).
  * @note   Address size is forced to 3 Bytes when the 4 Bytes address size
  *         command is not available for the specified interface mode
  * @retval error status
  */
static mx25lm51245g_status_t mx25lm51245g_write_cmd(mx25lm51245g_obj_t *pobj, mx25lm51245g_addr_size_t addr_size,
                                                    uint32_t write_addr, uint32_t size_byte)
{
  mx25lm51245g_status_t ret = MX25LM51245G_OK;
  hal_xspi_regular_cmd_t s_command;

  /* OPI mode and 3-bytes address size not supported by memory */
  if ((pobj->interface == MX25LM51245G_OPI_MODE) && (addr_size == MX25LM51245G_3BYTES_SIZE))
  {
    ret = MX25LM51245G_ERROR;
  }
  else
  {
    /* Initialize the program command */
    /* Set memory operation type */
    s_command.operation_type         = HAL_XSPI_OPERATION_COMMON_CFG;
    /* Select IOs to be used */
    s_command.io_select              = HAL_XSPI_IO_7_0;
    /* Set read/write address */
    s_command.addr                   = write_addr;
    /* Set memory size byte  */
    s_command.size_byte              = size_byte;
    /* Set memory dummy cycles  */
    s_command.dummy_cycle            = MX25LM51245G_PAGE_PROG_DUMMY_CYCLES;
    /* Set memory DQS mode  */
    s_command.dqs_mode_status               = HAL_XSPI_DQS_DISABLED;
    /* Set memory alternate bytes mode  */
    s_command.alternate_bytes_mode   = HAL_XSPI_ALTERNATE_BYTES_NONE;
    /* Set memory alternate bytes       */
    s_command.alternate_bytes        = 0U;
    /* Set memory alternate bytes width  */
    s_command.alternate_bytes_width  = HAL_XSPI_ALTERNATE_BYTES_8BIT;
    /* Set memory alternate bytes DTR mode  */
    s_command.alternate_bytes_dtr_mode_status =  HAL_XSPI_ALTERNATE_BYTES_DTR_DISABLED;
    switch (pobj->interface)
    {
      case MX25LM51245G_SPI_MODE:
        /* Set memory instruction mode */
        s_command.instruction_mode     = HAL_XSPI_INSTRUCTION_1LINE;
        /* Set memory Double Transfer Rate mode */
        s_command.instruction_dtr_mode_status = HAL_XSPI_INSTRUCTION_DTR_DISABLED;
        /* Set memory instruction width */
        s_command.instruction_width    = HAL_XSPI_INSTRUCTION_8BIT;
        /* Set memory instruction  */
        s_command.instruction          = (addr_size == MX25LM51245G_3BYTES_SIZE)
                                         ? MX25LM51245G_PAGE_PROG_CMD
                                         : MX25LM51245G_4_BYTE_PAGE_PROG_CMD;
        /* Set memory addressing mode  */
        s_command.addr_mode            = HAL_XSPI_ADDR_1LINE;
        /* Set memory address DTR mode  */
        s_command.addr_dtr_mode_status = HAL_XSPI_ADDR_DTR_DISABLED;
        /* Set memory address width  */
        s_command.addr_width           = (addr_size == MX25LM51245G_3BYTES_SIZE)
                                         ? HAL_XSPI_ADDR_24BIT
                                         : HAL_XSPI_ADDR_32BIT;
        /* Set memory data mode  */
        s_command.data_mode            =  HAL_XSPI_REGULAR_DATA_1LINE;
        /* Set memory data DTR mode  */
        s_command.data_dtr_mode_status        = HAL_XSPI_DATA_DTR_DISABLED;
        break;
      case MX25LM51245G_OPI_MODE:
        /* Common configuration for OPI mode */
        /* Set memory instruction mode */
        s_command.instruction_mode     = HAL_XSPI_INSTRUCTION_8LINES;
        /* Set memory instruction width */
        s_command.instruction_width    = HAL_XSPI_INSTRUCTION_16BIT;
        /* Set memory addressing mode  */
        s_command.addr_mode            = HAL_XSPI_ADDR_8LINES;
        /* Set memory data mode  */
        s_command.data_mode            = HAL_XSPI_REGULAR_DATA_8LINES;

        if (pobj->transfer_rate == MX25LM51245G_DTR_TRANSFER)
        {
          /* Set memory Double Transfer Rate mode */
          s_command.instruction_dtr_mode_status = HAL_XSPI_INSTRUCTION_DTR_ENABLED;
          /* Set memory instruction  */
          s_command.instruction          = MX25LM51245G_OCTA_PAGE_PROG_CMD;
          /* Set memory address DTR mode  */
          s_command.addr_dtr_mode_status        = HAL_XSPI_ADDR_DTR_ENABLED;
          /* Set memory data DTR mode  */
          s_command.data_dtr_mode_status        = HAL_XSPI_DATA_DTR_ENABLED;
          /* Set memory address width  */
          s_command.addr_width                  = HAL_XSPI_ADDR_32BIT;
        }
        else
        {
          /* Set memory Double Transfer Rate mode */
          s_command.instruction_dtr_mode_status = HAL_XSPI_INSTRUCTION_DTR_DISABLED;

          /* Set memory instruction  */
          s_command.instruction          = MX25LM51245G_OCTA_PAGE_PROG_CMD;
          /* Set memory address DTR mode  */
          s_command.addr_dtr_mode_status        = HAL_XSPI_ADDR_DTR_DISABLED;
          /* Set memory data DTR mode  */
          s_command.data_dtr_mode_status        = HAL_XSPI_DATA_DTR_DISABLED;
          /* Set memory address width  */
          s_command.addr_width                  = HAL_XSPI_ADDR_32BIT;
        }
        break;
      default:
        ret = MX25LM51245G_ERROR;
        break;
    }
    if (ret == MX25LM51245G_OK)
    {
      /* Configure the command */
      if (HAL_XSPI_SendRegularCmd(pobj->io.phxspi, &s_command, MX25LM51245G_CMD_TIMEOUT_VALUE) != HAL_OK)
      {
        ret = MX25LM51245G_ERROR;
      }
    }
  }

  return ret;
}

/**
  * @brief  Reads an amount of data from the OSPI memory.
  * @param  pobj      : Pointer to the part object structure
  * @param  addr_size : Address size
  * @param  read_addr : Read start address
  * @param  size_byte : Size of data to read in bytes
  * @retval error status
  */
static mx25lm51245g_status_t mx25lm51245g_read_cmd(mx25lm51245g_obj_t *pobj, mx25lm51245g_addr_size_t addr_size,
                                                   uint32_t read_addr, uint32_t size_byte)
{
  mx25lm51245g_status_t ret = MX25LM51245G_OK;
  hal_xspi_regular_cmd_t s_command;
  uint8_t lut_index = 0U;

  /* OPI mode and 3-bytes address size not supported by memory */
  if ((pobj->interface == MX25LM51245G_OPI_MODE) && (addr_size == MX25LM51245G_3BYTES_SIZE))
  {
    ret = MX25LM51245G_ERROR;
  }
  else
  {
    /* Initialize the read command */
    /* Set memory operation type */
    s_command.operation_type        = HAL_XSPI_OPERATION_COMMON_CFG;
    /* Select IOs to be used */
    s_command.io_select             = HAL_XSPI_IO_7_0;
    /* Set read/write address */
    s_command.addr                  = read_addr;
    /* Set memory alternate bytes mode  */
    s_command.alternate_bytes_mode  = HAL_XSPI_ALTERNATE_BYTES_NONE;
    /* Set memory alternate bytes       */
    s_command.alternate_bytes       = 0U;
    /* Set memory alternate bytes width  */
    s_command.alternate_bytes_width = HAL_XSPI_ALTERNATE_BYTES_8BIT;
    /* Set memory alternate bytes DTR mode  */
    s_command.alternate_bytes_dtr_mode_status =  HAL_XSPI_ALTERNATE_BYTES_DTR_DISABLED;
    /* Set memory size byte  */
    s_command.size_byte            = size_byte;
    switch (pobj->interface)
    {
      case MX25LM51245G_SPI_MODE:
        /* Set memory instruction mode */
        s_command.instruction_mode     = HAL_XSPI_INSTRUCTION_1LINE;
        /* Set memory Double Transfer Rate mode */
        s_command.instruction_dtr_mode_status = HAL_XSPI_INSTRUCTION_DTR_DISABLED;
        /* Set memory instruction width */
        s_command.instruction_width    = HAL_XSPI_INSTRUCTION_8BIT;
        /* Set memory instruction  */
        s_command.instruction          = (addr_size == MX25LM51245G_3BYTES_SIZE)
                                         ? MX25LM51245G_FAST_READ_CMD
                                         : MX25LM51245G_4_BYTE_ADDR_FAST_READ_CMD;
        /* Set memory addressing mode  */
        s_command.addr_mode            = HAL_XSPI_ADDR_1LINE;
        /* Set memory address DTR mode  */
        s_command.addr_dtr_mode_status        = HAL_XSPI_ADDR_DTR_DISABLED;
        /* Set memory address width  */
        s_command.addr_width           = (addr_size == MX25LM51245G_3BYTES_SIZE)
                                         ? HAL_XSPI_ADDR_24BIT
                                         : HAL_XSPI_ADDR_32BIT;
        /* Set memory data mode  */
        s_command.data_mode            = HAL_XSPI_REGULAR_DATA_1LINE;
        /* Set memory data DTR mode  */
        s_command.data_dtr_mode_status        = HAL_XSPI_DATA_DTR_DISABLED;
        /* Set memory dummy cycles  */
        s_command.dummy_cycle          = MX25LM51245G_FAST_READ_DUMMY_CYCLES;
        /* Set memory DQS mode  */
        s_command.dqs_mode_status             = HAL_XSPI_DQS_DISABLED;
        break;

      case MX25LM51245G_OPI_MODE:
        /* Retrieve the dummy cycles lookup table index based on the XSPI frequency */
        /* This index is used to reference the corresponding command parameter and dummy cycles register values */
        lut_index = (uint8_t)mx25lm51245g_get_lookup_dummy_cycles_index(pobj);

        /* Update the dummy cycles register only if the frequency index has changed */
        if (lut_index != pobj->dc_lut_index)
        {
          if (mx25lm51245g_update_dc_reg(pobj, lut_index) != MX25LM51245G_OK)
          {
            return MX25LM51245G_ERROR;
          }
        }
        /* Common configuration for OPI Mode */
        /* Set memory instruction mode */
        s_command.instruction_mode     = HAL_XSPI_INSTRUCTION_8LINES;
        /* Set memory instruction width */
        s_command.instruction_width    = HAL_XSPI_INSTRUCTION_16BIT;
        /* Set memory addressing mode  */
        s_command.addr_mode            = HAL_XSPI_ADDR_8LINES;
        /* Set memory data mode  */
        s_command.data_mode            = HAL_XSPI_REGULAR_DATA_8LINES;
        /* Set the dummy cycle parameter based on the XSPI frequency
           by referring to the mx25lm51245g_dc_freq_lut table */
        /* This value corresponds to the XSPI register dummy cycles */
        s_command.dummy_cycle          = mx25lm51245g_dc_freq_lut[lut_index].dc_cmd_value;

        if (pobj->transfer_rate == MX25LM51245G_DTR_TRANSFER)
        {

          /* Set memory Double Transfer Rate mode */
          s_command.instruction_dtr_mode_status = HAL_XSPI_INSTRUCTION_DTR_ENABLED;
          /* Set memory instruction  */
          s_command.instruction          = MX25LM51245G_OCTA_READ_DTR_CMD;
          /* Set memory address DTR mode  */
          s_command.addr_dtr_mode_status        = HAL_XSPI_ADDR_DTR_ENABLED;
          /* Set memory address width  */
          s_command.addr_width           = HAL_XSPI_ADDR_32BIT;
          /* Set memory data DTR mode  */
          s_command.data_dtr_mode_status        = HAL_XSPI_DATA_DTR_ENABLED;
          /* Set memory DQS mode  */
          s_command.dqs_mode_status             = HAL_XSPI_DQS_ENABLED;
        }
        else
        {
          /* Set memory Double Transfer Rate mode */
          s_command.instruction_dtr_mode_status = HAL_XSPI_INSTRUCTION_DTR_DISABLED;

          /* Set memory instruction  */
          s_command.instruction          = MX25LM51245G_OCTA_READ_CMD;
          /* Set memory address DTR mode  */
          s_command.addr_dtr_mode_status        = HAL_XSPI_ADDR_DTR_DISABLED;
          /* Set memory address width  */
          s_command.addr_width           = HAL_XSPI_ADDR_32BIT;
          /* Set memory data DTR mode  */
          s_command.data_dtr_mode_status        = HAL_XSPI_DATA_DTR_DISABLED;
          /* Set memory DQS mode  */
          s_command.dqs_mode_status             = HAL_XSPI_DQS_DISABLED;
        }
        break;
      default:
        ret = MX25LM51245G_ERROR;
        break;
    }
    if (ret == MX25LM51245G_OK)
    {
      /* Send the command */
      if (HAL_XSPI_SendRegularCmd(pobj->io.phxspi, &s_command, MX25LM51245G_CMD_TIMEOUT_VALUE) != HAL_OK)
      {
        ret = MX25LM51245G_ERROR;
      }
    }
  }

  return ret;
}

/**
  * @brief  Send sector erase command ID.
  * @param  pobj        : Part object pointer.
  * @param  sector_addr : start address.
  * @retval error status.
  */
static mx25lm51245g_status_t mx25lm51245g_send_sector_erase_cmd(mx25lm51245g_obj_t *pobj, uint32_t sector_addr)
{
  uint32_t cmd;
  hal_xspi_addr_mode_t addr_mode;

  /* configure the sector erase cmd */
  if (pobj->interface == MX25LM51245G_SPI_MODE)
  {
    cmd = MX25LM51245G_4_BYTE_SECTOR_ERASE_4K_CMD;
    addr_mode = HAL_XSPI_ADDR_1LINE;
  }
  else
  {
    cmd = MX25LM51245G_OCTA_SECTOR_ERASE_4K_CMD;
    addr_mode = HAL_XSPI_ADDR_8LINES;
  }

  /* send the sector erase cmd */
  if (mx25lm51245g_send_regular_cmd(pobj, addr_mode, HAL_XSPI_REGULAR_DATA_NONE, sector_addr, cmd,
                                    MX25LM51245G_SE_DUMMY_CYCLES) != MX25LM51245G_OK)
  {
    return MX25LM51245G_ERROR;
  }

  /* Return error status */
  return MX25LM51245G_OK;
}


/**
  * @brief  Send block erase command ID.
  * @param  pobj       : Part object pointer.
  * @param  block_addr : start address.
  * @retval error status.
  */
static mx25lm51245g_status_t mx25lm51245g_send_block_erase_cmd(mx25lm51245g_obj_t *pobj, uint32_t block_addr)
{
  uint32_t cmd;
  hal_xspi_addr_mode_t addr_mode;

  /* configure the block erase cmd */
  if (pobj->interface == MX25LM51245G_SPI_MODE)
  {
    cmd = MX25LM51245G_4_BYTE_BLOCK_ERASE_64K_CMD;
    addr_mode = HAL_XSPI_ADDR_1LINE;
  }
  else
  {
    cmd = MX25LM51245G_OCTA_BLOCK_ERASE_64K_CMD;
    addr_mode = HAL_XSPI_ADDR_8LINES;
  }

  /* send the block erase cmd */
  if (mx25lm51245g_send_regular_cmd(pobj, addr_mode, HAL_XSPI_REGULAR_DATA_NONE, block_addr, cmd,
                                    MX25LM51245G_BE_DUMMY_CYCLES) != MX25LM51245G_OK)
  {
    return MX25LM51245G_ERROR;
  }

  /* Return error status */
  return MX25LM51245G_OK;
}

/**
  * @brief  Send read status register command ID.
  * @param  pobj       : Part object pointer.
  * @retval error status.
  */
static mx25lm51245g_status_t mx25lm51245g_send_read_status_reg_cmd(mx25lm51245g_obj_t *pobj)
{
  hal_xspi_addr_mode_t addr_mode;
  hal_xspi_regular_data_mode_t data_mode;
  uint32_t cmd;
  uint32_t dummy_cycle;

  /* configure the params for the read status register cmd */
  if (pobj->interface == MX25LM51245G_SPI_MODE)
  {
    cmd = MX25LM51245G_READ_STATUS_REG_CMD;
    addr_mode = HAL_XSPI_ADDR_NONE;
    data_mode = HAL_XSPI_REGULAR_DATA_1LINE;
    dummy_cycle = MX25LM51245G_RDSR_DUMMY_CYCLES;
  }
  else
  {
    cmd = MX25LM51245G_OCTA_READ_STATUS_REG_CMD;
    addr_mode = HAL_XSPI_ADDR_8LINES;
    data_mode = HAL_XSPI_REGULAR_DATA_8LINES;
    dummy_cycle = MX25LM51245G_OPI_RDSR_DUMMY_CYCLES;
  }

  /* send read status register cmd */
  if (mx25lm51245g_send_regular_cmd(pobj, addr_mode, data_mode, 0U, cmd, dummy_cycle) != MX25LM51245G_OK)
  {
    return MX25LM51245G_ERROR;
  }
  /* Return error status */
  return MX25LM51245G_OK;
}

/**
  * @brief  Sends a regular cmd through the XSPI bus.
  * @param  pobj        : Part object pointer.
  * @param  addr_mode   : Precises number of lines for address .
  * @param  addr        : Contains the address to be sent to the device.
  * @param  data_mode   : Precises number of lines for data exchange.
  * @param  cmd         : Contains the instruction to be sent to the device.
  * @param  dummy_cycle : Contains the number of dummy cycles inserted before data phase.
  * @retval error status.
  */
static mx25lm51245g_status_t mx25lm51245g_send_regular_cmd(mx25lm51245g_obj_t *pobj, hal_xspi_addr_mode_t addr_mode,
                                                           hal_xspi_regular_data_mode_t data_mode, uint32_t addr,
                                                           uint32_t cmd, uint32_t dummy_cycle)
{
  mx25lm51245g_status_t ret = MX25LM51245G_OK;
  hal_xspi_regular_cmd_t s_command;

  /* Initialize the command */
  /* Set memory operation type */
  s_command.operation_type                  = HAL_XSPI_OPERATION_COMMON_CFG;
  /* Select IOs to be used */
  s_command.io_select                       = HAL_XSPI_IO_7_0;
  /* Set memory address width  */
  s_command.addr_width                      = HAL_XSPI_ADDR_32BIT;
  /* Set read/write address */
  s_command.addr                            = addr;
  /* Set memory alternate bytes mode  */
  s_command.alternate_bytes_mode            = HAL_XSPI_ALTERNATE_BYTES_NONE;
  /* Set memory alternate bytes       */
  s_command.alternate_bytes                 = 0U;
  /* Set memory alternate bytes width  */
  s_command.alternate_bytes_width           = HAL_XSPI_ALTERNATE_BYTES_8BIT;
  /* Set memory alternate bytes DTR mode  */
  s_command.alternate_bytes_dtr_mode_status = HAL_XSPI_ALTERNATE_BYTES_DTR_DISABLED;
  /* Set memory DQS mode  */
  s_command.dqs_mode_status                 = HAL_XSPI_DQS_DISABLED;
  /* Set memory dummy cycles  */
  s_command.dummy_cycle                     = dummy_cycle;

  switch (pobj->interface)
  {
    case MX25LM51245G_SPI_MODE:
      /* Set memory instruction mode */
      s_command.instruction_mode             = HAL_XSPI_INSTRUCTION_1LINE;
      /* Set memory Double Transfer Rate mode */
      s_command.instruction_dtr_mode_status  = HAL_XSPI_INSTRUCTION_DTR_DISABLED;
      /* Set memory instruction width */
      s_command.instruction_width            = HAL_XSPI_INSTRUCTION_8BIT;
      /* Set memory instruction  */
      s_command.instruction                  = cmd;
      /* Set memory addressing mode  */
      s_command.addr_mode                    = addr_mode;
      /* Set memory address DTR mode  */
      s_command.addr_dtr_mode_status         = HAL_XSPI_ADDR_DTR_DISABLED;
      /* Set memory data mode  */
      s_command.data_mode                    = data_mode;
      /* Set memory data DTR mode  */
      s_command.data_dtr_mode_status         = HAL_XSPI_DATA_DTR_DISABLED;
      /* Set memory size byte  */
      s_command.size_byte                    = 1U;
      break;

    case MX25LM51245G_OPI_MODE:
      /* Common configuration for OPI */
      /* Set memory instruction mode */
      s_command.instruction_mode      = HAL_XSPI_INSTRUCTION_8LINES;
      /* Set memory instruction width */
      s_command.instruction_width     = HAL_XSPI_INSTRUCTION_16BIT;
      /* Set memory instruction  */
      s_command.instruction           = cmd;
      /* Set memory addressing mode  */
      s_command.addr_mode             = addr_mode;
      /* Set memory data mode  */
      s_command.data_mode             = data_mode;

      if (pobj->transfer_rate == MX25LM51245G_DTR_TRANSFER)
      {
        /* Set memory Double Transfer Rate mode */
        s_command.instruction_dtr_mode_status = HAL_XSPI_INSTRUCTION_DTR_ENABLED;
        /* Set memory address DTR mode  */
        s_command.addr_dtr_mode_status        = HAL_XSPI_ADDR_DTR_ENABLED;
        /* Set memory data DTR mode  */
        s_command.data_dtr_mode_status        = HAL_XSPI_DATA_DTR_ENABLED;
        /* Set memory size byte  */
        s_command.size_byte                   = 2U;
      }
      else
      {
        /* Set memory Double Transfer Rate mode */
        s_command.instruction_dtr_mode_status = HAL_XSPI_INSTRUCTION_DTR_DISABLED;
        /* Set memory address DTR mode  */
        s_command.addr_dtr_mode_status        = HAL_XSPI_ADDR_DTR_DISABLED;
        /* Set memory data DTR mode  */
        s_command.data_dtr_mode_status        = HAL_XSPI_DATA_DTR_DISABLED;
        /* Set memory size byte  */
        s_command.size_byte                   = 1U;
      }
      break;

    default:
      /* Unsupported interface */
      ret = MX25LM51245G_ERROR;
      break;
  }

  /* Send the command */
  if (HAL_XSPI_SendRegularCmd(pobj->io.phxspi, &s_command, MX25LM51245G_CMD_TIMEOUT_VALUE) != HAL_OK)
  {
    ret = MX25LM51245G_ERROR;
  }

  /* Return error status */
  return ret;
}

/* Dummy cycles management functions  -----------------------------------------------------*/

/**
  * @brief  Retrieves the effective memory interface clock frequency for the XSPI peripheral.
  * @param  pobj   : Pointer to the part object.
  * @param  p_freq  : Pointer to the xspi frequency in Hz.
  * @retval error status.
  */
mx25lm51245g_status_t mx25lm51245g_get_xspi_memory_frequency(mx25lm51245g_obj_t *pobj, uint32_t *p_freq)
{
  uint32_t clock_frequency;
  uint32_t clock_prescaler;

  /* Check param */
  if ((pobj == NULL) || (p_freq == NULL))
  {
    return MX25LM51245G_ERROR;
  }

  clock_frequency = HAL_XSPI_GetClockFreq(pobj->io.phxspi);
  clock_prescaler = HAL_XSPI_GetPrescaler(pobj->io.phxspi);

  if (clock_prescaler != 0U)
  {
    *p_freq = clock_frequency / clock_prescaler;
  }
  else
  {
    *p_freq = 0;
    return MX25LM51245G_ERROR;
  }

  return MX25LM51245G_OK;
}

/**
  * @brief  Retrieves the index of the dummy cycles lookup table based on the XSPI clock frequency.
  * @param  pobj         : Pointer to the part object.
  * @retval Index in the dummy cycles frequency lookup table.
  */
static uint32_t mx25lm51245g_get_lookup_dummy_cycles_index(mx25lm51245g_obj_t *pobj)
{
  uint32_t interface_freq = 0U;

  /* Retrieve the clock frequency -- no need to check the returned status */
  (void)mx25lm51245g_get_xspi_memory_frequency(pobj, &interface_freq);

  /* Iterate from highest frequency cutoff to lowest */
  for (size_t i = 0; i < MX25LM51245G_DC_TABLE_SIZE; i++)
  {
    /* If interface frequency is higher than or equal to the cutoff frequency,
     * assign the corresponding dummy cycles.*/
    if (interface_freq >= mx25lm51245g_dc_freq_lut[i].freq)
    {
      /* return the corresponding index */
      return i;
    }
  }

  /* If frequency is lower than all cutoff frequencies assign the lower value */
  /* return the lower value */
  return (MX25LM51245G_DC_TABLE_SIZE - 1U);
}

/**
  * @brief  Update the dummy cycles in the configuration register 2.
  * @param  pobj         : Part object pointer.
  * @param  lut_index    : dummy cycles lookup table index.
  * @retval error status
  */
static mx25lm51245g_status_t mx25lm51245g_update_dc_reg(mx25lm51245g_obj_t *pobj, uint8_t lut_index)
{
  /* Check params */
  if ((pobj == NULL) || (lut_index >= MX25LM51245G_DC_TABLE_SIZE))
  {
    return MX25LM51245G_ERROR;
  }
  /* Enable write operations */
  if (mx25lm51245g_write_enable(pobj) != MX25LM51245G_OK)
  {
    return MX25LM51245G_ERROR;
  }

  /* Write Configuration register with the new dummy cycles */
  if (mx25lm51245g_write_cfg2_register(pobj, MX25LM51245G_CR2_REG3_ADDR,
                                       mx25lm51245g_dc_freq_lut[lut_index].dc_reg_value) != MX25LM51245G_OK)
  {
    return MX25LM51245G_ERROR;
  }
  /* Update the current dc_lut_index value */
  pobj->dc_lut_index = lut_index;

  return MX25LM51245G_OK;
}

/* Memory Reset/Suspend/resume --------------------------------------------------------*/

/**
  * @brief  This function reset the OSPI memory.
  * @param  pobj : Pointer to the part object structure
  * @retval error status
  */
mx25lm51245g_status_t mx25lm51245g_reset(mx25lm51245g_obj_t *pobj)
{
  /* Check param */
  if (pobj == NULL)
  {
    return MX25LM51245G_ERROR;
  }

  /* Disable MMP */
  if (pobj->access_mode == MX25LM51245G_ACCESS_MMP)
  {
    /* Disable Memory mapped */
    if (mx25lm51245g_disable_memory_mapped(pobj) != MX25LM51245G_OK)
    {
      return MX25LM51245G_ERROR;
    }
  }

  uint32_t cmd;

  /* configure the reset enable cmd */
  if (pobj->interface == MX25LM51245G_SPI_MODE)
  {
    cmd = MX25LM51245G_RESET_ENABLE_CMD;
  }
  else
  {
    cmd = MX25LM51245G_OCTA_RESET_ENABLE_CMD;
  }

  /* send the reset enable cmd */
  if (mx25lm51245g_send_regular_cmd(pobj, HAL_XSPI_ADDR_NONE, HAL_XSPI_REGULAR_DATA_NONE, 0U, cmd,
                                    MX25LM51245G_RSTEN_DUMMY_CYCLES) != MX25LM51245G_OK)
  {
    return MX25LM51245G_ERROR;
  }

  /* configure the reset cmd */
  if (pobj->interface == MX25LM51245G_SPI_MODE)
  {
    cmd = MX25LM51245G_RESET_MEMORY_CMD;
  }
  else
  {
    cmd = MX25LM51245G_OCTA_RESET_MEMORY_CMD;
  }

  /* send the reset cmd */
  if (mx25lm51245g_send_regular_cmd(pobj, HAL_XSPI_ADDR_NONE, HAL_XSPI_REGULAR_DATA_NONE, 0U, cmd,
                                    MX25LM51245G_RST_DUMMY_CYCLES) != MX25LM51245G_OK)
  {
    return MX25LM51245G_ERROR;
  }

  if (pobj->transfer_rate  == MX25LM51245G_DTR_TRANSFER)
  {
    if (HAL_XSPI_DisableFreeRunningClock(pobj->io.phxspi) != HAL_OK)
    {
      return MX25LM51245G_ERROR;
    }
  }

  /* Set default interface & transfer_rate pboj values */
  pobj->access_mode    = MX25LM51245G_ACCESS_INDIRECT;     /* After reset S/W setting to indirect access  */
  pobj->interface      = MX25LM51245G_SPI_MODE;            /* After reset H/W back to SPI mode by default */
  pobj->transfer_rate  = MX25LM51245G_STR_TRANSFER;        /* After reset S/W setting to STR mode         */

  /* After SWreset CMD, wait in case SWReset occurred during erase operation */
  HAL_Delay(MX25LM51245G_RESET_MAX_TIME);

  return MX25LM51245G_OK;
}

#if defined (MX25LM_CALLBACKS) && (MX25LM_CALLBACKS == 1)

/**
  * @brief  Suspend program/erase operation.
  * @param  pobj  : Part object pointer.
  * @retval error status.
  * @note the suspend/resume can only be used with the mx25lm51245g_erase_it() api.
  */
mx25lm51245g_status_t mx25lm51245g_suspend(mx25lm51245g_obj_t *pobj)
{
  /* Check param */
  if (pobj == NULL)
  {
    return MX25LM51245G_ERROR;
  }

  /* Check the memory access mode & current async state (only mx25lm51245g_erase_it can be suspended) */
  if ((pobj->access_mode == MX25LM51245G_ACCESS_MMP)
      || (pobj->async_context.async_state != MX25LM51245G_ASYNC_OPERATION_ERASE))
  {
    return MX25LM51245G_ERROR;
  }
  /* Stop the XSPI auto-polling */
  if (HAL_XSPI_Abort(pobj->io.phxspi, MX25LM51245G_CMD_TIMEOUT_VALUE) != HAL_OK)
  {
    pobj->async_context.async_state = MX25LM51245G_ASYNC_OPERATION_NONE;
    return MX25LM51245G_ERROR;
  }

  uint32_t cmd;

  /* configure the params for the suspend cmd */
  if (pobj->interface == MX25LM51245G_SPI_MODE)
  {
    cmd = MX25LM51245G_PROG_ERASE_SUSPEND_CMD;
  }
  else
  {
    cmd = MX25LM51245G_OCTA_PROG_ERASE_SUSPEND_CMD;
  }

  /* send suspend cmd */
  if (mx25lm51245g_send_regular_cmd(pobj, HAL_XSPI_ADDR_NONE, HAL_XSPI_REGULAR_DATA_NONE, 0U, cmd,
                                    MX25LM51245G_PGM_ERS_SUSPEND_DUMMY_CYCLES) != MX25LM51245G_OK)
  {
    pobj->async_context.async_state = MX25LM51245G_ASYNC_OPERATION_NONE;
    return MX25LM51245G_ERROR;
  }
  pobj->async_context.async_state = MX25LM51245G_ASYNC_OPERATION_SUSPENDED_ERASE;

  return MX25LM51245G_OK;
}

/**
  * @brief  Resumes a suspended program/erase operation.
  * @param  pobj  : Part object pointer.
  * @retval error status.
  * @note the suspend/resume can only be used with the mx25lm51245g_erase_it() api.
  */
mx25lm51245g_status_t mx25lm51245g_resume(mx25lm51245g_obj_t *pobj)
{
  hal_xspi_auto_polling_config_t s_config;

  /* Check param */
  if (pobj == NULL)
  {
    return MX25LM51245G_ERROR;
  }

  /* Check the memory access mode & current async state */
  if ((pobj->access_mode == MX25LM51245G_ACCESS_MMP)
      || (pobj->async_context.async_state != MX25LM51245G_ASYNC_OPERATION_SUSPENDED_ERASE))
  {
    return MX25LM51245G_ERROR;
  }

  uint32_t cmd;

  /* configure the params for the resume cmd */
  if (pobj->interface == MX25LM51245G_SPI_MODE)
  {
    cmd = MX25LM51245G_PROG_ERASE_RESUME_CMD;
  }
  else
  {
    cmd = MX25LM51245G_OCTA_PROG_ERASE_RESUME_CMD;
  }

  /* send the resume cmd */
  if (mx25lm51245g_send_regular_cmd(pobj, HAL_XSPI_ADDR_NONE, HAL_XSPI_REGULAR_DATA_NONE, 0U, cmd,
                                    MX25LM51245G_PGM_ERS_RESUME_DUMMY_CYCLES) != MX25LM51245G_OK)
  {
    pobj->async_context.async_state = MX25LM51245G_ASYNC_OPERATION_NONE;
    return MX25LM51245G_ERROR;
  }
  /* Send the Read status command */
  if (mx25lm51245g_send_read_status_reg_cmd(pobj) != MX25LM51245G_OK)
  {
    pobj->async_context.async_state = MX25LM51245G_ASYNC_OPERATION_NONE;
    return MX25LM51245G_ERROR;
  }
  /* Configure and start WIP & WEL autopolling */
  s_config.match_value           = 0U;
  s_config.match_mask            = MX25LM51245G_SR_WIP | MX25LM51245G_SR_WEL;
  s_config.match_mode            = HAL_XSPI_MATCH_MODE_AND;
  s_config.interval_cycle        = MX25LM51245G_AUTOPOLLING_INTERVAL_TIME;
  s_config.automatic_stop_status = HAL_XSPI_AUTOMATIC_STOP_ENABLED;

  pobj->async_context.async_state = MX25LM51245G_ASYNC_OPERATION_ERASE;
  if (HAL_XSPI_ExecRegularAutoPoll_IT(pobj->io.phxspi, &s_config) != HAL_OK)
  {
    pobj->async_context.async_state = MX25LM51245G_ASYNC_OPERATION_NONE;
    return MX25LM51245G_ERROR;
  }

  return MX25LM51245G_OK;
}

/* Memory async operation getter function --------------------------------------------*/

/**
  * @brief  Gets the current asynchronous operation type.
  * @retval asynchronous operation type
  */
mx25lm51245g_async_operation_t mx25lm51245g_get_async_operation_type(mx25lm51245g_obj_t *pobj)
{
  /* return the asynchronous operation type */
  return pobj->async_context.async_state;
}

/* Memory erase internal callbacks -------------------------------------------------*/

/**
  * @brief  Internal async (block/sector) erase complete callback function.
  * @param  hxspi : xspi handle
  */
static void xspi_erase_cplt_cb(hal_xspi_handle_t *hxspi)
{
  mx25lm51245g_obj_t *pobj = (mx25lm51245g_obj_t *)HAL_XSPI_GetUserData(hxspi);
  pobj->async_context.async_state = MX25LM51245G_ASYNC_OPERATION_NONE;
  pobj->be_cb_ctx.callback(pobj, pobj->be_cb_ctx.arg);
}

/**
  * @brief  Internal chip erase complete callback function.
  * @param  hxspi : xspi handle
  */
static void xspi_ce_cplt_cb(hal_xspi_handle_t *hxspi)
{
  mx25lm51245g_obj_t *pobj = (mx25lm51245g_obj_t *)HAL_XSPI_GetUserData(hxspi);
  pobj->async_context.async_state = MX25LM51245G_ASYNC_OPERATION_NONE;
  pobj->ce_cb_ctx.callback(pobj, pobj->ce_cb_ctx.arg);
}

/* Memory register callback functions -------------------------------------------------*/

/**
  * @brief  Registers the block erase complete Callback and store user data pointer into the handle.
  * @param  pobj  : Pointer to the part object structure
  * @param  cb    : Pointer the mass erase complete callback function
  * @param  p_arg : Pointer to the user data
  */
mx25lm51245g_status_t mx25lm51245g_register_erase_cplt_callback(mx25lm51245g_obj_t *pobj,
                                                                mx25lm51245g_callback_t cb, void *p_arg)
{
  mx25lm51245g_status_t ret = MX25LM51245G_OK;
  pobj->be_cb_ctx.callback = cb;
  pobj->be_cb_ctx.arg      = p_arg;

  HAL_XSPI_SetUserData(pobj->io.phxspi, pobj);

  if (HAL_XSPI_RegisterStatusMatchCallback(pobj->io.phxspi, xspi_erase_cplt_cb) != HAL_OK)
  {
    ret = MX25LM51245G_ERROR;
  }

  return ret;
}

/**
  * @brief  Registers the mass erase complete Callback and store user data pointer into the handle.
  * @param  pobj  : Pointer to the part object structure
  * @param  cb    : Pointer the mass erase complete callback function
  * @param  p_arg : Pointer to the user data
  */
mx25lm51245g_status_t mx25lm51245g_register_mass_erase_cplt_callback(mx25lm51245g_obj_t *pobj,
                                                                     mx25lm51245g_callback_t cb, void *p_arg)
{
  mx25lm51245g_status_t ret = MX25LM51245G_OK;
  pobj->ce_cb_ctx.callback = cb;
  pobj->ce_cb_ctx.arg = p_arg;

  HAL_XSPI_SetUserData(pobj->io.phxspi, pobj);
  if (HAL_XSPI_RegisterStatusMatchCallback(pobj->io.phxspi, xspi_ce_cplt_cb) != HAL_OK)
  {
    ret = MX25LM51245G_ERROR;
  }

  return ret;
}

#if defined (USE_HAL_XSPI_DMA) && (USE_HAL_XSPI_DMA == 1)

/**
  * @brief  Registers the receive complete callback and store user data pointer into the handle.
  * @param  pobj  : Pointer to the part object structure
  * @param  cb    : Pointer the Rx complete callback function
  * @param  p_arg : Pointer to the user data
  */
mx25lm51245g_status_t mx25lm51245g_register_read_cplt_callback(mx25lm51245g_obj_t *pobj, mx25lm51245g_callback_t cb,
                                                               void *parg)
{
  if ((pobj == NULL) || (cb == NULL))
  {
    return MX25LM51245G_ERROR;
  }
  pobj->rd_cb_ctx.callback = cb;
  pobj->rd_cb_ctx.arg      = parg;

  return MX25LM51245G_OK;
}

/**
  * @brief  Registers the transmit complete Callback and store user data pointer into the handle.
  * @param  pobj  : Pointer to the part object structure
  * @param  cb    : Pointer the Tx complete callback function
  * @param  p_arg : Pointer to the user data
  */
mx25lm51245g_status_t mx25lm51245g_register_write_cplt_callback(mx25lm51245g_obj_t *pobj, mx25lm51245g_callback_t cb,
                                                                void *parg)
{

  if ((pobj == NULL) || (cb == NULL))
  {
    return MX25LM51245G_ERROR;
  }
  pobj->wr_cb_ctx.callback = cb;
  pobj->wr_cb_ctx.arg      = parg;

  return MX25LM51245G_OK;
}

/* Memory read/write internal callbacks -------------------------------------------------*/

/**
  * @brief  Internal receive complete callback function.
  * @param  hxspi : xspi handle
  */
static void xspi_rd_cplt_cb(hal_xspi_handle_t *phxspi)
{
  mx25lm51245g_obj_t *pobj = (mx25lm51245g_obj_t *)HAL_XSPI_GetUserData(phxspi);
  pobj->async_context.async_state = MX25LM51245G_ASYNC_OPERATION_NONE;
  pobj->async_context.async_read_phase = MX25LM51245G_ASYNC_READ_IDLE;

  if (pobj->rd_cb_ctx.callback != NULL)
  {
    pobj->rd_cb_ctx.callback(pobj, pobj->rd_cb_ctx.arg);
  }
}


/* Memory internal asynchronous write state machine functions ------------------------------*/

/**
  * @brief  Gets the current phase of the asynchronous write state machine.
  * @retval asynchronous write phase
  */
mx25lm51245g_async_write_phase_t mx25lm51245g_get_async_write_phase(mx25lm51245g_obj_t *pobj)
{
  /* return the current phase of the asynchronous write state machine  */
  return pobj->async_context.async_write_phase;
}

/**
  * @brief  Gets the current phase of the asynchronous read state machine.
  * @retval asynchronous read phase
  */
mx25lm51245g_async_read_phase_t mx25lm51245g_get_async_read_phase(mx25lm51245g_obj_t *pobj)
{
  /* return the current phase of the asynchronous read state machine  */
  return pobj->async_context.async_read_phase;
}


/**
  * @brief  Writes an amount of data to the OSPI memory using the Page Program command.
  * @param  pobj       : Part object pointer
  * @param  addr_size  : Address size
  * @param  write_addr : Write start address
  * @param  pbuffer    : Pointer to data to be written
  * @param  size_byte  : Size of data to write in bytes (Range 1 ~ MX25LM51245G_PAGE_SIZE).
  */
static void mx25lm51245g_async_write(mx25lm51245g_obj_t *pobj, mx25lm51245g_addr_size_t addr_size,
                                     uint32_t write_addr, uint8_t *pbuffer, uint32_t size_byte)
{
  hal_xspi_regular_cmd_t s_command;

  /* Initialize the program command */
  /* Set memory operation type */
  s_command.operation_type         = HAL_XSPI_OPERATION_COMMON_CFG;
  /* Select IOs to be used */
  s_command.io_select              = HAL_XSPI_IO_7_0;
  /* Set read/write address */
  s_command.addr                   = write_addr;
  /* Set memory size byte  */
  s_command.size_byte              = size_byte;
  /* Set memory dummy cycles  */
  s_command.dummy_cycle            = MX25LM51245G_PAGE_PROG_DUMMY_CYCLES;
  /* Set memory DQS mode  */
  s_command.dqs_mode_status        = HAL_XSPI_DQS_DISABLED;
  /* Set memory alternate bytes mode  */
  s_command.alternate_bytes_mode   = HAL_XSPI_ALTERNATE_BYTES_NONE;
  /* Set memory alternate bytes       */
  s_command.alternate_bytes        = 0U;
  /* Set memory alternate bytes width  */
  s_command.alternate_bytes_width  = HAL_XSPI_ALTERNATE_BYTES_8BIT;
  /* Set memory alternate bytes DTR mode  */
  s_command.alternate_bytes_dtr_mode_status =  HAL_XSPI_ALTERNATE_BYTES_DTR_DISABLED;
  switch (pobj->interface)
  {
    case MX25LM51245G_SPI_MODE:
      /* Set memory instruction mode */
      s_command.instruction_mode     = HAL_XSPI_INSTRUCTION_1LINE;
      /* Set memory Double Transfer Rate mode */
      s_command.instruction_dtr_mode_status = HAL_XSPI_INSTRUCTION_DTR_DISABLED;
      /* Set memory instruction width */
      s_command.instruction_width    = HAL_XSPI_INSTRUCTION_8BIT;
      /* Set memory instruction  */
      s_command.instruction          = (addr_size == MX25LM51245G_3BYTES_SIZE)
                                       ? MX25LM51245G_PAGE_PROG_CMD
                                       : MX25LM51245G_4_BYTE_PAGE_PROG_CMD;
      /* Set memory addressing mode  */
      s_command.addr_mode            = HAL_XSPI_ADDR_1LINE;
      /* Set memory address DTR mode  */
      s_command.addr_dtr_mode_status = HAL_XSPI_ADDR_DTR_DISABLED;
      /* Set memory address width  */
      s_command.addr_width           = (addr_size == MX25LM51245G_3BYTES_SIZE)
                                       ? HAL_XSPI_ADDR_24BIT
                                       : HAL_XSPI_ADDR_32BIT;
      /* Set memory data mode  */
      s_command.data_mode            =  HAL_XSPI_REGULAR_DATA_1LINE;
      /* Set memory data DTR mode  */
      s_command.data_dtr_mode_status = HAL_XSPI_DATA_DTR_DISABLED;
      break;
    case MX25LM51245G_OPI_MODE:
      /* Common configuration for OPI mode */
      /* Set memory instruction mode */
      s_command.instruction_mode     = HAL_XSPI_INSTRUCTION_8LINES;
      /* Set memory instruction width */
      s_command.instruction_width    = HAL_XSPI_INSTRUCTION_16BIT;
      /* Set memory addressing mode  */
      s_command.addr_mode            = HAL_XSPI_ADDR_8LINES;
      /* Set memory data mode  */
      s_command.data_mode            = HAL_XSPI_REGULAR_DATA_8LINES;
      /* Set memory address width  */
      s_command.addr_width           = HAL_XSPI_ADDR_32BIT;
      if (pobj->transfer_rate == MX25LM51245G_DTR_TRANSFER)
      {
        /* Set memory Double Transfer Rate mode */
        s_command.instruction_dtr_mode_status = HAL_XSPI_INSTRUCTION_DTR_ENABLED;
        /* Set memory instruction  */
        s_command.instruction          = MX25LM51245G_OCTA_PAGE_PROG_CMD;
        /* Set memory address DTR mode  */
        s_command.addr_dtr_mode_status  = HAL_XSPI_ADDR_DTR_ENABLED;

        /* Set memory data DTR mode  */
        s_command.data_dtr_mode_status  = HAL_XSPI_DATA_DTR_ENABLED;
      }
      else
      {
        /* Set memory Double Transfer Rate mode */
        s_command.instruction_dtr_mode_status = HAL_XSPI_INSTRUCTION_DTR_DISABLED;

        /* Set memory instruction  */
        s_command.instruction           = MX25LM51245G_OCTA_PAGE_PROG_CMD;
        /* Set memory address DTR mode  */
        s_command.addr_dtr_mode_status  = HAL_XSPI_ADDR_DTR_DISABLED;
        /* Set memory data DTR mode  */
        s_command.data_dtr_mode_status  = HAL_XSPI_DATA_DTR_DISABLED;
      }
      break;
    default:
      break;
  }
  /* Configure the command */
  (void)HAL_XSPI_SendRegularCmd(pobj->io.phxspi, &s_command, MX25LM51245G_CMD_TIMEOUT_VALUE);
#if defined (__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
  SCB_CleanDCache_by_Addr(pbuffer, (int32_t)size_byte);
#endif /* __DCACHE_PRESENT */
  (void)HAL_XSPI_Transmit_DMA(pobj->io.phxspi, pbuffer);

}

/**
  * @brief  Polling WIP(Write In Progress) bit until it becomes 0 in non-blocking mode.
  * @param  pobj : Pointer to the part object structure
  */
static void mx25lm51245g_wip_autopolling_it(mx25lm51245g_obj_t *pobj)
{
  hal_xspi_auto_polling_config_t s_config;

  /* Send Command */
  if (mx25lm51245g_send_read_status_reg_cmd(pobj) == MX25LM51245G_OK)
  {
    /* Configure and start WIP autopolling - start data phase */
    s_config.match_value           = 0U;
    s_config.match_mask            = MX25LM51245G_SR_WIP;
    s_config.match_mode            = HAL_XSPI_MATCH_MODE_AND;
    s_config.interval_cycle        = MX25LM51245G_AUTOPOLLING_INTERVAL_TIME;
    s_config.automatic_stop_status = HAL_XSPI_AUTOMATIC_STOP_ENABLED;

    (void)HAL_XSPI_ExecRegularAutoPoll_IT(pobj->io.phxspi, &s_config);
  }
}

/**
  * @brief  Sends the write enable command in non-blocking mode.
  * @param  pobj : Pointer to the part object structure
  */
static void mx25lm51245g_write_enable_it(mx25lm51245g_obj_t *pobj)
{
  hal_xspi_regular_cmd_t     s_command;

  /* Initialize the write enable command */
  /* Set memory operation type */
  s_command.operation_type       = HAL_XSPI_OPERATION_COMMON_CFG;
  /* Select IOs to be used */
  s_command.io_select            = HAL_XSPI_IO_7_0;

  /* Set memory addressing mode  */
  s_command.addr_mode             = HAL_XSPI_ADDR_NONE;
  /* Set memory address  */
  s_command.addr                  = 0U;
  /* Set memory address width  */
  s_command.addr_width            = HAL_XSPI_ADDR_32BIT;
  /* Set memory address DTR mode  */
  s_command.addr_dtr_mode_status  = HAL_XSPI_ADDR_DTR_DISABLED;
  /* Set memory alternate bytes mode  */
  s_command.alternate_bytes_mode = HAL_XSPI_ALTERNATE_BYTES_NONE;
  /* Set memory alternate bytes       */
  s_command.alternate_bytes      = 0U;
  /* Set memory alternate bytes width */
  s_command.alternate_bytes_width = HAL_XSPI_ALTERNATE_BYTES_8BIT;
  /* Set memory alternate bytes DTR mode  */
  s_command.alternate_bytes_dtr_mode_status =  HAL_XSPI_ALTERNATE_BYTES_DTR_DISABLED;
  /* Set size byte */
  s_command.size_byte             = 1U;
  /* Set memory data mode  */
  s_command.data_mode            = HAL_XSPI_REGULAR_DATA_NONE;
  /* Set memory data DTR mode  */
  s_command.data_dtr_mode_status = HAL_XSPI_DATA_DTR_DISABLED;
  /* Set memory dummy cycles  */
  s_command.dummy_cycle          = MX25LM51245G_WREN_DUMMY_CYCLES;
  /* Set memory DQS mode  */
  s_command.dqs_mode_status             = HAL_XSPI_DQS_DISABLED;

  switch (pobj->interface)
  {
    case MX25LM51245G_SPI_MODE:
      /* Set memory instruction mode */
      s_command.instruction_mode     = HAL_XSPI_INSTRUCTION_1LINE;
      /* Set memory instruction width */
      s_command.instruction_width    = HAL_XSPI_INSTRUCTION_8BIT;
      /* Set memory instruction  */
      s_command.instruction          = MX25LM51245G_WRITE_ENABLE_CMD;
      /* Set memory Double Transfer Rate mode */
      s_command.instruction_dtr_mode_status = HAL_XSPI_INSTRUCTION_DTR_DISABLED;
      break;
    case MX25LM51245G_OPI_MODE:
      /* Set memory instruction mode */
      s_command.instruction_mode     = HAL_XSPI_INSTRUCTION_8LINES;
      /* Set memory instruction width */
      s_command.instruction_width    = HAL_XSPI_INSTRUCTION_16BIT;
      /* Set memory instruction  */
      s_command.instruction          = MX25LM51245G_OCTA_WRITE_ENABLE_CMD;
      /* Set memory Double Transfer Rate mode */
      s_command.instruction_dtr_mode_status = (pobj->transfer_rate == MX25LM51245G_DTR_TRANSFER)
                                              ? HAL_XSPI_INSTRUCTION_DTR_ENABLED
                                              : HAL_XSPI_INSTRUCTION_DTR_DISABLED;
      break;
    default:
      break;
  }
  /* Send the command */
  (void)HAL_XSPI_SendRegularCmd_IT(pobj->io.phxspi, &s_command);

}

/**
  * @brief  Polling WEL(Write Enable Latch) bit until it is set in non-blocking mode.
  * @param  pobj : Pointer to the part object structure
  */
static void mx25lm51245g_wel_autopolling_it(mx25lm51245g_obj_t *pobj)
{
  hal_xspi_auto_polling_config_t s_config;

  if (mx25lm51245g_send_read_status_reg_cmd(pobj) == MX25LM51245G_OK)
  {
    s_config.match_value           = 2U;
    s_config.match_mask            = MX25LM51245G_SR_WEL;
    s_config.match_mode            = HAL_XSPI_MATCH_MODE_AND;
    s_config.interval_cycle        = MX25LM51245G_AUTOPOLLING_INTERVAL_TIME;
    s_config.automatic_stop_status = HAL_XSPI_AUTOMATIC_STOP_ENABLED;

    (void)HAL_XSPI_ExecRegularAutoPoll_IT(pobj->io.phxspi, &s_config);
  }

}

/**
  * @brief  Internal callback function to handle the Write Enable Latch (WEL) check for asynchronous write operations.
  * @param  phxpi   xspi handle
  */
static void mx25lm51245g_wel_check_cb(hal_xspi_handle_t *phxspi)
{
  mx25lm51245g_obj_t *pobj = (mx25lm51245g_obj_t *)HAL_XSPI_GetUserData(phxspi);

  if (pobj->async_context.async_write_phase == MX25LM51245G_ASYNC_WEL_CHECK)
  {
    uint8_t *buffer;

    if (pobj->async_context.is_aligned != 0U)
    {
      /* Aligned user buffer: transmit directly from it, no bounce buffer / no cpy */
      buffer = &pobj->async_context.transfer_buffer[pobj->async_context.current_index];
    }
    else
    {
      buffer = pobj->async_context.dma_buffer;
    }

    pobj->async_context.async_write_phase = MX25LM51245G_ASYNC_WRITE_ONGOING;
    /* Transmit the active buffer content in a single DMA transfer */
    (void)HAL_XSPI_RegisterTxCpltCallback(pobj->io.phxspi, xspi_wr_cplt_cb);
    mx25lm51245g_async_write(pobj, pobj->async_context.addr_size, pobj->async_context.current_addr,
                             buffer, pobj->async_context.current_transfer_len);
  }
}

/**
  * @brief  Internal async page write complete callback function.
  * @param  phxspi   xspi handle
  */
static void xspi_wr_cplt_cb(hal_xspi_handle_t *phxspi)
{
  mx25lm51245g_obj_t *pobj = (mx25lm51245g_obj_t *)HAL_XSPI_GetUserData(phxspi);

  if (pobj->async_context.async_write_phase == MX25LM51245G_ASYNC_WRITE_ONGOING)
  {
    pobj->async_context.current_addr  += pobj->async_context.current_transfer_len;
    pobj->async_context.current_index += pobj->async_context.current_transfer_len;

    if (pobj->async_context.current_index >= pobj->async_context.total_size)
    {
      /* All data programmed - wait for the last page program to complete */
      pobj->async_context.async_write_phase = MX25LM51245G_ASYNC_WRITE_DONE;
      mx25lm51245g_register_status_match_callback(pobj, mx25lm51245g_clean_up_cb);
      mx25lm51245g_wip_autopolling_it(pobj);
    }
    else
    {
      /* Program the next chunk */
      mx25lm51245g_async_tx_setup(pobj);
    }
  }
}

/**
  * @brief  Internal status match callback function for write in progress done (RESET).
  * @param  hxspi : xspi handle
  */
static void mx25lm51245g_wip_check_cb(hal_xspi_handle_t *hxspi)
{
  mx25lm51245g_obj_t *pobj = (mx25lm51245g_obj_t *)HAL_XSPI_GetUserData(hxspi);
  if (pobj->async_context.async_write_phase == MX25LM51245G_ASYNC_WIP_CHECK)
  {
    pobj->async_context.async_write_phase = MX25LM51245G_ASYNC_WRITE_ENABLE;
    mx25lm51245g_register_cmd_cplt_callback(pobj, mx25lm51245g_write_enable_cb);
    mx25lm51245g_write_enable_it(pobj);
  }
}

/**
  * @brief  Internal function to set up the next asynchronous write transfer and start it.
  * @param  pobj : Pointer to the part object structure
  */
static void mx25lm51245g_async_tx_setup(mx25lm51245g_obj_t *pobj)
{
  uint32_t remaining = pobj->async_context.total_size - pobj->async_context.current_index;
  /* A Page Program command must not cross a page boundary */
  uint32_t page_room = MX25LM51245G_PAGE_SIZE - (pobj->async_context.current_addr % MX25LM51245G_PAGE_SIZE);
  uint32_t chunk     = (remaining < page_room) ? remaining : page_room;

  pobj->async_context.current_transfer_len = chunk;

  /* Defer the copy and write sequence to thread mode (mx25lm51245g_exec_data_handler). */
  pobj->async_context.async_write_phase = MX25LM51245G_ASYNC_WRITE_CPY_PENDING;
}

/**
  * @brief  Internal command complete callback function for write enable.
  * @param  phxspi   xspi handle
  */
static void mx25lm51245g_write_enable_cb(hal_xspi_handle_t *phxspi)
{
  mx25lm51245g_obj_t *pobj = (mx25lm51245g_obj_t *)HAL_XSPI_GetUserData(phxspi);

  if (pobj->async_context.async_write_phase == MX25LM51245G_ASYNC_WRITE_ENABLE)
  {
    pobj->async_context.async_write_phase = MX25LM51245G_ASYNC_WEL_CHECK;

    mx25lm51245g_register_status_match_callback(pobj, mx25lm51245g_wel_check_cb);
    mx25lm51245g_wel_autopolling_it(pobj);
  }
}

/**
  * @brief  Internal status match callback function for write in progress done (RESET).
  * @param  phxspi   xspi handle
  */
static void mx25lm51245g_clean_up_cb(hal_xspi_handle_t *phxspi)
{
  mx25lm51245g_obj_t *pobj = (mx25lm51245g_obj_t *)HAL_XSPI_GetUserData(phxspi);

  if (pobj->async_context.async_write_phase == MX25LM51245G_ASYNC_WRITE_DONE)
  {
    /* Transfer Done - return the state to IDLE */
    pobj->async_context.async_write_phase = MX25LM51245G_ASYNC_WRITE_IDLE;
    /* Update async state */
    pobj->async_context.async_state = MX25LM51245G_ASYNC_OPERATION_NONE;
    /*  call the registered callback */
    if (pobj->wr_cb_ctx.callback != NULL)
    {
      pobj->wr_cb_ctx.callback(pobj, pobj->wr_cb_ctx.arg);
    }
  }
}

/**
  * @brief  Registers the command complete callback.
  * @param  pobj : Pointer to the part object structure
  * @param  p_cb : Pointer the command complete callback function
  */
static void mx25lm51245g_register_cmd_cplt_callback(mx25lm51245g_obj_t *pobj, hal_xspi_cb_t p_cb)
{
  (void)HAL_XSPI_RegisterCmdCpltCallback(pobj->io.phxspi, p_cb);
}

/**
  * @brief  Registers the status match callback.
  * @param  pobj : Pointer to the part object structure
  * @param  p_cb : Pointer the status match callback function
  */
static void mx25lm51245g_register_status_match_callback(mx25lm51245g_obj_t *pobj, hal_xspi_cb_t p_cb)
{
  (void)HAL_XSPI_RegisterStatusMatchCallback(pobj->io.phxspi, p_cb);
}

#endif /* USE_HAL_XSPI_DMA */
#endif /* MX25LM_CALLBACKS */
