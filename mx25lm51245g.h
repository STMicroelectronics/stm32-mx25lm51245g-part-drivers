/**
  ******************************************************************************
  * @file    mx25lm51245g.h
  * @brief   This file contains all the description of the
  *          MX25LM51245G OSPI memory.
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
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef MX25LM51245G_H
#define MX25LM51245G_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32_hal.h"

#if defined (USE_HAL_XSPI_REGISTER_CALLBACKS) && (USE_HAL_XSPI_REGISTER_CALLBACKS == 1)
#if defined (USE_HAL_XSPI_USER_DATA) && (USE_HAL_XSPI_USER_DATA == 1)
#define MX25LM_CALLBACKS 1
#endif /* USE_HAL_XSPI_USER_DATA */
#endif /* USE_HAL_XSPI_REGISTER_CALLBACKS */

#if defined (__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
#define MX25LM51245G_ALIGNMENT_SIZE   __SCB_DCACHE_LINE_SIZE
#else
#define MX25LM51245G_ALIGNMENT_SIZE   4U
#endif /* __DCACHE_PRESENT */

typedef struct
{
  uint32_t flash_size;                       /*!< Size of the flash                             */
  uint32_t erase_block_size;                 /*!< Size of blocks for the erase operation        */
  uint32_t erase_block_number;               /*!< Number of blocks for the erase operation      */
  uint32_t erase_sector_size;                /*!< Size of sector for the erase operation        */
  uint32_t erase_sector_number;              /*!< Number of sectors for the erase operation     */
  uint32_t prog_page_size;                   /*!< Size of pages for the program operation       */
  uint32_t prog_pages_number;                /*!< Number of pages for the program operation     */
} mx25lm51245g_info_t;

typedef enum
{
  MX25LM51245G_INIT_NONE = 0,                 /*!<  Instance is not initialized                */
  MX25LM51245G_INIT_OK                        /*!<  Instance is initialized                    */
} mx25lm51245g_init_state_t;

typedef enum
{
  MX25LM51245G_ACCESS_NONE = 0,              /*!<  Instance not initialized,                    */
  MX25LM51245G_ACCESS_INDIRECT,              /*!<  Instance use indirect mode access            */
  MX25LM51245G_ACCESS_MMP                    /*!<  Instance use Memory Mapped Mode read         */
} mx25lm51245g_access_t;

typedef enum
{
  MX25LM51245G_SPI_MODE = 0,                 /*!< 1-1-1 commands, Power on H/W default setting  */
  MX25LM51245G_OPI_MODE                      /*!< 8-8-8 commands                                */
} mx25lm51245g_interface_t;

typedef enum
{
  MX25LM51245G_STR_TRANSFER = 0,             /*!< Single Transfer Rate  */
  MX25LM51245G_DTR_TRANSFER                  /*!< Double Transfer Rate  */
} mx25lm51245g_transfer_rate_t;

typedef enum
{
  MX25LM51245G_ERASE_SECTOR = 0,                 /*!< 4K size Sector erase  */
  MX25LM51245G_ERASE_BLOCK                     /*!< 64K size Block erase  */
} mx25lm51245g_erase_t;

typedef enum
{
  MX25LM51245G_3BYTES_SIZE = 0,              /*!< 3 Bytes address mode  */
  MX25LM51245G_4BYTES_SIZE                   /*!< 4 Bytes address mode  */
} mx25lm51245g_addr_size_t;

/**
  * @brief Enumeration defining block protection values.
  */
typedef enum
{
  MX25LM51245G_BP_NONE         = 0x00U,         /*!< [BP0-BP3] = 0  - 0 blocks are protected               */
  MX25LM51245G_BP_1_BLOCK      = 0x04U,         /*!< [BP0-BP3] = 1  - 1 block is protected                 */
  MX25LM51245G_BP_2_BLOCKS     = 0x08U,         /*!< [BP0-BP3] = 2  - 2 blocks are protected               */
  MX25LM51245G_BP_3_BLOCKS     = 0x0CU,         /*!< [BP0-BP3] = 3  - 3 blocks are protected               */
  MX25LM51245G_BP_4_BLOCKS     = 0x10U,         /*!< [BP0-BP3] = 4  - 4 blocks are protected               */
  MX25LM51245G_BP_5_BLOCKS     = 0x14U,         /*!< [BP0-BP3] = 5  - 5 blocks are protected               */
  MX25LM51245G_BP_6_BLOCKS     = 0x18U,         /*!< [BP0-BP3] = 6  - 6 blocks are protected               */
  MX25LM51245G_BP_7_BLOCKS     = 0x1CU,         /*!< [BP0-BP3] = 7  - 7 blocks are protected               */
  MX25LM51245G_BP_8_BLOCKS     = 0x20U,         /*!< [BP0-BP3] = 8  - 8 blocks are protected               */
  MX25LM51245G_BP_9_BLOCKS     = 0x24U,         /*!< [BP0-BP3] = 9  - 9 blocks are protected               */
  MX25LM51245G_BP_10_BLOCKS    = 0x28U,         /*!< [BP0-BP3] = 10 - 10 blocks are protected              */
  MX25LM51245G_BP_11_BLOCKS    = 0x2CU,         /*!< [BP0-BP3] = 11 - 11 blocks are protected              */
  MX25LM51245G_BP_ALL_BLOCKS   = 0x30U          /*!< [BP0-BP3] = (12,13,14,15) - All blocks are protected  */
} mx25lm51245g_bp_cfg_t;

/**
  * @brief Enumeration defining the Top/Bottom OTP bit possible values.
  */
typedef enum
{
  MX25LM51245G_BP_OTP_TB_TOP      = 0U,         /*!< TB = 0 default - Top area protect  */
  MX25LM51245G_BP_OTP_TB_BOTTOM   = 1U          /*!< TB = 1 - Bottom area protect       */
} mx25lm51245g_bp_otp_tb_cfg_t;

typedef struct
{
  uint32_t                      id;           /*!< Instance ID             */
  hal_xspi_handle_t             *phxspi;      /*!< pointer to XSPI Handle           */
  uint32_t                      memory_base_address; /*!< XSPI Base address using for memory mapped */
} mx25lm51245g_io_t;

typedef struct mx25lm51245g_obj_s mx25lm51245g_obj_t;

#if defined (MX25LM_CALLBACKS) && (MX25LM_CALLBACKS == 1)
typedef void (*mx25lm51245g_callback_t)(mx25lm51245g_obj_t *pobj, void *arg);

/**
  * @brief Callback context for MX25LM51245G operations.
  *
  * This structure holds the callback function and its associated argument
  * for MX25LM51245G operations.
  */
typedef struct
{
  mx25lm51245g_callback_t callback; /*!< Callback function to register           */
  void *arg;                        /*!< Optional argument given to the callback */
} mx25lm51245g_callback_ctx_t;

/**
  * @brief Enumeration defining the current asynchronous operation type
  */
typedef enum
{
  MX25LM51245G_ASYNC_OPERATION_NONE = 0,        /*!< No async operation ongoing   */
  MX25LM51245G_ASYNC_OPERATION_WRITE,           /*!< Async write ongoing          */
  MX25LM51245G_ASYNC_OPERATION_READ,            /*!< Async read ongoing           */
  MX25LM51245G_ASYNC_OPERATION_ERASE,           /*!< Async erase ongoing          */
  MX25LM51245G_ASYNC_OPERATION_MASS_ERASE,      /*!< Async mass erase ongoing     */
  MX25LM51245G_ASYNC_OPERATION_SUSPENDED_ERASE  /*!< Async erase is suspended     */
} mx25lm51245g_async_operation_t;

/**
  * @brief Enumeration defining the asynchronous write state machine steps
  */
typedef enum
{
  MX25LM51245G_ASYNC_WRITE_IDLE = 0,        /*!< Idle state - no ongoing write transfer        */
  MX25LM51245G_ASYNC_WRITE_CPY_PENDING,     /*!< Copy user data to bounce buffer (thread mode) */
  MX25LM51245G_ASYNC_WIP_CHECK,             /*!< Autopoll WIP bit        - Async write step 1  */
  MX25LM51245G_ASYNC_WRITE_ENABLE,          /*!< Send write enable cmd   - Async write step 2  */
  MX25LM51245G_ASYNC_WEL_CHECK,             /*!< Autopoll WEL bit        - Async write step 3  */
  MX25LM51245G_ASYNC_WRITE_ONGOING,         /*!< Data transmission phase - Async write step 4  */
  MX25LM51245G_ASYNC_WRITE_DONE             /*!< Clean-up step WIP check - Async write step 5  */
} mx25lm51245g_async_write_phase_t;

/**
  * @brief Enumeration defining the asynchronous read state machine steps
  */
typedef enum
{
  MX25LM51245G_ASYNC_READ_IDLE = 0,     /*!< Idle state - no ongoing read transfer        */
  MX25LM51245G_ASYNC_READING,           /*!< Async read running  */
  MX25LM51245G_ASYNC_READ_CPY_PENDING   /*!< Copy bounce buffer to user data (thread mode) */
} mx25lm51245g_async_read_phase_t;

typedef struct
{
  mx25lm51245g_async_operation_t      async_state;           /*!< Current async operation type          */
  uint32_t                            current_addr;          /*!< Current memory address in the transfer*/
  uint32_t                            current_index;         /*!< Current index in the user buffer      */
  uint32_t                            total_size;            /*!< Total number of bytes to transfer     */
  uint32_t                            current_transfer_len;  /*!< Size of the ongoing page-buffer chunk */
  uint32_t                            lut_index;             /*!< Cached DC LUT index                   */
  uint8_t                            *transfer_buffer;       /*!< User data buffer (source/destination) */
  mx25lm51245g_addr_size_t            addr_size;             /*!< Addressing mode 3/4bytes              */
  mx25lm51245g_async_write_phase_t    async_write_phase;     /*!< Memory async write Transfer step      */
  mx25lm51245g_async_read_phase_t     async_read_phase;      /*!< Memory async read Transfer step       */
  uint8_t                             is_aligned;            /*!< User buffer aligned to MX25LM51245G_ALIGNMENT_SIZE: bounce buffer/memcpy skipped */
  uint8_t                            *dma_buffer;            /*!< Optional user-provided aligned DMA buffer   */
  uint32_t                            dma_buffer_size;       /*!< Size of the user-provided DMA buffer (bytes) */
} mx25lm51245g_async_transfer_context_t;

#endif /* MX25LM_CALLBACKS */

struct mx25lm51245g_obj_s
{
  mx25lm51245g_init_state_t       is_initialized;    /*!< Init state                              */
  mx25lm51245g_access_t           access_mode;       /*!< Access mode: indirect, MMP or none      */
  mx25lm51245g_io_t               io;                /*!< IO interface structure                  */
  mx25lm51245g_interface_t        interface;         /*!< Interface: OSP or OPI                   */
  mx25lm51245g_transfer_rate_t    transfer_rate;     /*!< Transfer rate: STR or DTR               */
  uint8_t                         dc_lut_index;      /*!< Dummy Cycle LUT index                   */
#if defined (MX25LM_CALLBACKS) && (MX25LM_CALLBACKS == 1)
  mx25lm51245g_callback_ctx_t     rd_cb_ctx;           /*!< Read  Complete: Callback data           */
  mx25lm51245g_callback_ctx_t     wr_cb_ctx;           /*!< Write Complete: Callback data           */
  mx25lm51245g_callback_ctx_t     ce_cb_ctx;           /*!< Chip Erase Complete: Callback data      */
  mx25lm51245g_callback_ctx_t     be_cb_ctx;           /*!< Block Erase Complete: Callback data     */
  mx25lm51245g_async_transfer_context_t async_context; /*!< Asynchronous transfer context           */
#endif /* MX25LM_CALLBACKS */
};

typedef struct
{
  mx25lm51245g_interface_t        interface;         /*!< Interface: OSP or OPI                   */
  mx25lm51245g_transfer_rate_t    transfer_rate;     /*!< Transfer rate: STR or DTR               */
} mx25lm51245g_memcfg_t;

/**
  * @brief Macro defining the Write status reg without Block secure protection.
  */
#define MX25LM51245G_WRITE_STATUS_REG_DEFAULT_VALUE           0x00

/* MX25LM51245G Default timeout */
#if !defined(MX25LM51245G_CMD_TIMEOUT_VALUE)
#define MX25LM51245G_CMD_TIMEOUT_VALUE       (5000U)  /* 5 s */
#endif /* MX25LM51245G_CMD_TIMEOUT_VALUE */

#define MX25LM51245G_CHIP_ERASE_TIMEOUT       (300000U)  /* 300 s */

/* MX25LM51245G Size configuration */
#define MX25LM51245G_BLOCK_64K                   (uint32_t)(64 * 1024)         /* 1024 blocks of 64KBytes   */
#define MX25LM51245G_SECTOR_4K                   (uint32_t)(4  * 1024)         /* 16384 sectors of 4KBytes  */

#define MX25LM51245G_FLASH_SIZE                   (uint32_t)(512*1024*1024/8)  /* 512 Mbits => 64MBytes        */
#define MX25LM51245G_PAGE_SIZE                    (uint32_t)256                /* 262144 pages of 256 Bytes    */

/* MX25LM51245G Timing configuration */
#define MX25LM51245G_BULK_ERASE_MAX_TIME          460000U
#define MX25LM51245G_BLOCK_ERASE_MAX_TIME         1000U
#define MX25LM51245G_SECTOR_4K_ERASE_MAX_TIME     400U
#define MX25LM51245G_WRITE_REG_MAX_TIME           40U

#define MX25LM51245G_RESET_MAX_TIME               100U                 /* when SWreset during erase operation */

#define MX25LM51245G_AUTOPOLLING_INTERVAL_TIME    0x10U

/* MX25LM51245G dummy cycles look up table index */
#define MX25LM51245G_DC_20_LUT_INDEX_0     0U
#define MX25LM51245G_DC_12_LUT_INDEX_1     1U
#define MX25LM51245G_DC_8_LUT_INDEX_2      2U
#define MX25LM51245G_DC_6_LUT_INDEX_3      3U
#define MX25LM51245G_DC_LUT_INIT_INDEX    60U /* Arbitrary initial index value - will be updated at the first command */

typedef enum
{
  MX25LM51245G_OK = 0,
  MX25LM51245G_ERROR,
  MX25LM51245G_SUSPENDED,
  MX25LM51245G_BUSY
} mx25lm51245g_status_t;

/**********************  MX25LM51245G Commands ***********************/

/*******************************************************************/
/********************************* SPI  ****************************/
/*******************************************************************/

/***** READ/WRITE MEMORY Operations with 3-Byte Address ****************************/
#define MX25LM51245G_READ_CMD                             0x03U   /*!< Normal Read 3 Byte Address                     */
#define MX25LM51245G_FAST_READ_CMD                        0x0BU   /*!< Fast Read 3 Byte Address                       */
#define MX25LM51245G_PAGE_PROG_CMD                        0x02U   /*!< Page Program 3 Byte Address                    */
#define MX25LM51245G_SECTOR_ERASE_4K_CMD                  0x20U   /*!< Sector Erase 4KB 3 Byte Address                */
#define MX25LM51245G_BLOCK_ERASE_64K_CMD                  0xD8U   /*!< Block Erase 64KB 3 Byte Address                */
#define MX25LM51245G_BULK_ERASE_CMD                       0x60U   /*!< Bulk Erase                                     */

/***** READ/WRITE MEMORY Operations with 4-Byte Address ****************************/
#define MX25LM51245G_4_BYTE_ADDR_READ_CMD                 0x13U   /*!< Normal Read 4 Byte address                     */
#define MX25LM51245G_4_BYTE_ADDR_FAST_READ_CMD            0x0CU   /*!< Fast Read 4 Byte address                       */
#define MX25LM51245G_4_BYTE_PAGE_PROG_CMD                 0x12U   /*!< Page Program 4 Byte Address                    */
#define MX25LM51245G_4_BYTE_SECTOR_ERASE_4K_CMD           0x21U   /*!< Sector Erase 4KB 4 Byte Address                */
#define MX25LM51245G_4_BYTE_BLOCK_ERASE_64K_CMD           0xDCU   /*!< Block Erase 64KB 4 Byte Address                */

/***** Setting commands ************************************************************/
#define MX25LM51245G_WRITE_ENABLE_CMD                     0x06U   /*!< Write Enable                                   */
#define MX25LM51245G_WRITE_DISABLE_CMD                    0x04U   /*!< Write Disable                                  */
#define MX25LM51245G_PROG_ERASE_SUSPEND_CMD               0xB0U   /*!< Program/Erase suspend                          */
#define MX25LM51245G_PROG_ERASE_RESUME_CMD                0x30U   /*!< Program/Erase resume                           */
#define MX25LM51245G_ENTER_DEEP_POWER_DOWN_CMD            0xB9U   /*!< Enter deep power down                          */
#define MX25LM51245G_SET_BURST_LENGTH_CMD                 0xC0U   /*!< Set burst length                               */
#define MX25LM51245G_ENTER_SECURED_OTP_CMD                0xB1U   /*!< Enter secured OTP)                             */
#define MX25LM51245G_EXIT_SECURED_OTP_CMD                 0xC1U   /*!< Exit secured OTP)                              */

/***** RESET commands ************************************************************/
#define MX25LM51245G_NOP_CMD                              0x00U   /*!< No operation                                   */
#define MX25LM51245G_RESET_ENABLE_CMD                     0x66U   /*!< Reset Enable                                   */
#define MX25LM51245G_RESET_MEMORY_CMD                     0x99U   /*!< Reset Memory                                   */

/***** Register Commands (SPI) ****************************************************/
#define MX25LM51245G_READ_ID_CMD                          0x9FU   /*!< Read IDentification                            */
#define MX25LM51245G_READ_SERIAL_FLASH_DISCO_PARAM_CMD    0x5AU   /*!< Read Serial Flash Discoverable Parameter       */
#define MX25LM51245G_READ_STATUS_REG_CMD                  0x05U   /*!< Read Status Register                           */
#define MX25LM51245G_READ_CFG_REG_CMD                     0x15U   /*!< Read configuration Register                    */
#define MX25LM51245G_WRITE_STATUS_REG_CMD                 0x01U   /*!< Write Status Register                          */
#define MX25LM51245G_WRITE_CFG_REG_CMD                    0x01U   /*!< Write configuration Register                   */
#define MX25LM51245G_READ_CFG_REG2_CMD                    0x71U   /*!< Read configuration Register2                   */
#define MX25LM51245G_WRITE_CFG_REG2_CMD                   0x72U   /*!< Write configuration Register2                  */
#define MX25LM51245G_READ_FAST_BOOT_REG_CMD               0x16U   /*!< Read fast boot Register                        */
#define MX25LM51245G_WRITE_FAST_BOOT_REG_CMD              0x17U   /*!< Write fast boot Register                       */
#define MX25LM51245G_ERASE_FAST_BOOT_REG_CMD              0x18U   /*!< Erase fast boot Register                       */
#define MX25LM51245G_READ_SECURITY_REG_CMD                0x2BU   /*!< Read security Register                         */
#define MX25LM51245G_WRITE_SECURITY_REG_CMD               0x2FU   /*!< Write security Register                        */
#define MX25LM51245G_READ_LOCK_REG_CMD                    0x2DU   /*!< Read lock Register                             */
#define MX25LM51245G_WRITE_LOCK_REG_CMD                   0x2CU   /*!< Write lock Register                            */
#define MX25LM51245G_READ_DPB_REG_CMD                     0xE0U   /*!< Read DPB register                              */
#define MX25LM51245G_WRITE_DPB_REG_CMD                    0xE1U   /*!< Write DPB register                             */
#define MX25LM51245G_READ_SPB_STATUS_CMD                  0xE2U   /*!< Read SPB status                                */
#define MX25LM51245G_WRITE_SPB_BIT_CMD                    0xE3U   /*!< SPB bit program                                */
#define MX25LM51245G_ERASE_ALL_SPB_CMD                    0xE4U   /*!< Erase all SPB bit                              */
#define MX25LM51245G_WRITE_PROTECT_SEL_CMD                0x68U   /*!< Write Protect selection                        */
#define MX25LM51245G_GANG_BLOCK_LOCK_CMD                  0x7EU   /*!< Gang block lock: whole chip write protect      */
#define MX25LM51245G_GANG_BLOCK_UNLOCK_CMD                0x98U   /*!< Gang block unlock: whole chip write unprotect  */
#define MX25LM51245G_READ_PASSWORD_REGISTER_CMD           0x27U   /*!< Read Password                                  */
#define MX25LM51245G_WRITE_PASSWORD_REGISTER_CMD          0x28U   /*!< Write Password                                 */
#define MX25LM51245G_PASSWORD_UNLOCK_CMD                  0x29U   /*!< Unlock Password                                */


/*******************************************************************/
/********************************* OPI  ****************************/
/*******************************************************************/

/***** READ/WRITE MEMORY Operations  ****************************/
#define MX25LM51245G_OCTA_READ_CMD                             0xEC13U  /*!< Octa IO Read                             */
#define MX25LM51245G_OCTA_READ_DTR_CMD                         0xEE11U  /*!< Octa IO Read DTR                         */
#define MX25LM51245G_OCTA_PAGE_PROG_CMD                        0x12EDU  /*!< Octa Page Program                        */
#define MX25LM51245G_OCTA_SECTOR_ERASE_4K_CMD                  0x21DEU  /*!< Octa Sector Erase 4KB                    */
#define MX25LM51245G_OCTA_BLOCK_ERASE_64K_CMD                  0xDC23U  /*!< Octa Block Erase 64KB                    */
#define MX25LM51245G_OCTA_BULK_ERASE_CMD                       0x609FU  /*!< Octa Bulk Erase                          */

/***** Setting commands ************************************************************/
#define MX25LM51245G_OCTA_WRITE_ENABLE_CMD                     0x06F9U   /*!< Octa Write Enable                       */
#define MX25LM51245G_OCTA_WRITE_DISABLE_CMD                    0x04FBU   /*!< Octa Write Disable                      */
#define MX25LM51245G_OCTA_PROG_ERASE_SUSPEND_CMD               0xB04FU   /*!< Octa Program/Erase suspend              */
#define MX25LM51245G_OCTA_PROG_ERASE_RESUME_CMD                0x30CFU   /*!< Octa Program/Erase resume               */
#define MX25LM51245G_OCTA_ENTER_DEEP_POWER_DOWN_CMD            0xB946U   /*!< Octa Enter deep power down              */
#define MX25LM51245G_OCTA_SET_BURST_LENGTH_CMD                 0xC03FU   /*!< Octa Set burst length                   */
#define MX25LM51245G_OCTA_ENTER_SECURED_OTP_CMD                0xB14EU   /*!< Octa Enter secured OTP)                 */
#define MX25LM51245G_OCTA_EXIT_SECURED_OTP_CMD                 0xC13EU   /*!< Octa Exit secured OTP)                  */

/***** RESET commands ************************************************************/
#define MX25LM51245G_OCTA_NOP_CMD                              0x00FFU   /*!< Octa No operation                       */
#define MX25LM51245G_OCTA_RESET_ENABLE_CMD                     0x6699U   /*!< Octa Reset Enable                       */
#define MX25LM51245G_OCTA_RESET_MEMORY_CMD                     0x9966U   /*!< Octa Reset Memory                       */

/***** Register Commands (OPI) ****************************************************/
#define MX25LM51245G_OCTA_READ_ID_CMD                          0x9F60U   /*!< Octa Read IDentification                */
#define MX25LM51245G_OCTA_READ_SERIAL_FLASH_DISCO_PARAM_CMD    0x5AA5U   /*!< Octa Read Serial Flash Discoverable
                                                                              Parameter                               */
#define MX25LM51245G_OCTA_READ_STATUS_REG_CMD                  0x05FAU   /*!< Octa Read Status Register               */
#define MX25LM51245G_OCTA_READ_CFG_REG_CMD                     0x15EAU   /*!< Octa Read configuration Register        */
#define MX25LM51245G_OCTA_WRITE_STATUS_REG_CMD                 0x01FEU   /*!< Octa Write Status Register              */
#define MX25LM51245G_OCTA_WRITE_CFG_REG_CMD                    0x01FEU   /*!< Octa Write Configuration Register       */
#define MX25LM51245G_OCTA_READ_CFG_REG2_CMD                    0x718EU   /*!< Octa Read configuration Register2       */
#define MX25LM51245G_OCTA_WRITE_CFG_REG2_CMD                   0x728DU   /*!< Octa Write configuration Register2      */
#define MX25LM51245G_OCTA_READ_FAST_BOOT_REG_CMD               0x16E9U   /*!< Octa Read fast boot Register            */
#define MX25LM51245G_OCTA_WRITE_FAST_BOOT_REG_CMD              0x17E8U   /*!< Octa Write fast boot Register           */
#define MX25LM51245G_OCTA_ERASE_FAST_BOOT_REG_CMD              0x18E7U   /*!< Octa Erase fast boot Register           */
#define MX25LM51245G_OCTA_READ_SECURITY_REG_CMD                0x2BD4U   /*!< Octa Read security Register             */
#define MX25LM51245G_OCTA_WRITE_SECURITY_REG_CMD               0x2FD0U   /*!< Octa Write security Register            */
#define MX25LM51245G_OCTA_READ_LOCK_REG_CMD                    0x2DD2U   /*!< Octa Read lock Register                 */
#define MX25LM51245G_OCTA_WRITE_LOCK_REG_CMD                   0x2CD3U   /*!< Octa Write lock Register                */
#define MX25LM51245G_OCTA_READ_DPB_REG_CMD                     0xE01FU   /*!< Octa Read DPB register                  */
#define MX25LM51245G_OCTA_WRITE_DPB_REG_CMD                    0xE11EU   /*!< Octa Write DPB register                 */
#define MX25LM51245G_OCTA_READ_SPB_STATUS_CMD                  0xE21DU   /*!< Octa Read SPB status                    */
#define MX25LM51245G_OCTA_WRITE_SPB_BIT_CMD                    0xE31CU   /*!< Octa SPB bit program                    */
#define MX25LM51245G_OCTA_ERASE_ALL_SPB_CMD                    0xE41BU   /*!< Octa Erase all SPB bit                  */
#define MX25LM51245G_OCTA_WRITE_PROTECT_SEL_CMD                0x6897U   /*!< Octa Write Protect selection            */
#define MX25LM51245G_OCTA_GANG_BLOCK_LOCK_CMD                  0x7E81U   /*!< Octa Gang block lock: whole chip write
                                                                              protect                                 */
#define MX25LM51245G_OCTA_GANG_BLOCK_UNLOCK_CMD                0x9867U   /*!< Octa Gang block unlock: whole chip write
                                                                              unprotect                               */
#define MX25LM51245G_OCTA_READ_PASSWORD_REGISTER_CMD           0x27D8U   /*!< Octa Read Password                      */
#define MX25LM51245G_OCTA_WRITE_PASSWORD_REGISTER_CMD          0x28D7U   /*!< Octa Write Password                     */
#define MX25LM51245G_OCTA_PASSWORD_UNLOCK_CMD                  0x29D6U   /*!< Octa Unlock Password                    */

/**
  * @brief Macros defining the Dummy cycle
  */
#define MX25LM51245G_FAST_READ_DUMMY_CYCLES                 8U /*!< FAST READ (Fast Read Data)                        */
#define MX25LM51245G_PAGE_PROG_DUMMY_CYCLES                 0U /*!< PP (Page Program)                                 */
#define MX25LM51245G_SE_DUMMY_CYCLES                        0U /*!< SE3B (Sector Erase)                               */
#define MX25LM51245G_BE_DUMMY_CYCLES                        0U /*!< BE3B (Block Erase 64KB)                           */
#define MX25LM51245G_CE_DUMMY_CYCLES                        0U /*!< CE (Chip Erase)                                   */
#define MX25LM51245G_WREN_DUMMY_CYCLES                      0U /*!< WREN (Write Enable)                               */
#define MX25LM51245G_PGM_ERS_SUSPEND_DUMMY_CYCLES           0U /*!< PGM/ERS Suspend                                   */
#define MX25LM51245G_PGM_ERS_RESUME_DUMMY_CYCLES            0U /*!< PGM/ERS Resume                                    */
#define MX25LM51245G_DP_DUMMY_CYCLES                        0U /*!< DP (Deep Power Down)                              */
#define MX25LM51245G_RDP_DUMMY_CYCLES                       0U /*!< RDP (Release from Deep Power Down)                */
#define MX25LM51245G_RSTEN_DUMMY_CYCLES                     0U /*!< RSTEN (Reset Enable)                              */
#define MX25LM51245G_RST_DUMMY_CYCLES                       0U /*!< RST (Reset Memory)                                */
#define MX25LM51245G_RDID_DUMMY_CYCLES                      0U /*!< RDID (Read Identification)                        */
#define MX25LM51245G_RDCR_DUMMY_CYCLES                      0U /*!< RDCR (Read Configuration Register)                */
#define MX25LM51245G_RDSR_DUMMY_CYCLES                      0U /*!< RDSR (Read Status Register)                       */
#define MX25LM51245G_WRSR_WRCR_DUMMY_CYCLES                 0U /*!< WRSR/WRCR (Write Status/Configuration Register)   */
#define MX25LM51245G_RDCR2_DUMMY_CYCLES                     0U /*!< RDCR2 (Read Configuration Register 2)             */
#define MX25LM51245G_WRCR2_DUMMY_CYCLES                     0U /*!< WRCR2 (Write Configuration Register 2)            */
#define MX25LM51245G_RDSCUR_DUMMY_CYCLES                    0U /*!< RDSCUR (Read Security Register)                   */
#define MX25LM51245G_WRITE_STATUS_DUMMY_CYCLES              0U /*!< Write status command dummy cycles                 */
#define MX25LM51245G_OPI_RDSCUR_DUMMY_CYCLES                6U /*!< RDSCUR (Read Security Register)                   */
#define MX25LM51245G_OPI_RDSR_DUMMY_CYCLES                  4U /*!< RDSR (Read Status Register)                       */
#define MX25LM51245G_OPI_RDCR_DUMMY_CYCLES                  4U /*!< RDCR (Read Configuration Register)                */
#define MX25LM51245G_OPI_RDID_DUMMY_CYCLES                  4U /*!< RDID (Read Identification)                        */
#define MX25LM51245G_OPI_RDCR2_DUMMY_CYCLES                 4U /*!< RDCR2 (Read Configuration Register 2)             */

/********************  MX25LM51245G Registers  **************************/
/* Status Register */
#define MX25LM51245G_SR_WIP                               0x01U   /*!< Write in progress                              */
#define MX25LM51245G_SR_WEL                               0x02U   /*!< Write enable latch                             */
#define MX25LM51245G_SR_PB                                0x3CU   /*!< Block protected against program and erase
                                                                       operations                                     */

/* Configuration Register 1 */
#define MX25LM51245G_CR1_ODS                              0x07U   /*!< Output driver strength                         */
#define MX25LM51245G_CR1_TB                               0x08U   /*!< Top / bottom  selected                         */
#define MX25LM51245G_CR1_PBE                              0x10U   /*!< Preamble bit enable                            */

/* Configuration Register 2 */
/* Address : 0x00000000 */
#define MX25LM51245G_CR2_REG1_ADDR                        0x00000000U  /*!< CR2 register address 0x00000000           */
#define MX25LM51245G_CR2_SOPI                             0x01U        /*!< STR OPI Enable                            */
#define MX25LM51245G_CR2_DOPI                             0x02U        /*!< DTR OPI Enable                            */
/* Address : 0x00000200 */
#define MX25LM51245G_CR2_REG2_ADDR                        0x00000200U  /*!< CR2 register address 0x00000200           */
#define MX25LM51245G_CR2_DQSPRC                           0x01U        /*!< DTR DQS pre-cycle                         */
#define MX25LM51245G_CR2_DOS                              0x02U        /*!< DQS on STR mode                           */
/* Address : 0x00000300 */
#define MX25LM51245G_CR2_REG3_ADDR                        0x00000300U  /*!< CR2 register address 0x00000300           */
#define MX25LM51245G_CR2_DC                               0x07U        /*!< Dummy cycle                               */
#define MX25LM51245G_CR2_DC_20_CYCLES                     0x00U        /*!< 20 Dummy cycles                           */
#define MX25LM51245G_CR2_DC_18_CYCLES                     0x01U        /*!< 18 Dummy cycles                           */
#define MX25LM51245G_CR2_DC_16_CYCLES                     0x02U        /*!< 16 Dummy cycles                           */
#define MX25LM51245G_CR2_DC_14_CYCLES                     0x03U        /*!< 14 Dummy cycles                           */
#define MX25LM51245G_CR2_DC_12_CYCLES                     0x04U        /*!< 12 Dummy cycles                           */
#define MX25LM51245G_CR2_DC_10_CYCLES                     0x05U        /*!< 10 Dummy cycles                           */
#define MX25LM51245G_CR2_DC_8_CYCLES                      0x06U        /*!< 8 Dummy cycles                            */
#define MX25LM51245G_CR2_DC_6_CYCLES                      0x07U        /*!< 6 Dummy cycles                            */
/* Address : 0x00000400 */
#define MX25LM51245G_CR2_REG4_ADDR                        0x00000400U  /*!< CR2 register address 0x00000400           */
#define MX25LM51245G_CR2_ECS_2BIT_OR_DP                   0x00U        /*!< 2 bit error or double programmed          */
#define MX25LM51245G_CR2_ECS_1_2BIT_OR_DP                 0x01U        /*!< 1 or 2 bit error or double programmed     */
#define MX25LM51245G_CR2_ECS_2BIT                         0x02U        /*!< 2 bit error only                          */
#define MX25LM51245G_CR2_ECS_1_2BIT                       0x03U        /*!< 1 or 2 bit error only                     */
/* Address : 0x00000500 */
#define MX25LM51245G_CR2_REG5_ADDR                        0x00000500U  /*!< CR2 register address 0x00000500           */
#define MX25LM51245G_CR2_PPTSEL                           0x01U        /*!< Preamble pattern selection                */
/* Address : 0x00000800 */
#define MX25LM51245G_CR2_REG6_ADDR                        0x00000800U  /*!< CR2 register address 0x00000800(read-only)*/
#define MX25LM51245G_CR2_ECC_FAIL_ADDR_INVALID            0x0U        /*!< ECC failure address invalid (no fail)      */
#define MX25LM51245G_CR2_ECC_FAIL_ADDR_VALID              0x1U        /*!< ECC failure address valid                  */

/* Address : 0x40000000 */
#define MX25LM51245G_CR2_REG7_ADDR                        0x40000000U  /*!< CR2 register address 0x40000000           */
#define MX25LM51245G_CR2_DEFSOPI                          0x01U        /*!< Enable SOPI after power on reset          */
#define MX25LM51245G_CR2_DEFDOPI                          0x02U        /*!< Enable DOPI after power on reset          */

/* Security Register */
#define MX25LM51245G_SECR_SOI                             0x01U        /*!< Secured OTP indicator                     */
#define MX25LM51245G_SECR_LDSO                            0x02U        /*!< Lock-down secured OTP                     */
#define MX25LM51245G_SECR_PSB                             0x04U        /*!< Program suspend bit                       */
#define MX25LM51245G_SECR_ESB                             0x08U        /*!< Erase suspend bit                         */
#define MX25LM51245G_SECR_P_FAIL                          0x20U        /*!< Program fail flag                         */
#define MX25LM51245G_SECR_E_FAIL                          0x40U        /*!< Erase fail flag                           */
#define MX25LM51245G_SECR_WPSEL                           0x80U        /*!< Write protection selection                */

/* Memory initialization and configuration */
int32_t mx25lm51245g_io_init(mx25lm51245g_io_t *pio);
mx25lm51245g_status_t mx25lm51245g_init(mx25lm51245g_obj_t *pobj, uint32_t dev_id);
mx25lm51245g_status_t mx25lm51245g_deinit(mx25lm51245g_obj_t *pobj);
mx25lm51245g_status_t mx25lm51245g_set_memconfig(mx25lm51245g_obj_t *pobj, mx25lm51245g_interface_t mode,
                                                 mx25lm51245g_transfer_rate_t rate);

/* Memory information and status */
void mx25lm51245g_get_info(mx25lm51245g_obj_t const *pobj, mx25lm51245g_info_t *p_info);
mx25lm51245g_status_t mx25lm51245g_read_id(mx25lm51245g_obj_t *pobj, uint8_t *id);
mx25lm51245g_status_t mx25lm51245g_get_status(mx25lm51245g_obj_t *pobj, mx25lm51245g_status_t *p_status);
mx25lm51245g_status_t mx25lm51245g_read_status_register(mx25lm51245g_obj_t *pobj, uint8_t *p_value);
mx25lm51245g_status_t mx25lm51245g_read_security_register(mx25lm51245g_obj_t *pobj, uint8_t *p_value);
mx25lm51245g_status_t mx25lm51245g_get_xspi_memory_frequency(mx25lm51245g_obj_t *pobj, uint32_t *p_freq);
mx25lm51245g_status_t mx25lm51245g_get_memconfig(mx25lm51245g_obj_t *pobj, mx25lm51245g_memcfg_t *p_memcfg);
mx25lm51245g_transfer_rate_t mx25lm51245g_get_transfer_rate_cfg(mx25lm51245g_obj_t *pobj);
mx25lm51245g_interface_t mx25lm51245g_get_interface_cfg(mx25lm51245g_obj_t *pobj);

/* Memory erase */
mx25lm51245g_status_t mx25lm51245g_erase(mx25lm51245g_obj_t *pobj, uint32_t addr, mx25lm51245g_erase_t size);
mx25lm51245g_status_t mx25lm51245g_erase_it(mx25lm51245g_obj_t *pobj, uint32_t addr,
                                            mx25lm51245g_erase_t size_cfg);
mx25lm51245g_status_t mx25lm51245g_mass_erase(mx25lm51245g_obj_t *pobj);
mx25lm51245g_status_t mx25lm51245g_mass_erase_it(mx25lm51245g_obj_t *pobj);
mx25lm51245g_status_t mx25lm51245g_sectors_erase(mx25lm51245g_obj_t *pobj, uint32_t erase_addr, uint32_t size_byte);

/* Read/Write Array Commands */
mx25lm51245g_status_t mx25lm51245g_read(mx25lm51245g_obj_t *pobj, uint32_t addr, mx25lm51245g_addr_size_t addr_size,
                                        uint8_t *p_data, uint32_t size_byte);
mx25lm51245g_status_t mx25lm51245g_write(mx25lm51245g_obj_t *pobj, uint32_t addr, mx25lm51245g_addr_size_t addr_size,
                                         uint8_t *p_data, uint32_t size_byte);
#if defined (USE_HAL_XSPI_DMA) && (USE_HAL_XSPI_DMA == 1)
#if defined (MX25LM_CALLBACKS) && (MX25LM_CALLBACKS == 1)
mx25lm51245g_status_t mx25lm51245g_read_dma_async(mx25lm51245g_obj_t *pobj, uint32_t addr,
                                                  mx25lm51245g_addr_size_t addr_size,
                                                  uint8_t *p_data, uint32_t size_byte);
mx25lm51245g_status_t mx25lm51245g_write_dma_async(mx25lm51245g_obj_t *pobj, uint32_t addr,
                                                   mx25lm51245g_addr_size_t addr_size,
                                                   uint8_t *p_data, uint32_t size_byte);
mx25lm51245g_status_t mx25lm51245g_set_dma_buffer(mx25lm51245g_obj_t *pobj, uint8_t *buffer, uint32_t size);
#endif /* MX25LM_CALLBACKS */
#endif /* USE_HAL_XSPI_DMA */

/* Memory mapped control */
mx25lm51245g_status_t mx25lm51245g_enable_memory_mapped(mx25lm51245g_obj_t *pobj, mx25lm51245g_addr_size_t addr_size);
mx25lm51245g_status_t mx25lm51245g_disable_memory_mapped(mx25lm51245g_obj_t *pobj);
uint32_t mx25lm51245g_get_memory_mapped_base_address(mx25lm51245g_obj_t *pobj);

/* Memory reset/suspend/resume */
mx25lm51245g_status_t mx25lm51245g_reset(mx25lm51245g_obj_t *pobj);
#if defined (MX25LM_CALLBACKS) && (MX25LM_CALLBACKS == 1)
mx25lm51245g_status_t mx25lm51245g_suspend(mx25lm51245g_obj_t *pobj);
mx25lm51245g_status_t mx25lm51245g_resume(mx25lm51245g_obj_t *pobj);
#endif /* MX25LM_CALLBACKS */

/* Memory power down */
mx25lm51245g_status_t mx25lm51245g_enter_power_down(mx25lm51245g_obj_t *pobj);
mx25lm51245g_status_t mx25lm51245g_exit_power_down(mx25lm51245g_obj_t *pobj);

/* Memory block protection */
mx25lm51245g_status_t mx25lm51245g_set_block_protection_cfg(mx25lm51245g_obj_t *pobj, mx25lm51245g_bp_cfg_t bp_cfg);
mx25lm51245g_status_t mx25lm51245g_get_block_protection_cfg(mx25lm51245g_obj_t *pobj, mx25lm51245g_bp_cfg_t *p_bp_cfg);
mx25lm51245g_status_t mx25lm51245g_set_top_bottom_cfg(mx25lm51245g_obj_t *pobj, mx25lm51245g_bp_otp_tb_cfg_t tb_cfg);
mx25lm51245g_status_t mx25lm51245g_get_top_bottom_cfg(mx25lm51245g_obj_t *pobj, mx25lm51245g_bp_otp_tb_cfg_t *p_tb_cfg);

#if defined (MX25LM_CALLBACKS) && (MX25LM_CALLBACKS == 1)
mx25lm51245g_async_operation_t mx25lm51245g_get_async_operation_type(mx25lm51245g_obj_t *pobj);
mx25lm51245g_status_t mx25lm51245g_register_mass_erase_cplt_callback(mx25lm51245g_obj_t *pobj,
                                                                     mx25lm51245g_callback_t cb, void *p_arg);
mx25lm51245g_status_t mx25lm51245g_register_erase_cplt_callback(mx25lm51245g_obj_t *pobj,
                                                                mx25lm51245g_callback_t cb, void *p_arg);
#if defined (USE_HAL_XSPI_DMA) && (USE_HAL_XSPI_DMA == 1)
mx25lm51245g_async_write_phase_t mx25lm51245g_get_async_write_phase(mx25lm51245g_obj_t *pobj);
mx25lm51245g_async_read_phase_t mx25lm51245g_get_async_read_phase(mx25lm51245g_obj_t *pobj);
mx25lm51245g_status_t mx25lm51245g_register_read_cplt_callback(mx25lm51245g_obj_t *pobj, mx25lm51245g_callback_t cb,
                                                               void *p_arg);
mx25lm51245g_status_t mx25lm51245g_register_write_cplt_callback(mx25lm51245g_obj_t *pobj, mx25lm51245g_callback_t cb,
                                                                void *p_arg);
mx25lm51245g_status_t mx25lm51245g_exec_data_handler(mx25lm51245g_obj_t *pobj);
#endif /* USE_HAL_XSPI_DMA */
#endif /* MX25LM_CALLBACKS */

#ifdef __cplusplus
}
#endif

#endif /* MX25LM51245G_H */
