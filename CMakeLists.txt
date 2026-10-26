# origin-pack: STMicroelectronics::mx25lm51245g_part_drivers@2.0.0
# file-format: 1.0.0
project(STMicroelectronics_mx25lm51245g_part_drivers_2_0_0)
cmake_minimum_required(VERSION 3.30)
add_library(STMicroelectronics_mx25lm51245g_part_drivers_2_0_0 INTERFACE)

# List of all CMSIS properties that influence conditions for this package


# Device specific defined symbols








# Enable all components in this package
if(CMSIS_ENTIRE_STMicroelectronics_mx25lm51245g_part_drivers_2_0_0)
  list(APPEND CMSIS_COMPONENTS_LIST "Cvendor:STMicroelectronics#Cclass:Board Part#Cgroup:Memory#Csub:MX25LM51245G Core#Cversion:1.0.0")
endif()

# All conditions used by this package

# condition: STMicroelectronics.mx25lm51245g_part_drivers.2.0.0:MX25LM51245G CFG Condition
# description: STMicroelectronics configuration for MX25LM51245G part driver
set(STMicroelectronics.mx25lm51245g_part_drivers.2.0.0_MX25LM51245G_CFG_Condition "1")
message(DEBUG "CMSIS condition STMicroelectronics.mx25lm51245g_part_drivers.2.0.0_MX25LM51245G_CFG_Condition enabled")


# condition: STMicroelectronics.mx25lm51245g_part_drivers.2.0.0:MX25LM51245G Core Condition
# description: STMicroelectronics MX25LM51245G Core Driver
set(STMicroelectronics.mx25lm51245g_part_drivers.2.0.0_MX25LM51245G_Core_Condition "$<NOT:$<STREQUAL:$<FILTER:${CMSIS_COMPONENTS_LIST},INCLUDE,.*Cclass:Device#.*Cgroup:STM32 HAL#.*Csub:XSPI(#.*|$)>,>>")
message(DEBUG "CMSIS condition STMicroelectronics.mx25lm51245g_part_drivers.2.0.0_MX25LM51245G_Core_Condition enabled")

# Files and components in this package
if("Cvendor:STMicroelectronics#Cclass:Board Part#Cgroup:Memory#Csub:MX25LM51245G Core#Cversion:1.0.0" IN_LIST CMSIS_COMPONENTS_LIST)  # Driver for MX25LM51245G OCTOSPI memory
  message(DEBUG "Using component Board_Part_Memory_MX25LM51245G_Core_1_0_0")
  target_compile_definitions(STMicroelectronics_mx25lm51245g_part_drivers_2_0_0 INTERFACE "$<${STMicroelectronics.mx25lm51245g_part_drivers.2.0.0_MX25LM51245G_Core_Condition}:-DCMSIS_USE_Board_Part_Memory_MX25LM51245G_Core_1_0_0=1>")
  target_include_directories(STMicroelectronics_mx25lm51245g_part_drivers_2_0_0 INTERFACE "$<${STMicroelectronics.mx25lm51245g_part_drivers.2.0.0_MX25LM51245G_Core_Condition}:${CMAKE_CURRENT_LIST_DIR}/.>")
  target_sources(STMicroelectronics_mx25lm51245g_part_drivers_2_0_0 INTERFACE "$<${STMicroelectronics.mx25lm51245g_part_drivers.2.0.0_MX25LM51245G_Core_Condition}:${CMAKE_CURRENT_LIST_DIR}/mx25lm51245g.c>")
endif()

