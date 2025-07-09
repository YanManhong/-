#===============================================================================
# @brief    cmake file
# Copyright (c) CompanyNameMagicTag 2023-2023. All rights reserved.
#===============================================================================
if(DEFINED CONFIG_SAMPLE_SUPPORT_SLE_UART_SERVER)
set(SOURCES_LIST
    ${CMAKE_CURRENT_SOURCE_DIR}/sle_uart_server/sle_uart_server_adv.c
    ${CMAKE_CURRENT_SOURCE_DIR}/sle_uart_server/sle_uart_server.c
    ${CMAKE_CURRENT_SOURCE_DIR}/sle_uart.c
    ${CMAKE_CURRENT_SOURCE_DIR}/bh1750.c
    ${CMAKE_CURRENT_SOURCE_DIR}/INA219.c
    ${CMAKE_CURRENT_SOURCE_DIR}/aht20_test.c
    ${CMAKE_CURRENT_SOURCE_DIR}/aht20.c
    ${CMAKE_CURRENT_SOURCE_DIR}/ssd1306_fonts.c
    ${CMAKE_CURRENT_SOURCE_DIR}/ssd1306.c
    ${CMAKE_CURRENT_SOURCE_DIR}/wifi_connect.c
)
set(HEADER_LIST ${CMAKE_CURRENT_SOURCE_DIR}/sle_uart_server)
elseif(DEFINED CONFIG_SAMPLE_SUPPORT_SLE_UART_CLIENT)
set(SOURCES_LIST
    ${CMAKE_CURRENT_SOURCE_DIR}/sle_uart_client/sle_uart_client.c
    ${CMAKE_CURRENT_SOURCE_DIR}/sle_uart.c
    ${CMAKE_CURRENT_SOURCE_DIR}/ssd1306_fonts.c
    ${CMAKE_CURRENT_SOURCE_DIR}/ssd1306.c
)
set(HEADER_LIST ${CMAKE_CURRENT_SOURCE_DIR}/sle_uart_client)
endif()

set(SOURCES "${SOURCES}" ${SOURCES_LIST} PARENT_SCOPE)
set(PUBLIC_HEADER "${PUBLIC_HEADER}" ${HEADER_LIST} PARENT_SCOPE)