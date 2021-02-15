/**
 * Copyright (c) 2014 - 2018, Nordic Semiconductor ASA
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 * 
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * 
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 * 
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 * 
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */
/** @file
 *
 * @defgroup ble_sdk_app_beacon_main main.c
 * @{
 * @ingroup ble_sdk_app_beacon
 * @brief Beacon Transmitter Sample Application main file.
 *
 * This file contains the source code for an Beacon transmitter sample application.
 */


// Include the necessary libraries
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "nordic_common.h"
#include "bsp.h"
#include "nrf_soc.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "ble_advdata.h"
#include "app_timer.h"
#include "nrf_pwr_mgmt.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

// The libraries for UART are added to the example project
#include "app_uart.h"
#include "app_error.h"
#include "nrf_delay.h"
#include "nrf.h"
#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif

#include "app_fifo.h"


// Define the necessary macros
#define DEVICE_NAME                   "Comodule_Task"                   /**< The name of the Bluetooth device */
#define GPS_DATA_SIZE                 27                                /**< The GPS coordinate character number. The coordinates are in the form of DD°MM'SS.S"N/S, DDD°MM'SS.S"E/W */

#define APP_BLE_CONN_CFG_TAG          1                                 /**< A tag identifying the SoftDevice BLE configuration. */

#define NON_CONNECTABLE_ADV_INTERVAL  MSEC_TO_UNITS(100, UNIT_0_625_MS) /**< The advertising interval for non-connectable advertisement (100 ms). This value can vary between 100ms to 10.24s). */

#define APP_BEACON_INFO_LENGTH        0x17                              /**< Total length of information advertised by the Beacon. */
#define APP_ADV_DATA_LENGTH           0x15                              /**< Length of manufacturer specific data in the advertisement. */
#define APP_DEVICE_TYPE               0x02                              /**< 0x02 refers to Beacon. */
#define APP_MEASURED_RSSI             0xC3                              /**< The Beacon's measured RSSI at 1 meter distance in dBm. */
#define APP_COMPANY_IDENTIFIER        0x0059                            /**< Company identifier for Nordic Semiconductor ASA. as per www.bluetooth.org. */
#define APP_MAJOR_VALUE               0x01, 0x02                        /**< Major value used to identify Beacons. */
#define APP_MINOR_VALUE               0x03, 0x04                        /**< Minor value used to identify Beacons. */
#define APP_BEACON_UUID               0x01, 0x12, 0x23, 0x34, \
                                      0x45, 0x56, 0x67, 0x78, \
                                      0x89, 0x9a, 0xab, 0xbc, \
                                      0xcd, 0xde, 0xef, 0xf0            /**< Proprietary UUID for Beacon. */

#define MAX_TEST_DATA_BYTES           (15U)                             /**< Max number of test bytes to be used for TX and RX. */
#define UART_TX_BUF_SIZE              256                               /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE              256                               /**< UART RX buffer size. */

#define UART_HWFC APP_UART_FLOW_CONTROL_DISABLED                        /**< When UART is used for communication with the host do not use flow control.*/

#define DEAD_BEEF                     0xDEADBEEF                        /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#if defined(USE_UICR_FOR_MAJ_MIN_VALUES)
#define MAJ_VAL_OFFSET_IN_BEACON_INFO 18                                /**< Position of the MSB of the Major Value in m_beacon_info array. */
#define UICR_ADDRESS                  0x10001080                        /**< Address of the UICR register used by this example. The major and minor versions to be encoded into the advertising data will be picked up from this location. */
#endif


// Global variables and structs
static ble_gap_adv_params_t m_adv_params;                                  /**< Parameters to be passed to the stack when starting advertising. */
static uint8_t              m_adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET; /**< Advertising handle used to identify an advertising set. */
static uint8_t              m_enc_advdata[BLE_GAP_ADV_SET_DATA_SIZE_MAX];  /**< Buffer for storing an encoded advertising set. */
static uint8_t              m_enc_srdata[BLE_GAP_ADV_SET_DATA_SIZE_MAX];   /**< Buffer for storing an encoded scan set. */
static uint8_t              *data;                                         /**< Pointer to the char array holding the GPS coordinates to be transmitted */

/**@brief Struct that contains pointers to the encoded advertising data. */
static ble_gap_adv_data_t m_adv_data =
{
    .adv_data =
    {
        .p_data = m_enc_advdata,
        .len    = BLE_GAP_ADV_SET_DATA_SIZE_MAX
    },
    .scan_rsp_data =
    {
        .p_data = m_enc_srdata,
        .len    = BLE_GAP_ADV_SET_DATA_SIZE_MAX
    }
};

static uint8_t m_beacon_info[APP_BEACON_INFO_LENGTH] =  /**< Information advertised by the Beacon. */
{
    APP_DEVICE_TYPE,     // Manufacturer specific information. Specifies the device type in this implementation.
    APP_ADV_DATA_LENGTH, // Manufacturer specific information. Specifies the length of the manufacturer specific data in this implementation.
    APP_BEACON_UUID,     // 128 bit UUID value.
    APP_MAJOR_VALUE,     // Major arbitrary value that can be used to distinguish between Beacons.
    APP_MINOR_VALUE,     // Minor arbitrary value that can be used to distinguish between Beacons.
    APP_MEASURED_RSSI    // Manufacturer specific information. The Beacon's measured TX power in this implementation.
};


// Function declarations
/**@brief Function for UART Error Handling.
 *
 * @details Raises the error handler during a communication or FIFO error.
 */
void uart_error_handle(app_uart_evt_t * p_event)
{
    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_communication);
    }
    else if (p_event->evt_type == APP_UART_FIFO_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_code);
    }
}

/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in]   line_num   Line number of the failing ASSERT call.
 * @param[in]   file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/**@brief Function for counting the number of characters in the char array.
 *
 * @details The function is called inside advertising_init() to calculate
 *          the number of characters inside the array pointed by data.
 *          This array includes the advertising data.
 *
 * @warning If a NULL pointer is inputted, the function will return 0.
 *
 * @warning If the number of characters in the inputted array is larger than 47, 
 *          the function will return 47 as the size.
 *
 * @param[in]   *string   The pointer pointing to the array having the GPS coordinates to be advertised.
 * @param[out]  charCount The number of the characters in the array.
 */
static int size_of_data(uint8_t *string)
{
  int charCount = 0; // The counter holding the character number
  
  // If the pointer is NULL, return 0
  if (!string)
  {
    return 0; // A non-existing string has `0` length
  }

  // The counting loop until '\0' is detected
  while (*string++) 
  {
    charCount++;

    // Do not count more than 47 characters
    if (charCount == 47)
    { 
      break;
    }
  }
  return charCount;
}


/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(void)
{
    uint32_t      err_code;
    ble_advdata_t advdata;

    int data_size = 0;  // Holds the number of characters returned by size_of_data()
    
    ble_advdata_t srdata;

    uint8_t       flags = BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED;

    ble_advdata_manuf_data_t manuf_specific_data;

    manuf_specific_data.company_identifier = APP_COMPANY_IDENTIFIER;

#if defined(USE_UICR_FOR_MAJ_MIN_VALUES)
    // If USE_UICR_FOR_MAJ_MIN_VALUES is defined, the major and minor values will be read from the
    // UICR instead of using the default values. The major and minor values obtained from the UICR
    // are encoded into advertising data in big endian order (MSB First).
    // To set the UICR used by this example to a desired value, write to the address 0x10001080
    // using the nrfjprog tool. The command to be used is as follows.
    // nrfjprog --snr <Segger-chip-Serial-Number> --memwr 0x10001080 --val <your major/minor value>
    // For example, for a major value and minor value of 0xabcd and 0x0102 respectively, the
    // the following command should be used.
    // nrfjprog --snr <Segger-chip-Serial-Number> --memwr 0x10001080 --val 0xabcd0102
    uint16_t major_value = ((*(uint32_t *)UICR_ADDRESS) & 0xFFFF0000) >> 16;
    uint16_t minor_value = ((*(uint32_t *)UICR_ADDRESS) & 0x0000FFFF);

    uint8_t index = MAJ_VAL_OFFSET_IN_BEACON_INFO;

    m_beacon_info[index++] = MSB_16(major_value);
    m_beacon_info[index++] = LSB_16(major_value);

    m_beacon_info[index++] = MSB_16(minor_value);
    m_beacon_info[index++] = LSB_16(minor_value);
#endif

    manuf_specific_data.company_identifier = APP_COMPANY_IDENTIFIER;  // Nordics company ID
    manuf_specific_data.data.p_data = (uint8_t *) data;               // GPS coordinates coming from UART
    manuf_specific_data.data.size = 24;                               // The first 24 characters are advertised here

    // Build and set advertising data.
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type             = BLE_ADVDATA_NO_NAME;
    advdata.flags                 = flags;
    advdata.p_manuf_specific_data = &manuf_specific_data;

    // Build and set scan response data.
    memset(&srdata, 0, sizeof(srdata));
    srdata.name_type             = BLE_ADVDATA_FULL_NAME;

    // Initialize advertising parameters (used when starting advertising).
    memset(&m_adv_params, 0, sizeof(m_adv_params));

    m_adv_params.properties.type = BLE_GAP_ADV_TYPE_NONCONNECTABLE_SCANNABLE_UNDIRECTED; //Beacon needs to be scannable to receive scan response
    m_adv_params.p_peer_addr     = NULL;    // Undirected advertisement.
    m_adv_params.filter_policy   = BLE_GAP_ADV_FP_ANY;
    m_adv_params.interval        = NON_CONNECTABLE_ADV_INTERVAL;
    m_adv_params.duration        = 0;       // Never time out.

    data_size = size_of_data(data); // Measure the data size

    // Prepare the scan response manufacturer specific data packet
    ble_advdata_manuf_data_t  manuf_data_response;
    manuf_data_response.company_identifier  = APP_COMPANY_IDENTIFIER;
    manuf_data_response.data.p_data         = (uint8_t *) (data + 24);  // The rest of the characters are advertised here
    manuf_data_response.data.size           = data_size - 24 + 1;       // The number of remaining characters including '\0'
    srdata.p_manuf_specific_data = &manuf_data_response;
    

    err_code = ble_advdata_encode(&advdata, m_adv_data.adv_data.p_data, &m_adv_data.adv_data.len);
    APP_ERROR_CHECK(err_code);

    err_code = ble_advdata_encode(&srdata, m_adv_data.scan_rsp_data.p_data, &m_adv_data.scan_rsp_data.len);
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_adv_set_configure(&m_adv_handle, &m_adv_data, &m_adv_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    ret_code_t err_code;

    err_code = sd_ble_gap_adv_start(m_adv_handle, APP_BLE_CONN_CFG_TAG);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing LEDs. */
static void leds_init(void)
{
    ret_code_t err_code = bsp_init(BSP_INIT_LEDS, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing timers. */
static void timers_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}


/**
 * @brief Function for application main entry.
 */
int main(void)
{
    uint32_t err_code_uart;                                                   // Holds UART Error Code
    uint8_t coordinates[GPS_DATA_SIZE + 1] = "59°24'40.5\"N, 24°44'31.3\"E";  // UART data is taken to this local variable, initialized with default coordinates
    uint8_t correct_input = 0;                                                // Indicates if the user input is formatted correctly
    bsp_board_init(BSP_INIT_LEDS);

    const app_uart_comm_params_t comm_params =
      {
          RX_PIN_NUMBER,
          TX_PIN_NUMBER,
          RTS_PIN_NUMBER,
          CTS_PIN_NUMBER,
          UART_HWFC,
          false,
#if defined (UART_PRESENT)
          NRF_UART_BAUDRATE_115200
#else
          NRF_UARTE_BAUDRATE_115200
#endif
      };
    
    // Initialize.
    timers_init();
    leds_init();
    power_management_init();
    ble_stack_init();
    
    APP_UART_FIFO_INIT(&comm_params,
                         UART_RX_BUF_SIZE,
                         UART_TX_BUF_SIZE,
                         uart_error_handle,
                         APP_IRQ_PRIORITY_LOWEST,
                         err_code_uart);
    
    APP_ERROR_CHECK(err_code_uart);

    ret_code_t err_code;
    ble_gap_conn_sec_mode_t sec_mode;
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                     (const uint8_t *)DEVICE_NAME,
                                      strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    data = malloc(sizeof(uint8_t) * (GPS_DATA_SIZE + 1)); // Our data to advertise, allocate 28 bytes for advertising data
    
    // Inform the user on how to input the coordinates
    printf("\r\nUART module started.\r\n");

    while (!correct_input)
    {
        printf("Enter the GPS coordinates\r\n");
        printf("Accepted format is DD°MM'SS.S\"N/S, DDD°MM'SS.S\"E/W\r\n");
        printf("An example of the expected format is\r\n");
        printf("59°24'40.5\"N, 24°44'31.3\"E\r\n");

        uint8_t cr; // Holds the character coming from UART
        // Loop to obtain each character coming from UART, at most 27 character can be obtained (GPS_DATA_SIZE)
        for (int i = 0; i < GPS_DATA_SIZE; i++)
        {
          while (app_uart_get(&cr) != NRF_SUCCESS); // Get the character
          coordinates[i] = cr;                      // Put the character to the coordinates char array

          // If the user ends the coordinates before 27 characters are reached, end the loop
          // This can happen when the longitude degree is smaller than 100
          if ((cr == 'E') || (cr == 'W'))
          {
            break;
          }
        }

        // Check the validity of the input
        if ((coordinates[2] == '°') && (coordinates[5] == '\'') && (coordinates[8] == '.') && (coordinates[10] == '"') &&
            ((coordinates[11] == 'N') || (coordinates[11] == 'S')) && (coordinates[12] == ',') && (coordinates[13] == ' ') &&
            (((coordinates[16] == '°') && (coordinates[19] == '\'') && (coordinates[22] == '.') && (coordinates[24] == '"') && 
            ((coordinates[25] == 'E') || (coordinates[25] == 'W'))) || ((coordinates[17] == '°') && (coordinates[20] == '\'') && 
            (coordinates[23] == '.') && (coordinates[25] == '"') && ((coordinates[26] == 'E') || (coordinates[26] == 'W')))))
        {
            correct_input = 1;
        }
        else
        {
            printf("\nGPS coordinates are not inputted correctly. Please give an input according to the format.\r\n");
        }
    }

    data = coordinates; // Assign the obtained GPS data to the global variable to pass to advertising_init()
    
    // Inform the user about their input
    printf("\r\nYou have entered the following coordinates:\r\n");
    printf("%s\r\n", data);
    

    advertising_init();

    // Start execution.
    NRF_LOG_INFO("Beacon module started.");
    printf("Beacon module started\r\n");
    printf("Advertising data...\r\n");
    advertising_start();

    // Enter main loop.
    for (;; )
    {
        idle_state_handle();
    }
}


/**
 * @}
 */