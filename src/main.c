/**
 * @brief 
 * @author: Gilfredo Remon Salazar, remong89@gmail.com
 *          0725148080
 *
 * This file contains the source code for a application to scan VOI Electrical scooters via BLE.
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf_sdm.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_db_discovery.h"
#include "ble_srv_common.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"
#include "nrf_pwr_mgmt.h"
#include "app_util.h"
#include "app_error.h"
#include "peer_manager.h"
#include "peer_manager_handler.h"
#include "ble_hrs_c.h"
#include "ble_bas_c.h"
#include "app_util.h"
#include "app_timer.h"
#include "bsp_btn_ble.h"
#include "fds.h"
#include "nrf_fstorage.h"
#include "ble_conn_state.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_lesc.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_ble_scan.h"

#include "blinker.h"
#include "logging.h"
#include "interface.h"

#define APP_BLE_CONN_CFG_TAG        1                                   /**< A tag identifying the SoftDevice BLE configuration. */
#define APP_IPSP_TAG                50

#define APP_BLE_OBSERVER_PRIO       3                                   /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_SOC_OBSERVER_PRIO       1                                   /**< Applications' SoC observer priority. You shouldn't need to modify this value. */

#define LESC_DEBUG_MODE             0                                   /**< Set to 1 to use LESC debug keys, allows you to use a sniffer to inspect traffic. */

#define SEC_PARAM_BOND              1                                   /**< Perform bonding. */
#define SEC_PARAM_MITM              0                                   /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC              1                                   /**< LE Secure Connections enabled. */
#define SEC_PARAM_KEYPRESS          0                                   /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES   BLE_GAP_IO_CAPS_NONE                /**< No I/O capabilities. */
#define SEC_PARAM_OOB               0                                   /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE      7                                   /**< Minimum encryption key size in octets. */
#define SEC_PARAM_MAX_KEY_SIZE      16                                  /**< Maximum encryption key size in octets. */

#define MIN_CONNECTION_INTERVAL             MSEC_TO_UNITS(30, UNIT_1_25_MS)                         /**< Determines maximum connection interval in millisecond. */
#define MAX_CONNECTION_INTERVAL             MSEC_TO_UNITS(60, UNIT_1_25_MS)                         /**< Determines maximum connection interval in millisecond. */
#define SLAVE_LATENCY                       0                                                       /**< Determines slave latency in counts of connection events. */
#define SUPERVISION_TIMEOUT                 MSEC_TO_UNITS(4000, UNIT_10_MS)  

#define SCAN_DURATION_WITELIST      3000                                /**< Duration of the scanning in units of 10 milliseconds. */

#define TARGET_UUID                 BLE_UUID_DEVICE_INFORMATION_SERVICE         /**< Target device uuid that application is looking for. */

/**@brief Macro to unpack 16bit unsigned UUID from octet stream. */
#define UUID16_EXTRACT(DST, SRC) \
    do                           \
    {                            \
        (*(DST))   = (SRC)[1];   \
        (*(DST)) <<= 8;          \
        (*(DST))  |= (SRC)[0];   \
    } while (0)


NRF_BLE_GQ_DEF(m_ble_gatt_queue,                                    /**< BLE GATT Queue instance. */
               NRF_SDH_BLE_CENTRAL_LINK_COUNT,
               NRF_BLE_GQ_QUEUE_SIZE);
BLE_HRS_C_DEF(m_hrs_c);                                             /**< Structure used to identify the heart rate client module. */
BLE_BAS_C_DEF(m_bas_c);                                             /**< Structure used to identify the Battery Service client module. */
NRF_BLE_GATT_DEF(m_gatt);                                           /**< GATT module instance. */
BLE_DB_DISCOVERY_DEF(m_db_disc);                                    /**< DB discovery module instance. */
NRF_BLE_SCAN_DEF(m_scan);                                           /**< Scanning module instance. */

static uint16_t m_conn_handle;                                      /**< Current connection handle. */
static bool     m_whitelist_disabled;                               /**< True if whitelist has been temporarily disabled. */
static bool     m_memory_access_in_progress;                        /**< Flag to keep track of ongoing operations on persistent memory. */

/**< Scan parameters requested for scanning and connection. */
static ble_gap_scan_params_t const m_scan_param =
{
    .active        = 0x01,
    .interval      = NRF_BLE_SCAN_SCAN_INTERVAL,
    .window        = NRF_BLE_SCAN_SCAN_WINDOW,
    .filter_policy = BLE_GAP_SCAN_FP_WHITELIST,
    .timeout       = SCAN_DURATION_WITELIST,
    .scan_phys     = BLE_GAP_PHY_1MBPS,
};

/**@brief Names which the central applications will scan for, and which will be advertised by the peripherals.
 *  if these are set to empty strings, the UUIDs defined below will be used
 */
static char const m_target_periph_name[] = "Voi_Scooter_IoT";      /**< If you want to connect to a peripheral using a given advertising name, type its name here. */
static bool is_connect_per_addr = false;            /**< If you want to connect to a peripheral with a given address, set this to true and put the correct address in the variable below. */

static ble_gap_addr_t const m_target_periph_addr =
{
    /* Possible values for addr_type:
       BLE_GAP_ADDR_TYPE_PUBLIC,
       BLE_GAP_ADDR_TYPE_RANDOM_STATIC,
       BLE_GAP_ADDR_TYPE_RANDOM_PRIVATE_RESOLVABLE,
       BLE_GAP_ADDR_TYPE_RANDOM_PRIVATE_NON_RESOLVABLE. */
    .addr_type = BLE_GAP_ADDR_TYPE_RANDOM_STATIC,
    .addr      = {0x8D, 0xFE, 0x23, 0x86, 0x77, 0xD9}
};


static const ble_gap_conn_params_t m_connection_param =
{
    .min_conn_interval = (uint16_t)MIN_CONNECTION_INTERVAL,   // Minimum connection
    .max_conn_interval = (uint16_t)MAX_CONNECTION_INTERVAL,   // Maximum connection
    .slave_latency     = 0,                                   // Slave latency
    .conn_sup_timeout  = (uint16_t)SUPERVISION_TIMEOUT        // Supervision time-out
};


static void scan_start(void);


/**@brief Function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num     Line number of the failing ASSERT call.
 * @param[in] p_file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(0xDEADBEEF, line_num, p_file_name);
}


/**@brief Function for handling the Heart Rate Service Client and Battery Service Client errors.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void service_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for handling database discovery events.
 *
 * @details This function is callback function to handle events from the database discovery module.
 *          Depending on the UUIDs that are discovered, this function should forward the events
 *          to their respective services.
 *
 * @param[in] p_event  Pointer to the database discovery event.
 */
static void db_disc_handler(ble_db_discovery_evt_t * p_evt)
{
    ble_hrs_on_db_disc_evt(&m_hrs_c, p_evt);
    ble_bas_on_db_disc_evt(&m_bas_c, p_evt);
}


/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const * p_evt)
{
    pm_handler_on_pm_evt(p_evt);
    pm_handler_flash_clean(p_evt);

    switch (p_evt->evt_id)
    {
        case PM_EVT_PEERS_DELETE_SUCCEEDED:
            // Bonds are deleted. Start scanning.
            scan_start();
            break;

        default:
            break;
    }
}


/**
 * @brief Function for shutdown events.
 *
 * @param[in]   event       Shutdown type.
 */
static bool shutdown_handler(nrf_pwr_mgmt_evt_t event)
{
    ret_code_t err_code;

    err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    switch (event)
    {
        case NRF_PWR_MGMT_EVT_PREPARE_WAKEUP:
            // Prepare wakeup buttons.
            err_code = bsp_btn_ble_sleep_mode_prepare();
            APP_ERROR_CHECK(err_code);
            break;
        default:
            break;
    }

    return true;
}

NRF_PWR_MGMT_HANDLER_REGISTER(shutdown_handler, APP_SHUTDOWN_HANDLER_PRIORITY);


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t            err_code;
    ble_gap_evt_t const * p_gap_evt = &p_ble_evt->evt.gap_evt;

    switch (p_ble_evt->header.evt_id)
    {
#if ADDRESS_SUPPORTED
        case BLE_GAP_EVT_ADV_REPORT:
            if (get_state() == SCANNING)
            {
                if (strcmp(p_ble_evt->evt.gap_evt.params.adv_report.data.p_data, (uint8_t*)(m_target_periph_name)))
                {
                    if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
                    {
                        err_code = sd_ble_gap_connect(&p_ble_evt->evt.gap_evt.params.adv_report.peer_addr,
                                                      &m_scan_param,
                                                      &m_connection_param,
                                                      APP_IPSP_TAG);
                        APP_ERROR_CHECK(err_code);
                    }
                }
            }
            break;
#endif

        case BLE_GAP_EVT_CONNECTED:
        {
            NRF_LOG_INFO("Connected.");
            if (get_state() == SCANNING)
            {
                ble_gap_addr_t  addr;
                memcpy(addr.addr, p_ble_evt->evt.gap_evt.params.connected.peer_addr.addr, 6);
                uint16_t const conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
                insert_address_if_not_available(addr.addr);
                sd_ble_gap_disconnect(conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            }
            scan_start();
        } break;

        case BLE_GAP_EVT_DISCONNECTED:
        {
            NRF_LOG_INFO("Disconnected, reason 0x%x.",
                         p_gap_evt->params.disconnected.reason);

            err_code = bsp_indication_set(BSP_INDICATE_IDLE);
            APP_ERROR_CHECK(err_code);

            if (ble_conn_state_central_conn_count() < NRF_SDH_BLE_CENTRAL_LINK_COUNT)
            {
                scan_start();
            }
        } break;

        case BLE_GAP_EVT_TIMEOUT:
        {
            if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN)
            {
                NRF_LOG_INFO("Connection Request timed out.");
            }
        } break;

        case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
            // Accepting parameters requested by peer.
            err_code = sd_ble_gap_conn_param_update(p_gap_evt->conn_handle,
                                                    &p_gap_evt->params.conn_param_update_request.conn_params);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;
    
        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            NRF_LOG_DEBUG("BLE_GAP_EVT_SEC_PARAMS_REQUEST");
            break;

        case BLE_GAP_EVT_AUTH_KEY_REQUEST:
            NRF_LOG_INFO("BLE_GAP_EVT_AUTH_KEY_REQUEST");
            break;

        case BLE_GAP_EVT_LESC_DHKEY_REQUEST:
            NRF_LOG_INFO("BLE_GAP_EVT_LESC_DHKEY_REQUEST");
            break;

         case BLE_GAP_EVT_AUTH_STATUS:
             NRF_LOG_INFO("BLE_GAP_EVT_AUTH_STATUS: status=0x%x bond=0x%x lv4: %d kdist_own:0x%x kdist_peer:0x%x",
                          p_ble_evt->evt.gap_evt.params.auth_status.auth_status,
                          p_ble_evt->evt.gap_evt.params.auth_status.bonded,
                          p_ble_evt->evt.gap_evt.params.auth_status.sm1_levels.lv4,
                          *((uint8_t *)&p_ble_evt->evt.gap_evt.params.auth_status.kdist_own),
                          *((uint8_t *)&p_ble_evt->evt.gap_evt.params.auth_status.kdist_peer));
            break;

        default:
            break;
    }
}


/**@brief SoftDevice SoC event handler.
 *
 * @param[in]   evt_id      SoC event.
 * @param[in]   p_context   Context.
 */
static void soc_evt_handler(uint32_t evt_id, void * p_context)
{
    switch (evt_id)
    {
        case NRF_EVT_FLASH_OPERATION_SUCCESS:
            /* fall through */
        case NRF_EVT_FLASH_OPERATION_ERROR:

            if (m_memory_access_in_progress)
            {
                m_memory_access_in_progress = false;
                scan_start();
            }
            break;

        default:
            // No implementation needed.
            break;
    }
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

    // Register handlers for BLE and SoC events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
    NRF_SDH_SOC_OBSERVER(m_soc_observer, APP_SOC_OBSERVER_PRIO, soc_evt_handler, NULL);
}



/**@brief Function for the Peer Manager initialization.
 */
static void peer_manager_init(void)
{
    ble_gap_sec_params_t sec_param;
    ret_code_t err_code;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    // Security parameters to be used for all security procedures.
    sec_param.bond           = SEC_PARAM_BOND;
    sec_param.mitm           = SEC_PARAM_MITM;
    sec_param.lesc           = SEC_PARAM_LESC;
    sec_param.keypress       = SEC_PARAM_KEYPRESS;
    sec_param.io_caps        = SEC_PARAM_IO_CAPABILITIES;
    sec_param.oob            = SEC_PARAM_OOB;
    sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
    sec_param.kdist_own.enc  = 1;
    sec_param.kdist_own.id   = 1;
    sec_param.kdist_peer.enc = 1;
    sec_param.kdist_peer.id  = 1;

    err_code = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);
}


/** @brief Clear bonding information from persistent storage
 */
static void delete_bonds(void)
{
    ret_code_t err_code;

    NRF_LOG_INFO("Erase bonds!");

    err_code = pm_peers_delete();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for disabling the use of whitelist for scanning.
 */
static void whitelist_disable(void)
{
    if (!m_whitelist_disabled)
    {
        NRF_LOG_INFO("Whitelist temporarily disabled.");
        m_whitelist_disabled = true;
        nrf_ble_scan_stop();
        scan_start();
    }
}


/**
 * @brief Database discovery collector initialization.
 */
static void db_discovery_init(void)
{
    ble_db_discovery_init_t db_init;

    memset(&db_init, 0, sizeof(db_init));

    db_init.evt_handler  = db_disc_handler;
    db_init.p_gatt_queue = &m_ble_gatt_queue;

    ret_code_t err_code = ble_db_discovery_init(&db_init);

    APP_ERROR_CHECK(err_code);
}


/**@brief Retrieve a list of peer manager peer IDs.
 *
 * @param[inout] p_peers   The buffer where to store the list of peer IDs.
 * @param[inout] p_size    In: The size of the @p p_peers buffer.
 *                         Out: The number of peers copied in the buffer.
 */
static void peer_list_get(pm_peer_id_t * p_peers, uint32_t * p_size)
{
    pm_peer_id_t peer_id;
    uint32_t     peers_to_copy;

    peers_to_copy = (*p_size < BLE_GAP_WHITELIST_ADDR_MAX_COUNT) ?
                     *p_size : BLE_GAP_WHITELIST_ADDR_MAX_COUNT;

    peer_id = pm_next_peer_id_get(PM_PEER_ID_INVALID);
    *p_size = 0;

    while ((peer_id != PM_PEER_ID_INVALID) && (peers_to_copy--))
    {
        p_peers[(*p_size)++] = peer_id;
        peer_id = pm_next_peer_id_get(peer_id);
    }
}


static void whitelist_load()
{
    ret_code_t   ret;
    pm_peer_id_t peers[8];
    uint32_t     peer_cnt;

    memset(peers, PM_PEER_ID_INVALID, sizeof(peers));
    peer_cnt = (sizeof(peers) / sizeof(pm_peer_id_t));

    // Load all peers from flash and whitelist them.
    peer_list_get(peers, &peer_cnt);

    ret = pm_whitelist_set(peers, peer_cnt);
    APP_ERROR_CHECK(ret);

    // Setup the device identies list.
    // Some SoftDevices do not support this feature.
    ret = pm_device_identities_list_set(peers, peer_cnt);
    if (ret != NRF_ERROR_NOT_SUPPORTED)
    {
        APP_ERROR_CHECK(ret);
    }
}


static void on_whitelist_req(void)
{
    ret_code_t err_code;

    // Whitelist buffers.
    ble_gap_addr_t whitelist_addrs[8];
    ble_gap_irk_t  whitelist_irks[8];

    memset(whitelist_addrs, 0x00, sizeof(whitelist_addrs));
    memset(whitelist_irks,  0x00, sizeof(whitelist_irks));

    uint32_t addr_cnt = (sizeof(whitelist_addrs) / sizeof(ble_gap_addr_t));
    uint32_t irk_cnt  = (sizeof(whitelist_irks)  / sizeof(ble_gap_irk_t));

    // Reload the whitelist and whitelist all peers.
    whitelist_load();

    // Get the whitelist previously set using pm_whitelist_set().
    err_code = pm_whitelist_get(whitelist_addrs, &addr_cnt,
                                whitelist_irks,  &irk_cnt);

    if (((addr_cnt == 0) && (irk_cnt == 0)) ||
        (m_whitelist_disabled))
    {
        // Don't use whitelist.
        err_code = nrf_ble_scan_params_set(&m_scan, NULL);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function to start scanning.
 */
static void scan_start(void)
{
    ret_code_t err_code;

    if (nrf_fstorage_is_busy(NULL))
    {
        m_memory_access_in_progress = true;
        return;
    }

    NRF_LOG_INFO("Starting scan.");

    err_code = nrf_ble_scan_start(&m_scan);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_indication_set(BSP_INDICATE_SCANNING);
    APP_ERROR_CHECK(err_code);
}



/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing the power management module. */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief GATT module event handler.
 */
static void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    switch (p_evt->evt_id)
    {
        case NRF_BLE_GATT_EVT_ATT_MTU_UPDATED:
        {
            NRF_LOG_INFO("GATT ATT MTU on connection 0x%x changed to %d.",
                         p_evt->conn_handle,
                         p_evt->params.att_mtu_effective);
        } break;

        case NRF_BLE_GATT_EVT_DATA_LENGTH_UPDATED:
        {
            NRF_LOG_INFO("Data length for connection 0x%x updated to %d.",
                         p_evt->conn_handle,
                         p_evt->params.data_length);
        } break;

        default:
            break;
    }
}


static void scan_evt_handler(scan_evt_t const * p_scan_evt)
{
    ret_code_t err_code;
    switch(p_scan_evt->scan_evt_id)
    {
        case NRF_BLE_SCAN_EVT_WHITELIST_REQUEST:
        {
            on_whitelist_req();
            m_whitelist_disabled = false;
        } break;

        case NRF_BLE_SCAN_EVT_CONNECTING_ERROR:
        {
            err_code = p_scan_evt->params.connecting_err.err_code;
            APP_ERROR_CHECK(err_code);
        } break;

        case NRF_BLE_SCAN_EVT_SCAN_TIMEOUT:
        {
            NRF_LOG_INFO("Scan timed out.");
            scan_start();
        } break;

        case NRF_BLE_SCAN_EVT_FILTER_MATCH:
            break;
        case NRF_BLE_SCAN_EVT_WHITELIST_ADV_REPORT:
            break;

        default:
          break;
    }
}


/**@brief Function for initializing the timer.
 */
static void timer_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initialization scanning and setting filters.
 */
static void scan_init(void)
{
    ret_code_t          err_code;
    nrf_ble_scan_init_t init_scan;

    memset(&init_scan, 0, sizeof(init_scan));

    init_scan.p_scan_param     = &m_scan_param;
    init_scan.connect_if_match = true;
    init_scan.conn_cfg_tag     = APP_BLE_CONN_CFG_TAG;

    err_code = nrf_ble_scan_init(&m_scan, &init_scan, scan_evt_handler);
    APP_ERROR_CHECK(err_code);

    ble_uuid_t uuid =
    {
        .uuid = TARGET_UUID,
        .type = BLE_UUID_TYPE_BLE,
    };

    err_code = nrf_ble_scan_filter_set(&m_scan,
                                       SCAN_UUID_FILTER,
                                       &uuid);
    APP_ERROR_CHECK(err_code);

    if (strlen(m_target_periph_name) != 0)
    {
        err_code = nrf_ble_scan_filter_set(&m_scan,
                                           SCAN_NAME_FILTER,
                                           m_target_periph_name);
        APP_ERROR_CHECK(err_code);
    }

    if (is_connect_per_addr)
    {
       err_code = nrf_ble_scan_filter_set(&m_scan,
                                          SCAN_ADDR_FILTER,
                                          m_target_periph_addr.addr);
       APP_ERROR_CHECK(err_code);
    }

    err_code = nrf_ble_scan_filters_enable(&m_scan,
                                           NRF_BLE_SCAN_ALL_FILTER,
                                           false);
    APP_ERROR_CHECK(err_code);

}


/**@brief Function for handling the idle state (main loop).
 *
 * @details Handle any pending log operation(s), then sleep until the next event occurs.
 */
static void idle_state_handle(void)
{
    ret_code_t err_code;
    
    err_code = nrf_ble_lesc_request_handler();
    APP_ERROR_CHECK(err_code);
    
    NRF_LOG_FLUSH();
    nrf_pwr_mgmt_run();
}


/**@brief Function for starting a scan, or instead trigger it from peer manager (after
 *        deleting bonds).
 *
 * @param[in] p_erase_bonds Pointer to a bool to determine if bonds will be deleted before scanning.
 */
void scanning_start(bool * p_erase_bonds)
{
    // Start scanning for peripherals and initiate connection
    // with devices that advertise GATT Service UUID.
    if (*p_erase_bonds == true)
    {
        // Scan is started by the PM_EVT_PEERS_DELETE_SUCCEEDED event.
        delete_bonds();
    }
    else
    {
        scan_start();
    }
}


int main(void)
{
    // Initialize.
    bool erase_bonds;
    reset_count();
    ret_code_t ret = nrf_drv_clock_init();
    APP_ERROR_CHECK(ret);
    nrf_drv_clock_lfclk_request(NULL);

    bsp_board_init(BSP_INIT_LEDS);
    blinker_init();
    ble_stack_init();
    scan_init();
    init_voi_scanner();

    // Start execution.
    APP_ERROR_CHECK(start_blinking());
    logging_init();
    // Enter main loop.
    for (;;)
    {
        logging_process();
        if (get_state() == START)
        {
            set_state(SCANNING);
            scanning_start(&erase_bonds);
        }
    }
}


