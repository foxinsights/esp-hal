use core::ptr::addr_of_mut;

use procmacros::BuilderLite;

use super::*;
use crate::{
    ble::InvalidConfigError,
    common_adapter::*,
    hal::{interrupt::Priority, peripherals::BT},
    interrupt_dispatch::Handler,
    sys::include::{
        esp_bt_controller_config_t,
        esp_bt_mode_t,
        esp_bt_mode_t_ESP_BT_MODE_BLE,
        esp_bt_mode_t_ESP_BT_MODE_BTDM,
        esp_bt_mode_t_ESP_BT_MODE_CLASSIC_BT,
        esp_bt_mode_t_ESP_BT_MODE_IDLE,
    },
};

static ISR_INTERRUPT_5: Handler = Handler::new();
static ISR_INTERRUPT_7: Handler = Handler::new();
static ISR_INTERRUPT_8: Handler = Handler::new();

#[repr(C)]
pub(super) struct osi_funcs_s {
    version: u32,
    set_isr: Option<unsafe extern "C" fn(i32, unsafe extern "C" fn(), *const ()) -> i32>,
    ints_on: Option<unsafe extern "C" fn(u32)>,
    interrupt_disable: Option<unsafe extern "C" fn()>,
    interrupt_restore: Option<unsafe extern "C" fn()>,
    task_yield: Option<unsafe extern "C" fn()>,
    task_yield_from_isr: Option<unsafe extern "C" fn()>,
    semphr_create: Option<unsafe extern "C" fn(u32, u32) -> *mut c_void>,
    semphr_delete: Option<unsafe extern "C" fn(*mut c_void)>,
    semphr_take_from_isr: Option<unsafe extern "C" fn(*mut c_void, *mut bool) -> i32>,
    semphr_give_from_isr: Option<unsafe extern "C" fn(*mut c_void, *mut bool) -> i32>,
    semphr_take: Option<unsafe extern "C" fn(*mut c_void, u32) -> i32>,
    semphr_give: Option<unsafe extern "C" fn(*mut c_void) -> i32>,
    mutex_create: Option<unsafe extern "C" fn() -> *const ()>,
    mutex_delete: Option<unsafe extern "C" fn(*const ())>,
    mutex_lock: Option<unsafe extern "C" fn(*const ()) -> i32>,
    mutex_unlock: Option<unsafe extern "C" fn(*const ()) -> i32>,
    queue_create: Option<unsafe extern "C" fn(u32, u32) -> *mut c_void>,
    queue_delete: Option<unsafe extern "C" fn(*mut c_void)>,
    queue_send: Option<unsafe extern "C" fn(*mut c_void, *mut c_void, u32) -> i32>,
    queue_send_from_isr: Option<unsafe extern "C" fn(*mut c_void, *mut c_void, *mut c_void) -> i32>,
    queue_recv: Option<unsafe extern "C" fn(*mut c_void, *mut c_void, u32) -> i32>,
    queue_recv_from_isr: Option<unsafe extern "C" fn(*mut c_void, *mut c_void, *mut c_void) -> i32>,
    task_create: Option<
        unsafe extern "C" fn(
            *mut c_void,
            *const c_char,
            u32,
            *mut c_void,
            u32,
            *mut c_void,
            u32,
        ) -> i32,
    >,
    task_delete: Option<unsafe extern "C" fn(*mut ())>,
    is_in_isr: Option<unsafe extern "C" fn() -> i32>,
    cause_sw_intr_to_core: Option<unsafe extern "C" fn(i32, i32) -> i32>,
    malloc: Option<unsafe extern "C" fn(u32) -> *mut c_void>,
    malloc_internal: Option<unsafe extern "C" fn(u32) -> *mut c_void>,
    free: Option<unsafe extern "C" fn(*mut c_void)>,
    read_efuse_mac: Option<unsafe extern "C" fn(*const ()) -> i32>,
    srand: Option<unsafe extern "C" fn(u32)>,
    rand: Option<unsafe extern "C" fn() -> i32>,
    // Signatures as in ESP-IDF's `components/bt/controller/esp32/bt.c` (`struct osi_funcs_t`) of
    // the release that ships the linked `libbtdm_app.a` (compile version 045a658).
    btdm_lpcycles_2_us: Option<unsafe extern "C" fn(u32) -> u32>,
    btdm_us_2_lpcycles: Option<unsafe extern "C" fn(u32) -> u32>,
    btdm_sleep_check_duration: Option<unsafe extern "C" fn(*mut u32) -> bool>,
    btdm_sleep_enter_phase1: Option<unsafe extern "C" fn(u32)>,
    btdm_sleep_enter_phase2: Option<unsafe extern "C" fn()>,
    btdm_sleep_exit_phase1: Option<unsafe extern "C" fn()>,
    btdm_sleep_exit_phase2: Option<unsafe extern "C" fn()>,
    btdm_sleep_exit_phase3: Option<unsafe extern "C" fn()>,
    coex_bt_wakeup_request: Option<unsafe extern "C" fn() -> bool>,
    coex_bt_wakeup_request_end: Option<unsafe extern "C" fn()>,
    coex_bt_request: Option<unsafe extern "C" fn(u32, u32, u32) -> i32>,
    coex_bt_release: Option<unsafe extern "C" fn(u32) -> i32>,
    coex_register_bt_cb: Option<unsafe extern "C" fn(unsafe extern "C" fn()) -> i32>,
    coex_bb_reset_lock: Option<unsafe extern "C" fn() -> u32>,
    coex_bb_reset_unlock: Option<unsafe extern "C" fn(u32)>,
    coex_schm_register_btdm_callback: Option<unsafe extern "C" fn(unsafe extern "C" fn()) -> i32>,
    coex_schm_status_bit_clear: Option<unsafe extern "C" fn(i32, i32)>,
    coex_schm_status_bit_set: Option<unsafe extern "C" fn(i32, i32)>,
    coex_schm_interval_get: Option<unsafe extern "C" fn() -> u32>,
    coex_schm_curr_period_get: Option<unsafe extern "C" fn() -> u8>,
    coex_schm_curr_phase_get: Option<unsafe extern "C" fn() -> *mut c_void>,
    coex_wifi_channel_get: Option<unsafe extern "C" fn(*mut u8, *mut u8) -> i32>,
    coex_register_wifi_channel_change_callback:
        Option<unsafe extern "C" fn(unsafe extern "C" fn()) -> i32>,
    set_isr13: Option<unsafe extern "C" fn(i32, unsafe extern "C" fn(), *const ()) -> i32>,
    interrupt_l3_disable: Option<unsafe extern "C" fn()>,
    interrupt_l3_restore: Option<unsafe extern "C" fn()>,
    custom_queue_create: Option<unsafe extern "C" fn(u32, u32) -> *mut c_void>,
    coex_version_get: Option<unsafe extern "C" fn(*mut c_uint, *mut c_uint, *mut c_uint) -> c_int>,
    patch_apply: Option<unsafe extern "C" fn()>,
    magic: u32,
}

pub(super) static G_OSI_FUNCS: osi_funcs_s = osi_funcs_s {
    version: 0x00010005,
    set_isr: Some(set_isr),
    ints_on: Some(ints_on),
    interrupt_disable: Some(interrupt_disable),
    interrupt_restore: Some(interrupt_enable),
    task_yield: Some(task_yield),
    task_yield_from_isr: Some(task_yield_from_isr),
    semphr_create: Some(semphr_create),
    semphr_delete: Some(semphr_delete),
    semphr_take_from_isr: Some(semphr_take_from_isr),
    semphr_give_from_isr: Some(semphr_give_from_isr),
    semphr_take: Some(semphr_take),
    semphr_give: Some(semphr_give),
    mutex_create: Some(mutex_create),
    mutex_delete: Some(mutex_delete),
    mutex_lock: Some(mutex_lock),
    mutex_unlock: Some(mutex_unlock),
    queue_create: Some(queue_create),
    queue_delete: Some(queue_delete),
    queue_send: Some(queue_send),
    queue_send_from_isr: Some(queue_send_from_isr),
    queue_recv: Some(queue_recv),
    queue_recv_from_isr: Some(queue_recv_from_isr),
    task_create: Some(task_create),
    task_delete: Some(task_delete),
    is_in_isr: Some(is_in_isr),
    cause_sw_intr_to_core: Some(cause_sw_intr_to_core),
    malloc: Some(crate::ble::malloc),
    malloc_internal: Some(crate::ble::malloc_internal),
    free: Some(crate::ble::free),
    read_efuse_mac: Some(read_efuse_mac),
    srand: Some(crate::ble::btdm::srand),
    rand: Some(crate::ble::btdm::rand),
    // The sleep callbacks are only called by the controller in modem sleep mode, i.e. with the
    // `ble_modem_sleep` config option; ESP-IDF leaves the two `exit_phase1/2` slots NULL.
    btdm_lpcycles_2_us: Some(btdm_lpcycles_2_us),
    btdm_us_2_lpcycles: Some(btdm_us_2_lpcycles),
    btdm_sleep_check_duration: Some(btdm_sleep_check_duration),
    btdm_sleep_enter_phase1: Some(btdm_sleep_enter_phase1),
    btdm_sleep_enter_phase2: Some(btdm_sleep_enter_phase2),
    btdm_sleep_exit_phase1: None,
    btdm_sleep_exit_phase2: None,
    btdm_sleep_exit_phase3: Some(btdm_sleep_exit_phase3),
    coex_bt_wakeup_request: Some(coex_bt_wakeup_request),
    coex_bt_wakeup_request_end: Some(coex_bt_wakeup_request_end),
    coex_bt_request: Some(coex_bt_request),
    coex_bt_release: Some(coex_bt_release),
    coex_register_bt_cb: Some(coex_register_bt_cb),
    coex_bb_reset_lock: Some(coex_bb_reset_lock),
    coex_bb_reset_unlock: Some(coex_bb_reset_unlock),
    coex_schm_register_btdm_callback: Some(coex_schm_register_btdm_callback_wrapper),
    coex_schm_status_bit_clear: Some(coex_schm_status_bit_clear),
    coex_schm_status_bit_set: Some(coex_schm_status_bit_set),
    coex_schm_interval_get: Some(coex_schm_interval_get),
    coex_schm_curr_period_get: Some(coex_schm_curr_period_get),
    coex_schm_curr_phase_get: Some(coex_schm_curr_phase_get),
    coex_wifi_channel_get: Some(coex_wifi_channel_get),
    coex_register_wifi_channel_change_callback: Some(coex_register_wifi_channel_change_callback),
    set_isr13: Some(set_isr13),
    interrupt_l3_disable: Some(interrupt_l3_disable),
    interrupt_l3_restore: Some(interrupt_l3_restore),
    custom_queue_create: Some(custom_queue_create),
    coex_version_get: Some(coex_version_get_wrapper),
    patch_apply: Some(patch_apply),
    magic: 0xfadebead,
};

extern "C" fn patch_apply() {
    trace!("patch apply");

    unsafe extern "C" {
        fn config_ble_funcs_reset();
        fn config_btdm_funcs_reset();
    }

    unsafe {
        config_btdm_funcs_reset();
        config_ble_funcs_reset();
    }
}

extern "C" fn coex_version_get_wrapper(major: *mut u32, minor: *mut u32, patch: *mut u32) -> i32 {
    unsafe {
        let mut version = crate::sys::include::coex_version_t {
            major: 0,
            minor: 0,
            patch: 0,
        };
        if coex_version_get_value(&mut version) == 0 {
            info!(
                "COEX Version {}.{}.{}",
                version.major, version.minor, version.patch
            );

            major.write_volatile(version.major as u32);
            minor.write_volatile(version.minor as u32);
            patch.write_volatile(version.patch as u32);
        } else {
            error!("Unable to get COEX version");
        }
    }

    0
}

#[repr(C)]
struct btdm_dram_available_region_t {
    mode: esp_bt_mode_t,
    start: u32, // ptr
    end: u32,   // ptr
}

const SOC_MEM_BT_DATA_START: u32 = 0x3ffae6e0;
const SOC_MEM_BT_DATA_END: u32 = 0x3ffaff10;
const SOC_MEM_BT_EM_BTDM0_START: u32 = 0x3ffb0000;
const SOC_MEM_BT_EM_BTDM0_END: u32 = 0x3ffb09a8;
const SOC_MEM_BT_EM_BLE_START: u32 = 0x3ffb09a8;
const SOC_MEM_BT_EM_BLE_END: u32 = 0x3ffb1ddc;
const SOC_MEM_BT_EM_BTDM1_START: u32 = 0x3ffb1ddc;
const SOC_MEM_BT_EM_BTDM1_END: u32 = 0x3ffb2730;
const SOC_MEM_BT_EM_BREDR_START: u32 = 0x3ffb2730;
const SOC_MEM_BT_BSS_START: u32 = 0x3ffb8000;
const SOC_MEM_BT_BSS_END: u32 = 0x3ffb9a20;
const SOC_MEM_BT_MISC_START: u32 = 0x3ffbdb28;
const SOC_MEM_BT_MISC_END: u32 = 0x3ffbdb5c;

const SOC_MEM_BT_EM_BREDR_REAL_END: u32 = 0x3ffb6388; //  (SOC_MEM_BT_EM_BREDR_NO_SYNC_END + CONFIG_BTDM_CTRL_BR_EDR_MAX_SYNC_CONN_EFF
// * SOC_MEM_BT_EM_PER_SYNC_SIZE);

static BTDM_DRAM_AVAILABLE_REGION: [btdm_dram_available_region_t; 7] = [
    // following is .data
    btdm_dram_available_region_t {
        mode: esp_bt_mode_t_ESP_BT_MODE_BTDM,
        start: SOC_MEM_BT_DATA_START,
        end: SOC_MEM_BT_DATA_END,
    },
    // following is memory which HW will use
    btdm_dram_available_region_t {
        mode: esp_bt_mode_t_ESP_BT_MODE_BTDM,
        start: SOC_MEM_BT_EM_BTDM0_START,
        end: SOC_MEM_BT_EM_BTDM0_END,
    },
    btdm_dram_available_region_t {
        mode: esp_bt_mode_t_ESP_BT_MODE_BLE,
        start: SOC_MEM_BT_EM_BLE_START,
        end: SOC_MEM_BT_EM_BLE_END,
    },
    btdm_dram_available_region_t {
        mode: esp_bt_mode_t_ESP_BT_MODE_BTDM,
        start: SOC_MEM_BT_EM_BTDM1_START,
        end: SOC_MEM_BT_EM_BTDM1_END,
    },
    btdm_dram_available_region_t {
        mode: esp_bt_mode_t_ESP_BT_MODE_CLASSIC_BT,
        start: SOC_MEM_BT_EM_BREDR_START,
        end: SOC_MEM_BT_EM_BREDR_REAL_END,
    },
    // following is .bss
    btdm_dram_available_region_t {
        mode: esp_bt_mode_t_ESP_BT_MODE_BTDM,
        start: SOC_MEM_BT_BSS_START,
        end: SOC_MEM_BT_BSS_END,
    },
    btdm_dram_available_region_t {
        mode: esp_bt_mode_t_ESP_BT_MODE_BTDM,
        start: SOC_MEM_BT_MISC_START,
        end: SOC_MEM_BT_MISC_END,
    },
];

/// Bluetooth controller configuration.
#[derive(BuilderLite, Clone, Copy, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Config {
    /// The priority of the RTOS task.
    task_priority: u8,

    /// The stack size of the RTOS task.
    task_stack_size: u16,

    /// The maximum number of simultaneous connections.
    ///
    /// Range: 1 - 9
    max_connections: u8,

    /// Maximum number of devices in scan duplicate filtering list.
    ///
    /// Range: 10 - 1000
    scan_duplicate_list_count: u16,

    /// Scan duplicate filtering list refresh period in seconds.
    ///
    /// Range: 0 - 1000 seconds
    scan_duplicate_refresh_period: u16,

    /// Enable BLE scan backoff.
    ble_scan_backoff: bool,

    /// Minimum encryption key size.
    ///
    /// Range: 7 - 16
    enc_key_sz_min: u8,

    /// Enable verification of the Access Address within the `CONNECT_IND` PDU.
    verify_access_address: bool,

    /// Enable BLE channel assessment.
    channel_assessment: bool,

    /// Enable BLE ping procedure.
    ping: bool,

    /// Disconnect when Instant Passed (0x28) occurs during ACL connection update.
    disconnect_llcp_conn_update: bool,

    /// Disconnect when Instant Passed (0x28) occurs during ACL channel map update.
    disconnect_llcp_chan_map_update: bool,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            task_priority: crate::preempt::max_task_priority()
                .saturating_sub(2)
                .min(255) as u8,
            task_stack_size: 4096,
            max_connections: CONFIG_BTDM_CTRL_BLE_MAX_CONN_EFF as _,
            scan_duplicate_list_count: CONFIG_BTDM_SCAN_DUPL_CACHE_SIZE as _,
            scan_duplicate_refresh_period: SCAN_DUPL_CACHE_REFRESH_PERIOD as _,
            ble_scan_backoff: false,
            enc_key_sz_min: 7,
            verify_access_address: false,
            channel_assessment: false,
            ping: false,
            disconnect_llcp_conn_update: false,
            disconnect_llcp_chan_map_update: false,
        }
    }
}

impl Config {
    pub(crate) fn validate(&self) -> Result<(), InvalidConfigError> {
        crate::ble::validate_range!(
            self,
            task_priority,
            0,
            crate::preempt::max_task_priority().min(255) as u8
        );
        crate::ble::validate_range!(self, max_connections, 1, 9);
        crate::ble::validate_range!(self, scan_duplicate_list_count, 10, 1000);
        crate::ble::validate_range!(self, scan_duplicate_refresh_period, 0, 1000);
        crate::ble::validate_range!(self, enc_key_sz_min, 7, 16);

        Ok(())
    }
}

pub(crate) fn create_ble_config(config: &Config) -> esp_bt_controller_config_t {
    // keep them aligned with BT_CONTROLLER_INIT_CONFIG_DEFAULT in ESP-IDF
    // ideally _some_ of these values should be configurable
    esp_bt_controller_config_t {
        controller_task_stack_size: config.task_stack_size,
        controller_task_prio: config.task_priority,

        // We're using VHCI but esp-idf has these defaults:
        hci_uart_no: 1,
        hci_uart_baudrate: 921600,
        // Bluetooth mesh options, currently not supported
        scan_duplicate_mode: 0, // normal mode
        scan_duplicate_type: 0,
        mesh_adv_size: 0,

        normal_adv_size: config.scan_duplicate_list_count,
        send_adv_reserved_size: SCAN_SEND_ADV_RESERVED_SIZE as _,
        controller_debug_flag: BTDM_CTRL_CONTROLLER_DEBUG_FLAG as _,
        mode: esp_bt_mode_t_ESP_BT_MODE_BLE as _,
        ble_max_conn: config.max_connections,
        bt_max_acl_conn: 0,
        bt_sco_datapath: 0,
        auto_latency: false,
        bt_legacy_auth_vs_evt: false,
        bt_max_sync_conn: 1,
        ble_sca: 1,
        pcm_role: 0,
        pcm_polar: 0,
        hli: false,
        dup_list_refresh_period: config.scan_duplicate_refresh_period,
        ble_scan_backoff: config.ble_scan_backoff,
        pcm_fsyncshp: 0,
        enc_key_sz_min: config.enc_key_sz_min,
        ble_llcp_disc_flag: config.disconnect_llcp_conn_update as u8
            | ((config.disconnect_llcp_chan_map_update as u8) << 1),
        ble_aa_check: config.verify_access_address,
        ble_chan_ass_en: config.channel_assessment as u8,
        ble_ping_en: config.ping as u8,
        magic: ESP_BT_CONTROLLER_CONFIG_MAGIC_VAL,
    }
}

pub(crate) fn btdm_controller_mem_init() {
    unsafe extern "C" {
        static mut _data_start_btdm: u32;
        static mut _data_end_btdm: u32;
        static _data_start_btdm_rom: u32;
    }

    // initialise .data section
    unsafe {
        let data_start = addr_of_mut!(_data_start_btdm).cast::<u8>();
        let data_end = addr_of_mut!(_data_end_btdm).cast::<u8>();

        // `_data_start_btdm_rom` is a pointer to the actual initialization data in the
        // ROM.
        let data_start_rom = _data_start_btdm_rom as *const u8;

        let len = data_end as usize - data_start as usize;

        core::ptr::copy_nonoverlapping(data_start_rom, data_start, len);

        debug!(
            "btdm_controller_mem_init {:?} {:?} {}",
            data_start_rom, data_start, len,
        );
    }

    // initialize em, .bss section
    let btdm_dram_regions = BTDM_DRAM_AVAILABLE_REGION.len();

    #[allow(clippy::needless_range_loop)] // the alternative looks worse
    for i in 1..btdm_dram_regions {
        if BTDM_DRAM_AVAILABLE_REGION[i].mode != esp_bt_mode_t_ESP_BT_MODE_IDLE {
            unsafe {
                core::ptr::write_bytes(
                    BTDM_DRAM_AVAILABLE_REGION[i].start as *mut u8,
                    0x0,
                    (BTDM_DRAM_AVAILABLE_REGION[i].end - BTDM_DRAM_AVAILABLE_REGION[i].start)
                        as usize,
                );
            }
            debug!(
                ".bss initialise {:x} - {:x}\n",
                BTDM_DRAM_AVAILABLE_REGION[i].start, BTDM_DRAM_AVAILABLE_REGION[i].end
            );
        }
    }

    debug!("btdm_controller_mem_init done");
}

pub(crate) fn bt_periph_module_enable() {
    // nothing
}

// Modem sleep, ported from ESP-IDF `components/bt/controller/esp32/bt.c` at the commit that
// introduced the linked `libbtdm_app.a` (esp32-bt-lib 045a658b, esp-idf 44b05258d7), built the way
// fw-fox2 configures it: `CONFIG_BTDM_CTRL_MODEM_SLEEP_MODE_ORIG`,
// `CONFIG_BTDM_CTRL_LPCLK_SEL_MAIN_XTAL`, no `CONFIG_PM_ENABLE` (so no PM locks, no sleep timer).

unsafe extern "C" {
    fn btdm_controller_set_sleep_mode(mode: u8);
    fn btdm_controller_get_sleep_mode() -> u8;
    fn btdm_rf_bb_init_phase2();
}

#[cfg(ble_modem_sleep)]
unsafe extern "C" {
    fn btdm_controller_enable_sleep(enable: bool);
    fn btdm_power_state_active() -> bool;
    fn btdm_wakeup_request();
    fn btdm_in_wakeup_requesting_set(in_wakeup_requesting: bool);
    fn btdm_lpclk_select_src(sel: u32) -> bool;
    fn btdm_lpclk_set_div(div: u32) -> bool;
    fn btdm_dispatch_work_to_controller(
        callback: unsafe extern "C" fn(*mut c_void),
        arg: *mut c_void,
        blocking: bool,
    ) -> i32;
    fn sdk_config_set_bt_pll_track_enable(enable: bool);
}

const BTDM_MODEM_SLEEP_MODE_NONE: u8 = 0;
const BTDM_MODEM_SLEEP_MODE_ORIG: u8 = 1;

#[cfg(ble_modem_sleep)]
const BTDM_LPCLK_SEL_XTAL: u32 = 0;

/// Sleep durations shorter than this many slots are not worth the PHY/RF power cycle.
const BTDM_MIN_SLEEP_DURATION: u32 = 12;
/// Slots it takes to wake the modem up, including re-enabling the PHY/RF.
const BTDM_MODEM_WAKE_UP_DELAY: u32 = 4;

/// `RTC_CLK_CAL_FRACT`: fractional bits of [BTDM_LPCYCLE_US].
const BTDM_LPCYCLE_US_FRAC: u32 = 19;
/// Low-power clock period in µs, fixed point with [BTDM_LPCYCLE_US_FRAC] fractional bits.
///
/// The main XTAL divided by `2 * f_xtal[MHz]` runs at 500 kHz whatever the XTAL frequency, so a
/// cycle is 2 µs (ESP-IDF: `btdm_lpcycle_us = 2 << btdm_lpcycle_us_frac`).
const BTDM_LPCYCLE_US: u32 = 2 << BTDM_LPCYCLE_US_FRAC;

/// The PHY/RF is switched off by [btdm_sleep_enter_phase2] and not yet back on.
static PHY_OFF_FOR_SLEEP: AtomicBool = AtomicBool::new(false);
/// [esp_phy::rf_on_count] when the BT baseband RF was last initialized
/// (`btdm_rf_bb_init_phase2`); ESP-IDF's `s_time_phy_rf_just_enabled`.
static BB_INIT_RF_ON_COUNT: portable_atomic::AtomicU32 = portable_atomic::AtomicU32::new(0);

#[cfg(ble_modem_sleep)]
static WAKEUP_REQ_SEM: portable_atomic::AtomicPtr<c_void> =
    portable_atomic::AtomicPtr::new(core::ptr::null_mut());

#[ram]
unsafe extern "C" fn btdm_lpcycles_2_us(cycles: u32) -> u32 {
    // The number of lp cycles should not lead to overflow. Thrs: 100s
    let us = BTDM_LPCYCLE_US as u64 * cycles as u64;
    ((us + (1 << (BTDM_LPCYCLE_US_FRAC - 1))) >> BTDM_LPCYCLE_US_FRAC) as u32
}

/// Converts a duration in µs into a number of low-power clock cycles.
#[ram]
unsafe extern "C" fn btdm_us_2_lpcycles(us: u32) -> u32 {
    (((us as u64) << BTDM_LPCYCLE_US_FRAC) / BTDM_LPCYCLE_US as u64) as u32
}

#[ram]
unsafe extern "C" fn btdm_sleep_check_duration(slot_cnt: *mut u32) -> bool {
    unsafe {
        if *slot_cnt < BTDM_MIN_SLEEP_DURATION {
            return false;
        }
        // wake up in advance considering the delay in enabling PHY/RF
        *slot_cnt -= BTDM_MODEM_WAKE_UP_DELAY;
    }
    true
}

unsafe extern "C" fn btdm_sleep_enter_phase1(_lpcycles: u32) {
    // ESP-IDF only arms a light-sleep wakeup timer here (CONFIG_PM_ENABLE).
}

unsafe extern "C" fn btdm_sleep_enter_phase2() {
    if unsafe { btdm_controller_get_sleep_mode() } == BTDM_MODEM_SLEEP_MODE_ORIG {
        trace!("btdm_sleep_enter_phase2");
        // Idempotent so the PHY ref count can't drift should the controller not pair the calls.
        if !PHY_OFF_FOR_SLEEP.swap(true, Ordering::AcqRel) {
            esp_phy::disable_phy();
        }
    }
}

unsafe extern "C" fn btdm_sleep_exit_phase3() {
    if unsafe { btdm_controller_get_sleep_mode() } == BTDM_MODEM_SLEEP_MODE_ORIG {
        trace!("btdm_sleep_exit_phase3");
        phy_on_after_sleep();
        check_and_init_bb();
    }
}

/// Switch the PHY back on if [btdm_sleep_enter_phase2] switched it off.
fn phy_on_after_sleep() {
    if PHY_OFF_FOR_SLEEP.swap(false, Ordering::AcqRel) {
        // Balanced by `disable_phy` in `btdm_sleep_enter_phase2` or by the BLE connector's guard.
        core::mem::forget(esp_phy::enable_phy());
    }
}

/// Re-initialize the BT baseband RF if the PHY/RF was switched off since the last init
/// (ESP-IDF `btdm_check_and_init_bb`).
fn check_and_init_bb() {
    let rf_on_count = esp_phy::rf_on_count();
    if BB_INIT_RF_ON_COUNT.swap(rf_on_count, Ordering::AcqRel) != rf_on_count {
        unsafe { btdm_rf_bb_init_phase2() };
    }
}

/// Initialize the BT baseband RF after the PHY was enabled in `ble_init`.
pub(crate) fn init_bb() {
    BB_INIT_RF_ON_COUNT.store(esp_phy::rf_on_count(), Ordering::Release);
    unsafe { btdm_rf_bb_init_phase2() };
}

/// Select the low-power clock and the sleep mode; must run before `btdm_controller_init`
/// (ESP-IDF `btdm_low_power_mode_init`).
pub(crate) fn low_power_mode_init() {
    cfg_if::cfg_if! {
        if #[cfg(ble_modem_sleep)] {
            debug!("BT modem sleep: using main XTAL as low-power clock");
            let xtal_mhz = esp_hal::clock::xtal_clock().as_mhz();
            let select_src_ret = unsafe { btdm_lpclk_select_src(BTDM_LPCLK_SEL_XTAL) };
            let set_div_ret = unsafe { btdm_lpclk_set_div(xtal_mhz * 2 - 1) };
            assert!(
                select_src_ret && set_div_ret,
                "selecting the BT low-power clock failed"
            );

            let sem = crate::compat::semaphore::sem_create(1, 0);
            WAKEUP_REQ_SEM.store(sem, Ordering::Release);

            unsafe { btdm_controller_set_sleep_mode(BTDM_MODEM_SLEEP_MODE_ORIG) };
        } else {
            unsafe { btdm_controller_set_sleep_mode(BTDM_MODEM_SLEEP_MODE_NONE) };
        }
    }
}

/// Undo [low_power_mode_init] after `btdm_controller_deinit`
/// (ESP-IDF `btdm_low_power_mode_deinit` + `s_wakeup_req_sem` cleanup).
pub(crate) fn low_power_mode_deinit() {
    unsafe { btdm_controller_set_sleep_mode(BTDM_MODEM_SLEEP_MODE_NONE) };

    #[cfg(ble_modem_sleep)]
    {
        let sem = WAKEUP_REQ_SEM.swap(core::ptr::null_mut(), Ordering::AcqRel);
        if !sem.is_null() {
            crate::compat::semaphore::sem_delete(sem);
        }
    }
}

/// Let the controller sleep; call after enabling the PHY and before `btdm_rf_bb_init_phase2` /
/// `btdm_controller_enable` (the matching part of ESP-IDF `esp_bt_controller_enable`).
pub(crate) fn modem_sleep_enable() {
    #[cfg(ble_modem_sleep)]
    unsafe {
        if btdm_controller_get_sleep_mode() == BTDM_MODEM_SLEEP_MODE_ORIG {
            btdm_controller_enable_sleep(true);
        }
        sdk_config_set_bt_pll_track_enable(true);
    }
}

/// Stop modem sleep and wait until the controller is awake, so it can be torn down with the
/// PHY on (the matching part of ESP-IDF `esp_bt_controller_disable`).
pub(crate) fn modem_sleep_disable() {
    #[cfg(ble_modem_sleep)]
    unsafe {
        if btdm_controller_get_sleep_mode() == BTDM_MODEM_SLEEP_MODE_ORIG {
            btdm_controller_enable_sleep(false);
            async_wakeup_request(BTDM_ASYNC_WAKEUP_REQ_CTRL_DISA);
            while !btdm_power_state_active() {
                crate::hal::delay::Delay::new().delay_micros(1000);
            }
            async_wakeup_request_end(BTDM_ASYNC_WAKEUP_REQ_CTRL_DISA);
        }
    }

    // The BLE connector's PHY guard is dropped after this; make sure the PHY reference it
    // accounts for is held again.
    phy_on_after_sleep();
}

/// Wake the controller before handing it an HCI packet (ESP-IDF `esp_vhci_host_send_packet`).
#[cfg(ble_modem_sleep)]
pub(crate) fn vhci_wakeup_request() {
    async_wakeup_request(BTDM_ASYNC_WAKEUP_REQ_HCI);
}

#[cfg(ble_modem_sleep)]
pub(crate) fn vhci_wakeup_request_end() {
    async_wakeup_request_end(BTDM_ASYNC_WAKEUP_REQ_HCI);
}

#[cfg(ble_modem_sleep)]
const BTDM_ASYNC_WAKEUP_REQ_HCI: i32 = 0;
#[cfg(ble_modem_sleep)]
const BTDM_ASYNC_WAKEUP_REQ_COEX: i32 = 1;
#[cfg(ble_modem_sleep)]
const BTDM_ASYNC_WAKEUP_REQ_CTRL_DISA: i32 = 2;

/// Runs on the controller task (dispatched by [async_wakeup_request]).
#[cfg(ble_modem_sleep)]
unsafe extern "C" fn btdm_wakeup_request_callback(_arg: *mut c_void) {
    unsafe { btdm_wakeup_request() };
    let sem = WAKEUP_REQ_SEM.load(Ordering::Acquire);
    crate::compat::semaphore::sem_give(sem);
}

/// ESP-IDF `async_wakeup_request`; returns whether a wakeup was requested.
#[cfg(ble_modem_sleep)]
fn async_wakeup_request(event: i32) -> bool {
    trace!("async_wakeup_request {}", event);
    let mut do_wakeup_request = false;

    match event {
        BTDM_ASYNC_WAKEUP_REQ_HCI | BTDM_ASYNC_WAKEUP_REQ_CTRL_DISA => unsafe {
            btdm_in_wakeup_requesting_set(true);
            if !btdm_power_state_active() {
                do_wakeup_request = true;

                let res = btdm_dispatch_work_to_controller(
                    btdm_wakeup_request_callback,
                    core::ptr::null_mut(),
                    true,
                );
                if res == 0 {
                    let sem = WAKEUP_REQ_SEM.load(Ordering::Acquire);
                    crate::compat::semaphore::sem_take(sem, crate::compat::OSI_FUNCS_TIME_BLOCKING);
                } else {
                    warn!("btdm_dispatch_work_to_controller failed: {}", res);
                }
            }
        },
        BTDM_ASYNC_WAKEUP_REQ_COEX => unsafe {
            if !btdm_power_state_active() {
                do_wakeup_request = true;
                btdm_wakeup_request();
            }
        },
        _ => {}
    }

    do_wakeup_request
}

/// ESP-IDF `async_wakeup_request_end`.
#[cfg(ble_modem_sleep)]
fn async_wakeup_request_end(event: i32) {
    trace!("async_wakeup_request_end {}", event);
    if matches!(
        event,
        BTDM_ASYNC_WAKEUP_REQ_HCI | BTDM_ASYNC_WAKEUP_REQ_CTRL_DISA
    ) {
        unsafe { btdm_in_wakeup_requesting_set(false) };
    }
}

pub(crate) unsafe extern "C" fn coex_bt_wakeup_request() -> bool {
    trace!("coex_bt_wakeup_request");

    // With modem sleep the controller may be asleep when coex needs it: do what ESP-IDF does.
    // (Unlike the `coex`-feature `async_wakeup_request` below, ESP-IDF does not set
    // `btdm_in_wakeup_requesting` for a coex wakeup - it would never be cleared again.)
    #[cfg(ble_modem_sleep)]
    {
        async_wakeup_request(BTDM_ASYNC_WAKEUP_REQ_COEX)
    }

    // This should really be
    // ```rust,norun
    // #[cfg(feature = "coex")]
    // return async_wakeup_request(BTDM_ASYNC_WAKEUP_REQ_COEX);
    // #[cfg(not(feature = "coex"))]
    // true
    // ```
    //
    // But doing the right thing here keeps BT from working.
    // In a similar scenario this function isn't called in ESP-IDF.
    #[cfg(not(ble_modem_sleep))]
    true
}

pub(crate) unsafe extern "C" fn coex_bt_wakeup_request_end() {
    trace!("coex_bt_wakeup_request_end");

    // ESP-IDF's `async_wakeup_request_end(BTDM_ASYNC_WAKEUP_REQ_COEX)` does nothing.

    // Without modem sleep this should really be
    // ```rust,norun
    // #[cfg(feature = "coex")]
    // async_wakeup_request_end(BTDM_ASYNC_WAKEUP_REQ_COEX);
    // ```
    //
    // But doing the right thing here keeps BT from working.
    // In a similar scenario this function isn't called in ESP-IDF.
}

extern_coex_fns! {
    fn coex_bt_request(event: u32, latency: u32, duration: u32) -> i32;
    fn coex_bt_release(event: u32) -> i32;
    fn coex_register_bt_cb(callback: unsafe extern "C" fn()) -> i32;
    fn coex_bb_reset_lock() -> u32;
    fn coex_bb_reset_unlock(event: u32);
    fn coex_register_wifi_channel_change_callback(callback: unsafe extern "C" fn()) -> i32;
}

coex_fns! {
    fn coex_schm_interval_get() -> u32;
    fn coex_schm_curr_period_get() -> u8;
    fn coex_schm_curr_phase_get() -> *mut c_void;
}

pub(crate) unsafe extern "C" fn coex_schm_register_btdm_callback_wrapper(
    callback: unsafe extern "C" fn(),
) -> i32 {
    trace!("coex_schm_register_btdm_callback {:?}", callback);

    cfg_if::cfg_if! {
        if #[cfg(feature = "coex")] {
            unsafe extern "C" {
                fn coex_schm_register_callback(kind: u32, callback: unsafe extern "C" fn()) -> i32;
            }
            const COEX_SCHM_CALLBACK_TYPE_BT: u32 = 1;
            unsafe { coex_schm_register_callback(COEX_SCHM_CALLBACK_TYPE_BT, callback) }
        } else {
            0
        }
    }
}

#[allow(unused_variables)]
pub(crate) unsafe extern "C" fn coex_wifi_channel_get(
    _primary: *mut u8,
    _secondary: *mut u8,
) -> i32 {
    trace!("coex_wifi_channel_get");

    cfg_if::cfg_if! {
        if #[cfg(feature = "coex")] {
            unsafe extern "C" {
                fn coex_wifi_channel_get(_primary: *mut u8, _secondary: *mut u8) -> i32;
            }
            unsafe { coex_wifi_channel_get(_primary, _secondary) }
        } else {
            -1
        }
    }
}

pub(crate) unsafe extern "C" fn set_isr(n: i32, f: unsafe extern "C" fn(), arg: *const ()) -> i32 {
    trace!("set_isr called {} {:?} {:?}", n, f, arg);

    match n {
        5 => unsafe {
            ISR_INTERRUPT_5.set(f as *const c_void, arg.cast());
            BT::steal().enable_rwbt_interrupt(Priority::Priority1);
            BT::steal().enable_rwble_interrupt(Priority::Priority1);
        },
        7 => ISR_INTERRUPT_7.set(f as *const c_void, arg.cast()),
        8 => unsafe {
            ISR_INTERRUPT_8.set(f as *const c_void, arg.cast());
            BT::steal().enable_bb_interrupt(Priority::Priority1);
        },
        _ => panic!("set_isr - unsupported interrupt number {}", n),
    }

    0
}

pub(crate) unsafe extern "C" fn ints_on(mask: u32) {
    trace!("chip_ints_on esp32 {:b}", mask);
    unsafe {
        crate::hal::xtensa_lx::interrupt::enable_mask(mask);
    }
}

// Without modem sleep the controller never sleeps; these only flag the HCI send. With
// `ble_modem_sleep` the ESP-IDF versions further up replace them.
#[cfg(all(feature = "coex", not(ble_modem_sleep)))]
pub(crate) const BTDM_ASYNC_WAKEUP_REQ_HCI: i32 = 0;

#[cfg(all(feature = "coex", not(ble_modem_sleep)))]
pub(crate) const BTDM_ASYNC_WAKEUP_REQ_COEX: i32 = 1;

// const BTDM_ASYNC_WAKEUP_REQMAX: i32 = 2;

/// **************************************************************************
/// Name: async_wakeup_request
///
/// Description:
///   Request the BLE Controller to wakeup
///
/// Input Parameters:
///   event - the event that triggered the wakeup
///
/// Returned Value:
///   true if request lock is needed, false otherwise
///
/// *************************************************************************
#[cfg(all(feature = "coex", not(ble_modem_sleep)))]
pub(crate) fn async_wakeup_request(event: i32) -> bool {
    trace!("async_wakeup_request {}", event);

    unsafe extern "C" {
        fn btdm_in_wakeup_requesting_set(set: bool);

        fn btdm_power_state_active() -> bool;

        fn btdm_wakeup_request();
    }

    match event {
        e if e == BTDM_ASYNC_WAKEUP_REQ_HCI => {
            unsafe {
                btdm_in_wakeup_requesting_set(true);
            }
            false
        }
        e if e == BTDM_ASYNC_WAKEUP_REQ_COEX => {
            unsafe {
                btdm_in_wakeup_requesting_set(true);
            }

            if !unsafe { btdm_power_state_active() } {
                unsafe {
                    btdm_wakeup_request();
                }
                true
            } else {
                false
            }
        }
        _ => false,
    }
}

/// **************************************************************************
/// Name: async_wakeup_request_end
///
/// Description:
///   Finish a wakeup request
///
/// Input Parameters:
///   event - the event that triggered the wakeup
///
/// Returned Value:
///   true if request lock is needed, false otherwise
///
/// *************************************************************************
#[cfg(all(feature = "coex", not(ble_modem_sleep)))]
pub(crate) fn async_wakeup_request_end(event: i32) {
    trace!("async_wakeup_request_end {}", event);

    let request_lock = match event {
        e if e == BTDM_ASYNC_WAKEUP_REQ_HCI => true,
        e if e == BTDM_ASYNC_WAKEUP_REQ_COEX => false,
        _ => return,
    };

    unsafe extern "C" {
        fn btdm_in_wakeup_requesting_set(set: bool);
    }

    if request_lock {
        unsafe { btdm_in_wakeup_requesting_set(false) };
    }
}

#[allow(non_snake_case)]
#[unsafe(no_mangle)]
#[crate::hal::ram]
fn Software0() {
    ISR_INTERRUPT_7.dispatch();
}

#[unsafe(no_mangle)]
#[crate::hal::ram]
extern "C" fn RWBT() {
    ISR_INTERRUPT_5.dispatch();
}

#[unsafe(no_mangle)]
#[crate::hal::ram]
extern "C" fn RWBLE() {
    ISR_INTERRUPT_5.dispatch();
}

#[unsafe(no_mangle)]
#[crate::hal::ram]
extern "C" fn BT_BB() {
    ISR_INTERRUPT_8.dispatch();
}

pub(crate) fn shutdown_ble_isr() {
    unsafe {
        BT::steal().disable_rwbt_interrupt_on_all_cores();
        BT::steal().disable_rwble_interrupt_on_all_cores();
        BT::steal().disable_bb_interrupt_on_all_cores();
    }
}
