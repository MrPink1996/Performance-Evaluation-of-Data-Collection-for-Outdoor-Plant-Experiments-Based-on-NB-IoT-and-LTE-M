/*
 * Copyright (c) 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/kernel.h>
#include <ctype.h>
#include <zephyr/drivers/gpio.h>
#include <stdio.h>
#include <zephyr/net/lwm2m.h>
#include <modem/nrf_modem_lib.h>
#include <net/lwm2m_client_utils.h>
#include <app_event_manager.h>
#include <net/lwm2m_client_utils_location.h>
#include <date_time.h>
#include <zephyr/net/socket.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(app_lwm2m_client, CONFIG_APP_LOG_LEVEL);

#include <modem/lte_lc.h>
#include <modem/modem_info.h>
#include <nrf_modem_at.h>

#include "lwm2m_client_app.h"
#include "lwm2m_app_utils.h"
#include "gnss_module.h"
#include "location_events.h"

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include "fota_app_external.h"
#include <dk_buttons_and_leds.h>
#include <zephyr/sys/reboot.h>

#if defined(CONFIG_LWM2M_CLIENT_UTILS_LOCATION_ASSISTANCE)
#include "ui_input.h"
#include "ui_input_event.h"
#endif

#if !defined(CONFIG_LTE_LINK_CONTROL)
#error "Missing CONFIG_LTE_LINK_CONTROL"
#endif

#define APP_BANNER "Run LWM2M client"

#define IMEI_LEN 15
#define ENDPOINT_NAME_LEN (IMEI_LEN + sizeof(CONFIG_APP_ENDPOINT_PREFIX) + 1)

#define LWM2M_SECURITY_PRE_SHARED_KEY 0
#define LWM2M_SECURITY_RAW_PUBLIC_KEY 1
#define LWM2M_SECURITY_CERTIFICATE 2
#define LWM2M_SECURITY_NO_SEC 3

#define CONNEVAL_MAX_DELAY_S 60
#define CONNEVAL_POLL_PERIOD_MS 5000
#define RC_STR(rc) ((rc) == 0 ? "OK" : "ERROR")
#define PRINT_RESULT(func, rc) LOG_INF("%s: %d <%s>", (func), rc, RC_STR(rc))



/* Client State Machine states */
static enum client_state {
	START,		/* Start Connection to a server*/
	CONNECTING,	/* LwM2M engine is connecting to server */
	BOOTSTRAP,	/* LwM2M engine is doing a bootstrap */
	CONNECTED,	/* LwM2M Client connection establisment to server */
	LTE_OFFLINE,	/* LTE offline and LwM2M engine should be suspended */
	UPDATE_FIRMWARE, /* Prepare app ready for firmware update */
	RECONNECT_AFTER_UPDATE, /* Reconnect client after modem update */
	NETWORK_ERROR	/* Client network error handling. Client stop and modem reset */
} client_state = START;

static struct lte_config {
	char apn[100];
	int32_t rsrp;
	int32_t rsrq;
	int32_t mcc;
	int32_t mnc;
	int32_t snr;
	int32_t tx_power;
	int32_t cell_id;
	int32_t phy_cell_id;
	float edrx;
	int32_t active_time;
	int32_t energy_estimate;
	int32_t tau_trig;
	int32_t tau;
	int32_t rx_repetitions;
	int32_t tx_repetitions;
	int32_t dl_pathloss;
	int32_t ce_level;
	char tac[100];
	int32_t earfcn;
	int32_t band;
	int64_t connection_time_link;
	int64_t connection_time_protocol;
	int64_t connection_time_gps;
	enum lte_lc_lte_mode mode;
	float latitude;
	float longitude;
	float accuracy;
	int64_t timestamp;
	double motherboard_voltage;
	double throughput_protocol;
	double throughput_link;
	char device_id[32];
}lte_params;



static int speedtest(double *throughput){
	int sock;
	struct sockaddr_in server4;
	
	int err;

	sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (sock < 0) {
		LOG_ERR("Failed to create UDP socket: %d", errno);
		return -errno;
	}

	memset(&server4, 0, sizeof(server4));
	server4.sin_family = AF_INET;
	server4.sin_port = htons(5000);

	struct addrinfo *result;
	struct addrinfo *addr;
	struct addrinfo hints = {
		.ai_family = AF_INET,
		.ai_socktype = SOCK_DGRAM  // use SOCK_DGRAM for UDP
	};

	err = getaddrinfo("customadress", NULL, &hints, &result);
	if (err) {
		LOG_ERR("getaddrinfo failed: %d", err);
		return -ECHILD;
	}

	addr = result;
	server4.sin_addr = ((struct sockaddr_in *)addr->ai_addr)->sin_addr;
	char msg[512];
	char msg_1[1] = "1";
	char msg_rec[4];
	// snprintk(msg, sizeof(msg), "%lld", timestamp);
	for (int i = 0; i < sizeof(msg); i++) {
		msg[i] = 'a' + (i % 26);
	}

	err = sendto(sock, msg_1, sizeof(msg_1), 0, (struct sockaddr *)&server4, sizeof(server4));
	err = sendto(sock, msg, sizeof(msg), 0, (struct sockaddr *)&server4, sizeof(server4));
	err = recv(sock, msg_rec, sizeof(msg_rec), 0);
	if (err < 0){
		LOG_ERR("Failed to receive UDP message: %d", errno);
		close(sock);
		return -errno;	
	}
	uint8_t temp[4] = {msg_rec[3], msg_rec[2], msg_rec[1], msg_rec[0]};
	float value = 0;
	memcpy(&value, temp, sizeof(value));
	*throughput = (double)value;
	close(sock);
	return 0;
}

static uint8_t endpoint_name[ENDPOINT_NAME_LEN + 1];
static uint8_t imei_buf[IMEI_LEN + sizeof("\r\nOK\r\n")];
static struct lwm2m_ctx client = {0};
static bool reconnect;
static K_SEM_DEFINE(state_mutex, 0, 1);
static K_MUTEX_DEFINE(lte_mutex);
static bool modem_connected_to_network;
/* Enable session lifetime check for initial boot */
static bool update_session_lifetime = true;
static bool ready_for_firmware_update;

static void rd_client_event(struct lwm2m_ctx *client, enum lwm2m_rd_client_event client_event);
static void state_trigger_and_unlock(enum client_state new_state);

void client_acknowledge(void)
{
	lwm2m_acknowledge(&client);
}

#if defined(CONFIG_LWM2M_CLIENT_UTILS_SIGNAL_MEAS_INFO_OBJ_SUPPORT)
static struct k_work_delayable ncell_meas_work;
void ncell_meas_work_handler(struct k_work *work)
{
	lwm2m_ncell_schedule_measurement();
	k_work_schedule(&ncell_meas_work, K_SECONDS(CONFIG_APP_NEIGHBOUR_CELL_SCAN_INTERVAL));
}
#endif
#if defined(CONFIG_LWM2M_CLIENT_UTILS_VISIBLE_WIFI_AP_OBJ_SUPPORT)
static struct k_work_delayable ground_fix_work;
void ground_fix_work_handler(struct k_work *work)
{
	LOG_INF("Send ground fix location request event");
	struct ground_fix_location_request_event *ground_fix_event =
	new_ground_fix_location_request_event();

	APP_EVENT_SUBMIT(ground_fix_event);
}
#endif

#if defined(CONFIG_LWM2M_CLIENT_UTILS_LOCATION_ASSISTANCE)
static bool button_callback(const struct app_event_header *aeh)
{
	if (is_ui_input_event(aeh)) {
		struct ui_input_event *event = cast_ui_input_event(aeh);

		if (event->type != PUSH_BUTTON || event->state == 0) {
			return false;
		}

		switch (event->device_number) {
		case 1:
#if defined(CONFIG_LWM2M_CLIENT_UTILS_LOCATION_ASSIST_AGNSS) || \
defined(CONFIG_LWM2M_CLIENT_UTILS_LOCATION_ASSIST_PGPS)
			LOG_INF("Starting GNSS");
			start_gnss();
#else
			LOG_INF("A-GNSS not enabled");
#endif
			break;
		case 2:
#if defined(CONFIG_LWM2M_CLIENT_UTILS_GROUND_FIX_OBJ_SUPPORT)
			LOG_INF("Send cell location request event");
			struct ground_fix_location_request_event *ground_fix_event =
				new_ground_fix_location_request_event();

			APP_EVENT_SUBMIT(ground_fix_event);
#else
			LOG_INF("Ground fix location not enabled");
#endif
			break;
		}
	}
	return false;
}


APP_EVENT_LISTENER(app_lwm2m_client, button_callback);
APP_EVENT_SUBSCRIBE(app_lwm2m_client, ui_input_event);
#endif

#if defined(CONFIG_APP_LWM2M_CONFORMANCE_TESTING)
static struct k_work_delayable send_periodical_work;
static uint8_t send_count = 0;

static int server_send_mute_cb(uint16_t obj_inst_id, uint16_t res_id, uint16_t res_inst_id, uint8_t *data,
			uint16_t data_len, bool last_block, size_t total_size, size_t offset)
{
	if (*data) {
		LOG_INF("Server Muted Send");
	} else {

		if (send_count == 0) {
			LOG_INF("Server Activate Send");
			send_count = 5;
			k_work_schedule(&send_periodical_work, K_SECONDS(1));
		}
	}

	return 0;
}

static void lwm2m_register_server_send_mute_cb(void)
{
	int ret;
	lwm2m_engine_set_data_cb_t cb;

	cb = server_send_mute_cb;
	ret = lwm2m_register_post_write_callback(&LWM2M_OBJ(1, client.srv_obj_inst, 23), cb);
	if (ret) {
		LOG_ERR("Send enable CB fail %d", ret);
	}
}

void send_periodically_work_handler(struct k_work *work)
{
	int ret;
	const struct lwm2m_obj_path send_path[4] = {
		LWM2M_OBJ(3, 0, 0),
		LWM2M_OBJ(3, 0, 3),
		LWM2M_OBJ(3, 0, 13),
		LWM2M_OBJ(3, 0, 19),
	};

	/* lwm2m send post to server */
	ret = lwm2m_send_cb(&client, send_path, 4, NULL);
	if (ret) {
		if (ret == EPERM) {
			LOG_INF("Server Mute send block send operation");
		} else {
			LOG_INF("Periodically SEND test data fail %d", ret);
		}
	}

	if (send_count) {
		if (ret == 0) {
			send_count--;
		}

		k_work_schedule(&send_periodical_work, K_SECONDS(15));
	}
}
#endif

static void state_trigger_and_unlock(enum client_state new_state)
{
	if (new_state != client_state) {
		client_state = new_state;
		k_sem_give(&state_mutex);
	}
	k_mutex_unlock(&lte_mutex);
}

#if defined(CONFIG_LWM2M_CLIENT_UTILS_FIRMWARE_UPDATE_OBJ_SUPPORT)
static int lwm2m_firmware_event_cb(struct lwm2m_fota_event *event)
{
	k_mutex_lock(&lte_mutex, K_FOREVER);
	switch (event->id) {
	case LWM2M_FOTA_DOWNLOAD_START:
		ready_for_firmware_update = false;
		LOG_INF("FOTA download started for instance %d", event->download_start.obj_inst_id);
		break;
	/** FOTA download process finished */
	case LWM2M_FOTA_DOWNLOAD_FINISHED:
		LOG_INF("FOTA download ready for instance %d, dfu_type %d",
			event->download_ready.obj_inst_id, event->download_ready.dfu_type);
		break;
	/** FOTA update new image */
	case LWM2M_FOTA_UPDATE_IMAGE_REQ:
		if (!ready_for_firmware_update && event->update_req.obj_inst_id < 2) {
			state_trigger_and_unlock(UPDATE_FIRMWARE);
			/* Postpone request by 2 seconds */
			return 2;
		}
		LOG_INF("FOTA Update request for instance %d", event->update_req.obj_inst_id);

		break;
	case LWM2M_FOTA_UPDATE_MODEM_RECONNECT_REQ:
		ready_for_firmware_update = false;
		state_trigger_and_unlock(RECONNECT_AFTER_UPDATE);
		/* Indicate that app can support Modem Reconnect */
		return 0;
	/** Fota process fail or cancelled  */
	case LWM2M_FOTA_UPDATE_ERROR:
		ready_for_firmware_update = false;
		LOG_INF("FOTA failure %d by status %d", event->failure.obj_inst_id,
			event->failure.update_failure);
		break;
	}
	k_mutex_unlock(&lte_mutex);
	return 0;
}
#endif

static int lwm2m_setup(void)
{
	/* Save power by not updating timestamp on device object */
	lwm2m_update_device_service_period(0);

	/* Manufacturer dependent */
	/* use IMEI as serial number */
	lwm2m_app_init_device(imei_buf);
	lwm2m_init_security(&client, endpoint_name, NULL);

	if (sizeof(CONFIG_APP_LWM2M_PSK) > 1) {
		/* Write hard-coded PSK key to engine */
		/* First security instance is the right one, because in bootstrap mode, */
		/* it is the bootstrap PSK. In normal mode, it is the server key */
		lwm2m_security_set_psk(0, CONFIG_APP_LWM2M_PSK, sizeof(CONFIG_APP_LWM2M_PSK), true,
				       endpoint_name);
	}

#if defined(CONFIG_LWM2M_CLIENT_UTILS_FIRMWARE_UPDATE_OBJ_SUPPORT)
	lwm2m_init_firmware_cb(lwm2m_firmware_event_cb);
#endif
#if defined(CONFIG_LWM2M_CLIENT_UTILS_LOCATION_ASSISTANCE)
	location_event_handler_init(&client);
	location_assistance_retry_init(true);
#endif
	if (IS_ENABLED(CONFIG_LTE_LC_TAU_PRE_WARNING_NOTIFICATIONS) ||
	    IS_ENABLED(CONFIG_LWM2M_CLIENT_UTILS_NEIGHBOUR_CELL_LISTENER)) {
		lwm2m_ncell_handler_register();
	}

	return 0;
}

static void date_time_event_handler(const struct date_time_evt *evt)
{
	switch (evt->type) {
	case DATE_TIME_OBTAINED_MODEM: {
		int64_t time = 0;

		LOG_INF("Obtained date-time from modem");
		date_time_now(&time);
		lwm2m_set_s32(&LWM2M_OBJ(LWM2M_OBJECT_DEVICE_ID, 0, CURRENT_TIME_RID),
				     (int32_t)(time / 1000));
		break;
	}

	case DATE_TIME_OBTAINED_NTP: {
		int64_t time = 0;

		LOG_INF("Obtained date-time from NTP server");
		date_time_now(&time);
		lwm2m_set_s32(&LWM2M_OBJ(LWM2M_OBJECT_DEVICE_ID, 0, CURRENT_TIME_RID),
				     (int32_t)(time / 1000));
		break;
	}

	case DATE_TIME_NOT_OBTAINED:
		LOG_INF("Could not obtain date-time update");
		break;

	default:
		break;
	}
}

static void rd_client_update_lifetime(int srv_obj_inst)
{
	uint32_t current_lifetime = 0;

	uint32_t lifetime = CONFIG_LWM2M_ENGINE_DEFAULT_LIFETIME;

	struct lwm2m_obj_path path = LWM2M_OBJ(1, srv_obj_inst, 1);

	lwm2m_get_u32(&path, &current_lifetime);

	if (current_lifetime != lifetime) {
		/* SET Configured value */
		lwm2m_set_u32(&path, lifetime);
		LOG_DBG("Update session lifetime from %d to %d", current_lifetime, lifetime);
	}
	update_session_lifetime = false;
}

static void state_set_and_unlock(enum client_state new_state)
{
	client_state = new_state;
	k_mutex_unlock(&lte_mutex);
}

static void rd_client_event(struct lwm2m_ctx *client, enum lwm2m_rd_client_event client_event)
{
	k_mutex_lock(&lte_mutex, K_FOREVER);

	if (client_state == LTE_OFFLINE && client_event != LWM2M_RD_CLIENT_EVENT_ENGINE_SUSPENDED) {
		LOG_DBG("Drop network event %d at LTE offline state", client_event);
		k_mutex_unlock(&lte_mutex);
		return;
	}

	lwm2m_utils_connection_manage(client, &client_event);

	switch (client_event) {
	case LWM2M_RD_CLIENT_EVENT_SERVER_DISABLED:
	case LWM2M_RD_CLIENT_EVENT_DEREGISTER:
	case LWM2M_RD_CLIENT_EVENT_NONE:
		/* do nothing */
		k_mutex_unlock(&lte_mutex);
		break;

	case LWM2M_RD_CLIENT_EVENT_BOOTSTRAP_REG_FAILURE:
		LOG_DBG("Bootstrap registration failure!");
		state_trigger_and_unlock(NETWORK_ERROR);
		break;

	case LWM2M_RD_CLIENT_EVENT_BOOTSTRAP_REG_COMPLETE:
		LOG_DBG("Bootstrap registration complete");
		update_session_lifetime = true;
		state_trigger_and_unlock(BOOTSTRAP);
		break;

	case LWM2M_RD_CLIENT_EVENT_BOOTSTRAP_TRANSFER_COMPLETE:
		LOG_DBG("Bootstrap transfer complete");
		k_mutex_unlock(&lte_mutex);
		break;

	case LWM2M_RD_CLIENT_EVENT_REGISTRATION_FAILURE:
		LOG_WRN("Registration failure!");
		state_trigger_and_unlock(CONNECTING);
		break;

	case LWM2M_RD_CLIENT_EVENT_REGISTRATION_COMPLETE:
		LOG_DBG("Registration complete");
		state_trigger_and_unlock(CONNECTED);
		break;

	case LWM2M_RD_CLIENT_EVENT_REG_TIMEOUT:
		LOG_DBG("Registration update failure!");
		state_trigger_and_unlock(CONNECTING);
		break;

	case LWM2M_RD_CLIENT_EVENT_REG_UPDATE:
		LOG_DBG("Registration update started");
		k_mutex_unlock(&lte_mutex);
		break;

	case LWM2M_RD_CLIENT_EVENT_REG_UPDATE_COMPLETE:
		LOG_DBG("Registration update complete");
		state_trigger_and_unlock(CONNECTED);
		break;

	case LWM2M_RD_CLIENT_EVENT_DEREGISTER_FAILURE:
		LOG_DBG("Deregister failure!");
		if (client_state != UPDATE_FIRMWARE) {
			state_set_and_unlock(START);
		} else {
			k_mutex_unlock(&lte_mutex);
		}
		break;

	case LWM2M_RD_CLIENT_EVENT_DISCONNECT:
		LOG_DBG("Disconnected");
		if (client_state != UPDATE_FIRMWARE && client_state != RECONNECT_AFTER_UPDATE) {
			state_set_and_unlock(START);
		} else {
			k_mutex_unlock(&lte_mutex);
		}
		break;

	case LWM2M_RD_CLIENT_EVENT_QUEUE_MODE_RX_OFF:
		LOG_DBG("Queue mode RX window closed");
		k_mutex_unlock(&lte_mutex);
		break;

	case LWM2M_RD_CLIENT_EVENT_ENGINE_SUSPENDED:
		LOG_DBG("LwM2M engine suspended");
		k_mutex_unlock(&lte_mutex);
		break;

	case LWM2M_RD_CLIENT_EVENT_NETWORK_ERROR:
		LOG_ERR("LwM2M engine reported a network error.");
		reconnect = true;
		state_trigger_and_unlock(NETWORK_ERROR);
		break;
	}
}

static int get_parameters(struct lte_config *lte_params, bool print_result) {
	int err;
	struct lte_lc_conn_eval_params params = {0};

	err = lte_lc_conn_eval_params_get(&params);
	PRINT_RESULT("lte_lc_conn_eval_params_get", err);

	lte_params->tx_power = params.tx_power;
	lte_params->band = params.band;
	lte_params->ce_level = params.ce_level;
	lte_params->cell_id = params.cell_id;
	lte_params->dl_pathloss = params.dl_pathloss;
	lte_params->earfcn = params.earfcn;
	lte_params->mcc = params.mcc;
	lte_params->mnc = params.mnc;
	lte_params->energy_estimate = params.energy_estimate;
	lte_params->phy_cell_id = params.phy_cid;
	lte_params->tau_trig = params.tau_trig;
	lte_params->rsrp = params.rsrp;
	lte_params->rsrq = params.rsrq;
	lte_params->snr = params.snr;
	lte_params->tx_repetitions = params.tx_rep;
	lte_params->rx_repetitions = params.rx_rep;
	lte_params->snr = params.snr;

	struct modem_param_info modem_param;

	err = modem_info_init();
	PRINT_RESULT("modem_info_init", err);

	err = modem_info_params_init(&modem_param);
	PRINT_RESULT("modem_info_params_init", err);

	err = modem_info_params_get(&modem_param);
	PRINT_RESULT("modem_info_params_get", err);

	snprintf(lte_params->apn, sizeof(lte_params->apn), "%s", modem_param.network.apn.value_string);
	snprintf(lte_params->tac, sizeof(lte_params->tac), "%s", modem_param.network.area_code.value_string);
	lte_params->band = modem_param.network.current_band.value;

	err = lte_lc_psm_get(&lte_params->tau, &lte_params->active_time);
	PRINT_RESULT("lte_lc_psm_get", err);

	struct lte_lc_edrx_cfg edrx_cfg;
	err = lte_lc_edrx_get(&edrx_cfg);
	PRINT_RESULT("lte_lc_edrx_get", err);
	lte_params->edrx = edrx_cfg.edrx;
	if (print_result == true){
		LOG_INF("Modem parameters:");
		LOG_INF("APN: %s", lte_params->apn);
		LOG_INF("TAC: %s", lte_params->tac);
		LOG_INF("MCC: %d", lte_params->mcc);
		LOG_INF("MNC: %d", lte_params->mnc);
		LOG_INF("Band: %d", lte_params->band);
		LOG_INF("Earfcn: %d", lte_params->earfcn);
		LOG_INF("Cell ID: %d", lte_params->cell_id);
		LOG_INF("Phy Cell ID: %d", lte_params->phy_cell_id);
		LOG_INF("Tx Power: %d", lte_params->tx_power);
		LOG_INF("Rx Repetitions: %d", lte_params->rx_repetitions);
		LOG_INF("Tx Repetitions: %d", lte_params->tx_repetitions);
		LOG_INF("Energy Estimate: %d", lte_params->energy_estimate);
		LOG_INF("Active Time: %d", lte_params->active_time);
		LOG_INF("CE Level: %d", lte_params->ce_level);
		LOG_INF("eDRX: %f", (double)lte_params->edrx);
		LOG_INF("RSRP: %d", lte_params->rsrp);
		LOG_INF("RSRQ: %d", lte_params->rsrq);
		LOG_INF("SNR: %d", lte_params->snr);
		LOG_INF("DL Pathloss: %d", lte_params->dl_pathloss);
		LOG_INF("Tau: %d", lte_params->tau);
		LOG_INF("TAU triggered: %d", lte_params->tau_trig);
		LOG_INF("Link: %d", lte_params->mode);
	}
	return 0;
}


static void modem_connect()
{
	int ret;

	do {

		LOG_INF("Connecting to network.");
		LOG_INF("This may take several minutes.");

		ret = lte_lc_connect();
		if (ret < 0) {
			LOG_WRN("Failed to establish LTE connection (%d).", ret);
			LOG_WRN("Will retry in a minute.");
			lte_lc_offline();
			k_sleep(K_SECONDS(60));
		} else {
			enum lte_lc_lte_mode mode;

			lte_lc_lte_mode_get(&mode);
			if (mode == LTE_LC_LTE_MODE_NBIOT) {
				LOG_INF("Connected to NB-IoT network");
			} else if (mode == LTE_LC_LTE_MODE_LTEM) {
				LOG_INF("Connected to LTE network");
			} else  {
				LOG_INF("Connected to unknown network");
			}
		}
	} while (ret < 0);

	if (IS_ENABLED(CONFIG_LWM2M_CLIENT_UTILS_LTE_CONNEVAL)) {
		ret = lwm2m_utils_enable_conneval(LTE_LC_ENERGY_CONSUMPTION_NORMAL,
						  CONNEVAL_MAX_DELAY_S, CONNEVAL_POLL_PERIOD_MS);
		if (ret < 0) {
			LOG_ERR("Failed to enable conneval (%d)", ret);
		} else {
			LOG_INF("Conneval enabled");
		}
	}
}

static bool lte_connected(enum lte_lc_nw_reg_status nw_reg_status)
{
	if ((nw_reg_status == LTE_LC_NW_REG_REGISTERED_HOME) ||
	    (nw_reg_status == LTE_LC_NW_REG_REGISTERED_ROAMING)) {
		return true;
	}

	return false;
}

static void lwm2m_lte_reg_handler_notify(enum lte_lc_nw_reg_status nw_reg_status)
{
	bool lte_registered;

	LOG_DBG("LTE NW status: %d", nw_reg_status);
	k_mutex_lock(&lte_mutex, K_FOREVER);
	lte_registered = lte_connected(nw_reg_status);
	if (lte_registered != modem_connected_to_network) {
		modem_connected_to_network = lte_registered;
		if (client_state != START && client_state != BOOTSTRAP &&
		    client_state != UPDATE_FIRMWARE && client_state != RECONNECT_AFTER_UPDATE) {
			k_sem_give(&state_mutex);
		}
	}
	k_mutex_unlock(&lte_mutex);
}

#ifdef CONFIG_LTE_LC_MODEM_SLEEP_NOTIFICATIONS
static void lte_modem_enter_sleep(const struct lte_lc_modem_sleep *event)
{
	switch (event->type) {
	case LTE_LC_MODEM_SLEEP_PSM:
	case LTE_LC_MODEM_SLEEP_PROPRIETARY_PSM:
		LOG_INF("Modem Enter PSM, time %lld", event->time);
		break;
	case LTE_LC_MODEM_SLEEP_RF_INACTIVITY:
		LOG_INF("Modem Enter eDRX state, time %lld", event->time);
		break;
	default:
		break;
	}
}
#endif

static void lte_notify_handler(const struct lte_lc_evt *const evt)
{
	switch (evt->type) {
	case LTE_LC_EVT_NW_REG_STATUS:
		lwm2m_lte_reg_handler_notify(evt->nw_reg_status);
		break;
#ifdef CONFIG_LTE_LC_MODEM_SLEEP_NOTIFICATIONS
	case LTE_LC_EVT_MODEM_SLEEP_ENTER:
		lte_modem_enter_sleep(&evt->modem_sleep);
		break;
#endif
	default:
		break;
	}
}

static void suspend_lwm2m_engine(void)
{
	int ret;

	state_trigger_and_unlock(LTE_OFFLINE);
	ret = lwm2m_engine_pause();
	if (ret) {
		LOG_ERR("LwM2M engine pause fail %d", ret);
		reconnect = true;
		k_mutex_lock(&lte_mutex, K_FOREVER);
		state_trigger_and_unlock(NETWORK_ERROR);
	}
}

static int switch_to_mode(enum lte_lc_system_mode set_mode, struct lte_config *lte_params) {
    int err;

    // Disconnect from the network
    err = lte_lc_offline();
    if (err) {
        LOG_ERR("Failed to set modem offline, error: %d", err);
        return err;
    }

    // Set the system mode
    err = lte_lc_system_mode_set(set_mode, LTE_LC_SYSTEM_MODE_PREFER_AUTO);
    if (err) {
        LOG_ERR("Failed to set system mode, error: %d", err);
        return err;
    }

	if (set_mode == LTE_LC_SYSTEM_MODE_GPS){
		return err;
	}

    // Reconnect to the network
	int64_t time_now = k_uptime_get();
    err = lte_lc_connect();
    if (err) {
        LOG_ERR("Failed to connect to the network, error: %d", err);
        return err;
    }
	lte_params->connection_time_link = k_uptime_get() - time_now;

	// Get the current LTE mode
	lte_lc_lte_mode_get(&lte_params->mode);
    LOG_INF("Connected to the network in mode: %d", lte_params->mode);
    return err;
}



int main(void)
{
	int ret;
	uint32_t bootstrap_flags = 0;

	LOG_INF("INITIALIZE DK");

	ret = dk_leds_init();
	PRINT_RESULT("dk_leds_init", ret);

	ret = dk_buttons_init(NULL);
	PRINT_RESULT("dk_buttons_init", ret);

	uint32_t button_state;
	uint32_t button_changed;

	while (true){
		dk_read_buttons(&button_state, &button_changed);
		if (button_state & DK_BTN1_MSK) {
			LOG_INF("Button 1 pressed, start Program");
			break;
		}
		if (button_state & DK_BTN2_MSK) {
			LOG_INF("Button 2 pressed, start Program");
			break;
		}
		k_sleep(K_MSEC(50));
	}
	for (int i = 0; i < 2; i ++){
		for (int i = 0; i < 4; i++) {
		ret = dk_set_led_on(i);
		k_sleep(K_MSEC(100));
		}
		for (int i = 3; i >= 0; i--) {
			ret = dk_set_led_off(i);
			k_sleep(K_MSEC(100));
		}
	}
	

	ret = nrf_modem_lib_init();
	if (ret < 0) {
		LOG_ERR("Unable to init modem library (%d)", ret);
		return 0;
	}

	if (IS_ENABLED(CONFIG_APP_SMP_CLIENT_FOTA_EXTERNAL)) {
		ret = fota_external_init();
		if (ret < 0) {
			LOG_ERR("Unable to init Fota external client (%d)", ret);
			return 0;
		}
	}

	ret = app_event_manager_init();
	if (ret) {
		LOG_ERR("Unable to init Application Event Manager (%d)", ret);
		return 0;
	}

	lte_lc_register_handler(lte_notify_handler);

	ret = modem_info_init();
	if (ret < 0) {
		LOG_ERR("Unable to init modem_info (%d)", ret);
		return 0;
	}

	/* query IMEI */
	ret = modem_info_string_get(MODEM_INFO_IMEI, imei_buf, sizeof(imei_buf));
	if (ret < 0) {
		LOG_ERR("Unable to get IMEI");
		return 0;
	}


	/* use IMEI as unique endpoint name */
	snprintk(endpoint_name, sizeof(endpoint_name), "%s%.*s", CONFIG_APP_ENDPOINT_PREFIX,
		 IMEI_LEN, imei_buf);
	LOG_INF("endpoint: %s", (char *)endpoint_name);

	/* Setup LwM2M */
	ret = lwm2m_setup();
	if (ret < 0) {
		LOG_ERR("Failed to setup LWM2M fields (%d)", ret);
		return 0;
	}

	if (IS_ENABLED(CONFIG_LWM2M_CLIENT_UTILS_FIRMWARE_UPDATE_OBJ_SUPPORT)) {
		ret = lwm2m_init_image();
		if (ret < 0) {
			LOG_ERR("Failed to setup image properties (%d)", ret);
			return 0;
		}
	}

	//modem_connect(LTE_LC_SYSTEM_MODE_LTEM);
	switch_to_mode(LTE_LC_SYSTEM_MODE_LTEM, &lte_params);
	dk_set_led_on(DK_LED1);
	k_sleep(K_SECONDS(3));
	get_parameters(&lte_params, true);
	speedtest(&lte_params.throughput_link);
	LOG_INF("Link throughput: %f", lte_params.throughput_link);

#if defined(CONFIG_APP_GNSS)
	initialise_gnss();
#endif

#if defined(CONFIG_LWM2M_CLIENT_UTILS_SIGNAL_MEAS_INFO_OBJ_SUPPORT)
	k_work_init_delayable(&ncell_meas_work, ncell_meas_work_handler);
	k_work_schedule(&ncell_meas_work, K_SECONDS(1));
#endif
#if defined(CONFIG_LWM2M_CLIENT_UTILS_VISIBLE_WIFI_AP_OBJ_SUPPORT)
	k_work_init_delayable(&ground_fix_work, ground_fix_work_handler);
	k_work_schedule(&ground_fix_work, K_SECONDS(60));
#if defined(CONFIG_LWM2M_CLIENT_UTILS_WIFI_AP_SCANNER)
	lwm2m_wifi_request_scan();
#endif
#endif
#if defined(CONFIG_APP_LWM2M_CONFORMANCE_TESTING)
	k_work_init_delayable(&send_periodical_work, send_periodically_work_handler);
#endif

	// Start timer 
	int64_t start_time = 0;
	int64_t time_now = 0;
	while (true) {

		if (start_time != 0 && (k_uptime_get() - start_time > 20000)){
			LOG_INF("Turning off LTE M ");
			//switch_to_mode(LTE_LC_SYSTEM_MODE_NBIOT);

			state_trigger_and_unlock(START);
			lwm2m_rd_client_stop(&client, rd_client_event, false);

			break;
		}
		
		if (lwm2m_security_needs_bootstrap()) {
			bootstrap_flags = LWM2M_RD_CLIENT_FLAG_BOOTSTRAP;
		} else {
			bootstrap_flags = 0;
		}

		k_mutex_lock(&lte_mutex, K_FOREVER);

		switch (client_state) {
		case START:
			LOG_INF("Client connect to server");
			time_now = k_uptime_get();
			ret = lwm2m_rd_client_start(&client, endpoint_name, bootstrap_flags,
						    rd_client_event, NULL);
			if (ret) {
				state_trigger_and_unlock(NETWORK_ERROR);
			} else {
				state_trigger_and_unlock(CONNECTING);
			}
			break;

		case BOOTSTRAP:
			state_set_and_unlock(BOOTSTRAP);
			LOG_INF("LwM2M is bootstrapping");
			break;

		case CONNECTING:
			LOG_INF("LwM2M is connecting to server");
			k_mutex_unlock(&lte_mutex);
			break;

		case CONNECTED:
			if (!modem_connected_to_network) {
				/* LTE connection down suspend LwM2M engine */
				suspend_lwm2m_engine();
			} else {
				k_mutex_unlock(&lte_mutex);
				LOG_INF("LwM2M is connected to server");
				dk_set_led_on(DK_LED2);
				lte_params.connection_time_protocol = k_uptime_get() - time_now;
				if (start_time == 0){
					LOG_INF("Start start_time");
					start_time = k_uptime_get();
				}
				if (update_session_lifetime) {
					/* Read a current server  lifetime value */
					rd_client_update_lifetime(client.srv_obj_inst);
				}

#if defined(CONFIG_APP_LWM2M_CONFORMANCE_TESTING)
				lwm2m_register_server_send_mute_cb();
#endif
				/* Get current time and date */
				date_time_update_async(date_time_event_handler);
			}
			break;

		case LTE_OFFLINE:
			if (modem_connected_to_network) {
				state_trigger_and_unlock(CONNECTING);
				LOG_INF("Resume LwM2M engine");
				ret = lwm2m_engine_resume();
				if (ret) {
					LOG_ERR("LwM2M engine Resume fail %d", ret);
				}
			} else {
				LOG_INF("LTE Offline");
				k_mutex_unlock(&lte_mutex);
			}
			break;

		case UPDATE_FIRMWARE:
			LOG_INF("Prepare for Firmware update: Stop client and disable Modem");
			k_mutex_unlock(&lte_mutex);
			lwm2m_rd_client_stop(&client, NULL, false);
			ready_for_firmware_update = true;
			LOG_INF("App ready for firmware update");
			break;

		case RECONNECT_AFTER_UPDATE:
			/* Enable client reconnect */
			LOG_INF("Restart modem and client after an update");
			state_trigger_and_unlock(START);
			modem_connect();
			break;

		case NETWORK_ERROR:
			/* Stop the LwM2M engine. */
			state_trigger_and_unlock(START);
			lwm2m_rd_client_stop(&client, rd_client_event, false);

			/* Set network state to start for blocking LTE */
			if (reconnect) {
				reconnect = false;

				LOG_INF("LwM2M restart requested. The sample will try to"
					" re-establish network connection.");

				/* Try to reconnect to the network. */
				ret = lte_lc_offline();
				if (ret < 0) {
					LOG_ERR("Failed to put LTE link in offline state (%d)",
						ret);
				}
				modem_connect();
			}
#if defined(CONFIG_APP_LWM2M_CONFORMANCE_TESTING)
			k_work_cancel_delayable(&send_periodical_work);
			send_count = 0;
#endif
			break;
		}

		/* Wait for statmachine update event */
		// k_sem_take(&state_mutex, K_FOREVER);
		k_sem_take(&state_mutex, K_MSEC(22000));
	
	}
	LOG_INF("CONNECTION_TIME_LINK_LTEM: %lld", lte_params.connection_time_link);
	LOG_INF("CONNECTION_TIME_PROTOCOL_LTEM: %lld", lte_params.connection_time_protocol);
	k_sleep(K_SECONDS(3));
	// Start timer 
	switch_to_mode(LTE_LC_SYSTEM_MODE_NBIOT, &lte_params);
	dk_set_led_on(DK_LED3);
	k_sleep(K_SECONDS(3));
	get_parameters(&lte_params, true);
	speedtest(&lte_params.throughput_link);
	LOG_INF("Link throughput: %f", lte_params.throughput_link);
	start_time = 0;
	while (true) {

		if (start_time != 0 && (k_uptime_get() - start_time > 20000)){
			LOG_INF("Turning off NBIOT ");
			//switch_to_mode(LTE_LC_SYSTEM_MODE_NBIOT);
			state_trigger_and_unlock(START);
			lwm2m_rd_client_stop(&client, rd_client_event, false);
			lte_lc_power_off();

			break;
		}
		
		if (lwm2m_security_needs_bootstrap()) {
			bootstrap_flags = LWM2M_RD_CLIENT_FLAG_BOOTSTRAP;
		} else {
			bootstrap_flags = 0;
		}

		k_mutex_lock(&lte_mutex, K_FOREVER);

		switch (client_state) {
		case START:
			LOG_INF("Client connect to server");
			time_now = k_uptime_get();
			ret = lwm2m_rd_client_start(&client, endpoint_name, bootstrap_flags,
						    rd_client_event, NULL);
			if (ret) {
				state_trigger_and_unlock(NETWORK_ERROR);
			} else {
				state_trigger_and_unlock(CONNECTING);
			}
			break;

		case BOOTSTRAP:
			state_set_and_unlock(BOOTSTRAP);
			LOG_INF("LwM2M is bootstrapping");
			break;

		case CONNECTING:
			LOG_INF("LwM2M is connecting to server");
			k_mutex_unlock(&lte_mutex);
			break;

		case CONNECTED:
			if (!modem_connected_to_network) {
				/* LTE connection down suspend LwM2M engine */
				suspend_lwm2m_engine();
			} else {
				k_mutex_unlock(&lte_mutex);
				lte_params.connection_time_protocol = k_uptime_get() - time_now;
				dk_set_led_on(DK_LED4);
				LOG_INF("LwM2M is connected to server");
				if (start_time == 0){
					LOG_INF("Start start_time");
					start_time = k_uptime_get();
				}
				if (update_session_lifetime) {
					/* Read a current server  lifetime value */
					rd_client_update_lifetime(client.srv_obj_inst);
				}

#if defined(CONFIG_APP_LWM2M_CONFORMANCE_TESTING)
				lwm2m_register_server_send_mute_cb();
#endif
				/* Get current time and date */
				date_time_update_async(date_time_event_handler);
			}
			break;

		case LTE_OFFLINE:
			if (modem_connected_to_network) {
				state_trigger_and_unlock(CONNECTING);
				LOG_INF("Resume LwM2M engine");
				ret = lwm2m_engine_resume();
				if (ret) {
					LOG_ERR("LwM2M engine Resume fail %d", ret);
				}
			} else {
				LOG_INF("LTE Offline");
				k_mutex_unlock(&lte_mutex);
			}
			break;

		case UPDATE_FIRMWARE:
			LOG_INF("Prepare for Firmware update: Stop client and disable Modem");
			k_mutex_unlock(&lte_mutex);
			lwm2m_rd_client_stop(&client, NULL, false);
			ready_for_firmware_update = true;
			LOG_INF("App ready for firmware update");
			break;

		case RECONNECT_AFTER_UPDATE:
			/* Enable client reconnect */
			LOG_INF("Restart modem and client after an update");
			state_trigger_and_unlock(START);
			modem_connect();
			break;

		case NETWORK_ERROR:
			/* Stop the LwM2M engine. */
			state_trigger_and_unlock(START);
			lwm2m_rd_client_stop(&client, rd_client_event, false);

			/* Set network state to start for blocking LTE */
			if (reconnect) {
				reconnect = false;

				LOG_INF("LwM2M restart requested. The sample will try to"
					" re-establish network connection.");

				/* Try to reconnect to the network. */
				ret = lte_lc_offline();
				if (ret < 0) {
					LOG_ERR("Failed to put LTE link in offline state (%d)",
						ret);
				}
				modem_connect();
			}
#if defined(CONFIG_APP_LWM2M_CONFORMANCE_TESTING)
			k_work_cancel_delayable(&send_periodical_work);
			send_count = 0;
#endif
			break;
		}

		/* Wait for statmachine update event */
		// k_sem_take(&state_mutex, K_FOREVER);
		k_sem_take(&state_mutex, K_MSEC(22000));
	
	}
	LOG_INF("CONNECTION_TIME_LINK_NBIOT: %lld", lte_params.connection_time_link);
	LOG_INF("CONNECTION_TIME_PROTOCOL_NBIOT: %lld", lte_params.connection_time_protocol);
	ret = nrf_modem_lib_shutdown();
	LOG_INF("nrf_modem_lib_shutdown() returned %d", ret);

	LOG_INF("PROGRAMM FINISHED");
	dk_set_led_off(DK_LED1);
	dk_set_led_off(DK_LED2);
	dk_set_led_off(DK_LED3);
	dk_set_led_off(DK_LED4);
	for(int i = 0; i < 2; i ++){
		for (int i = 0; i < 4; i++){
			dk_set_led_on(DK_LED1);
			dk_set_led_on(DK_LED2);
			dk_set_led_on(DK_LED3);
			dk_set_led_on(DK_LED4);
			k_sleep(K_MSEC(100));
			dk_set_led_off(DK_LED1);
			dk_set_led_off(DK_LED2);
			dk_set_led_off(DK_LED3);
			dk_set_led_off(DK_LED4);
			k_sleep(K_MSEC(100));
		}
	}
	
	LOG_INF("End of program, shutting down");
	sys_reboot(SYS_REBOOT_COLD);
	return 0;
}
