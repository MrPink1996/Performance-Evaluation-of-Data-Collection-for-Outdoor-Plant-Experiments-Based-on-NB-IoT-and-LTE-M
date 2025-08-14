#include <zephyr/kernel.h>
#include <stdio.h>
#include <string.h>
#include <nrf_modem_at.h>
#include <zephyr/console/console.h>
#include <modem/lte_lc.h>
#include <zephyr/sys/reboot.h>
#include <modem/nrf_modem_lib.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/socket.h>
#include <zephyr/net/mqtt.h>
#include <zephyr/sys/util.h>
#include <dk_buttons_and_leds.h>
#include <cJSON.h>
#include <date_time.h>
#include <zephyr/net/tls_credentials.h>
#include "certificates.h"
#include <modem/modem_key_mgmt.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/devicetree.h>
#include <zephyr/device.h>
#include <zephyr/drivers/hwinfo.h>
#include <inttypes.h>
#include <modem/modem_info.h>
#include <zephyr/net/net_core.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_stats.h>

#include <supl_os_client.h>
#include <supl_session.h>
#include <nrf_modem_gnss.h>

LOG_MODULE_REGISTER(lte_ble_gw, CONFIG_LTE_BLE_GW_LOG_LEVEL);

#define RC_STR(rc) ((rc) == 0 ? "OK" : "ERROR")
#define PRINT_RESULT(func, rc) LOG_INF("%s: %d <%s>", (func), rc, RC_STR(rc))
#define SUPL_SERVER "supl.google.com"
#define SUPL_SERVER_PORT 7276
#define SUNNY 12
#define RAINY 8
#define CLOUDY 4
#define PROTOCOLL_MQTT 1
#define PROTOCOLL_COAP 0
#define CONFIG_MQTT_TLS_SEC_TAG 42
#define CONFIG_MQTT_TLS_PEER_VERIFY 2
#define DT_SPEC_AND_COMMA(node_id, prop, idx) ADC_DT_SPEC_GET_BY_IDX(node_id, idx),

static int supl_fd = -1;
static volatile bool assistance_active;
static struct nrf_modem_gnss_agnss_data_frame last_agnss;
static bool gnss_ready = false;
static int64_t gnss_time = 0;
float latitude;
float longitude;
float accuracy;

/* Data of ADC io-channels specified in devicetree. */
static const struct adc_dt_spec adc_channels[] = {
	DT_FOREACH_PROP_ELEM(DT_PATH(zephyr_user), io_channels, DT_SPEC_AND_COMMA)
};

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

static volatile bool assistance_active;
static struct sockaddr_storage broker;
static struct mqtt_client client;
static uint8_t rx_buffer[128];
static uint8_t tx_buffer[2048];
static struct pollfd fds[1];
static int nfds;
static uint8_t connected = false;

void nrf_modem_fault_handler(struct nrf_modem_fault_info *fault_info)
{
	// error_handler(ERROR_MODEM_RECOVERABLE, (int)fault_info->reason);
	LOG_ERR("Modem fault: %d", fault_info->reason);
}

static void prepare_fds(struct mqtt_client *client)
{	
	fds[0].fd = client->transport.tls.sock;
	fds[0].events = POLLIN;
	nfds = 1;
}

static void clear_fds(void)
{
	nfds = 0;
}

static int wait(int timeout)
{
	int ret = 0;

	if (nfds > 0) {
		ret = poll(fds, nfds, mqtt_keepalive_time_left(&client));
		if (ret < 0) {
			LOG_ERR("poll error: %d", errno);
		}
	}

	return ret;
}

void mqtt_evt_handler(struct mqtt_client *const client,const struct mqtt_evt *evt)
{
	switch (evt->type) {
		case MQTT_EVT_CONNACK:
			if (evt->result != 0) {
			LOG_ERR("MQTT connect failed %d", evt->result);
			break;
			}
			connected = true;

			break;
		case MQTT_EVT_DISCONNECT:
			connected = false;
			clear_fds();

			break;
		case MQTT_EVT_PUBLISH:{
			uint8_t payload_buffer[128];  // Adjust size as needed
			int err = mqtt_read_publish_payload(client, payload_buffer, evt->param.publish.message.payload.len);
			if (err < 0) {
				LOG_INF("mqtt_read_publish_payload failed: %d", err);
			}

			cJSON *jsonfile = cJSON_Parse((char *)payload_buffer);
			if (jsonfile == NULL) {
				LOG_ERR("JSON parse failed");
				break;
			}

			cJSON *value = cJSON_GetObjectItem(jsonfile, "number");
			if (cJSON_IsNumber(value)){
				LOG_INF("Number: %d", value->valueint);
			}
			cJSON *value2 = cJSON_GetObjectItem(jsonfile, "key");
			if (cJSON_IsString(value2)){
				LOG_INF("Key: %s", value2->valuestring);
			}
			break;
		}
		case MQTT_EVT_SUBACK:	
			LOG_INF("SUCCESSFULLY SUBSCRIBED TO TOPIC");
			break;
		case MQTT_EVT_PUBACK:
			LOG_INF("Published packet");
			break;

		case MQTT_EVT_PINGRESP:
			LOG_INF("GET PINGRESP");
			break;

		default:
			break;
	}
}

static int client_init(struct mqtt_client *client)
{	
	int err = 0;
	struct addrinfo *result;
	struct addrinfo *addr;
	static struct mqtt_utf8 pass, name, client_id;
	struct addrinfo hints = {
		.ai_family = AF_INET,
		.ai_socktype = SOCK_STREAM
	};

	mqtt_client_init(client);

	err = getaddrinfo(CONFIG_MQTT_BROKER_HOSTNAME, NULL, &hints, &result);
	if (err) {
		LOG_ERR("getaddrinfo failed: %d", err);
		return -ECHILD;
	}

	addr = result;

	/* Look for address of the broker. */
	while (addr != NULL) {
		/* IPv4 Address. */
		if (addr->ai_addrlen == sizeof(struct sockaddr_in)) {
			struct sockaddr_in *broker4 =
				((struct sockaddr_in *)&broker);
			char ipv4_addr[NET_IPV4_ADDR_LEN];

			broker4->sin_addr.s_addr =
				((struct sockaddr_in *)addr->ai_addr)
				->sin_addr.s_addr;
			broker4->sin_family = AF_INET;
			broker4->sin_port = htons(CONFIG_MQTT_PORT);

			inet_ntop(AF_INET, &broker4->sin_addr.s_addr,
				  ipv4_addr, sizeof(ipv4_addr));
			//LOG_INF("IPv4 Address found %s", ipv4_addr);

			break;
		} else {
			LOG_INF("ai_addrlen = %u should be %u or %u",
				(unsigned int)addr->ai_addrlen,
				(unsigned int)sizeof(struct sockaddr_in),
				(unsigned int)sizeof(struct sockaddr_in6));
		}

		addr = addr->ai_next;
	}

	/* Free the address. */
	freeaddrinfo(result);

	name.size = strlen(CONFIG_MQTT_USERNAME);
	name.utf8 = (uint8_t *)CONFIG_MQTT_USERNAME;
	
	pass.size = strlen(CONFIG_MQTT_PASSWORD);
	pass.utf8 = (uint8_t *)CONFIG_MQTT_PASSWORD;
	client_id.size = strlen((char *)lte_params.device_id);
	client_id.utf8 = lte_params.device_id;

	/* MQTT client configuration */
	client->broker = &broker;
	client->evt_cb = mqtt_evt_handler;

	client->password = &pass;
	client->user_name = &name;
	client->client_id = client_id;
	client->protocol_version = MQTT_VERSION_3_1_1;

	/* MQTT buffers configuration */
	client->rx_buf = rx_buffer;
	client->rx_buf_size = sizeof(rx_buffer);
	client->tx_buf = tx_buffer;
	client->tx_buf_size = sizeof(tx_buffer);
	client->clean_session = 1;
	client->keepalive = CONFIG_MQTT_KEEPALIVE;


	struct mqtt_sec_config *tls_config = &client->transport.tls.config;
	static sec_tag_t sec_tag_list[] = { CONFIG_MQTT_TLS_SEC_TAG };


	//client->transport.type = MQTT_TRANSPORT_NON_SECURE;
	client->transport.type = MQTT_TRANSPORT_SECURE;
	tls_config->peer_verify = 2;
	tls_config->cipher_list = NULL;
	tls_config->sec_tag_count = ARRAY_SIZE(sec_tag_list);
	tls_config->sec_tag_list = sec_tag_list;
	tls_config->hostname = CONFIG_MQTT_BROKER_HOSTNAME;

	// tls_config->session_cache = IS_ENABLED(CONFIG_MQTT_TLS_SESSION_CACHING) ?
	// 				    TLS_SESSION_CACHE_ENABLED :
	// 				    TLS_SESSION_CACHE_DISABLED;

	// fds->fd = client->transport.tls.sock;

	// LOG_INF("Client ID: %s", client->client_id.utf8);
	// LOG_INF("Password: %s", client->password->utf8);
	// LOG_INF("Username: %s", client->user_name->utf8);
	return err;
}

static int try_to_connect(struct mqtt_client *client, struct lte_config *lte_params)
{
	int rc;
	int64_t start_time = k_uptime_get();
	rc = client_init(client);
	if (rc != 0) {
		LOG_ERR("client_init failed %d", rc);
		return rc;
	}

	int dummy_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	rc = mqtt_connect(client);
	if (rc != 0) {
		LOG_ERR("mqtt_connect failed %d", rc);
		return rc;
	}
	prepare_fds(client);


	if (wait(CONFIG_MQTT_CONNECT_TIMEOUT)) {
		mqtt_input(client);
	}

	if (!connected) {
		mqtt_abort(client);
	}

	if (connected) {
		lte_params->connection_time_protocol = k_uptime_get() - start_time;
		return 0;
	}

	return -EINVAL;
}


static void gnss_event_handler(int event)
{
	uint8_t tracked   = 0;
	uint8_t in_fix    = 0;
	uint8_t unhealthy = 0;

	struct nrf_modem_gnss_pvt_data_frame pvt_data;
	if (event == NRF_MODEM_GNSS_EVT_FIX){
		nrf_modem_gnss_read(&pvt_data, sizeof(pvt_data), NRF_MODEM_GNSS_DATA_PVT);
		LOG_INF("GNSS Fix: Latitude: %f, Longitude: %f, Accuracy: %f m",
			pvt_data.latitude, pvt_data.longitude, (double)pvt_data.accuracy);
		latitude = pvt_data.latitude;
		longitude = pvt_data.longitude;
		accuracy = (double)pvt_data.accuracy;
		gnss_ready = true;
	}

    else if (event == NRF_MODEM_GNSS_EVT_PVT ) {
        // Read GNSS PVT (Position, Velocity, Time) data
        if (nrf_modem_gnss_read(&pvt_data, sizeof(pvt_data), NRF_MODEM_GNSS_DATA_PVT) == 0) {
			for (int i = 0; i < NRF_MODEM_GNSS_MAX_SATELLITES; ++i) {
				if (pvt_data.sv[i].sv > 0) {
					tracked++;
		
					if (pvt_data.sv[i].flags & NRF_MODEM_GNSS_SV_FLAG_USED_IN_FIX) {
						in_fix++;
					}
		
					if (pvt_data.sv[i].flags & NRF_MODEM_GNSS_SV_FLAG_UNHEALTHY) {
						unhealthy++;
					}
				}
			}
			LOG_INF("Satalite data: Tracked: %d, In_fix: %d, Unhealthy: %d", tracked, in_fix, unhealthy);
        } 
		else {
            LOG_ERR("Failed to read GNSS PVT data");
        }
    }
}

static int open_supl_socket(void)
{
	int err;
	char port[6];
	struct addrinfo *info;

	struct addrinfo hints = {
		.ai_flags = AI_NUMERICSERV,
		.ai_family = AF_UNSPEC, /* Both IPv4 and IPv6 addresses accepted. */
		.ai_socktype = SOCK_STREAM
	};

	snprintf(port, sizeof(port), "%d", SUPL_SERVER_PORT);

	err = getaddrinfo(SUPL_SERVER, port, &hints, &info);
	if (err) {
		LOG_ERR("Failed to resolve hostname %s, error: %d", SUPL_SERVER, err);

		return -1;
	}

	/* Not connected. */
	err = -1;

	for (struct addrinfo *addr = info; addr != NULL; addr = addr->ai_next) {
		char ip[INET6_ADDRSTRLEN] = { 0 };
		struct sockaddr *const sa = addr->ai_addr;

		supl_fd = socket(sa->sa_family, SOCK_STREAM, IPPROTO_TCP);
		if (supl_fd < 0) {
			LOG_ERR("Failed to create socket, errno %d", errno);
			goto cleanup;
		}

		/* The SUPL library expects a 1 second timeout for the read function. */
		struct timeval timeout = {
			.tv_sec = 1,
			.tv_usec = 0,
		};

		err = setsockopt(supl_fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
		if (err) {
			LOG_ERR("Failed to set socket timeout, errno %d", errno);
			goto cleanup;
		}

		inet_ntop(sa->sa_family,
			  (void *)&((struct sockaddr_in *)sa)->sin_addr,
			  ip,
			  INET6_ADDRSTRLEN);
		LOG_INF("Connecting to %s port %d", ip, SUPL_SERVER_PORT);

		err = connect(supl_fd, sa, addr->ai_addrlen);
		if (err) {
			close(supl_fd);
			supl_fd = -1;

			/* Try the next address. */
			LOG_WRN("Connecting to server failed, errno %d", errno);
		} else {
			/* Connected. */
			break;
		}
	}

cleanup:
	freeaddrinfo(info);

	if (err) {
		/* Unable to connect, close socket. */
		LOG_ERR("Could not connect to SUPL server");
		if (supl_fd > -1) {
			close(supl_fd);
			supl_fd = -1;
		}
		return -1;
	}

	return 0;
}

static void close_supl_socket(void)
{	
	if (supl_fd >= 0) {
		zsock_shutdown(supl_fd, ZSOCK_SHUT_RDWR);
		k_sleep(K_MSEC(100));
	
		int err = zsock_close(supl_fd);
		if (err < 0) {
			LOG_WRN("SUPL socket close failed: %d / errno: %d (%s)", err, errno, strerror(errno));
		} else {
			LOG_INF("SUPL socket closed OK %d", supl_fd);
		}
		
		supl_fd = -1; // Invalidate
	} else {
		LOG_INF("SUPL fd already invalid, skipping close");
	}
}

static int assistance_request(struct nrf_modem_gnss_agnss_data_frame *agnss_request)
{
	int err;

	assistance_active = true;

	err = open_supl_socket();
	if (err) {
		goto exit;
	}

	LOG_INF("Starting SUPL session");
	err = supl_session(agnss_request);
	LOG_INF("Done");
	close_supl_socket();

exit:
	assistance_active = false;

	return err;
}

static ssize_t supl_read(void *p_buff, size_t nbytes, void *user_data)
{
	ARG_UNUSED(user_data);

	ssize_t rc = recv(supl_fd, p_buff, nbytes, 0);

	if (rc < 0 && (errno == EAGAIN)) {
		/* Return 0 to indicate a timeout. */
		rc = 0;
	} else if (rc == 0) {
		/* Peer closed the socket, return an error. */
		rc = -1;
	}

	return rc;
}

static ssize_t supl_write(const void *p_buff, size_t nbytes, void *user_data)
{
	ARG_UNUSED(user_data);

	return send(supl_fd, p_buff, nbytes, 0);
}

static int inject_agnss_type(void *agnss, size_t agnss_size, uint16_t type, void *user_data)
{
	ARG_UNUSED(user_data);

	int retval = nrf_modem_gnss_agnss_write(agnss, agnss_size, type);

	if (retval != 0) {
		LOG_ERR("Failed to write A-GNSS data, type: %d (errno: %d)", type, errno);
		return -1;
	}

	LOG_INF("Injected A-GNSS data, type: %d, size: %d", type, agnss_size);

	return 0;
}

static int supl_logger(int level, const char *fmt, ...)
{
	char buffer[256] = { 0 };
	va_list args;

	va_start(args, fmt);
	int ret = vsnprintk(buffer, sizeof(buffer), fmt, args);

	va_end(args);

	if (ret < 0) {
		LOG_ERR("%s: encoding error", __func__);
		return ret;
	} else if ((size_t)ret >= sizeof(buffer)) {
		LOG_ERR("%s: too long message,"
		       "it will be cut short", __func__);
	}

	LOG_INF("%s", buffer);

	return ret;
}

static int assistance_init(struct k_work_q *assistance_work_q)
{
	//ARG_UNUSED(assistance_work_q);

	static struct supl_api supl_api = {
		.read       = supl_read,
		.write      = supl_write,
		.handler    = inject_agnss_type,
		.logger     = supl_logger,
		.counter_ms = k_uptime_get
	};

	if (supl_init(&supl_api) != 0) {
		LOG_ERR("Failed to initialize SUPL library");
		return -1;
	}

	return 0;
}




// Function to generate random payload
void generate_random_payload(uint8_t *random_payload) {
    // If you have a hardware random number generator, use it instead
    // For demonstration, using a simple method
    for (int i = 0; i < 128; i++) {
        random_payload[i] = (uint8_t)(rand() % 256);
    }
}

static int publish_json(struct mqtt_client *client, struct lte_config *lte_params){
	struct mqtt_publish_param pub_param;
	struct mqtt_topic topic_list;
	int err;

	topic_list.topic.utf8 = (uint8_t*)"test";
	topic_list.topic.size = strlen("test");
	topic_list.qos = MQTT_QOS_0_AT_MOST_ONCE;

	pub_param.message.topic = topic_list;
	pub_param.message_id = 23;
	pub_param.dup_flag = 0U;
	pub_param.retain_flag = 0U;

	// uint32_t button_state = dk_get_buttons();
	// cJSON *json = cJSON_CreateObject();
	// cJSON *location = cJSON_CreateObject();

	// cJSON *json_empty = cJSON_CreateObject();
	// cJSON_AddStringToObject(json_empty, "empty", "packet");
	
	// cJSON_AddNumberToObject(location, "latitude", lte_params->latitude);
	// cJSON_AddNumberToObject(location, "longitude", lte_params->longitude);
	// cJSON_AddNumberToObject(location, "accuracy", lte_params->accuracy);
	// cJSON_AddItemToObject(json, "location", location);
	// cJSON_AddStringToObject(json, "apn", lte_params->apn);
	// cJSON_AddStringToObject(json, "tac", lte_params->tac);
	// cJSON_AddNumberToObject(json, "mcc", lte_params->mcc);
	// cJSON_AddNumberToObject(json, "mnc", lte_params->mnc);
	// cJSON_AddNumberToObject(json, "band", lte_params->band);
	// cJSON_AddNumberToObject(json, "earfcn", lte_params->earfcn);
	// cJSON_AddNumberToObject(json, "cell_id", lte_params->cell_id);
	// cJSON_AddNumberToObject(json, "phy_cell_id", lte_params->phy_cell_id);
	// cJSON_AddNumberToObject(json, "tx_power", lte_params->tx_power);
	// cJSON_AddNumberToObject(json, "rx_repetitions", lte_params->rx_repetitions);
	// cJSON_AddNumberToObject(json, "tx_repetitions", lte_params->tx_repetitions);
	// cJSON_AddNumberToObject(json, "energy_estimate", lte_params->energy_estimate);
	// cJSON_AddNumberToObject(json, "active_time", lte_params->active_time);
	// cJSON_AddNumberToObject(json, "ce_level", lte_params->ce_level);
	// cJSON_AddNumberToObject(json, "edrx", lte_params->edrx);
	// cJSON_AddNumberToObject(json, "rsrp", lte_params->rsrp);
	// cJSON_AddNumberToObject(json, "rsrq", lte_params->rsrq);
	// cJSON_AddNumberToObject(json, "snr", lte_params->snr);
	// cJSON_AddNumberToObject(json, "dl_pathloss", lte_params->dl_pathloss);
	// cJSON_AddNumberToObject(json, "tau", lte_params->tau);
	// cJSON_AddNumberToObject(json, "tau_triggered", lte_params->tau_trig);
	// cJSON_AddStringToObject(json, "client_id", lte_params->device_id);
	// cJSON_AddNumberToObject(json, "battery_voltage", lte_params->motherboard_voltage);
	// cJSON_AddNumberToObject(json, "link", lte_params->mode);
	// cJSON_AddStringToObject(json, "protocol", "MQTT");
	// cJSON_AddNumberToObject(json, "connection_time_link", lte_params->connection_time_link);
	// cJSON_AddNumberToObject(json, "connection_time_protocol", lte_params->connection_time_protocol);
	// cJSON_AddNumberToObject(json, "connection_time_gps", lte_params->connection_time_gps);
	// cJSON_AddNumberToObject(json, "throughput_link", lte_params->throughput_link);

	// // Convert the JSON object to a string
	// char *json_data = cJSON_PrintUnformatted(json);
	// char *json_empty_data = cJSON_PrintUnformatted(json_empty);
	// if (json_empty_data == NULL) {
	// 	LOG_ERR("Failed to create JSON empty string");
	// 	return EIO;
	// }
	// if (json_data == NULL) {
    //     LOG_ERR("Failed to create JSON string");
	// 	return EIO;
    // }

	// LOG_INF("Created Json");

	// pub_param.message.payload.data = json_empty_data;
	// pub_param.message.payload.len = strlen(json_empty_data);
	// err = mqtt_publish(client, &pub_param);
	// pub_param.message.payload.data = json_data;
	// pub_param.message.payload.len = strlen(json_data);
	// err = mqtt_publish(client, &pub_param);
	// cJSON_Delete(json);
	// cJSON_Delete(location);
	// cJSON_Delete(json_empty);
	uint8_t data[128];
	generate_random_payload(&data);

	pub_param.message.payload.data = data;
	pub_param.message.payload.len = 128;
	err = mqtt_publish(client, &pub_param);
	return err;
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

static int certificates_provision(void)
{
	int err = 0;
	err = modem_key_mgmt_write(CONFIG_MQTT_TLS_SEC_TAG,
		MODEM_KEY_MGMT_CRED_TYPE_CA_CHAIN,
		(void *)CA_CERTIFICATE,
		strlen(CA_CERTIFICATE));
	return err;
}

static int read_voltage(double *value){
	uint16_t buf;
	int err;
	struct adc_sequence sequence  = {
		.buffer = &buf,
		.buffer_size = sizeof(buf)
	};

	if (!device_is_ready(adc_channels[0].dev)) {
		printk("ADC controller device not ready\n");
		return -1;
	}

	err = adc_channel_setup_dt(&adc_channels[0]);
	if (err < 0) {
		printk("Could not setup channel #%d (%d)\n", 0, err);
		return err;
	}

	err = adc_sequence_init_dt(&adc_channels[0], &sequence);
	if (err < 0){
		LOG_ERR("Could not init sequence");
		return err;
	}

	k_sleep(K_SECONDS(1));
	int32_t val_mv = 0;
	for (int i = 0; i < 10; i ++){
		err = adc_read(adc_channels[0].dev, &sequence);
		if (err < 0) {
			LOG_ERR("Could not read channel");
			return err;
		}
		val_mv += buf;
		k_sleep(K_MSEC(100));
	}
	
	val_mv /= 10;
	err = adc_raw_to_millivolts_dt(&adc_channels[0], &val_mv);
	if (err < 0){
		LOG_ERR("Could not convert raw value to mV");
		return err;
	}

	*value = (((double)val_mv + 100.0) * 2.0);
	LOG_INF("Motherboard voltage: %f", *value);
	return 0;
}

static int get_device_uuid(char *device_id)  // Change to char array to hold string
{
    ssize_t length;
    uint8_t device_id_int[32];
    length = hwinfo_get_device_id(device_id_int, sizeof(device_id_int));
    if (length > 0) {
        // Ensure the string is null-terminated
        for (int i = 0; i < length; i++) {
            // Ensure there is enough space for each 2 chars + null terminator
            snprintf(device_id + i * 2, 3, "%02x", device_id_int[i]);  // 3 to include space for null-terminator
        }
        device_id[length * 2] = '\0';  // Null-terminate the string
        return 0;
    } else {
        LOG_ERR("Failed to retrieve device UUID");
        return -1;
    }
}

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

	err = getaddrinfo("customaddress", NULL, &hints, &result);
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

int main(void)
{	
	int err;

	LOG_INF("INITIALIZE DK");

	err = dk_leds_init();
	PRINT_RESULT("dk_leds_init", err);

	err = dk_buttons_init(NULL);
	PRINT_RESULT("dk_buttons_init", err);

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
		err = dk_set_led_on(i);
		k_sleep(K_MSEC(100));
		}
		for (int i = 3; i >= 0; i--) {
			err = dk_set_led_off(i);
			k_sleep(K_MSEC(100));
		}
	}
	

	// GET GPS
	// initialize the modem library
	LOG_INF("GET GPS DATA");
	err = nrf_modem_lib_init();
	PRINT_RESULT("nrf_modem_lib_init", err);

	err = switch_to_mode(LTE_LC_SYSTEM_MODE_LTEM_NBIOT_GPS, &lte_params);
	dk_set_led_on(DK_LED1);
	PRINT_RESULT("switch_to_mode", err);

	err = lte_lc_func_mode_set(LTE_LC_FUNC_MODE_ACTIVATE_GNSS);
    PRINT_RESULT("lte_lc_func_mode", err);

    err = nrf_modem_gnss_event_handler_set(gnss_event_handler);
	PRINT_RESULT("nrf_modem_gnss_event_handler_set", err);

	err = nrf_modem_gnss_fix_interval_set(0);
	PRINT_RESULT("nrf_modem_gnss_fix_interval_set", err);

	err = nrf_modem_gnss_fix_retry_set(0);
	PRINT_RESULT("nrf_modem_gnss_fix_retry_set", err);

	err = open_supl_socket();
	PRINT_RESULT("open_supl_socket", err);
	
	err = assistance_init(NULL);
	PRINT_RESULT("assistance_init", err);
	
    err = nrf_modem_gnss_start();
	PRINT_RESULT("nrf_modem_gnss_start", err);
	dk_set_led_on(DK_LED2);

	k_sleep(K_SECONDS(1));

	err = nrf_modem_gnss_read(&last_agnss, sizeof(last_agnss), NRF_MODEM_GNSS_DATA_AGNSS_REQ);
	PRINT_RESULT("nrf_modem_gnss_read", err);

	err = assistance_request(&last_agnss);
	PRINT_RESULT("assistance_request", err);

	err = lte_lc_func_mode_set(LTE_LC_FUNC_MODE_DEACTIVATE_LTE);
	PRINT_RESULT("lte_lc_func_mode", err);


	gnss_time = k_uptime_get();
    LOG_INF("GNSS started, waiting for fix...");
	dk_set_led_on(DK_LED3);
	while (1) {
        k_sleep(K_SECONDS(5));

		if (gnss_ready) {
			break;
		}
		if (k_uptime_get() - gnss_time > 600000) {
			LOG_ERR("GNSS fix not received, restarting...");
			// Reboot the system
			sys_reboot(SYS_REBOOT_COLD);
		}
    }
	
	dk_set_led_on(DK_LED4);
	LOG_INF("FOUND GNSS FIX: Latitude:%f, Longitude:%f, Accuracy:%f,", (double)latitude, (double)longitude, (double)accuracy);
	k_sleep(K_SECONDS(2));
	err = nrf_modem_gnss_stop();
	PRINT_RESULT("nrf_modem_gnss_stop", err);

	err = lte_lc_power_off();
	PRINT_RESULT("lte_lc_power_off", err);

	err = nrf_modem_lib_shutdown();
	PRINT_RESULT("nrf_modem_lib_shutdown", err);

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
	
	for (int i = 0; i < 2; i ++){
		for (int i = 0; i < 4; i++) {
		err = dk_set_led_on(i);
		k_sleep(K_MSEC(100));
		}
		for (int i = 3; i >= 0; i--) {
			err = dk_set_led_off(i);
			k_sleep(K_MSEC(100));
		}
	}

	// Lte-M
	LOG_INF("TRY TO CONNECT TO LTE-M");
	err = nrf_modem_lib_init();
	__ASSERT(err == 0, "Modem library could not be initialized, err %d.", err);

	err = certificates_provision();
	PRINT_RESULT("certificates_provision", err);

	err = switch_to_mode(LTE_LC_SYSTEM_MODE_LTEM, &lte_params);
	PRINT_RESULT("switch_to_mode", err);
	dk_set_led_on(DK_LED1);

	k_sleep(K_SECONDS(2));
	err = speedtest(&lte_params.throughput_link);
	PRINT_RESULT("socket_connect_and_send", err);

	err = try_to_connect(&client, &lte_params);
	PRINT_RESULT("try_to_connect", err);
	dk_set_led_on(DK_LED2);

	err = get_parameters(&lte_params, true);
	PRINT_RESULT("get_parameters", err);

	LOG_INF("Link throughput: %f", lte_params.throughput_link);

	err = publish_json(&client, &lte_params);
	PRINT_RESULT("mqtt_publish", err);

	err = mqtt_disconnect(&client);
	PRINT_RESULT("mqtt_disconnect", err);

	err = lte_lc_power_off();
	PRINT_RESULT("lte_lc_power_off", err);

	err = nrf_modem_lib_shutdown();
	PRINT_RESULT("nrf_modem_lib_shutdown", err);

	LOG_INF("CONNECTION_TIME_LINK_LTEM: %lld", lte_params.connection_time_link);
	LOG_INF("CONNECTION_TIME_PROTOCOL_LTEM: %lld", lte_params.connection_time_protocol);

	k_sleep(K_SECONDS(5));

	LOG_INF("TRY TO CONNECT TO NBIOT");
	err = nrf_modem_lib_init();
	PRINT_RESULT("nrf_modem_lib_init", err);

	err = certificates_provision();
	PRINT_RESULT("certificates_provision", err);

	err = switch_to_mode(LTE_LC_SYSTEM_MODE_NBIOT, &lte_params);
	dk_set_led_on(DK_LED3);
	PRINT_RESULT("switch_to_mode", err);

	k_sleep(K_SECONDS(2));
	err = speedtest(&lte_params.throughput_link);
	PRINT_RESULT("socket_connect_and_send", err);

	err = try_to_connect(&client, &lte_params);
	PRINT_RESULT("try_to_connect", err);
	dk_set_led_on(DK_LED4);

	err = get_parameters(&lte_params, true);
	PRINT_RESULT("get_parameters", err);
	LOG_INF("Link throughput: %f", lte_params.throughput_link);

	err = publish_json(&client, &lte_params);
	PRINT_RESULT("mqtt_publish", err);

	err = mqtt_disconnect(&client);
	PRINT_RESULT("mqtt_disconnect", err);

	err = lte_lc_power_off();
	PRINT_RESULT("lte_lc_power_off", err);

	err = nrf_modem_lib_shutdown();
	PRINT_RESULT("nrf_modem_lib_shutdown", err);

	LOG_INF("CONNECTION_TIME_LINK_NBIOT: %lld", lte_params.connection_time_link);
	LOG_INF("CONNECTION_TIME_PROTOCOL_NBIOT: %lld", lte_params.connection_time_protocol);


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
