#include "app_ethernet.h"

#include <assert.h>
#include <string.h>

#include "driver/gpio.h"
#include "esp_event.h"
#include "esp_eth.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "sdkconfig.h"

#if CONFIG_EXAMPLE_USE_SPI_ETHERNET
#include "driver/spi_master.h"
#endif

#define APP_ETH_NETIF_DESC "example_netif_eth"
#define APP_ETH_LINK_TIMEOUT_MS 10000
#define APP_ETH_DHCP_TIMEOUT_MS 8000

#define APP_ETH_STATIC_IP_A 192
#define APP_ETH_STATIC_IP_B 168
#define APP_ETH_STATIC_IP_C 137
#define APP_ETH_STATIC_IP_D 2
#define APP_ETH_STATIC_GW_D 1

static const char *TAG = "app_ethernet";

static SemaphoreHandle_t s_link_sem;
static SemaphoreHandle_t s_ipv4_sem;
static esp_eth_handle_t s_eth_handle;
static esp_eth_mac_t *s_mac;
static esp_eth_phy_t *s_phy;
static esp_eth_netif_glue_handle_t s_eth_glue;

static bool is_app_eth_netif(esp_netif_t *netif)
{
    const char *desc = esp_netif_get_desc(netif);
    return desc && strncmp(desc, APP_ETH_NETIF_DESC, strlen(APP_ETH_NETIF_DESC)) == 0;
}

static void on_eth_got_ip(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    (void)arg;
    (void)event_base;
    (void)event_id;

    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
    if (!is_app_eth_netif(event->esp_netif)) {
        return;
    }

    ESP_LOGI(TAG,
             "Got IPv4 event: Interface \"%s\" address: " IPSTR,
             esp_netif_get_desc(event->esp_netif),
             IP2STR(&event->ip_info.ip));

    if (s_ipv4_sem) {
        xSemaphoreGive(s_ipv4_sem);
    }
}

static void on_eth_event(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    (void)event_base;
    (void)event_data;

    esp_netif_t *netif = (esp_netif_t *)arg;

    switch (event_id) {
    case ETHERNET_EVENT_CONNECTED:
        ESP_LOGI(TAG, "Ethernet Link Up");
#if CONFIG_EXAMPLE_CONNECT_IPV6
        ESP_ERROR_CHECK(esp_netif_create_ip6_linklocal(netif));
#endif
        if (s_link_sem) {
            xSemaphoreGive(s_link_sem);
        }
        break;
    case ETHERNET_EVENT_DISCONNECTED:
        ESP_LOGW(TAG, "Ethernet Link Down");
        break;
    default:
        break;
    }
}

#if CONFIG_EXAMPLE_CONNECT_IPV6
static void on_eth_got_ipv6(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    (void)arg;
    (void)event_base;
    (void)event_id;

    ip_event_got_ip6_t *event = (ip_event_got_ip6_t *)event_data;
    if (!is_app_eth_netif(event->esp_netif)) {
        return;
    }

    ESP_LOGI(TAG,
             "Got IPv6 event: Interface \"%s\" address: " IPV6STR,
             esp_netif_get_desc(event->esp_netif),
             IPV62STR(event->ip6_info.ip));
}
#endif

static esp_netif_t *eth_start(void)
{
    esp_netif_inherent_config_t esp_netif_config = ESP_NETIF_INHERENT_DEFAULT_ETH();
    esp_netif_config.if_desc = APP_ETH_NETIF_DESC;
    esp_netif_config.route_prio = 64;

    esp_netif_config_t netif_config = {
        .base = &esp_netif_config,
        .stack = ESP_NETIF_NETSTACK_DEFAULT_ETH,
    };
    esp_netif_t *netif = esp_netif_new(&netif_config);
    assert(netif);

    eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
    mac_config.rx_task_stack_size = CONFIG_EXAMPLE_ETHERNET_EMAC_TASK_STACK_SIZE;

    eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
    phy_config.phy_addr = CONFIG_EXAMPLE_ETH_PHY_ADDR;
    phy_config.reset_gpio_num = CONFIG_EXAMPLE_ETH_PHY_RST_GPIO;

#if CONFIG_EXAMPLE_USE_INTERNAL_ETHERNET
    eth_esp32_emac_config_t esp32_emac_config = ETH_ESP32_EMAC_DEFAULT_CONFIG();
    esp32_emac_config.smi_gpio.mdc_num = CONFIG_EXAMPLE_ETH_MDC_GPIO;
    esp32_emac_config.smi_gpio.mdio_num = CONFIG_EXAMPLE_ETH_MDIO_GPIO;
    s_mac = esp_eth_mac_new_esp32(&esp32_emac_config, &mac_config);
#if CONFIG_EXAMPLE_ETH_PHY_GENERIC
    s_phy = esp_eth_phy_new_generic(&phy_config);
#elif CONFIG_EXAMPLE_ETH_PHY_IP101
    s_phy = esp_eth_phy_new_ip101(&phy_config);
#elif CONFIG_EXAMPLE_ETH_PHY_RTL8201
    s_phy = esp_eth_phy_new_rtl8201(&phy_config);
#elif CONFIG_EXAMPLE_ETH_PHY_LAN87XX
    s_phy = esp_eth_phy_new_lan87xx(&phy_config);
#elif CONFIG_EXAMPLE_ETH_PHY_DP83848
    s_phy = esp_eth_phy_new_dp83848(&phy_config);
#elif CONFIG_EXAMPLE_ETH_PHY_KSZ80XX
    s_phy = esp_eth_phy_new_ksz80xx(&phy_config);
#endif
#elif CONFIG_EXAMPLE_USE_SPI_ETHERNET
    gpio_install_isr_service(0);
    spi_bus_config_t buscfg = {
        .miso_io_num = CONFIG_EXAMPLE_ETH_SPI_MISO_GPIO,
        .mosi_io_num = CONFIG_EXAMPLE_ETH_SPI_MOSI_GPIO,
        .sclk_io_num = CONFIG_EXAMPLE_ETH_SPI_SCLK_GPIO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(CONFIG_EXAMPLE_ETH_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO));

    spi_device_interface_config_t spi_devcfg = {
        .mode = 0,
        .clock_speed_hz = CONFIG_EXAMPLE_ETH_SPI_CLOCK_MHZ * 1000 * 1000,
        .spics_io_num = CONFIG_EXAMPLE_ETH_SPI_CS_GPIO,
        .queue_size = 20,
    };
#if CONFIG_EXAMPLE_USE_DM9051
    eth_dm9051_config_t dm9051_config = ETH_DM9051_DEFAULT_CONFIG(CONFIG_EXAMPLE_ETH_SPI_HOST, &spi_devcfg);
    dm9051_config.int_gpio_num = CONFIG_EXAMPLE_ETH_SPI_INT_GPIO;
    s_mac = esp_eth_mac_new_dm9051(&dm9051_config, &mac_config);
    s_phy = esp_eth_phy_new_dm9051(&phy_config);
#elif CONFIG_EXAMPLE_USE_W5500
    eth_w5500_config_t w5500_config = ETH_W5500_DEFAULT_CONFIG(CONFIG_EXAMPLE_ETH_SPI_HOST, &spi_devcfg);
    w5500_config.int_gpio_num = CONFIG_EXAMPLE_ETH_SPI_INT_GPIO;
    s_mac = esp_eth_mac_new_w5500(&w5500_config, &mac_config);
    s_phy = esp_eth_phy_new_w5500(&phy_config);
#endif
#elif CONFIG_EXAMPLE_USE_OPENETH
    phy_config.autonego_timeout_ms = 100;
    s_mac = esp_eth_mac_new_openeth(&mac_config);
    s_phy = esp_eth_phy_new_dp83848(&phy_config);
#endif

    esp_eth_config_t config = ETH_DEFAULT_CONFIG(s_mac, s_phy);
    ESP_ERROR_CHECK(esp_eth_driver_install(&config, &s_eth_handle));

#if CONFIG_EXAMPLE_USE_SPI_ETHERNET
    uint8_t eth_mac[6] = {0};
    ESP_ERROR_CHECK(esp_read_mac(eth_mac, ESP_MAC_ETH));
    ESP_ERROR_CHECK(esp_eth_ioctl(s_eth_handle, ETH_CMD_S_MAC_ADDR, eth_mac));
#endif

    s_eth_glue = esp_eth_new_netif_glue(s_eth_handle);
    ESP_ERROR_CHECK(esp_netif_attach(netif, s_eth_glue));

    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &on_eth_got_ip, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, &on_eth_event, netif));
#if CONFIG_EXAMPLE_CONNECT_IPV6
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_GOT_IP6, &on_eth_got_ipv6, NULL));
#endif

    ESP_ERROR_CHECK(esp_eth_start(s_eth_handle));
    return netif;
}

static esp_err_t set_static_ipv4(esp_netif_t *netif)
{
    esp_err_t ret = esp_netif_dhcpc_stop(netif);
    if (ret != ESP_OK && ret != ESP_ERR_ESP_NETIF_DHCP_ALREADY_STOPPED) {
        return ret;
    }

    esp_netif_ip_info_t ip_info = {0};
    esp_netif_set_ip4_addr(&ip_info.ip,
                           APP_ETH_STATIC_IP_A,
                           APP_ETH_STATIC_IP_B,
                           APP_ETH_STATIC_IP_C,
                           APP_ETH_STATIC_IP_D);
    esp_netif_set_ip4_addr(&ip_info.netmask, 255, 255, 255, 0);
    esp_netif_set_ip4_addr(&ip_info.gw,
                           APP_ETH_STATIC_IP_A,
                           APP_ETH_STATIC_IP_B,
                           APP_ETH_STATIC_IP_C,
                           APP_ETH_STATIC_GW_D);

    ret = esp_netif_set_ip_info(netif, &ip_info);
    if (ret == ESP_OK) {
        ESP_LOGW(TAG,
                 "DHCP timeout, using static IPv4: " IPSTR " gateway: " IPSTR,
                 IP2STR(&ip_info.ip),
                 IP2STR(&ip_info.gw));
    }
    return ret;
}

esp_netif_t *app_ethernet_connect(void)
{
    s_link_sem = xSemaphoreCreateBinary();
    s_ipv4_sem = xSemaphoreCreateBinary();
    if (!s_link_sem || !s_ipv4_sem) {
        ESP_LOGE(TAG, "failed to create ethernet semaphores");
        return NULL;
    }

    esp_netif_t *netif = eth_start();

    ESP_LOGI(TAG, "Waiting for Ethernet link.");
    if (xSemaphoreTake(s_link_sem, pdMS_TO_TICKS(APP_ETH_LINK_TIMEOUT_MS)) != pdTRUE) {
        ESP_LOGE(TAG, "Ethernet link timeout");
        return NULL;
    }

    ESP_LOGI(TAG, "Waiting for DHCP IPv4.");
    if (xSemaphoreTake(s_ipv4_sem, pdMS_TO_TICKS(APP_ETH_DHCP_TIMEOUT_MS)) != pdTRUE) {
        ESP_ERROR_CHECK(set_static_ipv4(netif));
    }

    return netif;
}
