################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
C:\Users\39339\Documents\Arduino\libraries\average-value-master\src\AverageValue.cpp 

LINK_OBJ += \
.\libraries\average-value-master\src\AverageValue.cpp.o 

CPP_DEPS += \
.\libraries\average-value-master\src\AverageValue.cpp.d 


# Each subdirectory must supply rules for building sources it contributes
libraries\average-value-master\src\AverageValue.cpp.o: C:\Users\39339\Documents\Arduino\libraries\average-value-master\src\AverageValue.cpp
	@echo 'Building file: $<'
	@echo 'Avvio compilazione C++'
	"C:\Sloeber440\arduinoPlugin\packages\esp32\tools\xtensa-esp32s3-elf-gcc\esp-2021r2-patch5-8.4.0/bin/xtensa-esp32s3-elf-g++" -DHAVE_CONFIG_H "-DMBEDTLS_CONFIG_FILE=\"mbedtls/esp_config.h\"" -DUNITY_INCLUDE_CONFIG_H -DWITH_POSIX -D_GNU_SOURCE -DIDF_VER="v4.4.5" -DESP_PLATFORM -D_POSIX_READER_WRITER_LOCKS   "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/newlib/platform_include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/freertos/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/freertos/include/esp_additions/freertos" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/freertos/port/xtensa/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/freertos/include/esp_additions" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/esp_hw_support/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/esp_hw_support/include/soc" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/esp_hw_support/include/soc/esp32s3" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/esp_hw_support/port/esp32s3" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/esp_hw_support/port/esp32s3/private_include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/heap/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/log/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/lwip/include/apps" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/lwip/include/apps/sntp" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/lwip/lwip/src/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/lwip/port/esp32/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/lwip/port/esp32/include/arch" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/soc/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/soc/esp32s3" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/soc/esp32s3/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/hal/esp32s3/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/hal/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/hal/platform_port/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/esp_rom/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/esp_rom/include/esp32s3" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/esp_rom/esp32s3" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/esp_common/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/esp_system/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/esp_system/port/soc" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/esp_system/port/public_compat" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/xtensa/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/xtensa/esp32s3/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/driver/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/driver/esp32s3/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/esp_pm/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/esp_ringbuf/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/efuse/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/efuse/esp32s3/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/vfs/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/esp_wifi/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/esp_event/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/esp_netif/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/esp_eth/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/tcpip_adapter/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/esp_phy/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/esp_phy/esp32s3/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/esp_ipc/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/app_trace/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/esp_timer/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/mbedtls/port/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/mbedtls/mbedtls/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/mbedtls/esp_crt_bundle/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/app_update/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/spi_flash/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/bootloader_support/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/nvs_flash/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/pthread/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/esp_gdbstub/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/esp_gdbstub/xtensa" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/esp_gdbstub/esp32s3" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/espcoredump/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/espcoredump/include/port/xtensa" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/wpa_supplicant/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/wpa_supplicant/port/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/wpa_supplicant/esp_supplicant/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/ieee802154/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/console" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/asio/asio/asio/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/asio/port/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/bt/common/osi/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/bt/include/esp32c3/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/bt/common/api/include/api" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/bt/common/btc/profile/esp/blufi/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/bt/common/btc/profile/esp/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/bt/host/bluedroid/api/include/api" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/bt/esp_ble_mesh/mesh_common/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/bt/esp_ble_mesh/mesh_common/tinycrypt/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/bt/esp_ble_mesh/mesh_core" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/bt/esp_ble_mesh/mesh_core/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/bt/esp_ble_mesh/mesh_core/storage" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/bt/esp_ble_mesh/btc/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/bt/esp_ble_mesh/mesh_models/common/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/bt/esp_ble_mesh/mesh_models/client/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/bt/esp_ble_mesh/mesh_models/server/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/bt/esp_ble_mesh/api/core/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/bt/esp_ble_mesh/api/models/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/bt/esp_ble_mesh/api" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/cbor/port/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/unity/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/unity/unity/src" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/cmock/CMock/src" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/coap/port/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/coap/libcoap/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/nghttp/port/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/nghttp/nghttp2/lib/includes" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/esp-tls" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/esp-tls/esp-tls-crypto" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/esp_adc_cal/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/esp_hid/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/tcp_transport/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/esp_http_client/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/esp_http_server/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/esp_https_ota/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/esp_https_server/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/esp_lcd/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/esp_lcd/interface" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/protobuf-c/protobuf-c" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/protocomm/include/common" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/protocomm/include/security" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/protocomm/include/transports" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/mdns/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/esp_local_ctrl/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/sdmmc/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/esp_serial_slave_link/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/esp_websocket_client/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/expat/expat/expat/lib" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/expat/port/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/wear_levelling/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/fatfs/diskio" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/fatfs/vfs" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/fatfs/src" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/freemodbus/freemodbus/common/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/idf_test/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/idf_test/include/esp32s3" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/jsmn/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/json/cJSON" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/libsodium/libsodium/src/libsodium/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/libsodium/port_include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/mqtt/esp-mqtt/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/openssl/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/perfmon/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/spiffs/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/usb/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/ulp/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/wifi_provisioning/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/rmaker_common/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/esp_diagnostics/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/rtc_store/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/esp_insights/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/json_parser/upstream/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/json_parser/upstream" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/json_generator/upstream" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/esp_schedule/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/espressif__esp_secure_cert_mgr/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/esp_rainmaker/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/gpio_button/button/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/qrcode/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/ws2812_led" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/freertos/include/freertos" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/arduino_tinyusb/tinyusb/src" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/arduino_tinyusb/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/esp_littlefs/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/esp-dl/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/esp-dl/include/tool" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/esp-dl/include/typedef" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/esp-dl/include/image" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/esp-dl/include/math" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/esp-dl/include/nn" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/esp-dl/include/layer" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/esp-dl/include/detect" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/esp-dl/include/model_zoo" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/esp32-camera/driver/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/esp32-camera/conversions/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/espressif__esp-dsp/modules/dotprod/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/espressif__esp-dsp/modules/support/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/espressif__esp-dsp/modules/support/mem/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/espressif__esp-dsp/modules/windows/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/espressif__esp-dsp/modules/windows/hann/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/espressif__esp-dsp/modules/windows/blackman/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/espressif__esp-dsp/modules/windows/blackman_harris/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/espressif__esp-dsp/modules/windows/blackman_nuttall/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/espressif__esp-dsp/modules/windows/nuttall/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/espressif__esp-dsp/modules/windows/flat_top/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/espressif__esp-dsp/modules/iir/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/espressif__esp-dsp/modules/fir/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/espressif__esp-dsp/modules/math/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/espressif__esp-dsp/modules/math/add/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/espressif__esp-dsp/modules/math/sub/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/espressif__esp-dsp/modules/math/mul/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/espressif__esp-dsp/modules/math/addc/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/espressif__esp-dsp/modules/math/mulc/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/espressif__esp-dsp/modules/math/sqrt/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/espressif__esp-dsp/modules/matrix/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/espressif__esp-dsp/modules/fft/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/espressif__esp-dsp/modules/dct/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/espressif__esp-dsp/modules/conv/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/espressif__esp-dsp/modules/common/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/espressif__esp-dsp/modules/kalman/ekf/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/espressif__esp-dsp/modules/kalman/ekf_imu13states/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/include/fb_gfx/include" "-IC:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10/tools/sdk/esp32s3/dio_qspi/include" -mlongcalls -ffunction-sections -fdata-sections -Wno-error=unused-function -Wno-error=unused-variable -Wno-error=deprecated-declarations -Wno-unused-parameter -Wno-sign-compare -ggdb -freorder-blocks -Wwrite-strings -fstack-protector -fstrict-volatile-bitfields -Wno-error=unused-but-set-variable -fno-jump-tables -fno-tree-switch-conversion -std=gnu++11 -fexceptions -fno-rtti  -MMD -c -w -Os -DF_CPU=80000000L -DARDUINO=10812 -DARDUINO_ESP32S3_DEV -DARDUINO_ARCH_ESP32 "-DARDUINO_BOARD=\"ESP32S3_DEV\"" -DARDUINO_VARIANT="esp32s3" -DARDUINO_PARTITION_noota_ffat  -DESP32 -DCORE_DEBUG_LEVEL=0 -DARDUINO_RUNNING_CORE=0 -DARDUINO_EVENT_RUNNING_CORE=1  -DARDUINO_USB_MODE=1 -DARDUINO_USB_CDC_ON_BOOT=1 -DARDUINO_USB_MSC_ON_BOOT=0 -DARDUINO_USB_DFU_ON_BOOT=0 "@C:\Users\39339\Documents\sloeber-workspace\ESP32_s3\Release/build_opt.h" "@C:\Users\39339\Documents\sloeber-workspace\ESP32_s3\Release/file_opts"   -I"C:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10\cores\esp32" -I"C:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.10\variants\esp32s3" -I"C:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.14\libraries\SD_MMC\src" -I"C:\Sloeber440\arduinoPlugin\libraries\arduino-timer\2.3.1\src" -I"C:\Users\39339\Documents\Arduino\libraries\average-value-master\src" -I"C:\Sloeber440\arduinoPlugin\libraries\MPU6050\1.3.1\src" -I"C:\Users\39339\Documents\Arduino\libraries\PWMOutESP32-1.0.2" -I"C:\Users\39339\Documents\Arduino\libraries\PlotPlus-master" -I"C:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.14\libraries\FS\src" -I"C:\Sloeber440\arduinoPlugin\packages\esp32\hardware\esp32\2.0.14\libraries\Wire\src" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -D__IN_ECLIPSE__=1 -x c++ "$<"   -o "$@"
	@echo 'Finished building: $<'
	@echo ' '

