################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ..\makefile.init

RM := rm -f

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir.mk
-include libraries\Wire\src\subdir.mk
-include libraries\SD_MMC\src\subdir.mk
-include libraries\PlotPlus\subdir.mk
-include libraries\MPU6050\src\subdir.mk
-include libraries\FS\src\subdir.mk
-include libraries\AverageValue\src\subdir.mk
-include core\core\libb64\subdir.mk
-include core\core\subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(CCM_DEPS)),)
-include $(CCM_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(PDE_DEPS)),)
-include $(PDE_DEPS)
endif
ifneq ($(strip $(CXXM_DEPS)),)
-include $(CXXM_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(INO_DEPS)),)
-include $(INO_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C++M_DEPS)),)
-include $(C++M_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
endif

-include ..\makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
AR += \
C:\Users\Utente\git\ESP32_S3_lightsaber\Release\arduino.ar \

ELF += \
ESP32_s3.elf \

HEX += \
ESP32_s3.hex \



#bootloaderTest
BurnBootLoader: 
	@echo trying to burn bootloader ${bootloader.tool}
	${tools.${bootloader.tool}.erase.pattern}
	${tools.${bootloader.tool}.bootloader.pattern}

uploadWithBuild: all
	@echo trying to build and upload with upload tool ${upload.tool}
	${tools.${upload.tool}.upload.pattern}

uploadWithoutBuild: 
	@echo trying to upload without build with upload tool ${upload.tool}
	${tools.${upload.tool}.upload.pattern}
    
uploadWithProgrammerWithBuild: all
	@echo trying to build and upload with programmer ${program.tool}
	${tools.${program.tool}.program.pattern}

uploadWithProgrammerWithoutBuild: 
	@echo trying to upload with programmer ${program.tool} without build
	${tools.${program.tool}.program.pattern}

# All Target
all:
	-${recipe.hooks.prebuild.1.pattern}
	${recipe.hooks.prebuild.2.pattern}
	${recipe.hooks.prebuild.3.pattern}
	${recipe.hooks.prebuild.4.pattern}
	${recipe.hooks.prebuild.5.pattern}
	${recipe.hooks.prebuild.6.pattern} ||:
	$(MAKE) --no-print-directory main-build

# Main-build Target
main-build: ESP32_s3

# Tool invocations
ESP32_s3: $(HEX)
	@echo 'Building target: $@'
	@echo 'Report dimensioni:'
	"C:\Sloeber\arduinoPlugin\packages\esp32\tools\xtensa-esp32s3-elf-gcc\esp-2021r2-patch5-8.4.0/bin/xtensa-esp32s3-elf-size" -A "C:\Users\Utente\git\ESP32_S3_lightsaber\Release/ESP32_s3.elf"
	@echo 'Finished building target: $@'
	@echo ' '

C:\Users\Utente\git\ESP32_S3_lightsaber\Release\arduino.ar: $(AR_OBJ)
	@echo 'Avvio archiviazione'
	"C:\Sloeber\arduinoPlugin\packages\esp32\tools\xtensa-esp32s3-elf-gcc\esp-2021r2-patch5-8.4.0/bin/xtensa-esp32s3-elf-ar" cr  "C:\Users\Utente\git\ESP32_S3_lightsaber\Release\arduino.ar" $(AR_OBJ)
	@echo 'Finished building: $@'
	@echo ' '

ESP32_s3.elf: $(LINK_OBJ) $(AR)
	@echo 'Avvio del combiner'
	"C:\Sloeber\arduinoPlugin\packages\esp32\tools\xtensa-esp32s3-elf-gcc\esp-2021r2-patch5-8.4.0/bin/xtensa-esp32s3-elf-g++" "-Wl,--Map=C:\Users\Utente\git\ESP32_S3_lightsaber\Release/ESP32_s3.map" "-LC:\Sloeber\arduinoPlugin\packages\esp32\hardware\esp32\2.0.14/tools/sdk/esp32s3/lib" "-LC:\Sloeber\arduinoPlugin\packages\esp32\hardware\esp32\2.0.14/tools/sdk/esp32s3/ld" "-LC:\Sloeber\arduinoPlugin\packages\esp32\hardware\esp32\2.0.14/tools/sdk/esp32s3/dio_qspi" -T memory.ld -T sections.ld -T esp32s3.rom.ld -T esp32s3.rom.api.ld -T esp32s3.rom.libgcc.ld -T esp32s3.rom.newlib.ld -T esp32s3.rom.version.ld -T esp32s3.rom.newlib-time.ld -T esp32s3.peripherals.ld  -mlongcalls -Wl,--cref -Wl,--gc-sections -fno-rtti -fno-lto -Wl,--wrap=esp_log_write -Wl,--wrap=esp_log_writev -Wl,--wrap=log_printf -u _Z5setupv -u _Z4loopv -u esp_app_desc -u pthread_include_pthread_impl -u pthread_include_pthread_cond_impl -u pthread_include_pthread_local_storage_impl -u pthread_include_pthread_rwlock_impl -u include_esp_phy_override -u ld_include_highint_hdl -u start_app -u start_app_other_cores -u __ubsan_include -Wl,--wrap=longjmp -u __assert_func -u vfs_include_syscalls_impl -Wl,--undefined=uxTopUsedPriority -u app_main -u newlib_include_heap_impl -u newlib_include_syscalls_impl -u newlib_include_pthread_impl -u newlib_include_assert_impl -u __cxa_guard_dummy  -DESP32 -DCORE_DEBUG_LEVEL=0 -DARDUINO_RUNNING_CORE=0 -DARDUINO_EVENT_RUNNING_CORE=1  -DARDUINO_USB_MODE=1 -DARDUINO_USB_CDC_ON_BOOT=1 -DARDUINO_USB_MSC_ON_BOOT=0 -DARDUINO_USB_DFU_ON_BOOT=0 -Wl,--start-group  $(LINK_OBJ) $(AR)  -lesp_ringbuf -lefuse -lesp_ipc -ldriver -lesp_pm -lmbedtls -lapp_update -lbootloader_support -lspi_flash -lnvs_flash -lpthread -lesp_gdbstub -lespcoredump -lesp_phy -lesp_system -lesp_rom -lhal -lvfs -lesp_eth -ltcpip_adapter -lesp_netif -lesp_event -lwpa_supplicant -lesp_wifi -lconsole -llwip -llog -lheap -lsoc -lesp_hw_support -lxtensa -lesp_common -lesp_timer -lfreertos -lnewlib -lcxx -lapp_trace -lasio -lbt -lcbor -lunity -lcmock -lcoap -lnghttp -lesp-tls -lesp_adc_cal -lesp_hid -ltcp_transport -lesp_http_client -lesp_http_server -lesp_https_ota -lesp_https_server -lesp_lcd -lprotobuf-c -lprotocomm -lmdns -lesp_local_ctrl -lsdmmc -lesp_serial_slave_link -lesp_websocket_client -lexpat -lwear_levelling -lfatfs -lfreemodbus -ljsmn -ljson -llibsodium -lmqtt -lopenssl -lperfmon -lspiffs -lusb -lulp -lwifi_provisioning -lrmaker_common -lesp_diagnostics -lrtc_store -lesp_insights -ljson_parser -ljson_generator -lesp_schedule -lespressif__esp_secure_cert_mgr -lesp_rainmaker -lgpio_button -lqrcode -lws2812_led -lesp32-camera -lesp_littlefs -lespressif__esp-dsp -lfb_gfx -lasio -lcmock -lunity -lcoap -lesp_lcd -lesp_websocket_client -lexpat -lfreemodbus -ljsmn -llibsodium -lperfmon -lusb -lesp_adc_cal -lesp_hid -lfatfs -lwear_levelling -lopenssl -lspiffs -lesp_insights -lcbor -lesp_diagnostics -lrtc_store -lesp_rainmaker -lesp_local_ctrl -lesp_https_server -lwifi_provisioning -lprotocomm -lbt -lbtdm_app -lprotobuf-c -lmdns -ljson -ljson_parser -ljson_generator -lesp_schedule -lespressif__esp_secure_cert_mgr -lqrcode -lrmaker_common -lmqtt -larduino_tinyusb -lcat_face_detect -lhuman_face_detect -lcolor_detect -lmfn -ldl -lesp_ringbuf -lefuse -lesp_ipc -ldriver -lesp_pm -lmbedtls -lapp_update -lbootloader_support -lspi_flash -lnvs_flash -lpthread -lesp_gdbstub -lespcoredump -lesp_phy -lesp_system -lesp_rom -lhal -lvfs -lesp_eth -ltcpip_adapter -lesp_netif -lesp_event -lwpa_supplicant -lesp_wifi -lconsole -llwip -llog -lheap -lsoc -lesp_hw_support -lxtensa -lesp_common -lesp_timer -lfreertos -lnewlib -lcxx -lapp_trace -lnghttp -lesp-tls -ltcp_transport -lesp_http_client -lesp_http_server -lesp_https_ota -lsdmmc -lesp_serial_slave_link -lulp -lmbedtls_2 -lmbedcrypto -lmbedx509 -lcoexist -lcore -lespnow -lmesh -lnet80211 -lpp -lsmartconfig -lwapi -lesp_ringbuf -lefuse -lesp_ipc -ldriver -lesp_pm -lmbedtls -lapp_update -lbootloader_support -lspi_flash -lnvs_flash -lpthread -lesp_gdbstub -lespcoredump -lesp_phy -lesp_system -lesp_rom -lhal -lvfs -lesp_eth -ltcpip_adapter -lesp_netif -lesp_event -lwpa_supplicant -lesp_wifi -lconsole -llwip -llog -lheap -lsoc -lesp_hw_support -lxtensa -lesp_common -lesp_timer -lfreertos -lnewlib -lcxx -lapp_trace -lnghttp -lesp-tls -ltcp_transport -lesp_http_client -lesp_http_server -lesp_https_ota -lsdmmc -lesp_serial_slave_link -lulp -lmbedtls_2 -lmbedcrypto -lmbedx509 -lcoexist -lcore -lespnow -lmesh -lnet80211 -lpp -lsmartconfig -lwapi -lesp_ringbuf -lefuse -lesp_ipc -ldriver -lesp_pm -lmbedtls -lapp_update -lbootloader_support -lspi_flash -lnvs_flash -lpthread -lesp_gdbstub -lespcoredump -lesp_phy -lesp_system -lesp_rom -lhal -lvfs -lesp_eth -ltcpip_adapter -lesp_netif -lesp_event -lwpa_supplicant -lesp_wifi -lconsole -llwip -llog -lheap -lsoc -lesp_hw_support -lxtensa -lesp_common -lesp_timer -lfreertos -lnewlib -lcxx -lapp_trace -lnghttp -lesp-tls -ltcp_transport -lesp_http_client -lesp_http_server -lesp_https_ota -lsdmmc -lesp_serial_slave_link -lulp -lmbedtls_2 -lmbedcrypto -lmbedx509 -lcoexist -lcore -lespnow -lmesh -lnet80211 -lpp -lsmartconfig -lwapi -lesp_ringbuf -lefuse -lesp_ipc -ldriver -lesp_pm -lmbedtls -lapp_update -lbootloader_support -lspi_flash -lnvs_flash -lpthread -lesp_gdbstub -lespcoredump -lesp_phy -lesp_system -lesp_rom -lhal -lvfs -lesp_eth -ltcpip_adapter -lesp_netif -lesp_event -lwpa_supplicant -lesp_wifi -lconsole -llwip -llog -lheap -lsoc -lesp_hw_support -lxtensa -lesp_common -lesp_timer -lfreertos -lnewlib -lcxx -lapp_trace -lnghttp -lesp-tls -ltcp_transport -lesp_http_client -lesp_http_server -lesp_https_ota -lsdmmc -lesp_serial_slave_link -lulp -lmbedtls_2 -lmbedcrypto -lmbedx509 -lcoexist -lcore -lespnow -lmesh -lnet80211 -lpp -lsmartconfig -lwapi -lesp_ringbuf -lefuse -lesp_ipc -ldriver -lesp_pm -lmbedtls -lapp_update -lbootloader_support -lspi_flash -lnvs_flash -lpthread -lesp_gdbstub -lespcoredump -lesp_phy -lesp_system -lesp_rom -lhal -lvfs -lesp_eth -ltcpip_adapter -lesp_netif -lesp_event -lwpa_supplicant -lesp_wifi -lconsole -llwip -llog -lheap -lsoc -lesp_hw_support -lxtensa -lesp_common -lesp_timer -lfreertos -lnewlib -lcxx -lapp_trace -lnghttp -lesp-tls -ltcp_transport -lesp_http_client -lesp_http_server -lesp_https_ota -lsdmmc -lesp_serial_slave_link -lulp -lmbedtls_2 -lmbedcrypto -lmbedx509 -lcoexist -lcore -lespnow -lmesh -lnet80211 -lpp -lsmartconfig -lwapi -lesp_ringbuf -lefuse -lesp_ipc -ldriver -lesp_pm -lmbedtls -lapp_update -lbootloader_support -lspi_flash -lnvs_flash -lpthread -lesp_gdbstub -lespcoredump -lesp_phy -lesp_system -lesp_rom -lhal -lvfs -lesp_eth -ltcpip_adapter -lesp_netif -lesp_event -lwpa_supplicant -lesp_wifi -lconsole -llwip -llog -lheap -lsoc -lesp_hw_support -lxtensa -lesp_common -lesp_timer -lfreertos -lnewlib -lcxx -lapp_trace -lnghttp -lesp-tls -ltcp_transport -lesp_http_client -lesp_http_server -lesp_https_ota -lsdmmc -lesp_serial_slave_link -lulp -lmbedtls_2 -lmbedcrypto -lmbedx509 -lcoexist -lcore -lespnow -lmesh -lnet80211 -lpp -lsmartconfig -lwapi -lphy -lbtbb -lesp_phy -lphy -lbtbb -lesp_phy -lphy -lbtbb -lxt_hal -lc -lm -lnewlib -lstdc++ -lpthread -lgcc -lcxx  -Wl,--end-group -Wl,-EL -o "C:\Users\Utente\git\ESP32_S3_lightsaber\Release/ESP32_s3.elf"
	@echo 'Finished building: $@'
	@echo ' '

ESP32_s3.hex: $(ELF)
	@echo 'Esecuzione dei comandi objcopy'
	${recipe.objcopy.bin.pattern}
	${recipe.objcopy.hex.pattern}
	${recipe.objcopy.partitions.bin.pattern}
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(CCM_DEPS)$(C_UPPER_DEPS)$(PDE_DEPS)$(CXXM_DEPS)$(C_DEPS)$(AR)$(CC_DEPS)$(AR_OBJ)$(C++_DEPS)$(LINK_OBJ)$(CXX_DEPS)$(ASM_DEPS)$(HEX)$(INO_DEPS)$(SIZEDUMMY)$(S_UPPER_DEPS)$(C++M_DEPS)$(ELF)$(CPP_DEPS)
	-@echo ' '

.PHONY: all clean dependents main-build pre-build

-include ..\makefile.targets
