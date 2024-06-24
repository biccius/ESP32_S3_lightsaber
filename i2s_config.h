/*
 * i2s_audio_driver.h
 *
 *  Created on: 2 mar 2024
 *      Author: fabry
 */

#ifndef I2S_CONFIG_H_
#define I2S_CONFIG_H_

//------------------------------------------------------------------------------------------------------------------------
// structures and also variables
//  I2S configuration

      static const i2s_config_t i2s_config =
      {
          .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
          .sample_rate = 22050,                                 // Note, all files must be this
          .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
          .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
          .communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
          .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,             // high interrupt priority
          .dma_buf_count = 32,                                   // 32 buffers
          .dma_buf_len = 256,                                   // 256 bytes per buffer, so 8192K of buffer space
          .use_apll=0,
          .tx_desc_auto_clear= true,
          .fixed_mclk=-1
      };


      static const i2s_pin_config_t pin_config =
      {
          .bck_io_num = I2S_BCLK,                           // The bit clock connectiom, goes to pin 27 of ESP32
          .ws_io_num = I2S_LRC,                             // Word select, also known as word select or left right clock
          .data_out_num = I2S_DOUT,                         // Data out from the ESP32, connect to DIN on 38357A
          .data_in_num = I2S_PIN_NO_CHANGE                  // we are not interested in I2S data into the ESP32
      };


#endif /* I2S_CONFIG_H_ */
