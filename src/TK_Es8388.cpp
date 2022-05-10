#include <TK_Es8388.h>

TK_Es8388::TK_Es8388(Adafruit_I2CDevice *i2cdevice) : AudioAdapter(i2cdevice) {

}

bool TK_Es8388::es_read_reg(uint8_t reg_add, uint8_t *p_data)
{
    return i2cdevice->write_then_read(&reg_add, sizeof(reg_add), p_data, 1, true);
}

bool TK_Es8388::es_write_reg(uint8_t reg_add, uint8_t data)
{
    return i2cdevice->write( &data, sizeof(data), true, &reg_add, sizeof(reg_add));
}

bool TK_Es8388::init() {
  // if(true)
  //   return true;
  bool res = true;

  res &= es_write_reg(ES8388_DACCONTROL3, 0x04);  // 0x04 mute/0x00 unmute&ramp;DAC unmute and  disabled digital volume control soft ramp
  /* Chip Control and Power Management */
  res &= es_write_reg(ES8388_CONTROL2, 0x40);  //Enable vref
  res &= es_write_reg(ES8388_CHIPPOWER, 0x00); //normal all and power up all

  // Disable the internal DLL to improve 8K sample rate
  res &= es_write_reg(0x35, 0xA0);
  res &= es_write_reg(0x37, 0xD0);
  res &= es_write_reg(0x39, 0xD0);

  res &= es_write_reg(ES8388_MASTERMODE, AUDIO_HAL_MODE_SLAVE); //CODEC IN I2S SLAVE MODE

  /* dac */
  res &= es_write_reg(ES8388_DACPOWER, 0xC0);  //disable DAC and disable Lout/Rout/1/2
  res &= es_write_reg(ES8388_CONTROL1, 0x15);  //Enfr=0,Play&Record Mode,(0x17-both of mic&paly)
  res &= es_write_reg(ES8388_DACCONTROL1, 0x18);//1a 0x18:16bit iis , 0x00:24
  res &= es_write_reg(ES8388_DACCONTROL2, 0x02);  //DACFsMode,SINGLE SPEED; DACFsRatio,256
  res &= es_write_reg(ES8388_DACCONTROL16, 0x00); // 0x00 audio on LIN1&RIN1,  0x09 LIN2&RIN2
  res &= es_write_reg(ES8388_DACCONTROL17, 0x90); // only left DAC to left mixer enable 0db
  res &= es_write_reg(ES8388_DACCONTROL20, 0x90); // only right DAC to right mixer enable 0db

  res &= es_write_reg(ES8388_DACCONTROL21, 0x80); //set internal ADC and DAC use the same LRCK clock, ADC LRCK as internal LRCK
  res &= es_write_reg(ES8388_DACCONTROL23, 0x00);   //vroi=0
  res &= es8388_set_adc_dac_volume(ES_MODULE_DAC, 0, 0);          // 0db
  int tmp = 0;
  //TODO: define DAC output by paramter
  audio_hal_dac_output_t dac_output = AUDIO_HAL_DAC_OUTPUT_ALL;
  if (AUDIO_HAL_DAC_OUTPUT_LINE1 == dac_output) {
      tmp = DAC_OUTPUT_LOUT1 | DAC_OUTPUT_ROUT1;
  } else if (AUDIO_HAL_DAC_OUTPUT_LINE2 == dac_output) {
      tmp = DAC_OUTPUT_LOUT2 | DAC_OUTPUT_ROUT2;
  } else {
      tmp = DAC_OUTPUT_LOUT1 | DAC_OUTPUT_LOUT2 | DAC_OUTPUT_ROUT1 | DAC_OUTPUT_ROUT2;
  }
  res &= es_write_reg(ES8388_DACPOWER, tmp);  //0x3c Enable DAC and Enable Lout/Rout/1/2
  /* adc */
  res &= es_write_reg(ES8388_ADCPOWER, 0xFF);
  res &= es_write_reg(ES8388_ADCCONTROL1, 0x88); // MIC Left and Right channel PGA gain
  tmp = 0;
  //TODO: define DAC output by paramter
  audio_hal_adc_input_t adc_input = AUDIO_HAL_ADC_INPUT_DIFFERENCE;
  if (AUDIO_HAL_ADC_INPUT_LINE1 == adc_input) {
      tmp = ADC_INPUT_LINPUT1_RINPUT1;
  } else if (AUDIO_HAL_ADC_INPUT_LINE2 == adc_input) {
      tmp = ADC_INPUT_LINPUT2_RINPUT2;
  } else {
      tmp = ADC_INPUT_DIFFERENCE;
  }
  res &= es_write_reg(ES8388_ADCCONTROL2, tmp);  //0x00 LINSEL & RINSEL, LIN1/RIN1 as ADC Input; DSSEL,use one DS Reg11; DSR, LINPUT1-RINPUT1
  res &= es_write_reg(ES8388_ADCCONTROL3, 0x02);  //L1-R1
  //res &= es_write_reg(ES8388_ADCCONTROL3, 0x82);   //L2-R2

  res &= es_write_reg(ES8388_ADCCONTROL4, 0x0c); // Left data, Left/Right justified mode, Bits length, I2S format
  res &= es_write_reg(ES8388_ADCCONTROL5, 0x02);  //ADCFsMode,singel SPEED,RATIO=256
  //ALC for Microphone
  res &= es_write_reg(ES8388_ADCCONTROL10, 0xea);  //0x38 ALC gain range, Stero
  res &= es_write_reg(ES8388_ADCCONTROL11, 0xc0);  //0xb0 ALC target level and hold time
  res &= es_write_reg(ES8388_ADCCONTROL12, 0x12);  //0x32 ALC ramp up & ramp down mode
  res &= es_write_reg(ES8388_ADCCONTROL13, 0x06);  //0x06 ALC mode of operation
  res &= es_write_reg(ES8388_ADCCONTROL14, 0xc3);  //0x00 ALC noise gate threshold

  res &= es8388_set_adc_dac_volume(ES_MODULE_ADC, 0, 0);      // 0db
  res &= es_write_reg(ES8388_ADCPOWER, 0x00); //Power on ADC, Enable LIN&RIN, Power off MICBIAS, set int1lp to low power mode
  
  return res;
}

/**
 * @brief Configure ES8388 ADC and DAC volume. Basicly you can consider this as ADC and DAC gain
 *
 * @param mode:             set ADC or DAC or all
 * @param volume:           -96 ~ 0              for example Es8388SetAdcDacVolume(ES_MODULE_ADC, 30, 6); means set ADC volume -30.5db
 * @param dot:              whether include 0.5. for example Es8388SetAdcDacVolume(ES_MODULE_ADC, 30, 4); means set ADC volume -30db
 *
 * @return
 *     - (false) Parameter error
 *     - (true)   Success
 */
bool TK_Es8388::es8388_set_adc_dac_volume(int mode, int volume, int dot)
{
    bool res = true;
    ESP_LOGW(ES_TAG, "es8388_set_adc_dac_volume\n");
    if ( volume < -96 || volume > 0 ) {
        ESP_LOGW(ES_TAG, "Warning: volume < -96! or > 0!\n");
        if (volume < -96)
            volume = -96;
        else
            volume = 0;
    }
    dot = (dot >= 5 ? 1 : 0);
    volume = (-volume << 1) + dot;
    if (mode == ES_MODULE_ADC || mode == ES_MODULE_ADC_DAC) {
        res &= es_write_reg(ES8388_ADCCONTROL8, volume);
        res &= es_write_reg(ES8388_ADCCONTROL9, volume);  //ADC Right Volume=0db
    }
    if (mode == ES_MODULE_DAC || mode == ES_MODULE_ADC_DAC) {
        res &= es_write_reg(ES8388_DACCONTROL5, volume);
        res &= es_write_reg(ES8388_DACCONTROL4, volume);
    }
    return res;
}

bool TK_Es8388::setVolume(int8_t volume) {
  
  return es8388_set_voice_volume(volume);
}

/**
 * @brief Power Management
 *
 * @param mod:      if ES_POWER_CHIP, the whole chip including ADC and DAC is enabled
 * @param enable:   false to disable true to enable
 *
 * @return
 *     - (-1)  Error
 *     - (0)   Success
 */
bool TK_Es8388::es8388_stop(es_module_t mode)
{
    bool res = true;
    if (mode == ES_MODULE_LINE) {
        res &= es_write_reg(ES8388_DACCONTROL21, 0x80); //enable dac
        res &= es_write_reg(ES8388_DACCONTROL16, 0x00); // 0x00 audio on LIN1&RIN1,  0x09 LIN2&RIN2
        res &= es_write_reg(ES8388_DACCONTROL17, 0x90); // only left DAC to left mixer enable 0db
        res &= es_write_reg(ES8388_DACCONTROL20, 0x90); // only right DAC to right mixer enable 0db
        return res;
    }
    if (mode == ES_MODULE_DAC || mode == ES_MODULE_ADC_DAC) {
        res &= es_write_reg(ES8388_DACPOWER, 0x00);
        res &= es8388_set_voice_mute(true); //res |= Es8388SetAdcDacVolume(ES_MODULE_DAC, -96, 5);      // 0db
        res &= es_write_reg(ES8388_DACPOWER, 0xC0);  //power down dac and line out
    }
    if (mode == ES_MODULE_ADC || mode == ES_MODULE_ADC_DAC) {
        //res |= Es8388SetAdcDacVolume(ES_MODULE_ADC, -96, 5);      // 0db
        res &= es_write_reg(ES8388_ADCPOWER, 0xFF);  //power down adc and line in
    }
    if (mode == ES_MODULE_ADC_DAC) {
        res &= es_write_reg(ES8388_DACCONTROL21, 0x9C);  //disable mclk
//        res |= es_write_reg(ES8388_CONTROL1, 0x00);
//        res |= es_write_reg(ES8388_CONTROL2, 0x58);
       res &= es_write_reg(ES8388_CHIPPOWER, 0xF3);  //stop state machine
    }

    return res;
}

/**
 * @brief Configure ES8388 DAC mute or not. Basically you can use this function to mute the output or unmute
 *
 * @param enable: enable or disable
 *
 * @return
 *     - (-1) Parameter error
 *     - (0)   Success
 */
bool TK_Es8388::es8388_set_voice_mute(bool enable)
{
    bool res = true;
    uint8_t reg = 0;
    res = es_read_reg(ES8388_DACCONTROL3, &reg);
    reg = reg & 0xFB;
    res &= es_write_reg(ES8388_DACCONTROL3, reg | (((int)enable) << 2));
    return res;
}

/**
 * @param volume: 0 ~ 100
 *
 * @return
 *     - (-1)  Error
 *     - (0)   Success
 */
bool TK_Es8388::es8388_set_voice_volume(int volume)
{
    bool res = true;
    ESP_LOGE(ES_TAG, "es8388_set_voice_volume");
    if (volume < 0)
        volume = 0;
    else if (volume > 100)
        volume = 100;
    volume /= 3;
    res = es_write_reg(ES8388_DACCONTROL24, volume);
    res &= es_write_reg(ES8388_DACCONTROL25, volume);
    res &= es_write_reg(ES8388_DACCONTROL26, volume);
    res &= es_write_reg(ES8388_DACCONTROL27, volume);
    return res;
}

/*!
 *    @brief  设置采样位宽
 *    @param  bits 采样位宽。支持 16, 24, 32
 *    @return 设置成功则返回 true
 */
bool TK_Es8388::setBitsPerSample(int bits_length) {
  esp_err_t res = ESP_OK;
  uint8_t reg = true;
  //Default is 16 bit
  int bits = BIT_LENGTH_16BITS;
  if(bits_length == 32) {
    bits = BIT_LENGTH_32BITS;
  } else if(bits_length == 24) {
    bits = BIT_LENGTH_24BITS;
  } else if(bits_length == 16) {
    bits = BIT_LENGTH_16BITS;
  } else {
    ESP_LOGE(ES_TAG, "The bit length %d is not supported by codec", bits_length);
  }

  //TODO: Now we only support ADC & DAC in same format, if want to support different format, do it later or use soft resampler

  // if (mode == ES_MODULE_ADC || mode == ES_MODULE_ADC_DAC) {
      res = es_read_reg(ES8388_ADCCONTROL4, &reg);
      reg = reg & 0xe3;
      res &= es_write_reg(ES8388_ADCCONTROL4, reg | (bits << 2));
  // }
  // if (mode == ES_MODULE_DAC || mode == ES_MODULE_ADC_DAC) {
      res = es_read_reg(ES8388_DACCONTROL1, &reg);
      reg = reg & 0xc7;
      res &= es_write_reg(ES8388_DACCONTROL1, reg | (bits << 3));
  // }
  return res;
}
/*!
 *    @brief  设置声道数
 *    @param  chan 声道数。可选值 1, 2
 *    @return 设置成功则返回 true
 */
bool TK_Es8388::setChannels(int chan){
  //TODO: configure single channel
  return true;
};

/*!
 *    @brief  启动音频外设
 *    @param  mode 外设工作模式，放音、录音还是全双工
 *    @return 设置成功则返回 true
 */
bool TK_Es8388::start(audio_hal_codec_mode_t mode) {
  bool res = true;
  uint8_t prev_data = 0, data = 0;
  es_read_reg(ES8388_DACCONTROL21, &prev_data);
  if (mode == AUDIO_HAL_CODEC_MODE_LINE_IN) {
      res &= es_write_reg(ES8388_DACCONTROL16, 0x09); // 0x00 audio on LIN1&RIN1,  0x09 LIN2&RIN2 by pass enable
      res &= es_write_reg(ES8388_DACCONTROL17, 0x50); // left DAC to left mixer enable  and  LIN signal to left mixer enable 0db  : bupass enable
      res &= es_write_reg(ES8388_DACCONTROL20, 0x50); // right DAC to right mixer enable  and  LIN signal to right mixer enable 0db : bupass enable
      res &= es_write_reg(ES8388_DACCONTROL21, 0xC0); //enable adc
  } else {
      res &= es_write_reg(ES8388_DACCONTROL21, 0x80);   //enable dac
  }
  es_read_reg(ES8388_DACCONTROL21, &data);
  if (prev_data != data) {
      res &= es_write_reg(ES8388_CHIPPOWER, 0xF0);   //start state machine
      // res |= es_write_reg(ES8388_CONTROL1, 0x16);
      // res |= es_write_reg(ES8388_CONTROL2, 0x50);
      res &= es_write_reg(ES8388_CHIPPOWER, 0x00);   //start state machine
  }
  if (mode == AUDIO_HAL_CODEC_MODE_ENCODE || mode == AUDIO_HAL_CODEC_MODE_BOTH || mode == AUDIO_HAL_CODEC_MODE_LINE_IN) {
      res &= es_write_reg(ES8388_ADCPOWER, 0x00);   //power up adc and line in
  }
  if (mode == AUDIO_HAL_CODEC_MODE_DECODE || mode == AUDIO_HAL_CODEC_MODE_BOTH || mode == AUDIO_HAL_CODEC_MODE_LINE_IN) {
      res &= es_write_reg(ES8388_DACPOWER, 0x3c);   //power up dac and line out
      res &= es8388_set_voice_mute(false);
  }

  return res;
}

/*!
 *    @brief  停止音频外设
 *    @param  mode 外设工作模式，放音、录音还是全双工
 *    @return 设置成功则返回 true
 */
bool TK_Es8388::stop(audio_hal_codec_mode_t mode) {

  bool res = true;
  if (mode == AUDIO_HAL_CODEC_MODE_LINE_IN) {
      res &= es_write_reg(ES8388_DACCONTROL21, 0x80); //enable dac
      res &= es_write_reg(ES8388_DACCONTROL16, 0x00); // 0x00 audio on LIN1&RIN1,  0x09 LIN2&RIN2
      res &= es_write_reg(ES8388_DACCONTROL17, 0x90); // only left DAC to left mixer enable 0db
      res &= es_write_reg(ES8388_DACCONTROL20, 0x90); // only right DAC to right mixer enable 0db
      return res;
  }
  if (mode == AUDIO_HAL_CODEC_MODE_DECODE || mode == AUDIO_HAL_CODEC_MODE_BOTH) {
      res &= es_write_reg(ES8388_DACPOWER, 0x00);
      res &= es8388_set_voice_mute(true); //res |= Es8388SetAdcDacVolume(ES_MODULE_DAC, -96, 5);      // 0db
      res &= es_write_reg(ES8388_DACPOWER, 0xC0);  //power down dac and line out
  }
  if (mode == AUDIO_HAL_CODEC_MODE_ENCODE || mode == AUDIO_HAL_CODEC_MODE_BOTH) {
      //res |= Es8388SetAdcDacVolume(ES_MODULE_ADC, -96, 5);      // 0db
      res &= es_write_reg(ES8388_ADCPOWER, 0xFF);  //power down adc and line in
  }
  if (mode == AUDIO_HAL_CODEC_MODE_BOTH) {
      res &= es_write_reg(ES8388_DACCONTROL21, 0x9C);  //disable mclk
//        res |= es_write_reg(ES8388_CONTROL1, 0x00);
//        res |= es_write_reg(ES8388_CONTROL2, 0x58);
      res &= es_write_reg(ES8388_CHIPPOWER, 0xF3);  //stop state machine
  }

  return res;
}