#ifndef _ADC_BUTTON_
#define _ADC_BUTTON_


	


void adc_button_init(void);
void adc_button_read(void);
void adc_button_scan(void);

extern float Button_ADCResult[2];
extern uint8_t Key_Value[2];
extern button_state   ADC_Button1,ADC_Button2,ADC_Button3,ADC_Button4;
extern uint16_t ADC_PPM_Databuf[10];


extern uint8_t adc_ppm_update_flag;

extern uint8_t adc_key1_ctrl_enable,adc_key2_ctrl_enable,wireless_adc_key_ctrl_enable;

#endif


