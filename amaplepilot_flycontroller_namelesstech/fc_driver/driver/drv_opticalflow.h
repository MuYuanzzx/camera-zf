





void uart4_init(uint32_t baud);
void uart4_send_bytes(unsigned char *buf, int len);

void opticalflow_pretreat(void);
void uart4_optical_flow_init(void);

void lc307_opticalflow_prase(uint8_t data);


extern opticalflow opt_data;
extern filter_parameter opticalflow_filter_parameter,gyro_sync_filter_parameter;


