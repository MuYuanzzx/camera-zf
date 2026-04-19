
typedef uint8_t qdatatype;


#define QUENE_MAX   200
typedef struct
{
	uint8_t *data_ptr;
	int16_t front;
	int16_t rear;
	uint16_t len;
}linear_quene;



uint8_t is_full(linear_quene* ptr);
uint8_t is_empty(linear_quene* ptr);
void enquene(linear_quene* ptr,qdatatype value);
qdatatype dequene(linear_quene* ptr);
void initquene(linear_quene *ptr,qdatatype *pdata,uint16_t len);
void resetquene(linear_quene *ptr);




#define  RINGBUFF_LEN   200     //定义最大接收字节数
typedef struct
{
  uint16_t head;           
  uint16_t tail;
  uint16_t lenght;
  uint8_t  ring_buff[RINGBUFF_LEN];
}ring_buff_t;

void ringbuff_init(ring_buff_t *ptr);
uint8_t write_ringbuff(uint8_t data,ring_buff_t *ptr);
uint8_t read_ringbuff(uint8_t *rdata,ring_buff_t *ptr);
/**************************************************************************/
void ringbuf_write(unsigned char data,ring_buff_t *ptr,uint16_t len);
uint8_t ringbuf_read(unsigned char* pData,ring_buff_t *ptr);






