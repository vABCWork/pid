
//  ���M����LED�p�@�o�̓|�[�g P16
#define LED_TX_PMR      (PORT1.PMR.BIT.B6)   // 0:�ėp���o�̓|�[�g
#define LED_TX_PDR      (PORT1.PDR.BIT.B6)   // 1: �o�̓|�[�g�Ɏw��
#define LED_TX_PODR     (PORT1.PODR.BIT.B6)  //   �o�̓f�[�^


//  ��M����LED�p�@�o�̓|�[�g P15
#define LED_RX_PMR      (PORT1.PMR.BIT.B5)   // 0:�ėp���o�̓|�[�g
#define LED_RX_PDR      (PORT1.PDR.BIT.B5)   // 1: �o�̓|�[�g�Ɏw��
#define LED_RX_PODR     (PORT1.PODR.BIT.B5)  //   �o�̓f�[�^



extern  volatile uint8_t  rcv_cnt;
extern	volatile uint8_t rcv_over;


void initSCI_1(void);
void LED_comm_port_set(void);

void comm_cmd(void);

uint32_t write_para_data(void);
uint32_t set_mode_index(void);
uint32_t set_mv_data(void);
uint32_t resp_temp_para_read(void);




