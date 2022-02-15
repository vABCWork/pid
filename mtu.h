

// COOL PWM (P17) MTIOC3A
#define COOL_PWM_PMR      (PORT1.PMR.BIT.B7)   // 0:汎用入出力ポート, 1:周辺機能として使用

// HEAT PWM (P27) MTIOC4A
#define HEAT_PWM_PMR      (PORT2.PMR.BIT.B7)   // 0:汎用入出力ポート, 1:周辺機能として使用



void PWM_Init_Reg(void);

void PWM_Port_Set(void);

void PWM_Stop(void);

void Set_PWM_duty(float out_mv);
