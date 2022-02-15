
extern	uint8_t   mode_stop_run;	// 0:Stop, 1:Run
extern	uint8_t	  pre_mode_stop_run; 
extern  uint8_t   mode_auto_manual;     // 0:Auto, 1:Manual
extern  uint8_t   pre_mode_auto_manual;
extern  uint8_t heat_cool;
extern	uint8_t pid_type;

extern  float   pid_pv;

extern	float   pid_sv;	
extern	float	pid_p;
extern	float	pid_i;
extern	float   pid_d;
extern  float   pid_mr;		
extern  float   pid_hys;

			
extern  float   pid_kp;

extern	float   pid_en;	
extern  float   pid_out;
extern  float   pid_mv;	
extern  float   pid_mvn1;

extern float   pid_p_mv;
extern float   pid_i_mv;
extern float   pid_d_mv;

void control(void);
void manual_control(void);
void on_off_control(void);

