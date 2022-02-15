#include "iodefine.h"
#include  "misratypes.h"
#include  "thermocouple.h"
#include  "pid.h"

uint8_t   mode_stop_run;	// 現在の状態 (0:Stop, 1:Run)
uint8_t	  pre_mode_stop_run;    // １制御周期前の状態
uint8_t   mode_auto_manual;     // 現在の状態 (0:Auto, 1:Manual)
uint8_t	  pre_mode_auto_manual; // １制御周期前の状態

uint8_t heat_cool;             // 0:heater(逆動作), 1:cooler(正動作)
uint8_t pid_type; 	       // 0:PID位置型演算(D動作:完全微分), 1:PID位置型演算(D動作:不完全微分),
			       // 2:PID速度型演算(D動作:不完全微分)


const float  pid_sampling_time = 0.2;	// 制御周期 (0.2 sec )
const float  pid_d_eta = 0.125;		// 不完全微分　定数

float   pid_pv;			// 現在値　PV　 　単位=℃

				// パラメータ
float   pid_sv;			// 設定値(目標値) SP 　単位=℃
float	pid_p;			// 比例帯 (0.0-100.0)
float	pid_i;			// 積分時間 (0-3600 sec)
float   pid_d;			// 微分時間算 (0-1800 sec) 
float   pid_mr;			// マニュアルリセット量 (0.0-100.0)
float   pid_hys;		// ON/OFF制御(P=0)でのヒステリシス　単位=℃

// PID計算用
float	pid_kp;			// ゲイン= 100 / 比例帯
float   pid_ts_ti;		// 制御周期/積分時間 (Ts/Ti)

float   pid_en;			// 偏差 (今回)
float   pid_en1;		// 偏差 (前回)(1制御周期前)(200msec前)

float   pid_out;                // 出力( 0.0 -100.0 に制限された値)
float   pid_mv;			// PID演算後の出力(今回)  
float   pid_mvn1;		// PID演算後の出力(1周期前)  

float   pid_p_mv;		// P動作による出力
float   pid_i_mv;		// I動作による制御出力(今回) 
float   pid_d_mv;               // D動作による制御出力(今回)

float   pid_i_mvn1;		// I動作による制御出力(前回(1制御周期前))

float	pid_lagged;		// 不完全微分 出力(今回)
float   pid_lagged1;		// 不完全微分 出力(前回(1制御周期前))

float  pid_eta_td;		// ηTd
float  pid_ts_plus_eta_td;	// Ts + ηTd

//
//  制御動作
//
// モード　　　　　　　　  制御出力
//  stop  -------------  制御出力は、0%
//  run   --+- auto ---  P=0の場合、ON/OFF動作(出力　100%/ 0% )(ヒステリシス 1℃)
//          |            P>0の場合、PID演算による制御出力
//          +- manual--- 制御出力は外部(通信等)により変更   
//
//
// 制御
//   偏差(en) :  en = SV -PV (加熱制御時)(逆動作)
//               en = PV -SV (冷却制御時)(正動作)
//
//   ON/OFF 動作(P=0の場合)
//   PID動作(P> 0の場合)
//       P動作出力 P_MVn = Kp * en ,   Kp = 100/P
//       I動作出力 I_MVn = Kp * (( en / Ti )*Ts ) + I_MVn  
//       D動作出力 lagged = (ηTd / ( Ts +ηTd)) * lagged1 +  ( Td / (Ts +ηTd)) * (en - en1) 
//                  D_MVn = Kp * lagged
//
void control(void)
{
					// 現在の値を1制御周期前の値として保存
	
	pid_en1 = pid_en;		//  偏差
	pid_i_mvn1 = pid_i_mv;          // I動作の値( Kg を掛ける前の値)				
	pid_mvn1 = pid_mv;		// PID演算後の出力
	pid_lagged1 = pid_lagged;	// 不完全微分用出力
	
 				   //　偏差 en の計算
	if ( heat_cool == 0 ) {    //　逆動作(加熱制御)
	
		pid_en = pid_sv - pid_pv;    // 
	}
					            
        else if ( heat_cool == 1 ) {	     //　正動作(冷却制御)
		pid_en = pid_pv - pid_sv;    // 
	}
	
	if ( pid_p > 0 ){
		pid_kp = 100.0 / pid_p;      // P動作　ゲイン = 100/比例帯
	}
	else {
		pid_kp = 0.0;
	}
	
	if ( mode_stop_run == 0 ) {	// stopモード
	
		pid_i_mv = 0;
		pid_i_mvn1 = 0;
		pid_mv = 0;
		pid_out = 0;
		
		pid_p_mv = 0;
		pid_i_mv = 0;
		pid_d_mv = 0;
		
		pid_lagged1 = 0;
	}
	else {				// runモード
	
	   if ( mode_auto_manual == 0 ) {	// auto モード時
	   	if ( pid_p == 0.0 ) {   // P =0 の場合
		    on_off_control();	// ON/OFF 制御
		    
		    pid_p_mv = 0;
		    pid_i_mv = 0;
		    pid_d_mv = 0;
		}
		else {                  // P > 0　の場合
		
		  if ( pid_d  == 0.0 ) {		// 微分動作なし
	    		pid_d_mv = 0.0;		// 微分出力 = 0
		  }
		  else {			// 微分動作あり (不完全微分で計算)
		  
		  	pid_eta_td = pid_d_eta * pid_d;     //　ηTd
			pid_ts_plus_eta_td = pid_sampling_time + pid_eta_td; // Ts + ηTd
						// 不完全微分の計算		       			
			pid_lagged = ( pid_eta_td / pid_ts_plus_eta_td) * pid_lagged1 + ( pid_d /  pid_ts_plus_eta_td) * ( pid_en - pid_en1);
		        pid_d_mv = pid_kp *  pid_lagged;
			
			                        // 完全微分の計算(参考)
			// pid_d_mv = pid_kp *  ( pid_d / pid_sampling_time ) * (pid_en - pid_en1);   // ( Td/Ts ) * (En - En1)
		  }
		
		  if ( pid_i == 0.0 ) {    // I動作無しの場合
		  	pid_p_mv =  pid_kp * pid_en;
		   	pid_mv = pid_p_mv + pid_d_mv + pid_mr;
		   	pid_i_mv = 0;
		  }
		  else {                  // I動作ありの場合
		       pid_p_mv =  pid_kp * pid_en;

	     	       pid_i_mv = pid_kp * (  pid_sampling_time / pid_i ) * pid_en + pid_i_mvn1;           // I動作 出力
	              
		       pid_mv =   pid_p_mv + pid_i_mv + pid_d_mv ;   // PID動作　出力
		   
		  }
		   
		} // P > 0
	   }   
	   else if ( mode_auto_manual == 1 ){	// manual　モード
 						// 通信で pid_mv入力
		pid_p_mv = 0;
		pid_i_mv = 0;
		pid_d_mv = 0;				
	   }
	}
	 
	 	                         // 出力リミット 
					 // 速度型PID演算の場合、PID演算結果(pid_mv)をリミットすると情報が失われるため、pid_mvは制限しない。
	if ( pid_mv > 100.0 ) {          // 演算後の出力が100以上の場合、
		pid_out = 100.0;         // 出力 100%にリミット
	}
	else if ( pid_mv < 0.0 ) {        // 演算後の出力が0未満の場合
		pid_out = 0.0;            // 出力 0%にリミット
	}
	else {                            //　PID演算後の値が0から100%以内の場合、そのまま出力
		pid_out = pid_mv;
	}
	
	
	Set_PWM_duty(pid_out);		// PWM ON/OFF時間設定
	 
	 
}



//
//  ON-OFF 制御　
// pid_p = 0でON-OFF制御となる。 
//
// 加熱制御(逆動作）(heat_cool = 0) の場合、
//   PV <= (SV - ヒステリシス) ならばON (pid_mv = 100%) 
//   PV >= SP ならば、OFF (pid_mv= 0%)
//   (SP-ヒステリシス) < PV < SV ならば、出力維持(1周期前のpid_mv)
//
// 冷却制御(正動作）(heat_cool = 1) の場合、
//   PV >= (SV + ヒステリシス) ならば、ON(pid_mv = 100%) 
//   PV <= SP ならば、OFF(pid_mv= 0%)
//
void on_off_control(void)
{
	
	if ( heat_cool == 0 ) {	//　逆動作(加熱制御)
		if (  pid_pv >=  pid_sv  ) {    

			pid_mv = 0.0;		// 出力 OFF
		}
		else if ( pid_pv <= (pid_sv - pid_hys )) {  
		
			pid_mv = 100.0;		// 出力 ON 
		}
		else {				
			pid_mv = pid_mvn1;	// 前回の出力
		}
	}
	
	else if ( heat_cool == 1 ) {		//　正動作(冷却制御)
		if (  pid_pv <=  pid_sv  ) {  

			pid_mv = 0.0;		// 出力 OFF
		}
		else if ( pid_pv >= (pid_sv + pid_hys )) { 
		
			pid_mv = 100.0;		// 出力 ON 
		}
		else {			
			pid_mv = pid_mvn1;	// 前回の出力
		}
	}
}



