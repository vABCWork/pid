
#include "iodefine.h"
#include "misratypes.h"
#include "sci.h"
#include "dsad.h"
#include "thermocouple.h"
#include  "pid.h"

//
//  SCI1 シリアル通信(調歩同期)処理
//

// 受信用
volatile uint8_t  rcv_data[128];
volatile uint8_t rxdata;
volatile uint8_t rcv_over;
volatile uint8_t  rcv_cnt;

// 送信用
volatile uint8_t sd_data[4004];
volatile uint8_t  send_cnt;
volatile uint8_t  send_pt;


#pragma interrupt (Excep_SCI1_RXI1(vect=219))
//
// 受信割り込み
//
void Excep_SCI1_RXI1(void)
{
	uint8_t cmd;
	
	rxdata = SCI1.RDR;	// 受信データ読み出し
 	
 	rcv_data[rcv_cnt] = rxdata;
	
	rcv_cnt++;
	
	LED_RX_PODR = 1;	        // 受信 LEDの点灯
	
	cmd = rcv_data[0];
	
	if ((cmd == 0x03) || (cmd == 0x05)) {   // パラメータ書き込みコマンド(0x03) または、MV値変更コマンド(0x05)の場合
	    if ( rcv_cnt == 8 ) {   //  合計 8 byte 受信で、受信完了
		rcv_over = 1;
	    }
	}
	
	if (( cmd == 0x04) || ( cmd == 0x51)) {    // モード変更コマンド(0x04) または、温度とパラメータ読み出しコマンド(0x51)	
 		if ( rcv_cnt == 4 ) {   //  合計 4 byte 受信で、受信完了
		  rcv_over = 1;
 		}
	}
	
}



#pragma interrupt (Excep_SCI1_TEI1(vect=221))

//
// 送信終了割り込み
//
void Excep_SCI1_TEI1(void)
{	 
	SCI1.SCR.BIT.TE = 0;            // 送信禁止
        SCI1.SCR.BIT.TIE = 0;           // 送信割り込み禁止	        
	SCI1.SCR.BIT.TEIE = 0;  	// TEI割り込み(送信終了割り込み)禁止

	LED_TX_PODR = 0;	        // 送信 LEDの消灯
	
 }
 
 
 
// コマンド受信の対する、コマンド処理とレスポンス作成処理
//
// 0x50 :モニタコマンド(ch1,ch2,ch3,ch4,cjtを読み出す)
//

void comm_cmd(void){
   
	uint8_t  cmd;
	uint32_t sd_cnt;

	cmd = rcv_data[0];
        
	sd_cnt = 0;
	
	if ( cmd == 0x03) {        // パラメータ書き込みコマンド(0x03)
	    sd_cnt = write_para_data();	// パラメータ書込みと用送信データ作成
	}
	
	else if ( cmd == 0x04) {	 // モード変更コマンド(0x04)
	     sd_cnt = set_mode_index();	 // モード変更と、送信データ作成
	}
	
	else if ( cmd == 0x05) {	// MV値変更コマンド(0x05)
	     sd_cnt = set_mv_data();	// MV値の変更
	}
	
	else if ( cmd == 0x51 ) {        // 温度パラメータモニタコマンド
	    sd_cnt = resp_temp_para_read();
	}
	
	
	 
	DMAC1.DMSAR = (void *)&sd_data[0];	 // 転送元アドレス		
	DMAC1.DMDAR = (void *)&SCI1.TDR;	 // 転送先アドレス TXD12 送信データ

	DMAC1.DMCRA = sd_cnt; 	 	// 転送回数 (送信バイト数)	
	    
	DMAC1.DMCNT.BIT.DTE = 1;    // DMAC1 (DMAC チャンネル1) 転送許可
	
	    			   // 一番最初の送信割り込み(TXI)を発生させる処理。 ( RX23E-A ユーザーズマニュアル　ハードウェア編　28.3.7 シリアルデータの送信（調歩同期式モード）)　
	SCI1.SCR.BIT.TIE = 1;      // 送信割り込み許可
	SCI1.SCR.BIT.TE = 1;	   // 送信許可
	
	LED_TX_PODR = 1;	   // 送信 LEDの点灯
}




//
//  パラメータ書き込みコマンド(0x03)の処理
//
//  受信データ
//  rcv_data[0];　0x03 (パラメータ書き込みコマンド)
//  rcv_data[1]:  書き込みパラメータのインデックス
//  rcv_data[2]:  dummy
//  rcv_data[3]:  dummy
//  rcv_data[4]:  書き込みデータ (float) 単精度浮動小数点データの最下位バイト または byte型
//  rcv_data[5]:
//  rcv_data[6]:
//  rcv_data[7]: (float) 単精度浮動小数点データの最上位バイト
//
//  para_index:parameter 
//  0:SV, 1:P, 2:I, 3:D, 4:Mr, 5:Hys, 6:heat_cool, 7:pid_type
//
//
// 送信データ :
//     sd_data[0] : 0x83 (パラメータ書き込みコマンドに対するレスポンス)
//     se_data[1] : status ( 0x00:成功 , 0xff:書き込みデータ範囲外)
//     sd_data[2] : dummy 
//     sd_data[3] : dummy 
//
uint32_t	write_para_data(void)
{
	uint8_t id;
	uint8_t rcv_d;
	uint8_t status;
	
	float *pt;
	
	
	uint32_t cnt;
	
	
	id  = rcv_data[1];    // パラメータのインデックス
	
	pt = (float *)&rcv_data[4];       // 書き込みデータのアドレス
	rcv_d = rcv_data[4];
	
	
	if ( id == 0 ) {	// SV (float型)
	   if ( ( *pt >= 0.0 ) && ( *pt <= 100.0 ) ) {  // 範囲内の場合 
		pid_sv = *pt;
		status = 0x00;
           }
	   else {					// 範囲外
	   	status = 0xff;
	   }
	}
	
	else if ( id == 1 ) {	// P (float型)
	  if ( ( *pt >= 0.0 ) && ( *pt <= 100.0 ) ) {  // 範囲内の場合 
		pid_p = *pt;
		status = 0x00;
           }
	   else {					// 範囲外
	   	status = 0xff;
	   }	
	}
	
	else if ( id == 2 ) {	// I (float型)
	  if ( ( *pt >= 0.0 ) && ( *pt <= 360.0) ) {  // 範囲内の場合 
		pid_i = *pt;
		status = 0x00;
	  }
	  else {			// 範囲外
	   	status = 0xff;
	   }	
	}
	else if ( id == 3 ) {   // D (float型)
	  if ( ( *pt >= 0.0 ) && ( *pt <= 360.0) ) {  // 範囲内の場合 
	 	pid_d = *pt;
		status = 0x00;
	  }
	  else {			// 範囲外
	   	status = 0xff;
	  }	
	}
	else if ( id == 4 ) {   // Mr (float型)
	  if ( ( *pt >= 0.0 ) && ( *pt <= 100.0 ) ) {  // 範囲内の場合 
		pid_mr = *pt;
		status = 0x00;
	  }
	  else {			// 範囲外
	   	status = 0xff;
	  }	
	}
	else if ( id == 5 ) {   	// Hys (float型)
	  if ( ( *pt >= 0.0 ) && ( *pt <= 100.0 ) ) {  // 範囲内の場合 
		pid_hys = *pt;
		status = 0x00;
	  }
	  else {			// 範囲外
	   	status = 0xff;
	  }	
	}
	
	else if ( id == 6 ) {			// heat_cool (byte型)
	  if ( ( rcv_d >= 0 ) && ( rcv_d <= 1 ) ) {  // 範囲内の場合 
		heat_cool = rcv_d;
		status = 0x00;
	  }
	  else {			// 範囲外
	   	status = 0xff;
	  }	
	}
	
	else if ( id == 7 ) {			// pid_type (byte 型)
	  if ( ( rcv_d >= 0 ) && ( rcv_d <= 2 ) ) {  // 範囲内の場合 
		pid_type = rcv_d;
	  	status = 0x00;
	  }
	  else {			// 範囲外
	   	status = 0xff;
	  }	
	}
	
	
	cnt = 4;			// 送信バイト数
	
	sd_data[0] = 0x83;	 	// パラメータ書き込みコマンドに対するレスポンス	
	sd_data[1] = status;		//( 0x00:成功 , 0xff:書き込みデータ範囲外)
	sd_data[2] = 0;			
	sd_data[3] = 0;
	
	
	return cnt;
	
}


// 
// 
//  制御モード変更コマンド(0x04)の処理
//
//  受信データ
//  rcv_data[0];　0x04 (制御モード変更コマンド)
//  rcv_data[1]:  mode_stop_run （動作モード)
//  rcv_data[2]:  mode_auto_manual (制御モード)
//  rcv_data[3]:  dummy
//
//
// 送信データ :
//     sd_data[0] : 0x84 (制御モード変更コマンドに対するレスポンス)
//     se_data[1] : mode_stop_run 
//     sd_data[2] : mode_auto_manual
//     sd_data[3] : dummy 
//
uint32_t	set_mode_index(void)
{
	uint32_t cnt;
	
	pre_mode_stop_run = mode_stop_run;	// 変更前の値を保存
	pre_mode_auto_manual = mode_auto_manual;
	
	mode_stop_run = rcv_data[1];	//  mode_stop_run  ( 0:stop, 1:run )
	mode_auto_manual = rcv_data[2]; //  mode_auto_manual  ( 0:auto, 1:manul) 
	
	cnt = 4;			// 送信バイト数
	
	sd_data[0] = 0x84;	 	// 制御モード変更コマンドに対するレスポンス	
	sd_data[1] = mode_stop_run;
	sd_data[2] = mode_auto_manual;			
	sd_data[3] = 0;
	
	
	return cnt;
	
}


//
//  MV値変更コマンド(0x05)の処理
//
//  受信データ
//  rcv_data[0];　0x05 (MV価変更コマンド)
//  rcv_data[1]:  dummy 
//  rcv_data[2]:  dummy
//  rcv_data[3]:  dummy
//  rcv_data[4]:  書き込みデータ (float) 単精度浮動小数点データの最下位バイト
//  rcv_data[5]:
//  rcv_data[6]:
//  rcv_data[7]: (float) 単精度浮動小数点データの最上位バイト
//
//
// 送信データ :
//     sd_data[0] : 0x85 (MV価変更コマンドに対するレスポンス)
//     se_data[1] :  status ( 0x00:成功 , 0xff:書き込みデータ範囲外)
//     sd_data[2] : dummy 
//     sd_data[3] : dummy 
//
//
uint32_t	set_mv_data(void)
{
	uint32_t cnt;
	uint8_t status;
	
	float *pt;
	
	
	pt = (float *)&rcv_data[4];       // 書き込みデータのアドレス
	
	if ( ( *pt >= 0.0 ) && ( *pt <= 100.0 ) ) {  // 範囲内の場合 
		pid_mv = *pt;
		status = 0x00;
	 }
	 else {			// 範囲外
	   	status = 0xff;
	 }
	 
	 cnt = 4;			// 送信バイト数
	
	 sd_data[0] = 0x85;	 	// 制御モード変更コマンドに対するレスポンス	
	 sd_data[1] = status;
	 sd_data[2] = 0;			
	 sd_data[3] = 0;
	
	 return cnt;
}

//
// 温度とパラメータ(SV等)を読み出すコマンドのレスポンス作成
//  受信データ
//  rcv_data[0];　0x51 (温度とパラメータ　読み出しコマンド)
//  rcv_data[1]: dummy 0
//  rcv_data[2]: dummy 0 
//  rcv_data[3]: dummy 0
//
//   送信データ :
//     sd_data[0] : 0xd1 (コマンドに対するレスポンス)
//     se_data[1] : mode_stop_run ( 0:Stop, 1:Run)
//     sd_data[2] : mode_auto_manual (0:Auto, 1:Manual)
//     sd_data[3] : dummy 0
//     sd_data[4] : PV(ch1)のLowバイト  (10倍した値 例:300ならば30.0℃を表す)
//     sd_data[5] : PV(ch1)のHighバイト
//     sd_data[6] : ch2 のLowバイト   (10倍した値 例:300ならば30.0℃を表す)
//     sd_data[7] : ch2 のHighバイト   
//     sd_data[8] : ch3 のLowバイト   (10倍した値 例:300ならば30.0℃を表す)
//     sd_data[9] : ch3 のHighバイト   
//     sd_data[10] : ch4 のLowバイト   (10倍した値 例:300ならば30.0℃を表す)
//     sd_data[11] : ch4 のHighバイト 
//     sd_data[12] : cjt のLowバイト  (10倍した値 例:150ならば15.0℃を表す)
//     sd_data[13] : cjt のHighバイト 
//     sd_data[14] : MV のLowバイト   (10倍した値 例:100ならば10.0%を表す)
//     sd_data[15] : MV のHighバイト
//     sd_data[16] : SV のLowバイト   (10倍した値 例:300ならば30.0℃を表す)   
//     sd_data[17] : SV のHighバイト
//     sd_data[18] : P  のLowバイト ( 10倍した値 例:10ならば1%を表す)
//     sd_data[19] : P  のHighバイト      
//     sd_data[20] : I  のLowバイト
//     sd_data[21] : I  のHighバイト
//     sd_data[22] : D  のLowバイト
//     sd_data[23] : D  のHighバイト
//     sd_data[24] : Mr のLowバイト  (10倍した値 (例:100ならば10.0%を表す)
//     sd_data[25] : Mr のHighバイト
//     sd_data[26] : Hys のLowバイト  (10倍した値 (例:10ならば1.0℃を表す)
//     sd_data[27] : Hys のHighバイト    
//     sd_data[28] : H/C (0:Heater(逆動作), 1:Coller(正動作))
//     sd_data[29] : pid_type
//     sd_data[30] : pid_kp(ゲイン)  のLowバイト
//     sd_data[31] :  :        	       Highバイト
//     sd_data[32] : En(偏差)のLowバイト (10倍した値 (例:10ならば1.0℃を表す)
//     sd_data[33] :  :        Highバイト
//     sd_data[34] : P_MVの Lowバイト (10倍した値 (例:100ならば10.0%を表す)
//     sd_data[35] :   :    Highバイト
//     sd_data[36] : I_MVの Lowバイト (10倍した値 (例:100ならば10.0%を表す)
//     sd_data[37] :   :    Highバイト
//     sd_data[38] : D_MVの Lowバイト (10倍した値 (例:100ならば10.0%を表す)
//     sd_data[39] :   :    Highバイト
//

uint32_t	resp_temp_para_read(void)
{
	uint32_t cnt;
	
	int16_t   x_ch1, x_ch2, x_ch3, x_ch4, x_cjt;
	int16_t   x_sv, x_out, x_p, x_i, x_d, x_mr, x_hys;
	int16_t   x_kp, x_en, x_p_mv, x_i_mv, x_d_mv;
	
	sd_data[0] = 0xd1;	 	// コマンドに対するレスポンス	
	sd_data[1] = mode_stop_run;
	sd_data[2] = mode_auto_manual;			
	sd_data[3] = 0;
	
        x_ch1 = tc_temp[0] * 10.0;	// ch1の10倍
	sd_data[4] = x_ch1;		// Lowバイト側
	sd_data[5] = x_ch1 >> 8;	// Highバイト側
	
        x_ch2 = tc_temp[1] * 10.0;	// ch2の10倍
	sd_data[6] = x_ch2;		// Lowバイト側
	sd_data[7] = x_ch2 >> 8;	// Highバイト側

        x_ch3 = tc_temp[2] * 10.0;	// ch3の10倍
	sd_data[8] = x_ch3;		// Lowバイト側
	sd_data[9] = x_ch3 >> 8;	// Highバイト側
	
        x_ch4 = tc_temp[3] * 10.0;	// ch4の10倍
	sd_data[10] = x_ch4;		// Lowバイト側
	sd_data[11] = x_ch4 >> 8;	// Highバイト側
			
	x_cjt = cj_temp * 10.0;		// 基準接点温度の10倍
	sd_data[12] = x_cjt;
	sd_data[13] = x_cjt >> 8;
	
	x_out = pid_out * 10.0;		// 出力の10倍
	sd_data[14] = x_out;
	sd_data[15] = x_out >> 8;
	
	x_sv = pid_sv * 10.0;		// SVの10倍
	sd_data[16] = x_sv;
	sd_data[17] = x_sv >> 8;
	
	x_p = pid_p * 10.0;		// P値 の10倍
	sd_data[18] = x_p;
	sd_data[19] = x_p >> 8;
	
	x_i = pid_i;			// I値
	sd_data[20] = x_i;
	sd_data[21] = x_i >> 8;
	
	x_d = pid_d;			// D値
	sd_data[22] = x_d;
	sd_data[23] = x_d >> 8;
	
	x_mr = pid_mr * 10.0;		// MR値の10倍
	sd_data[24] = x_mr;
	sd_data[25] = x_mr >> 8;
	
	x_hys = pid_hys * 10.0;		// Hys値の10倍
	sd_data[26] = x_hys;
	sd_data[27] = x_hys >> 8;
	
	sd_data[28] = heat_cool;
	sd_data[29] = pid_type;

	x_kp = pid_kp * 1.0;		// Kp = 100/P
	sd_data[30] = x_kp;
	sd_data[31] = x_kp >> 8;
	
	x_en = pid_en * 10.0;		// En(偏差)
	sd_data[32] = x_en;
	sd_data[33] = x_en >> 8;
	
	x_p_mv = pid_p_mv * 10.0;	// P動作による出力 の 10倍
	sd_data[34] = x_p_mv;
	sd_data[35] = x_p_mv >> 8;
	
	x_i_mv = pid_i_mv * 10.0;	// I動作による出力 の 10倍
	sd_data[36] = x_i_mv;
	sd_data[37] = x_i_mv >> 8;
	
	x_d_mv = pid_d_mv * 10.0;	// D動作による出力 の 10倍
	sd_data[38] = x_d_mv;
	sd_data[39] = x_d_mv >> 8;
	
	cnt = 40;
	
	return cnt;
}



// 
// SCI11 初期設定
//  39.4kbps,   8bit-non parity-1stop
//  PCLKB = 32MHz
//  TXD1= P16,  RXD1 = P15
//
//   BRRの値(N):
//   SEMR ABCS (調歩同期基本クロックセレクトビット) = 0
//        BGDM (ボーレートジェネレータ倍速モードセレクトビット) = 0 の場合
//
//    N= (32 x 1000000/(64/2)xB)-1
//　　　　B: ボーレート bps
//        
// 例1)    B=38.4 kbpsとする。　32x38.4K = 1228.8 K	
//     32000 K / 1228.8 K= 26.04	
//     N= 26 - 1 = 25
//
// 38.4Kbps:
//     SCI12.BRR = 25
//     SCI12.SEMR.BIT.BGDM = 0
//
//
// 例2)  B=76.8 kbpsとする。　32x76.8K = 2457.6 K	
//     32000 K / 2457.6 K= 13.02	
//     N= 13 - 1 = 12
//
// 76.8Kbps:
//     SCI12.BRR = 12
//     SCI12.SEMR.BIT.BGDM = 0
//
//
// 例2)  B=153.6 kbpsとする。　32x153.6K = 4915.2 K	
//     32000 K / 4915.2 K= 6.5	
//     N= 6 - 1 = 5
//
// 76.8Kbps:
//     SCI12.BRR = 5
//     SCI12.SEMR.BIT.BGDM = 0
//

void initSCI_1(void)
{
	
	MPC.PWPR.BIT.B0WI = 0;   // マルチファンクションピンコントローラ　プロテクト解除
	MPC.PWPR.BIT.PFSWE = 1;  // PmnPFS ライトプロテクト解除
	
	MPC.P30PFS.BYTE = 0x0A;  // P30 = RXD1
	MPC.P26PFS.BYTE = 0x0A;  // P26 = TXD1
	
	
	MPC.PWPR.BYTE = 0x80;    //  PmnPFS ライトプロテクト 設定
			
	PORT3.PMR.BIT.B0 = 1;	// P30 周辺モジュールとして使用
	PORT2.PMR.BIT.B6 = 1;   // P26 周辺モジュールとして使用
		
	
	SCI1.SCR.BYTE = 0;	// 内蔵ボーレートジェネレータ、送受信禁止
	SCI1.SMR.BYTE = 0;	// PCLKB(=27MHz), 調歩同期,8bit,parity なし,1stop
	
	//SCI1.BRR = 25;		// 38.4K 
	
	SCI1.BRR = 12;			// 76.8K  
	SCI1.SEMR.BIT.BGDM = 0;     
	SCI1.SEMR.BIT.ABCS = 0;
	
	
	//SCI12.SEMR.BIT.BGDM = 1;        // 倍速モード で 153.6 Kbps 
	//SCI12.SEMR.BIT.ABCS = 0;
	
	//SCI12.SEMR.BIT.ABCS = 1;	// 基本クロック8サイクルの期間が1ビット期間の転送レート で、307.2 Kbps

	
	SCI1.SCR.BIT.TIE = 0;		// TXI割り込み要求を 禁止
	SCI1.SCR.BIT.RIE = 1;		// RXIおよびERI割り込み要求を 許可
	SCI1.SCR.BIT.TE = 0;		// シリアル送信動作を 禁止　（ここで TE=1にすると、一番最初の送信割り込みが発生しない)
	SCI1.SCR.BIT.RE = 1;		// シリアル受信動作を 許可
	
	SCI1.SCR.BIT.MPIE = 0;         // (調歩同期式モードで、SMR.MPビット= 1のとき有効)
	SCI1.SCR.BIT.TEIE = 0;         // TEI割り込み要求を禁止
	SCI1.SCR.BIT.CKE = 0;          // 内蔵ボーレートジェネレータ
	
	
	IPR(SCI1,RXI1) = 12;		// 受信 割り込みレベル = 12（15が最高レベル)
	IEN(SCI1,RXI1) = 1;		// 受信割り込み許可
	
	IPR(SCI1,TXI1) = 12;		// 送信 割り込みレベル = 12 （15が最高レベル)  
	IEN(SCI1,TXI1) = 1;		// 送信割り込み許可
	
	IPR(SCI1,TEI1) = 12;		// 送信完了 割り込みレベル = 12 （15が最高レベル)
	IEN(SCI1,TEI1) = 1;		// 送信完了割り込み許可
	
	rcv_cnt = 0;			// 受信バイト数の初期化
	
	
}


//  送信時と受信時のLED用　出力ポート設定
 void LED_comm_port_set(void)	
 {
	 				// 送信　表示用LED
	  LED_TX_PMR = 0;		// 汎用入出力ポート
	  LED_TX_PDR = 1;		// 出力ポートに指定
	  
	 				// 受信　表示用LED
	  LED_RX_PMR = 0;		// 汎用入出力ポート
	  LED_RX_PDR = 1;		// 出力ポートに指定
 }

