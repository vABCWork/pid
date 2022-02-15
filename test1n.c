#include	<machine.h>
#include	 "iodefine.h"
#include	 "misratypes.h"
#include	"debug_port.h"
#include	 "timer.h"
#include	"dsad.h"
#include	"sci.h"
#include 	"dma.h"
#include        "pid.h"
#include         "thermocouple.h"

void clear_module_stop(void);
void  delay_msec( uint32_t  n_msec );
static void debug_port_ini(void);

uint8_t flg_control_interval; 

void main(void)
{
	debug_port_ini();	// P31を出力ポートに設定 (デバック用)
	
	clear_module_stop();	//  モジュールストップの解除
 
	Timer10msec_Set();	//  10msec タイマ(CMT0)設定
	Timer10msec_Start();	//  10msec タイマカウント開始
	
	afe_ini();		// AFE(アナログフロントエンド)設定
	
	dsad0_ini();		// DASD0の設定　(熱電対用 4チャンネル)
	dsad1_ini();            // DASD1の設定 (基準接点補償 RTD 100 ohm)
	
	ad_index = 0;		// 各チャンネルのデータ格納位置の初期化
	ad1_index = 0;
	
	PWM_Init_Reg();		// PWM用の設定 周期 200msec(=制御周期)
	PWM_Port_Set();		// Heat用,Cool用出力ポートの設定
	
	DMA1_ini();           	// PCへのシリアルデータ送信用のDMA処理　初期設定	
	initSCI_1();		// SCI 1初期設定 76.8K
	LED_comm_port_set();	// 送信時のLEDポート設定
	
	heat_cool = 0;	        //　逆動作(加熱制御)
	
	while(1) {
	   if ( flg_100msec_interval == 1 ) {  // 100msec経過
	      
	      flg_100msec_interval = 0;		// 100msec経過フラグのクリア
	      
	      if ( flg_control_interval  == 0 ) { // 非制御タイミング(=A/D変換開始)	
		
		  dsad0_start();		// DSAD0開始
		  dsad1_start();		// DSAD1開始
		  
		  DEBUG_0_PODR = 1;		// P31 = ON
		  
		  dsad0_scan_over = 0;
		  dsad1_scan_over = 0;

		 flg_control_interval = 1;	// 次回は制御タイミング
		 
	      }
	      else  {	// 制御タイミング
                 	  
		while ( (dsad0_scan_over == 0 )||( dsad1_scan_over == 0) ) {  // 66.6 msec, dsad0 (ch0～ch3) とdsad1(ch0)のスキャン完了確認待ち
		}    

		dsad0_stop();		 // DSAD0 停止 (dsad0とdsad1の停止　1.3 usec)
		dsad1_stop();		 // DSAD1 停止
		
		Cal_ad_avg();		   // dsad0 各チャンネルの平均値を得る
			     
	   	Cal_ad1_avg();		   // dsad1 各チャンネルの平均値を得る
		

		tc_temp_cal();		  // 温度計算
		
		pid_pv = tc_temp[0];	  // ch1を PV値とする。
		
		control();		  // 制御処理
		
		flg_control_interval = 0;		// 次回は非制御タイミング
		
		
		if ( rcv_over == 1 ) {		// 通信処理 コマンド受信の場合
  		    comm_cmd();			// レスポンス作成、送信
	   	    rcv_over = 0;		// コマンド受信フラグのクリア
		    rcv_cnt  = 0;		//  受信バイト数の初期
		    LED_RX_PODR = 0;		// 受信 LEDの消灯
	         }
		
	     }
	   
	      
	
	      
	      
	    } // 100msec経過
	   
	}

}



// モジュールストップの解除
//   CMTユニット0(CMT0, CMT1)
//   マルチファンクションタイマパルスユニット（MTU0 ～ MTU5）
//   アナログフロントエンド(AFE)
//   24ビットΔ-Σ A/D コンバータ(DSAD0) ユニット0
//   24ビットΔ-Σ A/D コンバータ(DSAD1) ユニット1
//   DMA コントローラ(DMACA)
//  シリアルコミュニケーションインタフェース(SCI)
//
void clear_module_stop(void)
{
	SYSTEM.PRCR.WORD = 0xA50F;	// クロック発生、消費電力低減機能関連レジスタの書き込み許可	
	
	MSTP(CMT0) = 0;			// コンペアマッチタイマ(CMT) ユニット0(CMT0, CMT1) モジュールストップの解除
	MSTP(MTU) = 0;			// マルチファンクションタイマパルスユニット モジュールストップの解除
	MSTP(AFE) = 0;			// アナログフロントエンド(AFE) モジュールストップの解除
	MSTP(DSAD0) = 0;		// 24 ビットΔ-Σ A/D コンバータ(DSAD0) ユニット0 モジュールストップの解除
	MSTP(DSAD1) = 0;		//             :                        ユニット1      
	MSTP(DMAC) = 0;                //  DMA モジュールストップ解除
	MSTP(SCI1) = 0;	        	// SCI1 モジュールストップの解除
	
	SYSTEM.PRCR.WORD = 0xA500;	// クロック発生、消費電力低減機能関連レジスタ書き込み禁止
}




//   N [msec] 待つ関数 (ICLK = 32MHz)
//   命令語は、 RX V2 アーキテクチャ
//    コンパイル時　最適化レベル = 2   (-optimize = 2 )
//   n_msec=  1:  1msec
//      

#pragma instalign4 delay_msec

void  delay_msec( unsigned long  n_msec )
{
	unsigned long delay_cnt;

	while( n_msec-- ) {

	   delay_cnt = 10656;
		
	   while(delay_cnt --) 
	   { 			 
	   }

	}
}



//   デバックポートの設定 
//   (debug_port.h)

static void debug_port_ini(void)
{	
        DEBUG_0_PMR = 0;    //  P31 汎用入出力ポート
	DEBUG_0_PDR = 1;    //  出力ポートに指定
	
}
