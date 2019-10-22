# fbsdpx4 - FreeBSD driver for PLEX PX-W3U4/Q3U4/W3PE4/Q3PE4 ISDB-T/S receivers

FreeNASで録画サーバーを運用したいためにPLEX PX-W3U4/Q3U4/W3PE4/Q3PE4用の非公式版Linuxドライバをfbsdpt3を参考にFreeBSDに移植してみました。  
[非公式版Linuxドライバ](https://github.com/nns779/px4_drv)のv0.2.1aをベースにしています。  

Kernel Panicが発生する可能性があります。  
BSがないので、PX-W3U4/W3PE4相当のデバイスに地上波2channelの多重でMirakurun/Chinachu動作確認中。  
~~10msのpauseではmtx_lockが奪えないみたいで、~~  
~~channel変更中にもう一方のdevがcloseされるとsuできない状態に...~~  
tc90522_is_signal_locked_t関数を10msで実施するUSBの通信が停止するので100msに変更。  
mutexの場合、USB制御中にDeadlock判定になるので、sxlockを追加。  
1channelだとdropが発生しないけれど、2channel目の制御することでdropが発生している感じ。  
また、Mirakurun動作中にデバイスロードするとKernel Panic発生します。  

## 対応デバイス

- PX-Q3PE4

## インストール

このドライバを使用する前に、ファームウェアを公式ドライバより抽出しインストールを行う必要があります。

### 1. ファームウェアの抽出とインストール

	$ cd fwtool
	$ make
	$ cd ../fwimage
	$ make
	$ sudo make install
	$ cd ../

### 2. ドライバのインストール

	$ cd driver
	$ make
	$ sudo make install
	
### 3. ドライバのロード
	$ sudo kldload px4.ko

### 3. 確認

インストールに成功した状態でデバイスが接続されると、`/dev/` 以下に `px4video*` という名前のデバイスファイルが作成されます。

チューナーは、px4video0.0s,px4video0.1s,px4video0.2t,px4video0.3tの順に ISDB-S, ISDB-S, ISDB-T, ISDB-T が割り当てられます。  


## 受信方法

recpt1を使用してTSデータの受信ができます。  
[努力したＷｉｋｉ](https://hgotoh.jp/wiki/doku.php/documents/freebsd/ptx/ptx-001)のrecpt1に、
デバイスファイルを書き換えた[recpt1](https://github.com/kurosukelab/recpt1)が利用可能です。  
