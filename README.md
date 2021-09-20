# fbsdpx4 - FreeBSD driver for PLEX PX-W3U4/Q3U4/W3PE4/Q3PE4 ISDB-T/S receivers

FreeNASで録画サーバーを運用したいためにPLEX PX-W3U4/Q3U4/W3PE4/Q3PE4用の非公式版Linuxドライバをfbsdpt3を参考にFreeBSDに移植してみました。  
[非公式版Linuxドライバ](https://github.com/nns779/px4_drv)のv0.2.1a(90e0a4b30b812e7e5fff4483144f165de8914157)をベースにしています。  

Kernel Panicが発生する可能性があります。
Freebsdのmtx(mutex)は,一定時間以上SleepしているととKernel Panic(Sleeping Thread)が発生するので、sx_xlockに置き換え。 

1channelだとdropが発生しないけれど、2channel目の制御することでdropが発生している感じ。 
continuty_counterをcheckしていると、周波数を設定後や、他のchannelをclose時に不連続が発生する。
周波数設定後については、ts packetをdiscardすることで、回避可能だが、他のchannelの制御信号が別のchannelに影響を与える理由が不明。 
また、channelの制御未実施のTimingで、decode ngも発生することがある。(壊れた？) 


## 対応デバイス

- PX-Q3PE4

## インストール

このドライバを使用する前に、ファームウェアを公式ドライバより抽出しインストールを行う必要があります。

### 1. ファームウェアの抽出とインストール

	$ cd fwtool
	$ make
	$ cd ../fwimage
	$ wget http://plex-net.co.jp/plex/pxw3u4/pxw3u4_BDA_ver1x64.zip -O pxw3u4_BDA_ver1x64.zip
	$ unzip -oj pxw3u4_BDA_ver1x64.zip pxw3u4_BDA_ver1x64/PXW3U4.sys
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
