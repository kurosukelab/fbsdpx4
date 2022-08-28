# fbsdpx4 - FreeBSD driver for PLEX PX-W3U4/Q3U4/W3PE4/Q3PE4 ISDB-T/S receivers

FreeNASで録画サーバーを運用したいためにPLEX PX-W3U4/Q3U4/W3PE4/Q3PE4用の非公式版Linuxドライバをfbsdpt3を参考にFreeBSDに移植してみました。  
[非公式版Linuxドライバ](https://github.com/nns779/px4_drv)のv0.2.1a(90e0a4b30b812e7e5fff4483144f165de8914157)をベースにしています。  
Kernel Panicが発生する可能性があります。  

Drop時、Syncが外れずに、TS Packetが連続的に消失していることから、usbd_transfer_submitが間に合っていないと推測。    
FreebsdのUSB driverの構成では、
callback関数内でDMA bufferの処理をしないと、次の転送ができないため、copyのみでcallback関数を終了し、別ThreadでTS Packetを処理するように変更しましたが、、、  
PX-W3U4での結果は
- Freebsd 13.0-Release CPU AMD FX-8800P 
  - Dropする(数Drop/30minutes)
- TrueNAS-13.0-U1.1(Freebsd 13.1-Release) CPU Pentium G4600
  - Dropしていない(上記と比べると安定している)

copyのみでsubmitが間に合わないのは、信じ難いが、残りのdeviceの制御等で、処理時間が遅延している可能性が高い(処理能力の可能性が高い)。  
Freebsdは、DMAの構成上callbackがネックになり、linuxに比べて、CPUの処理能力が必要。  
(CPU AMD FX-8800P+Linux非公式driverはDropしていない)  
ext_bufferのoptionを利用してDMA bufferを切り替えれば、改善する可能はあるが、うまく動作していない。  
TrueNAS Scaleとか、Dockerとか状況をみると、ext_bufferで動作させるモチベーションはないです。

TrueNAS Coreのjail環境でdriverをcompile後、firmwareとdriverをホスト側にinstallして、jail環境のmirakurun+epgstation+mariadbで動作中  
- epgstationのdrop checkを有効にしてみたところ、ErrorとDropが発生していることある
  - 録画終了後に、tsselectでdrop checkを実施するとerrorもdropも未発生。
  - 同一時間、同一チャンネルの非公式linux版で録画したファイルにもnode-atibtsのsampleを利用して、drop checkをすると同じように検出しているので、driverの問題ではなさそう。

## 対応デバイス

- PX-W3U4
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
デバイスファイルを書き換えた[recpt1](https://github.com/kurosukelab/recpt1) (branch fbsdpx4)が利用可能です。  
