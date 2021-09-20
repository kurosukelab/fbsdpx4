KMOD= it930x_fw
FIRMWS= it930x-firmware.bin:it930x_fw

it930x-firmware.bin:PXW3U4.sys
	-ln -s ../fwtool/fwinfo.tsv .
	../fwtool/fwtool PXW3U4.sys it930x-firmware.bin

#PXW3U4.sys:pxw3u4_BDA_ver1x64.zip
#	unzip -oj  pxw3u4_BDA_ver1x64.zip pxw3u4_BDA_ver1x64/PXW3U4.sys

#pxw3u4_BDA_ver1x64.zip:
#	wget http://plex-net.co.jp/plex/pxw3u4/pxw3u4_BDA_ver1x64.zip -O pxw3u4_BDA_ver1x64.zip

.include <bsd.kmod.mk>

