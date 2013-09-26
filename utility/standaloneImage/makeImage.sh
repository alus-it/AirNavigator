#!/bin/sh
if [ -e ./ttsystem ] ; then
	rm ttsystem
fi
cd image
sudo find * | sort | cpio --create --format=newc --quiet | gzip -9 > ../ttsystem.0
cd ..
./mkttimage ttsystem.0 ttsystem.1 > ttsystem
rm ttsystem.0
