#!/bin/sh

if [ "" = "$1" ];then

echo "Please set the dev."
exit

else

echo "Dev name = $1"

fi

if [ "" = "$2" ];then

echo "Please set the image name."
exit

else

echo "Image name = $2"

fi

if [ "u-boot.img" = "$2" ];then

echo "Start flashing u-boot.img"
sudo dd if=../u-boot-rockchip/u-boot-sd.img of=$1 conv=sync seek=64 
echo "Done."
exit

elif [ "parameter.img" = "$2" ];then

echo "Start flashing parameter.img"
sudo dd if=../parameter/parameter.img of=$1 conv=sync seek=$((0x2000))
echo "Done."
exit

elif [ "boot.img" = "$2" ];then

echo "Start flashing boot.img"
sudo dd if=boot.img of=$1 conv=sync seek=$((0x2000+0x2000))
echo "Done."
exit

elif [ "rootfs.ext4" = "$2" ];then

echo "Start flashing rootfs.ext4"
sudo dd if=../rootfs/rootfs.ext4 of=$1'1' conv=sync
echo "Done."
exit

elif [ "all" = "$2" ];then

echo "Format the SD card"
sudo dd if=/dev/zero of=$1 bs=1M count=1
sudo fdisk $1  << EOF
n
p
1
65536

w
EOF

echo "Start flashing u-boot.img"
sudo dd if=../u-boot-rockchip/u-boot-sd.img of=$1 conv=sync seek=64 
echo "Start flashing parameter.img"
sudo dd if=../parameter/parameter.img of=$1 conv=sync seek=$((0x2000))
echo "Start flashing boot.img"
sudo dd if=boot.img of=$1 conv=sync seek=$((0x2000+0x2000))
echo "Start flashing rootfs.ext4"
sudo dd if=../rootfs/rootfs-debian.ext4 of=$1'1' conv=sync
echo "Done."
exit

else

echo "parameter is not allowed."

fi



