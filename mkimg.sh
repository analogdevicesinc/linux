make -j8 zImage dtbs ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf-

cat arch/arm/boot/zImage arch/arm/boot/dts/rk3188-radxarock.dtb > zImage-dtb

mkbootimg --kernel zImage-dtb --ramdisk ../rk30_linux_initramfs/fakeramdisk.gz -o boot.img

