#!/bin/sh

build_path="../build_linux"
module_install="../out"
firmware_install="../out"
image_filename="$build_path/arch/arm/boot/uImage"
target_filename="uImage.imx6"

if [ -f .config ]; then
	echo ".....mrproper"
	make mrproper
fi

if [ ! -d $build_path ]; then
	mkdir $build_path
	chmod 777 $build_path
fi


if [ ! -f $build_path/.config ]; then
	echo ".....imx6q falinux defconfig"	
	ARCH=arm make arm=ARM CROSS_COMPILE=arm-linux-gnueabihf- O=$build_path distclean 
	ARCH=arm make arm=ARM CROSS_COMPILE=arm-linux-gnueabihf- O=$build_path imx6q_falinux_defconfig
fi


if [ "$1" = "" ]; then
	ARCH=arm make arm=ARM CROSS_COMPILE=arm-linux-gnueabihf- O=$build_path -j4 uImage LOADADDR=0x10008000
else
        ARCH=arm make arm=ARM CROSS_COMPILE=arm-linux-gnueabihf- O=$build_path $1 $2 $3
fi

if [ -f $image_filename ]; then
   echo "copy from $image_filename to /tftpboot/$target_filename"
   cp  $image_filename /tftpboot/$target_filename
   chmod 777 /tftpboot/$target_filename
fi

