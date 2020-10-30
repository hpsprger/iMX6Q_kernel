#/bin/bash
export ARCH=arm
export CROSS_COMPILE=arm-linux-gnueabihf-
cp config_linux .config

echo make dtbs *********************************************************************************************************************************************************************************************
make dtbs

#make zImage -j8
echo make zImage --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
make zImage

#exit

echo  make modules ............................................................................................................................................................................................
make modules
echo all done ...... 
cp arch/arm/boot/dts/e9v2-sabresd.dtb ./
cp arch/arm/boot/dts/e9v3-sabresd.dtb ./
cp arch/arm/boot/dts/imx6q-sabresd.dtb ./
cp arch/arm/boot/dts/imx6q-corea-sabresd.dtb ./
cp arch/arm/boot/zImage ./




