
PARS=" -v -p"
PARS="${PARS} spl spl32.bin"
PARS="${PARS} write 0x00104000 __bl31.bin"
PARS="${PARS} write 0x4a000000 u-boot.bin"
#PARS="${PARS} write 0x40080000 kernel.bin"
#PARS="${PARS} write 0x4fa00000 dtb"
#PARS="${PARS} write 0x4fc00000 boot.scr"
#PARS="${PARS} write 0x4fe00000 ramfs.cpio"
PARS="${PARS} reset64 0x00104000"

#setenv bootargs 'earlyprintk console=ttyS0,115200 root=/dev/ram0 rootwait'
#booti 0x40080000 - 0x4fa00000

make -C ../arm-trusted-firmware && cp ../arm-trusted-firmware/build/sun50i_h6/debug/bl31.bin ./__bl31.bin

sunxi-fel ${PARS} $*
