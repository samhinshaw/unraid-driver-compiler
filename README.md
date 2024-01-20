This Docker container can be used to compile drivers for unRAID. 

## Instructions

1. Copy unRAID's kernel patches from your unRAID server, found at `/usr/src/<kernel version>` into `patches/`.
1. Copy your driver source into `drivers`.
1. Then build the container
    ```sh
    docker build .
    ```
1. `<Instructions TBD>` Run the container? Extract?

## References

Big thanks to the instructions in this thread: https://forums.unraid.net/topic/82625-kernelcustom-kernel-build-with-treaks-20200307-v683558419108naviveganfsv4r8125zen2/

```
1. download kernel sources from kernel.org, notice that should be same or related version of unraid(like 4.19.94 which 6.8.2 used)
2. unarchive the kernel source zip, like kernel-4.19.94/
3. copy the patches and .config (important!) from unraid server which located at /usr/src (like /usr/src/linux-4.19.94-Unraid/) to step 2 source directory
4. (Optional) copy custom patches like navi patches or others to source directory too
5. apply patches: find . -type f -iname '*.patch' -print0|xargs -n1 -0 patch -p 1 -i
6. use old config : make oldconfig
7. compile the kernel and modules: make -j5 bzImage; make -j5; make -j5 modules
8. installing the modules, then you can find the module directory in /lib/modules: sudo make modules_install
9. Copy the kernel image: cp sources/linux-4.19.94/arch/x86_64/boot/bzImage releases/6.8.2-4.19.94/bzimage
10. (Optional) ThirdParty modules compiling (like nic r8125 outtree driver):
    1. enter the thirdparty driver directory
    2. compile the module: make -C /lib/modules/4.19.94-Unraid//build M=(pwd)/src modules
    3. install the module to direcotry: sudo make -C /lib/modules/4.19.94-Unraid/build M=(pwd)/src INSTALL_MOD_DIR=kernel/drivers/net/ethernet/realtek/ modules_install
    4. you can check whether the module exists in /lib/modules/4.19.94-Unraid/kernel/drivers/net/ethernet/realtek/
11. archive the modules to bzimage: mksquashfs /lib/modules/4.19.94-Unraid/ releases/4.19.94/bzmodules -keep-as-directory -noappend
12. Then you get the bzimage and bzmodules, copy it to unraid server: /boot/

```