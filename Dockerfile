# FROM linux-build-base
FROM aclemons/slackware:15.0
# FROM andy5995/slackware-build-essential:15.0

# ? can we remove kernel-headers and kernel source from this list?
# RUN slackpkg install make gc guile libunistring libffi gcc libffi mpfr glibc binutils bison flex m4 bc

# install all development tool packages
RUN slackpkg install d elfutils cpio rsync lz4 xxHash kmod

# ? Can we add an ARG for kernel version?

# pull down kernel source
RUN mkdir /linux-kernel
WORKDIR /linux-kernel
# download the kernel with wget and pipe the ouput directly to tar. 
# the strip components command means the kernel won't be extracted into a subfolder
RUN wget --no-check-certificate -c https://mirrors.edge.kernel.org/pub/linux/kernel/v6.x/linux-6.1.64.tar.gz -O - | tar -xz --strip-components=1

# copy over patches to the kernel
COPY ./patches /linux-kernel

# Apply Patches
RUN find . -type f -iname '*.patch' -print0|xargs -n1 -0 patch -p 1 -i

# Apply the patch necessary for our driver.
COPY 8250_pci_patched.c /usr/src/linux-5.15.145/drivers/tty/serial/8250/8250_pci.c
# Copy the custom driver I patched
COPY 99xx_patched.c /drivers/Linux

# configure
RUN make oldconfig

RUN make headers_install

# buld the kernel!
RUN make -j5 bzImage
RUN make -j5
RUN make -j5 modules
RUN make modules_install

RUN slackpkg install 

# Driver Compilation
RUN mkdir /drivers
COPY ./drivers /drivers
WORKDIR /drivers/Linux

# RUN make
# or is it
# RUN make -C =/lib/modules/6.1.64-Unraid/build/ M=/drivers/Linux modules 
