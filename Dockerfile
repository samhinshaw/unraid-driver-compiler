# FROM linux-build-base
FROM aclemons/slackware:15.0
# FROM andy5995/slackware-build-essential:15.0

# ? can we remove kernel-headers and kernel source from this list?
RUN slackpkg install make gc guile libunistring libffi gcc libffi mpfr glibc binutils kernel-headers kernel-source bison flex m4 bc

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

RUN make oldconfig

RUN make -j5 bzImage
RUN make -j5
RUN make -j5 modules
RUN make modules_install

# Driver Compilation
RUN mkdir /drivers
COPY ./drivers /drivers
WORKDIR /drivers/Linux

# RUN ln -s /usr/include/asm-x86_64/ /usr/include/asm

# RUN mkdir -p /lib/modules/6.5.11-linuxkit
# RUN ln -s /usr/include/linux /lib/modules/6.5.11-linuxkit

# RUN SUBDIRS=/linux-kernel/include/linux make oldconfig
# RUN make oldconfig
# RUN ln -s /lib/modules/6.5.11-linuxkit
# RUN make