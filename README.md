AirNavigator
============

Welcome to AirNavigator repository!  
This is an open source HSI navigation application for TomTom devices.  
For more informations about this project: http://www.alus.it/airnavigator  

This is the README file for developers.  
The README file for final users is located here: ./release/README.txt  

Contributors are warmly welcome!  


Preparing the development environment
=====================================

In order to compile C programs to ARM executables through a Linux PC, follow the steps described here. Alternatively, you may consider building your own toolchain.

1. Start bash and verify that a C source program can be correctly compiled to a Linux executable.
2. Download: http://www.alus.it/airnavigator/toolchain_redhat_gcc-3.3.4_glibc-2.3.2-20060131a.tar.gz
3. Unpack the file to /usr/local/cross
4. Perform then the following links:  
  `$ cd /usr/local/cross/gcc-3.3.4_glibc-2.3.2/arm-linux/sys-root/usr/include/asm`  
  `$ sudo ln -s arch-s3c2410 arch`  
  `$ cd /usr/local/cross/gcc-3.3.4_glibc-2.3.2/arm-linux/sys-root/usr/include/asm`  
  `$ sudo ln -s proc-armv proc`  
  This is because the TTGO processor is a SAMSUNG ELECTRONICS S3C2410 (Arm920T), 32-bit architecture (so armv, while armo is for the old 26-bit ones).  
5. To cross-compile on 64 bit make sure to have installed the GNU C 32-bit shared libraries for AMD64  
  `$ sudo apt-get install libc6-i386`  
6. Optionally export the PATH:  
  `export PATH=$PATH:/usr/local/cross/gcc-3.3.4_glibc-2.3.2/bin`  
7. The cross compiler is now ready!


Compiling AirNavigator
======================

Having the toolchain installed you can compile AirNavigator, go to his folder and type:

$ make all


With the command:

$ make zip

The make process will produce automatically the two distributions zip files.


And with:

$ make upload

The make process will copy just the AirNavigator executable to your TomTom connected via USB.
In this case the TomTom disk should be mounted under: /media/INTERNAL/



CONTACTS
========

Author: Alberto Realis-Luc
Web: http://www.alus.it/airnavigator/
E-mail: airnavigator@alus.it
