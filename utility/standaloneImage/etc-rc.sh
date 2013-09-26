#!/bin/sh

fstype="vfat"
mntopt="rw,exec,noatime,nodev,nosuid"
devbase="/dev"
gl_baudrate="115200"


initial_mount() {
	mount -n -t proc none /proc
	mount -n -t sysfs none /sys
	# Remount root fs as R/W
	mount -o remount,rw /
	
	# Mount Ramfs in dir likely to be written
	mount -n -t tmpfs  none /dev/shm      
	mount -n -t tmpfs  none /tmp          
	mount -n -t tmpfs  none /var/run     
	#mount -n -t usbfs none /proc/bus/usb 
	mount -n -t devpts none /dev/pts     
	
	#Mount flash where it is used to be found on TT...
	mount -n -t ngffs mtd0 /mnt/flash
}

do_mount_storage() {

    # Start out assuming we don't have to reboot...
    echo 0 > /proc/barcelona/rebootonsdremoval
    cnt=""
    for part in ${storagedevs}; do
      #blkdev = ${part} without the pX for partition
      blkdev=${part%p?}
      if test "${#cnt}" = "0"; then
        # first mount, is bootdisk
	rm -f ${devbase}/sdcard
        ln -s ${devbase}/${part} ${devbase}/sdcard

	mntpt="/mnt/sdcard"

        if test -e /sys/block/${blkdev}/device/scr; then
          # We boot from SD, tell the kernel (for sd removal while sleeping)
          echo 1 > /proc/barcelona/rebootonsdremoval
        fi
      elif test "${#cnt}" = "1"; then
        # second mount, must be movinand
        mntpt="/mnt/movinand"
      else
        mntpt="/mnt/storage${cnt}"
      fi

      # Now check if storage that we're mounting is a removable device, and
      # if so, mount as 'sync'.
      if test -e /sys/block/${blkdev}/device/scr; then
        curmntopt="${mntopt},sync"
    else
        curmntopt="${mntopt}"
      fi

      # Now it is safe to do the mounting...
      mkdir -p ${mntpt}
      if mount -t ${fstype} -o ${curmntopt} ${devbase}/${part} ${mntpt}  >/dev/null 2>&1; then
        echo "* Storage ${part} mounted on ${mntpt}"
        cnt="${cnt}1"      
      fi 
    done

    if test "${cnt}" != ""; then
      mnt_ok="y"
    fi
}

vars_init() {
	hw_bootdevice=`cat /proc/barcelona/bootdevice`
	hw_gpsdev=`cat /proc/barcelona/gpsdev`
	hw_gpstype=`cat /proc/barcelona/gpstype`
	hw_gldetected=`cat /proc/barcelona/gldetected`
	hw_gpsephemeris=`cat /proc/barcelona/gpsephemeris`
	hw_resetstate=`cat /proc/barcelona/resetstate`
}

mount_storage() {
  echo "* Detecting main storage partitions"

  partitions=`sed -r "s/( )+/ /g" /proc/partitions | cut -s -d\  -f 5`
  storagedevs=""

    for part in ${partitions}; do
    #blkdev = ${part} without the pX for partition
    blkdev=${part%p?}
    if test "${part#mmcblk}" != "${part}"; then
      # Found SD/MMC, this could be our boot disk!
      if test "${hw_bootdevice}" = ""; then
        # Bootloader didn't give a CID, so assume this is the boot device
        storagedevs="${part} ${storagedevs}"
      elif test "`cat /sys/block/${blkdev}/device/cid`" = "${hw_bootdevice}"; then
        # this was the boot device...
        storagedevs="${part} ${storagedevs}"
      else
        # just storage device
        storagedevs="${storagedevs} ${part}"
        fi
    elif test "${part#hd}" != "${part}"; then
      # Found Harddisk, so must be boot disk (we ship no devices with HDD & other storage)
      storagedevs="${part} ${storagedevs}"
    elif test "${part#mtd}" != "${part}"; then
      # Found Flash chip, we don't mount that here
      storagedevs="${storagedevs}"
    else
      echo Ignoring unknown storage device ${part}...
        fi
  done

  # Trim spaces from start/end
  #NOTE: the space after # has intentionally been left blank ;)
  storagedevs="${storagedevs# }"
  storagedevs="${storagedevs% }"

  if test "${storagedevs}" = ""; then
    echo "* No storage detected"
    unset storagedevs
  else
    mnt_ok="n"
    do_mount_storage
  fi
}

gps_init() {
  if test -n "${hw_gpsdev}"; then
    ln -sf "${hw_gpsdev}" /dev/gpsdata
  fi
  if test "${hw_gldetected}" = "0"; then
    if test "${hw_gpsephemeris}" = "1"; then
      /bin/sirfreset version printver outsirf verbose &
    else
      /bin/sirfreset version printver outnmea verbose &
    fi
  fi
}

start_clm() {
  if test "${hw_gpsephemeris}" = "1"; then
    if test -f /var/run/sirfreset.pid; then
      wait $(cat /var/run/sirfreset.pid)
    fi
    if test -f /mnt/sdcard/clmdata; then
      cp /mnt/sdcard/clmdata /var/run/
    fi
    clmapp &
  fi
}

start_gl() {
  if [ -e /mnt/sdcard/noqfixdel ]; then
	echo GL Quickfix delete after reset disabled >>$j
  else
	if test "${hw_resetstate}" = "1"; then
		echo "User pressed reset button" | /etc/gllog.sh
		if test -f /mnt/flash/extra_settings/0001.dat; then
			echo "Erasing GL NVRAM ..." | /etc/gllog.sh
			rm /mnt/flash/extra_settings/0001.dat
		fi
		if test -f /mnt/sdcard/ephem/lto.dat; then
			echo "Erasing LTO ..." | /etc/gllog.sh
			rm /mnt/sdcard/ephem/lto.dat
		fi
	fi
  fi
  /etc/rc.gltt start ${gl_baudrate}
}

start_gps() {
  if test "${hw_gldetected}" = "1"; then
    start_gl
  else
    start_clm
  fi
}


# *** Startup, one time only ***

# Initial mount of filesystem
initial_mount 

# Touchscreen calibration
calib

# Note : This sleep is Absolutly necessary in order to let mmc driver enough time to probe all cards !
sleep 3

# Initialize system variables
vars_init

#Generic mount storage device (sdcards and hard disk) 
mount_storage

#GPS initialization
gps_init


# *** Main loop ***
while true; do
	start_gps
	if [ -e /mnt/sdcard/AirNavigator ] ; then
	  cd /mnt/sdcard/AirNavigator
	  ./AirNavigator.sh            
	else 
	  if [ -e /mnt/movinand/AirNavigator ] ; then
	    cd /mnt/movinand/AirNavigator
	    ./AirNavigator.sh
          else 
            sleep 5
	  fi 
	fi
	/etc/rc.gltt stop	
	/bin/turn_off
done
# EOF

