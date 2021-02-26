# freebsd-src-mrsas
===================================================
LSI MegaRAID Drivers and Software for FreeBSD
===================================================

 o  FreeBSD System Administration Information
 o  Installing LSI Drivers and Software under FreeBSD


===================================================
FreeBSD System Administration Information
===================================================

   The latest FreeBSD System Administration information can be found here:
      http://www.freebsd.org/doc/en_US.ISO8859-1/books/handbook
      http://www.freebsd.org/doc/en_US.ISO8859-1/books/handbook/disks-adding.html

   Online man pages can be searched here:
      http://www.freebsd.org/cgi/man.cgi

===================================================
Installing LSI Drivers and Software under FreeBSD
===================================================

   This section provides detailed instructions for installing the LSI driver
   under a FreeBSD operating system.

   The following topics are included in this section:
   o  Driver Installation for FreeBSD
   o  Updating Drivers under FreeBSD


===============================
===============================
Driver Installation for FreeBSD
===============================
===============================

Note: 1) If your FreeBSD system does not have the mpt driver issue, then before
         installing the LSI driver, you should already have installed your
         MegaRAID controller in the system. Consult the installation guide that
         came with your controller for how to do this.
         You can download the installation guide from:
                  http://www.lsi.com/channel/ChannelDownloads.

      2) If your FreeBSD system has the mpt driver issue, then you must disable
         the mpt driver and recompile the kernel first.  If you have a MegaRAID
         controller installed, you must power down and remove the controller
         from your system and recompile the kernel.  If you leave the controller
         in your system the mpt driver will claim the device ID and go into a
         soft reset continuously.  Your system will not come out of boot.


This section provides details about how to install the driver for your
MegaRAID controller under FreeBSD.

   o If the VDs you have created are your boot device, you install the driver
     for the controller as you install FreeBSD.

   o If the operating system is already installed on a unit connected to another
     controller or to the motherboard, you start FreeBSD and then install the
     driver.

This section includes these topics:

   o "Obtaining LSI FreeBSD Drivers"

   o "Creating a FreeBSD Driver Diskette"

   o "Installing FreeBSD on Storage Managed by a MegaRAID Controller
	(Primary Storage)"

   o "Installing the Kernel Driver on a FreeBSD System that Boots from
      a Device NOT Managed by a MegaRAID Controller(Secondary Storage)"


-------------------------------
Obtaining LSI FreeBSD Drivers
-------------------------------

LSI drivers can be compiled from source files into the kernel as built-in
drivers or can be modules that are loaded by the operating system. Both
source files and modules are available from LSI, but modules with
current controller drivers are available for FreeBSD 7.4, 8.2, and 9.0.

You can obtain the MegaRAID controller driver for FreeBSD from one of
these two sources:

   o LSI software CD-ROM
     This CD includes:
        Compiled and tested kernel driver modules for FreeBSD 7.4,
        located here:
           32-bit: components/7.4/x86
           64-bit: components/7.4/x86_64

        Compiled and tested kernel driver modules for FreeBSD 8.2,
        located here:
           32-bit: components/8.2/x86
           64-bit: components/8.2/x86_64

        Compiled and tested kernel driver modules for FreeBSD 9.0,
        located here:
           32-bit: components/9.0/x86
           64-bit: components/9.0/x86_64

        Driver source files for FreeBSD 7.x, 8.x, and 9.x
        located here:

        For mfi driver,
           components/src/mfi.tgz

        For mrsas driver,
           components/src/mrsas.tgz

   o LSI web site. You can download the latest compiled and tested driver
     modules and driver source files for FreeBSD from the LSI web site at
     http://www.lsi.com/support/Pages/download-search.aspx.


----------------------------------
Creating a FreeBSD Driver Diskette
----------------------------------

You need a diskette drive if you are going to be installing FreeBSD on a VD
or drive managed by a MegaRAID controller card which becomes the boot
unit for which your version of FreeBSD does not have a built-in driver.

To create a driver diskette

     1 Insert a blank diskette and the LSI software CD into a FreeBSD
       installed system.

     2 For FreeBSD 7.4 Navigate to: components/7.4
       For FreeBSD 8.2 Navigate to: components/8.2
       For FreeBSD 9.0 Navigate to: components/9.0

     3 Copy the appropriate driver to the diskette, i.e., driver from x86
       folder if you are installing a 32-bit FreeBSD, x86_64 folder if you
       are installing 64-bit FreeBSD.  You need the driver source from
       components/src/driver_name.tgz if you are going to update the kernel 
       with a new driver.

       For example:

       For mfi driver, the driver source is in components/src/mfi.tgz file.

       For mrsas driver, the driver source is in components/src/mrsas.tgz.


----------------------------------------------------------------
Installing FreeBSD on Storage Managed by a MegaRAID Controller
(Primary Storage)
----------------------------------------------------------------

       Use this procedure if your boot unit is going to be managed by the
       MegaRAID controller.

       Due to an issue with LSI mpt driver, and FreeBSD versions 6.1-9.X 
       include outdated LSI drivers, your MegaRAID controller for primary 
       storage is impacted.  

       If the mpt driver issue occurs, the system will generate a continuous 
       soft and hard reset and the system cannot boot.  The installation disk 
       will not install the OS and the process must be terminated.  To avoid 
       the issue with the mpt driver, you need to disable the mpt driver.

       If you have FreeBSD versions up to 7.4, 8.2, or 9.0, your FreeBSD OS 
       most likely contains the outdated mfi driver in the kernel. A new 
       FreeBSD kernel ISO needs to be created to exclude the outdated mfi 
       driver before you load the updated mfi (ver 4.8 or newer) or mrsas 
       driver.  With the newly created FreeBSD kernel ISO and the updated 
       mfi (ver 4.8 or newer) or mrsas driver loaded, you can complete the
       installation and boot your FreeBSD system through a MegaRAID VD.

       Please note that this situation would no longer exist when the mpt 
       driver issue is fixed and when the outdated inbox mfi driver is 
       replaced with the new mfi driver.  Effort toward the resolution of
       this problem is underway and in the future the extra steps would
       not be necessary.  

       For the interim however, please follow the direction in the KB 
       article 16687 (Installing FreeBSD driver for MR Primary Storage) on
       LSI website (www.lsi.com -> SUPPORT->Knowledgebase) to download the
       FreeBSD ISO and/or procedures necessary to accomplish the aforementioned.

       After you have the FreeBSD Installation disk with the new FreeBSD 
       kernel ISO and the mfi (ver 4.8 or newer) or mrsas driver ready, 
       follow the instructions below.  It describes how to load the FreeBSD
       kernel driver module to enable boot device support, and how to then 
       compile the current drivers into the kernel from source files.

Prerequisite: You need an OS Installation CD for
              FreeBSD 7.4, FreeBSD 8.2, or FreeBSD 9.0 (x86 or amd64)

Note: This procedure is specific to FreeBSD 7.4, FreeBSD 8.2 or FreeBSD 9.0
      since it requires a compiled module.  For other versions of FreeBSD for
      which a compiled module is not supplied by LSI it is necessary to compile
      your own module from source files.
      See "Compiling and Loading the Driver as a Module using kldload 
          (dynamically loadable module)".

     1. Disconnect all SCSI, ATA, and SAS devices in the system, except the CD
        or DVD and hard drives connected to the MegaRAID controller.

     2. Create the RAID VDs on the MegaRAID controller using WebBIOS, or
        Preboot Cli.

     3. Insert the FreeBSD Installation disk to CD/DVD drive and boot from it.

     4. At the FreeBSD boot menu, select "Escape to loader prompt." (option 6
        in FreeBSD 7.x and 8.x, and option 2 in FreeBSD 9.x).

     5. Insert the diskette containing driver_name.ko module into the diskette 
        drive.

        For mfi driver, the driver_name.ko is mfi.ko. 
        For mrsas driver, the driver_name.ko is mrsas.ko.

     6. At the loader prompt.

        To disable the mpt driver, type the following at the prompt:

        OK set hint.mpt.0.disable=1
        OK set hint.mpt.1.disable=1

        To load a new driver from a diskette drive:

        For mfi driver,
        OK load disk0:mfi.ko

        For mrsas driver,
        OK load disk0:mrsas.ko

Note_1: If you have multiple devices/units for OS installation, and the device/unit
        that you want to use for the installation is not disk0, please change disk0 
        to the appropriate number, i.e. disk1 or disk5, etc.

Note_2: Remember, at this point the old driver is not used but will still be 
        installed as a part of the new installation.  The installed kernel will 
        have to be rebuilt.

     7. Continue with the installation by typing.

	OK boot


     8. Install the FreeBSD OS - including kernel source.
	Please, remember including kernel source files is critical.	
	
     9. Remove both the FreeBSD installation disk from the CD/DVD driver and the 
        floppy disk from the floppy drive, and reboot.

     10.On the bootup, repeat step 4-7. 

     11.Upon boot, continue with the instruction in the section of 
        "Updating the Kernel with the New Driver Source" for 
        statically linked module.

 
----------------------------------------------------------------------
Installing the Kernel Driver on a FreeBSD System that Boots from
      a Device NOT Managed by a MegaRAID Controller(Secondary Storage)
----------------------------------------------------------------------

     Use the steps in this section if FreeBSD boots from a device NOT managed by a 
     MegaRAID controller, but MegaRAID controller is or will be present on the system 
     and used for managing secondary storage.

     When you use the VD managed by the MegaRAID controller for secondary storage, 
     you do not need to use a driver diskette for driver installation.

Note: Due to an issue with LSI mpt driver in FreeBSD 7.x, 8.x and 9.x, your Megaraid
      controllers may be impacted.  To avoid the issue with mpt driver, you may 
      need to disable the mpt driver before install or boot your FreeBSD system
      until you exclude the mpt driver from the installed kernel.  If the mpt 
      driver is not removed, the system will generate a soft reset and hard 
      reset errors on the bootup.  DVD will not load.

Tip: Install FreeBSD on the drive attached to the motherboard before installing
     the MegaRAID controller. This avoids the possibility of installing to the
     wrong drive or unit.

     1 Get the latest driver source files for your version of FreeBSD.
       See "Obtaining LSI FreeBSD Drivers".


A. System does not include an LSI controller before the OS installation:
-------------------------------------------------------------------------

     1. Install FreeBSD - including kernel source - on a motherboard attached drive. 
        Please, remember including kernel source files is critical.

     2. After the OS installation, please reboot.

     3. Follow the instructions for driver installation in 

        "Updating the Kernel with the New Driver Source" for Statically Linked
         Module,  or
 
        "Compiling and Loading the Driver as a Module using kldload" for Dynamically 
         Loadable Module.

     4. Once FreeBSD is installed and the driver is updated, power down the system and 
        install the MegaRAID controller. For assistance, see the installation guide
        that came with the controller.


B. System includes a Megaraid controller before the OS installation:
--------------------------------------------------------------------

     1. Begin the installation process. 

     2. As the DVD loads, the user is offered boot options.

     3. Select "Escape to loader prompt." (option 6 in FreeBSD 7.x and 8.x, 
        and option 2 in FreeBSD 9.x).
 
	To disable the mpt driver, type the following at the prompt:

        OK set hint.mpt.0.disable=1
        OK set hint.mpt.1.disable=1
        OK boot

     4. Proceed with the installation on a disk attached to the motherboard.
           Be sure to install the full FreeBSD source.

     5. After the OS installation, follow the instructions for driver installation in

        "Updating the Kernel with the New Driver Source" for Statically Linked
        Module,  or
 
        "Compiling and Loading the Driver as a Module using kldload" (for Dynamically 
         Loadable Module).


==============================
==============================
Updating Drivers under FreeBSD
==============================
==============================

     Drivers can be updated either from source files or with driver modules.

     Using source files, you can compile drivers into the kernel or you can
     create modules for versions of FreeBSD for which LSI does not supply
     modules.

     This section includes these topics:

        o "Updating the Kernel with the New Driver Source"

        o "Compiling and Loading the Driver as a Module using kldload" (dynamically 
          loadable module)

        o "Updating the MegaRAID Kernel Driver Module Under FreeBSD"
     
     
----------------------------------------------
Updating the Kernel with the New Driver Source 
----------------------------------------------

     Use this procedure if you wish to update your kernel from driver source files
     for both dynamically loadable module and statically linked module.  To obtain 
     source files, see "Obtaining LSI FreeBSD Drivers".

     1. Make sure that kernel source has been installed on the system.
        The kernel sources are in the /usr/src/sys directory.

     2. Proceed to the /usr/src/sys/dev/ directory
        Check if a directory with the driver name already exists.  If it is,
        move it to a different folder.

        For example:
        Do the following for mfi driver.
        # mv /usr/src/sys/dev/mfi /root/mfi.backup

        Or do the following for mrsas driver.
        # mv /usr/src/sys/dev/mrsas /root/mrsas.backup

        And make sure the directory exists.

        For example:
        Perform the following command for mfi driver.
        # mkdir /usr/src/sys/dev/mfi

        Or perform the following command for mrsas driver.
        # mkdir /usr/src/sys/dev/mrsas

     3. Proceed to the /usr/src/sys/modules/ directory
        Check if a directory with the name of your driver already exists.
        If it is, move it to a different folder.

        For example:

        For mfi driver,
        # mv /usr/src/sys/modules/mfi /root/mfi_module.backup

        For mrsas driver,
        # mv /usr/src/sys/modules/mrsas /root/mrsas_module.backup

        And make sure the directory exists.

        For example:

        For mfi driver,
        # mkdir /usr/src/sys/modules/mfi

        For mrsas driver,
        # mkdir /usr/src/sys/modules/mrsas

     4. Unpack the compressed driver source file (mfi.tgz for mfi driver and 
        mrsas.tgz for mrsas driver) and move the *.c and *.h files to 
        /usr/src/sys/dev/driver_name (which is mfi for mfi driver and mrsas 
        for mrsas driver), and move the Makefile to the 
        /usr/src/sys/modules/driver_name (which is mfi for mfi driver and 
        mrsas for mrsas driver).

        For example:

        # cd /usr/src/sys/dev

        For mfi driver,
        # tar -xf mfi.tgz
        # mv mfi/Makefile /usr/src/sys/modules/mfi/.

        For mrsas driver,
        # tar -xf mrsas.tgz
        # mv mrsas/Makefile /usr/src/sys/modules/mrsas/.

     5. At this point, proceed to 
	/usr/src/sys/i386/conf if you are running a 32bit version of FreeBSD
	/usr/src/sys/amd64/conf if you are running a 64bit version of FreeBSD

        a. copy the GENERIC configuration file to another name. For example:

           # cp GENERIC MYKERNEL


        b. Open your current configuration file: (GENERIC, MYKERNEL, SMP, or PAE
           or custom config), with vi or other editor.  For example:

           # vi MYKERNEL

           Comment out the following lines by placing '#' at the beginning of the line:

           #device   cbb   # cardbus (yenta) bridge
           #device   mpt   # LSI-Logic MPT-Fusion

           For a dynamically loadable module
	   ---------------------------------

	   Comment out the following line by placing '#' at the beginning of the line.

           #device   mfi   # LSI MegaRAID SAS

           For mrsas driver, comment out the following line by placing '#' at the 
           beginning of the line if it exists. 

           #device   mrsas  # LSI MegaRAID SAS2

           For a statically linked module
	   ------------------------------

	   For mfi driver, make sure the following line is NOT commented out.

           device   mfi   # LSI MegaRAID SAS

           For mrsas driver, make sure the following line is NOT commented out if 
           it exists.

           device   mrsas  # LSI MegaRAID SAS2

           If you do not have any MR Liberator or older controller installed in 
           your system, make sure to comment out the following line.

           #device   mfi   # LSI MegaRAID SAS

        c. Save changes.

        d. In /usr/src/sys/conf/files, do the following:

           For mfi driver, add the following entries after the entry
           for "dev/mfi/mfi_cam.c":

           dev/mfi/mfi_syspd.c   optional mfi

           With an older mfi driver v4.3x on MR Thunderbolt controller support, also 
           add the following:

           dev/mfi/mfi_fp.c      optional mfi
           dev/mfi/mfi_tbolt.c   optional mfi

           For mrsas driver, make sure the following entries are present.

           dev/mrsas/mrsas.c          optional mrsas
           dev/mrsas/mrsas_cam.c      optional mrsas
           dev/mrsas/mrsas_ioctl.c    optional mrsas

           NOTE:  Each line is an entry in this file.
                  Make sure a newly added entry is on a new line.

        e. Save changes.

        f. To compile the kernel, go to /usr/src directory and type the following if 
           your current configuration is MYKERNEL.

           # cd /usr/src
           # make buildkernel KERNCONF=MYKERNEL

        g. To install the new kernel, type the following if your current configuration 
           is MYKERNEL.

           # make installkernel KERNCONF=MYKERNEL

           The new kernel will be copied to the /boot/kernel directory.
           The old kernel will be moved to the /boot/kernel.old directory.
           
     6. Reboot your system.

       When the system reboots, the new kernel driver module will load
       automatically.
       
       

--------------------------------------------------------------------------------------
Compiling and Loading the Driver as a Module using kldload (dynamically linked module)
--------------------------------------------------------------------------------------

     If you want to use a driver module and LSI does not supply one for your
     version of FreeBSD, use the following procedure. If you just want to
     install an LSI-supplied module, see "Updating the MegaRAID Kernel Driver
     Module Under FreeBSD".
     
     Note: You can only use kldload to load the driver as a module if your boot
           drive is attached to the mother board and is not managed by the
           MegaRAID controller.

     To compile the driver as a module
     
     1. Boot to FreeBSD.

     2. Obtain driver source files, see "Obtaining LSI FreeBSD Drivers".

     3. Prepare your kernel is prepared for Dynamically Loadable Module described in

        "Updating the Kernel with the New Driver Source" 
        
     4. To build the driver module, type the following commands, and press Enter.

        For mfi driver,
        # cd /usr/src/sys/modules/mfi
        # make

        For mrsas driver,
        # cd /usr/src/sys/modules/mrsas
        # make

        These steps create the driver driver_name.ko module in the 
        /usr/src/sys/modules/driver_name directory.

     5. Load the kernel driver module

        For mfi driver,
        # cd /usr/src/sys/modules/mfi
        # kldload -v ./mfi.ko

        For mrsas driver,
        # cd /usr/src/sys/modules/mrsas
        # kldload -v ./mrsas.ko

     6. If you wish to load the driver automatically on the bootup - but still 
        be linked dynamically:

        a.  Copy the driver into the boot folder.

            For mfi driver,
            # cp /usr/src/sys/modules/mfi/mfi.ko /boot/kernel/mfi.ko

            For mrsas driver,
            # cp /usr/src/sys/modules/mrsas/mrsas.ko /boot/kernel/mrsas.ko

            (Backup the old module in case you want to revert back to it.)

        b.  Add the following line to the file /boot/loader.conf.

            For mfi driver,
            mfi_load="YES"

            For mrsas driver,
            mrsas_load="YES"


--------------------------------------------------------
Updating the MegaRAID Kernel Driver Module Under FreeBSD
--------------------------------------------------------

     The following steps describe how to update the MegaRAID driver with a
     kernel driver module under FreeBSD.
     
     Make a backup of your original driver before updating in case you need
     to revert back to it.  However, you will not be able to revert back to
     the original driver if you are booting from that VD.

     1. Obtain the driver, see "Obtaining LSI FreeBSD Drivers".

     2. Make a backup of any critical data prior to updating the MegaRAID driver.

     3. Change the directory to the location with the driver.
     
     4. Copy the driver into /boot/kernel.

        For mfi driver,
        # cp mfi.ko /boot/kernel

        For mrsas driver,
        # cp mrsas.ko /boot/kernel

        Make sure the module version matches the FreeBSD version. If the
        versions don't match there could be a kernel panic.

     5. Load the driver.

        For mfi driver,
        # kldload -v /boot/kernel/mfi.ko

        For mrsas driver,
        # kldload -v /boot/kernel/mrsas.ko

        If storage is present, you should see information in the system
        log (usually, /var/log/messages).

     6. If you wish to load the driver automatically every time the system 
        reboots, add the following line to the file /boot/loader.conf

        For mfi driver,
        mfi_load="YES"

        For mrsas driver,
        mrsas_load="YES"
