#!/bin/bash

# EPOS Command Library 6.6.2.0 installation script
# Copyright (c) maxon motor ag 2014-2020

if [[ $UID != 0 ]]; then
	echo 'Please run this installation script with sudo:'
	echo 'sudo' $0 $*
	exit 1
fi

function check_result {
	if (($? > 0)); then
		printf ' [FAILED]\n'
	else
		printf ' [OK]\n'
	fi
}
function install {

echo '---------------------------------------------------------'
echo 'EPOS Command Library 6.6.2.0 installation started'
echo '---------------------------------------------------------'

#remove previous installation
printf ' - Remove existing installation'
rm -rf '/opt/EposCmdLib_6.6.2.0' > /dev/null
check_result

#copy examples, include and misc files
printf ' - Install library into directory: /opt/EposCmdLib_6.6.2.0'
mkdir '/opt/EposCmdLib_6.6.2.0' > /dev/null
mkdir '/opt/EposCmdLib_6.6.2.0/lib' > /dev/null

cp -rf ./include '/opt/EposCmdLib_6.6.2.0' > /dev/null
architecture=$(uname -m)
case $architecture in
	armv6l)
		cp -rf ./lib/arm/v6 '/opt/EposCmdLib_6.6.2.0/lib' > /dev/null
		;;
	armv7l)
		cp -rf ./lib/arm/v7 '/opt/EposCmdLib_6.6.2.0/lib' > /dev/null
		;;
	aarch64)
		cp -rf ./lib/arm/v8 '/opt/EposCmdLib_6.6.2.0/lib' > /dev/null
		;;
	 x86|i386|i486|i586|i686)
		cp -rf ./lib/intel/x86 '/opt/EposCmdLib_6.6.2.0/lib' > /dev/null
		;;
	 x86_64)
		cp -rf ./lib/intel/x86_64 '/opt/EposCmdLib_6.6.2.0/lib' > /dev/null
		;;
esac
check_result

#copy examples, include and misc files
printf ' - Install examples into directory: /opt/EposCmdLib_6.6.2.0'
if [ ! -d "/opt/EposCmdLib_6.6.2.0" ]; then
mkdir '/opt/EposCmdLib_6.6.2.0' > /dev/null
fi

cp -rf ./examples '/opt/EposCmdLib_6.6.2.0' > /dev/null
cp -rf ./misc '/opt/EposCmdLib_6.6.2.0' > /dev/null
find "/opt/EposCmdLib_6.6.2.0/examples" -type d -exec chmod 777 {} \;
find "/opt/EposCmdLib_6.6.2.0/examples" -type f -exec chmod 666 {} \;
check_result

#create symlinks
printf ' - Library system integration'
architecture=$(uname -m)
case $architecture in
	armv6)
		ln -sf '/opt/EposCmdLib_6.6.2.0/lib/v6/libEposCmd.so.6.6.2.0' /usr/lib/libEposCmd.so
		ln -sf '/opt/EposCmdLib_6.6.2.0/lib/v6/libftd2xx.so.1.4.8' /usr/lib/libftd2xx.so
		;;
	armv7l)
		ln -sf '/opt/EposCmdLib_6.6.2.0/lib/v7/libEposCmd.so.6.6.2.0' /usr/lib/libEposCmd.so
		ln -sf '/opt/EposCmdLib_6.6.2.0/lib/v7/libftd2xx.so.1.4.8' /usr/lib/libftd2xx.so
		;;
	aarch64)
		ln -sf '/opt/EposCmdLib_6.6.2.0/lib/v8/libEposCmd.so.6.6.2.0' /usr/lib/libEposCmd.so
		ln -sf '/opt/EposCmdLib_6.6.2.0/lib/v8/libftd2xx.so.1.4.8' /usr/lib/libftd2xx.so
		;;
	 x86|i386|i486|i586|i686)
		ln -sf '/opt/EposCmdLib_6.6.2.0/lib/x86/libEposCmd.so.6.6.2.0' /usr/lib/libEposCmd.so
		ln -sf '/opt/EposCmdLib_6.6.2.0/lib/x86/libftd2xx.so.1.4.8' /usr/lib/libftd2xx.so
		;;
	 x86_64)
		ln -sf '/opt/EposCmdLib_6.6.2.0/lib/x86_64/libEposCmd.so.6.6.2.0' /usr/lib/libEposCmd.so
		ln -sf '/opt/EposCmdLib_6.6.2.0/lib/x86_64/libftd2xx.so.1.4.8' /usr/lib/libftd2xx.so
		;;
esac
check_result

#add udev rules
printf ' - Configure device access rights'
cp -f './misc/99-ftdi.rules' /etc/udev/rules.d > /dev/null
cp -f './misc/99-epos4.rules' /etc/udev/rules.d > /dev/null
check_result

udevadm control --reload-rules && udevadm trigger

#add sudo rules
printf ' - Configure user access rights'
touch -f /etc/sudoers.d/mmc_rule
echo $SUDO_USER 'ALL=(ALL) NOPASSWD: /bin/ip' > /etc/sudoers.d/mmc_rule
chmod 0440 /etc/sudoers.d/mmc_rule
check_result

echo '---------------------------------------------------------'
echo 'EPOS Command Library 6.6.2.0 installed'
echo '---------------------------------------------------------'
}
function uninstall {

echo '---------------------------------------------------------'
echo 'EPOS Command Library 6.6.2.0 deinstallation started'
echo '---------------------------------------------------------'

#remove access rights
printf ' - Reconfigure user access rights'
rm -f /etc/sudoers.d/mmc_rule > /dev/null
check_result

#remove udev rules
printf ' - Reconfigure device access rights'
rm -f /etc/udev/rules.d/99-epos4.rules > /dev/null
rm -f /etc/udev/rules.d/99-ftdi.rules > /dev/null
check_result

service udev restart

#remove symbolic links
printf ' - Remove library system integration'
rm -f /usr/lib/libEposCmd.so > /dev/null
rm -f /usr/lib/libftd2xx.so > /dev/null
check_result

#remove previous installation
printf ' - Remove existing installation'
rm -rf '/opt/EposCmdLib_6.6.2.0' > /dev/null
check_result

#remove user data
read -p " - Remove user ($SUDO_USER) data: /home/$SUDO_USER/.maxon_motor_ag [Yy/n]? " -n 1 -r
if [[ $REPLY =~ ^[Yy]$ ]]
then
	rm -rf "/home/$SUDO_USER/.maxon_motor_ag" > /dev/null
fi
echo

echo '---------------------------------------------------------'
echo 'EPOS Command Library 6.6.2.0 uninstalled'
echo '---------------------------------------------------------'
}

if [ "$#" -eq 0 ]; then
	install
else
	for i in "$@"
	do
	case $i in
		"-u"|"--uninstall")
		uninstall
		shift
		;;
		"-i"|"--install")
		install
		shift
		;;
		*)
		echo "usage install.sh -i /--install/ [default] -u /--uninstall/"
		shift
		;;
	esac
	done
fi
