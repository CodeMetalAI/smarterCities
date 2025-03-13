
# this is a summary of the instructions on: 
# https://wiki.analog.com/university/tools/pluto/building_the_image

vivado_dir='/tools/Xilinx/2023.2/Vivado/2023.2'
export CROSS_COMPILE=arm-linux-gnueabihf-
export PATH=$PATH:${vivado_dir}/gnu/aarch32/lin/gcc-arm-linux-gnueabi/bin
export VIVADO_SETTINGS=${vivado_dir}/settings64.sh
source ${vivado_dir}/settings64.sh

# do this 
sudo apt-get install dfu-util u-boot-tools device-tree-compiler mtools
sudo apt-get install bc python-is-python3 cpio zip unzip rsync file wget

cd ~
git clone --recursive https://github.com/analogdevicesinc/plutosdr-fw.git

mkdir -p ~/toolchains
cd ~/toolchains
wget https://releases.linaro.org/components/toolchain/binaries/7.3-2018.05/arm-linux-gnueabihf/gcc-linaro-7.3.1-2018.05-x86_64_arm-linux-gnueabihf.tar.xz
tar -xf gcc-linaro-7.3.1-2018.05-x86_64_arm-linux-gnueabihf.tar.xz
# > .bashrc (the following 2 lines) 
export PATH=$PATH:~/toolchains/gcc-linaro-7.3.1-2018.05-x86_64_arm-linux-gnueabihf/bin
export SWT_GTK3=0  # To prevent GTK3 issues mentioned in the docs

# this is the vhdl project directory 
cd ~/plutosdr-fw/hdl/projects/pluto
make  # make vhdl  
# vivado # run vivado 
#### fun part ... edit the vhdl code in this folder 

# if using vivado, export hardware (include bitstream) to 
# ~/plutosdr-fw/hdl/projects/pluto/pluto.sdk/system_top.xsa


cd ~/plutosdr-fw # make the whole firmware 
make # not recommended to use extra threads -j8 only takes ~3 minutes if cached

# ssh root@192.168.2.1 analog
pluto_reboot ram
# in computer
dfu-util -l # check it's there
# looks like some plutos are b673, b674, if this errors match device ID to error. Liekly fixable in build process
sudo dfu-util -d 0456:b673 -D ~/plutosdr-fw/build/pluto.dfu -a firmware.dfu -R
# ssh in to check that it's now running a dirty firmware (e.g.):  "v0.39-dirty"
# if not done in sudo, doesn't work, idk why 



#### slightly imprecise list of things
# sudo apt-get install git build-essential ccache device-tree-compiler dfu-util fakeroot libncurses5-dev libssl-dev mtools rsync u-boot-tools bc python cpio zip unzip file wget

#### official list (python problem in linux) 
#  sudo apt-get install git build-essential fakeroot libncurses5-dev libssl-dev ccache
#  sudo apt-get install dfu-util u-boot-tools device-tree-compiler libssl1.0-dev mtools
#  sudo apt-get install bc python cpio zip unzip rsync file wget

# started adding one by one: sudo apt-get install libmpc-dev xvfb 

###### would be nice if this worked ...
# cd ~ 
# git clone https://github.com/analogdevicesinc/plutosdr_scripts
# sudo apt-get install sshpass
# cd ~/smarterCities/plutosdr_scripts
# sudo ./pluto_ramboot ~/smarterCities/plutosdr-fw/build/pluto.dfu
