
# https://github.com/analogdevicesinc/libiio/blob/main/README_BUILD.md
# this did not provide iio_info
brew install libxml2 zstd bison flex cmake
brew install libusb libserialport avahi
brew install doxygen graphviz
brew install python3
cd ~
git clone https://github.com/analogdevicesinc/libiio.git
cd libiio
mkdir build
cd build
cmake ../ -DOSX_FRAMEWORK=ON -DOSX_PACKAGE=ON
cmake ../ -DOSX_FRAMEWORK=ON -DOSX_PACKAGE=ON -DCPP_BINDINGS=ON -DPYTHON_BINDINGS=ON
make -j$(sysctl -n hw.ncpu)
sudo make install