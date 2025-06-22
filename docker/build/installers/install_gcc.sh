#! /usr/bin/env bash

if [ ! -x "$(which add-apt-repository)" ]; then
  sudo apt-get -y update
  sudo apt-get -y install software-properties-common
fi

echo "$(lsb_release -rs)"
sudo add-apt-repository -y ppa:ubuntu-toolchain-r/test

sudo apt-get -y update \
  && sudo apt-get -y install \
    gcc-9 \
    g++-9

sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-9 90
sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-9 90

# TODO(zero): Update to GCC 11 if needed
# sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-11 110
# sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-11 110

echo "You can manually configure the default version of GCC and G++ by running:"
echo "  sudo update-alternatives --config gcc"
echo "  sudo update-alternatives --config g++"

gcc --version
g++ --version
