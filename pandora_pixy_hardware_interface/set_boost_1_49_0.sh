#!/bin/bash


wget "http://sourceforge.net/projects/boost/files/boost/1.49.0/boost_1_49_0.tar.gz/download" -O boost_1_49_0.tar.gz

tar -zxvf boost_1_49_0.tar.gz
 
sudo mkdir /usr/boost_1_49_0

cd boost_1_49_0/

sudo ./bootstrap.sh --prefix=/usr/boost_1_49_0 --libdir=/usr/boost_1_49_0/lib --includedir=/usr/boost_1_49_0/include

sudo ./b2 install

cd ..


rm boost_1_49_0.tar.gz
sudo rm -r boost_1_49_0
