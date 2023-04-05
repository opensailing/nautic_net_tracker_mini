#! /bin/sh

cd src
python2.7 ../.pio/libdeps/adafruit_feather_m0/Nanopb/generator/nanopb_generator.py lora.proto
cd -