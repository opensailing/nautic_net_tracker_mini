#! /bin/sh

# The nautic_net_protobuf repository must exist alongside nautic_net_tracker_mini

python2.7 \
    .pio/libdeps/adafruit_feather_m0/Nanopb/generator/nanopb_generator.py \
    -I ../nautic_net_protobuf/lib/nautic_net/protobuf \
    -D src \
    lora_packet.proto 
