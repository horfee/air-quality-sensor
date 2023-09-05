#!/bin/bash


setup_code=`./random_setupcode.sh`
setup_id=`./random_setupid.sh`


name=factory
$IDF_PATH/../esp-homekit-sdk/tools/factory_nvs_gen/factory_nvs_gen.py $setup_code $setup_id $name
echo "Setup Information"
echo "----------------------"
echo "Setup code  : $setup_code"
echo "Setup id    : $setup_id"
echo "Output file : $name.bin"
echo "----------------------"
echo "run"
echo "esptool.py -p \$ESPPORT write_flash 0x340000 $name.bin"
echo "(0x340000 is defined in partitions_hap.csv file)