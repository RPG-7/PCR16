#!/bin/sh
#

cd ./
pwd
target_file="./PCR16_UpdateFW.bin"
target_bin1="./firmware_infor.bin"
target_bin2="./PCR16_CODE.bin"
#�ϲ�iapbin.bin��tempfile.bin��RTCA_IG_MAIN_IAP.bin
cat "${target_bin1}" "${target_bin2}" > "${target_file}"