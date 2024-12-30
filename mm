#!/bin/bash -e
# Example: Specify CAN ID to select parameter files
# e.g. CAN ID = 83200000
# ./mm 83200000 

#rm build/shaft_encoder2.elf
export FLOAT_TYPE=hard

# Parameters for CAN ID specified by $1
if [ -r params/$1-adc_idx_v_struct.c ]; then
	export ADC_PARAM=$1-adc_idx_v_struct.c
else
	echo params/$1-adc_idx_v_struct.c does not exist
	exit 1;
fi	

if [ -r params/$1-en_idx_v_struct.c ]; then
	export EN_PARAM=$1-en_idx_v_struct.c
else
	echo params/$1-en_idx_v_struct.c does not exist
	exit 2;
fi	
echo "################ CAN ID  ################"
echo $1
echo "################ PARAMETER FILES ################"
echo $ADC_PARAM : $EN_PARAM

export I_AM_CANID=0x$1
echo I_AM_CANID
make clean
./script-all shaft_encoder2 $1
