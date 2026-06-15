#! /bin/bash

attempt=0
while [ $attempt -le 1000 ]; do
	time cat ./prof_fdd_61_44_sr_mcs.json > /sys/bus/iio/devices/iio:device1/profile_config
	stat=$?

	if [ "$stat" != "0" ]; then
		echo "load profile #$attempt FAILED"
		break
	else
		echo " --$attempt profile loaded successfully -- "
	fi
	attempt=$(($attempt +1))
done
