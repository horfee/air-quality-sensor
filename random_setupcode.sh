#!/bin/bash
for i in {1..8}
do
	echo -n $(($RANDOM%10))
done
