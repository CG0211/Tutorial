#!/bin/bash

file_name="00_Test.txt"


sed -i '2,4s/^/# /g' $file_name

sed -i 'bjev/s/^/# /g' $file_name

