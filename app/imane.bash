#!/bin/bash

pid=$(ps | grep Kinocto | cut -d ' ' -f1)
echo ps
for line in $pid
do
   echo $line
   kill -9 $line
done

pid=$(ps | grep BaseStation | cut -d ' ' -f1)
for line in $pid
do
   echo $line
   kill -9 $line
done

