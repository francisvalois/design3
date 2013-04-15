#!/bin/bash

pid=$(ps | grep Kinocto | cut -d ' ' -f2)
for line in $pid
do
   kill -9 $line
done

pid=$(ps | grep BaseStation | cut -d ' ' -f2)
for line in $pid
do
   kill -9 $line
done

