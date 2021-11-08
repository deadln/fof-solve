#!/bin/bash

cd `dirname $0`
dir=`pwd`

dels="core *.debug *.sym Saved"

for d in $dels
do
  find -L $dir -name "$d"
  find -L $dir -name "$d" -print0 | xargs -0 /bin/rm -rf
done
