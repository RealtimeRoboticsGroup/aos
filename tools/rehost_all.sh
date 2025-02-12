#!/bin/bash

set -e

if [ "$#" -lt 1 ];
then
  echo "Usage: rehost_all.sh buildkite_logfile.log"
  exit 1
fi

for url in $(cat "$1" | sed 's/\x1b_bk;t=[0-9]*\x07//g' | sed 's/\x1b[[(]*[ ?!,0-9;]*[\x0-\x1a=>@-Za-z~]//g;s/\x1b.*[\\\x7\$]//g' | sed 's/\r//g' | grep WARNING | grep 'returned 404 Not Found' | sed 's,WARNING: Download from https://realtimeroboticsgroup.org/build-dependencies/,,' | sed 's, failed: class java.io.FileNotFoundException GET returned 404 Not Found,,' | sort | uniq);
do
  ./rehost.sh "${url}"
done
