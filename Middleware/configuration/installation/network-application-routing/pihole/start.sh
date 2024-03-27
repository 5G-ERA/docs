#!/bin/bash

dir="etc-dnsmasq.d"
file="02-my-wildcard-dns.conf"
mkdir $dir
touch "$dir/$file"
echo "address=/$1/$2" >> "$dir/$file"

docker compose up -d
