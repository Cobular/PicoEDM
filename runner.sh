#! /usr/bin/env bash
echo "Filename: $1"
echo "flashing with elf2uf2-rs"
elf2uf2-rs -d "$1"

echo "Attaching defmt-print.."
picocom -b 256000 /dev/tty.usbserial-110 --nolock | defmt-print -e "$1"

