#! /bin/sh
b=`basename -s .asm "$1"`
zasm -uw ${b}.asm
