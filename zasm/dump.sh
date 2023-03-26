#! /bin/sh
case "$1" in
-b*)  s=`echo "$1" |sed 's/^-b/0x/'`; shift;;
esac
hd "$@" |sed '
s/^\([0-9a-f][0-9a-f]*\) */0x\1= /
s/\([0-9a-f][0-9a-f]\) \([0-9a-f][0-9a-f]\) /\1\2 /g
s/ \([0-9a-f][0-9a-f]\) / \100 /
s/ *|.*$//
s/=//
/^\*/d
' |awk 'BEGIN { s=strtonum('"$s"'); }
{ a = strtonum($1); printf "=%X ", a + s; print $2,$3,$4,$5,$6,$7,$8,$9; }'
