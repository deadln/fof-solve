if [ "$1" == "prof" ]; then
 arg=""
elif [ "$1" == "nonprof" ]; then
 arg=""
else
  echo "$0 (prof | nonprof) [num] [pkg]"
  exit
fi
mode=$1
shift

if [ "$1" != "" ]; then
  num=$1
  shift
fi

if [ "$1" != "" ]; then
  pkg=$1
  shift
fi
