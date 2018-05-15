#!/bin/bash

echo "#!/bin/sh"
for filename in `ls *.in`;
  do
     echo "./bitstar 10 50 <map.sw  >$filename"
  done
