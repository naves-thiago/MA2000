rm log.txt
mv /Volumes/ELUA/log.txt ./
diskutil unmount /Volumes/ELUA/
./convert log.txt
gnuplot script.txt
