make clean
make
./hw3 ../testcase/n$1.hardblocks ../testcase/n$1.nets ../testcase/n$1.pl ../output/n$1.floorplan $2
#valgrind ./hw3 ../testcase/n100.hardblocks ../testcase/n100.nets ../testcase/n100.pl ../output/n100.floorplan 0.1
