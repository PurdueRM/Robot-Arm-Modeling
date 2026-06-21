cd build
cmake ..
make
./executable 

# for debug
make && valgrind --leak-check=full ./executable

gcc arg_parser.c user_math.c -lm -o temp.out
