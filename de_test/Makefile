all: cmdline.cpp de_test.cpp tutorial.cpp cmdline.h objective_function.h testfunctions.h
	g++ -I.. tutorial.cpp -o tutorial.exe
	g++ -I.. cmdline.cpp de_test.cpp -lboost_program_options -lboost_regex -o de_test.exe