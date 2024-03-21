CC=g++ 
GT= -lgtest -pthread -lgtest_main -std=c++17
FLAGS= -Wall -Werror -Wextra -std=c++17
LIBS_LINUX= -lcheck -lm -lpthread -lrt -lsubunit -D_GNU_SOURCE
LIBS_MACOS= -L /usr/local/lib -lcheck

OS = $(shell uname)
ifeq ($(OS), Darwin)
OPEN=open gcov_report/index.html
LIBS=$(LIBS_MACOS)
MYLIBS=$()
LEAK_CHECK = CK_FORK=no leaks -atExit -- ./navigator
endif

ifeq ($(OS), Linux)
OPEN=lynx
LIBS=$(LIBS_LINUX)
endif

all: test gcov_report

s21_graph.o_with_gcov: backend/s21_graph.cc
	$(CC) -c backend/s21_graph.cc --coverage

s21_graph.a_with_gcov: s21_graph.o_with_gcov 	
	ar -rcs s21_graph.a s21_graph.o

s21_graph_algorithms.o_with_gcov: backend/s21_graph_algorithms.cc
	$(CC) -c backend/s21_graph_algorithms.cc --coverage

s21_graph_algorithms.a_with_gcov: s21_graph_algorithms.o_with_gcov 	
	ar -rcs s21_graph_algorithms.a s21_graph_algorithms.o

s21_graph.a:
	$(CC) -c backend/s21_graph.cc
	ar -rcs s21_graph.a s21_graph.o

s21_graph_algorithms.a:
	$(CC) -c backend/s21_graph_algorithms.cc
	ar -rcs s21_graph_algorithms.a s21_graph_algorithms.o

s21_stack.a:
	$(CC) -c utils/s21_stack.cc
	ar -rcs s21_stack.a s21_stack.o

s21_queue.a:
	$(CC) -c utils/s21_queue.cc
	ar -rcs s21_queue.a s21_queue.o	

test: clean s21_queue.a s21_stack.a s21_graph.a_with_gcov s21_graph_algorithms.a_with_gcov
	$(CC) $(FLAGS) test.cc s21_queue.a s21_stack.a s21_graph.a s21_graph_algorithms.a -o navigator $(GT) --coverage
	./navigator

run: clean 
	$(CC) $(FLAGS) -g utils/*.cc backend/*.cc -o a.out --coverage
	./a.out

gcov_report: test
	gcovr --html --html-details -o report.html
	open report.html

style:

# clang-format -style=google -i *.cc
# clang-format -style=google -i backend/*.cc
# clang-format -style=google -i backend/*.h
# clang-format -style=google -i utils/*.h
# clang-format -style=google -i utils/*.cc
# clang-format -style=google -i utils/*.inl

	clang-format -style=google -n *.cc
	clang-format -style=google -n backend/*.cc
	clang-format -style=google -n backend/*.h
	clang-format -style=google -n utils/*.h
	clang-format -style=google -n utils/*.cc
	clang-format -style=google -n utils/*.inl

clean:
	rm -rf *.out *.dSYM *.gch *.gcda *.gcno *.o *.a report.* s21_test.* test_with_gcov* *.gcov navigator *.dot
