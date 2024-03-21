#ifndef CONSOLEVIEW_H
#define CONSOLEVIEW_H

#include "controller.h"
#include "s21_graph.h"

namespace s21 {

enum Choice {
  LOAD = 1,
  BREADTH = 2,
  DEPTH = 3,
  TWO = 4,
  ALL = 5,
  TREE = 6,
  SALE = 7,
  DOT = 8,
  EXIT = 0,
  NONE = -1
};

class ConsoleView {
 private:
  ExampleController *controller_;

 public:
  ConsoleView(ExampleController *c) : controller_(c){};
  void DisplayMenu();
  int PerformChoice();
  void StartEventLoop();
  void PrintMatrix(std::vector<std::vector<int>> matrix);
};
}  // namespace s21

#endif