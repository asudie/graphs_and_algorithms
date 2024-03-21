// #include "parser.h"
#include "console_view.h"
#include "s21_graph.h"

int main() {
  s21::Graph *gr = new s21::Graph;
  s21::ExampleController c(gr);
  s21::ConsoleView view(&c);
  view.StartEventLoop();
  return (0);
}
