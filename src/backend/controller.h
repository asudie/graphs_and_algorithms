#ifndef EXAMPLECONTROLLER_H
#define EXAMPLECONTROLLER_H

#include "s21_graph.h"
#include "s21_graph_algorithms.h"

namespace s21 {
class ExampleController {
 private:
 public:
  ExampleController(Graph *m) { model_ = m; };
  void Load(std::string filename);
  void Breadth(int start_vertex);
  void Depth(int start_vertex);
  void Two(int start_vertex, int end_vertex);
  std::vector<std::vector<int>> Tree();
  std::vector<std::vector<int>> All();
  void Sale();
  void Dot(std::string filename);
  Graph *model_;
};
}  // namespace s21

#endif