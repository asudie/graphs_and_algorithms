#ifndef S21_GRAPH_H_
#define S21_GRAPH_H_

#include <exception>
#include <fstream>
#include <initializer_list>
#include <iostream>
#include <sstream>
#include <vector>

namespace s21 {
class Graph {
 public:
  Graph();
  ~Graph(){};
  void LoadGraphFromFile(std::string filename);
  std::vector<std::vector<int>> GetGraph() const;
  int GetDataSize() { return size_; };
  void SetDataSize(int size) { size_ = size; };
  void ClearData() { matrix_.clear(); };
  void PushToMatrix(std::vector<int> v) { matrix_.push_back(v); };
  void ExportGraphToDot(std::string filename);
  void SetData(std::string filename);
  void FillData();
  void PrintMatrix() const;
  int CheckMatrix();
  std::string GetPath() { return path_; };

 private:
  std::string path_;
  int size_;
  std::vector<std::vector<int>> matrix_;
};
}  // namespace s21

#endif  // S21_GRAPH_H_