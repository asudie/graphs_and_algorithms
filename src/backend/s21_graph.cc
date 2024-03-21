
#include "s21_graph.h"

namespace s21 {

Graph::Graph(){};

void Graph::SetData(std::string filename) {
  path_ = filename;
  FillData();
}

void Graph::FillData() {
  std::ifstream file(path_);
  if (!file) throw std::logic_error("Failed to open the file!");
  std::string line;
  int row, col;
  std::getline(file, line);
  std::istringstream iss(line);
  iss >> row;
  col = row;
  if (row > 0) {
    SetDataSize(row);
  } else {
    file.close();
    throw std::logic_error("Incorrect matrix size!");
  }
  ClearData();
  int num;

  for (int i = 0; i < row; i++) {
    if (std::getline(file, line)) {
      std::istringstream new_iss(line);
      std::vector<int> v1;

      for (int j = 0; j < col; j++) {
        if (new_iss >> num) v1.push_back(num);
      }
      PushToMatrix(v1);
    }
  }
  file.close();
  if (CheckMatrix()) {
    PrintMatrix();
  } else {
    ClearData();
    size_ = 0;
    throw std::logic_error("Incorrect matrix!");
  }
}

void Graph::LoadGraphFromFile(std::string filename) { SetData(filename); }

int Graph::CheckMatrix() {
  if (matrix_.size() == (size_t)size_) {
    for (int i = 0; i < size_; i++) {
      if (matrix_[i].size() != (size_t)size_) return 0;
    }
  } else {
    return 0;
  }
  return 1;
}

void Graph::PrintMatrix() const {
  std::cout << "Matrix of Data:" << std::endl;
  for (int i = 0; i < size_; ++i) {
    for (int j = 0; j < this->size_; ++j) {
      std::cout << matrix_[i][j] << " ";
    }
    std::cout << std::endl;
  }
};

void Graph::ExportGraphToDot(std::string filename) {
  std::ofstream out;
  out.open(filename);
  if (out.is_open()) {
    out << "graph graphname {" << std::endl;
    for (int i = 0; i < size_; ++i) out << '\t' << (i + 1) << ';' << std::endl;
    for (int i = 0; i < size_; ++i) {
      for (int j = 0; j < size_; ++j) {
        if (matrix_[i][j] != 0) {
          out << '\t' << (i + 1) << " -- " << (j + 1) << ';' << std::endl;
        }
      }
    }
    out << "}";
    out.close();
  } else {
    throw std::logic_error("Can't save file!");
  }
};

std::vector<std::vector<int>> Graph::GetGraph() const { return matrix_; }
}  // namespace s21