#include "controller.h"

namespace s21 {
void ExampleController::Load(std::string filename) {
  try {
    model_->LoadGraphFromFile(filename);
  } catch (std::exception &err) {
    std::cerr << err.what() << std::endl;
  }
}

void ExampleController::Breadth(int start_vertex) {
  if (model_->GetDataSize() > 1) {
    try {
      s21::GraphAlgorithms *new_alg = new s21::GraphAlgorithms;
      new_alg->s21::GraphAlgorithms::BreadthFirstSearch(*model_, start_vertex);
      new_alg->PrintRoute();
      delete new_alg;
    } catch (std::logic_error &err) {
      std::cerr << err.what() << std::endl;
    }

  } else {
    std::cerr << "Please, load the file first." << std::endl;
  }
}

void ExampleController::Depth(int start_vertex) {
  if (model_->GetDataSize() > 1) {
    try {
      s21::GraphAlgorithms *new_alg = new s21::GraphAlgorithms;
      new_alg->s21::GraphAlgorithms::BreadthFirstSearch(*model_, start_vertex);
      new_alg->PrintRoute();
      delete new_alg;
    } catch (std::exception &err) {
      std::cerr << err.what() << std::endl;
    }

  } else {
    std::cerr << "Please, load the file first." << std::endl;
  }
}

void ExampleController::Two(int start_vertex, int end_vertex) {
  if (model_->GetDataSize() > 1) {
    try {
      s21::GraphAlgorithms *new_alg = new s21::GraphAlgorithms;
      std::cout
          << new_alg->s21::GraphAlgorithms::GetShortestPathBetweenVertices(
                 *model_, start_vertex, end_vertex)
          << std::endl;
      delete new_alg;
    } catch (std::exception &err) {
      std::cerr << err.what() << std::endl;
    }
  } else {
    std::cerr << "Please, load the file first." << std::endl;
  }
}

std::vector<std::vector<int>> ExampleController::All() {
  std::vector<std::vector<int>> matrix;
  if (model_->GetDataSize() > 1) {
    try {
      s21::GraphAlgorithms *new_alg = new s21::GraphAlgorithms;
      matrix =
          new_alg->s21::GraphAlgorithms::GetShortestPathsBetweenAllVertices(
              *model_);
      delete new_alg;
    } catch (std::exception &err) {
      std::cerr << err.what() << std::endl;
    }

  } else {
    std::cerr << "Please, load the file first." << std::endl;
  }
  return matrix;
}

std::vector<std::vector<int>> ExampleController::Tree() {
  std::vector<std::vector<int>> matrix;
  if (model_->GetDataSize() > 1) {
    s21::GraphAlgorithms *new_alg = new s21::GraphAlgorithms;

    matrix = new_alg->s21::GraphAlgorithms::GetLeastSpanningTree(*model_);
    delete new_alg;

  } else {
    std::cerr << "Please, load the file first." << std::endl;
  }
  return matrix;
}

void ExampleController::Sale() {
  TsmResult res;
  if (model_->GetDataSize() > 1) {
    try {
      s21::GraphAlgorithms *new_alg = new s21::GraphAlgorithms;
      res = new_alg->SolveTravelingSalesmanProblem(*model_);
      std::cout << "Vertices: ";
      new_alg->PrintVector(res.vertices);
      std::cout << std::endl;
      std::cout << "Distance: " << res.distance;
      std::cout << res.distance << std::endl;
      delete new_alg;
    } catch (std::logic_error &err) {
      std::cerr << err.what() << std::endl;
    }

  } else {
    std::cerr << "Please, load the file first." << std::endl;
  }
}

void ExampleController::Dot(std::string filename) {
  if (model_->GetDataSize() > 1) {
    model_->ExportGraphToDot(filename);
    std::ifstream f(filename);

    if (f.is_open()) std::cout << f.rdbuf();
    std::cout << std::endl;
  } else {
    std::cerr << "Please, load the file first." << std::endl;
  }
  // make it print
}
}  // namespace s21