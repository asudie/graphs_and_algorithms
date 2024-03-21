#include "console_view.h"

#include <iostream>

namespace s21 {

using std::cin;
using std::cout;
using std::endl;

void ConsoleView::DisplayMenu() {
  std::cout << "=========" << std::endl;
  std::cout << " M E N U " << std::endl;
  std::cout << "=========" << std::endl;
  std::cout << "1. Load from a file" << std::endl;
  std::cout << "2. Graph traversal in breadth" << std::endl;
  std::cout << "3. Graph traversal in depth" << std::endl;
  std::cout << "4. Find the shortest path between any two vertices"
            << std::endl;
  std::cout << "5. Find the shortest path between all pairs of vertices"
            << std::endl;
  std::cout << "6. Find the minimal spanning tree" << std::endl;
  std::cout << "7. Solve the salesman problem" << std::endl;
  std::cout << "8. Export DOT" << std::endl << std::endl;
  std::cout << "0. Quit" << std::endl << std::endl;
}

int ConsoleView::PerformChoice() {
  char choice;
  int res = 0;
  std::cout << "Input a menu item digit: ";
  std::cin >> choice;
  res = atoi(&choice);
  if (choice != 48 && res == 0) res = -1;

  return res;
}

void ConsoleView::StartEventLoop() {
  while (true) {
    std::string filename;
    int start_vertex;
    int end_vertex;
    DisplayMenu();
    switch ((Choice)PerformChoice()) {
      case LOAD:
        std::cout << "Input a file name: ";
        std::cin >> filename;
        controller_->Load(filename);
        break;

      case BREADTH:
        std::cout << "Input a start vertex: ";
        if (!(std::cin >> start_vertex)) {
          std::cout << "Please input only non-negative integers.\n";
          std::cin.clear();
          std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        } else {
          controller_->Breadth(start_vertex);
        }

        break;

      case DEPTH:
        std::cout << "Input a start vertex: ";
        if (!(std::cin >> start_vertex)) {
          std::cout << "Please input only non-negative integers.\n";
          std::cin.clear();
          std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        } else {
          controller_->Depth(start_vertex);
        }

        break;

      case ALL:
        PrintMatrix(controller_->All());
        break;

      case TWO:
        std::cout << "Input a start vertex: ";
        if (!(std::cin >> start_vertex)) {
          std::cout << "Please input only non-negative integers.\n";
          std::cin.clear();
          std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        } else {
          std::cout << "Input an end vertex: ";
          if (!(std::cin >> end_vertex)) {
            std::cout << "Please input only non-negative integers.\n";
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
          } else {
            controller_->Two(start_vertex, end_vertex);
          }
        }
        break;

      case TREE:
        PrintMatrix(controller_->Tree());
        break;

      case SALE:
        controller_->Sale();
        break;

      case DOT:
        std::cout << "Input a file name to save: ";
        std::cin >> filename;
        controller_->Dot(filename);
        break;

      case EXIT:
        exit(1);

      default:
        std::cout << "Wrong menu item number!" << std::endl;
        continue;
    }
  }
}

void ConsoleView::PrintMatrix(std::vector<std::vector<int>> matrix) {
  std::cout << "Matrix of Data:" << std::endl;
  for (int i = 0; i < controller_->model_->GetDataSize(); ++i) {
    for (int j = 0; j < this->controller_->model_->GetDataSize(); ++j) {
      std::cout << matrix[i][j] << " ";
    }
    std::cout << std::endl;
  }
};

}  // namespace s21