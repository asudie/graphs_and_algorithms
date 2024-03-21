#ifndef S21_GRAPH_ALGORITHMS_H_
#define S21_GRAPH_ALGORITHMS_H_

#include <climits>
#include <cstring>
#include <exception>
#include <initializer_list>
#include <limits>
#include <queue>
#include <random>
#include <vector>

#include "../utils/s21_queue.h"
#include "../utils/s21_stack.h"
#include "s21_graph.h"

namespace s21 {

struct TsmResult {
  std::vector<int> vertices;
  double distance;
};

class GraphAlgorithms {
 public:
  GraphAlgorithms(){};
  ~GraphAlgorithms(){};

  void PrintRoute();
  void PrintSalesmanRoute();
  void PrintVector(std::vector<int>);
  std::vector<int> DepthFirstSearch(const Graph& graph, int start_vertex);
  std::vector<int> BreadthFirstSearch(const Graph& graph, int start_vertex);
  int GetShortestPathBetweenVertices(const Graph& graph, int vertex1,
                                     int vertex2);
  const std::vector<std::vector<int>>& GetShortestPathsBetweenAllVertices(
      const Graph& graph);

  std::vector<std::vector<int>> GetLeastSpanningTree(Graph& graph);
  TsmResult SolveTravelingSalesmanProblem(const Graph& graph);

 private:
  void MakeAdjacencyList(
      std::vector<std::vector<std::pair<int, int>>>& graph_list);
  void ChangeGraphEdge(int from, int to);
  void PrintMatrix(std::vector<std::vector<int>> res);
  //   int check_vertex(Graph &graph, std::vector<int> visited);
  //   int path_result;
  std::vector<std::vector<int>> graph_;
  std::vector<int> route_{};
  TsmResult salesman_route_;
};

class AntAlgorithms {
 public:
  AntAlgorithms(){};
  void CreateAntColony(const Graph& graph);
  TsmResult GetBestAntRoute();
  ~AntAlgorithms(){};

 private:
  void MakeProximityMatrix();
  void CountPheromoneByAntRoute(TsmResult route);
  void CalculateChanceVisitVertexes(
      std::vector<std::pair<int, double>>& vertexes, int from);
  void GetAntRoute(int start);
  double GenerateRandomChoice();
  int ChooseNextVertex(std::vector<std::pair<int, double>>& vertexes);
  bool IsAllVertexesVisited(std::vector<bool>& visited);
  bool IsAllVertexesVisited();
  void UpdatePheromone();
  void UpdateSalesmanRoute(TsmResult& res);
  std::vector<std::vector<std::pair<double, double>>> proximity_matrix_;
  std::vector<std::vector<int>> graph_;
  std::vector<std::vector<double>> pheromone_by_iteration_;
  TsmResult salesman_route_;
  int size_;
  const double kPheromone = 0.3;
  const double kEvaporation = 0.9;
  const double kQ = 10.;
  const double kAlpha = 1.;
  const double kBeta = 2.;
  const int kMaxIteration = 300;
};

}  // namespace s21
#endif  // S21_GRAPH_ALGORITHMS_H_