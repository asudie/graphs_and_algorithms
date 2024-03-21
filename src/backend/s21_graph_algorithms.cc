#include "s21_graph_algorithms.h"

namespace s21 {

std::vector<int> GraphAlgorithms::DepthFirstSearch(const Graph& graph,
                                                   int start_vertex) {
  graph_ = graph.GetGraph();
  if (start_vertex < 1 || start_vertex > (int)graph_.size())
    throw std::logic_error("There is no such vertex!");
  s21::Stack<int> stack;
  route_.clear();
  std::vector<int> visited(graph_.size(), 0);
  stack.push(start_vertex - 1);
  while (!stack.empty()) {
    int current_node = stack.top();
    stack.pop();
    if (visited[current_node] == 2) continue;
    visited[current_node] = 2;
    for (int j = (int)graph_.size() - 1; j >= 0; j--) {
      if (graph_[current_node][j] > 0 && visited[j] != 2) {
        stack.push(j);
        visited[j] = 1;
      }
    }
    route_.push_back(current_node + 1);
  }
  if (route_.empty()) throw std::logic_error("This way is empty!");
  return route_;
}

std::vector<int> GraphAlgorithms::BreadthFirstSearch(const Graph& graph,
                                                     int start_vertex) {
  graph_ = graph.GetGraph();
  if (start_vertex < 1 || start_vertex > (int)graph_.size())
    throw std::logic_error("There is no such vertex!");
  s21::Queue<int> queue;
  route_.clear();
  std::vector<int> visited(graph_.size(), 0);
  queue.push(start_vertex - 1);
  while (!queue.empty()) {
    int current_node = queue.front();
    queue.pop();
    if (visited[current_node] == 2) continue;
    visited[current_node] = 2;
    for (int j = 0; j < (int)graph_.size(); j++) {
      if (graph_[current_node][j] > 0 && visited[j] == 0) {
        queue.push(j);
        visited[j] = 1;
      }
    }
    route_.push_back(current_node + 1);
  }
  if (route_.empty()) throw std::logic_error("This way is empty!");
  return route_;
}

void GraphAlgorithms::MakeAdjacencyList(
    std::vector<std::vector<std::pair<int, int>>>& graph_list) {
  int vertex = graph_.size();
  for (int i = 0; i < vertex; ++i) {
    for (int j = 0; j < vertex; ++j) {
      if (graph_[i][j] > 0) {
        graph_list[i].push_back(std::pair<int, int>(j, graph_[i][j]));
      }
    }
  }
}

int GraphAlgorithms::GetShortestPathBetweenVertices(const Graph& graph,
                                                    int start, int finish) {
  graph_ = graph.GetGraph();
  if (start < 1 || start > (int)graph_.size() || finish < 1 ||
      finish > (int)graph_.size())
    throw std::logic_error("There is no way between these points!!");
  if (start == finish) throw std::logic_error("This way is empty!");
  std::vector<std::vector<std::pair<int, int>>> graph_list(graph_.size());
  MakeAdjacencyList(graph_list);
  std::vector<int> path_from_start(graph_.size(),
                                   std::numeric_limits<int>::max());
  path_from_start[start - 1] = 0;
  std::priority_queue<std::pair<int, int>, std::vector<std::pair<int, int>>,
                      std::greater<std::pair<int, int>>>
      queue;
  queue.push({0, start - 1});
  while (!queue.empty()) {
    auto [cur_way, to] = queue.top();
    queue.pop();
    if (cur_way > path_from_start[to]) continue;
    for (auto [vertex, way] : graph_list[to]) {
      if (path_from_start[vertex] > path_from_start[to] + way) {
        path_from_start[vertex] = path_from_start[to] + way;
        queue.push({path_from_start[vertex], vertex});
      }
    }
  }
  if (path_from_start[finish - 1] == std::numeric_limits<int>::max())
    throw std::logic_error("There is no way between these points!");
  return path_from_start[finish - 1];
}

const std::vector<std::vector<int>>&
GraphAlgorithms::GetShortestPathsBetweenAllVertices(const Graph& graph) {
  graph_ = graph.GetGraph();
  int size = graph_.size();
  ChangeGraphEdge(0, std::numeric_limits<int>::max());
  for (int k = 0; k < size; ++k) {
    for (int i = 0; i < size; ++i) {
      for (int j = 0; j < size; ++j) {
        if ((graph_[i][k] < std::numeric_limits<int>::max()) &&
            (graph_[k][j] < std::numeric_limits<int>::max()) &&
            (graph_[i][j] > graph_[i][k] + graph_[k][j])) {
          graph_[i][j] = std::min(graph_[i][j], graph_[i][k] + graph_[k][j]);
        }
      }
    }
  }
  ChangeGraphEdge(std::numeric_limits<int>::max(), 0);
  return graph_;
}

void GraphAlgorithms::ChangeGraphEdge(int from, int to) {
  int size = graph_.size();
  for (int i = 0; i < size; ++i) {
    for (int j = 0; j < size; j++) {
      if (i != j && graph_[i][j] == from) {
        graph_[i][j] = to;
      }
    }
  }
}

void GraphAlgorithms::PrintRoute() {
  std::cout << "Path";
  for (size_t i = 0; i < route_.size(); ++i) {
    std::cout << " " << route_[i];
  }
  std::cout << std::endl;
}

void GraphAlgorithms::PrintVector(std::vector<int> vec) {
  std::cout << "Vector";
  for (auto it = vec.begin(); it != vec.end(); ++it) {
    std::cout << ' ' << *it;
  }
  std::cout << std::endl;
}

void GraphAlgorithms::PrintMatrix(std::vector<std::vector<int>> v) {
  std::cout << "Matrix of Data:" << std::endl;
  for (int i = 0; i < (int)v.size(); ++i) {
    for (int j = 0; j < (int)v.size(); ++j) {
      std::cout << v[i][j] << " ";
    }
    std::cout << std::endl;
  }
}

std::vector<std::vector<int>> GraphAlgorithms::GetLeastSpanningTree(
    Graph& graph) {
  std::vector<std::vector<int>> res;

  int size = graph.GetDataSize();
  int selected[size];
  graph_ = graph.GetGraph();

  for (int i = 0; i < size; i++) {
    std::vector<int> v1(size, 0);
    res.push_back(v1);
  }

  int num_edge = 0;
  memset(selected, false, sizeof(selected));

  int x;
  int y;
  selected[0] = true;
  while (num_edge < size - 1) {
    int min = INT_MAX;
    x = 0;
    y = 0;

    for (int i = 0; i < size; i++) {
      if (selected[i]) {
        for (int j = 0; j < size; j++) {
          if (!selected[j] && graph_[i][j]) {
            if (min > graph_[i][j]) {
              min = graph_[i][j];
              x = i;
              y = j;
            }
          }
        }
      }
    }
    selected[y] = true;
    res[x][y] = graph_[x][y];
    num_edge++;
  }
  return res;
}

TsmResult GraphAlgorithms::SolveTravelingSalesmanProblem(const Graph& graph) {
  AntAlgorithms ant;
  ant.CreateAntColony(graph);
  salesman_route_ = ant.GetBestAntRoute();
  return salesman_route_;
}

void GraphAlgorithms::PrintSalesmanRoute() {
  std::cout << "Distance = " << salesman_route_.distance << std::endl;
  std::cout << "Path";
  for (size_t i = 0; i < salesman_route_.vertices.size(); ++i) {
    std::cout << " " << salesman_route_.vertices[i] + 1;
  }
  std::cout << std::endl;
}

/////////////////

void AntAlgorithms::CreateAntColony(const Graph& graph) {
  graph_ = graph.GetGraph();
  size_ = graph_.size();
  proximity_matrix_.resize(size_,
                           std::vector<std::pair<double, double>>(size_));
  MakeProximityMatrix();
  pheromone_by_iteration_.resize(size_, std::vector<double>(size_ + 1, 0));
  salesman_route_.distance = std::numeric_limits<int>::max();
  salesman_route_.vertices.resize(size_ + 1);
}

// начальная матрица близости и феромонов
void AntAlgorithms::MakeProximityMatrix() {
  for (int i = 0; i < size_; ++i) {
    for (int j = 0; j < size_; ++j) {
      if (i != j) {
        if (graph_[i][j] == 0)
          graph_[i][j] =
              std::numeric_limits<int>::max();  // добавляем недостающие ребра
                                                // максимальной длины
        proximity_matrix_[i][j].first = pow(kQ / graph_[i][j], kBeta);
        proximity_matrix_[i][j].second = kPheromone;
      }
    }
  }
}

// расчет добавочного феромона по пути муравья
void AntAlgorithms::CountPheromoneByAntRoute(TsmResult route) {
  for (int i = 0; i < size_; i++) {
    pheromone_by_iteration_[route.vertices[i]][route.vertices[i + 1]] +=
        kQ / route.distance;
  }
}

// пересчет желания посетить точку при выборе из оставшихся вершин
void AntAlgorithms::CalculateChanceVisitVertexes(
    std::vector<std::pair<int, double>>& availible_vertex, int from) {
  std::vector<double> tmp(availible_vertex.size(), 0);
  double total_desire = 0;
  for (int to = 0; to < (int)availible_vertex.size(); ++to) {
    tmp[to] =
        (proximity_matrix_[from][availible_vertex[to].first].first) *
        pow(proximity_matrix_[from][availible_vertex[to].first].second, kAlpha);
    total_desire += tmp[to];
  }
  for (int to = 0; to < (int)availible_vertex.size(); ++to) {
    availible_vertex[to].second = tmp[to] / total_desire;
  }
}

// рандомный выбор
double AntAlgorithms::GenerateRandomChoice() {
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dist(0.0, 1.0);
  return dist(gen);
}

// выбор следующей вершины
int AntAlgorithms::ChooseNextVertex(
    std::vector<std::pair<int, double>>& vertexes) {
  double choosing_vertex = GenerateRandomChoice();
  for (int i = 1; i < (int)vertexes.size(); ++i) {
    vertexes[i].second += vertexes[i - 1].second;
  }
  for (int i = 0; i < (int)vertexes.size(); ++i) {
    if (choosing_vertex <= vertexes[i].second) {
      return vertexes[i].first;
    }
  }
  return -1;
}

// сборка для одного муравья
void AntAlgorithms::GetAntRoute(int start) {
  std::vector<bool> visited(size_, false);
  visited[start] = true;
  TsmResult ant_route;
  ant_route.vertices.resize(size_ + 1);
  ant_route.vertices[0] = start;
  ant_route.distance = 0;
  int step = start;
  int count = 1;
  while (count <= size_) {
    std::vector<std::pair<int, double>>
        availible_vertex;  // только доступные из этой вершины!!!
    for (int i = 0; i < size_; ++i) {
      if (graph_[step][i] != 0 && !visited[i]) {
        availible_vertex.push_back({i, 0});
      }
    }
    if (availible_vertex.empty())
      break;
    else if (availible_vertex.size() == 1)
      step = availible_vertex[0].first;
    else {
      CalculateChanceVisitVertexes(availible_vertex, step);
      step = ChooseNextVertex(availible_vertex);
    }
    ant_route.vertices[count] = step;
    ant_route.distance += graph_[ant_route.vertices[count - 1]][step];
    visited[step] = true;
    ++count;
    if (count == size_) visited[start] = false;
  }
  if (IsAllVertexesVisited(visited)) {
    CountPheromoneByAntRoute(ant_route);
    UpdateSalesmanRoute(ant_route);
  }
}

void AntAlgorithms::UpdateSalesmanRoute(TsmResult& res) {
  if (res.distance < salesman_route_.distance) {
    salesman_route_.distance = res.distance;
    salesman_route_.vertices.swap(res.vertices);
  }
}

bool AntAlgorithms::IsAllVertexesVisited(std::vector<bool>& visited) {
  for (int i = 0; i < (int)visited.size(); ++i) {
    if (!visited[i]) return false;
  }
  return true;
}

bool AntAlgorithms::IsAllVertexesVisited() {
  std::vector<bool> visited(salesman_route_.vertices.size(), false);
  for (int i = 0; i < (int)salesman_route_.vertices.size(); ++i) {
    if (i == (int)salesman_route_.vertices.size() - 1)
      visited[salesman_route_.vertices.size() - 1] = true;
    visited[salesman_route_.vertices[i]] = true;
  }
  return IsAllVertexesVisited(visited);
}

void AntAlgorithms::UpdatePheromone() {
  for (int i = 0; i < size_; ++i) {
    for (int j = 0; j < size_; ++j) {
      proximity_matrix_[i][j].second =
          proximity_matrix_[i][j].second * kEvaporation +
          pheromone_by_iteration_[i][j];
      pheromone_by_iteration_[i][j] = 0;  // обнуляем после каждой итерации
    }
  }
}

TsmResult AntAlgorithms::GetBestAntRoute() {
  int iteration = 0;
  int ant_colony_size = 2 * size_;
  while (iteration < kMaxIteration) {
    for (int member = 0; member < ant_colony_size; member++) {
      GetAntRoute(member % size_);
    }
    iteration++;
    UpdatePheromone();
  }
  if (!IsAllVertexesVisited())
    throw std::logic_error("Graph does not contain a cycle!");
  return salesman_route_;
}

}  // namespace s21