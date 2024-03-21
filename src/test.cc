#include <gtest/gtest.h>

#include <cstddef>
#include <iostream>
#include <stack>
#include <string>

#include "backend/s21_graph.h"
#include "backend/s21_graph_algorithms.h"

TEST(s21_graph, test1) {
  s21::Graph graph;
  std::string path = "test1.txt";
  graph.LoadGraphFromFile(path);
  graph.PrintMatrix();
}

TEST(s21_dfs, test1) {
  s21::Graph graph;
  std::string path = "test2.txt";
  graph.LoadGraphFromFile(path);
  s21::GraphAlgorithms route;
  std::vector<int> res = route.DepthFirstSearch(graph, 1);
  std::vector<int> orig{1, 2, 3, 5, 4, 8, 7, 6, 9, 10, 12, 11};
  route.PrintRoute();
  EXPECT_EQ(res, orig);
}

TEST(s21_bfs, test1) {
  s21::Graph graph;
  std::string path = "test2.txt";
  graph.LoadGraphFromFile(path);
  s21::GraphAlgorithms route;
  std::vector<int> res = route.BreadthFirstSearch(graph, 1);
  std::vector<int> orig{1, 2, 6, 7, 8, 10, 12, 3, 4, 9, 11, 5};
  route.PrintRoute();
  EXPECT_EQ(res, orig);
}

TEST(s21_dkstr, test1) {
  s21::Graph graph;
  std::string path = "test1.txt";
  graph.LoadGraphFromFile(path);
  s21::GraphAlgorithms route;
  int res = route.GetShortestPathBetweenVertices(graph, 2, 10);
  int orig = 41;
  EXPECT_EQ(res, orig);
}

TEST(s21_dkstr, test2) {
  s21::Graph graph;
  std::string path = "test3.txt";
  graph.LoadGraphFromFile(path);
  s21::GraphAlgorithms route;
  int res = route.GetShortestPathBetweenVertices(graph, 10, 11);
  int orig = 67;
  EXPECT_EQ(res, orig);
}

TEST(s21_fl_uor, test1) {
  s21::Graph graph;
  std::string path = "test3.txt";
  graph.LoadGraphFromFile(path);
  s21::GraphAlgorithms route;
  std::vector<std::vector<int>> res =
      route.GetShortestPathsBetweenAllVertices(graph);
  std::vector<std::vector<int>> orig = {
      {0, 29, 28, 29, 16, 32, 101, 19, 4, 0, 38},
      {49, 0, 42, 49, 28, 44, 72, 37, 52, 0, 50},
      {21, 15, 0, 15, 18, 33, 87, 9, 23, 0, 40},
      {25, 33, 18, 0, 4, 20, 92, 13, 25, 0, 26},
      {21, 29, 14, 21, 0, 16, 101, 9, 24, 0, 22},
      {40, 40, 60, 61, 56, 0, 112, 51, 36, 0, 78},
      {102, 72, 99, 102, 99, 114, 0, 90, 105, 0, 84},
      {12, 24, 9, 12, 9, 24, 96, 0, 15, 0, 31},
      {4, 29, 24, 25, 20, 36, 101, 15, 0, 0, 42},
      {39, 42, 27, 42, 45, 3, 114, 36, 35, 0, 67},
      {61, 12, 54, 61, 40, 37, 84, 49, 64, 0, 0}};
  EXPECT_EQ(res, orig);
}

TEST(s21_fl_uor, test2) {
  s21::Graph graph;
  std::string path = "test2.txt";
  graph.LoadGraphFromFile(path);
  s21::GraphAlgorithms route;
  std::vector<std::vector<int>> res =
      route.GetShortestPathsBetweenAllVertices(graph);
  std::vector<std::vector<int>> orig = {{0, 1, 2, 2, 2, 1, 1, 1, 2, 1, 2, 1},
                                        {1, 0, 1, 1, 2, 2, 2, 1, 2, 1, 2, 2},
                                        {2, 3, 0, 2, 1, 2, 1, 2, 1, 1, 2, 1},
                                        {1, 2, 1, 0, 2, 2, 2, 1, 2, 2, 1, 1},
                                        {1, 2, 1, 1, 0, 1, 2, 2, 2, 2, 1, 2},
                                        {1, 2, 2, 2, 2, 0, 2, 1, 1, 1, 2, 1},
                                        {2, 2, 2, 1, 2, 1, 0, 1, 2, 1, 1, 1},
                                        {1, 2, 1, 1, 1, 2, 1, 0, 1, 2, 2, 2},
                                        {2, 2, 2, 1, 3, 2, 1, 2, 0, 1, 1, 2},
                                        {1, 2, 2, 2, 2, 1, 1, 1, 1, 0, 2, 1},
                                        {1, 1, 1, 1, 2, 2, 2, 1, 2, 1, 0, 1},
                                        {2, 3, 2, 2, 1, 2, 1, 2, 1, 1, 2, 0}};
  EXPECT_EQ(res, orig);
}

TEST(s21_ant, test1) {
  s21::Graph graph;
  std::string path = "test4.txt";
  graph.LoadGraphFromFile(path);
  s21::GraphAlgorithms route;
  s21::TsmResult res = route.SolveTravelingSalesmanProblem(graph);
  route.PrintSalesmanRoute();
  double orig_distance = 40;
  EXPECT_DOUBLE_EQ(orig_distance, res.distance);
}

TEST(s21_ant, test2) {
  s21::Graph graph;
  std::string path = "test1.txt";
  graph.LoadGraphFromFile(path);
  s21::GraphAlgorithms route;
  s21::TsmResult res = route.SolveTravelingSalesmanProblem(graph);
  double orig_distance = 253;
  route.PrintSalesmanRoute();
  EXPECT_DOUBLE_EQ(orig_distance, res.distance);
}

TEST(s21_ant, test3) {
  s21::Graph graph;
  std::string path = "test5.txt";
  graph.LoadGraphFromFile(path);
  s21::GraphAlgorithms route;
  s21::TsmResult res = route.SolveTravelingSalesmanProblem(graph);
  double orig_distance = 87;
  route.PrintSalesmanRoute();
  EXPECT_DOUBLE_EQ(orig_distance, res.distance);
}

TEST(s21_ant, test4) {  // нет цикла
  s21::Graph graph;
  std::string path = "test3.txt";
  graph.LoadGraphFromFile(path);
  s21::GraphAlgorithms route;
  EXPECT_ANY_THROW(route.SolveTravelingSalesmanProblem(graph));
}

TEST(s21_ant, test5) {
  s21::Graph graph;
  std::string path = "test6.txt";
  graph.LoadGraphFromFile(path);
  s21::GraphAlgorithms route;
  s21::TsmResult res = route.SolveTravelingSalesmanProblem(graph);
  double orig_distance = 294;
  route.PrintSalesmanRoute();
  EXPECT_DOUBLE_EQ(orig_distance, res.distance);
}

TEST(s21_ant, test6) {  // нет цикла
  s21::Graph graph;
  std::string path = "test7.txt";
  graph.LoadGraphFromFile(path);
  s21::GraphAlgorithms route;
  EXPECT_ANY_THROW(route.SolveTravelingSalesmanProblem(graph));
}

TEST(s21_spanning_tree, test1) {
  s21::Graph graph;
  std::string path = "s_test.txt";
  std::cout << path << std::endl;
  graph.LoadGraphFromFile(path);
  s21::GraphAlgorithms route;
  std::vector<std::vector<int>> res = route.GetLeastSpanningTree(graph);
  std::vector<std::vector<int>> orig = {{0, 1, 1, 1, 0},
                                        {0, 0, 0, 0, 1},
                                        {0, 0, 0, 0, 0},
                                        {0, 0, 0, 0, 0},
                                        {0, 0, 0, 0, 0}};
  EXPECT_EQ(res, orig);
}

TEST(s21_export, test1) {
  s21::Graph graph;
  std::string path = "s_test.txt";
  std::cout << path << std::endl;
  graph.LoadGraphFromFile(path);
  graph.ExportGraphToDot("export.dot");
  std::ifstream t("export.dot");
  std::stringstream mybuffer;
  mybuffer << t.rdbuf();
  std::ifstream s("export");
  std::stringstream orbuffer;
  orbuffer << s.rdbuf();

  EXPECT_EQ(orbuffer.str(), mybuffer.str());
}

int main(int argc, char *argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
