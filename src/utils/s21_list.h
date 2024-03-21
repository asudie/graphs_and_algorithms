#ifndef S21_LIST_H_
#define S21_LIST_H_
#include <cstddef>
#include <exception>
#include <initializer_list>
#include <iostream>
namespace s21 {
template <typename T>
class List {
 public:
  class ListIterator;
  class ListConstIterator;
  using value_type = T;
  using reference = T&;
  using const_reference = const T&;
  using iterator = ListIterator;
  using const_iterator = ListConstIterator;
  using size_type = size_t;
  List();
  explicit List(size_type n);
  List(std::initializer_list<value_type> const& items);
  List(const List& ls);
  List(List&& ls);
  List& operator=(const List& ls);
  List& operator=(List&& ls);

  ~List();
  iterator begin() noexcept;
  iterator end() noexcept;
  iterator insert(iterator pos, const_reference value);
  void erase(iterator pos);
  const_iterator begin() const noexcept;
  const_iterator end() const noexcept;

  void clear();
  void push_back(const_reference value);
  void push_front(const_reference value);
  void pop_back();
  void pop_front();
  void reverse();
  void unique();
  void sort();
  void merge(List& ls);
  void splice(iterator pos, List& ls);
  void swap(List& ls);

  reference front() noexcept;
  const_reference front() const noexcept;
  reference back() noexcept;
  const_reference back() const noexcept;

  size_type size() const noexcept;
  size_type max_size() const noexcept;
  bool empty() const noexcept;

 private:
  struct NodeList {
    value_type data_;
    NodeList* next_;
    NodeList* prev_;
  } * end_, *begin_;
  size_type size_;

  iterator DeleteNode(iterator pos);
  iterator FindMin();
  void SwapPtr(NodeList*& s1, NodeList*& s2);
};

#include "s21_list.inl"
}  // namespace s21
#endif  // S21_LIST_H_
