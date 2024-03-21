#ifndef S21_QUEUE_H_
#define S21_QUEUE_H_

#include <cstddef>
#include <initializer_list>
#include <iostream>

#include "s21_list.h"
namespace s21 {
template <typename T>
class Queue : protected List<T> {
 public:
  using value_type = T;
  using size_type = std::size_t;
  using reference = T&;
  using const_reference = const T&;

  Queue() : List<T>() {}

  Queue(std::initializer_list<value_type> const& items) : List<T>(items) {}

  Queue(const Queue& q) : List<T>(q) {}

  Queue(Queue&& q) noexcept : List<T>(std::move(q)) {}

  Queue& operator=(Queue&& q) noexcept {
    List<T>::operator=(std::move(q));
    return *this;
  }

  void push(const_reference value) { this->push_back(value); }

  void pop() { this->pop_front(); }

  const_reference front() { return List<T>::front(); }
  const_reference back() { return List<T>::back(); };
  size_type size() { return List<T>::size(); }

  bool empty() { return List<T>::empty(); }

  void swap(Queue& other) { List<T>::swap(other); }

  Queue& operator=(const Queue& q) {
    List<T>::operator=(q);
    return *this;
  }
};
}  // namespace s21

#endif  //  S21_QUEUE_H_
