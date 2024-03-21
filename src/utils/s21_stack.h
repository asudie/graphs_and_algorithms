#ifndef S21_STACK_H_
#define S21_STACK_H_
#include <cstddef>
#include <exception>
#include <initializer_list>
#include <iostream>

#include "s21_list.h"
namespace s21 {
template <typename T, typename List = List<T>>
class Stack {
 public:
  using value_type = T;
  using reference = T&;
  using const_reference = typename List::const_reference;
  using size_type = typename List::size_type;
  Stack() : lst_(){};
  Stack(std::initializer_list<value_type> const& items) : lst_(items){};
  Stack(const Stack& s) : lst_(s.lst_){};
  Stack(Stack&& s) noexcept : lst_(std::move(s.lst_)){};
  ~Stack() = default;

  Stack& operator=(const Stack& s) {
    lst_ = s.lst_;
    return *this;
  };
  Stack& operator=(Stack&& s) noexcept {
    lst_ = std::move(s.lst_);
    return *this;
  };

  bool empty() { return lst_.empty(); };
  size_type size() { return lst_.size(); };
  const_reference top() { return lst_.back(); };
  void push(const_reference value) { lst_.push_back(value); };
  void pop() { lst_.pop_back(); };
  void swap(Stack& other) { lst_.swap(other.lst_); };

 private:
  List lst_;
};
}  // namespace s21
#endif  // S21_STACK_H_
