using namespace s21;

template <typename value_type>
class List<value_type>::ListIterator {
 public:
  friend class List;

  ListIterator(NodeList* ptr_ = nullptr) : ptr_(ptr_) {}

  reference operator*() const noexcept { return ptr_->data_; }

  bool operator==(const ListIterator& it) const noexcept {
    return (ptr_ == it.ptr_);
  }

  bool operator!=(const ListIterator& it) const noexcept {
    return (ptr_ != it.ptr_);
  }

  ListIterator& operator++() {
    NodeList* temp = ptr_;
    if (temp->next_ != nullptr) {
      ptr_ = temp->next_;
    } else {
      while (temp->prev_ != nullptr) {
        temp = temp->prev_;
        ptr_ = temp;
      }
    }
    return *this;
  }

  ListIterator& operator--() {
    NodeList* temp = ptr_;
    if (temp->prev_ != nullptr) {
      ptr_ = temp->prev_;
    } else {
      while (temp->next_ != nullptr) {
        temp = temp->next_;
        ptr_ = temp;
      }
    }
    return *this;
  };

 private:
  NodeList* ptr_;
};

template <typename value_type>
class List<value_type>::ListConstIterator {
 public:
  friend class List;

  ListConstIterator(NodeList* ptr_ = nullptr) : ptr_(ptr_) {}

  const_reference operator*() const noexcept { return ptr_->data_; }

  bool operator==(const ListConstIterator& it) const noexcept {
    return (ptr_ == it.ptr_);
  }

  bool operator!=(const ListConstIterator& it) const noexcept {
    return (ptr_ != it.ptr_);
  }

  ListConstIterator& operator++() {
    NodeList* temp = ptr_;
    if (temp->next_ != nullptr) {
      ptr_ = temp->next_;
    } else {
      while (temp->prev_ != nullptr) {
        temp = temp->prev_;
        ptr_ = temp;
      }
    }
    return *this;
  }

  ListConstIterator& operator--() {
    NodeList* temp = ptr_;
    if (temp->prev_ != nullptr) {
      ptr_ = temp->prev_;
    } else {
      while (temp->next_ != nullptr) {
        temp = temp->next_;
        ptr_ = temp;
      }
    }
    return *this;
  }

 private:
  NodeList* ptr_;
};

template <typename value_type>
typename List<value_type>::iterator List<value_type>::begin() noexcept {
  return iterator(begin_);
}

template <typename value_type>
typename List<value_type>::iterator List<value_type>::end() noexcept {
  return iterator(end_);
}

template <typename value_type>
typename List<value_type>::const_iterator List<value_type>::begin()
    const noexcept {
  return const_iterator(begin_);
}

template <typename value_type>
typename List<value_type>::const_iterator List<value_type>::end()
    const noexcept {
  return const_iterator(end_);
}

template <typename value_type>
typename List<value_type>::iterator List<value_type>::insert(
    iterator pos, const_reference value) {
  NodeList* new_node;
  new_node = new NodeList;
  new_node->data_ = value;
  if (pos.ptr_ == begin_) {
    new_node->prev_ = nullptr;
    new_node->next_ = begin_;
    begin_->prev_ = new_node;
    begin_ = new_node;
  } else {
    new_node->prev_ = pos.ptr_->prev_;
    pos.ptr_->prev_->next_ = new_node;
    new_node->next_ = pos.ptr_;
    pos.ptr_->prev_ = new_node;
  }
  ++size_;
  pos.ptr_ = new_node;
  return pos;
}

template <typename value_type>
void List<value_type>::erase(iterator pos) {
  DeleteNode(pos);
}

template <typename value_type>
typename List<value_type>::iterator List<value_type>::DeleteNode(iterator pos) {
  NodeList* current = pos.ptr_;
  if (size_ > 0) {
    if (size_ == 1) {
      end_->prev_ = nullptr;
      begin_ = end_;
      pos.ptr_ = end_;
    } else if (current == begin_) {
      current->next_->prev_ = nullptr;
      begin_ = current->next_;
      pos.ptr_ = current->next_;
    } else {
      current->next_->prev_ = pos.ptr_->prev_;
      current->prev_->next_ = pos.ptr_->next_;
      pos.ptr_ = current->next_;
    }
    --size_;
    delete current;
  }
  return pos;
}

template <typename value_type>
void List<value_type>::SwapPtr(NodeList*& s1, NodeList*& s2) {
  NodeList* tmp = s1;
  s1 = s2;
  s2 = tmp;
}

template <typename value_type>
List<value_type>::List() {
  NodeList* empty_last_node;
  empty_last_node = new NodeList;
  empty_last_node->data_ = value_type();
  empty_last_node->next_ = nullptr;
  empty_last_node->prev_ = nullptr;
  begin_ = empty_last_node;
  end_ = empty_last_node;
  size_ = 0;
}

template <typename value_type>
List<value_type>::List(size_type n) : List() {
  while (n) {
    push_back(value_type());
    --n;
  }
}

template <typename value_type>
List<value_type>::List(std::initializer_list<value_type> const& items)
    : List() {
  for (const auto& element : items) push_back(element);
}

template <typename value_type>
List<value_type>::List(const List& ls) : List() {
  *this = ls;
}

template <typename value_type>
List<value_type>::List(List&& ls) : List() {
  swap(ls);
}

template <typename value_type>
List<value_type>& List<value_type>::operator=(const List& ls) {
  if (this != &ls) {
    clear();
    for (const auto& el : ls) push_back(el);
  }
  return *this;
}

template <typename value_type>
List<value_type>& List<value_type>::operator=(List&& ls) {
  swap(ls);
  return *this;
}

template <typename value_type>
List<value_type>::~List() {
  clear();
  NodeList* tmp = end_;
  delete tmp;
}

template <typename value_type>
void List<value_type>::clear() {
  while (size_) {
    pop_back();
  }
}

template <typename value_type>
void List<value_type>::push_front(const_reference value) {
  insert(begin_, value);
}

template <typename value_type>
void List<value_type>::push_back(const_reference value) {
  insert(end_, value);
}

template <typename value_type>
void List<value_type>::pop_front() {
  erase(begin_);
}

template <typename value_type>
void List<value_type>::pop_back() {
  iterator pos = end();
  --pos;
  erase(pos);
}

template <typename value_type>
void List<value_type>::reverse() {
  if (size_ > 1) {
    NodeList* current = end_->prev_;
    NodeList* prev;
    while (current != nullptr) {
      prev = current->prev_;
      SwapPtr(current->prev_, current->next_);
      current = prev;
    }
    SwapPtr(end_->prev_, begin_);
    end_->prev_->next_ = end_;
  }
}

template <typename value_type>
void List<value_type>::unique() {
  iterator it = begin();
  iterator prev = it;
  ++it;
  while (it != end()) {
    if (*prev == *it) {
      it = DeleteNode(it);
    } else {
      prev = it;
      ++it;
    }
  }
}

template <typename value_type>
typename List<value_type>::iterator List<value_type>::FindMin() {
  iterator min_iter = begin();
  if (size_ > 1) {
    iterator it = begin();
    ++it;
    while (it != end()) {
      if (*it < *min_iter) {
        min_iter = it;
      }
      ++it;
    }
  }
  return min_iter;
}

template <typename value_type>
void List<value_type>::sort() {
  if (size_ > 1) {
    List<value_type> sorted_list;
    iterator it;
    while (size_) {
      it = FindMin();
      sorted_list.push_back(*it);
      erase(it);
    }
    *this = std::move(sorted_list);
  }
}

template <typename value_type>
void List<value_type>::merge(List& ls) {
  if (size_ > 0 || ls.size_ > 0) {
    List<value_type> sorted_list;
    iterator it = begin();
    iterator it_ls = ls.begin();
    while (size_ || ls.size_) {
      if (size_ == 0) {
        sorted_list.push_back(*it_ls);
        it_ls = ls.DeleteNode(it_ls);
      } else if (ls.size_ == 0) {
        sorted_list.push_back(*it);
        it = DeleteNode(it);
      } else if (*it < *it_ls) {
        sorted_list.push_back(*it);
        it = DeleteNode(it);
      } else {
        sorted_list.push_back(*it_ls);
        it_ls = ls.DeleteNode(it_ls);
      }
    }
    *this = std::move(sorted_list);
  }
}

template <typename value_type>
void List<value_type>::splice(iterator pos, List& ls) {
  if (this != &ls) {
    iterator it_ls = ls.begin();
    while (it_ls != ls.end()) {
      insert(pos, *it_ls);
      ls.erase(it_ls);
      it_ls = ls.begin();
    }
  }
}

template <typename value_type>
void List<value_type>::swap(List& ls) {
  if (this != &ls) {
    SwapPtr(begin_, ls.begin_);
    SwapPtr(end_, ls.end_);
    if (size_ != ls.size_) {
      std::swap(size_, ls.size_);
    }
  }
}

template <typename value_type>
typename List<value_type>::reference List<value_type>::front() noexcept {
  return begin_->data_;
}
template <typename value_type>
typename List<value_type>::const_reference List<value_type>::front()
    const noexcept {
  return begin_->data_;
}

template <typename value_type>
typename List<value_type>::reference List<value_type>::back() noexcept {
  if (size_ > 0)
    return end_->prev_->data_;
  else
    return end_->data_;
}
template <typename value_type>
typename List<value_type>::const_reference List<value_type>::back()
    const noexcept {
  if (size_ > 0)
    return end_->prev_->data_;
  else
    return end_->data_;
}

template <typename value_type>
bool List<value_type>::empty() const noexcept {
  return (size_ == 0);
}

template <typename value_type>
typename List<value_type>::size_type List<value_type>::size() const noexcept {
  return size_;
}

template <typename value_type>
typename List<value_type>::size_type List<value_type>::max_size()
    const noexcept {
  return SIZE_MAX / sizeof(NodeList) / 2;
}
