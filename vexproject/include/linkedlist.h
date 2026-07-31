#ifndef ROBOT_LINKEDLIST_H
#define ROBOT_LINKEDLIST_H

#include <stdint.h>

// the trail of waypoints the robot has driven through: one node per direction
// change, in the order they happened. doubly linked so it can be walked from
// either end, and the tail pointer keeps appending O(1) rather than walking the
// whole list every time.

struct Node {
  Node* next;
  Node* prev;
  float xPos;
  float yPos;
  float headingDeg;
};

class LinkedList {
  public:
    LinkedList() : root_(nullptr), tail_(nullptr), numNodes_(0) {}

    ~LinkedList() {
      Node* currentNode = root_;
      while (currentNode != nullptr) {
        Node* nextNode = currentNode->next;
        delete currentNode;
        currentNode = nextNode;
      }
      root_ = nullptr;
      tail_ = nullptr;
      numNodes_ = 0;
    }

    void add(float x, float y, float headingDeg) {
      Node* newNode = new Node;
      newNode->next = nullptr;
      newNode->prev = tail_;
      newNode->xPos = x;
      newNode->yPos = y;
      newNode->headingDeg = headingDeg;

      if (root_ == nullptr) {
        root_ = newNode;
      } else {
        tail_->next = newNode;
      }
      tail_ = newNode;
      numNodes_++;
    }

    const Node* first() const { return root_; }
    const Node* last() const { return tail_; }
    uint8_t size() const { return numNodes_; }

    // cause a compiler error if the copy constructor or assignment operator is
    // called on this class, to prevent use after free and nullptr dereference
    // bugs
    LinkedList(const LinkedList&) = delete;
    LinkedList& operator=(const LinkedList&) = delete;

  private:
    Node* root_;
    Node* tail_;
    uint8_t numNodes_;
};

#endif  // ROBOT_LINKEDLIST_H
