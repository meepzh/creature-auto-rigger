#include "HalfEdge.h"

#include <cassert>

HalfEdge::HalfEdge(Vertex *vertex, Face *face)
    : vertex_(vertex), face_(face) {
}

Face *HalfEdge::face() {
  return face_;
}

double HalfEdge::length() const {
  return (vertex_->point() - prev_.lock()->vertex()->point()).length();
}

Vertex *HalfEdge::vertex() const {
  return vertex_;
}

Vertex *HalfEdge::prevVertex() const {
  if (!prev_.expired()) return prev_.lock()->vertex();
  return nullptr;
}

std::shared_ptr<HalfEdge> HalfEdge::next() {
  return next_;
}

std::weak_ptr<HalfEdge> HalfEdge::opposite() {
  return opposite_;
}

std::weak_ptr<HalfEdge> HalfEdge::prev() {
  return prev_;
}

void HalfEdge::clearNext() {
  next_->prev_.reset();
  next_.reset();
}

void HalfEdge::setFace(Face *face) {
  face_ = face;
}

void HalfEdge::setNext(std::shared_ptr<HalfEdge> next) {
  next_ = next;
  next_->prev_ = shared_from_this();
}

void HalfEdge::setOpposite(std::weak_ptr<HalfEdge> opposite) {
  assert(!(opposite.expired()));
  opposite_ = opposite;
  opposite_.lock()->opposite_ = shared_from_this();
}

std::vector<Vertex *> HalfEdge::getNeighbors() {
  std::vector<Vertex *> neighbors;

  std::shared_ptr<HalfEdge> endEdge = next_;
  std::shared_ptr<HalfEdge> curEdge = endEdge;

  do {
    neighbors.push_back(curEdge->vertex());
    curEdge = curEdge->opposite().lock()->next();
  } while (curEdge != endEdge);

  neighbors.shrink_to_fit();
  return neighbors;
}
