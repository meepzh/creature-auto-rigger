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
  assert(next);
  next_ = next;
  next_->prev_ = shared_from_this();
}

void HalfEdge::setOpposite(std::weak_ptr<HalfEdge> opposite) {
  assert(!(opposite.expired()));
  opposite_ = opposite;
  opposite_.lock()->opposite_ = shared_from_this();
}
