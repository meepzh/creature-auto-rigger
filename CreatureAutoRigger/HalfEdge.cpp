#include "HalfEdge.h"

HalfEdge::HalfEdge(Vertex *vertex, Face *face)
    : vertex_(vertex), face_(face),
      next_(nullptr), opposite_(nullptr), prev_(nullptr) {
}

Face *HalfEdge::face() {
  return face_;
}

double HalfEdge::length() const {
  return (vertex_->point() - prev_->vertex()->point()).length();
}

Vertex *HalfEdge::vertex() const {
  return vertex_;
}

Vertex *HalfEdge::prevVertex() const {
  if (prev_) return prev_->vertex();
  return nullptr;
}

HalfEdge *HalfEdge::next() {
  return next_.get();
}

HalfEdge *HalfEdge::opposite() {
  return opposite_;
}

HalfEdge *HalfEdge::prev() {
  return prev_;
}

void HalfEdge::clearNext() {
  next_->prev_ = nullptr;
  next_.reset();
}

void HalfEdge::setFace(Face *face) {
  face_ = face;
}

void HalfEdge::setNext(std::shared_ptr<HalfEdge> &next) {
  next_ = next;
  next_->prev_ = this;
}

void HalfEdge::setOpposite(HalfEdge *opposite) {
  opposite_ = opposite;
  opposite_->opposite_ = this;
}
