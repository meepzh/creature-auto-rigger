#pragma once

#include <memory>
#include "Vertex.h"

class Face;

class HalfEdge {
public:
  HalfEdge(Vertex *vertex, Face *face);
  
  Face *face();
  double length() const;
  Vertex *vertex() const;

  HalfEdge *next();
  HalfEdge *opposite();
  HalfEdge *prev();

  void clearNext();
  void setNext(std::shared_ptr<HalfEdge> &next);
  void setOpposite(HalfEdge *opposite);

private:
  Face *face_;
  std::shared_ptr<HalfEdge> next_;
  HalfEdge *opposite_;
  HalfEdge *prev_;
  Vertex *vertex_;
};
