#pragma once

#include <memory>
#include <vector>
#include "Vertex.h"

class Face;

class HalfEdge : public std::enable_shared_from_this<HalfEdge> {
public:
  HalfEdge(Vertex *vertex, Face *face);
  
  Face *face();
  double length() const;
  Vertex *vertex() const;
  Vertex *prevVertex() const;

  std::shared_ptr<HalfEdge> next();
  std::weak_ptr<HalfEdge> opposite();
  std::weak_ptr<HalfEdge> prev();

  void clearNext();
  void setFace(Face *face);
  void setNext(std::shared_ptr<HalfEdge> next);
  void setOpposite(std::weak_ptr<HalfEdge> opposite);
  
  std::vector<Vertex *> getNeighbors();

private:
  Face *face_;
  std::shared_ptr<HalfEdge> next_;
  std::weak_ptr<HalfEdge> opposite_;
  std::weak_ptr<HalfEdge> prev_;
  Vertex *vertex_;
};
