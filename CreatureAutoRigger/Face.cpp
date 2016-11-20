#include "Face.h"

#include <algorithm>
#include <vector>
#include <cassert>

Face::Face()
  : area_(0), edge_(nullptr), hasOutside_(false),
    numVertices_(0), planeOffset_(0), flag(Flag::VISIBLE) {
}

Face::~Face() {
  std::shared_ptr<HalfEdge> curEdge = edge_;
  bool circularDependency = true;
  std::vector<HalfEdge *> seenEdges;
  bool unseenEdge = true;
  do {
    seenEdges.push_back(curEdge.get());
    if (!curEdge) {
      circularDependency = false;
      break;
    }
    curEdge = curEdge->next();
    unseenEdge = std::find(seenEdges.begin(), seenEdges.end(), curEdge.get()) == seenEdges.end();
  } while (curEdge != edge_ && unseenEdge);

  // Only care if face's edge is part of a circle
  circularDependency = circularDependency && curEdge == edge_;

  if (circularDependency && (edge_.use_count() == 2) && !(edge_->prev().expired())) {
    // Prevent circular reference
    edge_->prev().lock()->clearNext();
  }
}

void Face::checkConsistency(bool checkPtrCounts) {
  std::shared_ptr<HalfEdge> edge = edge_;
  assert(edge);

  int numVertices = 0;
  assert(numVertices_ >= 3);

  do {
    // Shared use count
    if (checkPtrCounts) {
      if (edge == edge_) {
        // Should only be referred to by this function, its prev edge, and its face
        assert(edge.use_count() <= 3);
      } else {
        // Should only be referred to by this function and its prev edge
        assert(edge.use_count() <= 2);
      }
    }

    // Opposite edge
    assert(!edge->opposite().expired());
    std::shared_ptr<HalfEdge> oppositeEdge = edge->opposite().lock();
    assert(oppositeEdge->opposite().lock() == edge);
    assert(oppositeEdge->vertex() == edge->prevVertex());
    assert(edge->vertex() == oppositeEdge->prevVertex());

    // Opposite face
    Face *oppositeFace = oppositeEdge->face();
    assert(oppositeFace);
    assert(oppositeFace->flag != Flag::DELETED);

    // Prev/next edges
    assert(!(edge->prev().expired()));
    assert(edge->next());
    assert(edge->next()->prev().lock() == edge);
    edge = edge->next();

    ++numVertices;
  } while (edge != edge_);

  assert(numVertices == numVertices_);
}

void Face::checkVertexCount() {
  unsigned int numVertices = 0;
  std::shared_ptr<HalfEdge> edge = edge_;
  do {
    ++numVertices;
    assert(numVertices <= numVertices_);
    edge = edge->next();
  } while (edge != edge_);
}

void Face::computeCentroid() {
  centroid_ = MPoint::origin;
  HalfEdge *faceEdge = edge_.get();
  HalfEdge *curEdge = faceEdge;
  do {
    centroid_ += curEdge->vertex()->point();
    curEdge = curEdge->next().get();
  } while (curEdge != faceEdge);
  centroid_ = centroid_ / (double) numVertices_;
}

void Face::computeNormal() {
  HalfEdge *faceEdge = edge_.get();
  HalfEdge *curEdge = faceEdge->next()->next().get();

  MVector v1;
  MVector v2 = faceEdge->next()->vertex()->point() - faceEdge->vertex()->point();

  normal_ = MVector::zero;
  numVertices_ = 2;

  while (curEdge != faceEdge) {
    v1 = v2;
    v2 = curEdge->vertex()->point() - faceEdge->vertex()->point();
    normal_ += v1 ^ v2;

    curEdge = curEdge->next().get();
    ++numVertices_;
  }

  area_ = normal_.length();
  normal_.normalize();
}

void Face::computeNormal(double minArea) {
  computeNormal();

  if (area_ < minArea) {
    // Recompute normal without the longest edge
    HalfEdge *faceEdge = edge_.get();
    HalfEdge *curEdge = faceEdge;
    HalfEdge *maxEdge = nullptr;
    double maxLength = 0;

    // Find the longest edge
    do {
      double length = curEdge->length();
      if (length > maxLength) {
        maxEdge = curEdge;
        maxLength = length;
      }
      curEdge = curEdge->next().get();
    } while (curEdge != faceEdge);

    // Recompute normal
    assert(!(curEdge->prev().expired()));
    MVector maxVector = curEdge->vertex()->point() - curEdge->prev().lock()->vertex()->point();
    maxVector.normalize();
    double maxProjection = normal_ * maxVector;
    normal_ += maxVector * (-maxProjection);
    normal_.normalize();
  }
}

void Face::mergeAdjacentFaces(std::shared_ptr<HalfEdge> adjacentEdge, std::vector<Face *> &discardedFaces) {
  assert(!(adjacentEdge->opposite().expired()));
  std::shared_ptr<HalfEdge> oppositeEdge = adjacentEdge->opposite().lock();
  Face *oppositeFace = oppositeEdge->face();

  discardedFaces.push_back(oppositeFace);
  oppositeFace->flag = Flag::DELETED;

  assert(!(adjacentEdge->prev().expired()));
  assert(!(oppositeEdge->prev().expired()));
  std::shared_ptr<HalfEdge> adjacentEdgePrev = adjacentEdge->prev().lock();
  std::shared_ptr<HalfEdge> adjacentEdgeNext = adjacentEdge->next();
  std::shared_ptr<HalfEdge> oppositeEdgePrev = oppositeEdge->prev().lock();
  std::shared_ptr<HalfEdge> oppositeEdgeNext = oppositeEdge->next();

  // Find opposite edge chain
  // Left edge
  assert(!(adjacentEdgePrev->opposite().expired()));
  while (adjacentEdgePrev->opposite().lock()->face() == oppositeFace) {
    assert(!(adjacentEdgePrev->prev().expired()));
    adjacentEdgePrev = adjacentEdgePrev->prev().lock();
    oppositeEdgeNext = oppositeEdgeNext->next();
  }

  // Right edge
  assert(!(adjacentEdgeNext->opposite().expired()));
  while (adjacentEdgeNext->opposite().lock()->face() == oppositeFace) {
    assert(!(oppositeEdgePrev->prev().expired()));
    adjacentEdgeNext = adjacentEdgeNext->next();
    oppositeEdgePrev = oppositeEdgePrev->prev().lock();
  }

  // Fix face references
  std::shared_ptr<HalfEdge> edge;
  for (edge = oppositeEdgeNext; edge != oppositeEdgePrev->next(); edge = edge->next()) {
    edge->setFace(this);
  }

  // Preserve with edge that won't be destroyed
  edge_ = adjacentEdgeNext;

  // Connect
  Face *discardedFace = connectHalfEdges(oppositeEdgePrev, adjacentEdgeNext);
  if (discardedFace) {
    discardedFaces.push_back(discardedFace);
  }
  discardedFace = connectHalfEdges(adjacentEdgePrev, oppositeEdgeNext);
  if (discardedFace) {
    discardedFaces.push_back(discardedFace);
  }

  computeNormalAndCentroid();
  checkConsistency();
}

double Face::pointPlaneDistance(const MPoint &pt) const {
  return normal_ * pt - planeOffset_;
}

std::shared_ptr<HalfEdge> Face::edge() {
  return edge_;
}

std::shared_ptr<HalfEdge> Face::edge(int i) {
  std::shared_ptr<HalfEdge> curEdge = edge_;

  while (i > 0) {
    curEdge = curEdge->next();
    --i;
  }
  while (i < 0) {
    curEdge = curEdge->prev().lock();
    ++i;
  }

  return curEdge;
}

void Face::setEdge(std::shared_ptr<HalfEdge> edge) {
  edge_ = edge;
}

double Face::area() const {
  return area_;
}

MPoint Face::centroid() const {
  return centroid_;
}

void Face::clearOutside() {
  hasOutside_ = false;
}

bool Face::hasOutside() const {
  return hasOutside_;
}

unsigned int Face::numVertices() const {
  return numVertices_;
}

std::list<Vertex *>::iterator Face::outside() const {
  return outside_;
}

void Face::setOutside(std::list<Vertex *>::iterator outside) {
  hasOutside_ = true;
  outside_ = outside;
}

std::unique_ptr<Face> Face::createTriangle(Vertex *v0, Vertex *v1, Vertex *v2, double minArea) {
  std::unique_ptr<Face> face(new Face);

  std::shared_ptr<HalfEdge> e0 = std::make_shared<HalfEdge>(v0, face.get());
  std::shared_ptr<HalfEdge> e1 = std::make_shared<HalfEdge>(v1, face.get());
  std::shared_ptr<HalfEdge> e2 = std::make_shared<HalfEdge>(v2, face.get());

  face->setEdge(e0);
  e0->setNext(e1);
  e1->setNext(e2);
  e2->setNext(e0);

  face->computeNormalAndCentroid(minArea);

  return face;
}

void Face::computeNormalAndCentroid() {
  computeNormal();
  computeCentroid();
  planeOffset_ = normal_ * centroid_;

  checkVertexCount();
}

void Face::computeNormalAndCentroid(double minArea) {
  computeNormal(minArea);
  computeCentroid();
  planeOffset_ = normal_ * centroid_;

  checkVertexCount();
}

Face *Face::connectHalfEdges(std::shared_ptr<HalfEdge> prevEdge, std::shared_ptr<HalfEdge> nextEdge) {
  Face *discardedFace = nullptr;

  assert(!(nextEdge->opposite().expired()));
  std::shared_ptr<HalfEdge> oldOppositeEdge = nextEdge->opposite().lock();
  Face *oppositeFace = oldOppositeEdge->face();

  assert(!(prevEdge->opposite().expired()));
  if (prevEdge->opposite().lock()->face() == oppositeFace) {
    // Redundant edge
    std::shared_ptr<HalfEdge> newOppositeEdge;

    if (prevEdge == edge_) {
      edge_ = nextEdge;
    }

    if (oppositeFace->numVertices() == 3) {
      assert(!(oldOppositeEdge->prev().expired()));
      assert(!(oldOppositeEdge->prev().lock()->opposite().expired()));

      // Remove face
      newOppositeEdge = oldOppositeEdge->prev().lock()->opposite().lock();
      oppositeFace->flag = Flag::DELETED;
      discardedFace = oppositeFace;
    } else {
      newOppositeEdge = oldOppositeEdge->next();
      assert(!(newOppositeEdge->prev().expired()));
      if (oppositeFace->edge() == newOppositeEdge->prev().lock()) {
        oppositeFace->setEdge(newOppositeEdge);
      }
      assert(!(newOppositeEdge->prev().lock()->prev().expired()));
      newOppositeEdge->prev().lock()->prev().lock()->setNext(newOppositeEdge);
    }

    assert(!(prevEdge->prev().expired()));
    prevEdge->prev().lock()->setNext(nextEdge);
    nextEdge->setOpposite(newOppositeEdge);
    oppositeFace->computeNormalAndCentroid();
  } else {
    prevEdge->setNext(nextEdge);
  }

  return discardedFace;
}