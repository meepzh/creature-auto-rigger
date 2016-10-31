#include "Face.h"
#include <cassert>

Face::Face()
  : area_(0), edge_(nullptr), hasOutside_(false),
    numVertices_(0), planeOffset_(0), flag(Flag::VISIBLE) {
}

Face::~Face() {
  if (edge_) {
    // Prevent circular reference
    edge_->prev()->clearNext();
  }
}

void Face::checkConsistency() {
  HalfEdge *edge = edge_.get();
  int numVertices = 0;

  assert(numVertices_ >= 3);

  do {
    HalfEdge *oppositeEdge = edge->opposite();
    assert(oppositeEdge != nullptr);
    assert(oppositeEdge->opposite() == edge);
    assert(oppositeEdge->vertex() != edge->prevVertex());
    assert(edge->vertex() != oppositeEdge->prevVertex());

    Face *oppositeFace = oppositeEdge->face();
    assert(oppositeFace != nullptr);
    assert(oppositeFace->flag != Flag::DELETED);

    ++numVertices;
  } while (edge != edge_.get());

  assert(numVertices == numVertices_);
}

void Face::computeCentroid() {
  centroid_ = MPoint::origin;
  HalfEdge *faceEdge = edge_.get();
  HalfEdge *curEdge = faceEdge;
  do {
    centroid_ += curEdge->vertex()->point();
    curEdge = curEdge->next();
  } while (curEdge != faceEdge);
  centroid_ = centroid_ / (double) numVertices_;
}

void Face::computeNormal() {
  HalfEdge *faceEdge = edge_.get();
  HalfEdge *curEdge = faceEdge->next()->next();

  MVector v1;
  MVector v2 = faceEdge->next()->vertex()->point() - faceEdge->vertex()->point();

  normal_ = MVector::zero;
  numVertices_ = 2;

  while (curEdge != faceEdge) {
    v1 = v2;
    v2 = curEdge->vertex()->point() - faceEdge->vertex()->point();
    normal_ += v1 ^ v2;

    curEdge = curEdge->next();
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
      curEdge = curEdge->next();
    } while (curEdge != faceEdge);

    // Recompute normal
    MVector maxVector = curEdge->vertex()->point() - curEdge->prev()->vertex()->point();
    maxVector.normalize();
    double maxProjection = normal_ * maxVector;
    normal_ += maxVector * (-maxProjection);
    normal_.normalize();
  }
}

void Face::mergeAdjacentFaces(HalfEdge *adjacentEdge, std::vector<Face *> &discardedFaces) {
  HalfEdge *oppositeEdge = adjacentEdge->opposite();
  Face *oppositeFace = oppositeEdge->face();

  discardedFaces.push_back(oppositeFace);
  oppositeFace->flag = Flag::DELETED;

  HalfEdge *adjacentEdgePrev = adjacentEdge->prev();
  HalfEdge *adjacentEdgeNext = adjacentEdge->next();
  HalfEdge *oppositeEdgePrev = oppositeEdge->prev();
  HalfEdge *oppositeEdgeNext = oppositeEdge->next();

  // Find opposite edge chain
  // Left edge
  while (adjacentEdgePrev->opposite()->face() == oppositeFace) {
    adjacentEdgePrev = adjacentEdgePrev->prev();
    oppositeEdgeNext = oppositeEdgeNext->next();
  }

  // Right edge
  while (adjacentEdgeNext->opposite()->face() == oppositeFace) {
    adjacentEdgeNext = adjacentEdgeNext->next();
    oppositeEdgePrev = oppositeEdgePrev->prev();
  }

  // Fix face references
  HalfEdge *edge = nullptr;
  for (edge = oppositeEdgeNext; edge != oppositeEdgePrev->next(); edge = edge->next()) {
    edge->setFace(this);
  }

  // Preserve with edge that won't be destroyed
  edge_ = std::shared_ptr<HalfEdge>(adjacentEdgeNext);

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

HalfEdge *Face::edge() {
  return edge_.get();
}

HalfEdge *Face::edge(int i) {
  HalfEdge *curEdge = edge_.get();

  while (i > 0) {
    curEdge = curEdge->next();
    --i;
  }
  while (i < 0) {
    curEdge = curEdge->prev();
    ++i;
  }

  return curEdge;
}

void Face::setEdge(std::shared_ptr<HalfEdge> &edge) {
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

}

void Face::computeNormalAndCentroid(double minArea) {
  computeNormal(minArea);
  computeCentroid();
  planeOffset_ = normal_ * centroid_;
}

Face *Face::connectHalfEdges(HalfEdge *prevEdge, HalfEdge *edge) {
  Face *discardedFace = nullptr;

  if (prevEdge->opposite()->face() == edge->opposite()->face()) {
    // Redundant edge
    Face *oppositeFace = edge->opposite()->face();
    HalfEdge *oppositeEdge;

    if (prevEdge == edge_.get()) {
      edge_ = std::shared_ptr<HalfEdge>(edge);
    }

    if (oppositeFace->numVertices() == 3) {
      // Remove face
      oppositeEdge = edge->opposite()->prev()->opposite();
      oppositeFace->flag = Flag::DELETED;
      discardedFace = oppositeFace;
    } else {
      oppositeEdge = edge->opposite()->next();
      if (oppositeFace->edge() == oppositeEdge->prev()) {
        oppositeFace->setEdge(std::shared_ptr<HalfEdge>(oppositeEdge));
      }
      oppositeEdge->prev()->prev()->setNext(std::shared_ptr<HalfEdge>(oppositeEdge));
    }

    prevEdge->prev()->setNext(std::shared_ptr<HalfEdge>(edge));
    edge->setOpposite(oppositeEdge);
    oppositeEdge->setOpposite(edge);
    oppositeFace->computeNormalAndCentroid();
  } else {
    prevEdge->setNext(std::shared_ptr<HalfEdge>(edge));
  }

  return discardedFace;
}