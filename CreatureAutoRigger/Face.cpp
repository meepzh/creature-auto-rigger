#include "Face.h"

Face::Face()
    : area_(0), edge_(nullptr), hasOutside_(false), numVertices_(0), planeOffset_(0) {
}

Face::~Face() {
  if (edge_) {
    // Prevent circular reference
    edge_->prev()->clearNext();
  }
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

double Face::pointPlaneDistance(const MPoint &pt) {
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

bool Face::hasOutside() const {
  return hasOutside;
}

std::list<Vertex>::iterator Face::outside() const {
  return outside_;
}

void Face::setOutside(std::list<Vertex>::iterator outside) {
  hasOutside_ = true;
  outside_ = outside;
}

std::unique_ptr<Face> Face::createTriangle(const Vertex &v0, const Vertex &v1, const Vertex &v2, double minArea = 0) {
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
