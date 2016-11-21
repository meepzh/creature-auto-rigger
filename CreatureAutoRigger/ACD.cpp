#include "ACD.h"

#include <functional>
#include <queue>
#include "Utils.h"
#include "VertexPair.h"

ACD::ACD(MItMeshVertex &vertexIt, MStatus *status)
    : quickHull_(vertexIt, -1, status) {
  if (MZH::hasError(*status, "Error running QuickHull")) return;

  vertices_ = quickHull_.vertices();

  getHullVertices();
  getNeighbors(vertexIt);
  projectHullEdges();
}

std::vector<Vertex *> &ACD::hullVertices() {
  return hullVertices_;
}

QuickHull &ACD::quickHull() {
  return quickHull_;
}

void ACD::getHullVertices() {
  std::shared_ptr<std::vector<std::unique_ptr<Face>>> faces = quickHull_.faces();
  std::vector<bool> vertexSeen;
  vertexSeen.resize(vertices_->size(), false);

  for (std::unique_ptr<Face> &face : *faces) {
    std::vector<Vertex *> faceVertices = face->vertices();
    for (Vertex *vertex : faceVertices) {
      if (vertexSeen[vertex->index()]) continue;
      vertexSeen[vertex->index()] = true;
      hullVertices_.push_back(vertex);
    }
  }
}

void ACD::getNeighbors(MItMeshVertex &vertexIt) {
  neighbors_.reserve(vertexIt.count());
  vertexIt.reset();

  MIntArray neighborIndices;
  for (; !vertexIt.isDone(); vertexIt.next()) {
    std::vector<Vertex *> neighbors;
    vertexIt.getConnectedVertices(neighborIndices);
    for (unsigned int i = 0; i < neighborIndices.length(); ++i) {
      neighbors.push_back(&(*vertices_)[neighborIndices[i]]);
    }
    neighbors_.insert(std::make_pair(&(*vertices_)[vertexIt.index()], neighbors));
  }
}

void ACD::projectHullEdges() {
  for (Vertex &source : *vertices_) {
    std::priority_queue<VertexPair, std::vector<VertexPair>, std::greater<VertexPair>> queue;

  }
}
