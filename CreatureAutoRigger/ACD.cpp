#include "ACD.h"

#include <algorithm>
#include <queue>
#include "Utils.h"

typedef std::pair<const Vertex *, double> queuePair;

struct queuePairComparator {
  bool operator()(const queuePair &a, const queuePair &b) {
    return a > b;
  }
};

ACD::ACD(MItMeshVertex &vertexIt, MStatus *status)
    : quickHull_(vertexIt, -1, status) {
  if (MZH::hasError(*status, "Error running QuickHull")) return;

  vertices_ = quickHull_.vertices();

  vertices_->shrink_to_fit();
  quickHull_.faces()->shrink_to_fit();

  getHullVertices();
  getNeighbors(vertexIt);
  projectHullEdges();
}

std::vector<Vertex *> &ACD::hullVertices() {
  return hullVertices_;
}

pEdgeMap &ACD::projectedEdges() {
  return projectedEdges_;
}

QuickHull &ACD::quickHull() {
  return quickHull_;
}

void ACD::getHullVertices() {
  std::shared_ptr<std::vector<std::unique_ptr<Face>>> faces = quickHull_.faces();
  std::vector<bool> vertexSeen;
  vertexSeen.resize(vertices_->size(), false);

  for (std::unique_ptr<Face> &face : *faces) {
    std::shared_ptr<HalfEdge> faceEdge = face->edge();
    std::shared_ptr<HalfEdge> curEdge = faceEdge;

    do {
      Vertex *vertex = curEdge->vertex();

      if (!vertexSeen[vertex->index()]) {
        vertexSeen[vertex->index()] = true;
        hullVertices_.push_back(vertex);
        hullNeighbors_.insert(std::make_pair(vertex, curEdge->getNeighbors()));
      }

      curEdge = curEdge->next();
    } while (curEdge != faceEdge);
  }

  hullVertices_.shrink_to_fit();
}

void ACD::getNeighbors(MItMeshVertex &vertexIt) {
  neighbors_.resize(vertexIt.count());
  vertexIt.reset();

  MIntArray neighborIndices;
  for (; !vertexIt.isDone(); vertexIt.next()) {
    std::vector<Vertex *> neighbors;

    vertexIt.getConnectedVertices(neighborIndices);
    for (unsigned int i = 0; i < neighborIndices.length(); ++i) {
      neighbors.push_back(&(*vertices_)[neighborIndices[i]]);
    }

    neighbors.shrink_to_fit();
    neighbors_[vertexIt.index()] = neighbors;
  }
}

void ACD::projectHullEdges() {
  for (Vertex *source : hullVertices_) {
    std::vector<Vertex *> targets = hullNeighbors_.at(source); // Copy, will modify

    // Create projectedEdges_
    projectedEdges_.insert(std::make_pair(source, std::unordered_map<Vertex *, std::shared_ptr<std::vector<Vertex *>>>()));
    auto &edgeMap = projectedEdges_.at(source);

    // Remove targets that have already been resolved
    for (auto targetIt = targets.begin(); targetIt != targets.end();) {
      auto edgeMapIt = projectedEdges_.find(*targetIt);
      if (edgeMapIt != projectedEdges_.end()) {
        auto pathIt = edgeMapIt->second.find(source);
        if (pathIt == edgeMapIt->second.end()) {
          // Copy
          edgeMap.insert(std::make_pair(*targetIt, pathIt->second));
          // Erase
          targetIt = targets.erase(targetIt);
          continue; // Skip increment
        }
      }
      ++targetIt;
    }
    std::vector<Vertex *> missingTargets = targets;

    if (missingTargets.empty()) return;

    std::vector<double> distances;
    std::vector<const Vertex *> previouses;
    distances.resize(vertices_->size(), std::numeric_limits<double>::infinity());
    previouses.resize(vertices_->size(), nullptr);

    std::priority_queue<queuePair, std::vector<queuePair>, queuePairComparator> queue;

    queue.push(std::make_pair(source, 0));
    distances[source->index()] = 0;

    while (!queue.empty()) {
      const Vertex *closest = queue.top().first;
      queue.pop();
      
      // Check with targets
      auto it = std::find(missingTargets.begin(), missingTargets.end(), closest);
      if (it != missingTargets.end()) {
        missingTargets.erase(it);
        if (missingTargets.empty()) break;
      }

      double distance = distances[closest->index()];

      const std::vector<Vertex *> &neighbors = neighbors_[closest->index()];
      for (const Vertex *neighbor : neighbors) {
        double newDistance = distance + (source->point() - neighbor->point()).length();
        if (newDistance < distances[neighbor->index()]) {
          distances[neighbor->index()] = newDistance;
          previouses[neighbor->index()] = closest;
          queue.push(std::make_pair(neighbor, newDistance));
        }
      } //end-for
    } //end-while

    // Extract paths to targets
    missingTargets = targets;
    for (Vertex *target : missingTargets) {
      std::shared_ptr<std::vector<Vertex *>> projectedEdge = std::make_shared<std::vector<Vertex *>>();
      projectedEdge->push_back(target);

      const Vertex *curVertex = previouses[target->index()];
      while (curVertex != source) {
        projectedEdge->push_back(const_cast<Vertex *>(curVertex));
        curVertex = previouses[curVertex->index()];
      }
      projectedEdge->push_back(source);

      // Add to projectedEdges_
      edgeMap.insert(std::make_pair(target, projectedEdge));
    } //end-foreach target
  } //end-foreach hullVertex
}
