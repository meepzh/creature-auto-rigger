#include "ACD.h"

#include <algorithm>
#include <iterator>
#include <maya/MPxCommand.h>
#include <queue>
#include "Utils.h"

typedef std::pair<const Vertex *, double> queuePair;

struct queuePairComparator {
  bool operator()(const queuePair &a, const queuePair &b) {
    return a.second > b.second;
  }
};

ACD::ACD(MItMeshVertex &vertexIt, MStatus *status)
    : quickHull_(vertexIt, -1, &returnStatus_), maxConvexity_(0) {
  if (MZH::hasError(returnStatus_, "Error running QuickHull")) return;

  vertices_ = quickHull_.vertices();

  vertices_->shrink_to_fit();
  quickHull_.faces()->shrink_to_fit();

  getHullVertices();
  getNeighbors(vertexIt);
  projectHullEdges();
  if (MZH::hasError(returnStatus_, "Error projecting hull edges")) return;
  matchPointsToBridge();
  if (MZH::hasError(returnStatus_, "Error matching points to a bridge")) return;
  calculateConvexities();
  if (MZH::hasError(returnStatus_, "Error calculating convexities")) return;

  if (status) *status = returnStatus_;
}

std::vector<double> &ACD::convexities() {
  return convexities_;
}

std::vector<Vertex *> &ACD::hullVertices() {
  return hullVertices_;
}

double ACD::maxConvexity() {
  return maxConvexity_;
}

pEdgeMap &ACD::projectedEdges() {
  return projectedEdges_;
}

QuickHull &ACD::quickHull() {
  return quickHull_;
}

std::vector<Face *> &ACD::vertexBridges() {
  return vertexBridges_;
}

void ACD::getHullVertices() {
  std::shared_ptr<std::vector<std::unique_ptr<Face>>> faces = quickHull_.faces();
  std::vector<bool> vertexSeen;
  vertexSeen.resize(vertices_->size(), false);

  for (std::unique_ptr<Face> &face : *faces) {
    std::shared_ptr<HalfEdge> faceEdge = face->edge();
    std::shared_ptr<HalfEdge> curEdge = faceEdge;
    Face *facePtr = face.get();

    do {
      Vertex *vertex = curEdge->vertex();
      std::set<Face *> &bridges = vertexBridgeList_[vertex];
      if (bridges.find(facePtr) == bridges.end()) bridges.insert(facePtr);

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
    std::set<Face *> &sourceBridges = vertexBridgeList_[source];

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

      // Get potential bridge faces
      std::set<Face *> &targetBridges = vertexBridgeList_[target];
      std::set<Face *> pairBridges;
      std::set_intersection(sourceBridges.begin(), sourceBridges.end(),
        targetBridges.begin(), targetBridges.end(),
        std::inserter(pairBridges, pairBridges.end())
      );

      const Vertex *curVertex = previouses[target->index()];
      while (curVertex != source) {
        projectedEdge->push_back(const_cast<Vertex *>(curVertex));
        std::set<Face *> &curBridges = vertexBridgeList_[const_cast<Vertex *>(curVertex)];
        curBridges.insert(pairBridges.begin(), pairBridges.end());
        
        if (previouses[curVertex->index()] == nullptr) {
          // Couldn't find path...
          MPxCommand::displayError("Could not find path for vertex " + MZH::toS(curVertex->index())
            + " to target " + MZH::toS(target->index())
            + " from source " + MZH::toS(source->index()));
          returnStatus_ = MS::kFailure;
          return;
        }
        curVertex = previouses[curVertex->index()];
      }
      projectedEdge->push_back(source);

      // Add to projectedEdges_
      edgeMap.insert(std::make_pair(target, projectedEdge));
    } //end-foreach target
  } //end-foreach hullVertex
}

void ACD::matchPointsToBridge() {
  std::queue<Vertex *> queue;
  std::vector<Vertex *> visited;

  for (Vertex &curVertex : *vertices_) {
    if (vertexBridgeList_.find(&curVertex) != vertexBridgeList_.end()) continue;

    std::vector<bool> vertexSeen;
    vertexSeen.resize(vertices_->size(), false);
    vertexSeen[curVertex.index()] = true;

    std::set<Face *> bridges;
    queue.push(&curVertex);
    visited.clear();
    bool initedBridges = false;

    while (!queue.empty()) {
      Vertex *vertex = queue.front();
      queue.pop();
      
      // Skip visited
      if (vertexSeen[vertex->index()]) continue;
      vertexSeen[vertex->index()] = true;

      if (initedBridges && bridges.empty()) {
        // Should not be empty
        MPxCommand::displayError("Bridges emptied for vertex " + MZH::toS(curVertex.index()));
        returnStatus_ = MS::kFailure;
        return;
      }

      // Add known connected bridges
      auto vertexBridgeListIt = vertexBridgeList_.find(vertex);
      if (vertexBridgeListIt != vertexBridgeList_.end()) {
        if (initedBridges) {
          for (auto it = bridges.begin(); it != bridges.end();) {
            // Erase faces that aren't shared
            if (vertexBridgeListIt->second.find(*it) == vertexBridgeListIt->second.end()) {
              it = bridges.erase(it);
            } else {
              ++it;
            }
          }
          continue;
        } else {
          bridges.insert(vertexBridgeListIt->second.begin(), vertexBridgeListIt->second.end());
          initedBridges = true;
        }
      }
      
      visited.push_back(vertex);

      // Add neighbors
      std::vector<Vertex *> &neighbors = neighbors_[vertex->index()];
      for (Vertex *neighbor : neighbors) {
        if (!vertexSeen[vertex->index()]) queue.push(neighbor);
      }
    } //end-while !queue.empty()

    // Set vertex bridge faces
    for (Vertex *vertex : visited) {
      vertexBridgeList_[vertex] = bridges;
    }
  }
}

void ACD::calculateConvexities() {
  convexities_.resize(vertices_->size(), 0);
  vertexBridges_.resize(vertices_->size(), nullptr);

  for (Vertex &vertex : *vertices_) {
    auto vertexBridgeListIt = vertexBridgeList_.find(&vertex);
    if (vertexBridgeListIt == vertexBridgeList_.end()) {
      MPxCommand::displayError("No bridge faces found for " + MZH::toS(vertex.index()));
      returnStatus_ = MS::kFailure;
      return;
    }

    // Find smallest convexity
    double minConvexity = std::numeric_limits<double>::infinity();
    Face *bridge = nullptr;
    for (Face *bridgeCandidate : vertexBridgeListIt->second) {
      const double convexity = bridgeCandidate->pointPlaneDistance(vertex.point());
      if (convexity >= minConvexity) break;
      minConvexity = convexity;
      bridge = bridgeCandidate;
    }

    convexities_[vertex.index()] = minConvexity;
    vertexBridges_[vertex.index()] = bridge;

    if (minConvexity > maxConvexity_) maxConvexity_ = minConvexity;
  } //end-foreach vertices
}