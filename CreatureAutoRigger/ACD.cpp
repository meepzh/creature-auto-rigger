#include "ACD.h"

#include <algorithm>
#include <iterator>
#include <maya/MPxCommand.h>
#include "MathUtils.h"
#include <queue>
#include "Utils.h"

typedef std::pair<const Vertex *, double> queuePair;

struct queuePairComparator {
  bool operator()(const queuePair &a, const queuePair &b) {
    return a.second > b.second;
  }
};

ACD::ACD(MItMeshVertex &vertexIt, double concavityTolerance, double douglasPeuckerThreshold, MStatus *status)
    : quickHull_(vertexIt, -1, &returnStatus_),
      averageConcavity_(0), concavityTolerance_(concavityTolerance),
      clusteringThreshold_(concavityTolerance / 2.0), douglasPeuckerThreshold_(douglasPeuckerThreshold),
      maxConcavity_(0) {
  constructor(vertexIt);
  if (status) *status = returnStatus_;
}

ACD::ACD(MItMeshVertex &vertexIt, double concavityTolerance, double clusteringThreshold, double douglasPeuckerThreshold, MStatus *status)
     : quickHull_(vertexIt, -1, &returnStatus_),
       averageConcavity_(0), concavityTolerance_(concavityTolerance),
       clusteringThreshold_(clusteringThreshold), douglasPeuckerThreshold_(douglasPeuckerThreshold),
       maxConcavity_(0) {
  constructor(vertexIt);
  if (status) *status = returnStatus_;
}

void ACD::constructor(MItMeshVertex &vertexIt) {
  if (MZH::hasError(returnStatus_, "Error running QuickHull")) return;

  if (concavityTolerance_ < 0) {
    concavityTolerance_ = 0;
    MPxCommand::displayWarning("Concavity tolerance set below 0. Returned to 0.");
  }
  if (douglasPeuckerThreshold_ < 0) {
    douglasPeuckerThreshold_ = 0;
    MPxCommand::displayWarning("Douglas Peucker threshold set below 0. Returned to 0.");
  }
  if (douglasPeuckerThreshold_ > concavityTolerance_) {
    douglasPeuckerThreshold_ = concavityTolerance_;
    MPxCommand::displayWarning("Douglas Peucker threshold set above concavity tolerance. Clamped.");
  }

  MPxCommand::displayInfo("ACD arguments - concavity: " + MZH::toS(concavityTolerance_) + ", douglasPeucker: " + MZH::toS(douglasPeuckerThreshold_));

  vertices_ = quickHull_.vertices();

  vertices_->shrink_to_fit();
  quickHull_.faces()->shrink_to_fit();

  getHullVertices();
  getNeighbors(vertexIt);
  projectHullEdges();
  if (MZH::hasError(returnStatus_, "Error projecting hull edges")) return;
  matchPointsToBridge();
  if (MZH::hasError(returnStatus_, "Error matching points to a bridge")) return;
  calculateConcavities();
  if (MZH::hasError(returnStatus_, "Error calculating concavities")) return;
  findKnots();
}

double ACD::averageConcavity() {
  return averageConcavity_;
}

std::vector<double> &ACD::concavities() {
  return concavities_;
}

std::vector<Vertex *> &ACD::hullVertices() {
  return hullVertices_;
}

std::unordered_set<Vertex *> &ACD::knots() {
  return knots_;
}

double ACD::maxConcavity() {
  return maxConcavity_;
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
      bridges.insert(facePtr);

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
        //double newDistance = distance + (source->point() - neighbor->point()).length();
        double newDistance = distance + (closest->point() - neighbor->point()).length();
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

      if (pairBridges.empty()) {
        MPxCommand::displayError("No shared faces between target " + MZH::toS(target->index())
          + " and source " + MZH::toS(source->index()));
        returnStatus_ = MS::kFailure;
        return;
      }

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

    int index = curVertex.index();

    std::vector<bool> vertexSeen;
    vertexSeen.resize(vertices_->size(), false);

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
        MPxCommand::displayError("No common bridge faces found for vertex " + MZH::toS(curVertex.index()));
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
        } else {
          bridges.insert(vertexBridgeListIt->second.begin(), vertexBridgeListIt->second.end());
          initedBridges = true;
        }
        continue;
      }
      
      visited.push_back(vertex);

      // Add neighbors
      std::vector<Vertex *> &neighbors = neighbors_[vertex->index()];
      for (Vertex *neighbor : neighbors) {
        if (!vertexSeen[neighbor->index()]) queue.push(neighbor);
      }
    } //end-while !queue.empty()

    if (bridges.empty()) {
      MPxCommand::displayError("No bridge faces for vertex " + MZH::toS(curVertex.index()));
      returnStatus_ = MS::kFailure;
      return;
    }

    // Set vertex bridge faces
    for (Vertex *vertex : visited) {
      vertexBridgeList_[vertex] = bridges;
    }
  } //end-foreach vertices
}

void ACD::calculateConcavities() {
  concavities_.resize(vertices_->size(), 0);
  vertexBridges_.resize(vertices_->size(), nullptr);
  averageConcavity_ = 0;

  for (Vertex &vertex : *vertices_) {
    auto vertexBridgeListIt = vertexBridgeList_.find(&vertex);

    if (vertexBridgeListIt == vertexBridgeList_.end()) {
      MPxCommand::displayError("No bridge face list created for vertex " + MZH::toS(vertex.index()));
      returnStatus_ = MS::kFailure;
      return;
    }

    // Find smallest concavity
    double minConcavity = std::numeric_limits<double>::infinity();
    Face *bridge = nullptr;
    for (Face *bridgeCandidate : vertexBridgeListIt->second) {
      const double concavity = -1 * bridgeCandidate->pointPlaneDistance(vertex.point());
      if (concavity >= minConcavity) break;
      minConcavity = concavity;
      bridge = bridgeCandidate;
    }

    concavities_[vertex.index()] = minConcavity;
    vertexBridges_[vertex.index()] = bridge;

    if (minConcavity > maxConcavity_) maxConcavity_ = minConcavity;
    averageConcavity_ += minConcavity;
  } //end-foreach vertices

  averageConcavity_ /= vertices_->size();
}

void ACD::findKnots() {
  std::unordered_map<Vertex *, std::unordered_set<Vertex *>> visitedPairs;

  for (auto it1 = projectedEdges_.begin(); it1 != projectedEdges_.end(); ++it1) {
    std::unordered_map<Vertex *, std::shared_ptr<std::vector<Vertex *>>> &edgeMap = it1->second;

    for (auto it2 = it1->second.begin(); it2 != it1->second.end(); ++it2) {
      // Check if visited
      auto visitedIt = visitedPairs.find(it1->first);
      if (visitedIt != visitedPairs.end() && visitedIt->second.find(it2->first) != visitedIt->second.end()) continue;
      
      // Add pair as visited
      visitedPairs[it1->first].insert(it2->first);
      visitedPairs[it2->first].insert(it1->first);

      // Convert points
      std::shared_ptr<std::vector<Vertex *>> &path = it2->second;
      std::vector<DPVertex> vertices;

      double length = 0;
      for (size_t i = 0; i < path->size() - 1; ++i) {
        vertices.emplace_back(MPoint(length, concavities_[(*path)[i]->index()]), (*path)[i]);
        length += ((*path)[i + 1]->point() - (*path)[i]->point()).length();
      }

      Vertex *start = path->front();
      Vertex *end = path->back();

      // Add knots
      std::vector<DPVertex> knots = douglasPeucker(vertices);
      for (DPVertex &vertex : knots) {
        if (vertex.point.y <= douglasPeuckerThreshold_) continue;
        knots_.insert(vertex.vertex);
      }
    } //end-foreach target path
  } //end-foreach projected edge
}

std::vector<DPVertex> ACD::douglasPeucker(std::vector<DPVertex> &vertices) {
  if (vertices.size() < 3) return vertices;
    
  std::vector<DPVertex> results;
  double maxDistanceSquared = 0.0;
  unsigned int maxIndex = 0;

  DPVertex &start = vertices.front();
  DPVertex &end = vertices.back();

  for (unsigned int i = 1; i < vertices.size() - 1; ++i) {
    // TODO: Should we only be using concavity (as we are using point-line distance, not point-segment distance)
    const double distanceSquared = MZH::pointLineDistanceSquared(vertices[i].point, start.point, end.point);
    if (distanceSquared <= maxDistanceSquared) continue;
    maxIndex = i;
    maxDistanceSquared = distanceSquared;
  }

  if (maxDistanceSquared > douglasPeuckerThreshold_ * douglasPeuckerThreshold_) {
    std::vector<DPVertex> results1 = douglasPeucker(std::vector<DPVertex>(vertices.begin(), vertices.begin() + maxIndex + 1));
    std::vector<DPVertex> results2 = douglasPeucker(std::vector<DPVertex>(vertices.begin() + maxIndex, vertices.end()));
    results = results1;
    results.insert(results.end(), results2.begin(), results2.end() - 1);
  } else {
    results.push_back(start);
    results.push_back(end);
  }

  return results;
}

