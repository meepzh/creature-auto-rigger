#include "ACD.h"

#include <algorithm>
#include <Eigen/Dense>
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

// Constructors

// ACD with automatically calculated clustering threshold
ACD::ACD(MItMeshVertex &vertexIt, double concavityTolerance, double douglasPeuckerThreshold, MStatus *status)
    : quickHull_(vertexIt, -1, &returnStatus_),
      averageConcavity_(0), concavityTolerance_(concavityTolerance),
      clusteringThreshold_(concavityTolerance / 2.0), douglasPeuckerThreshold_(douglasPeuckerThreshold),
      maxConcavity_(0) {
  constructor(vertexIt);
  if (status) *status = returnStatus_;
}

// ACD with manual clustering threshold
ACD::ACD(MItMeshVertex &vertexIt, double concavityTolerance, double clusteringThreshold, double douglasPeuckerThreshold, MStatus *status)
     : quickHull_(vertexIt, -1, &returnStatus_),
       averageConcavity_(0), concavityTolerance_(concavityTolerance),
       clusteringThreshold_(clusteringThreshold), douglasPeuckerThreshold_(douglasPeuckerThreshold),
       maxConcavity_(0) {
  constructor(vertexIt);
  if (status) *status = returnStatus_;
}

// Runs the ACD Algorithm.
// Should only be called by an ACD constructor
void ACD::constructor(MItMeshVertex &vertexIt) {
  if (MZH::hasError(returnStatus_, "Error running QuickHull")) return;

  // Parameter sanity check
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

  // Print args
  MPxCommand::displayInfo("ACD arguments - concavity: " + MZH::toS(concavityTolerance_) + ", douglasPeucker: " + MZH::toS(douglasPeuckerThreshold_));

  // Get vertex objects
  vertices_ = quickHull_.vertices();

  // Shrink fixed-length vectors
  vertices_->shrink_to_fit();
  quickHull_.faces()->shrink_to_fit();

  // Execute ACD Algorithm
  getHullVertices();
  getNeighbors(vertexIt);
  projectHullEdges();
  if (MZH::hasError(returnStatus_, "Error projecting hull edges")) return;
  matchPointsToBridge();
  if (MZH::hasError(returnStatus_, "Error matching points to a bridge")) return;
  calculateConcavities();
  if (MZH::hasError(returnStatus_, "Error calculating concavities")) return;
  findKnots();
  computeUsefulPocketCuts();
}

// Getters

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

// Algorithm Functions

// Extract convex hull vertices, vertex bridges (faces), and hull vertex neighbors
void ACD::getHullVertices() {
  std::shared_ptr<std::vector<std::unique_ptr<Face>>> faces = quickHull_.faces();
  std::vector<bool> vertexSeen;
  vertexSeen.resize(vertices_->size(), false);

  // For each bridge
  for (std::unique_ptr<Face> &face : *faces) {
    std::shared_ptr<HalfEdge> faceEdge = face->edge();
    std::shared_ptr<HalfEdge> curEdge = faceEdge;
    Face *facePtr = face.get();

    // Get vertex info
    do {
      Vertex *vertex = curEdge->vertex();
      std::set<Face *> &bridges = vertexBridgeList_[vertex];
      bridges.insert(facePtr);

      // Get hull vertex neighbors if haven't
      if (!vertexSeen[vertex->index()]) {
        vertexSeen[vertex->index()] = true;
        hullVertices_.push_back(vertex);
        hullVertexNeighbors_.insert(std::make_pair(vertex, curEdge->getNeighbors()));
      }

      // For each face vertex
      curEdge = curEdge->next();
    } while (curEdge != faceEdge);
  }
  
  hullVertices_.shrink_to_fit();
}

// Store vertex neighbors using the Maya API
void ACD::getNeighbors(MItMeshVertex &vertexIt) {
  neighbors_.resize(vertexIt.count());
  vertexIt.reset();

  // For each vertex
  MIntArray neighborIndices;
  for (; !vertexIt.isDone(); vertexIt.next()) {
    std::vector<Vertex *> neighbors;

    // Convert vertex ID to Vertex object and save neighbor relationship
    vertexIt.getConnectedVertices(neighborIndices);
    for (unsigned int i = 0; i < neighborIndices.length(); ++i) {
      neighbors.push_back(&(*vertices_)[neighborIndices[i]]);
    }

    // Save a compact version of the neighbors
    neighbors.shrink_to_fit();
    neighbors_[vertexIt.index()] = neighbors;
  }
}

// Project convex hull edges onto the target mesh using Dijkstra's
void ACD::projectHullEdges() {
  // For each hull vertex
  for (Vertex *source : hullVertices_) {
    std::vector<Vertex *> targets = hullVertexNeighbors_.at(source); // Copy, will modify
    std::set<Face *> &sourceBridges = vertexBridgeList_[source];

    // Create projectedEdges_ (source, target) -> (path from source to target)
    projectedEdges_.insert(std::make_pair(source, std::unordered_map<Vertex *, std::shared_ptr<std::vector<Vertex *>>>()));
    auto &edgeMap = projectedEdges_.at(source);

    // Remove targets (dest vertices) that have already been resolved
    for (auto targetIt = targets.begin(); targetIt != targets.end();) {
      auto edgeMapIt = projectedEdges_.find(*targetIt);
      if (edgeMapIt != projectedEdges_.end()) { // Found target
        auto pathIt = edgeMapIt->second.find(source);
        if (pathIt == edgeMapIt->second.end()) { // Path contains source
          // Copy the path
          edgeMap.insert(std::make_pair(*targetIt, pathIt->second));
          // Erase the target
          targetIt = targets.erase(targetIt);
          continue; // Skip increment
        }
      }
      ++targetIt;
    }
    if (targets.empty()) return;
    
    for (Vertex *target : targets) {
      // Init Dijkstra's algorithm structures
      std::vector<double> distances;
      std::vector<const Vertex *> previouses;
      distances.resize(vertices_->size(), std::numeric_limits<double>::infinity());
      previouses.resize(vertices_->size(), nullptr);

      std::priority_queue<queuePair, std::vector<queuePair>, queuePairComparator> queue;

      queue.push(std::make_pair(source, 0));
      distances[source->index()] = 0;

      // While there are vertices to process
      while (!queue.empty()) {
        const Vertex *closest = queue.top().first;
        queue.pop();
      
        // Check if target. Then success!
        if (closest == target) break;

        double distance = distances[closest->index()];

        // Add vertex neighbors that are closer
        const std::vector<Vertex *> &neighbors = neighbors_[closest->index()];
        for (const Vertex *neighbor : neighbors) {
          // Want to maintain hull edge integrity, not shortest length
          double newDistance = distance + MZH::pointLineDistance(neighbor->point(), source->point(), target->point());
          if (newDistance < distances[neighbor->index()]) {
            distances[neighbor->index()] = newDistance;
            previouses[neighbor->index()] = closest;
            queue.push(std::make_pair(neighbor, newDistance));
          }
        } //end-for
      } //end-while

      // Init vector path to target
      std::shared_ptr<std::vector<Vertex *>> projectedEdge = std::make_shared<std::vector<Vertex *>>();
      projectedEdge->push_back(target);

      // Find shared bridges between source and target
      std::set<Face *> &targetBridges = vertexBridgeList_[target];
      std::set<Face *> pairBridges;
      std::set_intersection(sourceBridges.begin(), sourceBridges.end(),
        targetBridges.begin(), targetBridges.end(),
        std::inserter(pairBridges, pairBridges.end())
      );

      // Fail if a common bridge can't be found. Impossible!
      if (pairBridges.empty()) {
        MPxCommand::displayError("No shared faces between target " + MZH::toS(target->index())
          + " and source " + MZH::toS(source->index()));
        returnStatus_ = MS::kFailure;
        return;
      }

      // Add shared bridges to projected vertex's possibly associated bridges
      const Vertex *curVertex = previouses[target->index()];
      while (curVertex != source) {
        projectedEdge->push_back(const_cast<Vertex *>(curVertex));
        std::set<Face *> &curBridges = vertexBridgeList_[const_cast<Vertex *>(curVertex)];
        curBridges.insert(pairBridges.begin(), pairBridges.end());
        
        // Fail if can't find a path from target to source
        if (previouses[curVertex->index()] == nullptr) {
          MPxCommand::displayError("Could not find path for vertex " + MZH::toS(curVertex->index())
            + " to target " + MZH::toS(target->index())
            + " from source " + MZH::toS(source->index()));
          returnStatus_ = MS::kFailure;
          return;
        }

        // Next vertex in path
        curVertex = previouses[curVertex->index()];
      }
      projectedEdge->push_back(source);

      // Add to projectedEdges_
      edgeMap.insert(std::make_pair(target, projectedEdge));
    } //end-foreach target
  } //end-foreach hullVertex
}

// Match every mesh vertex to the nearest bridge
void ACD::matchPointsToBridge() {
  std::queue<Vertex *> queue;
  std::vector<Vertex *> visited;

  // For each vertex with undetermined bridges
  for (Vertex &curVertex : *vertices_) {
    if (vertexBridgeList_.find(&curVertex) != vertexBridgeList_.end()) continue;

    int index = curVertex.index();

    std::vector<bool> vertexSeen;
    vertexSeen.resize(vertices_->size(), false);

    std::set<Face *> bridges; // Set of bridges shared by surrounding projected edge vertices
    std::set<Face *> seenBridges; // Set of all bridges encountered
    queue.push(&curVertex);
    visited.clear();
    bool initedBridges = false; // True if bridge sets have been inserted into

    // Flood fill vertices to projected hull edge boundaries
    while (!queue.empty()) {
      Vertex *vertex = queue.front();
      queue.pop();
      
      // Skip visited vertices
      if (vertexSeen[vertex->index()]) continue;
      vertexSeen[vertex->index()] = true;

      // Found vertex with known bridges
      auto vertexBridgeListIt = vertexBridgeList_.find(vertex);
      if (vertexBridgeListIt != vertexBridgeList_.end()) {
        if (initedBridges) { // Only keep shared bridges in the bridges set
          for (auto it = bridges.begin(); it != bridges.end();) {
            // Erase if not shared
            if (vertexBridgeListIt->second.find(*it) == vertexBridgeListIt->second.end()) {
              it = bridges.erase(it);
            } else {
              ++it;
            }
          }
        } else {
          // First time encountering known bridges, add them all
          bridges.insert(vertexBridgeListIt->second.begin(), vertexBridgeListIt->second.end());
          seenBridges.insert(vertexBridgeListIt->second.begin(), vertexBridgeListIt->second.end());
          initedBridges = true;
        }
        continue;
      }
      
      visited.push_back(vertex);

      // Add neighbors to process
      std::vector<Vertex *> &neighbors = neighbors_[vertex->index()];
      for (Vertex *neighbor : neighbors) {
        if (!vertexSeen[neighbor->index()]) queue.push(neighbor);
      }
    } //end-while !queue.empty()

    if (bridges.empty()) {
      // Fail if bridges have never been seen
      if (seenBridges.empty()) {
        MPxCommand::displayError("No bridge faces for vertex " + MZH::toS(curVertex.index()));
        returnStatus_ = MS::kFailure;
        return;
      }

      // Simply use all bridges seen
      // TODO: Why does this happen?
      MPxCommand::displayWarning("No common bridge face found for vertex " + MZH::toS(curVertex.index()));
      bridges = seenBridges;
    }

    // Set vertex bridge faces
    for (Vertex *vertex : visited) {
      vertexBridgeList_[vertex] = bridges;
    }
  } //end-foreach vertices
}

// Calculate concavity for each vertex and save the associated bridge
void ACD::calculateConcavities() {
  concavities_.resize(vertices_->size(), 0);
  vertexBridges_.resize(vertices_->size(), nullptr);
  averageConcavity_ = 0;

  for (Vertex &vertex : *vertices_) {
    // Get potential bridges
    auto vertexBridgeListIt = vertexBridgeList_.find(&vertex);

    // Fail if a vertex has no bridges, and therefore cannot calculate concavity
    if (vertexBridgeListIt == vertexBridgeList_.end()) {
      MPxCommand::displayError("No bridge face list created for vertex " + MZH::toS(vertex.index()));
      returnStatus_ = MS::kFailure;
      return;
    }

    // Find smallest concavity of the possible bridges
    double minConcavity = std::numeric_limits<double>::infinity();
    Face *bridge = nullptr;
    for (Face *bridgeCandidate : vertexBridgeListIt->second) {
      const double concavity = -1 * bridgeCandidate->pointPlaneDistance(vertex.point());
      if (concavity >= minConcavity) break;
      minConcavity = concavity;
      bridge = bridgeCandidate;
    }

    // Save concavity and bridge
    concavities_[vertex.index()] = minConcavity;
    vertexBridges_[vertex.index()] = bridge;

    if (minConcavity > maxConcavity_) maxConcavity_ = minConcavity;
    averageConcavity_ += minConcavity;
  } //end-foreach vertices

  averageConcavity_ /= vertices_->size();
}

// Find knots on pocket boundaries (bridge projected paths) using the Douglas-Peucker criteria
void ACD::findKnots() {
  // For each source-target path (pocket boundary)
  for (auto it1 = projectedEdges_.begin(); it1 != projectedEdges_.end(); ++it1) {
    std::unordered_map<Vertex *, std::shared_ptr<std::vector<Vertex *>>> &edgeMap = it1->second;
    
    // For each vertex in the path
    for (auto it2 = it1->second.begin(); it2 != it1->second.end(); ++it2) {
      std::shared_ptr<std::vector<Vertex *>> &path = it2->second;
      std::vector<DPVertex> vertices;

      // Create points in concavity space
      double length = 0;
      for (size_t i = 0; i < path->size() - 1; ++i) {
        vertices.emplace_back(MPoint(length, concavities_[(*path)[i]->index()]), (*path)[i]);
        length += ((*path)[i + 1]->point() - (*path)[i]->point()).length();
      }

      Vertex *start = path->front();
      Vertex *end = path->back();

      // Add knots matching criteria
      std::vector<DPVertex> knots = douglasPeucker(vertices);
      for (DPVertex &vertex : knots) {
        if (vertex.point.y <= douglasPeuckerThreshold_) continue;
        knots_.insert(vertex.vertex);
      }
    } //end-foreach target path
  } //end-foreach projected edge
}

// Return Douglas-Peucker optimized set of vertices
std::vector<DPVertex> ACD::douglasPeucker(std::vector<DPVertex> &vertices) {
  if (vertices.size() < 3) return vertices;
    
  std::vector<DPVertex> results;
  double maxDistanceSquared = 0.0;
  unsigned int maxIndex = 0;

  DPVertex &start = vertices.front();
  DPVertex &end = vertices.back();

  for (unsigned int i = 1; i < vertices.size() - 1; ++i) {
    // TODO: Should we only be using concavity? (as we are using point-line distance, not point-segment distance)
    const double distanceSquared = MZH::pointLineDistanceSquared(vertices[i].point, start.point, end.point);
    if (distanceSquared <= maxDistanceSquared) continue;
    maxIndex = i;
    maxDistanceSquared = distanceSquared;
  }

  if (maxDistanceSquared > douglasPeuckerThreshold_ * douglasPeuckerThreshold_) {
    // Recurse, found a critical point
    std::vector<DPVertex> results1 = douglasPeucker(std::vector<DPVertex>(vertices.begin(), vertices.begin() + maxIndex + 1));
    std::vector<DPVertex> results2 = douglasPeucker(std::vector<DPVertex>(vertices.begin() + maxIndex, vertices.end()));
    results = results1;
    results.insert(results.end(), results2.begin(), results2.end() - 1);
  } else {
    // No critical points. Return endpoints
    results.push_back(start);
    results.push_back(end);
  }

  return results;
}

// Find non-crossing pocket cuts (intersecting paths) within a pocket
// TODO: Handle cup-shaped pockets
void ACD::computeUsefulPocketCuts() {
  std::shared_ptr<std::vector<std::unique_ptr<Face>>> faces = quickHull_.faces();
  std::vector<std::shared_ptr<PocketCut>> pocketCuts;

  // For each pocket (via its corresponding bridge)
  for (std::unique_ptr<Face> &face : *faces) {
    std::shared_ptr<HalfEdge> faceEdge = face->edge();
    std::shared_ptr<HalfEdge> curEdge = faceEdge;

    std::vector<Vertex *> pocketKnots;
    std::unordered_set<const Vertex *> uncrossableVertices;
    std::unordered_map<const Vertex *, std::vector<std::shared_ptr<PocketCut>>> vertexToCutsMap;

    // Get all knots for this pocket
    do {
      Vertex *vertex = curEdge->vertex();
      if (knots_.find(vertex) != knots_.end()) {
        // Save for easy iteration
        // Knot vertices cannot be crossed
        uncrossableVertices.insert(vertex);
      }

      curEdge = curEdge->next();
    } while (curEdge != faceEdge);

    // Compute all connecting paths with minimum weight using Dijkstra's
    for (auto sourceKnotIt = pocketKnots.begin(); sourceKnotIt != pocketKnots.end(); ++sourceKnotIt) {
      auto destKnotIt = sourceKnotIt;

      for (++destKnotIt; destKnotIt != pocketKnots.end(); ++destKnotIt) {
        // Init Dijkstra's algorithm structures
        std::vector<double> weights;
        std::vector<const Vertex *> previouses;
        weights.resize(vertices_->size(), std::numeric_limits<double>::infinity());
        previouses.resize(vertices_->size(), nullptr);

        std::priority_queue<queuePair, std::vector<queuePair>, queuePairComparator> queue;

        queue.push(std::make_pair(*sourceKnotIt, 0));
        weights[(*sourceKnotIt)->index()] = 0;

        // While there are vertices to process
        while (!queue.empty()) {
          const Vertex *closest = queue.top().first;
          queue.pop();
      
          // Check if target. Then success!
          if (closest == *destKnotIt) break;

          double weight = weights[closest->index()];

          // Generate a base path vector
          std::vector<const Vertex *> basePath;
          const Vertex *curVertex = closest;
          basePath.push_back(nullptr); // Placeholder for the neighbor
          basePath.push_back(curVertex);
          while (curVertex != *sourceKnotIt) {
            curVertex = previouses[curVertex->index()];
            basePath.push_back(curVertex);
          }

          // Add vertex neighbors with the best weight
          const std::vector<Vertex *> &neighbors = neighbors_[closest->index()];
          for (const Vertex *neighbor : neighbors) {
            std::vector<const Vertex *> neighborPath = basePath;
            neighborPath[0] = neighbor;
            double newWeight = weighPath(neighborPath);

            if (newWeight < weights[neighbor->index()]) {
              weights[neighbor->index()] = newWeight;
              previouses[neighbor->index()] = closest;
              queue.push(std::make_pair(neighbor, newWeight));
            }
          } //end-for
        } //end-while !queue.empty()

        // Create a new cut path
        std::shared_ptr<PocketCut> newPocketCut = std::make_shared<PocketCut>();
        pocketCuts.push_back(newPocketCut);
        std::vector<const Vertex *> &cutPath = newPocketCut->path;
        const Vertex *curVertex = *destKnotIt;
        cutPath.push_back(curVertex);

        // Save the path
        while (curVertex != *sourceKnotIt) {
          // Fail if can't find a path from dest to source
          if (previouses[curVertex->index()] == nullptr) {
            MPxCommand::displayError("Could not find cut path for vertex " + MZH::toS(curVertex->index())
              + " to target " + MZH::toS((*destKnotIt)->index())
              + " from source " + MZH::toS((*sourceKnotIt)->index()));
            returnStatus_ = MS::kFailure;
            return;
          }

          // Check and mark if the path intersects
          if (uncrossableVertices.find(curVertex) != uncrossableVertices.end()) {
            newPocketCut->noncrossing = false;
          } else {
            uncrossableVertices.insert(curVertex);
          }

          curVertex = previouses[curVertex->index()];
          cutPath.push_back(curVertex);
        }

        // Save the weight
        newPocketCut->weight = weighPath(newPocketCut->path);

        // Save both copies 
        pocketCutMap_.insert(std::make_pair(*sourceKnotIt, std::unordered_map<const Vertex *, std::shared_ptr<PocketCut>>()));
        pocketCutMap_.insert(std::make_pair(*destKnotIt, std::unordered_map<const Vertex *, std::shared_ptr<PocketCut>>()));
        auto &sourceMap = pocketCutMap_.at(*sourceKnotIt);
        auto &destMap = pocketCutMap_.at(*destKnotIt);
        sourceMap.insert(std::make_pair(*destKnotIt, newPocketCut));
        destMap.insert(std::make_pair(*sourceKnotIt, newPocketCut));
      } //end-for destKnotIt
    } //end-for sourceKnotIt
  } //end-foreach pocket
}

double ACD::weighPath(const std::vector<const Vertex *>& path) {
  // Average the concavity of all vertices in the path
  double averageConcavity = 0.0;
  for (const Vertex *vertex : path) {
    averageConcavity += concavities_[vertex->index()];
  }
  averageConcavity /= path.size();

  // Sum the curvature of the edges in the path
  double totalCurvature = 0.0;
  for (size_t pathIndex = 0; pathIndex < path.size() - 1; ++pathIndex) {
    totalCurvature += calculateCurvature(path[pathIndex], path[pathIndex + 1]);
  }

  // Validate and return the result
  if (MZH::fequal<double>(totalCurvature, 0, 0.000001)) {
    MPxCommand::displayError("Divide-by-zero error while summing the curvature of the minimum weight path from " +
      MZH::toS(path.front()->index()) + " to " + MZH::toS(path.back()->index()));
    returnStatus_ = MS::kFailure;
    return 0.0;
  }
  return averageConcavity / totalCurvature;
}

double ACD::calculateCurvature(const Vertex *vertexA, const Vertex *vertexB) {
  // Check if previously calculated
  auto vertexAIt = curvatures_.find(vertexA);
  if (vertexAIt != curvatures_.end()) {
    auto vertexBIt = (*vertexAIt).second.find(vertexB);
    if (vertexBIt != (*vertexAIt).second.end()) {
      return (*vertexBIt).second;
    }
  }

  // Get the respective half edge
  

  // Store the edge curvature
  return 0.0;
}
