#include "QuickHull.h"

#include <cassert>
#include <limits>
#include <maya/MPxCommand.h>

#include "MathUtils.h"
#include "Utils.h"

QuickHull::QuickHull(MItMeshVertex &vertexIt, int maxIterations, MStatus *status)
    : maxIterations_(maxIterations) {
  MStatus returnStatus;
  returnStatus = build(vertexIt);
  if (status) *status = returnStatus;
}

MStatus QuickHull::build(MItMeshVertex &vertexIt) {
  if (vertexIt.count() < 4) {
    MStatus returnStatus(MS::kFailure);
    returnStatus.perror("At least 4 points required");
    return returnStatus;
  }

  initBuffers(vertexIt.count());
  setPoints(vertexIt);
  buildHull();

  return MS::kSuccess;
}

void QuickHull::mayaExport(int &numVertices, int &numPolygons, MPointArray &vertexArray, MIntArray &polygonCounts, MIntArray &polygonConnects) {
  numVertices = 0;
  numPolygons = (int) faces_->size();
  vertexArray.clear();
  polygonCounts.clear();
  polygonConnects.clear();

  for (std::unique_ptr<Face> &face : *faces_) {
    std::shared_ptr<HalfEdge> faceEdge = face->edge();
    std::shared_ptr<HalfEdge> curEdge = faceEdge;
    int polygonCount = 0;
    do {
      vertexArray.append(curEdge->vertex()->point());
      polygonCount++;
      polygonConnects.append(numVertices++);
      curEdge = curEdge->next();
    } while (curEdge != faceEdge);
    polygonCounts.append(polygonCount);
  }
}

std::shared_ptr<std::vector<std::unique_ptr<Face>>> QuickHull::faces() {
  return faces_;
}

std::shared_ptr<std::vector<Vertex>> QuickHull::vertices() {
  return vertices_;
}

void QuickHull::addNewFaces(Vertex *eyeVertex) {
  newFaces_.clear();

  assert(!horizon_.empty());

  std::shared_ptr<HalfEdge> firstSideEdge;
  std::shared_ptr<HalfEdge> prevSideEdge;
  
  for (std::shared_ptr<HalfEdge> horizonEdge : horizon_) {
    std::shared_ptr<HalfEdge> sideEdge = addAdjoiningFace(eyeVertex, horizonEdge);

    if (!firstSideEdge) {
      firstSideEdge = sideEdge;
    } else {
      sideEdge->next()->setOpposite(prevSideEdge);
    }
    
    newFaces_.push_back(sideEdge->face());
    prevSideEdge = sideEdge;
  }

  firstSideEdge->next()->setOpposite(prevSideEdge);
}

void QuickHull::addVertexToHull(Vertex *eyeVertex) {
  MPxCommand::displayInfo("Adding point " + MZH::toS(eyeVertex->point())
    + " at height " + eyeVertex->face()->pointPlaneDistance(eyeVertex->point()));

  horizon_.clear();
  unclaimed_.clear();

  removeVertexFromFace(eyeVertex, eyeVertex->face());
  computeHorizon(eyeVertex->point(), nullptr, eyeVertex->face());

  addNewFaces(eyeVertex);

  // First Merge
  for (Face *face : newFaces_) {
    if (face->flag != Face::Flag::VISIBLE) continue;
    while (doAdjacentMerge(face, MergeType::NONCONVEX_WRT_LARGER_FACE));
  }

  // Second Merge
  for (Face *face : newFaces_) {
    if (face->flag != Face::Flag::NONCONVEX) continue;
    face->flag = Face::Flag::VISIBLE;
    while (doAdjacentMerge(face, MergeType::NONCONVEX));
  }

  resolveUnclaimedPoints();
}

void QuickHull::sanityCheck() {
  for (std::unique_ptr<Face> &face : *faces_) {
    face->checkConsistency(true);
  }
}

void QuickHull::buildHull() {
  buildSimplexHull();

  Vertex *eyeVertex;
  int iterations = 0;
  while (eyeVertex = nextVertexToAdd()) {
    if (maxIterations_ >= 0 && iterations >= maxIterations_) break;
    addVertexToHull(eyeVertex);
    clearDeletedFaces();
    ++iterations;
    sanityCheck();
  }

  MPxCommand::displayInfo("Completed with " + MZH::toS(iterations) + " iterations");
}

void QuickHull::buildSimplexHull() {
  // Init vars
  Vertex *initialVertices[4];
  double maxDistance;

  for (unsigned int i = 0; i < 4; ++i) {
    initialVertices[i] = nullptr;
  }

  computeMinMax(initialVertices[0], initialVertices[1]);

  // Find farthest point v2
  maxDistance = -1.0;
  for (Vertex &vertex : *vertices_) {
    if (&vertex == initialVertices[0] || &vertex == initialVertices[1]) continue;
    const double distance = MZH::pointLineDistance(vertex.point(), initialVertices[0]->point(), initialVertices[1]->point());
    if (distance > maxDistance) {
      maxDistance = distance;
      initialVertices[2] = &vertex;
    }
  }
  MPxCommand::displayInfo("Found furthest point v2 " + MZH::toS(initialVertices[2]->point()) + " with distance " + maxDistance);

  MVector v012normal = MZH::unitPlaneNormal(initialVertices[0]->point(), initialVertices[1]->point(), initialVertices[2]->point());

  // Find farthest point v3
  maxDistance = -1.0;
  for (Vertex &vertex : *vertices_) {
    if (&vertex == initialVertices[0] || &vertex == initialVertices[1] || &vertex == initialVertices[2]) continue;
    const double distance = std::abs(MZH::pointPlaneDistance(vertex.point(), initialVertices[0]->point(), v012normal));
    if (distance > maxDistance) {
      maxDistance = distance;
      initialVertices[3] = &vertex;
    }
  }
  MPxCommand::displayInfo("Found furthest point v3 " + MZH::toS(initialVertices[3]->point()) + " with distance " + maxDistance);

  // Generate faces
  if (MZH::pointPlaneDistance(initialVertices[3]->point(), initialVertices[0]->point(), v012normal) < 0) {
    // v012plane not facing vertices[3]
    faces_->emplace_back(Face::createTriangle(initialVertices[0], initialVertices[1], initialVertices[2]));
    faces_->emplace_back(Face::createTriangle(initialVertices[3], initialVertices[1], initialVertices[0]));
    faces_->emplace_back(Face::createTriangle(initialVertices[3], initialVertices[2], initialVertices[1]));
    faces_->emplace_back(Face::createTriangle(initialVertices[3], initialVertices[0], initialVertices[2]));

    // Set opposite half-edges
    for (size_t i = 0; i < 3; ++i) {
      size_t j = (i + 1) % 3;
      // Link faces_[i] with faces_[0]
      (*faces_)[i + 1]->edge(2)->setOpposite((*faces_)[0]->edge((int) j));
      // Link faces_[i] with faces_[i + 1]
      (*faces_)[i + 1]->edge(1)->setOpposite((*faces_)[j + 1]->edge(0));
    }
  } else {
    // v012plane facing vertices[3]
    faces_->emplace_back(Face::createTriangle(initialVertices[0], initialVertices[2], initialVertices[1]));
    faces_->emplace_back(Face::createTriangle(initialVertices[3], initialVertices[0], initialVertices[1]));
    faces_->emplace_back(Face::createTriangle(initialVertices[3], initialVertices[1], initialVertices[2]));
    faces_->emplace_back(Face::createTriangle(initialVertices[3], initialVertices[2], initialVertices[0]));

    // Set opposite half-edges
    for (size_t i = 0; i < 3; ++i) {
      size_t j = (i + 1) % 3;
      // Link faces_[i] with faces_[0]
      (*faces_)[i + 1]->edge(2)->setOpposite((*faces_)[0]->edge((int) (3 - i)));
      // Link faces_[i] with faces_[i + 1]
      (*faces_)[i + 1]->edge(0)->setOpposite((*faces_)[j + 1]->edge(1));
    }
  }

  // Assign vertices to faces
  for (Vertex &vertex : *vertices_) {
    if (&vertex == initialVertices[0] || &vertex == initialVertices[1]
      || &vertex == initialVertices[2] || &vertex == initialVertices[3]) continue;
    maxDistance = tolerance_;
    Face *maxFace = nullptr;
    for (std::unique_ptr<Face> &face : *faces_) {
      const double distance = face->pointPlaneDistance(vertex.point());
      if (distance > maxDistance) {
        maxDistance = distance;
        maxFace = face.get();
      }
    }
    if (maxFace) {
      addVertexToFace(&vertex, maxFace);
    }
  }
}

void QuickHull::clearDeletedFaces() {
  for (auto it = faces_->begin(); it != faces_->end(); ) {
    if ((*it)->flag == Face::Flag::DELETED) {
      it = faces_->erase(it);
    } else {
      ++it;
    }
  }
}

void QuickHull::computeHorizon(const MPoint &eyePoint, std::shared_ptr<HalfEdge> crossedEdge, Face *face) {
  deleteFaceVertices(face);
  face->flag = Face::Flag::DELETED;

  std::shared_ptr<HalfEdge> edge;

  if (!crossedEdge) {
    crossedEdge = face->edge();
    edge = crossedEdge;
  } else {
    // crossedEdge already analyzed
    edge = crossedEdge->next();
  }

  int iterations = 0;
  do {
    ++iterations;
    assert(!(edge->opposite().expired()));
    std::shared_ptr<HalfEdge> oppositeEdge = edge->opposite().lock();
    Face *oppositeFace = oppositeEdge->face();
    if (oppositeFace->flag == Face::Flag::VISIBLE) {
      if (oppositeFace->pointPlaneDistance(eyePoint) > tolerance_) {
        computeHorizon(eyePoint, oppositeEdge, oppositeFace);
      } else {
        horizon_.push_back(edge);
      }
    }
    assert(edge->next());
    edge = edge->next();
  } while (edge != crossedEdge);
}

MStatus QuickHull::computeMinMax(Vertex *&v0, Vertex *&v1) {
  MStatus status;

  // Initialize extremes
  Vertex *extremes[3][2];
  for (unsigned int i = 0; i < 3; ++i) {
    extremes[i][0] = &(*vertices_)[0];
    extremes[i][1] = &(*vertices_)[0];
  }
  
  // Find axial extremes
  for (size_t i = 0; i < vertices_->size(); ++i) {
    Vertex &vertex = (*vertices_)[i];
    for (unsigned int i = 0; i < 3; ++i) {
      if (vertex.point()[i] < extremes[i][0]->point()[i]) {
        extremes[i][0] = &vertex;
      }
      if (vertex.point()[i] > extremes[i][1]->point()[i]) {
        extremes[i][1] = &vertex;
      }
    }
  }
  for (unsigned int i = 0; i < 3; ++i) {
    MPxCommand::displayInfo(MString("Found minimum ") + i + ": " + MZH::toS(extremes[i][0]->point()));
    MPxCommand::displayInfo(MString("Found maximum ") + i + ": " + MZH::toS(extremes[i][1]->point()));
  }

  // Find longest pair
  double maxDistance = -1.0;
  for (unsigned int i = 0; i < 6; ++i) {
    for (unsigned int j = i + 1; j < 6; ++j) {
      Vertex *tempA = extremes[i / 2][i % 2];
      Vertex *tempB = extremes[j / 2][j % 2];
      const double distance = tempA->point().distanceTo(tempB->point());
      if (distance > maxDistance) {
        v0 = tempA;
        v1 = tempB;
        maxDistance = distance;
      }
    }
  }
  MPxCommand::displayInfo("Longest pair from "
    + MZH::toS(v0->point()) + " to " + MZH::toS(v1->point())
    + " with distance " + maxDistance);

  // Calculate tolerance
  tolerance_ = 3.0 * std::numeric_limits<double>::epsilon()
    * std::max(std::abs(extremes[0][0]->point()[0]), std::abs(extremes[0][1]->point()[0]))
    * std::max(std::abs(extremes[1][0]->point()[1]), std::abs(extremes[1][1]->point()[1]))
    * std::max(std::abs(extremes[2][0]->point()[2]), std::abs(extremes[2][1]->point()[2]));
  MPxCommand::displayInfo("Tolerance: " + MZH::toS(tolerance_));

  // Check tolerance
  maxDistance = -1.0;
  for (unsigned int i = 0; i < 3; ++i) {
    const double distance = extremes[i][0]->point().distanceTo(extremes[i][1]->point());
    if (distance > maxDistance) maxDistance = distance;
  }
  if (maxDistance < tolerance_) {
    status = MS::kFailure;
    status.perror("Tolerance not met by most extreme points");
  }

  return status;
}

void QuickHull::deleteFaceVertices(Face *face, Face *absorbingFace) {
  std::list<Vertex *> faceVertices = removeAllVerticesFromFace(face);
  
  if (faceVertices.empty()) return;
  if (!absorbingFace) {
    // Let some other face claim it
    for (Vertex *vertex : faceVertices) {
      unclaimed_.push_back(vertex);
    }
    return;
  }

  for (Vertex *vertex : faceVertices) {
    const double distance = absorbingFace->pointPlaneDistance(vertex->point());
    if (distance > tolerance_) {
      addVertexToFace(vertex, absorbingFace);
    } else {
      unclaimed_.push_back(vertex);
    }
  }
}

void QuickHull::initBuffers(unsigned int numPoints) {
  faces_ = std::make_shared<std::vector<std::unique_ptr<Face>>>();
  vertices_ = std::make_shared<std::vector<Vertex>>();
  vertices_->reserve(numPoints);
  claimed_.clear();
}

Vertex *QuickHull::nextVertexToAdd() {
  if (claimed_.empty()) return nullptr;
  
  Vertex *eyeVertex = nullptr;
  double maxDistance = -1.0;
  //const Face *eyeFace = claimed_.front()->face();
  const Face *eyeFace = claimed_.front()->face();
  assert(eyeFace->outside() == claimed_.begin());

  for (auto vertexIt = eyeFace->outside(); vertexIt != claimed_.end() && (*vertexIt)->face() == eyeFace; ++vertexIt) {
    const double distance = eyeFace->pointPlaneDistance((*vertexIt)->point());
    if (distance > maxDistance) {
      eyeVertex = *vertexIt;
      maxDistance = distance;
    }
  }

  return eyeVertex;
}

double QuickHull::oppositeFaceDistance(HalfEdge *he) const {
  return he->face()->pointPlaneDistance(he->opposite().lock()->face()->centroid());
}

void QuickHull::resolveUnclaimedPoints() {
  for (Vertex *vertex : unclaimed_) {
    double maxDistance = tolerance_;
    Face *maxFace = nullptr;

    for (Face *face : newFaces_) {
      if (face->flag == Face::Flag::VISIBLE) {
        const double distance = face->pointPlaneDistance(vertex->point());
        if (distance > maxDistance) {
          maxDistance = distance;
          maxFace = face;
        }
        if (maxDistance > 1000 * tolerance_) break;
      }
    }

    if (maxFace) {
      addVertexToFace(vertex, maxFace);
    }
  }
}

void QuickHull::setPoints(MItMeshVertex &vertexIt) {
  for (; !vertexIt.isDone(); vertexIt.next()) {
    vertices_->push_back(Vertex(vertexIt.position(MSpace::kWorld), vertexIt.index()));
  }
}

void QuickHull::addVertexToFace(Vertex *vertex, Face *face) {
  vertex->setFace(face);
  if (face->hasOutside()) {
    face->setOutside(claimed_.insert(face->outside(), vertex));
  } else {
    claimed_.push_back(vertex);
    face->setOutside(--claimed_.end());
    assert(face->outside() != claimed_.end());
  }
}

std::shared_ptr<HalfEdge> QuickHull::addAdjoiningFace(Vertex *eyeVertex, std::shared_ptr<HalfEdge> horizonEdge) {
  faces_->emplace_back(Face::createTriangle(eyeVertex, horizonEdge->prevVertex(), horizonEdge->vertex()));
  Face *face = faces_->back().get();
  face->edge(-1)->setOpposite(horizonEdge->opposite());
  return face->edge();
}

bool QuickHull::doAdjacentMerge(Face *face, MergeType mergeType) {
  std::shared_ptr<HalfEdge> faceEdgeShared = face->edge();
  std::shared_ptr<HalfEdge> edgeShared = faceEdgeShared;
  HalfEdge *edge = edgeShared.get();
  bool convex = true;
  unsigned int counter = 0;
  
  do {
    assert(counter < face->numVertices());
    
    assert(!(edge->opposite().expired()));
    HalfEdge *opposite = edge->opposite().lock().get();
    Face *oppositeFace = opposite->face();
    bool merge = false;

    if (mergeType == MergeType::NONCONVEX) {
      if (oppositeFaceDistance(edge) > -tolerance_ || oppositeFaceDistance(opposite) > -tolerance_) {
        merge = true;
      }
    } else {
      if (face->area() > oppositeFace->area()) {
        if (oppositeFaceDistance(edge) > -tolerance_) {
          merge = true;
        } else if (oppositeFaceDistance(opposite) > -tolerance_) {
          convex = true;
        }
      } else {
        if (oppositeFaceDistance(opposite) > -tolerance_) {
          merge = true;
        } else if (oppositeFaceDistance(edge) > -tolerance_) {
          convex = true;
        }
      }

      if (merge) {
        std::vector<Face *> discardedFaces;
        face->mergeAdjacentFaces(edgeShared, discardedFaces);
        for (Face *discardedFace : discardedFaces) {
          deleteFaceVertices(discardedFace, face);
        }
        return true;
      }
    } // end-if-else MergeType::NONCONVEX

    edgeShared = edge->next();
    edge = edgeShared.get();
    ++counter;
  } while (edgeShared != faceEdgeShared);

  if (!convex) {
    face->flag = Face::Flag::NONCONVEX;
  }

  face->checkConsistency();

  return false;
}

std::list<Vertex *> QuickHull::removeAllVerticesFromFace(Face *face) {
  std::list<Vertex *> faceVertices;
  
  if (face->hasOutside()) {
    auto vertexIt = face->outside();
    do {
      faceVertices.push_back(*vertexIt);
      vertexIt = claimed_.erase(vertexIt);
    } while (vertexIt != claimed_.end() && (*vertexIt)->face() == face);
    face->clearOutside();
  }

  return faceVertices;
}

void QuickHull::removeVertexFromFace(Vertex *vertex, Face *face) {
  auto vertexIt = face->outside();
  if (*vertexIt == vertex) {
    vertexIt = claimed_.erase(vertexIt);
    if (vertexIt == claimed_.end() || (*vertexIt)->face() != face) {
      face->clearOutside();
    } else {
      face->setOutside(vertexIt);
    }
  } else {
    // Find vertex iterator
    for (; vertexIt != claimed_.end() && (*vertexIt)->face() == face; ++vertexIt) {
      if (*vertexIt == vertex) {
        claimed_.erase(vertexIt);
        break;
      }
    } // end-for
  } // end-if-else
}
