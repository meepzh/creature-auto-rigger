#include "QuickHull.h"

#include <limits>
#include "MathUtils.h"
#include <maya/MPxCommand.h>
#include "Utils.h"

QuickHull::QuickHull(const MPointArray &points, MStatus *status) {
  MStatus returnStatus;
  returnStatus = build(points);
  if (status) *status = returnStatus;
}

MStatus QuickHull::build(const MPointArray &points) {
  if (points.length() < 4) {
    MStatus returnStatus(MS::kFailure);
    returnStatus.perror("At least 4 points required");
    return returnStatus;
  }

  initBuffers(points.length());
  setPoints(points);
  buildHull();

  return MS::kSuccess;
}

void QuickHull::mayaExport(int &numVertices, int &numPolygons, MPointArray &vertexArray, MIntArray &polygonCounts, MIntArray &polygonConnects) {
  numVertices = 0;
  numPolygons = (int) faces_.size();
  vertexArray.clear();
  polygonCounts.clear();
  polygonConnects.clear();

  for (std::unique_ptr<Face> &face : faces_) {
    HalfEdge *faceEdge = face->edge();
    HalfEdge *curEdge = faceEdge;
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

void QuickHull::addNewFaces(Vertex *eyeVertex) {
  newFaces_.clear();


}

void QuickHull::addVertexToHull(Vertex *eyeVertex) {
  MPxCommand::displayInfo("Adding point " + MZH::toS(eyeVertex->point())
    + " at height " + eyeVertex->face()->pointPlaneDistance(eyeVertex->point()));

  horizon_.clear();
  unclaimed_.clear();

  removeVertexFromFace(eyeVertex, eyeVertex->face());
  computeHorizon(eyeVertex->point(), nullptr, eyeVertex->face());

  MPxCommand::displayInfo("Horizon size " + MZH::toS<unsigned int>((unsigned int) horizon_.size()));
}

void QuickHull::buildHull() {
  buildSimplexHull();

  Vertex *eyeVertex;
  unsigned int iterations = 0;
  /*while (eyeVertex = nextVertexToAdd()) {
    ++iterations;
    addVertexToHull(eyeVertex);
  }*/

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
  for (Vertex &vertex : vertices_) {
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
  for (Vertex &vertex : vertices_) {
    if (&vertex == initialVertices[0] || &vertex == initialVertices[1] || &vertex == initialVertices[2]) continue;
    const double distance = std::abs(MZH::pointPlaneDistance(vertex.point(), initialVertices[0]->point(), v012normal));
    if (distance > maxDistance) {
      maxDistance = distance;
      initialVertices[3] = &vertex;
    }
  }
  MPxCommand::displayInfo("Found furthest point v3 " + MZH::toS(initialVertices[3]->point()) + " with distance " + maxDistance);

  // Add vertices to debug vector
  for (unsigned int i = 0; i < 4; ++i) {
    debugVertices.push_back(initialVertices[i]);
  }

  // Generate faces
  if (MZH::pointPlaneDistance(initialVertices[3]->point(), initialVertices[0]->point(), v012normal) < 0) {
    // v012plane not facing vertices[3]
    faces_.emplace_back(Face::createTriangle(initialVertices[0], initialVertices[1], initialVertices[2]));
    faces_.emplace_back(Face::createTriangle(initialVertices[3], initialVertices[1], initialVertices[0]));
    faces_.emplace_back(Face::createTriangle(initialVertices[3], initialVertices[2], initialVertices[1]));
    faces_.emplace_back(Face::createTriangle(initialVertices[3], initialVertices[0], initialVertices[2]));

    // Set opposite half-edges
    for (size_t i = 0; i < 3; ++i) {
      size_t j = (i + 1) % 3;
      // Link faces_[i] with faces_[0]
      faces_[i + 1]->edge(2)->setOpposite(faces_[0]->edge((int) j));
      // Link faces_[i] with faces_[i + 1]
      faces_[i + 1]->edge(1)->setOpposite(faces_[j + 1]->edge(0));
    }
  } else {
    // v012plane facing vertices[3]
    faces_.emplace_back(Face::createTriangle(initialVertices[0], initialVertices[2], initialVertices[1]));
    faces_.emplace_back(Face::createTriangle(initialVertices[3], initialVertices[0], initialVertices[1]));
    faces_.emplace_back(Face::createTriangle(initialVertices[3], initialVertices[1], initialVertices[2]));
    faces_.emplace_back(Face::createTriangle(initialVertices[3], initialVertices[2], initialVertices[0]));

    // Set opposite half-edges
    for (size_t i = 0; i < 3; ++i) {
      size_t j = (i + 1) % 3;
      // Link faces_[i] with faces_[0]
      faces_[i + 1]->edge(2)->setOpposite(faces_[0]->edge((int) (3 - i)));
      // Link faces_[i] with faces_[i + 1]
      faces_[i + 1]->edge(0)->setOpposite(faces_[j + 1]->edge(1));
    }
  }

  // Assign vertices to faces
  for (Vertex &vertex : vertices_) {
    if (&vertex == initialVertices[0] || &vertex == initialVertices[1]
      || &vertex == initialVertices[2] || &vertex == initialVertices[3]) continue;
    maxDistance = tolerance_;
    Face *maxFace = nullptr;
    for (std::unique_ptr<Face> &face : faces_) {
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

void QuickHull::computeHorizon(const MPoint &eyePoint, HalfEdge *crossedEdge, Face *face) {
  deleteFaceVertices(face);
  face->flag = Face::Flag::DELETED;
  HalfEdge *edge;

  if (crossedEdge == nullptr) {
    crossedEdge = face->edge();
    edge = crossedEdge;
  } else {
    // crossedEdge already analyzed
    edge = crossedEdge->next();
  }

  do {
    HalfEdge *oppositeEdge = edge->opposite();
    Face *oppositeFace = oppositeEdge->face();
    if (oppositeFace->flag == Face::Flag::VISIBLE) {
      if  (oppositeFace->pointPlaneDistance(eyePoint)) {
        computeHorizon(eyePoint, oppositeEdge, oppositeFace);
      } else {
        horizon_.push_back(edge);
      }
    }
    edge = edge->next();
  } while (edge != crossedEdge);
}

MStatus QuickHull::computeMinMax(Vertex *&v0, Vertex *&v1) {
  MStatus status;

  // Initialize extremes
  Vertex *extremes[3][2];
  for (unsigned int i = 0; i < 3; ++i) {
    extremes[i][0] = &vertices_[0];
    extremes[i][1] = &vertices_[0];
  }
  
  // Find axial extremes
  for (size_t i = 0; i < vertices_.size(); ++i) {
    Vertex &vertex = vertices_[i];
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

}

void QuickHull::initBuffers(unsigned int numPoints) {
  vertices_.reserve(numPoints);
  faces_.clear();
  claimed_.clear();
}

Vertex *QuickHull::nextVertexToAdd() {
  if (claimed_.empty()) return nullptr;
  
  Vertex *eyeVertex = nullptr;
  double maxDistance = -1.0;
  const Face *eyeFace = claimed_.front()->face();

  for (auto vertexIt = eyeFace->outside(); vertexIt != claimed_.end() && (*vertexIt)->face() == eyeFace; ++vertexIt) {
    const double distance = eyeFace->pointPlaneDistance((*vertexIt)->point());
    if (distance > maxDistance) {
      eyeVertex = *vertexIt;
      maxDistance = distance;
    }
  }

  return eyeVertex;
}

void QuickHull::setPoints(const MPointArray &points) {
  for (unsigned int i = 0; i < points.length(); ++i) {
    vertices_.push_back(Vertex(points[i]));
  }
}

void QuickHull::addVertexToFace(Vertex *vertex, Face *face) {
  vertex->setFace(face);
  if (face->hasOutside()) {
    claimed_.insert(face->outside(), vertex);
    face->setOutside(--face->outside());
  } else {
    claimed_.push_back(vertex);
    face->setOutside(--claimed_.end());
  }
}

void QuickHull::removeVertexFromFace(Vertex *vertex, Face *face) {
  auto vertexIt = face->outside();
  if (vertex == *vertexIt) {
    vertexIt = claimed_.erase(vertexIt);
    if (vertexIt == claimed_.end()) {
      face->clearOutside();
    } else {
      face->setOutside(vertexIt);
    }
  } else {
    // Find vertex iterator
    for (; vertexIt != claimed_.end() && (*vertexIt)->face() == face; ++vertexIt) {
      if (*vertexIt == vertex) {
        claimed_.erase(vertexIt);
        continue;
      }
    } //end-for
  } //end-if
}
