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

  for (auto faceIt = faces_.begin(); faceIt != faces_.end(); ++faceIt) {
    HalfEdge *faceEdge = (*faceIt)->edge();
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

void QuickHull::buildHull() {
  buildSimplexHull();
}

void QuickHull::buildSimplexHull() {
  // Init vars
  Vertex *vertices[4];
  double maxDistance;

  for (unsigned int i = 0; i < 4; ++i) {
    vertices[i] = nullptr;
  }

  computeMinMax(vertices[0], vertices[1]);

  // Find farthest point v2
  maxDistance = -1.0;
  for (size_t i = 0; i < pointBuffer_.size(); ++i) {
    Vertex &vertex = pointBuffer_[i];
    if (&vertex == vertices[0] || &vertex == vertices[1]) continue;
    const double distance = MZH::pointLineDistance(vertex.point(), vertices[0]->point(), vertices[1]->point());
    if (distance > maxDistance) {
      maxDistance = distance;
      vertices[2] = &vertex;
    }
  }
  MPxCommand::displayInfo("Found furthest point v2 " + MZH::toS(vertices[2]->point()) + " with distance " + maxDistance);

  MVector v012normal = MZH::unitPlaneNormal(vertices[0]->point(), vertices[1]->point(), vertices[2]->point());

  // Find farthest point v3
  maxDistance = -1.0;
  for (size_t i = 0; i < pointBuffer_.size(); ++i) {
    Vertex &vertex = pointBuffer_[i];
    if (&vertex == vertices[0] || &vertex == vertices[1] || &vertex == vertices[2]) continue;
    const double distance = std::abs(MZH::pointPlaneDistance(vertex.point(), vertices[0]->point(), v012normal));
    if (distance > maxDistance) {
      maxDistance = distance;
      vertices[3] = &vertex;
    }
  }
  MPxCommand::displayInfo("Found furthest point v3 " + MZH::toS(vertices[3]->point()) + " with distance " + maxDistance);

  // Add vertices to debug vector
  for (unsigned int i = 0; i < 4; ++i) {
    debugVertices.push_back(vertices[i]);
  }

  // Generate faces
  if (MZH::pointPlaneDistance(vertices[3]->point(), vertices[0]->point(), v012normal) < 0) {
    // v012plane not facing vertices[3]
    faces_.emplace_back(Face::createTriangle(vertices[0], vertices[1], vertices[2]));
    faces_.emplace_back(Face::createTriangle(vertices[3], vertices[1], vertices[0]));
    faces_.emplace_back(Face::createTriangle(vertices[3], vertices[2], vertices[1]));
    faces_.emplace_back(Face::createTriangle(vertices[3], vertices[0], vertices[2]));

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
    faces_.emplace_back(Face::createTriangle(vertices[0], vertices[2], vertices[1]));
    faces_.emplace_back(Face::createTriangle(vertices[3], vertices[0], vertices[1]));
    faces_.emplace_back(Face::createTriangle(vertices[3], vertices[1], vertices[2]));
    faces_.emplace_back(Face::createTriangle(vertices[3], vertices[2], vertices[0]));

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
  for (size_t i = 0; i < pointBuffer_.size(); ++i) {
    Vertex &vertex = pointBuffer_[i];
    if (&vertex == vertices[0] || &vertex == vertices[1] || &vertex == vertices[2] || &vertex == vertices[3]) continue;
    maxDistance = tolerance_;
    Face *maxFace = nullptr;
    for (auto faceIt = faces_.begin(); faceIt != faces_.end(); ++faceIt) {
      const double distance = (*faceIt)->pointPlaneDistance(vertex.point());
      if (distance > maxDistance) {
        maxDistance = distance;
        maxFace = (*faceIt).get();
      }
    }
    if (maxFace) {
      addVertexToFace(&vertex, maxFace);
    }
  }
}

MStatus QuickHull::computeMinMax(Vertex *&v0, Vertex *&v1) {
  MStatus status;

  // Initialize extremes
  Vertex *extremes[3][2];
  for (unsigned int i = 0; i < 3; ++i) {
    extremes[i][0] = &pointBuffer_[0];
    extremes[i][1] = &pointBuffer_[0];
  }
  
  // Find axial extremes
  for (size_t i = 0; i < pointBuffer_.size(); ++i) {
    Vertex &vertex = pointBuffer_[i];
    for (unsigned int i = 0; i < 3; ++i) {
      if (vertex.point()[i] < extremes[i][0]->point()[i]) {
        extremes[i][0] = &vertex;
      }
      if (vertex.point()[i] < extremes[i][1]->point()[i]) {
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

void QuickHull::initBuffers(unsigned int numPoints) {
  pointBuffer_.reserve(numPoints);
  faces_.clear();
  claimed_.clear();
}

void QuickHull::setPoints(const MPointArray &points) {
  for (unsigned int i = 0; i < points.length(); ++i) {
    pointBuffer_.push_back(Vertex(points[i]));
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
