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
}

void QuickHull::buildHull() {
  buildSimplexHull();
}

void QuickHull::buildSimplexHull() {
  // Init vars
  Vertex *v0 = nullptr;
  Vertex *v1 = nullptr;
  Vertex *v2 = nullptr;
  Vertex *v3 = nullptr;
  double maxDistance = -1.0;

  computeMinMax(v0, v1);

  // Find farthest point v2
  for (size_t i = 0; i < pointBuffer_.size(); ++i) {
    Vertex &vertex = pointBuffer_[i];
    if (&vertex == v0 || &vertex == v1) continue;
    const double distance = MZH::pointLineDistance(vertex.point(), v0->point(), v1->point());
    if (distance > maxDistance) {
      maxDistance = distance;
      v2 = &vertex;
    }
  }
  MPxCommand::displayInfo("Found furthest point v2 " + MZH::toS(v2->point()) + " with distance " + maxDistance);

  MVector v012normal = MZH::unitPlaneNormal(v0->point(), v1->point(), v2->point());

  // Find farthest point v3
  for (size_t i = 0; i < pointBuffer_.size(); ++i) {
    Vertex &vertex = pointBuffer_[i];
    if (&vertex == v0 || &vertex == v1 || &vertex == v2) continue;
    const double distance = MZH::pointPlaneDistance(vertex.point(), v0->point(), v012normal);
    if (distance > maxDistance) {
      maxDistance = distance;
      v3 = &vertex;
    }
  }
  MPxCommand::displayInfo("Found furthest point v3 " + MZH::toS(v3->point()) + " with distance " + maxDistance);
}

void QuickHull::computeMinMax(Vertex *&v0, Vertex *&v1) {
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
      double distance = tempA->point().distanceTo(tempB->point());
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
}

void QuickHull::initBuffers(unsigned int numPoints) {
  pointBuffer_.reserve(numPoints);
  faces_.clear();
}

void QuickHull::setPoints(const MPointArray &points) {
  for (unsigned int i = 0; i < points.length(); ++i) {
    pointBuffer_.push_back(Vertex(points[i]));
  }
}