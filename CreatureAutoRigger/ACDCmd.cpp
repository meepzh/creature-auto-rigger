#include "ACDCmd.h"

#include <maya/MFnMesh.h>
#include <maya/MFnNurbsCurve.h>
#include <maya/MGlobal.h>
#include <maya/MItMeshVertex.h>
#include <maya/MItSelectionList.h>

#include "ACD.h"
#include "MathUtils.h"
#include "QuickHull.h"
#include "Utils.h"

MStatus ACDCmd::doIt(const MArgList& args) {
  MStatus status = MS::kSuccess;

  MSelectionList sList;
  MGlobal::getActiveSelectionList(sList);

  MItSelectionList sListIt(sList);
  bool selectedMesh = false;
  
  for (; !sListIt.isDone(); sListIt.next()) {
    MDagPath dagPath;
    sListIt.getDagPath(dagPath);
    
    if (dagPath.node().hasFn(MFn::kMesh)) {
      selectedMesh = true;
      runACD(dagPath, &status);
    } else if (dagPath.node().hasFn(MFn::kTransform) && dagPath.hasFn(MFn::kMesh)) {
      selectedMesh = true;
      runACD(dagPath, &status);
    }
  }

  if (sListIt.isDone() && !selectedMesh) {
    displayError("No shape or transform selected");
    return MS::kFailure;
  }

  if (status == MS::kSuccess) setResult("ACDCmd command executed!\n");
  return status;
}

void *ACDCmd::creator() {
  return new ACDCmd;
}

void ACDCmd::runACD(MDagPath dagPath, MStatus *status) {
  MItMeshVertex vertexIt(dagPath, MObject::kNullObj, status);
  if (MZH::hasError(*status, "Failed to get vertex iterator")) return;
  if (vertexIt.isDone()) {
    displayWarning("No vertices in object");
    return;
  }

  ACD acd(vertexIt, 0.04, 0.002, status);

  /*
  std::vector<Vertex *> &hullVertices = acd.hullVertices();
  for (Vertex *vertex : hullVertices) {
    *status = MZH::createLocator(dgModifier, vertex->point(), "hullVertex#", false);
    if (MZH::hasError(*status, "Error creating hull vertex locator")) return;
  }
  */

  // Draw projected hull edges
  MFnNurbsCurve nurbsFn;
  pEdgeMap &projectedEdges = acd.projectedEdges();
  for (auto it1 = projectedEdges.begin(); it1 != projectedEdges.end(); ++it1) {
    std::unordered_map<Vertex *, std::shared_ptr<std::vector<Vertex *>>> &edgeMap = it1->second;

    for (auto it2 = it1->second.begin(); it2 != it1->second.end(); ++it2) {
      std::shared_ptr<std::vector<Vertex *>> &path = it2->second;

      MPointArray controlVerts;
      MDoubleArray knots;
      double time = 0;
      
      for (size_t i = 0; i < path->size(); ++i) {
        controlVerts.append((*path)[i]->point());
        knots.append(time++);
      }
      controlVerts.append((*path)[0]->point());
      knots.append(time++);
      
      MObject curve = nurbsFn.create(controlVerts, knots, (unsigned int) 1, MFnNurbsCurve::kClosed, false, false, MObject::kNullObj, status);
      *status = dgModifier.renameNode(curve, "dijkstraPath#");
    }
  }
  
  // Convexities prep
  MFnMesh meshFn(dagPath, status);
  if (MZH::hasError(*status, "Error converting selection to mesh")) return;

  std::vector<double> &convexities = acd.concavities();
  MPxCommand::displayInfo("Average convexity: " + MZH::toS(acd.averageConcavity()));
  MPxCommand::displayInfo("Max convexity: " + MZH::toS(acd.maxConcavity()));

  MColorArray convexityColors;
  MIntArray vertexIndices;

  // Color convexities
  int convexityCounter = 0;
  double convexityBlack = (acd.averageConcavity() + acd.maxConcavity()) / 2.0;
  for (double convexity : convexities) {
    float lightness = (float) MZH::clamp(1.0 - convexity / convexityBlack, 0.0, 1.0);
    convexityColors.append(lightness, lightness, lightness);
    vertexIndices.append(convexityCounter++);
  }
  *status = meshFn.setVertexColors(convexityColors, vertexIndices, &dgModifier);
  if (MZH::hasError(*status, "Error adding step to color convexities")) return;

  // Show knots
  std::vector<Vertex *> &knots = acd.knots();
  MPxCommand::displayInfo("Number of knots: " + MZH::toS(knots.size()));
  for (Vertex *vertex : knots) {
    *status = MZH::createLocator(dgModifier, vertex->point(), "knot#", false);
    if (MZH::hasError(*status, "Error creating knot locator")) return;
  }

  *status = dgModifier.doIt();
  if (MZH::hasError(*status, "Error running dag modifier")) return;
}
