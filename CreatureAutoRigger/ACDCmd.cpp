#include "ACDCmd.h"

#include <maya/MGlobal.h>
#include <maya/MItMeshVertex.h>
#include <maya/MItSelectionList.h>

#include "ACD.h"
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

  ACD acd(vertexIt, status);

  /*
  std::vector<Vertex *> &hullVertices = acd.hullVertices();
  for (Vertex *vertex : hullVertices) {
    *status = MZH::createLocator(dgModifier, vertex->point(), "hullVertex#", false);
    if (MZH::hasError(*status, "Error creating hull vertex locator")) return;
  }
  */

  pEdgeMap &projectedEdges = acd.projectedEdges();
  for (auto it1 = projectedEdges.begin(); it1 != projectedEdges.end(); ++it1) {
    std::unordered_map<Vertex *, std::shared_ptr<std::vector<Vertex *>>> &edgeMap = it1->second;
    for (auto it2 = it1->second.begin(); it2 != it1->second.end(); ++it2) {
      std::shared_ptr<std::vector<Vertex *>> &path = it2->second;
      for (size_t i = 0; i < path->size() - 1; ++i) {
        MZH::createLocator(dgModifier, (*path)[i]->point(), "projectedVertex#", false);
        MZH::createLocator(dgModifier, (*path)[i]->point() * 0.34 + (*path)[i + 1]->point() * 0.66, "projectedVertex#", false);
        MZH::createLocator(dgModifier, (*path)[i]->point() * 0.66 + (*path)[i + 1]->point() * 0.34, "projectedVertex#", false);
        MZH::createLocator(dgModifier, (*path)[i + 1]->point(), "projectedVertex#", false);
      }
    }
  }

  *status = dgModifier.doIt();
  if (MZH::hasError(*status, "Error running dag modifier")) return;
}
