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
  
  std::shared_ptr<std::vector<std::unique_ptr<Face>>> faces = acd.quickHull().faces();
  std::vector<bool> vertexSeen;
  vertexSeen.resize(vertexIt.count(), false);

  for (std::unique_ptr<Face> &face : *faces) {
    std::vector<Vertex *> faceVertices = face->vertices();
    for (Vertex *vertex : faceVertices) {
      if (!vertexSeen[vertex->index()]) {
        vertexSeen[vertex->index()] = true;
        *status = MZH::createLocator(dgModifier, vertex->point(), "hullVertex#", false);
        if (MZH::hasError(*status, "Error creating hull vertex locator")) return;
      }
    }
  }

  *status = dgModifier.doIt();
  if (MZH::hasError(*status, "Error running dag modifier")) return;
}
