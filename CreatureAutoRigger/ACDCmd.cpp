#include "ACDCmd.h"

#include <maya/MArgDatabase.h>
#include <maya/MFnMesh.h>
#include <maya/MFnNurbsCurve.h>
#include <maya/MGlobal.h>
#include <maya/MItMeshVertex.h>
#include <maya/MItSelectionList.h>

#include "ACD.h"
#include "MathUtils.h"
#include "QuickHull.h"
#include "Utils.h"

#define kColorConcavitiesFlagLong "-colorConcavities"
#define kColorConcavitiesFlag "-cc"
#define kConcavityToleranceFlagLong "-concavity"
#define kConcavityToleranceFlag "-c"
#define kDouglasPeuckerThresholdFlagLong "-knot"
#define kDouglasPeuckerThresholdFlag "-k"
#define kShowKnotsFlagLong "-showKnots"
#define kShowKnotsFlag "-sk"
#define kShowProjectedEdgesFlagLong "-showProjectedEdges"
#define kShowProjectedEdgesFlag "-sp"

MStatus ACDCmd::doIt(const MArgList& args) {
  MStatus status = MS::kSuccess;

  concavityTolerance_ = 0.04;
  douglasPeuckerThreshold_ = 0.0008;

  status = parseArgs(args);
  if (MZH::hasError(status, "Error parsing arguments")) return status;

  MSelectionList sList;
  MGlobal::getActiveSelectionList(sList);

  MItSelectionList sListIt(sList);
  bool selectedMesh = false;
  
  for (; !sListIt.isDone(); sListIt.next()) {
    MStatus itemStatus;
    MDagPath dagPath;
    sListIt.getDagPath(dagPath);
    
    if (dagPath.node().hasFn(MFn::kMesh)) {
      selectedMesh = true;
      runACD(dagPath, &itemStatus);
    } else if (dagPath.node().hasFn(MFn::kTransform) && dagPath.hasFn(MFn::kMesh)) {
      selectedMesh = true;
      runACD(dagPath, &itemStatus);
    }

    // Store errors to note after list is complete
    if (itemStatus != MS::kSuccess) status = itemStatus;
  }

  if (sListIt.isDone() && !selectedMesh) {
    displayError("No shape or transform selected");
    return MS::kFailure;
  }

  if (status == MS::kSuccess) setResult("ACDCmd executed successfully!\n");
  else displayError("ACDCmd executed with errors. See Script Editor for details.\n");
  return status;
}

void *ACDCmd::creator() {
  return new ACDCmd;
}

MSyntax ACDCmd::newSyntax() {
  MSyntax syntax;
  syntax.addFlag(kColorConcavitiesFlag, kColorConcavitiesFlagLong, MSyntax::kNoArg);
  syntax.addFlag(kConcavityToleranceFlag, kConcavityToleranceFlagLong, MSyntax::kDouble);
  syntax.addFlag(kDouglasPeuckerThresholdFlag, kDouglasPeuckerThresholdFlagLong, MSyntax::kDouble);
  syntax.addFlag(kShowKnotsFlag, kShowKnotsFlagLong, MSyntax::kNoArg);
  syntax.addFlag(kShowProjectedEdgesFlag, kShowProjectedEdgesFlagLong, MSyntax::kNoArg);
  return syntax;
}

void ACDCmd::runACD(MDagPath dagPath, MStatus *status) {
  MItMeshVertex vertexIt(dagPath, MObject::kNullObj, status);
  if (MZH::hasError(*status, "Failed to get vertex iterator")) return;
  if (vertexIt.isDone()) {
    displayWarning("No vertices in object");
    return;
  }

  ACD acd(vertexIt, concavityTolerance_, douglasPeuckerThreshold_, status);
  MZH::hasError(*status, "ACD algorithm failed");

  MStatus displayOptionsStatus;

  // Draw projected hull edges
  if (showProjectedPaths_) {
    displayOptionsStatus = MS::kSuccess;

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
      
        MObject curve = nurbsFn.create(controlVerts, knots, (unsigned int) 1, MFnNurbsCurve::kClosed, false, false, MObject::kNullObj, &displayOptionsStatus);
        displayOptionsStatus = dgModifier_.renameNode(curve, "dijkstraPath#");
        MZH::hasWarning(displayOptionsStatus, "Could not create dijkstra path curve for edge from vertex " +
          MZH::toS(path->front()->index()) + " to vertex " + MZH::toS(path->back()->index()));
      }
    }
  }
  
  // Concavities
  MPxCommand::displayInfo("Average concavity: " + MZH::toS(acd.averageConcavity()));
  MPxCommand::displayInfo("Max concavity: " + MZH::toS(acd.maxConcavity()));

  if (colorConcavities_) {
    displayOptionsStatus = MS::kSuccess;

    MFnMesh meshFn(dagPath, &displayOptionsStatus);
    if (!MZH::hasWarning(displayOptionsStatus, "Could not convert selection to mesh to visualize concavities")) {
      std::vector<double> &concavities = acd.concavities();
      MColorArray concavityColors;
      MIntArray vertexIndices;

      // Color concavities
      int concavityCounter = 0;
      double concavityBlack = (acd.averageConcavity() + acd.maxConcavity()) / 2.0;
      for (double convexity : concavities) {
        float lightness = (float) MZH::clamp(1.0 - convexity / concavityBlack, 0.0, 1.0);
        concavityColors.append(lightness, lightness, lightness);
        vertexIndices.append(concavityCounter++);
      }
      displayOptionsStatus = meshFn.setVertexColors(concavityColors, vertexIndices, &dgModifier_);
      MZH::hasWarning(displayOptionsStatus, "Could not color concavities");
    }
  }

  // Show knots
  MPxCommand::displayInfo("Number of knots: " + MZH::toS((int) acd.knots().size()));
  if (showKnots_) {
    displayOptionsStatus = MS::kSuccess;

    std::unordered_set<Vertex *> &knots = acd.knots();
    for (Vertex *vertex : knots) {
      displayOptionsStatus = MZH::createLocator(dgModifier_, vertex->point(), "knot#", false);
      MZH::hasWarning(displayOptionsStatus, "Could not create knot locator");
    }
  }

  displayOptionsStatus = dgModifier_.doIt();
  MZH::hasWarning(displayOptionsStatus, "Error creating dijkstra paths and/or knot locators");
}

MStatus ACDCmd::parseArgs(const MArgList &args) {
  MStatus status = MS::kSuccess;

  MArgDatabase argData(syntax(), args);

  if (argData.isFlagSet(kConcavityToleranceFlag, &status)) {
    status = argData.getFlagArgument(kConcavityToleranceFlag, 0, concavityTolerance_);
  }
  if (argData.isFlagSet(kDouglasPeuckerThresholdFlag, &status)) {
    status = argData.getFlagArgument(kDouglasPeuckerThresholdFlag, 0, douglasPeuckerThreshold_);
  }

  colorConcavities_ = argData.isFlagSet(kColorConcavitiesFlag, &status);
  showKnots_ = argData.isFlagSet(kShowKnotsFlag, &status);
  showProjectedPaths_ = argData.isFlagSet(kShowProjectedEdgesFlag, &status);

  return status;
}
