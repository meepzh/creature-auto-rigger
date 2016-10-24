#include "Utils.h"

#include <maya/MFnDagNode.h>
#include <maya/MPxCommand.h>

namespace MZH {
  bool hasError(const MStatus &status, const char *message) {
    if (status != MS::kSuccess) {
      MPxCommand::displayError(MString(message) + ": " + status.errorString());
      return true;
    }
    return false;
  }

  MStatus setShadingGroup(MDGModifier &dgModifier, MObject transform, MString group) {
    MStatus status;

    // Get the mesh shape name
    MFnDagNode dagNodeFn(transform);
    dagNodeFn.setObject(dagNodeFn.child(0));

    // Generate the command
    MString cmd("sets -e -forceElement ");
    cmd += group + " " + dagNodeFn.name();
    
    // Execute the command
    status = dgModifier.commandToExecute(cmd);

    return status;
  }

  MFloatPoint toFP(const MPoint &point) {
    MFloatPoint fp;
    fp.setCast(point);
    return fp;
  }

  MString toS(const MPoint &point) {
    MString result = "(";
    result += point[0];
    result += ", ";
    result += point[1];
    result += ", ";
    result += point[2];
    result += ")";
    return result;
  }
}