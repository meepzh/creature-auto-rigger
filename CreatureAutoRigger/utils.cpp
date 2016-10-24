#include "Utils.h"

#include <maya/MFnDagNode.h>
#include <maya/MPxCommand.h>

namespace MZH {
  MStatus createLocator(MDGModifier &dgModifier, const MPoint &position, const MString &name, bool relative) {
    MStatus status;

    // Generate the command
    MString cmd("spaceLocator ");
    
    // Absolute
    if (!relative) cmd += "-a ";
    
    // Position
    cmd += "-p ";
    cmd += position[0];
    cmd += " ";
    cmd += position[1];
    cmd += " ";
    cmd += position[2];
    cmd += " ";

    // Name
    cmd += "-n " + name + " ";

    // Relative
    if (relative) cmd += "-r ";
    
    // Execute the command
    status = dgModifier.commandToExecute(cmd);

    return status;
  }

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

  MString toS(double value) {
    MString result;
    result += value;
    return result;
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