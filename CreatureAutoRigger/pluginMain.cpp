#include <maya/MFnPlugin.h>

#include "ConvexHullCmd.h"

MStatus initializePlugin(MObject obj) { 
  MStatus status;
  MFnPlugin plugin(obj, "Robert Zhou", "1.0", "Any");

  status = plugin.registerCommand("convexHullCmd", ConvexHullCmd::creator);

  if (!status) {
    status.perror("registerNode");
    return status;
  }
  return status;
}

MStatus uninitializePlugin(MObject obj) {
  MStatus status;
  MFnPlugin plugin(obj);

  status = plugin.deregisterCommand("convexHullCmd");

  if (!status) {
    status.perror("deregisterNode");
    return status;
  }
  return status;
}
