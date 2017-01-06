#include <maya/MFnPlugin.h>

#include "ACDCmd.h"
#include "ConvexHullCmd.h"

MStatus initializePlugin(MObject obj) { 
  MStatus status;
  MFnPlugin plugin(obj, "Robert Zhou", "1.0", "Any");

  status = plugin.registerCommand("acdCmd", ACDCmd::creator, ACDCmd::newSyntax);
  if (!status) {
    status.perror("registerNode");
    return status;
  }
  
  status = plugin.registerCommand("convexHullCmd", ConvexHullCmd::creator, ConvexHullCmd::newSyntax);
  if (!status) {
    status.perror("registerNode");
    return status;
  }

  return status;
}

MStatus uninitializePlugin(MObject obj) {
  MStatus status;
  MFnPlugin plugin(obj);

  status = plugin.deregisterCommand("acdCmd");
  if (!status) {
    status.perror("deregisterNode");
    return status;
  }

  status = plugin.deregisterCommand("convexHullCmd");
  if (!status) {
    status.perror("deregisterNode");
    return status;
  }

  return status;
}
