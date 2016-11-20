#pragma once

#include <maya/MArgList.h>
#include <maya/MDagPath.h>
#include <maya/MPxCommand.h>

class ACDCmd : public MPxCommand {
public:
  MStatus doIt(const MArgList& args);
  static void *creator();

protected:
  void runACD(MDagPath dagPath, MStatus *status);
};
