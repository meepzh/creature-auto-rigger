#pragma once

#include <maya/MArgList.h>
#include <maya/MDagPath.h>
#include <maya/MDagPathArray.h>
#include <maya/MDGModifier.h>
#include <maya/MPxCommand.h>
#include <maya/MSyntax.h>

class ACDCmd : public MPxCommand {
public:
  MStatus doIt(const MArgList& args);
  static void *creator();
  static MSyntax newSyntax();

protected:
  void runACD(MDagPath dagPath, MStatus *status);
  MStatus parseArgs(const MArgList &args);

  MDGModifier dgModifier_;
  MDagPathArray outputDagPaths_;

  // Arguments
  double concavityThreshold_;
  double douglasPeuckerThreshold_;
  bool colorConcavities_;
  bool showKnots_;
  bool showProjectedPaths_;
};
