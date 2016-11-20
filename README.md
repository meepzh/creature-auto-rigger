# Creature Auto-Rigger

This is a work-in-progress as part of my senior design project. You can read the project blog [here](http://seniorblog.meepzh.com/).

## Usage

Compile the project if you don't have a .mll binary already. The project will automatically copy the file to the default maya folder.

Load the plug-in, then use the following commands:

### Convex Hull - Finished
- `convexHullCmd` - Create a convex hull mesh for each selected mesh object
- `convexHullCmd -i #` - Create a convex hull mesh for each selected mesh object, limiting the algorithm to # iterations. You may also use the flag `-iterations`.

### Approximate Convex Decomposition (ACD) - WIP
- `acdCmd` - Run the ACD command on each selected mesh object
