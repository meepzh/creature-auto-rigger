# Creature Auto-Rigger

This is a work-in-progress as part of my senior design project. You can read the project blog [here](http://seniorblog.meepzh.com/).

## Usage

Compile the project if you don't have a .mll binary already. The project will automatically copy the file to the default maya folder.

Load the plug-in, then use the following commands:

### Convex Hull - Finished
- `convexHullCmd` - Create a convex hull mesh for each selected mesh object
- `-iterations #` or `-i #` - Limit the algorithm to # iterations

### Approximate Convex Decomposition (ACD) - WIP
- `acdCmd` - Run the ACD command on each selected mesh object
- `-colorConcavities` or `-cc` - Set the vertex color to the concavity measure. Black represents halfway between the average and the maximum concavity.
- `-concavity #.#` or `-c #.#` - Set the concavity threshold for separating components. Default is 0.04.
- `-knot #.#` or `-k #.#` - Set the Douglas-Peuker threshold for finding knots. Default is 0.0008;
- `-showKnots` or `-sk` - Show the knots as locators
- `-showProjectedEdges` or `-sp` - Show the convex hull edge projections on the model as linear curves.
