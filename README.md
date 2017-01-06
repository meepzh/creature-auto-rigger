# Creature Auto-Rigger

This is a work-in-progress as part of my senior design project. You can read the project blog [here](http://seniorblog.meepzh.com/).

## Usage

Compile the project if you don't have a .mll binary already. The project will automatically copy the file to the default Maya 2016 folder.

Load the plug-in, then use the following commands:

### Convex Hull - Finished

`convexHullCmd` - Create a convex hull mesh for each selected mesh object

#### Flags
- `-iterations #` or `-i #` - Limit the algorithm to # iterations

This implementation is derived from [John Lloyd's robust Java QuickHull3D implementation](http://www.cs.ubc.ca/~lloyd/java/quickhull3d.html).

### Approximate Convex Decomposition (ACD) - WIP

`acdCmd` - Run the ACD algorithm on each selected mesh object

#### Flags
- `-colorConcavities style` or `-cc style` - Set the vertex color to the concavity measure. There are two styles: `color` and `grayscale`.
	- The `color` style is replicated from one of Maya's skin weight gradient presets, described below:
	    - Minimum concavity (0%): white
		- >0% to 25%: red to orange
		- 25% to 50%: orange to yellow
		- 50% to 75%: yellow to green
		- 75% to <100%: green to blue
		- Maximum concavity (100%): black
    - The `grayscale` style is a gradient from white (minimum concavity) to black (maximum concavity). `gray`, `grey`, and `greyscale` are accepted alternatives.
    - To show these colors, under the Modeling menu set, click Mesh Display >> Toggle Display Colors Attribute
- `-concavity #.#` or `-c #.#` - Set the concavity tolerance for separating components. Must be nonnegative. Default is 0.04.
- `-knot #.#` or `-k #.#` - Set the Douglas-Peuker threshold for finding knots. Must be nonnegative and less than or equal to the concavity tolerance. Default is 0.0008.
- `-showKnots` or `-sk` - Show the knots as locators
- `-showProjectedEdges` or `-sp` - Show the convex hull edge projections on the model as linear curves.

This implementation is an interpretation of Jyh-Ming Lien and Nancy Amato's 2007 Approximate Convex Decomposition of Polyhedra paper. Note that this implementation does not have its own feature grouping (instead arbitrarily handled by QuickHull3D), and it does not handle non-zero genus models.

## Future Work
- Convert the above commands to nodes
- Add a skeletonization algorithm that uses ACD results
