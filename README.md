# glitter
Exploring the optics of glitter

# Links
- [Daily log](https://docs.google.com/document/d/1gc7nJyvbHaAru2s7WA66u_z2e6hI94dILIBkRiRvMyo/edit#)
- [Project notes](https://docs.google.com/document/d/1eWWUWsdsPODOIP9aJ0DG5k2nGUHaMJvG3zi6A7wgaEk/edit)
- [SparkleGeometry paper](https://www.cv-foundation.org/openaccess/content_cvpr_2016_workshops/w16/papers/Stylianou_SparkleGeometry_Glitter_Imaging_CVPR_2016_paper.pdf) (Pless, Stylianou)
- [Weekly blog](https://blogs.gwu.edu/pless/)

# Files/structure
## Glitter characterization
### Main pipeline
With a set of images from both a vertical and horizontal sweep, 
these scripts are run in this order:
- detectSpecs (finds and saves spec centroids)
- getGaussians (fits gaussians to brightness distributions, saving the means)
- characterizeCoords (uses means and measurements to compute surface normals of specs)
### Helper/utilities
- drawRig: shows a 3d model of the glitter rig with light vectors
- createMeasurementsStruct: saves a single matlab struct with all the glitter rig measurements and useful constants
### Directories
- imgs: contains all types of images from along the way of this project with subdirectories by day
- data: contains data outputs of scripts and other relevant data (spec centroid, brightness gaussian means, measurements, etc)
- util: other matlab code for random related tasks not part of the main pipeline (aperture comparisons, light bar images, etc)
- old: old code from throughout the project that might be useful for reference
### Glitter images
Actual glitter images used by this code are not stored in this repo