# Non Linear Microstructures

## Prerequisites
We currently only provide a setup to build the code on Windows, in particular using Visual Studio 2015, 2017 or 2019.
CUDA is required and should be installed prior to following the build instructions.

## Build
Unpack the files and run **premake-vs201X.bat** to generate a solution for Visual Studio 201X in a subdirectory **build**.
Open **build/Demo.sln** and build the solution. Run the project **Demo** to start the simulation.

## The simulation
*Note that the simulation requires preprocessing, so there will be a ~10 second delay before the simulation starts.*
The simulation should run at least at 60FPS on a wide range of CPUs and GPUs.
If your framerate is low there is likely to be an issue related to OpenMP, CUDA options, or compilation options.

In the simulation, the user can click and drag on the mesh to add position constraints for mesh vertices.
The camera can be moved using WASD (as in a first person game) and Q/E to move up and down.
Additionally, pressing F toggles a mode in which mouse movement is translated to rotating the camera.

## Simulation options
There is an expandable GUI in the top left corner containing various option groups.
We offer various options related to the camera, lighting and rendering. 
The FPS can be viewed when expanding the "Render options"  group.
The options controlling the simulation are grouped into setting that have an instant effect on the simulation and options that require the simulation to be restarted, meaning the precomputation has to be run again.

### Direct options

    Local/Global Iterations - Iterations used in the Projective Dynamics algorithm to solve for the elastic forces
    Jacobi Iterations - Number of Jacobi iterations to solve for incompressibility
    Material Stiffness - Changes the stiffness of the elastic deformable
    Solid:Fluid density - Controls the density of the elastic deformable relative to that of the fluid (0.5 means a 1:1 ratio)
    Fluid calmness - Controlls the ratio between PIC and FLIP
    Skip fluid step - Deactivates all fluid related computations (then handles advection of the elastic deformable in the Projective Dynamics framework).

### Options that require restart

    Reduction - Slider that interpolates between a very harsh reduction (0) and a good approximation (1) for the elasticity computations (controls subspace dimension, and force approximation quality)
    Fluid Resolution - Interpolates between coarse (0) and fine (1) resolutions for the grid used for incompressibility and advection
    Grid Size - Controls the absolute size of the simulation grid relative to the size of the mesh
    Add Pool - If set to true, adds a pool of water, the size of the simulation grid
    Pool height - Height of the pool relative to the height of the mesh
    Pool gap - Extends the simulation grid so that the pool initially doesn't fill the full X/Z-plane
    Fluid stream - Adds a stream of fluid that falls from the top with variable size
    Prevent intersections - Toggles the solid-solid particle intersection prevention step