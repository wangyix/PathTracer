Bidirectional path tracer implementation based on:
"Robust Monte Carlo Methods for Light Transport Simulation".  Eric Veach, 1997

Output images and their respective scene files can be found in the output_images
directory.  A video of quaternion julia sets made from rendered images can be
viewed here:
http://vimeo.com/115604738


The power heuristic with beta=2 was used for computing the weights of the path
contributions.  Importance sampling was used for selecting reflected/refracted
ray directions during subpath construction.  Three types of BSDFs were implemented:
Lambertian, Fresnel conductor, and Fresnel dielectric (the implmementation of the
latter two were based on those found in PBRT).

Also, quaternion julia sets were added as a shape that can be rendered.  The method
we used for finding ray intersections with Julia sets is the one described in the
1989 paper “Ray Tracing Deterministic 3-D Fractals” by Hart, et. al, with some extra
modifications to handle reflected and refracted rays.

To reduce fireflies, certain types of paths had their contributions put in a
separate image instead of the main output image (paths of type s=0, t>2,
and w_st=1).  This seperate image has suffix "_bright" appended to the filename.




Args:
Raytracer.exe (scenefile.txt)?  (x y i j)?

If scene file is not specified, CornellBox.txt is used as the scene file by default.
(x y i j) are 4 integer args that allow the pathtracer to only render a portion of
the image: x,y indicate how many columns and rows to split the image into, and i,j
are the column and row indices that identify which cell to render.  This is to allow
an image to be rendered by multiple independent Raytracer.exe processes, which could
come in handy if you want to render an image using multiple machines.  As an example,
if you want to render using 4 separate processes, you can do:

Raytracer.exe scenefile.txt 2 2 0 0
Raytracer.exe scenefile.txt 2 2 0 1
Raytracer.exe scenefile.txt 2 2 1 0
Raytracer.exe scenefile.txt 2 2 1 1

...which splits the image into a 2x2 grid, with the first process rendering the
bottom-left cell, the second process rendering the bottom-right cell, the third
process rendering the top-left cell, and the fourth process rendering the top-right
cell.  Each process will output a PNG image.  The output images then need to be added
together to form the final image.  This was done using PngAdder.exe, which was a
small program written using the lodepng library.


Raytracer.exe can render the image (or subimage) in multiple threads, by adding
the command "Threads k" in the scene file where k is the desired number of threads.

OBJ files referenced by the scene file need to be in the same directory as
Raytracer.exe.



The starting code was a raytracer provided by the Stanford CS148 staff written by
Ben Mildenhall.  Currently, most of the that code is unused or has been completely
changed. Parts of the original code that are still being used include the libst library,
the shape classes for intersection testing, and the AABBTree class for use in 
TriangleMesh intersection testing.  The code in this project is not very organized;
many unused classes are still included.
