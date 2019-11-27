The program for a given polygonal line L in the plane (`obstacle') and points A,B outside L 
constructs another polygonal line joining A to B and disjoint with L. 
The program can easily be modified to the case when the obstacle is a linear tree in the plane. 
The program does not use any complex algorithms. It builds a polygonal line that has O(n) vertexes in O(n^2) time,
where n is the number of vertexes in given polygonal line.

This program was prepared in frame of the course `Discrete structures and lgorithms in topology' 
in Moscow Institute of Physics and Technology. Website of the course: https://www.mccme.ru/circles/oim/home/combtop13.htm. 
The program illustrates the fact that a tree does not separate the plane (or a surface), 
which is the main ingredient for the Euler inequality. 

https://github.com/mmilunovic/Shortest-path-among-polygons this program builds a shortest path between two point,
avoiding obstacles that are a set of disjoined simple polygons. It solves more difficult problem in nearly equal time,
but it uses more complicated algorithms based on visibility graphs.
