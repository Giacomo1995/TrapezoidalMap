# TrapezoidalMap
Implementation of the Trapezoidal Map to efficiently perform the point location task.

## High-Level Architecture
The project contains five folders: *algorithms*, *data_structures*, *drawables*, *managers*, *utils*.<br/>
The *algorithms* folder only contains utility functions such as *intersection* (which computes the intersection between a vertical and a non-vertical segment), 
*lies_above* (which returns a boolean to understand whether a given point is above or below a certain segment) and *slope* (which computes the slope of a segment).

The *data_structures* folder contains: *Trapezoid*, *TrapezoidalMap*, *DAG*.<br/>
*Trapezoid* has the following attributes: top, bottom, leftp, rightp, an array of four adjacencies and the index to the corresponding leaf in the DAG.
These attributes are all indices, so it is actually an indexed trapezoid (it is named *Trapezoid* for simplicity).
*TrapezoidalMap* contains a vector of points, a vector of indexed segments, a vector of trapezoids (indexed trapezoids), the associated DAG, the Bounding Box and some structures to look for double points and double segments efficiently 
(the same ones of *TrapezoidalMapDataset*).
This class also provides the methods to build and perform the point location task.

*DAG* contains the index of the root, a vector of nodes, and the references to the points and indexed segments of the Trapezoidal Map.
The node is defined in this class as *struct*. In particular, it includes the type of node (internal or leaf) and the value.
The value is defined as *union* due to the fact it can be either a leaf (only the index for the corresponding trapezoid) or an internal node, which can be x-node or y-node.
Of course, the internal node has an index for the corresponding point/segment, the left child, and the right child (both indices to the vector of nodes).
This class also provides the methods to perform the query and update the DAG.

The *drawables* folder includes *DrawableTrapezoidalMap* which allows drawing trapezoids (through GL_POLYGON) with red highlighted vertical extensions.
It also contains information about each trapezoid color.

