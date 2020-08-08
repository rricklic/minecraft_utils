#!/usr/bin/python3

###############################################################################
# STL binary format
#   Note: little-endian 
#
# Header: UINT8[80]
# Number of triangles: UINT32
# foreach triangle
#     Normal vector: REAL32[3]
#     Vertex 1: REAL32[3] 
#     Vertex 2: REAL32[3]
#     Vertex 3: REAL32[3]
#     Attribute byte count: UINT16
# end
#
#
# STL ascii format
#
# "solid ..."
#   "facet normal <i> <j> <k>"
#     "outer loop"
#       "vertex <x> <y> <z>
#       "vertex <x> <y> <z>
#       "vertex <x> <y> <z>
#     "endloop
#   "endfacet
# "endsolid
#
###############################################################################

import argparse
import subprocess
import struct

HEADER_SIZE = 80

###############################################################################
class Vector:
    def __init__(self, p):
        self.points = p
        self.num_dimensions = len(p)

    def copy(self):
        return Vector(self.points.copy())

    # Assumes self.num_dimensions == v.num_dimensions
    def add(self, v):
        new_v = self.copy()
        for i in range(0, self.num_dimensions):
            new_v.points[i] += v.points[i]
        return new_v

    # Assumes self.num_dimensions == v.num_dimensions
    def sub(self, v):
        new_v = self.copy()
        for i in range(0, self.num_dimensions):
            new_v.points[i] -= v.points[i]
        return new_v

    def scalar_mult(self, scalar):
        new_v = self.copy()
        for i in range(0, self.num_dimensions):
            new_v.points[i] *= scalar
        return new_v

    # Assumes self.num_dimensions == v.num_dimensions
    def equals(self, v):
        for i in range(0, self.num_dimensions):
            if self.points[i] != v.points[i]:
                return False
        return True       

    def round(self):
        new_v = self.copy()
        for i in range(0, self.num_dimensions):
            new_v.points[i] = round(new_v.points[i])
        return new_v

    def abs(self):
        new_v = self.copy()
        for i in range(0, self.num_dimensions):
            new_v.points[i] = abs(new_v.points[i])
        return new_v

    # Assumes self.num_dimensions == v.num_dimensions
    def min(self, v):
        min_points = []
        for i in range(0, self.num_dimensions):
            if self.points[i] == None:
                min_points.append(v.points[i])
            elif v.points[i] == None:
                min_points.append(self.points[i])
            else:
                min_points.append(min(self.points[i], v.points[i]))
        return Vector(min_points)

    # Assumes self.num_dimensions == v.num_dimensions
    def max(self, v):
        max_points = []
        for i in range(0, self.num_dimensions):
            if self.points[i] == None:
                max_points.append(v.points[i])
            elif v.points[i] == None:
                max_points.append(self.points[i])
            else:
                max_points.append(max(self.points[i], v.points[i]))
        return Vector(max_points)

    def to_string(self):
        return self.points

ZERO_VECTOR_3D = Vector([0, 0, 0])

################################################################################
class Triangle:
    def __init__(self, v1, v2, v3):
        self.v1 = v1
        self.v2 = v2
        self.v3 = v3

################################################################################
def peek(fh, length):
    position = fh.tell()
    data = fh.read(length)
    fh.seek(position)
    return data

###############################################################################
def compute_line_blocks(v1, v2, blocks):
    dv = v2.sub(v1).abs()
    if dv.equals(ZERO_VECTOR_3D):
        blocks[(v1.points[0], v1.points[1], v1.points[2])] = 1
    else:
        # Note: float casting is not necessary in python3, but in case one
        #       changes to python2 the cast explicit
        max_delta = max(dv.points)
        x_step = float(dv.points[0]) / float(max_delta) * (1 if v2.points[0] - v1.points[0] > 0 else -1)
        y_step = float(dv.points[1]) / float(max_delta) * (1 if v2.points[1] - v1.points[1] > 0 else -1)
        z_step = float(dv.points[2]) / float(max_delta) * (1 if v2.points[2] - v1.points[2] > 0 else -1)
        for step in range(0, max_delta):
            blocks[(round(v1.points[0] + step * x_step), round(v1.points[1] + step * y_step), round(v1.points[2] + step * z_step))] = 1

###############################################################################
def compute_triangle_blocks(v1, v2, v3, blocks):
    dv = v2.sub(v1).abs()
    if dv.equals(ZERO_VECTOR_3D):
        blocks[(v1.points[0], v1.points[1], v1.points[2])] = 1
    else:
        # Note: float casting is not necessary in python3, but in case one
        #       changes to python2 the cast explicit
        max_delta = max(dv.points)
        x_step = float(dv.points[0]) / float(max_delta) * (1 if v2.points[0] - v1.points[0] > 0 else -1)
        y_step = float(dv.points[1]) / float(max_delta) * (1 if v2.points[1] - v1.points[1] > 0 else -1)
        z_step = float(dv.points[2]) / float(max_delta) * (1 if v2.points[2] - v1.points[2] > 0 else -1)
        for step in range(0, max_delta):
            x = round(v1.points[0] + step * x_step)
            y = round(v1.points[1] + step * y_step)
            z = round(v1.points[2] + step * z_step)
            compute_line_blocks(Vector([x, y, z]), v3, blocks)

###############################################################################
def compute_blocks(triangle, blocks):
    compute_triangle_blocks(triangle.v1, triangle.v2, triangle.v3, blocks)
    compute_triangle_blocks(triangle.v2, triangle.v3, triangle.v1, blocks)
    compute_triangle_blocks(triangle.v3, triangle.v1, triangle.v2, blocks)

###############################################################################
def transform(vertex, origin_vector, unit_vectors):
    return unit_vectors[0].scalar_mult(vertex[0]).add(unit_vectors[1].scalar_mult(vertex[1])).add(unit_vectors[2].scalar_mult(vertex[2])).add(origin_vector).round()

###############################################################################
def parse_ascii(fh, origin_vector, unit_vectors):
    triangles = []

    header = fh.readline()
    print("Header", header)

    for count, normal_vector in enumerate(fh): # facet normal n.i n.j n.k
        if normal_vector.decode("utf-8").startswith("endsolid"):
            break
        fh.readline() # outer loop
        vertex1 = fh.readline().decode("utf-8").split()[1:4] # vertex v1.x v1.y v1.z
        vertex2 = fh.readline().decode("utf-8").split()[1:4] # vertex v2.x v2.y v2.z
        vertex3 = fh.readline().decode("utf-8").split()[1:4] # vertex v3.x v3.y v3.z
        fh.readline() # endloop
        fh.readline() # endfacet
        print("Triangle", count, normal_vector, vertex1, vertex2, vertex3)
        for index, data in enumerate(vertex1):
            vertex1[index] = float(data)
        for index, data in enumerate(vertex2):
            vertex2[index] = float(data)
        for index, data in enumerate(vertex3):
            vertex3[index] = float(data)
        v1 = transform(vertex1, origin_vector, unit_vectors)
        v2 = transform(vertex2, origin_vector, unit_vectors)
        v3 = transform(vertex3, origin_vector, unit_vectors)
        triangles.append(Triangle(v1, v2, v3))

    return triangles

###############################################################################
def parse_binary(fh, origin_vector, unit_vectors):
    triangles = []

    header = fh.read(HEADER_SIZE)
    print("Header", header)
    num_triangles = struct.unpack("<I", fh.read(4))[0]
    print("Num Triangles ", num_triangles)

    for count in range(1, num_triangles):
        normal_vector = struct.unpack("<fff", fh.read(12))
        vertex1 = struct.unpack("<fff", fh.read(12))
        vertex2 = struct.unpack("<fff", fh.read(12))
        vertex3 = struct.unpack("<fff", fh.read(12))
        attribute = struct.unpack("<H", fh.read(2))[0]
        print("Triangle", count, normal_vector, vertex1, vertex2, vertex3, attribute)
        v1 = transform(vertex1, origin_vector, unit_vectors)
        v2 = transform(vertex2, origin_vector, unit_vectors)
        v3 = transform(vertex3, origin_vector, unit_vectors)
        triangles.append(Triangle(v1, v2, v3))

    return triangles

###############################################################################
def main(args):
    print("Start...")
    origin_vector = Vector([float(x) for x in args.origin.split(",")])
    unit_vectors = [Vector([float(x) for x in args.x_unit.split(",")]), Vector([float(x) for x in args.y_unit.split(",")]), Vector([float(x) for x in args.z_unit.split(",")])]

    # Parse STL file
    print("STL Filename", args.file)
    with open(args.file, "rb") as fh:
        header_prefix = peek(fh, 5).decode("utf-8")
        if header_prefix == "solid":
            try:
                triangles = parse_ascii(fh, origin_vector, unit_vectors)
            except UnicodeDecodeError:
                #Note: if file is binary with a header starting with "solid" the parsing will fail with UnicodeDecodeError
                fh.seek(0)
                triangles = parse_binary(fh, origin_vector, unit_vectors)
        else:
            triangles = parse_binary(fh, origin_vector, unit_vectors)

    # Compute unique Minecraft block coordinates 
    blocks = {}
    min_vector = Vector([None, None, None])
    max_vector = Vector([None, None, None])
    for count, triangle in enumerate(triangles):
        if count % 1000 == 0:
            print("Processed", count, "triangles...")
        min_vector = min_vector.min(triangle.v1).min(triangle.v2).min(triangle.v3)
        max_vector = max_vector.max(triangle.v1).max(triangle.v2).max(triangle.v3)
        compute_blocks(triangle, blocks)

    # Render blocks into Minecraft server console 
    for count, block in enumerate(blocks):
        if count % 1000 == 0:
            print("Submitted ", count, "blocks...")
        command = ["screen", "-S", args.screen, "-p", args.page, "-X", "stuff", "setblock " + str(block[0]) + " " + str(block[1]) + " " + str(block[2]) + " " + args.block_type + "\\n"]
        if args.verbose:
            print(command)
        if args.commit:
            subprocess.run(command)
   
    print("# triangles " + str(len(triangles)))
    print("# unique blocks " + str(len(blocks.keys())))
    print("min/max points ", min_vector.to_string(), max_vector.to_string())
    print("Done")

###############################################################################
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Parse 3d printer file in ascii/binary STL format, and render into Minecraft server console running within GNU screen')
    parser.add_argument('-f', '--file', required=True, type=str, metavar='FILENAME', help='filename of STL file')
    parser.add_argument('-o', '--origin', type=str, metavar='3D_COORDINATE', default="0,0,0", help='origin (x,y,z default=0,0,0)')
    parser.add_argument('-x', '--x_unit', type=str, metavar='3D_COORDINATE', default="1,0,0", help='x unit vector (x,y,z default=1,0,0)')
    parser.add_argument('-y', '--y_unit', type=str, metavar='3D_COORDINATE', default="0,0,1", help='y unit vector (x,y,z default=0,0,1)')
    parser.add_argument('-z', '--z_unit', type=str, metavar='3D_COORDINATE', default="0,1,0", help='z unit vector (x,y,z default=0,1,0)')
    parser.add_argument('-b', '--block_type', required=True, type=str, metavar='MINECRAFT_BLOCK_ID', help='Minecraft block id to render (eg: redstone_block)')
    parser.add_argument('-s', '--screen', type=str, metavar='SCREEN_NAME', default="minecraft", help='Name of screen where Minecraft server is running (default=minecraft)')
    parser.add_argument('-p', '--page', type=str, metavar='SCREEN_PAGE', default="0", help='Screen page number where Minecraft server is running (default=0)')
    parser.add_argument('-v', '--verbose', action="store_true", help='Turn on logging')
    parser.add_argument('-c', '--commit', action="store_true", help='Commit drawing into Minecraft')
    main(parser.parse_args())
