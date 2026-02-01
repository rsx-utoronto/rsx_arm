#usr/bin/python3

import numpy as np


# Recieving AR tag points
'''
 AR tag points
        [ [x1, y1, z1], 
        [x2, y2, z2], 
        [x3, y3, z3], 
        [x4, y4, z4]
        ]

Output:
    Key board key locations of requested key in global frame
    Example,
    Key A, global co-ordinates = [x, y, z]
    Key K, global co-ordinates = [x, y, z] 
    ...
    etc
 '''

# Finding Keyboard Key pt Coordinates in Global Frame
# main function

def find_key_pt_in_global_frame(key_pt, ar_tag_points):
    """
    Find the global frame coordinates of a keyboard key point given its local coordinates.

    Assuming all local co-ordinates of the keyboard are measured from the bottom left corner of the keyboard.
    Bottom left corner local co-ordinates = (0, 0),  z = 0 in local frame.

    Parameters:
        key_pt (np.ndarray): Local coordinates of the key point (x_local, y_local).
        ar_tag_points (np.ndarray): Array of AR tag points.
    Returns:
        key_pt_global (np.ndarray): Global coordinates of the key point.
    """
    x_local, y_local = key_pt

    # Step 1: Identify which AR tag points correspond to which corner of the keyboard

    # Step 1a: Rank lines by length
    long_edges, short_edges, diagonals = rank_lines_by_length(ar_tag_points)

    # Step 1b: Distinguish between Upper, Lower, Left, Right edges
    upper_edge, lower_edge, left_edge, right_edge = distinguish_edges(ar_tag_points, long_edges, short_edges)
    # Step 1c: Identify corner points

    bl_idx, br_idx, tr_idx, tl_idx  = identify_corner_points(ar_tag_points, left_edge, lower_edge, right_edge, upper_edge)
    bl_point_global = ar_tag_points[bl_idx]
    br_point_global = ar_tag_points[br_idx]
    tr_point_global = ar_tag_points[tr_idx]
    tl_point_global = ar_tag_points[tl_idx]

    # Step 2: Find unit vectors along bottom edge and left edge in global frame
    unit_vec_bottom_edge, unit_vec_left_edge = unit_vectors_bottom_and_left_edges_global_frame(bl_point_global, br_point_global, tl_point_global)

    # Step 3: Calculate global coordinates of the key point
    # (simple vector addition)
    # pk_global = p_bl_global + (x_local * unit_vec_bottom_edge) + (y_local * unit_vec_left_edge)
    key_pt_global = bl_point_global + (x_local * unit_vec_bottom_edge) + (y_local * unit_vec_left_edge)
    return key_pt_global


# the building blocks functions used in the main function above
# Step 1: Identify Points, which AR tags corresond to which corner

#Step 1a: Find distances between points to identify corners, rank the distances
# find long edges, short edges, diagonals

# Rank lines based on lengths
def rank_lines_by_length(ar_tag_points):
    
    from itertools import combinations

    # Lines = 4c2 from 4 points
    S = {0, 1, 2, 3}
    # All combinations of 2 points from 4 points
    comb = combinations(S, 2)
    # comb = [(0,1), (0,2), (0,3), (1,2), (1,3), (2,3)]   # all 4c2 combinations
    line_end_pts_idx = [(i, j) for i, j in comb]
    #lines = [(ar_tag_points[i], ar_tag_points[j]) for i, j in line_end_pts_idx]
    #line_end_pts_idx = [(0,1), (0,2), (0,3), (1,2), (1,3), (2,3)]   # all 4c2 combinations
    
    lengths = []
    for end_pts in line_end_pts_idx:
        p1_idx, p2_idx = end_pts
        p1 = ar_tag_points[p1_idx]
        p2 = ar_tag_points[p2_idx]
        length = np.linalg.norm(p2 - p1)
        lengths.append(length)
        # line_end_pts_idx = [(0,1), (0,2), (0,3), (1,2), (1,3), (2,3)]   # all 4c2 combinations
        # lenghts = [length1, length2, length3, length4, length5, length6]
        # One-to-one mapping between line_end_pts_idx and lengths, following same indexing
    
    ranked_lines = sorted(zip(line_end_pts_idx, lengths), key=lambda x: x[1], reverse=True)  # sorted on basis of lengths (x[1])

    # ranked_linse = [((i, j), length), ...]  # sorted in descending order of lengths
    #print("Ranked Lines by Length (Descending): ", ranked_lines)

    # first 2 longest lines
    diagonals = ranked_lines[:2]
    

    # Next 2 longest lines are long edges
    long_edges = ranked_lines[2:4]

    # Last 2 lines are short edges
    short_edges = ranked_lines[4:]

    # return indexes only
    diagonals_idx = [element[0] for element in diagonals]
    long_edges_idx = [element[0] for element in long_edges]
    short_edges_idx = [element[0] for element in short_edges]

    return  long_edges_idx, short_edges_idx, diagonals_idx


# Distinguish between Upper, Lower, Left, Right edges
def distinguish_edges(ar_tag_points, long_edges, short_edges):
    # long_edges = [(i, j), (k, l)]  # 2 long edges
    # short_edges = [(m, n), (o, p)]  # 2 short edges

    # For long edges, compare y-coordinates of midpoints (avg y value of entire line)
    long_edge_midpoints = []
    for edge in long_edges:
        p1_idx, p2_idx = edge
        p1 = ar_tag_points[p1_idx]
        p2 = ar_tag_points[p2_idx]
        midpoint = (p1 + p2) / 2  # coordinate values of midpoint
        long_edge_midpoints.append(midpoint)
        # long_edge_midpoints = [midpoint1, midpoint2]
                # where midpoint1 = (x1, y1, z1)
        # long_edges = [(i, j), (k, l)]  # 2 long edges
        # ono-to-one mapping between long_edges and long_edge_midpoints

    # Compare y-values [1] of midpoints to distinguish upper and lower edges
    if long_edge_midpoints[0][1] > long_edge_midpoints[1][1]:
        upper_edge = long_edges[0]  # upper edge will have y-value mid-pt > y-value of lower edge mid-point
        lower_edge = long_edges[1]
    else:
        upper_edge = long_edges[1]
        lower_edge = long_edges[0]

    # For short edges, compare x-coordinates of midpoints (avg x value of entire line)
    short_edge_midpoints = []
    for edge in short_edges:
        p1_idx, p2_idx = edge
        p1 = ar_tag_points[p1_idx]
        p2 = ar_tag_points[p2_idx]
        midpoint = (p1 + p2) / 2  # coordinate values of midpoint
        short_edge_midpoints.append(midpoint)
        # short_edge_midpoints = [midpoint1, midpoint2]
                # where midpoint1 = (x1, y1, z1)
        # short_edges = [(m, n), (o, p)]  # 2 short edges
        # one-to-one mapping between short_edges and short_edge_midpoints

    # Compare x-values [0] of midpoints to distinguish left and right edges
    if short_edge_midpoints[0][0] < short_edge_midpoints[1][0]:
        left_edge = short_edges[0]  # left edge will have x-value mid-pt < x-value of right edge mid-point
        right_edge = short_edges[1]
    else:
        left_edge = short_edges[1]
        right_edge = short_edges[0]

    return upper_edge, lower_edge, left_edge, right_edge

def identify_corner_points(ar_tag_points, left_edge, lower_edge, right_edge, upper_edge):
    """  
    Parameters:
        ar_tag_points (np.ndarray): Array of AR tag points.
        lower_edge (tuple): (i, j) Indexes of points forming the lower edge.
        left_edge (tuple): (i, j) Indexes of points forming the left edge.
        upper_edge (tuple): (i, j) Indexes of points forming the upper edge.
        right_edge (tuple): (i, j) Indexes of points forming the right edge.

    Returns:
        bottom_left_pt_idx (int): Index of the bottom left corner point in ar_tag_points.
        bottom_right_pt_idx (int): Index of the bottom right corner point in ar_tag_points.
        top_left_pt_idx (int): Index of the top left corner point in ar_tag_points.
        top_right_pt_idx (int): Index of the top right corner point in ar_tag_points.
    """

    left_edge_pts = set(left_edge)
    right_edge_pts = set(right_edge)
    upper_edge_pts = set(upper_edge)
    lower_edge_pts = set(lower_edge)

    # Bottom Left Point

    bottom_left_corner_idx = list(left_edge_pts.intersection(lower_edge_pts))[0]

    # Bottom Right Point
    bottom_right_corner_idx = list(right_edge_pts.intersection(lower_edge_pts))[0]


    # Top Right Point
    top_right_corner_idx = list(right_edge_pts.intersection(upper_edge_pts))[0]

    # Top Left Point
    top_left_corner_idx = list(left_edge_pts.intersection(upper_edge_pts))[0]



    return bottom_left_corner_idx, bottom_right_corner_idx,  top_right_corner_idx, top_left_corner_idx

# Finding plane (normal vector) for the keyboard
# using 3 points: bottom left, bottom right, top left
def find_keyboard_plane_normal(ar_tag_points, bl_idx, br_idx, tr_idx, tl_idx):
    """
    Find the normal vector of the keyboard plane using 3 points: bottom left, bottom right, top left.
    Returns:
        normal_unit_vec: normal vector of the keyboard plane.

    We need this to construct the plane equation.

    Parameters:
        ar_tag_points (np.ndarray): Array of AR tag points.
        bl_idx (int): Index of the bottom left corner point in ar_tag_points.
        br_idx (int): Index of the bottom right corner point in ar_tag_points.
        tr_idx (int): Index of the top right corner point in ar_tag_points.
        tl_idx (int): Index of the top left corner point in ar_tag_points.

    """


    # Bottom Left Point
    bottom_left_pt = ar_tag_points[bl_idx]
    # Bottom Right Point
    bottom_right_pt = ar_tag_points[br_idx]

    # Top Left Point
    top_left_pt = ar_tag_points[tl_idx]

    # Vectors along the edges
    vec_bottom_edge = bottom_right_pt - bottom_left_pt
    vec_left_edge = top_left_pt - bottom_left_pt

    # Normal Vector using cross product
    normal_vector = np.cross(vec_bottom_edge, vec_left_edge)
    normal_vector_unit = normal_vector / np.linalg.norm(normal_vector)  # unit normal vector

    return normal_vector_unit


# Find unit vectors along bottom edge and left edge
def unit_vectors_bottom_and_left_edges_global_frame(bl_point, br_point, tl_point):
    """
    Find unit vectors along bottom edge and left edge in global frame.

    Parameters:
        bl_point (np.ndarray): Bottom left point coordinates.
        br_point (np.ndarray): Bottom right point coordinates.
        tl_point (np.ndarray): Top left point coordinates.

    Returns:
        unit_vec_bottom_edge: Unit vector along bottom edge.
        unit_vec_left_edge: Unit vector along left edge.
    """
    vec_bottom_edge = br_point - bl_point
    unit_vec_bottom_edge = vec_bottom_edge / np.linalg.norm(vec_bottom_edge)
    vec_left_edge = tl_point - bl_point
    unit_vec_left_edge = vec_left_edge / np.linalg.norm(vec_left_edge)
    return unit_vec_bottom_edge, unit_vec_left_edge

def main():
    # Example AR tag points
    ar_tag_points = np.array([[30, 10*0.866, 10*0.5],
                            [0, 10*0.866, 10*0.5],
                            [0, 0, 0],
                            [30, 0, 0]])

    print("\n--- Testing Multiple Keys ---\n")
    print("AR Tag Points:\n", ar_tag_points)

    # Keys to test (replace with full dictionary after measurements)
    keyboard_keys_dict = {
        'k': np.array([17.5, 5]),
        'A': np.array([4.5, 5])
    }

    # Results should show change in z-coordinates of keys
    # and y coordinates  now *0.866
    # no change in x-coordinates

    for key, values in keyboard_keys_dict.items():
        key_local = values
        key_global = find_key_pt_in_global_frame(key_local, ar_tag_points)
        print(f"Global Coordinates of Key {key} (local co-ordinates {key_local}): {key_global}")

if __name__ == "__main__":
    main()


