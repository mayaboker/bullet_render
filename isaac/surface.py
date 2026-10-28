"""Vertical surface queries over world-space triangles; no USD, GPU or numpy imports."""

# Barycentric slack so a point exactly on a shared edge belongs to both triangles.
EDGE_TOLERANCE = 1e-9
# Triangles whose projected area is below this are vertical or degenerate and cannot be stood on.
MIN_PROJECTED_AREA_X2 = 1e-12
# Vertex order of the twelve triangles covering an axis-aligned box; corners are
# indexed by bit (x, y, z) with 1 meaning the positive half-size.
BOX_TRIANGLES = (
    (0, 2, 3), (0, 3, 1), (4, 5, 7), (4, 7, 6), (0, 1, 5), (0, 5, 4),
    (2, 6, 7), (2, 7, 3), (0, 4, 6), (0, 6, 2), (1, 3, 7), (1, 7, 5))


def fan_triangles(counts, indices):
    """Yield vertex index triples for polygon faces given USD-style counts and indices."""
    start = 0
    for count in counts:
        face = indices[start:start + count]
        start += count
        for i in range(1, count - 1):
            yield face[0], face[i], face[i + 1]


def box_triangles(centre, size):
    """World-space triangles of an axis-aligned box with the given centre and full size."""
    corners = [tuple(centre[axis] + (size[axis] / 2 if bit & (1 << axis) else -size[axis] / 2) for axis in range(3))
               for bit in range(8)]
    return [(corners[a], corners[b], corners[c]) for a, b, c in BOX_TRIANGLES]


def heights_at(x, y, triangles):
    """Z values where the vertical line through (x, y) crosses the triangles, highest first.

    Each triangle is three (x, y, z) points in the same frame as x and y.
    Vertical and degenerate triangles are ignored; a point on an edge counts.
    """
    heights = []
    for (ax, ay, az), (bx, by, bz), (cx, cy, cz) in triangles:
        denominator = (by - cy) * (ax - cx) + (cx - bx) * (ay - cy)
        if abs(denominator) < MIN_PROJECTED_AREA_X2:
            continue
        weight_a = ((by - cy) * (x - cx) + (cx - bx) * (y - cy)) / denominator
        weight_b = ((cy - ay) * (x - cx) + (ax - cx) * (y - cy)) / denominator
        weight_c = 1.0 - weight_a - weight_b
        if min(weight_a, weight_b, weight_c) >= -EDGE_TOLERANCE:
            heights.append(weight_a * az + weight_b * bz + weight_c * cz)
    return sorted(heights, reverse=True)
