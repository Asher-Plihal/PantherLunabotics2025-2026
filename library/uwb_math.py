import math


def compute_distance_to_anchor(d_left: float, d_right: float, tag_sep: float) -> float:
    """
    Distance from the robot center (midpoint of the two tags) to the anchor.
    Uses the triangle median formula.

    Args:
        d_left:   distance from left tag to anchor (metres)
        d_right:  distance from right tag to anchor (metres)
        tag_sep:  fixed separation between left and right tags (metres)
    """
    return math.sqrt(2 * d_left**2 + 2 * d_right**2 - tag_sep**2) / 2


def compute_heading_offset(d_left: float, d_right: float, tag_sep: float) -> float:
    """
    Angle (degrees) between the robot's forward direction and the
    anchor-to-robot-center line.

        0°  → robot facing directly toward or away from anchor
       90°  → robot broadside, right tag farther from anchor
      -90°  → robot broadside, left tag farther from anchor

    Args:
        d_left:   distance from left tag to anchor (metres)
        d_right:  distance from right tag to anchor (metres)
        tag_sep:  fixed separation between left and right tags (metres)
    """
    d_center = compute_distance_to_anchor(d_left, d_right, tag_sep)
    sin_offset = (d_right**2 - d_left**2) / (2 * d_center * tag_sep)
    return math.degrees(math.asin(sin_offset))
