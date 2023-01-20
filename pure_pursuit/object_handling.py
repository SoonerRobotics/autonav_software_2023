# get a list of object points that can be "seen"

# get the farthest points on each object from this list

# get a list of road edge points that can be "seen"
#draw a circle around the object points for avoidance

# Get the vertices
point1 = [3, 2]
point2 = [4, 3]
point3 = [2,1]

# Bisect 2 of the vertices
def bisect(p1, p2):
    bisection = [(p1[0] + p2[0]) / 2, p1[1] + p2[1] / 2]

    return bisection

bisection1 = bisect(point1, point2)
bisection2 = bisect(point2, point3)

# draw lines perpendicular to these points and the line
# equation of 1st perpendicular bisector y + by1  = - 1/m(x+bx1)
# m = (y2 - y1/ x2 - x1)
# y = (-(x2 - x1 / y2 - y1) * (x + bx1)) - by1

# equation of 2nd perpendicular bisector y + by3 = - 1/m(x+bx3)
# m = (y3 - y2/ x3 - x2)
# y = (-(x3 - x2 / y3 - y2) * (x + bx3)) - by3
# intersect them mathematically


# find intersection