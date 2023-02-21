import math
# get a list of object points that can be "seen"

# get the farthest points on each object from this list

# get a list of road edge points that can be "seen"
# draw a circle around the object points for avoidance

# This is a mathematical version of this technique: https://www.mathsisfun.com/geometry/construct-trianglecircum.html
class circumscriber:
    
    def __init__(self, p1, p2, p3):
        self.point1 = p1
        self.point2 = p2
        self.point3 = p3

    # finds the circumcenter of the triangle and the radius of the circumscribing circle
    def circumscribe(self):
        # solve the triangle: https://socratic.org/questions/how-do-you-find-the-three-angles-of-the-triangle-with-the-given-vertices-a-1-0-b
        # get side lengths of the triangle
        length_a = math.sqrt((self.point2[0] - self.point3[0])**2 + (self.point2[1] - self.point3[1])**2) 
        length_b = math.sqrt((self.point1[0] - self.point3[0])**2 + (self.point1[1] - self.point3[1])**2)
        length_c = math.sqrt((self.point1[0] - self.point2[0])**2 + (self.point1[1] - self.point2[1])**2)
        
        # apply the law of cosines to find each angle
        cos_arg_a = (length_b**2 + length_c**2 - length_a**2) / (2 * length_b * length_c)
        angle_a = math.acos(cos_arg_a)
        cos_arg_b = (length_a**2 + length_c**2 - length_b**2) / (2 * length_c * length_a)
        angle_b = math.acos(cos_arg_b)
        cos_arg_c = (length_a**2 + length_b**2 - length_c**2) / (2 * length_a * length_b)
        angle_c = math.acos(cos_arg_c)
        
        # circumcenter formula: https://www.cuemath.com/geometry/circumcenter/
        x = (self.point1[0] * math.sin(2 * angle_a) + self.point2[0] * math.sin(2 * angle_b) + self.point3[0] * math.sin(2 * angle_c)) / (math.sin(2 * angle_a) + math.sin(2 * angle_b) + math.sin(2 * angle_c))
        y = (self.point1[1] * math.sin(2 * angle_a) + self.point2[1] * math.sin(2 * angle_b) + self.point3[1] * math.sin(2 * angle_c)) / (math.sin(2 * angle_a) + math.sin(2 * angle_b) + math.sin(2 * angle_c))
        circumcenter = (x, y)     

        # the radius of the circumscribing circle
        radius = math.sqrt((circumcenter[0] - self.point1[0])**2 + (circumcenter[1] - self.point1[1])**2)

        return [circumcenter, radius]
        