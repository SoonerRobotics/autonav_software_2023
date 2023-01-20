import math

# Algorithm source: https://github.com/xiaoxiae/PurePursuitAlgorithm/blob/master/src/main/PurePursuit.java
class pure_pursuit:
    def initialize(self):
        self.path = []

    def setpath(self, list):
        self.path = list
    
    def add_to_path(self, x, y):
        self.path.append((x, y))

    def get_lookahead_point(self, a, b, r):
        lookahead = None

        for i in range(len(self.path)-1):

            #  Make segment between each point
            seg_start = self.path[i]
            seg_end = self.path[i+1]

            
            #  Intersect segments with circle
            
            # translate the segment to the origin
            p1 = (seg_start[0] - a, seg_start[1] - b)
            p2 = (seg_end[0] - a, seg_end[1] - b)

            # get dr, D, sgn(dy) and the discriminant to compute intersections from https://mathworld.wolfram.com/Circle-LineIntersection.html
            dx = p2[0] - p1[0]
            dy = p2[1] - p1[1]

            dr = math.sqrt((dx**2) + (dy**2))
            D = (p1[0]*p2[1]) - (p2[0]*p1[1])

            discrim = r**2 * dr**2 - D**2

            if dy < 0:
                sgn_dy = -1
            else:
                sgn_dy = 1
            
            # if the discriminant is < 0, the segment does not intersect the circle
            print(discrim)
            if discrim < 0 or p1 == p2:
                continue

            # x and y coordinates of the intersections are solved with the quadratic formula, giving 2 points, labeled _add and _sub respectively
            # x1 = (D * dy + sign(dy) * dx * math.sqrt(discriminant)) / (d * d)
            new_x_add = ((D * dy) + (sgn_dy * dx * math.sqrt(discrim))) / dr**2
            new_y_add = ((-D * dx) + (abs(dy) * math.sqrt(discrim))) / dr**2

            new_x_sub = ((D * dy) - (sgn_dy * dx * math.sqrt(discrim))) / dr**2
            new_y_sub = ((-D * dx) - (abs(dy) * math.sqrt(discrim))) / dr**2

            # check that the new point is actually on the segment
            # segment min x < goal x < segment max x, or segment min y < goal y < segment max y
            # print(f'segment_min_x {min(p1[0], p2[0])} < new_x_add {new_x_add} < segment_max_x {max(p1[0], p2[0])}')
            valid_intersection_add = min(p1[0], p2[0]) < new_x_add and new_x_add < max(p1[0], p2[0]) or min(p1[1], p2[1]) < new_y_add and new_y_add < max(p1[1], p2[1])
            valid_intersection_sub = min(p1[0], p2[0]) < new_x_sub and new_x_sub < max(p1[0], p2[0]) or min(p1[1], p2[1]) < new_y_sub and new_y_sub < max(p1[1], p2[1])

            if valid_intersection_add or valid_intersection_sub:
                lookahead = None
            
            if valid_intersection_add:
                lookahead = (new_x_add + a, new_y_add + b)
            
            if valid_intersection_sub:
                if lookahead == None or abs(new_x_add - p2[0]) > abs(new_x_sub - p2[0]) or abs(new_y_add - p2[1]) > abs(new_y_sub - p2[1]):
                    lookahead = (new_x_sub + a, new_y_sub + b)
                
            if len(self.path) > 0:
                lastPoint = self.path[len(self.path) - 1]

                endX = lastPoint[0]
                endY = lastPoint[1]

                if math.sqrt((endX - a) * (endX -a) + (endY - b) * (endY - b)) <= r:
                    return(endX, endY)

        return lookahead
        
