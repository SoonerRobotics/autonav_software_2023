import math

# Gracefully stolen from https://github.com/xiaoxiae/PurePursuitAlgorithm/blob/master/src/main/PurePursuit.java
class PurePursuit:

    def __init__(self):
        self.path = []
    
    def add_point(self, x, y):
        self.path.append((x,y))

    def set_points(self, pts):
        self.path = pts

    def get_lookahead_point(self, x, y, r):
        lookahead = None

        for i in range(len(self.path)-1):
            segStart = self.path[i]
            segEnd = self.path[i+1]

            p1 = (segStart[0] - x, segStart[1] - y)
            p2 = (segEnd[0] - x, segEnd[1] - y)

            dx = p2[0] - p1[0]
            dy = p2[1] - p1[1]

            d = math.sqrt(dx * dx + dy * dy)
            D = p1[0] * p2[1] - p2[0] * p1[1]

            discriminant = r * r * d * d - D * D
            if discriminant < 0 or p1 == p2:
                continue

            sign = lambda x: (1, -1)[x < 0]

            x1 = (D * dy + sign(dy) * dx * math.sqrt(discriminant)) / (d * d)
            x2 = (D * dy - sign(dy) * dx * math.sqrt(discriminant)) / (d * d)

            y1 = (-D * dx + abs(dy) * math.sqrt(discriminant)) / (d * d)
            y2 = (-D * dx - abs(dy) * math.sqrt(discriminant)) / (d * d)

            validIntersection1 = min(p1[0], p2[0]) < x1 and x1 < max(p1[0], p2[0]) or min(p1[1], p2[1]) < y1 and y1 < max(p1[1], p2[1])
            validIntersection2 = min(p1[0], p2[0]) < x2 and x2 < max(p1[0], p2[0]) or min(p1[1], p2[1]) < y2 and y2 < max(p1[1], p2[1])

            if validIntersection1 or validIntersection2:
                lookahead = None

            if validIntersection1:
                lookahead = (x1 + x, y1 + y)

            if validIntersection2:
                if lookahead == None or abs(x1 - p2[0]) > abs(x2 - p2[0]) or abs(y1 - p2[1]) > abs(y2 - p2[1]):
                    lookahead = (x2 + x, y2 + y)

        if len(self.path) > 0:
            lastPoint = self.path[len(self.path) - 1]

            endX = lastPoint[0]
            endY = lastPoint[1]

            if math.sqrt((endX - x) * (endX - x) + (endY - y) * (endY - y)) <= r:
                return (endX, endY)

        return lookahead