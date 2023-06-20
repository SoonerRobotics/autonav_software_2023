import matplotlib.pyplot as plt

class circumscription_tester:
    def initialize(self):
        self.blank = None

    def test(self, p1, p2, p3, geom_ctr, radius):
        fig, ax = plt.subplots()

        x, y = zip(p1, p2, p3)
        ax.scatter(x, y, color="Blue", label="Vertices")
        ax.plot([p1[0], p2[0]], [p1[1], p2[1]], color="Blue")
        ax.plot([p2[0], p3[0]], [p2[1], p3[1]], color="Blue")
        ax.plot([p3[0], p1[0]], [p3[1], p1[1]], color="Blue")
        ax.scatter(geom_ctr[0], geom_ctr[1], color="Green", label="Circumcenter")
        circle1 = plt.Circle(geom_ctr, radius, color="Red", label="Circumscribing circle", fill=False)
        ax.add_patch(circle1)

        plt.legend()
        plt.show()
