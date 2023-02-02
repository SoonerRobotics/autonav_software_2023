from object_handling import circumscriber
from circumscription_test import circumscription_tester
import random

# generates a random set of coordinates for 3 points
def random_triangle(p1, p2, p3):

    p1[0] = random.randint(-10, 10)
    p1[1] = random.randint(-10, 10)
    p2[0] = random.randint(-10, 10)
    p2[1] = random.randint(-10, 10)
    p3[0] = random.randint(-10, 10)
    p3[1] = random.randint(-10, 10)


if __name__ == "__main__":
    print("I am the main function for the object_handler")
    p1 = [0, 0]
    p2 = [0, 0]
    p3 = [0, 0]

    random_triangle(p1, p2, p3)

    # create a circumscriber object to get the circumcenter and radius of the circumscribing circle
    triangle1 = circumscriber(p1, p2, p3)
    center, radius = triangle1.circumscribe()

    # create a circumscription tester to test and plot the circle and triangle 
    test1 = circumscription_tester()
    print(f"p1 {p1}, p2 {p2}, p3 {p3}, geom_ctr {center}, radius {radius}")
    circumscription_tester.test(test1, p1, p2, p3, center, radius)