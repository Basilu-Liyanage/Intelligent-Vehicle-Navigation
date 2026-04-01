# spirograph.py
import turtle
import colorsys

def spirograph(turns=36, circle_radius=120):
    screen = turtle.Screen()
    screen.bgcolor("black")
    screen.title("Spirograph")

    t = turtle.Turtle()
    t.speed(0)
    t.width(2)
    t.hideturtle()

    # generate a rainbow of colors
    for i in range(turns):
        hue = i / turns
        r, g, b = [int(255*c) for c in colorsys.hsv_to_rgb(hue, 1, 1)]
        screen.colormode(255)
        t.pencolor(r, g, b)
        t.circle(circle_radius)
        t.right(360 / turns)

    screen.exitonclick()

if __name__ == "__main__":
    spirograph(turns=72, circle_radius=100)