
from manim import *


class BlimpForces(Scene):
    def construct(self):
        # Axes
        axes = Axes(
            x_range=[-5, 5, 1],
            y_range=[-3, 3, 1],
            axis_config={"include_tip": True}
        )
        self.play(Create(axes))

        # Blimp (ellipse)
        blimp = Ellipse(width=4, height=2, color=BLUE). shift(UP)
        self.play(FadeIn(blimp))

        # Gondola (rectangle)
        gondola = Rectangle(width=0.8, height=0.4, color=YELLOW). next_to(
            blimp, DOWN, buff=0.1)
        self.play(FadeIn(gondola))

        # Thrust force (arrows)
        thrust = Arrow(start=LEFT, end=RIGHT, color=RED). next_to(
            blimp, RIGHT, buff=0.1)
        self.play(Create(thrust))
        thrust_label = Text("Thrust", color=RED). next_to(thrust, UP)
        self.play(Write(thrust_label))

        # Weight force
        weight = Arrow(start=UP, end=DOWN, color=GREEN). next_to(
            gondola, DOWN, buff=0.1)
        self.play(Create(weight))
        weight_label = Text("Weight", color=GREEN). next_to(weight, LEFT)
        self.play(Write(weight_label))

        # Title
        title = Title("Blimp Forces")
        self.play(Write(title))

        # Wait before ending
        self.wait(2)


class BlimpTeleopExplanation(Scene):
    def construct(self):
        # Title
        title = Text("Sano Blimp Teleop Logic", font_size=36).to_edge(UP)
        self.play(Write(title))

        # Blimp representation as a circle
        blimp = Circle(radius=1.5, color=WHITE).shift(UP * 2)
        self.play(FadeIn(blimp))

        # Axes representation
        x_axis = Arrow(LEFT * 3, RIGHT * 3, buff=0).set_color(BLUE)
        y_axis = Arrow(DOWN * 2, UP * 2, buff=0).set_color(RED)
        z_axis = Arrow(IN * 2, OUT * 2, buff=0).set_color(GREEN)

        x_label = Text("X (Throttle)", font_size=24).next_to(x_axis, DOWN)
        y_label = Text("Y (Yaw)", font_size=24).next_to(y_axis, LEFT)
        z_label = Text("Z (Altitude)", font_size=24).next_to(z_axis, RIGHT)

        self.play(Create(x_axis), Write(x_label))
        self.play(Create(y_axis), Write(y_label))
        self.play(Create(z_axis), Write(z_label))

        # Motor and control surface labels
        m1_text = Text("M1", font_size=24).next_to(blimp, LEFT)
        m2_text = Text("M2", font_size=24).next_to(blimp, RIGHT)
        s3_text = Text("S3", font_size=24).next_to(blimp, UP + LEFT)
        s4_text = Text("S4", font_size=24).next_to(blimp, DOWN + RIGHT)
        s1_text = Text("S1 (Gate)", font_size=24).next_to(blimp, DOWN)

        self.play(Write(m1_text), Write(m2_text), Write(
            s3_text), Write(s4_text), Write(s1_text))

        # Throttle (X-axis) animation
        throttle_up = Arrow(
            blimp.get_bottom(), blimp.get_bottom() + UP * 2, buff=0).set_color(YELLOW)
        throttle_down = Arrow(
            blimp.get_bottom(), blimp.get_bottom() + DOWN * 2, buff=0).set_color(YELLOW)

        self.play(Create(throttle_up))
        self.wait(0.5)
        self.play(ReplacementTransform(throttle_up, throttle_down))
        self.wait(0.5)
        self.play(FadeOut(throttle_down))

        # Yaw (Y-axis) animation
        yaw_left = CurvedArrow(
            blimp.get_right(), blimp.get_left(), angle=-PI/2).set_color(ORANGE)
        yaw_right = CurvedArrow(
            blimp.get_left(), blimp.get_right(), angle=PI/2).set_color(ORANGE)

        self.play(Create(yaw_left))
        self.wait(0.5)
        self.play(ReplacementTransform(yaw_left, yaw_right))
        self.wait(0.5)
        self.play(FadeOut(yaw_right))

        # Altitude (Z-axis) animation
        altitude_up = Arrow(blimp.get_top(), blimp.get_top() +
                            UP * 2, buff=0).set_color(PURPLE)
        altitude_down = Arrow(
            blimp.get_top(), blimp.get_top() + DOWN * 2, buff=0).set_color(PURPLE)

        self.play(Create(altitude_up))
        self.wait(0.5)
        self.play(ReplacementTransform(altitude_up, altitude_down))
        self.wait(0.5)
        self.play(FadeOut(altitude_down))

        # Gate Mechanism (S1 open/close)
        gate_open = Text("Gate Open", font_size=24,
                         color=GREEN).next_to(s1_text, RIGHT)
        gate_closed = Text("Gate Closed", font_size=24,
                           color=RED).next_to(s1_text, RIGHT)

        self.play(Write(gate_open))
        self.wait(0.5)
        self.play(ReplacementTransform(gate_open, gate_closed))
        self.wait(0.5)
        self.play(FadeOut(gate_closed))

        # Summary
        summary = Text("Teleop Logic:\n- X controls Throttle (M1, M2)\n- Y controls Yaw (M1, M2 balance)\n- Z controls Altitude (S3, S4)\n- S1 controls the Gate", font_size=20).shift(DOWN * 3)
        self.play(Write(summary))

        self.wait(2)
