# Curve Fitter and Stroker

## Curve Fitter

This is a rust port of Raph Levien's smooth curve fitting using Two parameter curve approach.
Given a series of points, it constructs a smooth curve connecting all those points.

* Article: https://raphlinus.github.io/curves/2018/12/21/new-spline.html
* Paper: https://spline.technology/paper1.pdf
* Javascript implementation and demo: https://github.com/raphlinus/spline-research
* LICENSE MIT or Apache

This Rust port is also licensed under MIT

![](./docs/images/curve-fit.webp)

## Variable Stroker

The Kurbo library has a stroke algorithm to expand a path (skeleton or a list of
connected Cubic curves) with a given offset `d`. It works well and is based on extensive
research by the Kurbo team.

![](./docs/images/const-stroke.webp)

I adapted the system to try variable stroking where the value of `d` varies at curve joints.
This is useful in illustrating variable thickness letters. It reuses most of the
code from the Kurbo library. Wherever the constant value `d` is used, a value from an array of varying
widths is supplied.

The resulting stroke outline is acceptable to the eye,
but not great in terms of the number of points in the outline.

My objective was to see if such outlines can be used in type engineering (font making),
especially for variable fonts. However, variable fonts need interpolatable shapes.
They need the same number of points in all variations. The unpredictable number of points
in the outline as the input widths change is not acceptable for that workflow.
Simplification APIs are not usable as they also add non-deterministic points.


## Interpolatable Variable Strokes

A simple stupid trick I have used in this repo for achieving interpolation
is to make the sub-divisions of the outline
for a path segment deterministic. Initially I used 4 sub-divisions for every path segment.
But later I changed it to dynamic based on the curvature of the source path.
The resulting curves are interpolatable but lost the perfection from the previous step
where the stroke calculation was based on a complex error reduction strategy. My approach reduced that to a
simpler Tiller-Hanson-like approach.

When I shared this work with the Kurbo team, Raph suggested using a linear perturbation system

$B_{offset}(t) = B(t) + c \cdot D(t)$

*   **$B(t)$**: This is our **Source Spine** (the cubic Bézier you are stroking).
*   **$c$**: This is the **Scalar Width** (or half-width). In a variable stroke, this is our $w(t)$.
*   **$D(t)$**: This is a **"Direction Curve"**. It represents the Normal Vector, but approximated as a cubic Bézier itself.

Intuitively, instead of trying to calculate a perfect parallel curve
(which is mathematically impossible to represent exactly as a Bézier),
we are creating a "vector field" that points outwards from the curve.

1.  Define a curve $D(t)$ that represents "Straight Out" (Normal) along the spine.
2.  To generate the offset, take the spine point $B(t)$ and add $D(t)$ multiplied by the width at that point.

Why does this guarantee interpolatability? If you have a "Thin" shape and a "Bold" shape,
they share the exact same $B(t)$ and $D(t)$. The *only* thing that changes is $c$ (the width).
Because the formula is **linear** (it's just addition), point $P_{offset}$
moves in a straight line as you increase the weight.
This is the definition of perfect variable font interpolation.

Instead of my dynamic curvature-based sub-divisions, Raph recommended using a single midpoint.
Since we know the start point (from $t=0$) and the end point (from $t=1$),
and we know the tangents (from the derivative formula below),
we still have degrees of freedom: **how long are the control handles?**
Raph suggests calculating the exact offset point at $t=0.5$ (the middle).
You then mathematically solve for the handle lengths that force the cubic curve to pass through that middle point.
This is deterministic and fast.

The endpoint tangents for variable offset are $(1 + \kappa d)x' + n d'$

This formula tells you exactly **what direction the offset curve is pointing** at any moment $t$.

$$ \text{Tangent}_{offset} = \underbrace{(1 + \kappa d)x'}_{\text{Part A}} + \underbrace{n d'}_{\text{Part B}} $$

#### **Part A: The "Parallel" Movement**
*   **$x'$**: This is the tangent of the spine (moving forward along the road).
*   **$d$**: The current width.
*   **$\kappa$ (Kappa)**: The curvature (how tight the turn is).

#### **Part B: The "Taper" Movement** - sideways push
*   **$n$**: The Unit Normal (pointing 90° sideways).
*   **$d'$**: The **Derivative of Width** (The slope). How fast is the width changing?

If the pen is getting wider ($d' > 0$), the edge of the ink must move **outwards** away from the center. This adds a sideways vector component.


![](./docs/images/variable-stroke-interpolatable.webp)
