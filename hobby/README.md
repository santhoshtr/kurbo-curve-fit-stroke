# Hobby's Curves

John Hobby's curve-fitting algorithm is renowned for its ability to generate aes
thetically pleasing, smooth curves that pass through a series of given points. W
hile many implementations focus on the algorithm's automatic smoothing capabilit
ies, often controlled by a "tension" or "curl" parameter, its true power lies in
 the fine-grained control it offers over the curve's shape at each node, specifi
cally by allowing the user to specify entry and exit angles.

At its core, Hobby's algorithm is a sophisticated method for choosing the control
l points of a sequence of cubic Bézier curves that smoothly connect a series of
 nodes. Instead of requiring the user to manually place these control points, th
e algorithm calculates their optimal positions to create a visually pleasing cur
ve.

The "magic" of Hobby's algorithm lies in its concept of **"mock curvature."** Ra
ther than dealing with the computationally complex true curvature, it uses a sim
pler, linear approximation. The algorithm's primary goal is to make the mock cur
vature continuous at each node. This means the curve's "turning rate" is the sam
e as it leaves one segment and enters the next, resulting in a smooth transition
.

To achieve this, the algorithm determines the optimal entry and exit angles for
the curve at each node. It sets up a system of linear equations where the unknown
ns are these angles. The equations are designed to balance the angles at each no
de to ensure this continuity of mock curvature.

### Controlling the Curve

The key is to understand that when you provide an explicit angle for a node, you
 are essentially removing a variable from the algorithm's system of equations an
d replacing it with a constant.

*   **Default Behavior (No Specified Angles):** The algorithm treats the entry a
nd exit angles at each interior node as unknowns and solves for them to achieve
the smoothest possible curve based on the surrounding points.

*   **With Specified Angles:** When you specify an exit angle for a point `P_i`
and an entry angle for the next point `P_{i+1}`, you are directly telling the al
gorithm the direction the curve must take at those points. These angles are no l
onger variables to be solved for; they become fixed constraints. The algorithm t
hen solves the remaining system of equations for the unspecified angles, ensurin
g the rest of the curve smoothly accommodates your constraints.

For example, `(1,0)(dir 45) .. {dir 270}(0,1)`, you are fixing the exit directio
n at `(1,0)` to be 45 degrees and the entry direction at `(0,1)` to be 270 degre
es. The algorithm will then determine the Bézier control points that satisfy th
ese endpoint conditions while still producing a smooth path between them.

### The Mathematical Framework

Hobby's algorithm works with the angles that the curve's tangent makes with the
chord connecting two consecutive points. Let's denote the points as `z_0, z_1, .
.., z_n`. For a segment between `z_k` and `z_{k+1}`, the key variables are:

*   `θ_k`: The angle of the outgoing tangent at `z_k` with respect to the chord
 `z_{k+1} - z_k`.
*   `φ_{k+1}`: The angle of the incoming tangent at `z_{k+1}` with respect to t
he chord `z_{k+1} - z_k`.

The core of Hobby's algorithm is a set of equations that relate these angles to
ensure mock curvature continuity. For each interior node `z_k` (for `k = 1, ...,
 n-1`), the condition of mock curvature continuity leads to a linear equation of
 the form:

`a_k * θ_k + b_k * φ_k = c_k`

where `a_k`, `b_k`, and `c_k` are coefficients determined by the positions of th
e points `z_{k-1}`, `z_k`, and `z_{k+1}`, and the tension parameters. This creat
es a **tridiagonal system of linear equations**, which can be solved efficiently
.


If you specify an exit direction at `z_k`, the angle `θ_k` is no longer an unkn
own. It's a known value that you provide. Similarly, if you specify an entry dir
ection at `z_k`, `φ_k` becomes a known value.

When you provide these angles, the corresponding terms in the system of equation
s are moved to the right-hand side, as they are now constants. This modifies the
 system, but it remains a solvable tridiagonal system for the remaining unknown
angles.

## Tension

Our physical intuition suggests tension exists *along* something, like a rope, n
ot *at* a single point.

Hobby's genius was to reframe the problem. Instead of defining tension "between"
 points, he defined a control parameter "at" each point that governs the behavior
r of the curve as it passes *through* that point.

Let's break it down from intuition to mathematics.

### 1. The Core Intuition: A Knob at Each Point

Your intuition about tension being between two points is based on a physical mod
el, like stretching a rubber band between two nails.

Hobby's model is slightly different. Imagine your points are posts or nails hamm
ered into a board. The curve is a thin, flexible strip of wood (a spline) that y
ou are weaving through these posts.

Now, at each post, you have a **"stiffness knob"**. This knob controls how tight
ly the strip of wood is forced to hug the post.

*   **Low Tension (e.g., `tension = 0.7`)**: This is like a "loose" knob. The wo
oden strip is allowed to bulge and curve generously as it passes through the pos
t. The curve is "rounder" and "more relaxed."
*   **Default Tension (`tension = 1.0`)**: This is the default, aesthetically pl
easing setting that Hobby found to work well. It provides a balanced, natural-lo
oking curve.
*   **High Tension (e.g., `tension = 2.0`)**: This is like a "tight" knob. You a
re forcing the wooden strip to become very straight on either side of the post.
The curve is pulled taut, becoming "flatter" and closer to the straight-line pat
h (the chord).
*   **Infinite Tension**: If you could turn the knob to infinity, the wooden str
ip would become perfectly straight on either side of the post, turning the smooth
h curve into a series of straight line segments.

So, **tension at a point controls the straightness of the curve segments that co
nnect *to* that point.** You are controlling a property of the node, and its eff
etc radiates out to the connected paths.

### 2. How It Works Mathematically

The effect of tension is implemented in two key places in the algorithm.

#### A. In the System of Equations

Recall the central equation that balances the angles at each node `k`. The coeff
icients in that equation are what define the curve's "character." One of the key
 coefficients is `alpha`:

`alpha_k = (tensions[k+1] * length_of_chord_k) / (tensions[k] * length_of_chord_
{k-1})`

Look closely at how the tensions are used:

*   `tensions[k]` is in the **denominator**.
*   `tensions[k+1]` is in the **numerator**.

This `alpha_k` value is a weight. It determines how much the angle at point `k`
is influenced by the direction of the *next* chord (`k+1`).

If you **increase `tensions[k]`**, you make `alpha_k` smaller. This **reduces th
e influence** of the next part of the curve on the current point. The system of
equations will solve for angles that are more "local" and less forward-looking,
which results in a straighter path.

#### B. In the Calculation of Bézier Control Points

This is the most direct and easiest place to see the effect. After the algorithm
 has solved for the optimal entry and exit angles (`phi` and `theta`), it calcul
ates the positions of the two Bézier control points for the segment between `po
int_k` and `point_{k+1}`.

The formula for the first control point (`c1`) leaving `point_k` looks like this
:

`c1_distance = length_of_chord_k / (3.0 * tensions[k])`
`c1 = point_k + c1_distance * (vector in direction of exit angle)`

And for the second control point (`c2`) arriving at `point_{k+1}`:

`c2_distance = length_of_chord_k / (3.0 * tensions[k+1])`
`c2 = point_{k+1} - c2_distance * (vector in direction of entry angle)`

Notice that `tensions[k]` is in the denominator for the control point leaving `p
oint_k`, and `tensions[k+1]` is in the denominator for the one arriving at `point
t_{k+1}`.

This has a very clear geometric meaning:

*   If you **increase `tensions[k]`**, the `c1_distance` gets **smaller**. This
pulls the control point `c1` closer to `point_k`.
*   If you **increase `tensions[k+1]`**, the `c2_distance` gets **smaller**. Thi
s pulls the control point `c2` closer to `point_{k+1}`.

**Why does this matter?** In a cubic Bézier curve, the closer the control point
s are to their respective endpoints, the "tighter" and "straighter" the curve se
gment becomes. When a control point is very close to its endpoint, the curve bar
ely has time to bend before it's pulled back in the direction of the next endpoi
nt.

![./tension.png]

This diagram perfectly illustrates the effect. The top curve has lower tension,
so the control points (`c1`, `c2`) are farther from the endpoints (`P0`, `P1`),
allowing the curve to be full and round. The bottom curve has high tension, pull
ing the control points in and making the path much more direct and flat.

