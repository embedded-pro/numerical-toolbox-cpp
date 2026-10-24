# SE(2) Transform, Planar Twists and Wrenches — Overview

## What it is
The algebra of motion in a plane: a pose is a heading plus a 2-D position; a *planar twist* is a
body's instantaneous velocity (forward, sideways, turn rate); a *planar wrench* is a force in the plane
together with a moment about the vertical axis. The adjoint moves twists and wrenches between frames, and
the exponential and logarithm convert between a constant twist held for some time and the finite motion
it produces.

## Why it matters (embedded)
Ground robots, mobile bases, warehouse vehicles and planar mechanisms work with poses in the plane.
Odometry, path tracking, landmark and scan-matching localization all need the same few operations.
SE(2) costs a fraction of SE(3): four numbers per pose and no trigonometry in composition. Writing it
once, with item 53's ordering, keeps planar and spatial code consistent when a mobile base carries an arm.

## How it works (intuition)
The rotation is stored as the cosine and sine of the heading, so composing two poses multiplies two unit
complex numbers and rotates one offset — no angle arithmetic and no wrap-around bugs. A constant twist
held for a time moves the body along a circular arc (a straight line when the turn rate is zero). The
exponential computes the end of that arc exactly, which is why it is the right way to integrate wheel
odometry. The logarithm runs the other way: it returns the constant twist that would carry one pose to
another in unit time, and so gives a pose difference that respects the vehicle's geometry.

## Key parameters
- **Ordering convention** — linear part first for twists `(vx, vy, ω)` and wrenches `(fx, fy, τ)`.
- **Heading branch** — headings and `Log` angles are reported in `(−π, π]`.
- **Unit rotation** — `(cos, sin)` kept unit length; renormalised when a pose is integrated over time.

## Reference
J. Solà, J. Deray, D. Atchuthan, "A micro Lie theory for state estimation in robotics,"
arXiv:1812.01537 (2018), Appendix C; K. M. Lynch, F. C. Park, *Modern Robotics* (2017), Ch. 3 and 13.

## See also
`SE3Transform` (item 53, the spatial version and `ToSE3`'s target), `AngleUtilities` (item 54, heading
wrap and differences); robotics-toolbox-cpp's wheeled-robot models and planar odometry.
