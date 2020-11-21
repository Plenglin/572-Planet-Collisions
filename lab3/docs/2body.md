## 2-body momentum

The n-body calculation is done simultaneously, and using multiple 2-body calculations, so I'll first talk about how 
those are calculated.

We use conservation of momentum, but include a term for restitution, as derived [here](https://en.wikipedia.org/wiki/Coefficient_of_restitution#Derivation).
From that, we derive impulses $$I_a$$ and $$I_b$$ to be used for approximating the normal force $$n$$ for the friction.

If our two balls are leaving each other, or if they have low we don't do anything.

## 2-body friction

We use the following values to calculate friction:

- angular velocity vectors $$\omega_{A,B}$$ (right-hand rule relative to rotation, and scaled to angular velocity in
radians) 
- radial vectors $$r_A$$ and $$r_B$$ (a vector from particle to center of
contact point)

The relative surface velocity is $$\omega_B \times r_B - \omega_A \times r_A$$. If A and B were spinning, but not moving,
this is how fast someone standing on A would be moving relative to you. 

The relative tangential velocity is how fast the two particles are moving in the plane 
gets us the net relative surface velocity. If you were standing on B at the point of contact, and then
A lands on top of you, A's surface would be moving with that velocity relative to you. So, it is the direction
friction is applied in.

Friction is easily calculated using $$f = \mu n$$. $$I$$ and $$f$$ are applied to both contacts, and we are done.
