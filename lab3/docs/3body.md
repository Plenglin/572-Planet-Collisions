Since others might also be doing n-body collisions, I thought it might be useful to describe how our group did it. Please feel free to ask questions if you have any. Also, I'll probably be updating this over time. 

# Structs

Particles are spheres. They essentially contain the following information:

- mass
- radius
- moment of inertia 
- position vector $$s$$
- velocity vector $$v$$
- rotation matrix $$R$$
- angular velocity vector $$\omega$$ (right-hand rule, scaled to angular velocity in radians)

Contacts essentially contain the following information:

- a reference to the two particles, A and B. A and B both contain references to the contact, as well. 
- the position where it happened $$P$$
- the **contact normal** $$n$$, which is a vector from B to A, with length equal to the penetration depth
- a state value that says if this contact is stable or not

Every loop, we calculate our list of contacts. This, along with gravity calculation, are the most computationally expensive operations simply due to it being $$O(n^2)$$ operations, so they are offloaded onto the GPU.

# Calculating our list of contacts

This is a $$O(n^2)$$ operation that just checks if two particles are intersecting. If they are, it pushes a new contact onto the list `contacts`. We are currently working on parallelizing it on the GPU.

# n-body contact solving

Once we have our list of contacts `contacts`, to take care of the n-body case, we essentially do the following in pseudo code:

```
repeat contacts.size() times:
    for contact in contacts:
        contact.solve_momentum()
```
where `contact.solve_momentum()` takes care of the 2-body case (soon to be described below).

Solving contacts multiple times takes care of cases like the Newton's Cradle. This may seem inefficient in theory because if there's $$n$$ particles, there might be $$n^2$$ contacts and thus $$n^3$$ calls for `solve_momentum()`. However, in practice, most particles spend most of their time floating around freely so momentum calculations can be safely done on the CPU, and this step ends up only taking <10% of total processing time.

## Example

Suppose you have a bunch of balls in a line like this:
```
Velocity  |  > ....
Balls     |  o oooo
```

where `>`, `<`, and `.` are right, left, and zero respectively. We expect this situation to end up like this:
```
.... >
oooo o
```

So, by running it iteratively, we can propagate the momentum through the chain:

```
Before
   > ....
   o oooo

1.  >....
    ooooo

2.  .>...
    ooooo

3.  ..>..
    ooooo

4.  ...>.
    ooooo

5.  ....>
    ooooo

After
    .... >
    oooo o
```

The number of iterations could be tuned, or calculated in a smarter way (I was thinking, maybe calculate contacts within contiguous groups that are all touching each other to reduce the number of iterations) but it works well enough for our purposes.
