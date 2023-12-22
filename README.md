# AA3: Interaction

Authors: Adrià Pérez & Abel Pujol



## Instructions step-by-step
1) Adjust magnus effect with Z-X or by setting the slider with your mouse. By default, the ball is launched with a rightward effect. The more you increase it, the more it will go towards the left.
2) Charge the strength by holding space. When released, the scorpion will start moving. If you want to make this animation faster, you can lower "Max Time Animation" in the Route object through the inspector.
3) Once the scorpion reaches the ball, it will hit it with the strength you managed to get. The octopus will try to stop it but won't be able to reach some spots. If it succeeds, it will launch the ball outwards.
4) Reset the simulation by pressing R.

Note: During the ball's trajectory, a dotted path will appear showing its trajectory with and without magnus effect. This can be toggled by pressing I.

## Exercises:
1.1: MyOctopusController.cs

1.2: We didn't find a way to implement this in a way that makes sense for this assignment.

1.3: ForceCanvas.cs

1.4: Pressing R resets the simulation.

2.1-2.5: MovingBall.cs & RigidbodyState.cs (Magnus effect calculation: UpdateLinearMomentum() )

3.1-3.6: IK_Scorpion.cs & MyScorpionController.cs

4.1: ForceCanvas.cs & MyScorpionController.cs

4.2: MyScorpionController.cs

4.3: MyScorpionController.cs

5.1: MyOctopusController.cs

5.2: MovingBall.cs (& MyOctopusController.cs)

5.3: MyOctopusController.cs

## Rigidbody explanation:
We implemented a custom sphere rigidbody for this exercise. It stores mass, radius, linear momentum and angular momentum while updating the position and rotation of a transform given by a parameter.
Forces can be applied to it, which are defined by a force vector and a position. It also applies a toggleable magnus effect and gravity.

It updates linear momentum using the formula: ΔP = ∑F * dt and applies a set damping to it to simulate air friction.

It updates angular momentum using the formula: ΔL = τ * dt where τ = ∑(r x F) and also applies a set damping.

Position is updated using euler: x = x0 + (P / m) * dt given a mass m and linear momentum P.

Rotation is updated by finding the change in rotation from angular velocity: w = Inverse(iBody) * L given the inertia tensor of the sphere and angular momentum L.

The Magnus force direction is found by finding the cross product of angular and linear velocity (w x v), and its magnitude with the formula 4/3π * d * r^3 given an air density d and sphere radius r.
These are used since w x v finds the direction of the lift and we found that its magnitude can be represented by multiplying the volume of the object with the air resistance it encounters.
