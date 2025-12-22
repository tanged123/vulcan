# Equations of Motion
**Source:** *Methods - Equations of Motion* (Pages 2-36 to 2-37)

## 1. Fundamental Principle (Inertial Frame)
The equations of motion are derived from Newton's second law applied to a point mass. In an inertial coordinate system, the acceleration $\vec{a}_{I}$ is defined as:

$$\vec{a}_{I} = \frac{\Sigma\vec{F}}{m}$$

* [cite_start]$\Sigma\vec{F}$: Sum of all external forces [cite: 5]
* [cite_start]$m$: Vehicle mass [cite: 5]
* [cite_start]Reference Equation: **(2-97)** [cite: 6]

---

## 2. Transformation to Earth-Fixed Coordinates
[cite_start]For trajectory analysis, it is convenient to define acceleration in terms of an Earth-Fixed coordinate system ($\vec{a}_{\oplus}$) which rotates relative to the inertial system at the earth's spin rate $\vec{\omega}_{\oplus}$[cite: 7].

### Kinematic Derivation
The inertial acceleration is the second time derivative of the position vector $\vec{r}$ with respect to the inertial frame:

$$\vec{a}_{I} = \frac{^I d^2\vec{r}}{dt^2} = \frac{^I d}{dt} \left( \frac{^I d\vec{r}}{dt} \right)$$

**First Derivative (Velocity):**
Differentiating the position vector relative to the rotating Earth-fixed frame introduces a tangential velocity term:
$$\frac{^I d\vec{r}}{dt} = \frac{^e d\vec{r}}{dt} + (\vec{\omega}_{\oplus} \times \vec{r}) = \vec{V}_{\oplus} + (\vec{\omega}_{\oplus} \times \vec{r})$$

**Second Derivative (Acceleration):**
Differentiating again with respect to the inertial frame applies the transport theorem to the entire velocity vector:
$$\vec{a}_{I} = \frac{^e d}{dt} \left[ \vec{V}_{\oplus} + \vec{\omega}_{\oplus} \times \vec{r} \right] + \vec{\omega}_{\oplus} \times \left[ \vec{V}_{\oplus} + \vec{\omega}_{\oplus} \times \vec{r} \right]$$

[cite_start]Assuming the earth's spin rate $\vec{\omega}_{\oplus}$ is constant (its time derivative is zero)[cite: 21], the expansion simplifies to:

$$\vec{a}_{I} = \vec{a}_{\oplus} + 2(\vec{\omega}_{\oplus} \times \vec{V}_{\oplus}) + \vec{\omega}_{\oplus} \times (\vec{\omega}_{\oplus} \times \vec{r})$$

* [cite_start]$\vec{a}_{\oplus}$: Acceleration relative to the earth ($\frac{^e d^2\vec{r}}{dt^2}$) [cite: 25]
* [cite_start]$2(\vec{\omega}_{\oplus} \times \vec{V}_{\oplus})$: **Coriolis Acceleration** [cite: 35]
* [cite_start]$\vec{\omega}_{\oplus} \times (\vec{\omega}_{\oplus} \times \vec{r})$: **Centrifugal Acceleration** [cite: 35]
* [cite_start]Reference Equation: **(2-104)** [cite: 37]

---

## 3. Final Earth-Relative Equations of Motion
By substituting the kinematic expansion back into Newton's law (Equation 2-97), we isolate the earth-relative acceleration $\vec{a}_{\oplus}$:

$$\vec{a}_{\oplus} = \frac{\Sigma\vec{F}}{m} - 2(\vec{\omega}_{\oplus} \times \vec{V}_{\oplus}) - \vec{\omega}_{\oplus} \times (\vec{\omega}_{\oplus} \times \vec{r})$$

* [cite_start]Reference Equation: **(2-105)** [cite: 39]

### Differential System for Integration
[cite_start]For numerical implementation (specifically in the `derivs` module [cite: 45]), this second-order equation is written as a set of coupled first-order differential equations:

1.  **Velocity Derivative:**
    $$\dot{\vec{r}}_{\oplus} = \vec{V}_{\oplus}$$

2.  **Acceleration Derivative:**
    $$\dot{\vec{V}}_{\oplus} = \frac{\Sigma\vec{F}}{m} - 2(\vec{\omega}_{\oplus} \times \vec{V}_{\oplus}) - \vec{\omega}_{\oplus} \times (\vec{\omega}_{\oplus} \times \vec{r})$$

* [cite_start]Reference Equation: **(2-106)** [cite: 43]

### Implementation Notes
* [cite_start]**Inputs required:** Initial position $\vec{r}$, earth-relative velocity $\vec{V}_{\oplus}$, mass $m$, and total forces $\Sigma\vec{F}$[cite: 44].
* **Coordinate System:** The derivation assumes an Earth-Centered, Earth-Fixed (ECEF/ECFC) frame where the rotation vector $\vec{\omega}_{\oplus}$ is typically aligned with the Z-axis.