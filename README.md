# Buck Converter Modeling and Control (Discrete PID)

This repository provides the **continuous and discrete modeling** of a
**Buck Converter**, the derivation of its **parametric discrete model**,
and the design of a **discrete PID controller** using MATLAB.

Mathematical derivations are based on control theory, while MATLAB code
is provided for simulation and implementation.

------------------------------------------------------------------------

## 📌 Continuous Model

Given the Buck Converter:

$$
P(s) = \frac{Y(s)}{U(s)} = \frac{\left(\frac{V_{in}}{LC}\right)}{s^2 + \left(\frac{1}{RC}\right)s + \left(\frac{1}{LC}\right)}
$$

We rewrite in standard second-order form:

$$
\frac{Y(s)}{U(s)} = \frac{K \omega_n^2}{s^2 + 2 \zeta \omega_n s + \omega_n^2}
$$

where:

$$
\omega_n = \sqrt{\frac{1}{LC}}, \quad
\zeta = \frac{\sqrt{LC}}{2RC}, \quad
K = V_{in}
$$

------------------------------------------------------------------------

## 📌 Discrete Model (Euler Explicit)

We discretize using:

$$
s = \frac{z-1}{T_s}
$$

So:

$$
P(z) = \frac{K \omega_n^2 T_s^2}{z^2 + z(2 \zeta \omega_n T_s - 2) + (\omega_n^2 T_s^2 - 2 \zeta \omega_n T_s + 1)}
$$

Defining parameters:

$$
\alpha = K \omega_n^2 T_s^2, \quad
\beta = 2 \zeta \omega_n T_s - 2, \quad
\gamma = \omega_n^2 T_s^2 - 2 \zeta \omega_n T_s + 1
$$

We get:

$$
P(z) = \frac{\alpha}{z^2 + \beta z + \gamma}
$$

------------------------------------------------------------------------

## 📌 Parametric Formulation

Rewritten for estimation:

$$
y(k) = -\beta y(k-1) - \gamma y(k-2) + \alpha u(k-2)
$$

Or compactly:

$$
y(k) = \Theta^T \Phi
$$

where:

$$
\Theta = 
\begin{bmatrix} \beta \\ \gamma \\ \alpha \end{bmatrix}, 
\quad
\Phi =
\begin{bmatrix} -y(k-1) \\ -y(k-2) \\ u(k-2) \end{bmatrix}
$$

------------------------------------------------------------------------

## 📌 Discrete PID Controller (Tustin Method)

Starting with the continuous PID:

$$
C(s) = K_p + \frac{K_i}{s} + K_d s
$$

Discretization (Tustin, $s = \frac{z-1}{zT_s}$) yields:

$$
C(z) = \frac{c_2 z^{-2} + c_1 z^{-1} + c_0}{1 - z^{-1}}
$$

We allow a tunable gain $\lambda$:

$$
C(z) = \frac{\lambda (c_2^* z^{-2} + c_1^* z^{-1} + c_0^*)}{1 - z^{-1}}
$$

## 📌 Pole-Zero Cancellation Design

Givne the plant as:

$$
P(z) = \frac{\alpha}{z^2 + \beta z + \gamma}
$$

and the controller:

$$
C(z) = \frac{\lambda N_c(z)}{D_c(z)}
$$

Pole-zero cancellation gives:

$$
c_2 = \gamma, \quad c_1 = \beta, \quad c_0 = 1
$$

Closed-loop system:

$$
G_{mf}(z) = \frac{\lambda \alpha}{z^2 - z + \lambda \alpha}
$$

------------------------------------------------------------------------

### 📌 Pole Placement Strategies

When designing the closed-loop system

$$
G_{mf}(z) = \frac{\lambda \alpha}{z^2 - z + \lambda \alpha},
$$

we want to choose the parameter $\lambda$ such that the closed-loop poles are placed at desired locations.  
There are **two main approaches**, depending on whether the poles are chosen as complex conjugates or as distinct real poles:

---

#### 1. Complex Conjugate Poles (Oscillatory Response)

If we want oscillatory behavior, the poles are set as complex conjugates:

$$
\begin{cases}
z_1 = A + Bi \\
z_2 = A - Bi
\end{cases}
$$

Substituting into the closed-loop polynomial:

$$
(z - z_1)(z - z_2) = z^2 - 2Az + (A^2 + B^2)
$$

Matching coefficients with:

$$
z^2 - z + \lambda \alpha,
$$

we obtain the conditions:

$$
A = 0.5, \quad \lambda = \frac{\sqrt{0.25 + B^2}}{\alpha}.
$$

👉 Interpretation:  
- The **real part** of the poles is fixed at $0.5$, which determines the decay rate.  
- The **imaginary part** $B$ adjusts the oscillation frequency.  
- The gain $\lambda$ is chosen based on the desired oscillatory dynamics.  

---

#### 2. Real Poles (Non-oscillatory Response)

If we prefer a purely exponential (non-oscillatory) closed-loop response, we place the poles at two distinct real locations:

$$
\begin{cases}
z_1 = A_1 \\
z_2 = A_2
\end{cases}
$$

so the polynomial is:

$$
(z - A_1)(z - A_2) = z^2 - (A_1 + A_2)z + A_1 A_2.
$$

Comparing with:

$$
z^2 - z + \lambda \alpha,
$$

the conditions are:

$$
A_1 + A_2 = 1, \quad A_1 A_2 = \lambda \alpha.
$$

👉 Interpretation:  
- The **sum of the poles** is always $1$, fixing their average location.  
- The **product of the poles** depends on $\lambda$, which directly shapes the stability and speed of convergence.  
- Choosing $A_1$ and $A_2$ close together yields slower but smoother dynamics, while separating them yields faster but potentially less balanced dynamics.  

---

This way, you can select **oscillatory** (complex) or **non-oscillatory** (real) dynamics depending on your control objective, and tune $\lambda$ accordingly.


------------------------------------------------------------------------

## 📌 ARMAX Model Extension

Including disturbance model:

$$
y(k) = -\beta y(k-1) -\gamma y(k-2) + \alpha u(k-2) + \xi(k) - \Omega \xi(k-1)
$$

where $\xi(k)$ is the prediction error.

Parametric form:

$$
\hat{y}(k) =
\begin{bmatrix} \beta & \gamma & \alpha & \Omega \end{bmatrix}^T
\begin{bmatrix} -y(k-1) \\ -y(k-2) \\ u(k-2) \\ -\xi(k-1) \end{bmatrix}
$$

------------------------------------------------------------------------

## 📌 MATLAB Implementation

### Time Vector

``` matlab
n_timesteps = 1000;
Tsamp = 1e-3;
t = (0:n_timesteps-1)*Tsamp;
```

### Plant Parameters

``` matlab
Vin = 12;
L = 1e-3;
C = 100e-6;
R = 10;

wn = sqrt(1/(L*C));
zeta = sqrt(L*C)/(2*R*C);
K = Vin;
```

### Discrete Model Coefficients

``` matlab
alpha = K * wn^2 * Tsamp^2;
beta  = 2*zeta*wn*Tsamp - 2;
gamma = wn^2*Tsamp^2 - 2*zeta*wn*Tsamp + 1;
```

### Simulation Loop

``` matlab
% Preallocate
y = zeros(1,n_timesteps);
u = ones(1,n_timesteps); % step input

for k = 3:n_timesteps
    y(k) = -beta*y(k-1) - gamma*y(k-2) + alpha*u(k-2);
end

plot(t,y)
xlabel('Time (s)'); ylabel('Output');
grid on;
```

------------------------------------------------------------------------

## 📌 Summary

-   Derived **continuous Buck model**\
-   Discretized via **Euler Explicit**\
-   Built **parametric model for estimation**\
-   Designed a **discrete PID controller with pole-zero cancellation**\
-   Extended to **ARMAX modeling**\
-   Provided **MATLAB code** for simulation

------------------------------------------------------------------------

## 📜 License

MIT License. Free to use and modify.
