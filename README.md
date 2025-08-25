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

To cancel the plant's poles, the controller can be set as:

$$
c_2 = \gamma, \quad c_1 = \beta, \quad c_0 = 1
$$

The resulting closed-loop system is:

$$
G_{mf}(z) = \frac{\lambda \alpha}{z^2 - z + \lambda \alpha}
$$

The controller is designed based on the estimated model of the plant. As the estimation improves, the observed closed-loop response approaches the expected one.

------------------------------------------------------------------------

### 📌 Pole Placement Strategies and Close-Loop behavior design

When designing the closed-loop system

$$
G_{mf}(z) = \frac{\lambda \alpha}{z^2 - z + \lambda \alpha},
$$

we want to choose the parameter $\lambda$ such that the closed-loop poles are placed at desired locations.  
There are **two main approaches**, depending on whether the poles are chosen as complex conjugates or as distinct real poles:



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

## 📌 MATLAB Implementation Overview

The MATLAB code provided simulates a Buck Converter controlled with a discrete PID, while performing online parameter estimation. The plant simulation is performed using the function `simu_buck_linear`, which is an encrypted function provided as an **exercise in the "Identification of Dynamic Systems" classes** of the **PPGESE (Postgraduate Program in Embedded and Electronic Systems) at Universidade Federal de Santa Catarina (UFSC)**. This function models the Buck Converter dynamics, allowing students to focus on controller design and parameter estimation without seeing the internal plant equations.

1. **Simulation Setup**
   - Define total simulation time (`simu_Time`) and sampling time (`Tsamp`).
   - Compute number of timesteps and initialize the time vector.

2. **Controller Setup**
   - Define desired closed-loop poles (`A1`, `A2`) for pole placement.
   - Compute initial tunable gain `lambda = A1*A2`.
   - Important: This sets the starting behavior of the PID controller.

3. **Initial System Estimation**
   - Initialize plant parameters (`vin_0`, `R_0`, `L_0`, `C_0`) and compute initial discrete model (`theta`) using `get_theta_fromSys`.
   - Initialize the covariance matrix `P` for recursive estimation.
   - Important: Correct initialization ensures stable convergence of the estimator.

4. **Buffers and Plant Initialization**
   - Prepare history buffers for outputs (`y_buffer`), control inputs (`u_buffer`), and reference signals (`ref_buffer`).
   - Initialize plant states, e.g., inductor current `iL` and initial voltage `v0`.
   - Important: Buffers are used to handle past samples required by the discrete parametric model.

5. **Main Simulation Loop**
   - For each timestep:
     - Generate reference signal with `get_reference`.
     - Compute controller parameters using `get_controller`.
     - Compute control input via `get_control_signal`.
     - Simulate plant using `sim_buck_linear`.
     - Update buffers with new measurements and control input.
     - **Parameter Estimation:** Update `theta` and covariance `P` using `update_theta_P` (only during training).
     - Compute predicted output `y_pred_evolution` and residuals `y_resi_evolution`.
     - Convert discrete parameters back to continuous plant parameters for monitoring (`get_Cont_fromTheta`, `get_plant_params`).
   - Important: This loop integrates control, plant simulation, and online parameter estimation in real-time.

6. **Visualization**
   - **Figure 1:** System output vs reference and control input.
   - **Figure 2:** Normalized evolution of estimated parameters (`theta`) and system parameters (Vin, L, R).
   - **Figure 3:** Estimation residuals and their FFT for frequency analysis.
   - **Figure 4:** Histogram comparing residuals during training vs validation.
   - **Figure 5:** Trace of the covariance matrix over time.
   - Important: Visualizations allow verifying controller performance, estimator convergence, and signal quality.

7. **Helper Functions**
   - `get_reference` – Generates step or ramp reference signals.
   - `update_theta_P` – Updates model parameters recursively (RLS-like).
   - `get_controller` – Computes PID coefficients from estimated parameters.
   - `get_control_signal` – Computes control action using PID law and history buffers.
   - `get_theta_fromSys` – Converts initial plant parameters into discrete model form.
   - `get_Cont_fromTheta` – Converts discrete parameters to continuous plant representation.
   - `get_plant_params` – Extracts Vin, L, R from continuous parameters.
   - `get_Phi` – Builds the regressor vector for parametric prediction.
   - `sim_buck_linear` – Simulates discrete-time Buck converter linear model.
   - Important: These functions modularize the code, making it easy to modify or extend.

**Key Highlights:**
- The code demonstrates a **closed-loop system** with both **control** and **online parameter estimation**.
- Buffers and regressor vectors are critical for handling the discrete-time parametric model.
- The estimation stops updating during validation, allowing comparison of predicted vs actual behavior.
- Pole placement and `lambda` tuning control the **dynamic response** of the closed-loop system.

