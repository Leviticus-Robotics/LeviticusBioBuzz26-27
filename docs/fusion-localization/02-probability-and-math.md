# Probability and estimation mathematics

This document assumes familiarity with basic probability and linear algebra, but not with Kalman filters.

## Gaussian state representation

The estimator represents the state as

\[
X \sim \mathcal N(\mu, P),
\]

where:

- \(\mu\) is the state mean.
- \(P\) is the covariance.
- A diagonal entry \(P_{ii}\) is the variance of coordinate \(i\).
- An off-diagonal entry \(P_{ij}\) is the covariance between coordinates \(i\) and \(j\).

For localization:

\[
\mu = [x,y,\theta,v_x,v_y,\omega]^T.
\]

Correlations are operationally important. If pose and velocity are correlated, an accurate pose observation can also correct velocity.

`Gaussian` stores the mean/covariance form. `InformationGaussian` stores the equivalent information form:

\[
\Lambda=P^{-1},\qquad \eta=\Lambda\mu.
\]

Independent Gaussian information adds naturally, which motivates the factor update.

## Factors as likelihoods

A factor defines a residual

\[
r_i(X)
\]

and covariance \(R_i\). It contributes the likelihood

\[
\phi_i(X) \propto
\exp\left(-\frac12r_i(X)^TR_i^{-1}r_i(X)\right).
\]

The posterior is proportional to the prior multiplied by every factor likelihood:

\[
p(X\mid Z)\propto p(X)\prod_i\phi_i(X).
\]

A small covariance means the factor contributes more information. A large covariance means it contributes less.

## Linearization

Pose and motion residuals are nonlinear. Around the current mean \(\mu\), the engine uses the first-order approximation

\[
r_i(\mu+\delta)\approx r_i(\mu)+H_i\delta,
\]

where

\[
H_i=\left.\frac{\partial r_i}{\partial X}\right|_{\mu}.
\]

Pose factors provide analytic Jacobians. `ProcessModel` provides a central-difference Jacobian because process models are usually cheap to evaluate and this avoids maintaining hand-derived transition math. A future process model may override it analytically if profiling demonstrates a worthwhile benefit.

## Simultaneous information-form correction

For a prior \(\mathcal N(\mu,P)\), the locally linearized posterior information matrix is

\[
\Lambda^+=P^{-1}+\sum_iH_i^TR_i^{-1}H_i.
\]

The posterior covariance is

\[
P^+=(\Lambda^+)^{-1}.
\]

With the code's residual convention, the corrected mean is

\[
\mu^+=\mu-P^+\sum_iH_i^TR_i^{-1}r_i(\mu).
\]

`GaussianUpdater.applyAll()` implements this batch. All factors for a tick are linearized at the same prior mean and accumulated before the state changes. Consequently, reordering independent factors within that batch does not change the result, apart from floating-point roundoff.

This is different from applying factors one at a time and relinearizing after each one, which can make a nonlinear update order-dependent.

## Prediction and the two-state joint Gaussian

Let the previous posterior be

\[
X_{k-1}\sim\mathcal N(\mu_{k-1},P_{k-1}).
\]

A process model predicts

\[
X_k=f(X_{k-1},u_k)+w_k,\qquad w_k\sim\mathcal N(0,Q_k).
\]

Linearize the transition with

\[
F_k=\frac{\partial f}{\partial X_{k-1}}.
\]

Rather than first creating a special predicted Gaussian, the transition becomes the ordinary relative-factor residual

\[
r_u(X_{k-1},X_k)=X_k-f(X_{k-1},u_k),
\qquad H_u=[-F_k\;I].
\]

For the augmented state \(Z=[X_{k-1},X_k]\), the engine begins with information only on the old-state block:

\[
\Lambda_0=\begin{bmatrix}P_{k-1}^{-1}&0\\0&0\end{bmatrix}.
\]

It then adds the process, OctoQuad, AprilTag, and any other factor information in the same way. Solving this augmented system produces the same process-induced cross-covariance that the conventional EKF prediction equations imply, while avoiding any special status for prediction.

## Absolute factors

An AprilTag factor constrains the current pose directly. Its residual is

\[
r_A(X_k)=
\begin{bmatrix}
x_k-x_A\\
y_k-y_A\\
\operatorname{wrap}(\theta_k-\theta_A)
\end{bmatrix}.
\]

Its Jacobian selects the pose coordinates from the six-dimensional state:

\[
H_A=
\begin{bmatrix}
1&0&0&0&0&0\\
0&1&0&0&0&0\\
0&0&1&0&0&0
\end{bmatrix}.
\]

When this factor is applied to the joint state, its Jacobian is placed in the current-state columns.

## Relative factors

Let \(T(X)\) denote the planar pose in a state. The motion implied by two fused states is

\[
\widehat{\Delta X}_k=T(X_{k-1})^{-1}\oplus T(X_k).
\]

For a measured delta \(\Delta O_k\), the code uses

\[
r_O(X_{k-1},X_k)
=\operatorname{coords}\left(
\Delta O_k^{-1}\oplus\widehat{\Delta X}_k
\right).
\]

Here \(\oplus\) means planar pose composition, and the inverse expresses the error relative to the measurement. The factor supplies one Jacobian block for \(X_{k-1}\) and another for \(X_k\). The engine concatenates them into a Jacobian over the joint state.

The implementation performs valid local-coordinate Gaussian estimation, but `coords` is not the full \(SE(2)\) logarithm. Translation is represented directly and heading is wrapped. This is generally appropriate for small local corrections, but it is not a manifold-optimization implementation.

## Marginalization

After all constraints are applied, the engine has a corrected Gaussian over

\[
[X_{k-1},X_k].
\]

The current marginal is obtained by retaining the current mean and lower-right covariance block:

\[
p(X_k)=\int p(X_{k-1},X_k)\,dX_{k-1}.
\]

`GaussianUpdater.marginal()` performs this operation. The resulting posterior becomes the prior for the next tick.

## Count each observation once

Every independent likelihood belongs in the product once. Submitting the same process transition or sensor observation twice would square that likelihood and artificially reduce uncertainty. Factor `observationId()` values provide source-specific deduplication; process factors need no separate rule.

## Numerical constraints

The implementation:

- Rejects non-finite vectors and matrices.
- Requires symmetric positive-definite covariance and information matrices.
- Symmetrizes computed covariance matrices to remove floating-point asymmetry.
- Uses linear solves to implement inversion.
- Normalizes state coordinates, especially heading, after prediction and correction.

Positive definite is stricter than positive semidefinite. Every factor therefore needs nonzero uncertainty in each residual dimension.

