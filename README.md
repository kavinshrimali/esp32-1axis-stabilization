# esp32-1axis-stabilization
An ESP32-CAM-powered real-time single-axis stabilization platform that detects a reference line's tilt from the principal eigenvector of the image's scatter matrix and corrects for it via a servo. 

## Overview 
I started by building an IMU-based single-axis stabilization platform using an MPU6050, ESP32S, and SG90 Servo-Motor that used a complementary filter to stabilize a platform by taking the weighted average of fast-responding, but drifting, gyroscope-data and noisy, but stable, accelerometer data. Wanting to build a more technically-complex follow-up, I used the ESP32-CAM to derive tilt using the principles of computer vision rather than IMU data. 
1. The ESP32-CAM detects a reference line's tilt along the direction of the principal eigenvector of the image's scatter matrix.
2. The tilt is stabilized via a servo motor that counteracts the tilt angle calculated in the previous step.

## Tilt Angle Derivation
Computer vision relies on a scatter matrix in determining the axis along which an image is tilted. The scatter matrix arises from the following steps:
Let $v = \begin{bmatrix} \cos\theta \\ \sin\theta \end{bmatrix}$ be a unit direction vector.
Let $x_i = \begin{bmatrix} x_i \\ y_i \end{bmatrix}$ be the demeaned coordinate vector of a dark pixel.

To find the projection of $x_i$ along $v$, we compute their dot product:
$$
v \cdot x_i
$$

Computers determine the shape of objects by calculating their variance, which we compute as follows:
$$
(v \cdot x_i)^2 = v x_i x_i^T v^T
$$
This comes from the fact that the transpose of a scalar is equal to itself, so $(v \cdot x_i)^T$ can be rewritten as $x_i^T v^T$.

Summing over all dark pixels:
$$
\sum_i (v \cdot x_i)^2 = v \left( \sum_i x_i x_i^T \right) v^T = vMv^T
$$
Here, M denotes the Scatter Matrix.

Let us compute the product of $x_i$ and $x_i^T$:
$$
x_i x_i^T = \begin{bmatrix} x_i \\ y_i \end{bmatrix} \begin{bmatrix} x_i & y_i \end{bmatrix} = \begin{bmatrix} x_i^2 & x_iy_i \\y_ix_i & y_i^2 \end{bmatrix}
$$

Summing over all dark pixels to produce the Scatter Matrix, we have:
$$
M = \begin{bmatrix} \sum_i(x_i^2) & \sum_i(x_iy_i) \\ \sum_i(y_ix_i) & \sum_i(y_i^2) \end{bmatrix}
$$

Let: 
* $S_{xx} = \sum_i(x_i^2)$
* $S_{xy} = S_{yx} = \sum_i x_i y_i$
* $S_{yy} = \sum_i(y_i^2)$

We must now find the principal eigenvalue of the matrix M by first forming its characteristic polynomial:
$$
\begin{aligned}
\text{tr}(M) &= S_{xx} + S_{yy} \\
\det(M) &= S_{xx}S_{yy} - S_{xy}^2
\end{aligned}
$$

Given that the general form of the characteristic polynomial is $\lambda^2 - \text{tr}(M)\lambda + \det(M) = 0$ (with $\lambda$ representing the eigenvalues), we have:
$$
\lambda^2 - (S_{xx} + S_{yy})\lambda + (S_{xx}S_{yy} - (S_{xy})^2)
$$

By the quadratic formula, we have:
$$
\frac{(S_{xx} + S_{yy}) \pm \sqrt{(-(S_{xx} + S_{yy}))^2 - 4(1)(S_{xx}S_{yy} - (S_{xy})^2)}}{2(1)}
$$

Simplifying:
$$
\frac{(S_{xx} + S_{yy}) \pm \sqrt{S_{xx}^2 + 2S_{xx}S_{yy} + S_{yy}^2 - 4S_{xx}S_{yy} + 4(S_{xy})^2}}{2} = \frac{(S_{xx} + S_{yy}) \pm \sqrt{S_{xx}^2 + S_{yy}^2 - 2S_{xx}S_{yy} + 4(S_{xy})^2}}{2}
$$

Note that $S_{xx} + S_{yy}$ is positive and the term enclosed in the square root must be positive. So, the largest eigenvalue must be:
$$
\frac{(S_{xx} + S_{yy}) + \sqrt{S_{xx}^2 + S_{yy}^2 - 2S_{xx}S_{yy} + 4(S_{xy})^2}}{2}
$$

Let $a$ be the principal eigenvector of the general form $\begin{bmatrix} \cos\theta \\ \sin\theta \end{bmatrix}$.

Given that $Ma$ = Principal Eigenvalue * $a$, we have:
$$
\begin{aligned}
Ma &= \begin{bmatrix} \sum_i(x_i^2) & \sum_i(x_iy_i) \\ \sum_i(y_ix_i) & \sum_i(y_i^2) \end{bmatrix}\begin{bmatrix} \cos\theta \\ \sin\theta \end{bmatrix} \\
Ma &= \begin{bmatrix} S_{xx}\cos\theta + S_{xy}\sin\theta \\ S_{xy}\cos\theta + S_{yy}\sin\theta \end{bmatrix} \\
\lambda \cdot a &= \lambda\begin{bmatrix} \cos\theta \\ \sin\theta \end{bmatrix} &= \frac{(S_{xx} + S_{yy}) + \sqrt{S_{xx}^2 + S_{yy}^2 - 2S_{xx}S_{yy} + 4(S_{xy})^2}}{2}\begin{bmatrix} \cos\theta \\ \sin\theta \end{bmatrix} &= \begin{bmatrix} \left(\frac{((S_{xx} + S_{yy}) + \sqrt{S_{xx}^2 + S_{yy}^2 - 2S_{xx}S_{yy} + 4(S_{xy})^2})\cos\theta}{2}\right) \\ \left(\frac{((S_{xx} + S_{yy}) + \sqrt{S_{xx}^2 + S_{yy}^2 - 2S_{xx}S_{yy} + 4(S_{xy})^2})\sin\theta}{2}\right) \end{bmatrix}
\end{aligned}
$$

Letting $c$ be the principal eigenvalue to simplify the derivation, we equate both sides of the equation to arrive at the following equations:
$$
\begin{aligned}
S_{xx}\cos\theta + S_{xy}\sin\theta &= c\cos\theta \\
S_{xy}\cos\theta + S_{yy}\sin\theta &= c\sin\theta
\end{aligned}
$$

Multiplying the first equation by $\sin\theta$ and the second by $\cos\theta$ we have:
$$
\begin{aligned}
S_{xx}\cos\theta\sin\theta + S_{xy}(\sin\theta)^2 &= c\cos\theta\sin\theta \\
S_{xy}(\cos\theta)^2 + S_{yy}\sin\theta\cos\theta &= c\sin\theta\cos\theta
\end{aligned}
$$

We now equate the left hand-side of both equations:
$$
S_{xx}\cos\theta\sin\theta + S_{xy}(\sin\theta)^2 = S_{xy}(\cos\theta)^2 + S_{yy}\sin\theta\cos\theta
$$

Grouping like terms:
$$
(\cos\theta\sin\theta)(S_{xx} - S_{yy}) = S_{xy}((\cos\theta)^2 - (\sin\theta)^2)
$$

We now apply the following identities to further simplify the equation:
$$
\begin{aligned}
\cos\theta\sin\theta &= \frac{\sin\left(2\theta\right)}{2} \\
(\cos\theta)^2 - (\sin\theta)^2 &= \cos\left(2\theta\right)
\end{aligned}
$$

Applying these identities, we have:
$$
\frac{S_{xy}}{S_{xx} - S_{yy}} = \frac{\sin\left(2\theta\right)}{2\cos\left(2\theta\right)}
$$

Because $\frac{\sin(2\theta)}{\cos(2\theta)} = \tan\left(2\theta\right)$:
$$
\frac{S_{xy}}{S_{xx} - S_{yy}} = \frac{\tan\left(2\theta\right)}{2}
$$

Therefore:
$$
\theta = \frac{1}{2}\arctan\left(\frac{2S_{xy}}{S_{xx} - S_{yy}}\right)
$$

This is our tilt angle.
