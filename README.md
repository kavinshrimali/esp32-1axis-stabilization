# esp32-1axis-stabilization
An ESP32-CAM-powered real-time single-axis stabilization platform that detects a reference line's tilt from the principal eigenvector of the image's scatter matrix and corrects for it via a servo. 

## Overview 
I started by building an IMU-based single-axis stabilization platform using an MPU6050, ESP32S, and SG90 Servo-Motor that used a complementary filter to stabilize a platform by taking the weighted average of fast-responding, but drifting, gyroscope-data and noisy, but stable, accelerometer data. Wanting to build a more technically-complex follow-up, I used the ESP32-CAM to derive tilt using the principles of computer vision rather than IMU data. 
1. The ESP32-CAM detects a reference line's tilt along the direction of the principal eigenvector of the image's scatter matrix.
2. The tilt is stabilized via a servo motor that counteracts the tilt angle calculated in the previous step.

## Tilt Angle Derivation
Computer vision relies on a scatter matrix in determining the axis along which an image is tilted. The scatter matrix arises from the following steps:
Let $v = \begin{bmatrix} \cos\theta \\ \sin\theta \end{bmatrix}$ be a unit direction vector.
Let $x_i = \begin{bmatrix} x_i \\ y_i \end{bmatrix} be the demeaned coordinate vector of a dark pixel.

To find the projection of $x_i along $v, we compute their dot product:
$$
v \cdot x_i
$$

Computers determine the shape of objects by calculating their variance, which we compute as follows:
$$
(v \cdot x_i)^2 = v x_i x_i^T v^T
$$
This comes from the fact that the transpose of a scalar is equal to itself, so $(v \cdot x_i)^T can be rewritten as $x_i^T v^T.

Summing over all dark pixels:
$$
\sum_i (v \cdot x_i)^2 = v \left( sum_i x_i x_i^T \right) v^T = vMv^T
$$
Here, $M denotes the Scatter Matrix.

Let us compute the product of $x_i and $x_i^T:
$$
x_i x_i^T = \begin{bmatrix} x_i \\ y_i \end{bmatrix} \begin{bmatrix} x_i & y_i \end{bmatrix} = \begin{bmatrix} x_i^2 & x_iy_i \\y_ix_i & y_i^2 \end{bmatrix}
$$

Summing this over all dark pixels to produce the Scatter Matrix, we have:
$$
M = \begin{bmatrix} \sum_i(x_i^2) & \sum_i(x_iy_i) \\ \sum_i(y_ix_i) & \sum_i(y_i^2) \end{bmatrix}
$$

Let: 
* Sxx = $\sum_i(x_i^2)
* Sxy = Syx = $\sum_i(x_i*y_i)
* Syy = $\sum_i(y_i^2)

We must now find the principal eigenvalue of the matrix M by first forming its characteristic polynomial:
$$
tr(M) = Sxx + Syy
det(M) = SxxSyy - SxySyx = SxxSyy - (Sxy)^2
$$
$$
Given that the general form of the characteristic polynomial is \lambda^2 - tr(M)*\lambda + det(M) (with \lambda representing the eigenvalues), we have:
\lambda^2 - (Sxx + Syy)\lambda + (SxxSyy - (Sxy)^2)
$$
By the quadratic formula, we have:
$$
\frac{(Sxx + Syy) \pm \sqrt((-(Sxx + Syy))^2 - 4(1)(SxxSyy - (Sxy)^2)){2(1)}
$$
Simplifying:
$$
\frac{(Sxx + Syy) \pm \sqrt(Sxx^2 + 2SxxSyy + 2Syy^2 - 4SxxSyy + 4(Sxy)^2}{2}
\frac{(Sxx + Syy) \pm \sqrt(Sxx^2 + 2yy^2 -2SxxSyy + 4(Sxy)^2)}{2}
$$
$$
Note that Sxx + Syy is positive and the term enclosed in the square root must be positive. So, the largest eigenvalue must be:
\frac{(Sxx + Syy) + \sqrt(Sxx^2 + 2yy^2 -2SxxSyy + 4(Sxy)^2)}{2}
$$
Let $a be the principal eigenvector of the general form $\begin{bmatrix} \cos\theta \\ \sin\theta \end{bmatrix}.
Given that $Ma = Principal Eigenvalue * $a, we have:
$$
Ma = \begin{bmatrix} \sum_i(x_i^2) & \sum_i(x_iy_i) \\ \sum_i(y_ix_i) & \sum_i(y_i^2) \end{bmatrix}\begin{bmatrix} \cos\theta \\ \sin\theta \end{bmatrix}

Ma = \begin{bmatrix} Sxx\cos\theta + Sxy\sin\theta \\ Sxy\cos\theta + Syy\sin\theta \end{bmatrix}

\lambda \cdot a = \lambda\begin{bmatrix} \cos\theta \\ \sin\theta \end{bmatrix} = \frac{(Sxx + Syy) + \sqrt(Sxx^2 + 2yy^2 -2SxxSyy + 4(Sxy)^2)}{2}\begin{bmatrix} \cos\theta \\ \sin\theta \end{bmatrix} = \begin{bmatrix} \left(\frac{((Sxx + Syy) + \sqrt(Sxx^2 + 2yy^2 -2SxxSyy + 4(Sxy)^2))\cos\theta}{2}\right) \\ \left(\frac{((Sxx + Syy) + \sqrt(Sxx^2 + 2yy^2 -2SxxSyy + 4(Sxy)^2))\sin\theta}{2}\right) \end{bmatrix}
$$
Equating both sides, we arrive at the following equations:





5. Squaring this dot product gives us the variance - in other words, how much pixels vary from a defined 'center'. Given that computers don't have eyes, they can only determine an object's shape by relying on each pixel's variance from this center. Doing so yields the following: [INSERT EXPRESSION HERE]
6. Focusing on the outer product between the 2 vectors for the pixels, and summing across all pixels in the image, we arrive at a 2 x 2 matrix with the following structure [Sxx in the top left, Sxy in teh top right, Syx in the bottom left, and Syy in the bottom right]. This matrix is known as the 'scatter matrix'.
7. The principal eigenvector of this matrix (i.e. the matrix with the largest associated eignevalue) defines the principal direction along which the image is tilted. To find the principal eigenvector, we start by deriving ths scatter matrix's principal eigenvalue. To do so, we first calculate det(Scatter Matrix) (which is Sxx*Syy - Sxy*Syx = Sxx*Syy - Sxy^2 (since Sxy = Syx)) and tr(Scatter Matrix) (which is Sxx + Syy).
8. The characteristic polynomial is as follows: lambda^2 - tr(Scatter Matrix)*lambda + det(Scatter Matrix) = 
lambda^2 - 
