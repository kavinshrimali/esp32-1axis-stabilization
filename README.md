# esp32-1axis-stabilization
An ESP32-CAM-powered real-time single-axis stabilization platform that detects a reference line's tilt from the principal eigenvector of the image's scatter matrix and corrects for it via a servo. 

## Overview 
I started by building an IMU-based single-axis stabilization platform using an MPU6050, ESP32S, and SG90 Servo-Motor that used a complementary filter to stabilize a platform by taking the weighted average of fast-responding, but drifting, gyroscope-data and noisy, but stable, accelerometer data. Wanting to build a more technically-complex follow-up, I used the ESP32-CAM to derive tilt using the principles of computer vision rather than IMU data. 
1. The ESP32-CAM detects a reference line's tilt along the direction of the principal eigenvector of the image's scatter matrix.
2. The tilt is stabilized via a servo motor that counteracts the tilt angle calculated in the previous step.

## Tilt Angle Derivation
Computer vision relies on a scatter matrix in determining the axis along which an image is tilted. The scatter matrix arises from the following steps:
Let $v = [\cos\theta, \; \sin\theta]^T$ be a unit direction vector.
Let $x_i = [x_i, \; y_i]^T$ be the demeaned coordinate vector of a dark pixel.

To find the projection of $x_i$ along $v$, we compute their dot product:

$$
v \cdot x_i
$$

To determine the principal direction of the dark pixels, we seek the direction $v$ along which their projected coordinates have the greatest variance:

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
x_i x_i^T = \begin{bmatrix} x_i \\\\ y_i \end{bmatrix} \begin{bmatrix} x_i & y_i \end{bmatrix} = \begin{bmatrix} x_i^2 & x_i y_i \\\\ y_i x_i & y_i^2 \end{bmatrix}
$$

Summing over all dark pixels to produce the Scatter Matrix, we have:

$$
M = \begin{bmatrix} \sum_i(x_i^2) & \sum_i(x_i y_i) \\\\ \sum_i(y_i x_i) & \sum_i(y_i^2) \end{bmatrix}
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

Let $a = [\cos\theta, \; \sin\theta]^T$ be the principal eigenvector.

Given that $Ma = \lambda a$ (where $\lambda$ is the principal eigenvalue), we have:

$$
\begin{aligned}
Ma &= \begin{bmatrix} S_{xx} & S_{xy} \\\\ S_{xy} & S_{yy} \end{bmatrix} \begin{bmatrix} \cos\theta \\\\ \sin\theta \end{bmatrix} = \begin{bmatrix} S_{xx}\cos\theta + S_{xy}\sin\theta \\\\ S_{xy}\cos\theta + S_{yy}\sin\theta \end{bmatrix} \\\\
\lambda a &= \begin{bmatrix} \lambda\cos\theta \\\\ \lambda\sin\theta \end{bmatrix}
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

## Derivation of Algebraic Properties to Limit Per-Frame Processing Time

To avoid a second pass over the pixel array and reduce per-frame processing time, I used the following algebraic identities to compute $S_{xx}$, $S_{yy}$, and $S_{xy}$ directly from raw sums, without needing the mean calculated in advance:

To calculate $S_{xx}$, we sum the product of the x-component of all the demeaned (dark) pixels. In other words, we are taking the following sum: $\sum_i((x_i - \bar{x})^2)$. This sum can be simplified as follows:

$$
\begin{aligned}
S_{xx} &= \sum_i(x_i^2 - 2x_i\bar{x} + \bar{x}^2) \\
S_{xx} &= \sum_i(x_i^2) - 2\bar{x}\sum_i(x_i) + \sum_i(\bar{x}^2) \\
\end{aligned}
$$

We can re-write $\sum_i(\bar{x}^2)$ as $n\bar{x}^2$. 
Since $\bar{x} = \frac{\sum_i(x_i)}{n}$, $\sum_i(x_i) = n\bar{x}$.

Hence, we have:

$$
\begin{aligned}
S_{xx} &= \sum_i(x_i^2) - 2n\bar{x}^2 + n\bar{x}^2 = \sum_i(x_i^2) - n\bar{x}^2
\end{aligned}
$$

I applied the same property in calculating $S_{yy}$. 

To calculate $S_{xy}$, we sum the product of the x and y-components of all the demeaned (dark) pixels. In other words, we are taking the following sum: $\sum_i((x_i - \bar{x})(y_i - \bar{y}))$. This sum can be simplified as follows:

$$
\begin{aligned}
S_{xy} &= \sum_i(x_iy_i - x_i\bar{y} - \bar{x}y_i + \bar{x}\bar{y}) \\ 
&= \sum_i(x_iy_i) - \bar{y}\sum_i(x_i) - \bar{x}\sum_i(y_i) + \sum_i(\bar{x}\bar{y}) \\
&= \sum_i(x_iy_i) - \bar{y}\sum_i(x_i) - \bar{x}\sum_i(y_i) + n\bar{x}\bar{y} \\
\end{aligned}
$$

Substituting $\sum_i(x_i) = n\bar{x}$ and $\sum_i(y_i) = n\bar{y}$:

$$
\begin{aligned}
S_{xy} &= \sum_i(x_iy_i) - n\bar{y}\bar{x} - n\bar{x}\bar{y} + n\bar{x}\bar{y} \\
&= \sum_i(x_iy_i) - n\bar{x}\bar{y}
\end{aligned}
$$

## Key Design Decisions
* **Enabling PSRAM:** Enabling PSRAM allowed for smoother continuous image processing due to the ESP32-CAM's ability to process 2 images at the same time, allowing for smoother stabilization. Using a PSRAM-enabled module also allowed me to use VGA framesize for the images (640 x 480), allowing the camera to process larger images and thus more accurately determine the reference line's tilt. 
* **Power Supply Buffering (100 µF Capacitor):** I incorporated a 100 µF capacitor to prevent sudden current spikes due to activation of the servo motor from triggering a brownout that would cause the ESP32-CAM to reset.
* **Independent Servo Power Supply:** Initially, I connected the servo motor to the 5V pin of the ESP32-CAM, but this triggered brownouts due to the current spikes when the servo was stabilizing the tilt. To solve this issue, I powered the servo motor using a power supply made up of 4 AA batteries on a separate breadboard, ensuring the 2 breadboards in use had a common `GND`.
* **Single-Pass Computation:** Using the algebraic identities proved above, I was able to halve the per-frame processing time, reducing latency in the stabilization.
* **`LOOP_DELAY` Tuning:** I empirically measured per-cycle processing time via `millis()` logging to reduce latency.

## Bill of Materials 
* AI-Thinker ESP32-CAM
* SG90 Servo Motor
* ESP32-CAM-MB (FTDI Programmer Shield)
* 100 μF Electrolytic Capacitor
* 4* AA Batteries
* Battery Holder
* Breadboard and Jumper Cables

## Wiring Guide

### ESP32-CAM ↔ FTDI Programmer
| ESP32-CAM Pin | FTDI Pin | Notes |
|---|---|---|
| 5V | 5V | |
| GND | GND | |
| U0R | TX | |
| U0T | RX | |
| GPIO0 | GND | Only during flashing - disconnect before running |

### Servo Motor
| Servo Wire | Connects To | Notes |
|---|---|---|
| Signal (orange/yellow) | GPIO14 | |
| Power (red) | Battery pack `+` rail | Separate rail from ESP32-CAM's 5V |
| Ground (brown) | Battery pack `-` rail | Bridged to ESP32-CAM's GND — see below |

### Power Domains
| Component | Power Source | Ground |
|---|---|---|
| ESP32-CAM | FTDI 5V (via laptop USB) | Breadboard A `-` rail |
| Servo Motor | 4×AA battery pack (6V) | Breadboard B `-` rail |

**Important:** Rail A and Rail B grounds must be bridged with a jumper wire because the two power domains are electrically separate on `+`, but need a shared ground for GPIO14's signal to work correctly.

## Build and Flash Configuration

* **Board:** AI Thinker ESP32-CAM
* **Flash Frequency:** 80 MHz
* **Flash Mode:** QIO
* **Partition Scheme:** HUGE APP (3MB No OTA/1MB SPIFFS)
* **PSRAM:** Enabled
* **Baud Rate:** 115200
