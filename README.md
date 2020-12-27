# Robust-Model-free-Iterative-Learning-Control-with-Convergence-Rate-Acceleration 
## Abstract
A novel model-free iterative learning control algorithm is presented in this project to improve both the robustness against output disturbances and the tracking performance in steady-state. For model-free ILC, several methods have been investigated, such as the time- reversal error filtering, the Model-Free Inversion-based Iterative Control (MFIIC), and the Non- Linear Inversion-based Iterative Control (NLIIC). However, the time-reversal error filtering has a conservative learning rate. Other two methods, although with much faster error convergence, have either a high noise sensitivity or a non-optimized steady-state. To improve the performance and robustness of model-free ILC, in this project we apply the time-reversal based ILC algorithm and recursively accelerate its error convergence using the online identified learning filter. The effectiveness of the proposed algorithm has been validated from a numerical simulation. The proposed approach not only improves the transient response of the MFIIC, but also achieves lower tracking error in steady-state compared to that of the NLIIC.

## Control Block Diagram

[!image](https://github.com/Geng-Hao/Robust-Model-free-Iterative-Learning-Control-with-Convergence-Rate-Acceleration/blob/master/AlgorithmBlockDiag.png)

## Algorithm Pseudo Code

[!image](https://github.com/Geng-Hao/Robust-Model-free-Iterative-Learning-Control-with-Convergence-Rate-Acceleration/blob/master/PseudoCode.png)
