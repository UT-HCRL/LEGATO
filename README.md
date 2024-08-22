# LEGATO: Cross-Embodiment Imitation Using a Grasping Tool
[Mingyo Seo](https://mingyoseo.com), [Andy Park](https://www.linkedin.com/in/robodreamer), [Shenli Yuan](https://yuanshenli.com), [Yuke Zhu](https://www.cs.utexas.edu/~yukez), [Luis Sentis](https://sites.google.com/view/lsentis)

## Abstract
Cross-embodiment imitation allows for policies trained on demonstrations from specific embodiments to be transferred to different robot embodiments, unlocking the potential for large-scale imitation learning that is both cost-effective and highly reusable.
This paper presents LEGATO, a cross-embodiment imitation learning framework for visuomotor skills, facilitating the transfer of actions across various kinematic morphologies. 
We introduce a hand-held gripper that enables tasks to be defined within the same gripper's action and observation spaces across different robots.
Based on this hand-held gripper, we train visuomotor policies through imitation learning, incorporating a motion-invariant transformation to compute the training loss. 
We then retarget gripper motions into high-DOF whole-body motions for deployment across diverse embodiments using inverse kinematics.
Our evaluation of simulations and real-robot experiments highlights the framework’s effectiveness in learning and transferring visuomotor skills across various robots.

## Code will be released after arXiv release.

