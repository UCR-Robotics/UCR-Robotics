Kinematics and Control for Moblie Robot
====================
Review on State-space Model
--------

In control engineering, a `state-space representation`_ is a mathematical model of a physical system as a set of input, output and state variables related by first-order differential equations or difference equations. The values of state variables will evolve over time depends on the input variables.
It is a good tool (modelling method) to show how system variables (e.g., position, velcity, angle, ect.) are effected by internal or external inputs (e.g., forces, moments, etc.) over time.
More importantly, 
You could choose different subsets of system variables to represent the entire state of the system, 
while the smallest possible subset is the minimum number of state variables that could fully repersent the given physical system. 
In robotic systems, this minimum number are always related to the degree of freedoms of the robots. 
The states typically includes the configuration (position) and its derivative (velocity). 

.. _state-space representation: https://en.wikipedia.org/wiki/State-space_representation
   
   
- State-Space Modelling Steps

  - Given a set of differential equations (take a single variable q(t) as an example here)
  - Isolate the n-th highest derivative, :math:`q^{(n)} = g(q,\dot{q},\dots,q^{(n-1)},\mathbf{u})`
  - Set :math:`x_{1} = q(t)`, :math:`x_{2} = \dot{q}(t)`, :math:`\dots` , :math:`x_{n} = q^{(n-1)}(t)`.
  - Create state vector :math:`\mathbf{x} = [x_1,x_2,\dots,x_n]^T = [q, \dot{q},\dots,q^{(n-1)}]^T`
  - Rewrite these equations into a system of coupled first-order differential equations and (optional) rewrite in matrix form:
 
.. math::

    \begin{array}{l}
    \dot{x}_{1}=\dot{q}=x_{2} \\
    \dot{x}_{2}=\ddot{q}=x_{3} \\
    \dots \\
    \ddot{x}_{n}=q^{(n)}=g\left(q, \dot{q}, \ldots, q^{(n-1)}, \mathbf{u}\right)=g\left(x_{1}, x_{2}, \ldots, x_{n}, \mathbf{u}\right)
    \end{array} \to

.. math::

    \left [\begin{array}{l}
    \dot{x}_{1} \\
    \dot{x}_{2} \\
    \dots \\
    \dot{x}_{n}
    \end{array}\right]=
    \left [\begin{array}{l}
    x_{2} \\
    x_{3} \\
    \dots \\
    g\left(x_{1}, x_{2}, \ldots, x_{n}, \mathbf{u}\right)
    \end{array}\right]

Now we could express the evolution of the system use n states :math:`\mathbf{x} \in R^{n}` and m inputs :math:`\mathbf{u} \in R^{m}`.
    
  
Reviews on Kinematics
----------

- Forward Kinematics: To determine robot position (x, y) and orientation (:math:`\theta`) based on wheels rotation measurements.

- Inverse Kinematics: Callculate the joint coordinates (rotation measurements) to achieve the desired position and orientation.

Derive State-space Model for Mobile Robot
-----------------------------------------

- Differential drive model
- Unicycle Model

PID Controller

  
 

