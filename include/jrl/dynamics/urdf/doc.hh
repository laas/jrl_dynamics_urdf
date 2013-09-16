/**
\mainpage

\b jrl_dynamics_urdf is a jrl-dynamics loader for URDF files.

\section codeapi Code API

The entry point of this package is the jrl::dynamics::urdf::Parser
class. Once the object is instantiated (the constructor does not take
any argument), the parse method can be called with the URDF filename
and the root free-floating point name. This will return a jrl-dynamics
humanoid robot that can be used for dynamics computation.

For additional details regarding how to realize dynamics computation,
see the jrl-dynamics documentation directly.

*/
