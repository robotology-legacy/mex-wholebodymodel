function [Mx, aJcMinv, aJc_t] = cartmass(a_J_c, M)
    %% Cartesian mass matrix for redundant manipulators.
    %
    %  Sources:
    %   [1] Inertial Properties in Robotic Manipulation: An Object-Level Framework, Oussama Khatib, Volume 14, Issue 1,
    %       International Journal of Robotics Research (IJRR), 1995, <https://cs.stanford.edu/group/manips/publications/pdfs/Khatib_1995.pdf>,
    %       p. 8, eq. (17), p. 9.
    %   [2] A unified approach for motion and force control of robot manipulators: The operational space formulation, Oussama Khatib,
    %       IEEE Journal on Robotics and Automation, 1987, Volume 3, Issue 1, <https://cs.stanford.edu/group/manips/publications/pdfs/Khatib_1987_RA.pdf>,
    %       p. 49-50, eq. (51).
    %   [3] Cartesian Impedance Control of Redundant Robots: Recent Results with the DLR-Light-Weight-Arms, A. Albu-Schäffer & C. Ott & U. Frese & G. Hirzinger,
    %       IEEE International Conference on Robotics and Automation (ICRA), 2003, Volume 3, <http://www.robotic.de/fileadmin/robotic/ott/papers/albu_ott_icra2003.pdf>,
    %       p. 2, eq. (12).
    %   [4] Control of Generalized Contact Motion and Force in Physical Human-Robot Interaction, E. Magrini & F. Flacco & A. De Luca,
    %       IEEE International Conference on Robotics and Automation (ICRA), 2015, <http://www.dis.uniroma1.it/~labrob/pub/papers/ICRA15_ContactForceMotionControl.pdf>,
    %       p. 2301, eq. (24).
    %   [5] Shared Control for Natural Motion and Safety in Hands-on Robotic Surgery, J. G. Petersen, PhD, Imperial College London, 2015,
    %       <https://spiral.imperial.ac.uk/bitstream/10044/1/45398/1/Petersen-J-2016-PhD-Thesis.pdf>, p. 55. eq. (3.3).
    %   [6] Robot Kinematics and Dynamics, Herman Bruyninckx, Lecture Notes, Katholieke Universiteit Leuven, 2010,
    %       <https://people.mech.kuleuven.be/~bruyninc/tmp/HermanBruyninckx-robotics.pdf>, p. 97, eq. (5.43) & (5.44).
    %   [7] Control Strategies for Robots in Contact, J. Park, PhD-Thesis, Artificial Intelligence Laboratory,
    %       Department of Computer Science, Stanford University, 2006, <http://cs.stanford.edu/group/manips/publications/pdfs/Park_2006_thesis.pdf>,
    %       Chapter 5, pp. 109-110, eq. (5.6) & (5.10).
    %   [8] The Moore-Penrose Pseudoinverse. A Tutorial Review of the Theory, J. C. A. Barata & M. S. Hussein, Brazilian Journal of Physics, Springer,
    %       Volume 42, Issue 1-2, 2012, <https://arxiv.org/abs/1110.6882v1>, pp. 146-165.
    n = size(a_J_c,1);
    aJc_t   = a_J_c.';
    aJcMinv = a_J_c / M; % x*M = a_J_c --> x = a_J_c * M^(-1)
    Upsilon = aJcMinv * aJc_t; % inverse mass matrix Upsilon = Lambda^(-1) = a_J_c * M^(-1) * a_J_c^T
                               % in operational space {C} (Lambda^(-1) ... inverse pseudo-kinetic energy matrix).
    Mx = Upsilon \ eye(n,n); % Cartesian mass matrix Mx = Lambda = (a_J_c * M^(-1) * a_J_c^T)^(-1)
                             % in operational space {C} (Lambda ... pseudo-kinetic energy matrix).
end
