function [pos, dcm] = frame2posrot(vqT)
    % FRAME2POSROT converts a vector-quaternion transfromation (vector-frame) into an individual
    % position vector and the corresponding rotation matrix, the direction cosine matrix (DCM).
    %
    %   INPUT ARGUMENTS:
    %       vqT -- (7 x 1) VQ-Transformation (vector-frame) either from the whole body model's state,
    %              or from an arbitrary link/frame of the system. (*)
    %
    %   OUTPUT ARGUMENTS:
    %       pos -- (3 x 1) position vector (x,y,z) of the vector-frame
    %       dcm -- (3 x 3) rotation matrix (DCM)
    %
    %   (*) Transformation vector with 3D-postion followed by the quaternion with the scalar (real)
    %       part as first.
    %
    % Author: Naveen Kuppuswamy (naveen.kuppuswamy@iit.it) - modified from matlab toolbox source; Genova, Dec 2015
    % Modified by: Martin Neururer (martin.neururer@gmail.com); Genova, Jan 2017
    pos  = vqT(1:3);
    quat = vqT(4:7); % assuming quat = [q_sc; q_vec]

    dcm = quaternion2dcm(quat);
end
