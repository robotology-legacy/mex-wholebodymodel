classdef (Abstract) vbObject < handle & matlab.mixin.Heterogeneous
    properties(Abstract, Dependent)
        origin@double vector % position of the origin (x,y,z) of the object
        rotm@double   matrix % rotation matrix (x,y,z) of the object
        tform@double  matrix % transformation matrix of the object
        frame@double  vector % VQ-transformation (pos. & orientation) of the object
    end

    properties(Constant)
        DF_FRAME@double    vector = [0; 0; 0; 1; 0; 0; 0];
    end

    properties(Abstract)
        line_width@double  scalar
        edge_color
        face_color
        face_alpha@double  scalar % transparency of the fill area
        description@char          % annotation for the object
        ismovable@logical  scalar
    end

    properties(Abstract, SetAccess = private, GetAccess = public)
        obj_type@char             % type of the object (obstacle (obs) or volume body (bdy))
        isobstacle@logical scalar % defines if the cuboid is an obstacle
        issolid@logical    scalar % defines if the object is solid
        istool@logical     scalar % defines if the object is a tool
        init_frame@double  vector % initial frame (pos. & orientation) of the object
        dimension@double   vector % dimension (width, length & height) of the object
        com@double         vector % position of the object's center of mass (CoM)
        rho@double         scalar % volumetric mass density of the object
        m_rb@double        scalar % mass of the object (rigid body)
        I_cm@double        matrix % inertia of the object at CoM
        mgrid@struct              % internal 3D-meshgrid of the object (only for obstacles)
    end

    methods(Abstract)
        setInitFrame(obj, varargin)
        hgo    = getGObj(obj)
        hgo    = updGObj(obj, hgo)
        hmg    = drawMGrid(obj, pt_color)
        result = ptInObj(obj, pt_pos)
    end

    methods(Static, Sealed, Access = protected)
        function default_object = getDefaultScalarElement()
            default_object = WBM.vbCuboid;
        end

    end
end
