classdef wbmDrawProp
    properties
		% joints.marker@char
		% joints.marker_sz@double scalar
		% joints.color
    	joints = struct( 'marker',    '', ...
						 'marker_sz', 0, ...
						 'color',     [] );

		% links.line_width@double scalar
		% links.color
		links = struct( 'line_width', 0, ...
					    'color',      [] );

		% com.marker@char
		% com.marker_sz@double   scalar
		% com.color
		com = struct( 'marker',    '', ...
					  'marker_sz', 0, ...
					  'color',     [] );

		% shape.line_width@double scalar
		% shape.edge_color
		% shape.face_color
		% shape.face_alpha@double scalar
		shape = struct( 'line_width', 0, ...
						'edge_color', [], ...
						'face_color', [], ...
						'face_alpha', 0 );
		ground_color
	end
end
