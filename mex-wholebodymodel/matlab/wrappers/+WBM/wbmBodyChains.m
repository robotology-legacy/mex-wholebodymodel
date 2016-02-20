classdef wbmBodyChains
    properties(SetAccess=private, GetAccess=public)
       chains@cell    matrix = {};
       nChains@uint16 scalar = 0;
    end

    methods
    	function obj = wbmBodyChains(chain_names, vec_pos)
    		if ( ~iscell(chain_names) || ~ismatrix(vec_pos) )
    			error('wbmBodyChains::wbmBodyChains: %s', WBM.wbmErrMsg.WRONG_DATA_TYPE);		    			
    		end

    		[m, n] = size(vec_pos);
    		if (n ~= 2)
    			error('wbmBodyChains::wbmBodyChains: %s', WBM.wbmErrMsg.WRONG_MAT_DIM);		
    		end
    		if (size(chain_names,1) ~= m)
    			error('wbmBodyChains::wbmBodyChains: %s', WBM.wbmErrMsg.DIM_MISMATCH);
    		end
    		
    		obj.nChains = m;
    		obj.chains = horzcat(chain_names, num2cell(vec_pos));
    	end

    	function tbl = toTable(obj)
    		tbl = cell2table(obj.chains);
    	end

	end
end
