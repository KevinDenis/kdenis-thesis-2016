%> @file "predictor.m"
%> @authors Ivo Couckuyt
%> @version 1.4 ($Revision$)
%> @date $LastChangedDate$
%> @date Copyright 2010-2013
%>
%> This file is part of the ooDACE toolbox
%> and you can redistribute it and/or modify it under the terms of the
%> GNU Affero General Public License version 3 as published by the
%> Free Software Foundation.  With the additional provision that a commercial
%> license must be purchased if the ooDACE toolbox is used, modified, or extended
%> in a commercial setting. For details see the included LICENSE.txt file.
%> When referring to the ooDACE toolbox please make reference to the corresponding
%> publications:
%>   - Blind Kriging: Implementation and performance analysis
%>     I. Couckuyt, A. Forrester, D. Gorissen, F. De Turck, T. Dhaene,
%>     Advances in Engineering Software,
%>     Vol. 49, pp. 1-13, July 2012.
%>   - Surrogate-based infill optimization applied to electromagnetic problems
%>     I. Couckuyt, F. Declercq, T. Dhaene, H. Rogier, L. Knockaert,
%>     International Journal of RF and Microwave Computer-Aided Engineering (RFMiCAE),
%>     Special Issue on Advances in Design Optimization of Microwave/RF Circuits and Systems,
%>     Vol. 20, No. 5, pp. 492-501, September 2010. 
%>
%> Contact : ivo.couckuyt@ugent.be - http://sumo.intec.ugent.be/?q=ooDACE
%> Signature
%>	[y, or1, or2, dmse] = predictor(points, krige)
%
% ======================================================================
%> @brief  Calculates prediction of a kriging model
%>
%> DACE toolbox compatible interface to ooDACE (wrapper)
%>
%> @param points input points matrix
%> @param krige kriging model
%> @retval y prediction values
%> @retval or1 
%> @retval or2
%> @retval dmse
% ======================================================================
function  [y, or1, or2, dmse] = predictor(points, krige)

if nargin == 0
    disp('Usage: [y, or1, or2, dmse] = predictor(points, krige)');
    return;
end

% Default return values
or1 = NaN;
or2 = NaN;
dmse = NaN;

[mx nx] = size(points);

if  mx == 1  % one point
    
    if  nargout > 2  % MSE wanted
        [y or2] = krige.predict(points);
    else
        y = krige.predict(points);
    end
    
    if  nargout > 3  % gradient/Jacobian of MSE wanted
        [or1 dmse] = krige.predict_derivatives(points);
    elseif nargout > 1
        or1 = krige.predict_derivatives(points);
    end
    
else  % several points
    
    
    if  nargout > 1   % MSE wanted
        [y or1] = krige.predict(points);
        
        if  nargout > 2
            disp('WARNING from PREDICTOR.  Only  y  and  or1=mse  are computed')
        end
    else
        y = krige.predict(points);
    end
    
end

end
