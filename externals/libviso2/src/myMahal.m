%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright 2013. All rights reserved.                                    %
% Institute of Measurement and Control Systems                            %
% Karlsruhe Institute of Technology, Germany                              %
%                                                                         %
% This file is part of triTrack2.                                         %
% Authors:  Philip Lenz                                                   %
%           Please send any bugreports to lenz@kit.edu                    % 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function dist = myMahal(inv_sigma,vec)
% MYMAHAL computes mahalanobis distance
%   Mahalanobis distance d(x,y)=sqrt( (x-y)' S^-1 (x-y) ) for difference of
%   given vectors x and y (vec=x-y) and covariance matrix sigma
%
%   Example:
%       dist = myMahal( sigma^(-1), (x-y) )

    t1   = inv_sigma*vec;
    dist = vec'*t1;
    
    % sqrt only takes computational time
    % dsq=vec'*t1;
    % dist=sqrt(dsq);
end