%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright 2013. All rights reserved.                                    %
% Institute of Measurement and Control Systems                            %
% Karlsruhe Institute of Technology, Germany                              %
%                                                                         %
% This file is part of triTrack2.                                         %
% Authors:  Philip Lenz                                                   %
%           Please send any bugreports to lenz@kit.edu                    % 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [ u, v ] = project3dToImage( X,Y,Z,cam,camID )
% project3dToImage Summary of this function goes here
%   Detailed explanation goes here

if(~(exist('camID','var')))
    camID='left';
end

switch lower(camID)
    case 'left'
        u=cam.f*X./Z + cam.cu;
        v=-cam.f*Y./Z + cam.cv;
    case 'right'
        u=cam.f*(X+cam.base)./Z + cam.cu;
        v=-cam.f*Y./Z + cam.cv;
    otherwise
        error('Undefined camID. Use left or right.');
end

end

