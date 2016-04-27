%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright 2013. All rights reserved.                                    %
% Institute of Measurement and Control Systems                            %
% Karlsruhe Institute of Technology, Germany                              %
%                                                                         %
% This file is part of triTrack2.                                         %
% Authors:  Philip Lenz                                                   %
%           Please send any bugreports to lenz@kit.edu                    % 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [pts, obj_center] = objectToWorld(pts, track, Tr)
% OBJECTTOWORLD projects 3D points from an object centered coordinate
% system to the world coordinate system

  % prepare rotation of boxes
  % rotation angle: transform velocity to image coordinate system
  t     = Tr(1:3,1:3)^-1 * [track.kalman.t(1,end), track.kalman.t(2,end), track.kalman.t(3,end)]';
  alpha = atan2( t(1),t(3) );
  % set 3D rotational matrix
  R = [cos(alpha), 0, -sin(alpha);
       0,          1, 0;
       sin(alpha), 0, cos(alpha)];
     
  % mounting geometry of the stereo rig must be considered
  % bounding box must be determined with respect to the ground plane
  % pitch angle must be corrected
  % not nice -> use an estimated groundplane instead!
  obj_center      = track.kalman.pos; % get tracked object center
  obj_center      = Tr^-1*[obj_center;ones(1,size(obj_center,2))]; % transform object center track in current image coordinate system
  obj_center(2,:) = -obj_center(2,:);

  pts = (R^-1*pts')' + repmat([obj_center(1,end), obj_center(2,end), obj_center(3,end)], size(pts,1),1);

end