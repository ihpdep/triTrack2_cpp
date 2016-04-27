%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright 2013. All rights reserved.                                    %
% Institute of Measurement and Control Systems                            %
% Karlsruhe Institute of Technology, Germany                              %
%                                                                         %
% This file is part of triTrack2.                                         %
% Authors:  Philip Lenz                                                   %
%           Please send any bugreports to lenz@kit.edu                    % 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [ varargout ] = computeTranslation(pts3d, pts2d, cam)
% COMPUTEVELOCITY ONLY FOR ONE TIME HISTORY STEP

  for itVar=1:numel(pts3d)
    % compute translation
    t  = pts3d{itVar}(2,:) - pts3d{itVar}(1,:);
    varargout{itVar} = t;
    clear pts t
    
    % compute Jacobian of translation
    if(nargout==4 && itVar==1)
      u1=pts2d{1}; u2=pts2d{2}; v=pts2d{3};
      J_cell=cell(1,size(u1,2));
      for i=1:size(u1,2)
        t1 = u2(2,i) - u1(2,i);
        t2 = 1 / t1;
        t4 = t1 ^ 2;
        t5 = 0.1e1 / t4;
        t6 = (u1(2,i) - cam.cu) * t5;
        t8 = u2(1,i) - u1(1,i);
        t9 = 1 / t8;
        t11 = t8 ^ 2;
        t12 = 0.1e1 / t11;
        t13 = ((u1(1,i) - cam.cu) * t12);
        t16 = ((cam.cv - v(2,i)) * t5);
        t18 = ((cam.cv - v(1,i)) * t12);
        t19 = (cam.f * t5);
        t20 = (cam.f * t12);
        J(1,1) = t2 + t6;
        J(1,2) = -t6;
        J(1,3) = 0;
        J(1,4) = -t9 - t13;
        J(1,5) = t13;
        J(1,6) = 0;
        J(2,1) = t16;
        J(2,2) = -t16;
        J(2,3) = -t2;
        J(2,4) = -t18;
        J(2,5) = t18;
        J(2,6) = t9;
        J(3,1) = t19;
        J(3,2) = -t19;
        J(3,3) = 0;
        J(3,4) = -t20;
        J(3,5) = t20;
        J(3,6) = 0;
        J_cell{i}=J;
      end
    end % if
  end % for itVar
  if(nargout==4), varargout{4} = J_cell; end
end