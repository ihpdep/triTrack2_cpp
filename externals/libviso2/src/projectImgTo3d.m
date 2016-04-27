%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright 2013. All rights reserved.                                    %
% Institute of Measurement and Control Systems                            %
% Karlsruhe Institute of Technology, Germany                              %
%                                                                         %
% This file is part of triTrack2.                                         %
% Authors:  Philip Lenz                                                   %
%           Please send any bugreports to lenz@kit.edu                    % 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [X,Y,Z,jacobian] = projectImgTo3d(u1,u2,v,cam)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here


  % projection geometry
  % Zw = b*f/d
  % mu_w = (Zw - [0,0,1]*C)/([0,0,1]*R'*K^(-1)*xc);
  % Xw = C + mu_w * R' * K^(-1) * xc;

  assert(all(size(u1)==size(v)) & all(size(u2)==size(v)),'u1, u2 and v must have the same dimension');
  
  if isempty(u1)
    X = []; Y = []; Z = []; jacobian = {};
    return;
  end

  d = u2-u1;

  % X = zeros(numel(u1),1);
  % Y = zeros(numel(u1),1);
  % Z = zeros(numel(u1),1);

  % row U (x), column V (y); image coordinate system
  X = (u1-cam.cu).*cam.base./d;
  Y = (cam.cv-v).*cam.base./d;
  Z = cam.f*cam.base./d;

  % Jacobian
  if(nargout==4)
    jacobian=cell(numel(u1),1);
    for i=1:numel(u1)
      jacobian_( 1, 1 ) = (cam.base*u2(i) - cam.base*cam.cu)./(u2(i)^2 - 2*u1(i)*u2(i) + u1(i)^2);
      jacobian_( 2, 1 ) = (cam.base*v(i) - cam.base*cam.cv)./(u2(i)^2 - 2*u1(i)*u2(i) + u1(i)^2);
      jacobian_( 3, 1 ) = cam.base*cam.f./(u2(i)^2 - 2*u1(i)*u2(i) + u1(i)^2);

      jacobian_( 1, 2 ) = 0;
      jacobian_( 2, 2 ) = cam.base / d(i);
      jacobian_( 3, 2 ) = 0;

      jacobian_( 1, 3 ) = -(cam.base*u1(i) - cam.base*cam.cu)/(u2(i)^2 - 2*u1(i)*u2(i) + u1(i)^2); 
      jacobian_( 2, 3 ) = -(cam.base*v(i) - cam.base*cam.cv)/(u2(i)^2 - 2*u1(i)*u2(i) + u1(i)^2); 
      jacobian_( 3, 3 ) = -cam.base*cam.f/(u2(i)^2 - 2*u1(i)*u2(i) + u1(i)^2); 
      jacobian{i}=jacobian_;
    end
  end
  
%   % Jacobian
%   if(nargout==4)
%     jacobian=cell(numel(u1),1);
%     for i=1:numel(u1)
%       jacobian_( 1, 1 ) = (cam.T*u2(i) - cam.T*cam.cx)./(u2(i)^2 - 2*u1(i)*u2(i) + u1(i)^2);
%       jacobian_( 2, 1 ) = (cam.T*v(i) - cam.T*cam.cy)./(u2(i)^2 - 2*u1(i)*u2(i) + u1(i)^2);
%       jacobian_( 3, 1 ) = cam.T*cam.fx./(u2(i)^2 - 2*u1(i)*u2(i) + u1(i)^2);
% 
%       jacobian_( 1, 2 ) = 0;
%       jacobian_( 2, 2 ) = cam.T / d(i);
%       jacobian_( 3, 2 ) = 0;
% 
%       jacobian_( 1, 3 ) = -(cam.T*u1(i) - cam.T*cam.cx)/(u2(i)^2 - 2*u1(i)*u2(i) + u1(i)^2); 
%       jacobian_( 2, 3 ) = -(cam.T*v(i) - cam.T*cam.cy)/(u2(i)^2 - 2*u1(i)*u2(i) + u1(i)^2); 
%       jacobian_( 3, 3 ) = -cam.T*cam.fx/(u2(i)^2 - 2*u1(i)*u2(i) + u1(i)^2); 
%       jacobian{i}=jacobian_;
%     end
%   end
end %function projectImgTo3D
