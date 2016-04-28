%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright 2013. All rights reserved.                                    %
% Institute of Measurement and Control Systems                            %
% Karlsruhe Institute of Technology, Germany                              %
%                                                                         %
% This file is part of triTrack2.                                         %
% Authors:  Philip Lenz                                                   %
%           Please send any bugreports to lenz@kit.edu                    % 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [x_corrected, P_corrected] = kalman_correct(z, x_predicted, P_predicted, H, R)

  % correct state after measurement
  % gain K
  K = P_predicted*H'*((H*P_predicted*H'+R)^-1);
  % update state
  x_corrected = x_predicted + K*(z - H*x_predicted);
  % update error covariance
  P_corrected = (eye(size(P_predicted)) * K*H)*P_predicted;
  
end