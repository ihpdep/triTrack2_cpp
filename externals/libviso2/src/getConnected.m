%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright 2013. All rights reserved.                                    %
% Institute of Measurement and Control Systems                            %
% Karlsruhe Institute of Technology, Germany                              %
%                                                                         %
% This file is part of triTrack2.                                         %
% Authors:  Philip Lenz                                                   %
%           Please send any bugreports to lenz@kit.edu                    % 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [ obj ] = getConnected( N,D )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
    
    % init return value
    obj = {};
    % get connected comonents of the graph
    N=sparse(N);
    % components is part of matlab_bgl
    idx=components_mex(N); % matlab_bgl
    %[trash, idx] = graphconncomp(N); % built-in
    count=1;
    for it=1:max(idx)
        % object == connected graph with 5 or more nodes
        if(sum(idx==it)>=3)
            p = find(idx==it);
            obj{count}.pts = p;
            s = [];
            for i=p'
              s = [s,D(i,p(p>i))];
            end
            obj{count}.score = nanmean(s);
            count=count+1;
        end
    end
end

