%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright 2013. All rights reserved.                                    %
% Institute of Measurement and Control Systems                            %
% Karlsruhe Institute of Technology, Germany                              %
%                                                                         %
% This file is part of triTrack2.                                         %
% Authors:  Philip Lenz                                                   %
%           Please send any bugreports to lenz@kit.edu                    % 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [ indSingularity, indUnknown ] = graphSmoothing(Xc, Zc, indStatic, indUnknown, p_matched)
% GRAPHSMOOTHING filters single matches out of the graph

  % thresholds
  thres.matchDistAbs = 0.7;
  thres.matchDistRatio = 0.8;

  thres.farMatch = 30;
  thres.farMatchDistAbs = 5;

  thres.pixelDist = 30;
  
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  % smoothing in world domain

  % only (probably) dynamic matches
  X_d = Xc(~indStatic | indUnknown)';
  Z_d = Zc(~indStatic | indUnknown)';
  
  % indices of original matches
  ind_all = 1:numel(Xc);
  ind_all = ind_all(~indStatic | indUnknown);
  
  indSingularity = false(1,numel(Xc));

  tri=DelaunayTri(X_d,Z_d);
  
  % initalize evaluation
  szNodes=size(tri.X);
  neighbors = edges(tri);             % get neighborhoods of triangulation

  % compare neighbors just once (1->2 and not again 2->1)
  for triNo=1:szNodes(1)
    % get current neighbors
    indNeighbors=neighbors(neighbors(:,1)==triNo,2);

    % 3D interest point distance
    triDist_2d = sqrt((X_d(triNo) - X_d(indNeighbors)).^2 + ...
                      (Z_d(triNo) - Z_d(indNeighbors)).^2);
                                        
    if((sum(triDist_2d > thres.matchDistAbs)/numel(triDist_2d) > thres.matchDistRatio) && Z_d(triNo) < thres.farMatch)
      indSingularity(ind_all(triNo))=true;
      indUnknown(ind_all(triNo))=false; % a singularity is not an unknown match
%     elseif(all(triDist_2d > thres.farMatchDistAbs))
%       indSingularity(ind_all(triNo))=true;
%       indUnknown(ind_all(triNo))=false;
    end
  end %for triNo
  
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  % smoothing in image domain
  
  ind_d = ( (~indStatic & ~indSingularity) | indUnknown);

  u_d = p_matched(5,ind_d)';
  v_d = p_matched(6,ind_d)';
  % indices of original matches
  ind_all = 1:numel(Xc);
  ind_all = ind_all((~indStatic & ~indSingularity) | indUnknown );
  triImage=DelaunayTri(u_d,v_d);
  % initalize evaluation
  szNodes=size(triImage.X);
  neighbors = edges(triImage);             % get neighborhoods of triangulation
  % compare neighbors just once (1->2 and not again 2->1)
  for triNo=1:szNodes(1)
    % get current neighbors
    indNeighbors=neighbors(neighbors(:,1)==triNo,2);
                                        
    imageDist = sqrt((u_d(triNo) - u_d(indNeighbors)).^2 + ...
                     (v_d(triNo) - v_d(indNeighbors)).^2);
    
    if(all(imageDist>thres.pixelDist))
      indSingularity(ind_all(triNo))=true;
      indUnknown(ind_all(triNo))=false; % a singularity is not an unknown match
    end
    
  end %for triNo