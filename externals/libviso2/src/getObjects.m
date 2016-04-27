%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright 2013. All rights reserved.                                    %
% Institute of Measurement and Control Systems                            %
% Karlsruhe Institute of Technology, Germany                              %
%                                                                         %
% This file is part of triTrack2.                                         %
% Authors:  Philip Lenz                                                   %
%           Please send any bugreports to lenz@kit.edu                    % 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function obj = getObjects(params, Xc, Yc, Zc, tx, ty, tz, J, indStatic, indSingularity, indUnknown, cam)
% GETOBJECTS searches for objects in a graph structure

  if isempty(Xc)
    obj = {};
    return;
  end

  % Thresholds
  % thres.objMahal = 3;
  % thres.objDist  = 1.8;

  % only dynamic matches
  ind_d = ( (~indStatic & ~indSingularity) | indUnknown ) ;
  
  X_d = Xc(ind_d)';
  Y_d = Yc(ind_d)';
  Z_d = Zc(ind_d)';
  tx_d = tx(ind_d)';
  ty_d = ty(ind_d)';
  tz_d = tz(ind_d)';
  J_d = J(ind_d);

  try
    tri=DelaunayTri(X_d,Z_d);
  catch
    fprintf(2, 'Delauny Triangulation failed. Too less datapoints?\n');
    obj = {};
    return
  end
  
  % initalize evaluation
  szNodes=size(tri.X);
  N = zeros(szNodes(1),szNodes(1));   % preallocate neighborhood matrix N
  D = nan(szNodes(1),szNodes(1));   % preallocate neighborhood matrix N
  neighbors = edges(tri);             % get neighborhoods of triangulation

  % compare neighbors just once (1->2 and not again 2->1)
  for triNo=1:szNodes(1)
    % get current neighbors
    indNeighbors=neighbors(neighbors(:,1)==triNo,2);

    % 3D interest point distance
    triDist = sqrt((X_d(triNo) - X_d(indNeighbors)).^2 + ...
                   (Y_d(triNo) - Y_d(indNeighbors)).^2 + ...
                   (Z_d(triNo) - Z_d(indNeighbors)).^2);
    % 3D optical flow for neighbors
    flow1 = [tx_d(triNo), ty_d(triNo), tz_d(triNo)];
    flow2 = [tx_d(indNeighbors), ty_d(indNeighbors), tz_d(indNeighbors)]';

    % scene flow
    dx = (flow1(1)-flow2(1,:));
    dy = (flow1(2)-flow2(2,:));
    dz = (flow1(3)-flow2(3,:));
    d  = [dx; dy; dz];

    % measurement noise for current node V = J * sigma * J'
    flow1_noise = J_d{triNo}*J_d{triNo}';              % additive noise of flow (position difference)

    % compare all neighbors to current node
    for neighborsNo=1:numel(indNeighbors)
      % measurement noise for neighbor V = J * sigma * J'
      flow2_noise = J_d{indNeighbors(neighborsNo)} * J_d{indNeighbors(neighborsNo)}';
      noise = flow1_noise + flow2_noise;    % additive noise of flow (position difference)

      % distance between neighbor nodes
      dist=abs(triDist(neighborsNo));

      % compute mahalanobis distance of flow difference
      inv_noise=inv(noise);
      mahal_d=myMahal(inv_noise,d(:,neighborsNo));

      % mahalanobis thresholding and visualization of edge weights
      % nodes are connected -> green
      if(mahal_d<params.mahal_dist && dist<params.match_dist)
        N(triNo,indNeighbors(neighborsNo))=1;
        N(indNeighbors(neighborsNo), triNo)=1;
        D(triNo,indNeighbors(neighborsNo)) = mahal_d;
      end

    end %for neighborsNo
  end %for triNo

  obj = getConnected(N,D);

  if ~isempty(obj)
    for i=1:numel(obj)
      alpha = 0.2;
      [~, outlierX] = deleteoutliers(X_d(obj{i}.pts), alpha);
      [~, outlierY] = deleteoutliers(Y_d(obj{i}.pts), alpha);
      [~, outlierZ] = deleteoutliers(Z_d(obj{i}.pts), alpha);
      
      [~, outliertx] = deleteoutliers(tx_d(obj{i}.pts), alpha);
      [~, outlierty] = deleteoutliers(ty_d(obj{i}.pts), alpha);
      [~, outliertz] = deleteoutliers(tz_d(obj{i}.pts), alpha);
      
      outlier = unique(vertcat(outlierX, outlierY, outlierZ, outliertx, outlierty,outliertz));
      
      X = [X_d(obj{i}.pts), Y_d(obj{i}.pts), Z_d(obj{i}.pts)];
      t = [tx_d(obj{i}.pts), ty_d(obj{i}.pts), tz_d(obj{i}.pts)];
      % X(outlier,:)=[];
      % t(outlier,:)=[];
      
      % compute score
%       obj{i}.score = [];
%       for p=obj{i}.pts'
%         obj{i}.score = obj{i}.score + D(obj{
      
      % points, describing the object      
      obj{i}.X = X(:,1);
      obj{i}.Y = X(:,2);
      obj{i}.Z = X(:,3);
      % translation for every point
      obj{i}.tx = t(:,1);
      obj{i}.ty = t(:,2);
      obj{i}.tz = t(:,3);
      
      % mean does not consider, that most observations are in one corner of
      % the bounding box of the object
      % obj{i}.center.X = mean(obj{i}.X);
      % obj{i}.center.Y = mean(obj{i}.Y);
      % obj{i}.center.Z = mean(obj{i}.Z);
      % therefore, use mean of the bounding box
      obj{i}.center.X = (min(obj{i}.X)+max(obj{i}.X))/2;
      obj{i}.center.Y = (min(obj{i}.Y)+max(obj{i}.Y))/2;
%       obj{i}.center.Y = (min(obj{i}.Y));
      obj{i}.center.Z = (min(obj{i}.Z)+max(obj{i}.Z))/2;
      % translation for the object center
      obj{i}.center.tx = mean(obj{i}.tx);
      obj{i}.center.ty = mean(obj{i}.ty);
      obj{i}.center.tz = mean(obj{i}.tz);
      
      
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % compute bounding box for this object %
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      
%       Y        = -obj{i}.Y - tan(deg2rad(cam.mounting_pitch_deg))*obj{i}.Z + cam.height_over_ground;
%       Y_center = -obj{i}.center.Y - tan(deg2rad(cam.mounting_pitch_deg))*obj{i}.center.Z + cam.height_over_ground;
      
%       pos = [obj{i}.center.X, Y_center, obj{i}.center.Z];
%       pts = [obj{i}.X, Y, obj{i}.Z];
      pos = [obj{i}.center.X, obj{i}.center.Y, obj{i}.center.Z];
      pts = [obj{i}.X, obj{i}.Y, obj{i}.Z];
      t   = [ obj{i}.center.tx, obj{i}.center.ty, obj{i}.center.tz];
      % set object center as origin of CS
      pts_ocs = pts - repmat(pos,size(pts,1),1);
      
      % set 3D rotational matrix
      alpha = atan2( t(1),t(3) );
      R = [cos(alpha), 0, -sin(alpha);
           0,          1, 0;
           sin(alpha), 0, cos(alpha)];
      pts_rot = (R*pts_ocs')';
      
      % object dimensions
      % assuming, that the full object widht, height, length is visible, e.g. h = y_max-y_min
      % w = max(pts_ocs(:,1))-min(pts_ocs(:,1)); 
      % h = max(pts_rot(:,2))-min(pts_rot(:,2));
      % l = max(pts_ocs(:,3))-min(pts_ocs(:,3));
      % assuming, that the object is symmetric, e.g. width is 2*w/2 = 2*|w|_max
      obj{i}.dim.w = 2*max(abs(pts_rot(:,1)));
      obj{i}.dim.h = 2*max(abs(pts_rot(:,2)));
      obj{i}.dim.l = 2*max(abs(pts_rot(:,3)));
      
      % bounding box visualization for debugging
      % object coordinate system
      % figure(4);clf;
      % hold on
      % plot(0,0,'.r','markersize',20)
      % plot(pts_ocs(:,1), pts_ocs(:,3), '.')
      % quiver(0,0,t(1),t(3),0,'r')
      % plot(pts_rot(:,1), pts_rot(:,3), 'g.')
      % t_rot = R*t';
      % quiver(0,0,t_rot(1),t_rot(3),0,'g')
      % % rectangle('position',[-w/2,-l/2,w,l],'EdgeColor','g')
      % % bounding box for birdsview
      % bbox=[-w/2,  w/2,  w/2, -w/2, -w/2;
      %       -h/2, -h/2, -h/2, -h/2, -h/2;
      %       -l/2. -l/2,  l/2,  l/2, -l/2]';
      % plot(bbox(:,1),bbox(:,3),'g');
      % tmp = (R^-1*bbox')';
      % plot(tmp(:,1),tmp(:,3),'b');
      % 
      % % image coordinate system
      % figure(5);clf;
      % hold on
      % plot(pos(1), pos(3), '.r', 'markersize', 20)
      % plot(pts(:,1), pts(:,3), '.')
      % quiver(pos(1), pos(3), t(1), t(3), 0, 'r')
      % bbox_imgcs = (R^-1*bbox')' + repmat(pos,size(bbox,1),1);
      % plot(bbox_imgcs(:,1),bbox_imgcs(:,3),'g');
      
      obj{i}.ID    = nan;
      obj{i}.occur = 1;
    end
  end
  
end