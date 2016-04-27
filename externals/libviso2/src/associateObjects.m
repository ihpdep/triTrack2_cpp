%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright 2013. All rights reserved.                                    %
% Institute of Measurement and Control Systems                            %
% Karlsruhe Institute of Technology, Germany                              %
%                                                                         %
% This file is part of triTrack2.                                         %
% Authors:  Philip Lenz                                                   %
%           Please send any bugreports to lenz@kit.edu                    % 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [obj, track, newID] = associateObjects(last_obj, obj, track, newID, minOccurence, Tr_total, k, cam)

  % parameter for bbox-dimension Kalman-Filter
  kf.bbox.A     = eye(3); % x=[w,h,l]'
  kf.bbox.H     = eye(3);
  kf.bbox.Q     = eye(3)*1e-3; % 1e-3
%   kf.bbox.Q     = [1e-3 0 0; 0 1e-2 0; 0 0 1e-2]; % 1e-3
  kf.bbox.R     = eye(3)*1e1; % 1e1
  kf.bbox.P_init = zeros(3);
  
  % parameter for object-movement Kalman-Filter
  kf.pos.A      = [1 0 0 1 0 0;
                   0 1 0 0 1 0;
                   0 0 1 0 0 1;
                   0 0 0 1 0 0;
                   0 0 0 0 1 0;
                   0 0 0 0 0 1];
  kf.pos.H      = eye(6); 
  kf.pos.Q      = (eye(6)*5e-2).^2; 
  kf.pos.R      = (eye(6)*0.15).^2;
  kf.pos.P_init = zeros(6);

  % maximum tracking distance
  maxDist      = 2;
  maxCenterHeight = -1.5;
  maxDist_quad = 50;

  % compute distance to all previous objects for every current object's point
  % create empty matrix describing the neighborhood last_obj.pts ↓ / obj.pts →
  distToLast      = nan(numel(last_obj), numel(obj));
  distToLast_quad = nan(numel(last_obj), numel(obj));
  ptsToLast       = nan(numel(last_obj), numel(obj));

  % get number of associated points and object distances
  for i=1:numel(obj)
    for j=1:numel(last_obj)
      
      % TODO: Use predicted position for association instead of last object
      % used to use my_knnsearch, which produced different results!
      [n, d] = knnsearch([obj{i}.X, obj{i}.Y, obj{i}.Z],...
                        [last_obj{j}.X, last_obj{j}.Y, last_obj{j}.Z],'k',1);
      % compute the median distance between the points describing to object
      % using the median since sometimes one outlier (e.g. street lamps) belongs to one meaningful object
      distToLast(j,i)      = median(d);
      distToLast_quad(j,i) = sum(d.^2);
      
      % compute the number of points that can be associated
      % intersect returns the points that are equal in both vectors
      % e.g. n=[1 1 1] and ptsNo=[1 2 3] -> intersect(n, ptsNo)=1
      ptsToLast(j,i) = numel(intersect(n,1:numel(last_obj{j}.pts)))/numel(last_obj{j}.pts);
    end
  end
  distToLast(distToLast>maxDist)      = nan;
  % distToLast_quad(distToLast_quad>maxDist_quad) = nan;
  
  % not nice: allow only one associated current object per last object
  distToLast(distToLast ~= repmat(min(distToLast,[],2),1,size(distToLast,2)) & ~isnan(distToLast))=nan;

  for i=1:numel(obj)
    ind = find(distToLast(:,i)==min(distToLast(:,i)), 1);
    center_height_over_ground = obj{i}.center.Y(end) + tan(deg2rad(cam.mounting_pitch_deg))*obj{i}.center.Z(end) - cam.height_over_ground;
    
    % ind = find(distToLast_quad(:,i)==min(distToLast_quad(:,i)), 1);
    % min_height_over_ground = obj{i}.center.Y + tan(deg2rad(cam.mounting_pitch_deg))*obj{i}.center.Z - cam.height_over_ground;

    % there is a neighbor (last_obj(ind) and the object is not flying somewhere
    if(~isempty(ind) && center_height_over_ground > maxCenterHeight)
      
      
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % object is already tracked %
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      
      if(last_obj{ind}.occur > minOccurence-1) % last object occurs one time less than current object
        % copy ID
        obj{i}.ID = last_obj{ind}.ID;
        
        % continue object track
        % save current object center position
        % project translation to global coordinate system
        tmp = Tr_total{end} * [obj{i}.center.X(end), obj{i}.center.Y(end), obj{i}.center.Z(end), 1]';
        track{obj{i}.ID}.raw.pos(:,end+1) = tmp(1:3);
        track{obj{i}.ID}.obj_id = i;
        
        % project translation to global coordinate system
        track{obj{i}.ID}.raw.t(:,end+1) = ...
          Tr_total{end}(1:3,1:3)*[obj{i}.center.tx(end), obj{i}.center.ty(end), obj{i}.center.tz(end)]';
        
        % detections in image coordinate system
        track{obj{i}.ID}.detections.pos = [obj{i}.X, obj{i}.Y, obj{i}.Z]';
        track{obj{i}.ID}.detections.t   = [obj{i}.tx, obj{i}.ty, obj{i}.tz]';
        
        % set current frame of track
        track{obj{i}.ID}.frame(1,end+1) = k;

        [pos posP] = kalman_predict([track{obj{i}.ID}.kalman.pos(:,end); track{obj{i}.ID}.kalman.t(:,end)],...
                                    track{obj{i}.ID}.kalman.posP{end}, kf.pos.A, kf.pos.Q);
        z_pos = [track{obj{i}.ID}.raw.pos(:,end); track{obj{i}.ID}.raw.t(:,end)];
        [pos posP] = kalman_correct(z_pos, pos, posP, kf.pos.H, kf.pos.R);
        track{obj{i}.ID}.kalman.pos(:,end+1) = pos(1:3);
        track{obj{i}.ID}.kalman.t(:,end+1)   = pos(4:6);
        track{obj{i}.ID}.kalman.posP{end+1}  = posP;
        
        % track bounding box for track
        [dim dimP] = kalman_predict(track{obj{i}.ID}.kalman.dim(:,end), track{obj{i}.ID}.kalman.dimP{end}, kf.bbox.A, kf.bbox.Q);
        z_dim = [obj{i}.dim.w(end), obj{i}.dim.h(end), obj{i}.dim.l(end)]';
        [dim dimP] = kalman_correct(z_dim, dim, dimP, kf.bbox.H, kf.bbox.R);
        track{obj{i}.ID}.kalman.dim(:,end+1) = dim;
        track{obj{i}.ID}.kalman.dimP{end+1} = dimP;

        
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % object is tracked for the first time %
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      
      elseif(last_obj{ind}.occur == minOccurence-1)
        % set new ID
        obj{i}.ID = newID;
        newID     = newID+1;
        
        % save object as a track
        % save all previous object center position
        % track{i} 4x1 contains [X_pos, Y_pos, Z_pos, frame_k_t]'
        for j=1:minOccurence-1 % for last_object!
          % project position to global coordinate system
          tmp = Tr_total{end-minOccurence+j} * ...
            [last_obj{ind}.center.X(j), last_obj{ind}.center.Y(j), last_obj{ind}.center.Z(j), 1]';
          track{obj{i}.ID}.raw.pos(:,j) = tmp(1:3);
          
          % project translation to global coordinate system
          track{obj{i}.ID}.raw.t(:,j) = ...
            Tr_total{end-minOccurence+j}(1:3,1:3)*[last_obj{ind}.center.tx(j), last_obj{ind}.center.ty(j), last_obj{ind}.center.tz(j)]';
          
          % set current frame of track
          track{obj{i}.ID}.frame(1,j) = k-minOccurence+j;
          
          % initalization for track
          pos_init(:,j) = [track{obj{i}.ID}.raw.pos(1,j), track{obj{i}.ID}.raw.pos(2,j), track{obj{i}.ID}.raw.pos(3,j),...
                           track{obj{i}.ID}.raw.t(1,j), track{obj{i}.ID}.raw.t(2,j), track{obj{i}.ID}.raw.t(3,j)]';
          
          % set bounding box for track
          dim_init(:,j) = [last_obj{ind}.dim.w(j), last_obj{ind}.dim.h(j), last_obj{ind}.dim.l(j)]';
        end
        
        % save current object center position
        % project position to global coordinate system
        tmp = Tr_total{end} * [obj{i}.center.X(end), obj{i}.center.Y(end), obj{i}.center.Z(end), 1]';
        track{obj{i}.ID}.raw.pos(:,end+1) = tmp(1:3);
        track{obj{i}.ID}.obj_id = i;
        track{obj{i}.ID}.past.death = [];
                
        % project translation to global coordinate system
        track{obj{i}.ID}.raw.t(:,end+1) = ...
          Tr_total{end}(1:3,1:3)*[obj{i}.center.tx(end), obj{i}.center.ty(end), obj{i}.center.tz(end)]';
        
        % detections in image coordinate system
        track{obj{i}.ID}.detections.pos = [obj{i}.X, obj{i}.Y, obj{i}.Z]';
        track{obj{i}.ID}.detections.t   = [obj{i}.tx, obj{i}.ty, obj{i}.tz]';
        
        % set current frame of track
        track{obj{i}.ID}.frame(1,end+1) = k;
        
        % Need this additional information to know, how long a track is
        % predicted and when it should die.
        % Of course, when it is initiated, it is active and not predicted
        track{obj{i}.ID}.prediction  = 0;
        track{obj{i}.ID}.active      = 1;
        % needed to delete the predicted trajectory
        track{obj{i}.ID}.future.h{1} = [];
        track{obj{i}.ID}.future.h{2} = [];

        %%%%%%%%%%%%%%%%
        % track object %
        %%%%%%%%%%%%%%%%
        
        % init
        pos_init(:,end+1) = [track{obj{i}.ID}.raw.pos(1,end), track{obj{i}.ID}.raw.pos(2,end), track{obj{i}.ID}.raw.pos(3,end),...
                             track{obj{i}.ID}.raw.t(1,end), track{obj{i}.ID}.raw.t(2,end), track{obj{i}.ID}.raw.t(3,end)]';
        pos_init = trimmean(pos_init',51)';
        
        % predict
        [pos posP] = kalman_predict(pos_init, kf.pos.P_init, kf.pos.A, kf.pos.Q);
        
        % measurement z and update
        z_pos = [track{obj{i}.ID}.raw.pos(1,end), track{obj{i}.ID}.raw.pos(2,end), track{obj{i}.ID}.raw. pos(3,end),...
                 track{obj{i}.ID}.raw.t(1,end), track{obj{i}.ID}.raw.t(2,end), track{obj{i}.ID}.raw.t(3,end)]';
        [pos posP] = kalman_correct(z_pos, pos, posP, kf.pos.H, kf.pos.R);
        
        % write data to track
        % init previous steps with nans (steps before tracking are used for initialization
        track{obj{i}.ID}.kalman.pos(:,1:minOccurence-1) = nan(3, minOccurence-1);
        track{obj{i}.ID}.kalman.t(:,1:minOccurence-1)   = nan(3, minOccurence-1);
        track{obj{i}.ID}.kalman.posP(1:minOccurence-1)  = repmat({nan(6)},1, minOccurence-1);
        
        % set first tracking step
        track{obj{i}.ID}.kalman.pos(1:3,end+1) = pos(1:3);
        track{obj{i}.ID}.kalman.t(1:3,end+1)   = pos(4:6);
        track{obj{i}.ID}.kalman.posP{end+1}    = posP;
        
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % track bounding box for track %
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        % init
        dim_init(:,j) = [obj{i}.dim.w(end), obj{i}.dim.h(end), obj{i}.dim.l(end)]';
        dim_init = trimmean(dim_init',51)';
        
        % predict
        [dim dimP] = kalman_predict(dim_init, kf.bbox.P_init, kf.bbox.A, kf.bbox.Q);
        
        % measurement and update
        z_dim = [obj{i}.dim.w(end), obj{i}.dim.h(end), obj{i}.dim.l(end)]';
        [dim dimP] = kalman_correct(z_dim, dim, dimP, kf.bbox.H, kf.bbox.R);
        
        % write data to track
        track{obj{i}.ID}.kalman.dim(:,k) = dim;
        track{obj{i}.ID}.kalman.dimP{k} = dimP;
      end
      
      
      %%%%%%%%%%%%%%%%%
      % update object %
      %%%%%%%%%%%%%%%%%
      
      % occurence of the object
      obj{i}.occur = last_obj{ind}.occur + 1;
      
      % object center
      obj{i}.center.X = [last_obj{ind}.center.X, obj{i}.center.X];
      obj{i}.center.Y = [last_obj{ind}.center.Y, obj{i}.center.Y];
      obj{i}.center.Z = [last_obj{ind}.center.Z, obj{i}.center.Z];
      
      % object center translation
      obj{i}.center.tx = [last_obj{ind}.center.tx, obj{i}.center.tx];
      obj{i}.center.ty = [last_obj{ind}.center.ty, obj{i}.center.ty];
      obj{i}.center.tz = [last_obj{ind}.center.tz, obj{i}.center.tz];
      
      % object dimensions
      obj{i}.dim.w = [last_obj{ind}.dim.w, obj{i}.dim.w];
      obj{i}.dim.h = [last_obj{ind}.dim.h, obj{i}.dim.h];
      obj{i}.dim.l = [last_obj{ind}.dim.l, obj{i}.dim.l];
     
    end
  end
end % function associateObjects
