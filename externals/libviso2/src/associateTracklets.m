%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright 2013. All rights reserved.                                    %
% Institute of Measurement and Control Systems                            %
% Karlsruhe Institute of Technology, Germany                              %
%                                                                         %
% This file is part of triTrack2.                                         %
% Authors:  Philip Lenz                                                   %
%           Please send any bugreports to lenz@kit.edu                    % 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [track, obj] = associateTracklets(track, obj, Tr, world_pts, world_t, frame, maxTracking, maxDist, velocity_threshold, verbose)
% ASSOCIATETRACKLETS is a downstream function to check if lost tracks are
%   continued by new initiated tracks

%%%%%%%%%%%%%%%%%%%%%%
% init Kalman Filter %
%%%%%%%%%%%%%%%%%%%%%%

% parameter for bbox-dimension Kalman-Filter
kf.bbox.A     = eye(3);
kf.bbox.H     = eye(3);
kf.bbox.Q     = eye(3)*1e-1; % 1e-3
kf.bbox.R     = eye(3)*5e-1; % 1e1
kf.bbox.P_init = zeros(3);

% system model for prediction. x=[X,Y,Z,tx,ty,tz];
kf.pos.A      = [1 0 0 1 0 0;
                 0 1 0 0 1 0;
                 0 0 1 0 0 1;
                 0 0 0 1 0 0;
                 0 0 0 0 1 0;
                 0 0 0 0 0 1];
kf.pos.H      = eye(6); 
kf.pos.Q      = (eye(6)*1e-3).^2; 
kf.pos.R      = (eye(6)*0.2).^2;
kf.pos.P_init = zeros(6);

% no of. frames that are predicted for visualization
prediction_frames = 10;

for itTrack = 1:numel(track)

  track_update_needed = false;
  
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  % Track Management (init, continue, die) %
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

  % Is this track broken in the last frame(s)? Predict its state.
  if( track{itTrack}.frame(end) < frame && track{itTrack}.active>=0)
    
    % predict position
    [x_predicted, P_predicted] = ...
      kalman_predict([track{itTrack}.kalman.pos(:,end);track{itTrack}.kalman.t(:,end)],...
                      track{itTrack}.kalman.posP{end}, kf.pos.A,  kf.pos.Q);
    
    [dim, dimP] = kalman_predict(track{itTrack}.kalman.dim(:,end), track{itTrack}.kalman.dimP{end}, kf.bbox.A, kf.bbox.Q);
    track{itTrack}.kalman.dim(:,end+1) = dim;
    track{itTrack}.kalman.dimP{end+1}  = dimP;

    track{itTrack}.kalman.pos(:,end+1) = x_predicted(1:3);
    track{itTrack}.kalman.t(:,end+1)   = x_predicted(4:6);
    track{itTrack}.kalman.posP{end+1}  = P_predicted;
    
    % take a look around, if there is some other measurement that continues the broken track
    n=[]; d=[];
    for j = 1:numel(obj)
        % compute and evaluate 
        %   -> crossproduct (similar translation) and
        %   -> dotproduct (same orientation)

        % project translation to global coordinate system
        tmp = Tr{end}(1:3,1:3)*[obj{j}.center.tx(end), obj{j}.center.ty(end), obj{j}.center.tz(end)]';
        t_obj=tmp(1:3);
        % project position to global coordinate system
        tmp = Tr{end} * [obj{j}.center.X(end), obj{j}.center.Y(end), obj{j}.center.Z(end), 1]';
        pos_obj = tmp(1:3);

        cp_t(j,:) = cross(t_obj, track{itTrack}.kalman.t(:,end));
        dot_t(j)  = dot(t_obj, track{itTrack}.kalman.t(:,end));

        % only consider object with a translation in the same direction
        % by evaluating the dot product (dot>0 --> vectors are oriented in the same direction
        if(dot_t(j)>0 && norm(cp_t(j,:))<velocity_threshold)

          n(end+1) = j; % who is the neighbor
          d(end+1) = norm( track{itTrack}.kalman.pos(:,end) - pos_obj); % and how far away
        end
    end
    
    % check if 3D matches are within the predicted bounding box3
    if ~isempty(world_pts)
      box(:,1) = [-track{itTrack}.kalman.dim(1,end)/2, track{itTrack}.kalman.dim(1,end)/2];
      box(:,2) = [-track{itTrack}.kalman.dim(2,end)/2, track{itTrack}.kalman.dim(2,end)/2];
      box(:,3) = [-track{itTrack}.kalman.dim(3,end)/2, track{itTrack}.kalman.dim(3,end)/2]*1.1;
      box      = objectToWorld(box, track{itTrack}, Tr{end});
      ind = world_pts(1,:)>box(1,1) & world_pts(1,:)<box(2,1) & ...
            world_pts(3,:)>box(1,3) & world_pts(3,:)<box(2,3);
    else
      ind = [];
    end
    % enough points were found
    if sum(ind)>=5
      if verbose, fprintf('  . track %d is continued in static mode\n', itTrack); end
      % create auxillary measurement
      pos = mean(world_pts(:,ind),2);
      t = mean(world_t(:,ind),2);
      % 
      pos = Tr{end} * [pos', 1]';
      track{itTrack}.raw.pos(:,end+1) = pos(1:3);
      track{itTrack}.raw.t(:,end+1) = ...
          Tr{end}(1:3,1:3)*t;
      % track{itTrack}.raw.t(:,end+1) = t;
    else
      % the track is predicted for one more timestep (also maybe the first)
      track{itTrack}.prediction = track{itTrack}.prediction + 1;
    end
    track{itTrack}.frame(end+1) = frame;
    
    
    %%%%%%%%%%%%%%%%%%%%%%
    % Continue the track %
    %%%%%%%%%%%%%%%%%%%%%%
    
    % The track was only predicted: Is there a new measurement/track, that continues it?
    % TODO: consider quadratically growing stereo error!
    if( track{itTrack}.prediction > 0   &&    ~isempty(d)   &&   min(d) < maxDist )
      % -> If so, associate it
      indO = n(d==min(d));
      if verbose, fprintf('  . track %d is continued by object %d\n', itTrack, indO ); end
      
      % the closest object n(d==min(d)) gets the ID of the lost track
      obj{indO}.ID = itTrack;
      track{itTrack}.obj_id = indO;
      
      %%%%%%%%%%%%%%%%%%%%%
      % update the object %
      %%%%%%%%%%%%%%%%%%%%%
      % occurence of the object
      obj{indO}.occur = numel(track{itTrack}.frame) + 1;
      
      % this_obj is needed for a common updatestep for measurement
      % association and feature update by a selected ACC object
      
      % object center
      obj{indO}.center.X = [track{itTrack}.raw.pos(1,:), obj{indO}.center.X];
      obj{indO}.center.Y = [track{itTrack}.raw.pos(2,:), obj{indO}.center.Y];
      obj{indO}.center.Z = [track{itTrack}.raw.pos(3,:), obj{indO}.center.Z];
      
      this_obj.center.X = obj{indO}.center.X;
      this_obj.center.Y = obj{indO}.center.Y;
      this_obj.center.Z = obj{indO}.center.Z;
      
      % object center translation
      obj{indO}.center.tx = [track{itTrack}.raw.t(1,:), obj{indO}.center.tx];
      obj{indO}.center.ty = [track{itTrack}.raw.t(2,:), obj{indO}.center.ty];
      obj{indO}.center.tz = [track{itTrack}.raw.t(3,:), obj{indO}.center.tz];
      
      this_obj.center.tx = obj{indO}.center.tx;
      this_obj.center.ty = obj{indO}.center.ty;
      this_obj.center.tz = obj{indO}.center.tz;
      
      % object dimensions: don't need this anyway
      this_obj.dim.w = obj{indO}.dim.w(end);
      this_obj.dim.h = obj{indO}.dim.h(end);
      this_obj.dim.l = obj{indO}.dim.l(end);
      
      track_update_needed = true;
      
    % is this track broken for more than <maxTracking> frame(s)? Skip it.
    elseif( track{itTrack}.prediction > maxTracking   &&   track{itTrack}.active > 0 )
      % -> Track died.
      if verbose, fprintf('  . Track %d died.\n', itTrack); end
      track{itTrack}.active = 0;
      track{itTrack}.past.death_frame = frame;
      
    % set the track as dead
    elseif( track{itTrack}.active==0 )
      track{itTrack}.active = -1;
    end
    
    % update the track if necessary
    if(track_update_needed)
      
     
      %%%%%%%%%%%%%%%%%%%%%%%%%    
      % continue object track %
      %%%%%%%%%%%%%%%%%%%%%%%%%

      % save current object center position
      % project translation to global coordinate system
      tmp = Tr{end} * [this_obj.center.X(end), this_obj.center.Y(end), this_obj.center.Z(end), 1]';
      track{itTrack}.raw.pos(:,end) = tmp(1:3);

      % project translation to global coordinate system
      track{itTrack}.raw.t(:,end) = ...
        Tr{end}(1:3,1:3)*[this_obj.center.tx(end), this_obj.center.ty(end), this_obj.center.tz(end)]';

      % the predicted state is updated by the new measurement
      z_pos = [track{itTrack}.raw.pos(:,end); track{itTrack}.raw.t(:,end)];
      pos   = [track{itTrack}.kalman.pos(:,end); track{itTrack}.kalman.t(:,end)];
      [pos posP] = kalman_correct(z_pos, pos, track{itTrack}.kalman.posP{end}, kf.pos.H, kf.pos.R);
      track{itTrack}.kalman.pos(:,end) = pos(1:3);
      track{itTrack}.kalman.t(:,end)   = pos(4:6);
      track{itTrack}.kalman.posP{end}  = posP;
      
      % the track is not predicted anymore.
      track{itTrack}.prediction = 0;
      track{itTrack}.active = 1;

      % track bounding box for track
      z_dim = [this_obj.dim.w(end), this_obj.dim.h(end), this_obj.dim.l(end)]';
      dim   = track{itTrack}.kalman.dim(:,end);
      [dim dimP] = kalman_correct(z_dim, dim, track{itTrack}.kalman.dimP{end}, kf.bbox.H, kf.bbox.R);
      track{itTrack}.kalman.dim(:,end) = dim;
      track{itTrack}.kalman.dimP{end}  = dimP;
    end
   
  end % if: trackManagement
  
  % always predict the trajectory for 5 timesteps, if the track is active.
  if( track{itTrack}.active )
    
    % init prediction
    [x_predicted, P_predicted] = ...
      kalman_predict([track{itTrack}.kalman.pos(:,end); track{itTrack}.kalman.t(:,end)],...
                      track{itTrack}.kalman.posP{end}, kf.pos.A,  kf.pos.Q);
    track{itTrack}.future.pos(:,1) = x_predicted(1:3);
    track{itTrack}.future.t(:,1)   = x_predicted(4:6);
    
    % predict reamining timesteps
    for i=2:prediction_frames
      [x_predicted, P_predicted] = kalman_predict(x_predicted, P_predicted, kf.pos.A,  kf.pos.Q);
                    
      track{itTrack}.future.pos(:,i) = x_predicted(1:3);
      track{itTrack}.future.t(:,i)   = x_predicted(4:6);
    end
  end % if: predict active trajectory
end % for: track

end % function associateTracklets