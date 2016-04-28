%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright 2013-2014. All rights reserved.                               %
% Institute of Measurement and Control Systems                            %
% Karlsruhe Institute of Technology, Germany                              %
%                                                                         %
% This file is part of triTrack2.                                         %
% Authors:  Philip Lenz                                                   %
%           Please send any bugreports to lenz@kit.edu                    % 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function saveRawObjects(frame, obj, params, cam, Tr)

  % params.seq_idx
  %fid = 1;
  if isfield(params, 'drive') && isfield(params, 'date')
    filename = fullfile(params.detections_folder,sprintf('%04d.txt',params.drive));
  else
    filename = fullfile(params.detections_folder,sprintf('%04d.txt',params.seq_idx));
  end
  fid = fopen(filename,'a');
  %keyboard;
  %---
%   obj = [];
%   obj{1}.dim.w = 2;
%   obj{1}.dim.h = 1;
%   obj{1}.dim.l = 10;
%   obj{1}.center.X = 5;
%   obj{1}.center.Y = 1;
%   obj{1}.center.Z = 10;
%   obj{1}.center.tx = 1;
%   obj{1}.center.ty = 0;
%   obj{1}.center.tz = 0;
%   obj{1}.X = 5;
%   obj{1}.Y = 1;
%   obj{1}.Z = 10;
%   obj{1}.score = 0.2;
  %---
  for o=[obj{:}]
    %if o.dim.h(end)>2.5 || o.dim.h(end)<0.5 || max(o.Y)<-4
    vol = o.dim.h(end)*o.dim.w(end)*o.dim.l(end);
    if vol<0.5 || vol>80 || max(o.Y)<-2
      continue
    end
    % extract information to dump in KITTI tracking format
    dim = [o.dim.h(end), o.dim.w(end), o.dim.l(end)];
    %disp ([dim,o.score])
    pos = [(min(o.X)+max(o.X))/2, max(o.Y), (min(o.Z)+max(o.Z))/2];
    %pos = [mean(o.X), -min(o.Y), mean(o.Z)];
    rot = atan2( o.center.tx(end),o.center.tz(end) )-pi/2;
    bbox = get_bbox(o,rot,Tr,cam);
    score = 1-o.score;
    %keyboard;
    
    %center_height_over_ground = obj{i}.center.Y(end) + tan(deg2rad(cam.mounting_pitch_deg))*obj{i}.center.Z(end) - cam.height_over_ground;
    
    %   #Values    Name      Description
    % ----------------------------------------------------------------------------
    %  1    frame        Frame within the sequence where the object appearers
    %  1    track id     Unique tracking id of this object within this sequence
    %  1    type         Describes the type of object: 'Car', 'Van', 'Truck',
    %                    'Pedestrian', 'Person_sitting', 'Cyclist', 'Tram',
    %                    'Misc' or 'DontCare'
    %  1    truncated    Float from 0 (non-truncated) to 1 (truncated), where
    %                    truncated refers to the object leaving image boundaries
    %  1    occluded     Integer (0,1,2,3) indicating occlusion state:
    %                    0 = fully visible, 1 = partly occluded
    %                    2 = largely occluded, 3 = unknown
    %  1    alpha        Observation angle of object, ranging [-pi..pi]
    %  4    bbox         2D bounding box of object in the image (0-based index):
    %                    contains left, top, right, bottom pixel coordinates
    %  3    dimensions   3D object dimensions: height, width, length (in meters)
    %  3    location     3D object location x,y,z in camera coordinates (in meters)
    %  1    rotation_y   Rotation ry around Y-axis in camera coordinates [-pi..pi]
    %  1    score        Only for results: Float, indicating confidence in
    %                    detection, needed for p/r curves, higher is better.
    
    fprintf(fid, '%d %d %s %.1f %.1f %.1f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.6f %.6f\n',...
      frame-1, -1, 'Moving', -1, -1, -10, bbox, dim, pos, rot, score);
  end
  fclose(fid);
end

function bbox = get_bbox(o,alpha,Tr,cam)

  % set the vertices of the bounding box
  vertices(1:4,1)       = -o.dim.w(end)/2;
  vertices(5:8,1)       =  o.dim.w(end)/2;
  vertices([1 2 5 6],2) = -o.dim.h(end)/2;
  vertices([3 4 7 8],2) =  o.dim.h(end)/2;
  vertices(1:2:8,3)     = -o.dim.l(end)/2;
  vertices(2:2:8,3)     =  o.dim.l(end)/2;
  %[vertices, pos]       = objectToWorld(vertices, o, Tr); 

  % set 3D rotational matrix
  R = [cos(alpha), 0, -sin(alpha);
       0,          1, 0;
       sin(alpha), 0, cos(alpha)];
     
  % mounting geometry of the stereo rig must be considered
  % bounding box must be determined with respect to the ground plane
  % pitch angle must be corrected
  % not nice -> use an estimated groundplane instead!
  obj_center      = [o.center.X(end); o.center.Y(end); o.center.Z(end)]; % get tracked object center
  %obj_center      = Tr^-1*[obj_center;ones(1,size(obj_center,2))]; % transform object center track in current image coordinate system
%   obj_center      = [obj_center;ones(1,size(obj_center,2))]; % transform object center track in current image coordinate system
   obj_center(2,:) = -obj_center(2,:);

  vertices = (R*vertices')' + repmat([obj_center(1,end), obj_center(2,end), obj_center(3,end)], size(vertices,1),1);
  
  % project the bounding box edges back to image coordinates
  [u, v] = project3dToImage(vertices(:,1), vertices(:,2), vertices(:,3), cam);  % vertices of the bounding box
  
  % u1,v1, u2,v2
  bbox = [min(u),min(v),max(u),max(v)];
end
