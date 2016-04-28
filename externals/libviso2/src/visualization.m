%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright 2013. All rights reserved.                                    %
% Institute of Measurement and Control Systems                            %
% Karlsruhe Institute of Technology, Germany                              %
%                                                                         %
% This file is part of triTrack2.                                         %
% Authors:  Philip Lenz                                                   %
%           Please send any bugreports to lenz@kit.edu                    % 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function varargout = visualization(mode, varargin)
% VISUALIZATION sets up the screen for correct_egomotion and draws results

%%%%%%%%%%%%%%%%%%%%%%%%%%%
% visualization parameter %
%%%%%%%%%%%%%%%%%%%%%%%%%%%

global display

%%%%%%%%%%%%%%%
% Choose mode %
%%%%%%%%%%%%%%%

switch lower(mode)
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  % SET UP THE SCREEN
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  case 'init'
    % get input arguments
    img_file   = varargin{1};
    cam        = varargin{2};
    thres      = varargin{3};
    img        = imread(img_file);

    % visualization parameter
    if(display.image.figure)
      
      h{1}.width  = size(img,2);
      h{1}.height = size(img,1);
      h{1}.fig    = figure; set(h{1}.fig, 'position', [0, 30, size(img,2), size(img,1)],...
        'NumberTitle','off', 'name','IMAGE VIEW', 'PaperPositionMode','auto','color',[0 0 0],...
        'InvertHardcopy','off', 'Toolbar','none', 'Menubar','none');
      h{1}.axes = axes('Position', [0 0 1 1]);
    end

    if(display.ics.figure)
      h{2}.fig  = figure('Color',[1 1 1]); set(h{2}.fig,'position',[0,80+size(img,1),600,400],...
        'NumberTitle','off', 'name','BIRDSVIEW - car coordinate system', 'PaperPositionMode','auto',...
        'color',[0 0 0],'InvertHardcopy','off', 'Toolbar','none', 'Menubar','none');
      h{2}.axes = axes('Position',[0.05,0.05,0.9,0.9], 'color', 'black', 'xcolor','r','ycolor','r'); grid on; % axis equal
      set(h{2}.axes, 'XTick', -500:10:500);
      set(h{2}.axes, 'YTick', -500:10:500);
      set(h{2}.axes, 'ZTick', -500:10:500);
    end

    if(display.wcs.figure)
      h{3}.fig  = figure('Color',[1 1 1]); set(h{3}.fig,'position',[610,80+size(img,1),600,400],...
        'NumberTitle','off', 'name','BIRDSVIEW - world coordinate system', 'PaperPositionMode','auto',...
        'color',[0 0 0],'InvertHardcopy','off', 'Toolbar','none', 'Menubar','none');
      h{3}.axes = axes('Position',[0.05,0.05,0.9,0.9], 'color', 'black', 'xcolor','r','ycolor','r'); grid on; % axis equal
      set(h{3}.axes, 'XTick', -500:10:500);
      set(h{3}.axes, 'YTick', -500:10:500);
      set(h{3}.axes, 'ZTick', -500:10:500);
    end
    % angular aperture
    tan_aa = tan(pi/2 - atan(size(img,2)/cam.f/2));
    
    h{3}.area = [0 thres.maxLat,        thres.maxLat,  -thres.maxLat,  -thres.maxLat,        0;
                 0 thres.maxLat*tan_aa, thres.maxDist,  thres.maxDist, thres.maxLat*tan_aa, 0];
    h{3}.area(2,:) = h{3}.area(2,:)-thres.maxDist/2;
    h{3}.area_patch = [];

    % set output arguments
    varargout{1} = h;
    
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  % DRAW RESULTS
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  case 'draw'
   
    %%%%%%%%%%%%%%%%%%%%%%%%
    % get input parameters %
    %%%%%%%%%%%%%%%%%%%%%%%%
    
    h             = varargin{1};
    I1            = varargin{2};
    frame         = varargin{3};
    ind_d         = varargin{4};
    minOccurence  = varargin{5};
    deltaT        = varargin{6};
    thres         = varargin{7};
    Tr_total      = varargin{8};
    p_matched     = varargin{9};
    obj           = varargin{10};
    track         = varargin{11};
    acc           = varargin{12};
    cam           = varargin{13};
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % show image coordinate system %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %%%%%%%
    % image, detections and information in the image
    %%%%%%%

    % prepare axes for image and image coordinate system birdsview
    if(display.image.figure)
      cla(h{1}.axes);
      
      % set axis and show the image
      imagesc(I1,'parent', h{1}.axes); 
      colormap(h{1}.axes,'gray'); 
      axis(h{1}.axes,'image', 'off'); 
      hold(h{1}.axes, 'on');
      % show info in the image
      h{1}.frame = text(h{1}.width-10, h{1}.height-5,...
           sprintf('Frame %d',frame-1),...
           'parent', h{1}.axes, 'color','g', 'HorizontalAlignment','right','VerticalAlignment','bottom',...
           'FontSize',14,'FontWeight','bold', 'BackgroundColor','black');
      % usage instructions
      h{1}.usage = text(h{1}.width/2,h{1}.height-5,...
           sprintf('''SPACE'': Play/Pause  |  ''+'': Next Image  |  ''q'': quit'),...
           'parent', h{1}.axes, 'color','g','HorizontalAlignment','center','VerticalAlignment','bottom',...
           'FontSize',14,'FontWeight','bold', 'BackgroundColor','black');
    end
    
    % set birdsview axis
    if(display.ics.figure)
      cla(h{2}.axes); hold(h{2}.axes, 'on');
      axis(h{2}.axes, [-thres.maxLat,thres.maxLat,5,thres.maxDist]); 
      axis(h{2}.axes, 'equal'); 
    end
    
    % show static points in image
    if(display.image.figure && display.image.staticFeatures)
      plot(h{1}.axes, p_matched(5,~ind_d),  p_matched(6,~ind_d), 'r.', 'MarkerSize', 4)
    end

    %%%%%%%
    % show untracked objects
    %%%%%%%
    
    map_track_obj = [];
    u_d = p_matched(5,ind_d); v_d = p_matched(6,ind_d);
    if ~isempty(obj) % are there any objects?
      for i=1:numel(obj) % check all objects
        if isnan(obj{i}.ID) % is there no ID --> not tracked
          % get indices of dynamic matches and plot them in image and current birdsview
          if(display.image.figure   &&   display.image.untrackedFeatures)
            %u_d = p_matched(5,ind_d); v_d = p_matched(6,ind_d);
            plot(h{1}.axes, u_d(obj{i}.pts), v_d(obj{i}.pts),'.','color', getColor(i), 'MarkerSize',8)
          end
          
          % birdsview ICS
          if(display.ics.figure   &&   display.ics.untrackedFeatures)
            plot(h{2}.axes, obj{i}.X, obj{i}.Z,'.','color', getColor(i));
          
            % plot velocity for object center
            quiver(h{2}.axes, obj{i}.center.X(end), obj{i}.center.Z(end),...
                            obj{i}.center.tx(end)/deltaT, obj{i}.center.tz(end)/deltaT,...
                            0, 'color',  getColor(i)) % 0 --> do not scale!
                          
            % plot velocity for all pts
            quiver(h{2}.axes, obj{i}.X, obj{i}.Z, obj{i}.tx/deltaT, obj{i}.tz/deltaT, 0, 'color',  getColor(i))
          end
        elseif track{obj{i}.ID}.prediction==0 % only tracks with data evidence are shown as patches
          map_track_obj(obj{i}.ID) = i;
        end
      end
    end
    
    
    %%%%%%%
    % show tracked objects
    %%%%%%%
    
    if(~isempty(track)) % does any track exist?
      
      for itTrack=1:numel(track) % plot all existing tracks
        
        % for ACC just show selected object
        if(acc.active && ~any(itTrack==acc.track)), continue; end
        
        if(display.image.figure   &&   track{itTrack}.active == 1)
          % Show all matches belonging to one tracked object
          if display.image.trackedFeatures && track{itTrack}.prediction==0 && track{itTrack}.obj_id<=numel(obj)
            plot(h{1}.axes, u_d(obj{track{itTrack}.obj_id}.pts), v_d(obj{track{itTrack}.obj_id}.pts),...
              'o','color', getColor(obj{track{itTrack}.obj_id}.ID), 'MarkerSize',10, 'LineWidth', 3);
          end
          
          % draw a bounding box around a tracked object
          bbox_image(h{1}.axes, track{itTrack}, itTrack, getColor(itTrack), cam, deltaT, Tr_total{end}, h{3}.area);

          % draw a transparent patch (convex hull of the detections describing a detected object)
          % TODO: This can be done a bit nicer.
          %       -> Track the points of the convex hull or
          %       -> Use CCL in the disparity image or something like that to get the boundaries
          % patches are only shown if a current object detection is present, not for prediction
          if display.image.patches && track{itTrack}.prediction==0 && track{itTrack}.obj_id<=numel(obj)
            % patch_image(h{1}.axes, [u_d(obj{i}.pts); v_d(obj{i}.pts)], getColor(obj{i}.ID))
            patch_image(h{1}.axes, [u_d(obj{track{itTrack}.obj_id}.pts);...
                                    v_d(obj{track{itTrack}.obj_id}.pts)], getColor(itTrack))
          end
        end
        
        % show the bounding box in the current birdsview
        if(display.ics.figure   &&   track{itTrack}.active == 1)
          bbox_birdsview(h{2}.axes, track{itTrack}, itTrack, getColor(itTrack), Tr_total{end}, deltaT);
        end
      end
    end

    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % show world coordinate system %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    if(display.wcs.figure)
      % only plot tracks in the WCS
      hold(h{3}.axes, 'on'); 
      % draw the field of view to world coordinate system
      [h{3} FOV] = drawFOV(h{3}, Tr_total{end});

      if(~isempty(track)) % does any track exist?
        for itTrack=1:numel(track) % plot all existing tracks

          % if the track occurs for the first time, all previous measurements are plotted and the ID is shown
          if( numel(track{itTrack}.frame)==minOccurence )
            plot( h{3}.axes, track{itTrack}.raw.pos(1,:), track{itTrack}.raw.pos(3,:), '-', 'color', getColor(itTrack) );
            if(display.image.ID)
              if (track{itTrack}.raw.pos(1,end-1)-track{itTrack}.raw.pos(1,end)>0)
                offset_x = 2;
              else
                offset_x = -2;
              end
            end
          % otherwise, plot the current measurement, update and predicted trajectory (if any)
          elseif(track{itTrack}.prediction == 0)
            %plot( h{3}.axes, track{itTrack}.raw.pos(1,end), track{itTrack}.raw.pos(3,end), 'o', 'color', getColor(itTrack) );

            plot( h{3}.axes, track{itTrack}.kalman.pos(1,end-1:end), track{itTrack}.kalman.pos(3,end-1:end),...
              '-', 'color', getColor(itTrack), 'marker', 'None', 'LineWidth', 2, 'MarkerSize', 11 );
            if( ishandle(track{itTrack}.future.h{1}) )
              delete(track{itTrack}.future.h{:});
            else
              track{itTrack}.future.h{1} = [];
              track{itTrack}.future.h{2} = [];
            end
            track{itTrack}.future.h{1} = plot( h{3}.axes, track{itTrack}.future.pos(1,:), track{itTrack}.future.pos(3,:),...
              '-.', 'color', getColor(itTrack), 'marker', 'none', 'LineWidth', 1, 'MarkerSize', 11 );
            track{itTrack}.future.h{2} = quiver( h{3}.axes, track{itTrack}.future.pos(1,end), track{itTrack}.future.pos(3,end),...
              track{itTrack}.future.t(1,end)/deltaT/2, track{itTrack}.future.t(3,end)/deltaT/2, 0, 'color', getColor(itTrack));
            %-----
            track{itTrack}.future.h{3} = text(track{itTrack}.future.pos(1,end)+2, track{itTrack}.future.pos(3,end), num2str(itTrack),...
                  'parent', h{3}.axes, 'color', getColor(itTrack), 'FontWeight','bold', 'FontSize', 20);
            if ishandle(track{itTrack}.past.death)
              delete(track{itTrack}.past.death)
              ind = track{itTrack}.frame>=track{itTrack}.past.death_frame-1;
              plot( h{3}.axes, track{itTrack}.kalman.pos(1,ind), track{itTrack}.kalman.pos(3,ind),...
                '-', 'color', getColor(itTrack), 'marker', 'None', 'LineWidth', 2, 'MarkerSize', 11 );
            end

          % or plot the predicted position if there was no update
          elseif(track{itTrack}.prediction > 0   &&   track{itTrack}.active == 1)
            plot( h{3}.axes, track{itTrack}.kalman.pos(1,end-1:end), track{itTrack}.kalman.pos(3,end-1:end),...
              '--', 'color', getColor(itTrack), 'marker', 'none', 'LineWidth', 2);
          % mark a track as died
          elseif( track{itTrack}.active == 0  )
            track{itTrack}.past.death = plot( h{3}.axes,...
              track{itTrack}.kalman.pos(1,end), track{itTrack}.kalman.pos(3,end),...
              '-', 'color', getColor(itTrack), 'marker', 'x', 'MarkerSize', 10, 'LineWidth', 2);
            % delete predictions
            if( ishandle(track{itTrack}.future.h{1}) )
              delete(track{itTrack}.future.h{:});
            else
              track{itTrack}.future.h{1} = [];
              track{itTrack}.future.h{2} = [];
            end
          end
        end % for: track
      end % if: track

      % plot egomotion
      plot(h{3}.axes, [Tr_total{end-1}(1,4), Tr_total{end}(1,4)],...
                      [Tr_total{end-1}(3,4), Tr_total{end}(3,4)], '-*', 'color', [1.0 0.6 0]);
      % set axis for worldframe
      tmp = get(h{3}.axes); % old axis
      % visible area is either the old area or the current FOV in the WCS
      h{3}.Xmin = min(floor( min( [tmp.XLim(1), FOV(1,:) ] )/10)*10 ,-10);
      h{3}.Xmax = max(ceil( max( [tmp.XLim(2), FOV(1,:) ] )/10)*10 ,10);
      h{3}.Zmin = min(floor( min( [tmp.YLim(1), FOV(2,:) ] )/10)*10 , 0);
      h{3}.Zmax = max(ceil( max( [tmp.YLim(2), FOV(2,:) ])/10)*10 ,30);
      axis(h{3}.axes, [h{3}.Xmin, h{3}.Xmax, h{3}.Zmin, h{3}.Zmax]); 
      delta_x = (h{3}.Xmax - h{3}.Xmin)/6;
      delta_z = (h{3}.Zmax - h{3}.Zmin)/6;
      xticks = h{3}.Xmin:delta_x:h{3}.Xmax;
      zticks = h{3}.Zmin:delta_z:h{3}.Zmin;
      set(h{3}.axes, 'XTick',xticks, 'YTick', zticks);
    end
    % flush
    if(display.flush), drawnow; end
    % set output arguments
    varargout{1} = h;
    varargout{2} = track; % needed to delete future trajectory handle
  otherwise
    error('Unkown case. Use ''init'' or ''draw''.')
end

end % function visualization

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% HELPER FUNCTIONS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [h FOV] = drawFOV(h, Tr)
% prepare FOV in WCS
% set 2D rotational matrix

global display

R(1,:)=Tr(1,[1,3]);
R(2,:)=Tr(3,[1,3]);
R=R^-1;

% rotate FOV according to the ego vehicle's orientation
FOV = R^-1*h.area;
FOV(1,:) = FOV(1,:) - FOV(1,1) + Tr(1,4);
FOV(2,:) = FOV(2,:) - FOV(2,1) + Tr(3,4);

% erase old FOV and draw the new one
if(display.wcs.FOV)
  delete(h.area_patch);
  % You might encounter issues with a dual screen setup using a nvidia
  % graphics device and drawing transparent areas. Disabeling the
  % transparent areas should work fine instead.
  % There are several possible workarounds for drawing with such a setup on
  % the internet which might work for one or another system.
  % Using EraseMode=xor might work bad with image data, but at least it
  % does not lead to a Matlab crash.
  if display.nvidia_workaround
    h.area_patch = patch(FOV(1,:), FOV(2,:),[0.4, 0.4, 0.4], 'parent', h.axes,...
      'LineStyle', 'none', 'EraseMode','xor');
  else
    h.area_patch = patch(FOV(1,:), FOV(2,:),[0.4, 0.4, 0.4], 'parent', h.axes,...
      'LineStyle', 'none', 'FaceAlpha', 0.4);
  end
end
end %function drawFOV

function color_out = getColor(ind, colors)
% GETCOLOR returns the color from an optional given cell array colors 
%   according to the given index ind

if( nargin==1 )
  colors={'r','g','b', [0.8 0.2 0] ,'y','c'};
end

color_out = colors{ mod(ind,numel(colors)) + 1 };

end %function getColor

function patch_image(h, img_pts, col)
  global display
  ind = convhull(img_pts(1,:)',img_pts(2,:)');
  if display.nvidia_workaround
    patch(img_pts(1,ind)',img_pts(2,ind),col,...
      'parent', h, 'LineStyle', 'none', 'EraseMode','xor');
  else
    patch(img_pts(1,ind)',img_pts(2,ind),col,...
      'parent', h, 'LineStyle', 'none', 'FaceAlpha', 0.4);
  end
end

function track = bbox_image(h, track, ID, col, cam, deltaT, Tr, FOV)
%function bbox_image(h, obj, col, cam, deltaT, track, Tr, FOV)
% BBOX_IMAGE draws for a given object the backprojected bounding box in the image

global display

% set the field of view for the image coordinate system
FOV(2,:) = FOV(2,:)-min(FOV(2,:));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Bbox using filtered position and dimensions %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% set the vertices of the bounding box
vertices(1:4,1)       = -track.kalman.dim(1,end)/2;
vertices(5:8,1)       =  track.kalman.dim(1,end)/2;
vertices([1 2 5 6],2) = -track.kalman.dim(2,end)/2;
vertices([3 4 7 8],2) =  track.kalman.dim(2,end)/2;
vertices(1:2:8,3)     = -track.kalman.dim(3,end)/2;
vertices(2:2:8,3)     =  track.kalman.dim(3,end)/2;
[vertices, pos]       = objectToWorld(vertices, track, Tr); 

triangle(:,1) = [-track.kalman.dim(1,end)/2, 0, track.kalman.dim(1,end)/2]; %X
triangle(:,2) = [-track.kalman.dim(2,end)/2, -track.kalman.dim(2,end)/2, -track.kalman.dim(2,end)/2]; %Y
triangle(:,3) = [track.kalman.dim(3,end)/2, track.kalman.dim(3,end), track.kalman.dim(3,end)/2;]; %Z
triangle      = objectToWorld(triangle, track, Tr); 

if(display.image.bbox)
  % is at least the bounding box in the FOV and all vertices fullfil Z>0?
  if(   ~any(inpolygon( vertices(:,1), vertices(:,3), FOV(1,:), FOV(2,:) ))   || any( vertices(:,3)<0) ) % FOV has only two dimensions [X,Z]'
    return;
  end
  % project the bounding box edges back to image coordinates
  [u, v] = project3dToImage(vertices(:,1), vertices(:,2), vertices(:,3), cam);  % vertices of the bounding box
  % make bounding box
  vec=[u,v];      % point matrices to draw box
  sortrows(vec);  % sort to get indices according to front_lower, back_upper, etc.
  % set linestyle for bbox for predicted tracks to dotted
  if( track.prediction )
    linestyle = '--';
    linewidth=1;
  else
    linestyle = '-';
    linewidth=1.5;
  end
  % background box
  line([vec(1,1),vec(2,1),vec(4,1),vec(3,1),vec(1,1)],[vec(1,2),vec(2,2),vec(4,2),vec(3,2),vec(1,2)],...
      'color','black','parent',h,'LineStyle',linestyle,'Linewidth',linewidth+1) % front rectangle
  line([vec(5,1),vec(6,1),vec(8,1),vec(7,1),vec(5,1)],[vec(5,2),vec(6,2),vec(8,2),vec(7,2),vec(5,2)],...
      'color','black','parent',h,'LineStyle',linestyle,'Linewidth',linewidth+1) % back rectangle
  line([vec(1,1),vec(5,1),vec(6,1),vec(2,1),vec(1,1)],[vec(1,2),vec(5,2),vec(6,2),vec(2,2),vec(1,2)],...
      'color','black','parent',h,'LineStyle',linestyle,'Linewidth',linewidth+1) % left rectangle
  line([vec(4,1),vec(8,1),vec(7,1),vec(3,1),vec(4,1)],[vec(4,2),vec(8,2),vec(7,2),vec(3,2),vec(4,2)],...
      'color','black','parent',h,'LineStyle',linestyle,'Linewidth',linewidth+1) % left rectangle
  % foreground box
  line([vec(1,1),vec(2,1),vec(4,1),vec(3,1),vec(1,1)],[vec(1,2),vec(2,2),vec(4,2),vec(3,2),vec(1,2)],...
      'color',col,'parent',h,'LineStyle',linestyle,'Linewidth',linewidth) % front rectangle
  line([vec(5,1),vec(6,1),vec(8,1),vec(7,1),vec(5,1)],[vec(5,2),vec(6,2),vec(8,2),vec(7,2),vec(5,2)],...
      'color',col,'parent',h,'LineStyle',linestyle,'Linewidth',linewidth) % back rectangle
  line([vec(1,1),vec(5,1),vec(6,1),vec(2,1),vec(1,1)],[vec(1,2),vec(5,2),vec(6,2),vec(2,2),vec(1,2)],...
      'color',col,'parent',h,'LineStyle',linestyle,'Linewidth',linewidth) % left rectangle
  line([vec(4,1),vec(8,1),vec(7,1),vec(3,1),vec(4,1)],[vec(4,2),vec(8,2),vec(7,2),vec(3,2),vec(4,2)],...
      'color',col,'parent',h,'LineStyle',linestyle,'Linewidth',linewidth) % left rectangle
  % show object ID and information
  if(display.image.ID)
    text(max(u), max(v), num2str(ID),...
      'parent', h, 'color', col, 'FontWeight','bold', 'FontSize', 16, 'BackgroundColor','white')
  end
  % make triangle for orientation
  if all(triangle(:,3)>1)
    [u, v] = project3dToImage(triangle(:,1), triangle(:,2), triangle(:,3), cam);  % vertices of the bounding box
    patch([u(1),u(2),u(3)],[v(1),v(2),v(3)],col,'parent',h) % front rectangle
  end
end % if: bbox
    
% if(display.image.position && display.image.bbox)
%   text(max(u), max(v)+10,...
%     sprintf('X = %.2f  V_X = %.2f\nY = %.2f  V_Y = %.2f\nZ = %.2f  V_Z = %.2f',...
%       pos(1,end), t(1,end)/deltaT,...
%       pos(2,end), t(2,end)/deltaT,...
%       pos(3,end), t(3,end)/deltaT),...
%       'parent', h, 'color', col, 'FontSize', 10,'VerticalAlignment','top')
% end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% plot trajectory in image %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if(display.image.trajectory)
  % check if trajectory is still in the field of view
  % set FOV to origin of the coordinate system, not centered in the world
  % (this is done in the beginning: FOV(2,:) = FOV(2,:)-min(FOV(2,:));)
  % FOV is a polygon, which includes all visible points
  out_of_image = ~inpolygon(pos(1,:),pos(3,:),FOV(1,:),FOV(2,:)); % what is in the FOV polygon?

  % it is nice for the visualization to draw the closest point out of the FOV as well
  % but only, if the point is not too close to the camera |X|>1m and Z>4m seems reasonable
  if( abs(pos(1, find(out_of_image==1,1,'last')))<1   &&   pos(3, find(out_of_image==1,1,'last'))>4 )
    out_of_image(find(out_of_image==1,1,'last')) = 0;
  elseif( abs(pos(1, find(out_of_image==1,1,'last')))>1 )
    out_of_image(find(out_of_image==1,1,'last')) = 0;
  end
  % current position should not be plotted as part of the history
  out_of_image(end)=1;

  % Debug: what is visible in the FOV?
  % figure; plot(FOV(1,:), FOV(2,:), pos(1, ~out_of_image), pos(3,~out_of_image),'r+', pos(1,out_of_image), pos(3,out_of_image),'bo')

  % project trajectory back to the image and draw
  dh = track.kalman.dim(2,end)/2;
  [u_tr, v_tr] = project3dToImage(pos(1,:), pos(2,:)-dh, pos(3,:), cam);
  % plot current marker as a bigger circle
  plot(h, u_tr(end), v_tr(end), 'color', col, 'marker', 'none', 'markersize', 10, 'Linewidth', 2, 'LineStyle', '-.')
  plot(h, u_tr(~out_of_image(1:end-1)), v_tr(~out_of_image(1:end-1)),...
    'color', 'black', 'marker', 'none', 'Linewidth', 3, 'LineStyle', '-')
  plot(h, u_tr(~out_of_image(1:end-1)), v_tr(~out_of_image(1:end-1)),...
    'color', col, 'marker', 'none', 'Linewidth', 2, 'LineStyle', '-')

  % Debug: show raw measurements
  % out_of_image = ~inpolygon(raw(1,:),raw(3,:),FOV(1,:),FOV(2,:)); % what is in the FOV polygon?
  % [u_raw, v_raw] = project3dToImage(raw(1,:), raw(2,:), raw(3,:), cam);
  % plot(h, u_raw(~out_of_image(1:end-1)), v_raw(~out_of_image(1:end-1)), 'color', col, 'marker', 'o', 'markersize', 5, 'Linewidth', 2, 'LineStyle', 'none')
end % if: trajectory


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% draw predicted trajectory %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if(display.image.prediction)
  future_pos      = track.future.pos; % get tracked object center
  future_pos      = Tr^-1*[future_pos;ones(1,size(future_pos,2))]; % transform object center track in current image coordinate system
  future_pos(2,:) = -future_pos(2,:);
  % FOV is a polygon, which includes all visible points
  out_of_image = ~inpolygon(future_pos(1,:), future_pos(3,:), FOV(1,:), FOV(2,:)); % what is in the FOV polygon?
  % only draw if the predicted trajectory is visible
  if( ~all(out_of_image==1) )
    % project predicted trajectory back to the image and draw
    [u_future_pos, v_future_pos] = project3dToImage(future_pos(1,:), future_pos(2,:)-dh, future_pos(3,:), cam);

    u_future_pos(out_of_image) = []; v_future_pos(out_of_image)=[];
    plot(h, u_future_pos, v_future_pos, 'color', 'black', 'marker', 'none', 'markersize', 5, 'Linewidth', 3, 'LineStyle', '-')
    plot(h, u_future_pos, v_future_pos, 'color', col, 'marker', 'none', 'markersize', 5, 'Linewidth', 2, 'LineStyle', '--')
  end
  % DEBUG:
  %   figure;hold on;
  %   plot(future_pos(1,:), future_pos(3,:),'g')
  %   plot(future_pos(1,end), future_pos(3,end),'.g','markersize',20)
  %   plot(future_t(1,end), future_t(3,end),'.b','markersize',20)
end
end % function bbox_image

function bbox_birdsview(h, track, ID, col, Tr, deltaT)
% BBOX_BIRDSVIEW plots object in birdsview

global display

% transform position and velocity to image coordinate system
pos = Tr^-1*           [track.kalman.pos(1,:); track.kalman.pos(2,:); track.kalman.pos(3,:); ones(1,size(track.kalman.pos(3,:),2))];
raw = Tr^-1*           [track.raw.pos(1,:);    track.raw.pos(2,:);    track.raw.pos(3,:); ones(1,size(track.raw.pos(3,:),2))];
t   = Tr(1:3,1:3)^-1 * [track.kalman.t(1,end), track.kalman.t(2,end), track.kalman.t(3,end)]';

% plot all pts belonging to one object, only if there was a measurement
if( track.prediction == 0 && display.ics.trackedFeatures)
  plot(h, track.detections.pos(1,:), track.detections.pos(3,:), '.','color', col, 'MarkerSize',10);
end

% plot velocity for all pts belonging to one object
if( track.prediction == 0 && display.ics.trackedFeaturesVelocity )
  quiver(h, track.detections.pos(1,:),      track.detections.pos(3,:),...
            track.detections.t(1,:)/deltaT, track.detections.t(3,:)/deltaT, 0)
end

% get bounding box vertices
vertices(:,1) = [-track.kalman.dim(1,end)/2,  track.kalman.dim(1,end)/2, track.kalman.dim(1,end)/2, -track.kalman.dim(1,end)/2, -track.kalman.dim(1,end)/2]';
vertices(:,2) = [-track.kalman.dim(3,end)/2, -track.kalman.dim(3,end)/2, track.kalman.dim(3,end)/2,  track.kalman.dim(3,end)/2, -track.kalman.dim(3,end)/2]';

% prepare rotation of birdsview boxes
% rotation angle
alpha = atan( t(1)/t(3) );
% set 2D rotational matrix
R=[cos(alpha), -sin(alpha);
   sin(alpha), cos(alpha)];

% plot tracked object center
plot(h, pos(1,end), pos(3,end), '.', 'MarkerSize', 15, 'color', col)

% plot velocity for object center
quiver(h, pos(1,end), pos(3,end), t(1)/deltaT, t(3)/deltaT, 0, 'color', col) % 0: do not scale!

% plot trajectory
plot(h, pos(1,1:end-1), pos(3,1:end-1), '-', 'LineWidth', 2, 'color', col)


% plot raw measurements
plot(h, raw(1,1:end), raw(3,1:end), 'o', 'MarkerSize', 5, 'color', col)

% plot bounding box
% rotated (consider rotation around object center and not! the coordinate system)
vertices = (R^-1*vertices')' + repmat([pos(1,end), pos(3,end)], size(vertices,1), 1);

line(vertices(:,1), vertices(:,2),'color', col, 'Linewidth', 3, 'parent',h)

% show object ID below the lower right corner sign*(max(abs(bbox_Xpos))
if(display.image.ID)
  x_pos = sign(vertices(:,1))*min(abs(vertices(:,1))) - 0.5;
  text(x_pos(1), min(vertices(:,2)) - 0.5, num2str(ID),...
    'parent', h, 'color', col, 'FontSize', 14, 'FontWeight', 'bold', 'VerticalAlignment', 'top')
end

end 