%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright 2013. All rights reserved.                                    %
% Institute of Measurement and Control Systems                            %
% Karlsruhe Institute of Technology, Germany                              %
%                                                                         %
% This file is part of triTrack2.                                         %
% Authors:  Philip Lenz                                                   %
%           Please send any bugreports to lenz@kit.edu                    % 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function tracks = mergeTracks(tracks, Tr, verbose) 
% MERGETRACKS greedily merges tracks with overlap on the projected ground
% plane
%
% Tr = Tr_total{end}

  for itTrack=1:numel(tracks)
    track = tracks{itTrack};
    % only merge active tracks
    if track.active~=1, continue; end
    % bounding box
    vertices(1:2,1)       = -track.kalman.dim(1,end)/2;
    vertices(3:4,1)       =  track.kalman.dim(1,end)/2;
    vertices(1:4,2)       = -track.kalman.dim(2,end)/2;
    vertices(1:2:4,3)     = -track.kalman.dim(3,end)/2;
    vertices(2:2:4,3)     =  track.kalman.dim(3,end)/2;
    tmp       = objectToWorld(vertices, track, Tr);
    track_box = [min(tmp(:,[1,3])),max(tmp(:,[1,3]))];
    % candidates are all further tracks
    for itC=itTrack+1:numel(tracks)
      candidate = tracks{itC};
      if candidate.active~=1, continue; end

      % bounding box
      vertices(1:2,1)       = -candidate.kalman.dim(1,end)/2;
      vertices(3:4,1)       =  candidate.kalman.dim(1,end)/2;
      vertices(1:4,2)       = -candidate.kalman.dim(2,end)/2;
      vertices(1:2:4,3)     = -candidate.kalman.dim(3,end)/2;
      vertices(2:2:4,3)     =  candidate.kalman.dim(3,end)/2;
      tmp           = objectToWorld(vertices, candidate, Tr);
      candidate_box = [min(tmp(:,[1,3])),max(tmp(:,[1,3]))];
      overlap = boxoverlap(track_box, candidate_box);
      
      % boxes overlap -> merge everything in tracks{itTrack} and set
      % candidate to inactive, remembering the old track number
      if overlap>0.15
        % TODO: Merge whole trajectory, not just last frame
        if verbose, fprintf('  . merged track %d into %d\n', itC, itTrack); end
        % merge frames
        % tracks{itTrack}.frames = sort(unique([track.frame, candidate.frame]));
        % set candidate track to inactive (merged into itTrack)
        tracks{itC}.active = -10-itTrack;
        % merge raw data
        tracks{itTrack}.raw.pos(:,end) = (track.kalman.pos(:,end)+candidate.kalman.pos(:,end))/2;
        tracks{itTrack}.raw.t(:,end)   = (track.kalman.t(:,end)+candidate.kalman.t(:,end))/2;
        % merge detections
        tracks{itTrack}.detections.pos = [track.detections.pos, candidate.detections.pos];
        % merge future
        tracks{itTrack}.future.pos = (track.future.pos + candidate.future.pos)/2;
        tracks{itTrack}.future.t   = (track.future.t + candidate.future.t)/2;
        % merge kalman
        tracks{itTrack}.kalman.pos(:,end) = (track.kalman.pos(:,end)+candidate.kalman.pos(:,end))/2;
        tracks{itTrack}.kalman.t(:,end)   = (track.kalman.t(:,end)+candidate.kalman.t(:,end))/2;
        tracks{itTrack}.kalman.posP{end}  = (track.kalman.posP{end}+candidate.kalman.posP{end})/2;
        tracks{itTrack}.kalman.dim(:,end) = max(track.kalman.dim(:,end) , candidate.kalman.dim(:,end));
        tracks{itTrack}.kalman.dimP{end}  = max(track.kalman.dimP{end},candidate.kalman.dimP{end});
      end
    end
  end
end

function o = boxoverlap(a,b)
  x1 = max(a(:,1), b(1));
  y1 = max(a(:,2), b(2));
  x2 = min(a(:,3), b(3));
  y2 = min(a(:,4), b(4));

  w = x2-x1+1;
  h = y2-y1+1;
  inter = w.*h;
  aarea = (a(:,3)-a(:,1)+1) .* (a(:,4)-a(:,2)+1);
  barea = (b(3)-b(1)+1) * (b(4)-b(2)+1);
  % intersection over union overlap
  o = inter ./ (aarea+barea-inter);
  % set invalid entries to 0 overlap
  o(w <= 0) = 0;
  o(h <= 0) = 0;
end