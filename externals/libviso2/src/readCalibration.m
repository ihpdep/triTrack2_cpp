%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright 2013. All rights reserved.                                    %
% Institute of Measurement and Control Systems                            %
% Karlsruhe Institute of Technology, Germany                              %
%                                                                         %
% This file is part of triTrack2.                                         %
% Authors:  Philip Lenz                                                   %
%           Please send any bugreports to lenz@kit.edu                    % 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [ calib ] = readCalibration( calib_dir,seq_idx,cam)
%CALIBWRAPPER Summary of this function goes here
%   Detailed explanation goes here

if ~exist('cam','var'), cam=0; end
  try % benchmark calib
    % left gray camera in KITTI
    % cam = 0;
    fid = fopen(sprintf('%s/%04d.txt',calib_dir,seq_idx));
    % load 3x4 projection matrix
    C = textscan(fid,'%s %f %f %f %f %f %f %f %f %f %f %f %f',4);
    for i=0:11
      P0(floor(i/4)+1,mod(i,4)+1) = C{i+2}(cam+1);
    end
    fclose(fid);

    % rightgray camera in KITTI
    cam = cam+1;
    % fid = fopen(sprintf('%s/%04d.txt',calib_dir,seq_idx));
    % load 3x4 projection matrix
    % C = textscan(fid,'%s %f %f %f %f %f %f %f %f %f %f %f %f',4);
    for i=0:11
      P1(floor(i/4)+1,mod(i,4)+1) = C{i+2}(cam+1);
    end
    % fclose(fid);
  catch % raw calib
    fid = fopen(sprintf('%s/calib_cam_to_cam.txt',calib_dir));
    % load 3x4 projection matrix
    C = textscan(fid,'%s %f %f %f %f %f %f %f %f %f %f %f %f',1,'headerlines',8*(cam+1)+1);
    for i=0:11
      P0(floor(i/4)+1,mod(i,4)+1) = C{i+2};
    end
    fclose(fid);
    % rightgray camera in KITTI
    cam = cam+1;
    fid = fopen(sprintf('%s/calib_cam_to_cam.txt',calib_dir));
    % load 3x4 projection matrix
    C = textscan(fid,'%s %f %f %f %f %f %f %f %f %f %f %f %f',1,'headerlines',8*(cam+1)+1);
    for i=0:11
      P1(floor(i/4)+1,mod(i,4)+1) = C{i+2};
    end
    fclose(fid);
  end

  % wrap calib
  calib.P1=P0;
  calib.P2=P1;
  calib.f=calib.P1(1,1);
  calib.cu=calib.P1(1,3);
  %calib.fy=calib.P1(2,2);
  %calib.fy=-calib.P1(2,2);
  calib.cv=calib.P1(2,3);
  calib.base=calib.P2(1,4)/calib.f;
  
end

