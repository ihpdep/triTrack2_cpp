%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright 2013. All rights reserved.                                    %
% Institute of Measurement and Control Systems                            %
% Karlsruhe Institute of Technology, Germany                              %
%                                                                         %
% This file is part of triTrack2.                                         %
% Authors:  Philip Lenz                                                   %
%           Please send any bugreports to lenz@kit.edu                    % 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function trackSmoothing(track)

if(~exist('track','var'))
  load track.mat
end

figure()
hold on
col={'r','g','b', 'm' ,'y','c'};

for i=1:numel(track)
  if(size(track{1},2)>5)
    plot( track{i}(3,:), track{i}(1,:),'.', 'color', col{mod(i,numel(col))+1} )
    sp = spaps(track{i}(3,:), track{i}(1,:),0.5);
    fnplt(sp, 'color', col{mod(i,numel(col))+1} )
  end
end

end