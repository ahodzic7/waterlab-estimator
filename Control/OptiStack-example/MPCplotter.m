function [outputArg1,outputArg2] = MPCplotter(NumberOfSteps,inputArg2)
%MPCPLOTTER Summary of this function goes here
%   Detailed explanation goes here
% Animate and add animation frame to the movie structure

fig = figure('Renderer', 'painters', 'Position', [300 150 960 540]);
pos = get(fig, 'Position');
width = pos(3);
height = pos(4);
mov = zeros(height, width, 1, NumberOfSteps, 'uint8');

for index = 1:NumberOfSteps
   % Update XData and YData
   set(hh1(1), 'XData', T(id)          , 'YData', ang(id, 1));
   set(hh1(2), 'XData', T(id)          , 'YData', ang(id, 2));
   set(hh2(1), 'XData', [0, x(id, 1)]  , 'YData', [0, y(id, 1)]);
   set(hh2(2), 'XData', x(id, :)       , 'YData', y(id, :));
   set(ht, 'String', sprintf('Time: %0.2f sec', T(id)));

   % Get frame as an image
   f = getframe(gcf);

   % Create a colormap for the first frame. For the rest of the frames, use
   % the same colormap
   if id == 1
      [mov(:,:,1,index), map] = rgb2ind(f.cdata, 256, 'nodither');
   else
      mov(:,:,1,index) = rgb2ind(f.cdata, map, 'nodither');
   end
end

% Create animated GIF
imwrite(mov, map, 'animation.gif', 'DelayTime', 0, 'LoopCount', inf)
end

