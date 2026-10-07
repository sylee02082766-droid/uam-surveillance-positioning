function [buildings_for_physics, buildings_for_draw, mountains_for_physics] = loadObstacleData(cfg)
% LOADOBSTACLEDATA  Load building and mountain obstacles.

bfile = cfg.building_filename;
mfile = cfg.mountain_filename;
if ~isfile(bfile) && isfile(fullfile('UAM', bfile))
    bfile = fullfile('UAM', bfile);
end
if ~isfile(mfile) && isfile(fullfile('UAM', mfile))
    mfile = fullfile('UAM', mfile);
end

opts = detectImportOptions(bfile);
opts.VariableNames = {'ID','X','Y','W','H','Z'};
T = readtable(bfile, opts);

effective_radius = sqrt(T.W.^2 + T.H.^2) / 2;
buildings_for_physics = [T.X, T.Y, effective_radius, T.Z];
buildings_for_draw = T(:, {'X','Y','W','H','Z'});

% Mountain format: name, x, y, height. They are represented as simple
% cylindrical obstacles with fixed 200 m radius for LOS/NLOS testing.
M = readtable(mfile, 'Delimiter', ',', 'ReadVariableNames', false, 'Encoding','UTF-8');
if width(M) >= 4
    x = M{:,2}; y = M{:,3}; h = M{:,4};
    mountains_for_physics = [x, y, ones(height(M),1)*200, h];
else
    mountains_for_physics = zeros(0,4);
end
end
