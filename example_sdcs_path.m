function example_sdcs_path()
% example_sdcs_path
% Taxi mission planner on the Quanser SDCS roadmap.
%
% Mission:
%   taxi hub (node 10) -> pickup node -> dropoff node -> taxi hub (node 10)
%
% Exports to MAT file and base workspace:
%   pickupXY, dropoffXY, taxiHubXY
%   pickupPathIndex, dropoffPathIndex, taxiHubPathIndex
%   pickupStopRadius, dropoffStopRadius
%   pickupStopTime, dropoffStopTime

    clear; clc; close all;

    %% =========================
    % User settings
    %% =========================
    leftHandTraffic = false;
    useSmallMap = false;
    stepSize = 0.01;
    doPlot = true;
    saveOutput = true;
    outputMatFile = 'sdcs_taxi_mission.mat';

    % ============================================================
    % MISSION NODES (0-based indexing, same as Quanser/Python)
    % ============================================================
    taxiHubNode0 = 10;
    pickupNode0  = 20;
    dropoffNode0 = 9;

    %% =========================
    % Ride event settings
    %% =========================
    pickupStopRadius   = 0.20;   % [m]
    dropoffStopRadius  = 0.20;   % [m]
    pickupStopTime     = 2.0;    % [s]
    dropoffStopTime    = 2.0;    % [s]

    %% =========================
    % OPTIONAL PATH CORRECTION
    %% =========================
    usePathCorrection = true;
    loadCorrectionFromFile = false;
    correctionFile = 'astar_path_correction.mat';

    manualCorr.s             = 1.0000;
    manualCorr.rotDeg        = 0.0;
    manualCorr.tx            = -0.025;
    manualCorr.ty            = -0.05;

    manualCorr.xLeftScale    = 1.00
    manualCorr.xRightScale   = 1.015;
    manualCorr.yTopScale     = 0.998;
    manualCorr.yBottomScale  = 1.02;

    manualCorr.lateralOffset = 0.0005
    manualCorr.smoothWin     = 1;
    manualCorr.keepEndpoints = true;

    %% =========================
    % Build roadmap
    %% =========================
    roadmap = build_sdcs_roadmap(leftHandTraffic, useSmallMap, stepSize);

    %% =========================
    % Validate node IDs
    %% =========================
    maxNode0 = size(roadmap.nodes,1) - 1;
    validate_node_id(taxiHubNode0, maxNode0, 'taxiHubNode0');
    validate_node_id(pickupNode0,  maxNode0, 'pickupNode0');
    validate_node_id(dropoffNode0, maxNode0, 'dropoffNode0');

    %% =========================
    % Plan each leg with A*
    %% =========================
    leg1 = roadmap_find_shortest_node_sequence(roadmap, taxiHubNode0, pickupNode0);
    leg2 = roadmap_find_shortest_node_sequence(roadmap, pickupNode0, dropoffNode0);
    leg3 = roadmap_find_shortest_node_sequence(roadmap, dropoffNode0, taxiHubNode0);

    if isempty(leg1)
        error('A* failed for leg 1: taxiHubNode0 -> pickupNode0');
    end
    if isempty(leg2)
        error('A* failed for leg 2: pickupNode0 -> dropoffNode0');
    end
    if isempty(leg3)
        error('A* failed for leg 3: dropoffNode0 -> taxiHubNode0');
    end

    %% =========================
    % Combine mission node sequence
    %% =========================
    missionNodeSequence = concatenate_node_sequences({leg1, leg2, leg3});

    fprintf('Leg 1 (hub -> pickup)     = ['); fprintf('%d ', leg1); fprintf(']\n');
    fprintf('Leg 2 (pickup -> dropoff) = ['); fprintf('%d ', leg2); fprintf(']\n');
    fprintf('Leg 3 (dropoff -> hub)    = ['); fprintf('%d ', leg3); fprintf(']\n');
    fprintf('Mission node sequence     = ['); fprintf('%d ', missionNodeSequence); fprintf(']\n');

    %% =========================
    % Generate RAW path for each leg
    %% =========================
    leg1XY_raw = roadmap_generate_path(roadmap, leg1);
    leg2XY_raw = roadmap_generate_path(roadmap, leg2);
    leg3XY_raw = roadmap_generate_path(roadmap, leg3);

    if isempty(leg1XY_raw)
        error('Path generation failed for leg 1.');
    end
    if isempty(leg2XY_raw)
        error('Path generation failed for leg 2.');
    end
    if isempty(leg3XY_raw)
        error('Path generation failed for leg 3.');
    end

    %% =========================
    % Generate RAW full mission path
    %% =========================
    pathXY_raw = roadmap_generate_path(roadmap, missionNodeSequence);

    if isempty(pathXY_raw)
        error('Path generation failed after mission node-sequence planning.');
    end

    %% =========================
    % Optional correction
    %% =========================
    if usePathCorrection
        if loadCorrectionFromFile
            if ~isfile(correctionFile)
                error('Correction file not found: %s', correctionFile);
            end
            C = load(correctionFile);
            if ~isfield(C, 'corr')
                error('Correction file must contain a variable named "corr".');
            end
            corr = C.corr;
        else
            corr = manualCorr;
        end

        pathXY_corrected = correct_generated_path(pathXY_raw, corr);
        leg1XY_corrected = correct_generated_path(leg1XY_raw, corr);
        leg2XY_corrected = correct_generated_path(leg2XY_raw, corr);
        leg3XY_corrected = correct_generated_path(leg3XY_raw, corr);

        % Local tweaks only where needed
        pathXY_corrected = local_path_tweak(pathXY_corrected, roadmap);
        leg1XY_corrected = local_path_tweak(leg1XY_corrected, roadmap);
        leg2XY_corrected = local_path_tweak(leg2XY_corrected, roadmap);
        leg3XY_corrected = local_path_tweak(leg3XY_corrected, roadmap);

    else
        corr = [];
        pathXY_corrected = pathXY_raw;
        leg1XY_corrected = leg1XY_raw;
        leg2XY_corrected = leg2XY_raw;
        leg3XY_corrected = leg3XY_raw;
    end

    %% =========================
    % Final exported path
    %% =========================
    pathXY = pathXY_corrected;
    path_x_custom = pathXY(:,1);
    path_y_custom = pathXY(:,2);

    leg1_x = leg1XY_corrected(:,1); leg1_y = leg1XY_corrected(:,2);
    leg2_x = leg2XY_corrected(:,1); leg2_y = leg2XY_corrected(:,2);
    leg3_x = leg3XY_corrected(:,1); leg3_y = leg3XY_corrected(:,2);

    %% =========================
    % Pickup / Dropoff event points on final path
    %% =========================
    pickupMat  = pickupNode0 + 1;
    dropoffMat = dropoffNode0 + 1;
    taxiHubMat = taxiHubNode0 + 1;

    pickupNodeXY_raw  = roadmap.nodes(pickupMat,1:2);
    dropoffNodeXY_raw = roadmap.nodes(dropoffMat,1:2);
    taxiHubNodeXY_raw = roadmap.nodes(taxiHubMat,1:2);

    pickupPathIndex  = nearest_point_index(pathXY, pickupNodeXY_raw);
    dropoffPathIndex = nearest_point_index(pathXY, dropoffNodeXY_raw);
    taxiHubPathIndex = nearest_point_index(pathXY, taxiHubNodeXY_raw);

    pickupXY  = pathXY(pickupPathIndex,:);
    dropoffXY = pathXY(dropoffPathIndex,:);
    taxiHubXY = pathXY(taxiHubPathIndex,:);

    % Optional ride flag along path
    pathRideFlag = zeros(size(pathXY,1),1,'uint8');
    if pickupPathIndex < dropoffPathIndex
        if pickupPathIndex + 1 <= dropoffPathIndex
            pathRideFlag((pickupPathIndex+1):dropoffPathIndex) = uint8(1);
        end
        if dropoffPathIndex < size(pathXY,1)
            pathRideFlag((dropoffPathIndex+1):end) = uint8(2);
        end
    end

    %% =========================
    % Export to base workspace for Simulink
    %% =========================
    assignin('base', 'pickupXY', pickupXY);
    assignin('base', 'dropoffXY', dropoffXY);
    assignin('base', 'taxiHubXY', taxiHubXY);

    assignin('base', 'pickupPathIndex', pickupPathIndex);
    assignin('base', 'dropoffPathIndex', dropoffPathIndex);
    assignin('base', 'taxiHubPathIndex', taxiHubPathIndex);

    assignin('base', 'pickupStopRadius', pickupStopRadius);
    assignin('base', 'dropoffStopRadius', dropoffStopRadius);
    assignin('base', 'pickupStopTime', pickupStopTime);
    assignin('base', 'dropoffStopTime', dropoffStopTime);

    assignin('base', 'pathXY', pathXY);
    assignin('base', 'path_x_custom', path_x_custom);
    assignin('base', 'path_y_custom', path_y_custom);
    assignin('base', 'pathRideFlag', pathRideFlag);

    fprintf('pickupXY  = [%.4f %.4f]\n', pickupXY(1), pickupXY(2));
    fprintf('dropoffXY = [%.4f %.4f]\n', dropoffXY(1), dropoffXY(2));
    fprintf('taxiHubXY = [%.4f %.4f]\n', taxiHubXY(1), taxiHubXY(2));

    %% =========================
    % Save
    %% =========================
    if saveOutput
        save(outputMatFile, ...
            'pathXY_raw', ...
            'pathXY_corrected', ...
            'pathXY', ...
            'path_x_custom', ...
            'path_y_custom', ...
            'leg1XY_raw', 'leg2XY_raw', 'leg3XY_raw', ...
            'leg1XY_corrected', 'leg2XY_corrected', 'leg3XY_corrected', ...
            'leg1_x', 'leg1_y', ...
            'leg2_x', 'leg2_y', ...
            'leg3_x', 'leg3_y', ...
            'missionNodeSequence', ...
            'leg1', 'leg2', 'leg3', ...
            'taxiHubNode0', ...
            'pickupNode0', ...
            'dropoffNode0', ...
            'pickupXY', ...
            'dropoffXY', ...
            'taxiHubXY', ...
            'pickupPathIndex', ...
            'dropoffPathIndex', ...
            'taxiHubPathIndex', ...
            'pickupStopRadius', ...
            'dropoffStopRadius', ...
            'pickupStopTime', ...
            'dropoffStopTime', ...
            'pathRideFlag', ...
            'corr');
        fprintf('Saved %s\n', outputMatFile);
    end

    %% =========================
    % Plot
    %% =========================
    if doPlot
        figure('Name','Taxi Mission Path');
        hold on; axis equal; grid on;

        plot_roadmap(roadmap);
        plot(pathXY(:,1), pathXY(:,2), 'm-', 'LineWidth', 2.5, 'DisplayName', 'Final Path');

        plot(pickupXY(1), pickupXY(2), 'bs', 'MarkerFaceColor', 'b', 'MarkerSize', 9, ...
            'DisplayName', 'pickupXY');
        plot(dropoffXY(1), dropoffXY(2), 'md', 'MarkerFaceColor', 'm', 'MarkerSize', 9, ...
            'DisplayName', 'dropoffXY');
        plot(taxiHubXY(1), taxiHubXY(2), 'gs', 'MarkerFaceColor', 'g', 'MarkerSize', 9, ...
            'DisplayName', 'taxiHubXY');

        xlabel('x [m]');
        ylabel('y [m]');
        title('Final Path with Pickup / Dropoff Event Points');
        legend('Location','best');
        hold off;
    end
end

%% =========================================================================
function idx = nearest_point_index(pathXY, pointXY)
    d2 = sum((pathXY - pointXY).^2, 2);
    [~, idx] = min(d2);
end

%% =========================================================================
function pathXY_out = correct_generated_path(pathXY_in, corr)

    pathXY = sanitize_xy(pathXY_in);

    if isempty(pathXY)
        pathXY_out = pathXY;
        return;
    end

    pathXY = apply_similarity_2d_manual(pathXY, corr.s, corr.rotDeg, corr.tx, corr.ty);
    pathXY = scale_path_sides_xy(pathXY, corr);

    if isfield(corr, 'lateralOffset') && abs(corr.lateralOffset) > 0
        pathXY = offset_path_left_normal(pathXY, corr.lateralOffset);
    end

    if isfield(corr, 'smoothWin') && corr.smoothWin > 1
        keepEndpoints = false;
        if isfield(corr, 'keepEndpoints')
            keepEndpoints = logical(corr.keepEndpoints);
        end
        pathXY = smooth_polyline_ma(pathXY, corr.smoothWin, keepEndpoints);
    end

    pathXY_out = sanitize_xy(pathXY);
end

%% =========================================================================
function XY2 = apply_similarity_2d_manual(XY, s, rotDeg, tx, ty)

    theta = deg2rad(rotDeg);
    R = [cos(theta), -sin(theta);
         sin(theta),  cos(theta)];

    XY2 = (s * (R * XY.')).';
    XY2(:,1) = XY2(:,1) + tx;
    XY2(:,2) = XY2(:,2) + ty;
end

%% =========================================================================
function XY2 = scale_path_sides_xy(XY, corr)

    XY2 = XY;

    xCenter = 0.5 * (min(XY(:,1)) + max(XY(:,1)));
    yCenter = 0.5 * (min(XY(:,2)) + max(XY(:,2)));

    xLeftScale = 1.0;
    xRightScale = 1.0;
    yTopScale = 1.0;
    yBottomScale = 1.0;

    if isfield(corr, 'xLeftScale'),    xLeftScale = corr.xLeftScale;       end
    if isfield(corr, 'xRightScale'),   xRightScale = corr.xRightScale;     end
    if isfield(corr, 'yTopScale'),     yTopScale = corr.yTopScale;         end
    if isfield(corr, 'yBottomScale'),  yBottomScale = corr.yBottomScale;   end

    idxLeft   = XY(:,1) < xCenter;
    idxRight  = XY(:,1) > xCenter;
    idxBottom = XY(:,2) < yCenter;
    idxTop    = XY(:,2) > yCenter;

    XY2(idxLeft,1)   = xCenter + xLeftScale   * (XY(idxLeft,1)   - xCenter);
    XY2(idxRight,1)  = xCenter + xRightScale  * (XY(idxRight,1)  - xCenter);
    XY2(idxBottom,2) = yCenter + yBottomScale * (XY(idxBottom,2) - yCenter);
    XY2(idxTop,2)    = yCenter + yTopScale    * (XY(idxTop,2)    - yCenter);
end

%% =========================================================================
function XY2 = offset_path_left_normal(XY, d)

    XY = sanitize_xy(XY);
    n = size(XY,1);

    if n < 2
        XY2 = XY;
        return;
    end

    tang = zeros(n,2);
    tang(1,:)   = XY(2,:)   - XY(1,:);
    tang(end,:) = XY(end,:) - XY(end-1,:);

    for i = 2:n-1
        tang(i,:) = XY(i+1,:) - XY(i-1,:);
    end

    XY2 = XY;

    for i = 1:n
        t = tang(i,:);
        nt = norm(t);

        if nt < 1e-12
            continue;
        end

        t = t / nt;
        nL = [-t(2), t(1)];
        XY2(i,:) = XY(i,:) + d * nL;
    end
end

%% =========================================================================
function XY2 = local_path_tweak(XY, roadmap)
% local_path_tweak
% Small local corrections only:
% - near node 9: move path slightly upward
% - near lower-right loop: move path slightly outward/right

    XY2 = XY;

    % -----------------------------
    % 1) Fix near node 9
    % -----------------------------
    n9 = roadmap.nodes(9+1, 1:2);

    sigma9 = 0.28;   % influence radius [m]
    dy9    = 0.035;  % upward shift [m]

    d29 = sum((XY2 - n9).^2, 2);
    w9  = exp(-d29 / (2*sigma9^2));

    XY2(:,2) = XY2(:,2) + dy9 * w9;

    % -----------------------------
    % 2) Fix lower-right loop
    % -----------------------------
    % Use node 4 / 5 region as center of the lower-right loop
    n4 = roadmap.nodes(4+1, 1:2);
    n5 = roadmap.nodes(5+1, 1:2);
    cLoop = 0.5 * (n4 + n5);

    sigmaLoop = 0.45;   % influence radius [m]
    dxLoop    = 0.030;  % move outward/right [m]

    d2Loop = sum((XY2 - cLoop).^2, 2);
    wLoop  = exp(-d2Loop / (2*sigmaLoop^2));

    XY2(:,1) = XY2(:,1) + dxLoop * wLoop;
end

%% =========================================================================
function XYs = smooth_polyline_ma(XY, win, keepEndpoints)

    XY = sanitize_xy(XY);
    n = size(XY,1);

    if n < 3 || win <= 1
        XYs = XY;
        return;
    end

    if mod(win,2) == 0
        win = win + 1;
    end

    halfW = floor(win/2);
    XYs = XY;

    for i = 1:n
        i1 = max(1, i-halfW);
        i2 = min(n, i+halfW);
        XYs(i,:) = mean(XY(i1:i2,:), 1);
    end

    if keepEndpoints
        XYs(1,:) = XY(1,:);
        XYs(end,:) = XY(end,:);
    end
end

%% =========================================================================
function XY = sanitize_xy(XY)

    if isempty(XY)
        return;
    end

    XY = double(XY);

    if size(XY,2) ~= 2
        error('sanitize_xy expects an Nx2 array.');
    end

    good = all(isfinite(XY), 2);
    XY = XY(good,:);

    if isempty(XY)
        return;
    end

    keep = true(size(XY,1),1);
    if size(XY,1) > 1
        keep(2:end) = any(abs(diff(XY,1,1)) > 1e-12, 2);
    end
    XY = XY(keep,:);
end

%% =========================================================================
function validate_node_id(nodeID0, maxNode0, varName)
    if nodeID0 < 0 || nodeID0 > maxNode0
        error('%s must be between 0 and %d', varName, maxNode0);
    end
end

%% =========================================================================
function missionSeq = concatenate_node_sequences(seqCell)

    missionSeq = [];

    for k = 1:numel(seqCell)
        seq = seqCell{k};

        if isempty(seq)
            missionSeq = [];
            return;
        end

        seq = seq(:).';

        if isempty(missionSeq)
            missionSeq = seq;
        else
            if missionSeq(end) == seq(1)
                missionSeq = [missionSeq, seq(2:end)]; %#ok<AGROW>
            else
                missionSeq = [missionSeq, seq]; %#ok<AGROW>
            end
        end
    end
end

%% =========================================================================
function roadmap = build_sdcs_roadmap(leftHandTraffic, useSmallMap, stepSize)

    scale   = 0.002035;
    xOffset = 1134;
    yOffset = 2363;

    innerLaneRadius     = 305.5 * scale;
    outerLaneRadius     = 438   * scale;
    trafficCircleRadius = 333   * scale;
    oneWayStreetRadius  = 350   * scale;
    kinkStreetRadius    = 375   * scale;

    halfPi = pi/2;

    if leftHandTraffic
        nodePoses = [
            1134, 2427,  halfPi;
            1134, 2323,  halfPi;
            1266, 2323, -halfPi;
            1688, 2896,  pi;
            1688, 2763,  0;
            2242, 2323, -halfPi;
            2109, 2323,  halfPi;
            1741, 1822,  0;
            1634, 1955,  pi;
             766, 1822,  0;
             766, 1955,  pi;
             504, 2589, 138*pi/180;
        ];

        if ~useSmallMap
            nodePoses = [nodePoses;
                1134, 1428,  halfPi;
                1266, 1454, -halfPi;
                2242, 1454, -halfPi;
                2109, 1200,  halfPi;
                1854.5, 814.5, 170.6*pi/180;
                1580, 540,  99.4*pi/180;
                1440, 856,  42*pi/180;
                1523, 958, -138*pi/180;
                1400, 153,  0;
                1134, 286,  pi;
                 159, 905,  halfPi;
                 291, 905, -halfPi;
            ];
        end

        edgeConfigs = [
             0,  1, 0;
             1,  7, outerLaneRadius;
             1, 10, innerLaneRadius;
             2,  4, innerLaneRadius;
             2, 11, innerLaneRadius;
             3,  0, outerLaneRadius;
             3, 11, outerLaneRadius;
             4,  6, innerLaneRadius;
             5,  3, outerLaneRadius;
             6,  8, innerLaneRadius;
             7,  5, outerLaneRadius;
             8,  2, innerLaneRadius;
             8, 10, 0;
             9,  2, outerLaneRadius;
             9,  7, 0;
            10,  1, innerLaneRadius;
            11,  9, oneWayStreetRadius;
        ];

        if ~useSmallMap
            edgeConfigs = [edgeConfigs;
                 1, 12, 0;
                 6, 15, 0;
                 7, 15, innerLaneRadius;
                 8, 12, outerLaneRadius;
                 9, 12, innerLaneRadius;
                10, 22, outerLaneRadius;
                11, 22, outerLaneRadius;
                12, 18, outerLaneRadius;
                13,  2, 0;
                13,  7, innerLaneRadius;
                13, 10, outerLaneRadius;
                14,  5, 0;
                14,  8, outerLaneRadius;
                15, 16, innerLaneRadius;
                16, 17, trafficCircleRadius;
                16, 19, innerLaneRadius;
                17, 14, trafficCircleRadius;
                17, 16, trafficCircleRadius;
                17, 21, innerLaneRadius;
                18, 17, innerLaneRadius;
                19, 13, innerLaneRadius;
                20, 14, trafficCircleRadius;
                20, 16, trafficCircleRadius;
                21, 23, innerLaneRadius;
                22, 20, outerLaneRadius;
                23,  9, innerLaneRadius;
            ];
        end

    else
        nodePoses = [
            1134, 2299, -halfPi;
            1266, 2323,  halfPi;
            1688, 2896,  0;
            1688, 2763,  pi;
            2242, 2323,  halfPi;
            2109, 2323, -halfPi;
            1632, 1822,  pi;
            1741, 1955,  0;
             766, 1822,  pi;
             766, 1955,  0;
             504, 2589, -42*pi/180;
        ];

        if ~useSmallMap
            nodePoses = [nodePoses;
                1134, 1300, -halfPi;
                1134, 1454, -halfPi;
                1266, 1454,  halfPi;
                2242, 905,    halfPi;
                2109, 1454,  -halfPi;
                1580, 540,   -80.6*pi/180;
                1854.4, 814.5, -9.4*pi/180;
                1440, 856,   -138*pi/180;
                1523, 958,    42*pi/180;
                1134, 153,    pi;
                1134, 286,    0;
                 159, 905,   -halfPi;
                 291, 905,    halfPi;
            ];
        end

        edgeConfigs = [
             0,  2, outerLaneRadius;
             1,  7, innerLaneRadius;
             1,  8, outerLaneRadius;
             2,  4, outerLaneRadius;
             3,  1, innerLaneRadius;
             4,  6, outerLaneRadius;
             5,  3, innerLaneRadius;
             6,  0, outerLaneRadius;
             6,  8, 0;
             7,  5, innerLaneRadius;
             8, 10, oneWayStreetRadius;
             9,  0, innerLaneRadius;
             9,  7, 0;
            10,  1, innerLaneRadius;
            10,  2, innerLaneRadius;
        ];

        if ~useSmallMap
            edgeConfigs = [edgeConfigs;
                 1, 13, 0;
                 4, 14, 0;
                 6, 13, innerLaneRadius;
                 7, 14, outerLaneRadius;
                 8, 23, innerLaneRadius;
                 9, 13, outerLaneRadius;
                11, 12, 0;
                12,  0, 0;
                12,  7, outerLaneRadius;
                12,  8, innerLaneRadius;
                13, 19, innerLaneRadius;
                14, 16, trafficCircleRadius;
                14, 20, trafficCircleRadius;
                15,  5, outerLaneRadius;
                15,  6, innerLaneRadius;
                16, 17, trafficCircleRadius;
                16, 18, innerLaneRadius;
                17, 15, innerLaneRadius;
                17, 16, trafficCircleRadius;
                17, 20, trafficCircleRadius;
                18, 11, kinkStreetRadius;
                19, 17, innerLaneRadius;
                20, 22, outerLaneRadius;
                21, 16, innerLaneRadius;
                22,  9, outerLaneRadius;
                22, 10, outerLaneRadius;
                23, 21, innerLaneRadius;
            ];
        end
    end

    N = size(nodePoses,1);
    nodes = zeros(N,3);
    for i = 1:N
        nodes(i,1) = scale * (nodePoses(i,1) - xOffset);
        nodes(i,2) = scale * (yOffset - nodePoses(i,2));
        nodes(i,3) = nodePoses(i,3);
    end

    E = size(edgeConfigs,1);
    edges(E) = struct('from', [], 'to', [], 'radius', [], 'waypoints', [], 'length', []);
    outEdges = cell(N,1);

    for k = 1:E
        fromIdx = edgeConfigs(k,1) + 1;
        toIdx   = edgeConfigs(k,2) + 1;
        radius  = edgeConfigs(k,3);

        [wp, L] = scs_path(nodes(fromIdx,:).', nodes(toIdx,:).', radius, stepSize);

        edges(k).from = fromIdx;
        edges(k).to = toIdx;
        edges(k).radius = radius;
        edges(k).waypoints = wp;
        edges(k).length = L;

        outEdges{fromIdx}(end+1) = k; %#ok<AGROW>
    end

    roadmap.nodes = nodes;
    roadmap.edges = edges;
    roadmap.outEdges = outEdges;
end

%% =========================================================================
function nodeSequence = roadmap_find_shortest_node_sequence(roadmap, startNode0, goalNode0)

    startNode = startNode0 + 1;
    goalNode  = goalNode0 + 1;
    N = size(roadmap.nodes,1);

    if startNode == goalNode
        nodeSequence = startNode0;
        return;
    end

    gScore = inf(N,1);
    fScore = inf(N,1);
    cameFrom = zeros(N,1);
    openSet = false(N,1);
    closedSet = false(N,1);

    gScore(startNode) = 0;
    fScore(startNode) = heuristic_node(roadmap.nodes(startNode,:), roadmap.nodes(goalNode,:));
    openSet(startNode) = true;

    while any(openSet)
        openIdx = find(openSet);
        [~, k] = min(fScore(openIdx));
        current = openIdx(k);

        if current == goalNode
            seq = current;
            while seq(1) ~= startNode
                parent = cameFrom(seq(1));
                if parent == 0
                    nodeSequence = [];
                    return;
                end
                seq = [parent; seq]; %#ok<AGROW>
            end
            nodeSequence = (seq - 1).';
            return;
        end

        openSet(current) = false;
        closedSet(current) = true;

        edgeList = roadmap.outEdges{current};

        for j = 1:numel(edgeList)
            edgeID = edgeList(j);
            edge = roadmap.edges(edgeID);
            neighbor = edge.to;

            if closedSet(neighbor)
                continue;
            end

            tentativeG = gScore(current) + edge.length;

            if tentativeG < gScore(neighbor)
                cameFrom(neighbor) = current;
                gScore(neighbor) = tentativeG;
                fScore(neighbor) = tentativeG + heuristic_node(roadmap.nodes(neighbor,:), roadmap.nodes(goalNode,:));
                openSet(neighbor) = true;
            end
        end
    end

    nodeSequence = [];
end

%% =========================================================================
function h = heuristic_node(nodePose, goalPose)
    h = norm(goalPose(1:2) - nodePose(1:2));
end

%% =========================================================================
function pathXY = roadmap_generate_path(roadmap, nodeSequence0)

    if numel(nodeSequence0) < 2
        pathXY = roadmap.nodes(nodeSequence0(1)+1,1:2);
        return;
    end

    pathXY = [];

    for i = 1:(numel(nodeSequence0)-1)
        startNode = nodeSequence0(i) + 1;
        goalNode  = nodeSequence0(i+1) + 1;

        segmentXY = roadmap_find_shortest_path(roadmap, startNode, goalNode);

        if isempty(segmentXY)
            pathXY = [];
            return;
        end

        if isempty(pathXY)
            pathXY = segmentXY;
        else
            pathXY = [pathXY; segmentXY(2:end,:)]; %#ok<AGROW>
        end
    end
end

%% =========================================================================
function pathXY = roadmap_find_shortest_path(roadmap, startNode, goalNode)

    if startNode == goalNode
        pathXY = roadmap.nodes(startNode,1:2);
        return;
    end

    N = size(roadmap.nodes,1);

    gScore = inf(N,1);
    fScore = inf(N,1);
    cameFromNode = zeros(N,1);
    cameFromEdge = zeros(N,1);
    openSet = false(N,1);
    closedSet = false(N,1);

    gScore(startNode) = 0;
    fScore(startNode) = heuristic_node(roadmap.nodes(startNode,:), roadmap.nodes(goalNode,:));
    openSet(startNode) = true;

    while any(openSet)
        openIdx = find(openSet);
        [~, k] = min(fScore(openIdx));
        current = openIdx(k);

        if current == goalNode
            pathXY = reconstruct_path(roadmap, cameFromNode, cameFromEdge, startNode, goalNode);
            return;
        end

        openSet(current) = false;
        closedSet(current) = true;

        edgeList = roadmap.outEdges{current};

        for j = 1:numel(edgeList)
            edgeID = edgeList(j);
            edge = roadmap.edges(edgeID);
            neighbor = edge.to;

            if closedSet(neighbor)
                continue;
            end

            tentativeG = gScore(current) + edge.length;

            if tentativeG < gScore(neighbor)
                cameFromNode(neighbor) = current;
                cameFromEdge(neighbor) = edgeID;
                gScore(neighbor) = tentativeG;
                fScore(neighbor) = tentativeG + heuristic_node(roadmap.nodes(neighbor,:), roadmap.nodes(goalNode,:));
                openSet(neighbor) = true;
            end
        end
    end

    pathXY = [];
end

%% =========================================================================
function pathXY = reconstruct_path(roadmap, cameFromNode, cameFromEdge, startNode, goalNode)
    pathXY = roadmap.nodes(goalNode,1:2);
    current = goalNode;

    while current ~= startNode
        prev = cameFromNode(current);
        edgeID = cameFromEdge(current);

        if prev == 0 || edgeID == 0
            pathXY = [];
            return;
        end

        edge = roadmap.edges(edgeID);
        segXY = [roadmap.nodes(prev,1:2); edge.waypoints.'; pathXY];
        pathXY = segXY;
        current = prev;
    end

    keep = [true; any(abs(diff(pathXY,1,1)) > 1e-12, 2)];
    pathXY = pathXY(keep,:);
end

%% =========================================================================
function [path, path_length] = scs_path(startPose, endPose, radius, stepSize)

    p1 = startPose(1:2);
    th1 = startPose(3);
    p2 = endPose(1:2);
    th2 = endPose(3);

    t1 = [cos(th1); sin(th1)];
    t2 = [cos(th2); sin(th2)];

    if signed_angle(t1, p2 - p1) > 0
        dirSign = 1;
    else
        dirSign = -1;
    end

    n1 = radius * [-t1(2); t1(1)] * dirSign;
    n2 = radius * [-t2(2); t2(1)] * dirSign;

    tol = 0.01;

    if abs(wrap_to_pi(th2 - th1)) < tol
        v = p2 - p1;
        v_uv = v / norm(v);
        if 1 - abs(dot(t1, v_uv)) < tol
            c = p2 + n1;
        else
            path = [];
            path_length = inf;
            return;
        end

    elseif abs(wrap_to_pi(th2 - th1 + pi)) < tol
        v = (p2 + 2*n2) - p1;
        v_uv = v / norm(v);
        if 1 - abs(dot(t1, v_uv)) < tol
            s = dot(t1, v);
            if s < tol
                c = p1 + n1;
            else
                c = p2 + n2;
            end
        else
            path = [];
            path_length = inf;
            return;
        end

    else
        d1 = p1 + n1;
        d2 = p2 + n2;

        A = [t1, -t2];
        b = d2 - d1;

        x = A \ b;

        if x(1) >= -tol && x(2) <= tol
            c = d1 + x(1) * t1;
        else
            path = [];
            path_length = inf;
            return;
        end
    end

    b1 = c - n1;
    b2 = c - n2;

    line1 = zeros(2,0);
    line1_length = norm(b1 - p1);

    if line1_length > stepSize
        ds = (1.0 / line1_length) * stepSize;
        s = ds;
        while s < 1
            p = p1 + s * (b1 - p1);
            line1(:,end+1) = p; %#ok<AGROW>
            s = s + ds;
        end
    end

    arc = zeros(2,0);
    ang_dist = wrap_to_2pi(dirSign * signed_angle(b1 - c, b2 - c));
    arc_length = abs(ang_dist * radius);

    if arc_length > stepSize && radius > 0
        start_angle = atan2(b1(2) - c(2), b1(1) - c(1));
        dth = stepSize / radius;

        s = dth;
        while s < ang_dist
            th = start_angle + s * dirSign;
            p = c + [cos(th); sin(th)] * radius;
            arc(:,end+1) = p; %#ok<AGROW>
            s = s + dth;
        end
    end

    line2 = zeros(2,0);
    line2_length = norm(b2 - p2);

    if line2_length > stepSize
        ds = (1.0 / line2_length) * stepSize;
        s = ds;
        while s < 1
            p = b2 + s * (p2 - b2);
            line2(:,end+1) = p; %#ok<AGROW>
            s = s + ds;
        end
    end

    path = [line1, arc, line2];
    path_length = line1_length + arc_length + line2_length;
end

%% =========================================================================
function th = wrap_to_2pi(th)
    twoPi = 2*pi;
    th = mod(mod(th, twoPi) + twoPi, twoPi);
end

function th = wrap_to_pi(th)
    twoPi = 2*pi;
    th = mod(th, twoPi);
    th = mod(th + twoPi, twoPi);
    if th > pi
        th = th - twoPi;
    end
end

function th = signed_angle(v1, v2)
    th = wrap_to_pi(atan2(v2(2), v2(1)) - atan2(v1(2), v1(1)));
end

%% =========================================================================
function plot_roadmap(roadmap)

    plottedRoadmap = false;

    for k = 1:numel(roadmap.edges)
        wp = roadmap.edges(k).waypoints;
        if ~isempty(wp)
            if ~plottedRoadmap
                plot(wp(1,:), wp(2,:), 'Color', [0 0.6 0], 'LineWidth', 1.2, ...
                    'DisplayName', 'Roadmap');
                plottedRoadmap = true;
            else
                plot(wp(1,:), wp(2,:), 'Color', [0 0.6 0], 'LineWidth', 1.2, ...
                    'HandleVisibility', 'off');
            end
        end
    end

    for i = 1:size(roadmap.nodes,1)
        x = roadmap.nodes(i,1);
        y = roadmap.nodes(i,2);
        th = roadmap.nodes(i,3);

        plot(x, y, 'ro', 'MarkerSize', 5, 'MarkerFaceColor', 'r', 'HandleVisibility', 'off');
        text(x + 0.03, y + 0.03, num2str(i-1), 'Color', 'k', 'FontSize', 9);

        arrowLen = 0.10;
        quiver(x, y, arrowLen*cos(th), arrowLen*sin(th), 0, ...
            'Color', 'r', 'LineWidth', 1, 'MaxHeadSize', 2, 'HandleVisibility', 'off');
    end
end
