% Copyright (C) 2017 Jo?o Valente
% 
% This program is free software: you can redistribute it and/or modify
% it under the terms of the GNU General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
%
% This program is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU General Public License for more details.
%
% You should have received a copy of the GNU General Public License
% along with this program.  If not, see <http://www.gnu.org/licenses/>.
%
% Author: Jo?o Valente
% email: jvalente@ing.uc3m.es
%-----------------------------------------------------------------------

load data.mat

% Images overlapping
Overlapping = 0.6; 

% Image Pixels
Px = 4704; 
Py = 3136;

% Image resolution requirements
Req = 0.01; 

% Image Ground projection
Dx = Px*Req;
Dy = Py*Req;

% Field Polyline
lat1 = 4462829.22;
lon1 = 458798.25;
lat2 = 4462868.40;
lon2 = 458863.36;
lat3 = 4462846.79;
lon3 = 458875.39;
lat4 = 4462807.73;
lon4 = 458810.50;

Polyline = [[lon1 lat1]; [lon2,lat2]; [lon3 lat3]; [lon4 lat4]; [lon1 lat1]];

figure; hold;

% Field borders
plot(Polyline(:,1), Polyline(:,2), 'k-','LineWidth',2);

% Bounding box 
line(vlon, vlan);

% Smallest bounding box
plot(new_x, new_y, 'r-');

% Sampled Points
for m = 1:Nx
    for n = 1:Ny
        if(~isempty(D{n,m}))
            plot3(D{n,m}(1), D{n,m}(2), 0, 'ks');
        end
    end
end


%Image ground projection

for m = 1:Nx
    for n = 1:Ny
        
        if(~isempty(D{n,m}))
            
            cr1 = [D{n,m}(1) D{n,m}(2)] + (rot*[-Dx/2 Dy/2]')';
            cr2 = [D{n,m}(1) D{n,m}(2)] + (rot*[-Dx/2 -Dy/2]')';
            cr3 = [D{n,m}(1) D{n,m}(2)] + (rot*[Dx/2 -Dy/2]')';
            cr4 = [D{n,m}(1) D{n,m}(2)] + (rot*[Dx/2 Dy/2]')';
               
        end
        
         photo = [cr1; cr2; cr3; cr4; cr1];   
         line(photo(:,1), photo(:,2),'Color',[.1 .5 1]);
  
    end
end

tic;

% Functions
takeoff = 5;%5
landing = 6;%6
coordinates=order(D);
userConfig = struct('xy',coordinates);
resultStruct = tsp_ga(userConfig);

position = find(resultStruct.optRoute == takeoff);
reordered = zeros(size(resultStruct.optRoute));
reordered(1:end-position+1) = resultStruct.optRoute(position:end);
reordered(end-position+2:end) = resultStruct.optRoute(1:position-1);
finalDistance = resultStruct.minDist - resultStruct.dmat(takeoff,landing);

figure(1);
for i= 1:17
    plot([coordinates(reordered(i),1) coordinates(reordered(i+1),1)],[coordinates(reordered(i),2) coordinates(reordered(i+1),2)],'b','LineWidth',5);   
end

hold on;
plot([coordinates(reordered(1),1)],[coordinates(reordered(1),2)],'o','color','r','LineWidth',8);
plot([coordinates(reordered(18),1)],[coordinates(reordered(18),2)],'o','color','g', 'LineWidth',8);

title(sprintf('Total Distance = %1.4f',finalDistance));

toc; 
tiempo = toc;
display(tiempo)


% CASE B
tic;

coordinates2 = [coordinates(1:14,:); coordinates(17:18,:)];
userConfig2 = struct('xy',coordinates2);
resultStruct2 = tsp_ga(userConfig2);

position2 = find(resultStruct2.optRoute == takeoff);
reordered2 = zeros(size(resultStruct2.optRoute));
reordered2(1:end-position2+1) = resultStruct2.optRoute(position2:end);
reordered2(end-position2+2:end) = resultStruct2.optRoute(1:position2-1);
finalDistance2 = resultStruct2.minDist - resultStruct2.dmat(takeoff,landing);

figure(4);
hold on;
for i= 1:15
    plot([coordinates2(reordered2(i),1) coordinates2(reordered2(i+1),1)],[coordinates2(reordered2(i),2) coordinates2(reordered2(i+1),2)],'b','LineWidth',5);   
end

plot([coordinates2(reordered2(1),1)],[coordinates2(reordered2(1),2)],'o','color','r','LineWidth',8);
plot([coordinates2(reordered2(16),1)],[coordinates2(reordered2(16),2)],'o','color','g', 'LineWidth',8);

title(sprintf('Total Distance = %1.4f',finalDistance2));

toc; 
tiempo = toc;
display(tiempo)


%-----------------------------------------------------------------------
%Travel Agent Algorithm
function varargout = tsp_ga(varargin)
    
    % Initialize default configuration
    defaultConfig.xy          = 5;
    defaultConfig.dmat        = [];
    defaultConfig.popSize     = 100;
    defaultConfig.numIter     = 300;
    defaultConfig.showProg    = true;
    defaultConfig.showResult  = true;
    defaultConfig.showWaitbar = false;
    
    % Interpret user configuration inputs
    if ~nargin
        userConfig = struct();
    elseif isstruct(varargin{1})
        userConfig = varargin{1};
    else
        try
            userConfig = struct(varargin{:});
        catch
            error('Expected inputs are either a structure or parameter/value pairs');
        end
    end
    
    % Override default configuration with user inputs
    configStruct = get_config(defaultConfig,userConfig);
    
    % Extract configuration
    xy          = configStruct.xy;
    dmat        = configStruct.dmat;
    popSize     = configStruct.popSize;
    numIter     = configStruct.numIter;
    showProg    = configStruct.showProg;
    showResult  = configStruct.showResult;
    showWaitbar = configStruct.showWaitbar;
    if isempty(dmat)
        nPoints = size(xy,1);
        a = meshgrid(1:nPoints);
        dmat = reshape(sqrt(sum((xy(a,:)-xy(a',:)).^2,2)),nPoints,nPoints);
    end
    
    % Verify Inputs
    [N,dims] = size(xy);
    [nr,nc] = size(dmat);
    if N ~= nr || N ~= nc
        error('Invalid XY or DMAT inputs!')
    end
    n = N;
    
    % Sanity Checks
    popSize     = 4*ceil(popSize/4);
    numIter     = max(1,round(real(numIter(1))));
    showProg    = logical(showProg(1));
    showResult  = logical(showResult(1));
    showWaitbar = logical(showWaitbar(1));
    
    % Initialize the Population
    pop = zeros(popSize,n);
    pop(1,:) = (1:n);
    
    for k = 2:popSize
        pop(k,:) = randperm(n);
    end
    
    % Run the GA
    globalMin = Inf;
    totalDist = zeros(1,popSize);
    distHistory = zeros(1,numIter);
    tmpPop = zeros(4,n);
    newPop = zeros(popSize,n);
    if showProg
        figure('Name','TSP_GA | Current Best Solution','Numbertitle','off');
        hAx = gca;
    end
    if showWaitbar
        hWait = waitbar(0,'Searching for near-optimal solution ...');
    end
    for iter = 1:numIter
        % Evaluate Each Population Member (Calculate Total Distance)
        for p = 1:popSize
            d = dmat(pop(p,n),pop(p,1)); % Closed Path
            for k = 2:n
                d = d + dmat(pop(p,k-1),pop(p,k));
            end
            totalDist(p) = d;
        end
        
        % Find the Best Route in the Population
        [minDist,index] = min(totalDist);
        distHistory(iter) = minDist;
        if minDist < globalMin
            globalMin = minDist;
            optRoute = pop(index,:);
            if showProg
                % Plot the Best Route
                rte = optRoute([1:n 1]);
                if dims > 2, plot3(hAx,xy(rte,1),xy(rte,2),xy(rte,3),'r.-');
                else plot(hAx,xy(rte,1),xy(rte,2),'r.-'); end
                title(hAx,sprintf('Total Distance = %1.4f, Iteration = %d',minDist,iter));
                drawnow;
            end
        end
        
        % Genetic Algorithm Operators
        randomOrder = randperm(popSize);
        for p = 4:4:popSize
            rtes = pop(randomOrder(p-3:p),:);
            dists = totalDist(randomOrder(p-3:p));
            [ignore,idx] = min(dists); %#ok
            bestOf4Route = rtes(idx,:);
            routeInsertionPoints = sort(ceil(n*rand(1,2)));
            I = routeInsertionPoints(1);
            J = routeInsertionPoints(2);
            for k = 1:4 % Mutate the Best to get Three New Routes
                tmpPop(k,:) = bestOf4Route;
                switch k
                    case 2 % Flip
                        tmpPop(k,I:J) = tmpPop(k,J:-1:I);
                    case 3 % Swap
                        tmpPop(k,[I J]) = tmpPop(k,[J I]);
                    case 4 % Slide
                        tmpPop(k,I:J) = tmpPop(k,[I+1:J I]);
                    otherwise % Do Nothing
                end
            end
            newPop(p-3:p,:) = tmpPop;
        end
        pop = newPop;
        
        % Update the waitbar
        if showWaitbar && ~mod(iter,ceil(numIter/325))
            waitbar(iter/numIter,hWait);
        end
        
    end
    if showWaitbar
        close(hWait);
    end
    
%     if showResult
%          % Plots the GA Results
%          figure('Name','TSP_GA | Results','Numbertitle','off');
%          subplot(2,2,1);
%          pclr = ~get(0,'DefaultAxesColor');
%          if dims > 2, plot3(xy(:,1),xy(:,2),xy(:,3),'.','Color',pclr);
%          else plot(xy(:,1),xy(:,2),'.','Color',pclr); end
%          title('City Locations');
%          subplot(2,2,2);
%          imagesc(dmat(optRoute,optRoute));
%          title('Distance Matrix');
%          subplot(2,2,3);
%          rte = optRoute([1:n 1]);
%          if dims > 2, plot3(xy(rte,1),xy(rte,2),xy(rte,3),'r.-');
%          else plot(xy(rte,1),xy(rte,2),'r.-'); end
%          title(sprintf('Total Distance = %1.4f',minDist));
%          subplot(2,2,4);
%          plot(distHistory,'b','LineWidth',2);
%          title('Best Solution History');
%          set(gca,'XLim',[0 numIter+1],'YLim',[0 1.1*max([1 distHistory])]);
%     end
    
    % Return Output
    if nargout
        resultStruct = struct( ...
            'xy',          xy, ...
            'dmat',        dmat, ...
            'popSize',     popSize, ...
            'numIter',     numIter, ...
            'showProg',    showProg, ...
            'showResult',  showResult, ...
            'showWaitbar', showWaitbar, ...
            'optRoute',    optRoute, ...
            'minDist',     minDist);
        
        varargout = {resultStruct};
    end
    
end

% Subfunction to override the default configuration with user inputs
function config = get_config(defaultConfig,userConfig)
    
    % Initialize the configuration structure as the default
    config = defaultConfig;
    
    % Extract the field names of the default configuration structure
    defaultFields = fieldnames(defaultConfig);
    
    % Extract the field names of the user configuration structure
    userFields = fieldnames(userConfig);
    nUserFields = length(userFields);
    
    % Override any default configuration fields with user values
    for i = 1:nUserFields
        userField = userFields{i};
        isField = strcmpi(defaultFields,userField);
        if nnz(isField) == 1
            thisField = defaultFields{isField};
            config.(thisField) = userConfig.(userField);
        end
    end
    
end

%Order coordinates in matrix 18x2
function graph = order(points)

graph=[];
pointsXY = [];

for i = 1:size(points,1)
    for j=1:size(points,2)
        pointsXY=[points{i,j}(1) points{i,j}(2)]
        graph=[graph;pointsXY]
    end
end
end





