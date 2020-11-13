function G = ComputeStageCosts(stateSpace, map)
%COMPUTESTAGECOSTS Compute stage costs.
% 	Compute the stage costs for all states in the state space for all
%   control inputs.
%
%   G = ComputeStageCosts(stateSpace, map) 
%   computes the stage costs for all states in the state space for all
%   control inputs.
%
%   Input arguments:
%       stateSpace:
%           A (K x 3)-matrix, where the i-th row represents the i-th
%           element of the state space.
%
%       map:
%           A (M x N)-matrix describing the world. With
%           values: FREE TREE SHOOTER PICK_UP DROP_OFF BASE
%
%   Output arguments:
%       G:
%           A (K x L)-matrix containing the stage costs of all states in
%           the state space for all control inputs. The entry G(i, l)
%           represents the expected stage cost if we are in state i and 
%           apply control input l.

    global GAMMA R P_WIND Nc
    global FREE TREE SHOOTER PICK_UP DROP_OFF BASE
    global NORTH SOUTH EAST WEST HOVER
    global K
    global TERMINAL_STATE_INDEX
    
    
    size_map = size(map);
    M = size_map(1)   
    N = size_map(2)  
    G = zeros(K,5);

    % locations are always [m,n]
    shooters = find(map==SHOOTER);
    shooters_location = zeros(length(shooters),2);
    
    for shooter_index = 1:length(shooters)
    [m,n]= ind2sub(size(map),shooters(shooter_index));
    shooters_location(shooter_index,:)=[m,n];
    end
    
    % locations are always [m,n]
    trees = find(map==TREE);
    trees_location = zeros(length(trees),2);
    
    for trees_index = 1:length(trees)
    [m,n]= ind2sub(size(map),trees(trees_index));
    trees_location(trees_index,:)=[m,n];
    end    
    

    base = find(map==BASE);
    [m,n] = ind2sub(size(map),base);
    base_row = get_row([m,n],0);

    pickup = find(map==PICK_UP);
    [m,n] = ind2sub(size(map),pickup);
    pickup_loc = [m,n];

    dropoff = find(map==DROP_OFF);
    [m,n] = ind2sub(size(map),dropoff);
    dropoff_loc = [m,n];
    
    function x= get_row(coordinate,carry)
    x= find(ismember(stateSpace, [coordinate(1),coordinate(2),carry],'rows'));
    end

    function cs = get_cost_of_shooters(point)
            distances = zeros(1,length(shooters));
            for shooter = 1:length(shooters)
                xdiff = abs(point(1)-shooters_location(shooter,1));
                ydiff = abs(point(2)-shooters_location(shooter,2));
                distances(shooter) = xdiff + ydiff;
            end
            
            cs=0;
            for distance = distances
                if distance <= R
                    cs = (GAMMA/distance)*Nc;    %GAMMA/(distance+1);
                
                else
                    cs = 0;
                end
            end
    end

    function cR = get_cost_of_borders(point)
            cR = 0;
            
            if (point(1) == 1) || (point(1) == M)
                cR = 0.25 * Nc;
            end
            
            if ((point(2) == 1) || (point(2) == N))
                 cR = cR + 0.25 * Nc;
            end   
            end
            
    
            
  function ct = get_cost_of_trees(point)
        ct = 0;
        for singleTree = 1:length(trees)
            xdiff = abs(point(1)-trees_location(singleTree,1));
            ydiff = abs(point(2)-trees_location(singleTree,2));
            if ((xdiff == 1 && ydiff == 0) || (ydiff == 1 && xdiff == 0))
                ct = ct + 0.25 * Nc;
            end 
        end
  end
% Iterate over all startingstates and compute all possible states from
% there and their probabilities
    for from = 1:K
        m = stateSpace(from,1);
        n = stateSpace(from,2);
        carry = stateSpace(from,3);

        north = [m,n+1];
        south = [m,n-1];
        west = [m-1,n];
        east = [m+1,n];

        new_points = [north;south;east;west;[m,n]];

        for input = [NORTH, SOUTH, EAST, WEST, HOVER]
            new_point = new_points(input,:);
            if new_point(1)<=M && new_point(2)<=N && new_point(1)>0 && new_point(2)>0 && map(new_point(1),new_point(2)) ~= TREE            
                costTree = get_cost_of_trees(new_point);
                costShoot = get_cost_of_shooters(new_point);
                costBorder = get_cost_of_borders(new_point);
                G(from,input) = costBorder + costTree + costShoot + 1;
            else
                G(from,input) = inf;
            end
        end  
    end
end












