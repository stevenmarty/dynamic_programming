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



    function p = get_prob_of_shooters(point)
            distances = zeros(1,length(shooters));
            for shooter = 1:length(shooters)
                xdiff = abs(point(1)-shooters_location(shooter,1));
                ydiff = abs(point(2)-shooters_location(shooter,2));
                distances(shooter) = xdiff + ydiff;
            end
            
            p=0;
            for distance = distances
                if distance <= R
                    p = p + GAMMA/(distance+1);
                end
            end
    end

            
  function ct = is_Tree(point)
        ct = 0;
        for singleTree = 1:length(trees)
            xdiff = abs(point(1)-trees_location(singleTree,1));
            ydiff = abs(point(2)-trees_location(singleTree,2));
            if ((xdiff == 0 && ydiff == 0))
                ct = ct +  1;
            else
                ct = ct + 0;
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

    if isequal([m,n],dropoff_loc) && carry==1
        G(from, :) = 0;
        continue
    end
        new_points = [north;south;east;west;[m,n]];
        
       for input = [NORTH, SOUTH, EAST, WEST, HOVER]
        new_point = new_points(input,:);
%       inside grid and not in tree
        if new_point(1)<=M && new_point(2)<=N && new_point(1)>0 && new_point(2)>0 && map(new_point(1),new_point(2)) ~= TREE 

            
            P_crash_stay = get_prob_of_shooters(new_point);

            P_crash_wind_shooter = 0;
            P_crash_wind_tree = 0;
            nmbr_borderPoints = 0;

            northwind= [new_point(1),new_point(2)+1];
            southwind= [new_point(1),new_point(2)-1];
            westwind = [new_point(1)-1,new_point(2)];
            eastwind = [new_point(1)+1,new_point(2)];

            wind_new_points = [northwind;southwind;eastwind;westwind];
            
            for index_w = 1:4
                new_w_point = wind_new_points(index_w,:);
                if new_w_point(1)<=M && new_w_point(2)<=N && new_w_point(1)>0 && new_w_point(2)>0 && map(new_w_point(1),new_w_point(2)) ~= TREE   
                    
                    P_crash_wind_shooter = P_crash_wind_shooter + get_prob_of_shooters(new_w_point);
                    P_crash_wind_tree = P_crash_wind_tree + is_Tree(new_w_point);
                else
                nmbr_borderPoints = nmbr_borderPoints + 1;
                end
            end  
        
        else
              G(from,input) = inf;
              continue
        end
            % shooter
            crashWind = P_crash_wind_shooter + P_crash_wind_tree + nmbr_borderPoints;
            crashStay = P_crash_stay;

            G(from, input) = (crashStay*(1-P_WIND) + (crashWind * P_WIND * 0.25))*Nc + (1 - ((crashStay*(1-P_WIND) + (crashWind * P_WIND * 0.25))));
           


       end 
    
    end
end













