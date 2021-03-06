function P = ComputeTransitionProbabilities(stateSpace, map)
%COMPUTETRANSITIONPROBABILITIES Compute transition probabilities.
% 	Compute the transition probabilities between all states in the state
%   space for all control inputs.
%
%   P = ComputeTransitionProbabilities(stateSpace, map) 
%   computes the transition probabilities between all states in the state 
%   space for all control inputs.
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
%       P:
%           A (K x K x L)-matrix containing the transition probabilities
%           between all states in the state space for all control inputs.
%           The entry P(i, j, l) represents the transition probability
%           from state i to state j if control input l is applied.

global GAMMA R P_WIND
global FREE TREE SHOOTER PICK_UP DROP_OFF BASE
global NORTH SOUTH EAST WEST HOVER
global K TERMINAL_STATE_INDEX
        
size_map = size(map);
M = size_map(1)   
N = size_map(2)  
P = zeros(K,K, 5);

% locations are always [m,n]
shooters = find(map==SHOOTER);
shooters_location = zeros(length(shooters),2);

for shooter_index = 1:length(shooters)
    [m,n]= ind2sub(size(map),shooters(shooter_index));
    shooters_location(shooter_index,:)=[m,n];
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

% Iterate over all startingstates and compute all possible states from
% there and their probabilities
for from = 1:K
    m = stateSpace(from,1);
    n = stateSpace(from,2);
    carry = stateSpace(from,3);
%   If dropoff with carry we will not change location
    if isequal([m,n],dropoff_loc) && carry==1
        P(from,get_row([m,n],1),:)=1;
        continue
    end
    
    north = [m,n+1];
    south = [m,n-1];
    west = [m-1,n];
    east = [m+1,n];
    new_points = [north;south;east;west;[m,n]];
    
    for input = [NORTH, SOUTH, EAST, WEST, HOVER]
        new_point = new_points(input,:);
%       inside grid and not in tree
        if new_point(1)<=M && new_point(2)<=N && new_point(1)>0 && new_point(2)>0 && map(new_point(1),new_point(2)) ~= TREE 
            
            prob = get_prob_of_shooters(new_point);
            
            if new_point==pickup_loc
                P(from,get_row(new_point,1),input) = (1-P_WIND)*(1-prob);
            else 
                P(from,get_row(new_point,carry),input) = (1-P_WIND)*(1-prob);
            end
            
            if prob > 0
                P(from,base_row,input) = P(from,base_row,input) + (1-P_WIND)*prob;
            end

            northwind= [new_point(1),new_point(2)+1];
            southwind= [new_point(1),new_point(2)-1];
            westwind = [new_point(1)-1,new_point(2)];
            eastwind = [new_point(1)+1,new_point(2)];

            wind_new_points = [northwind;southwind;eastwind;westwind];

            for index_w = 1:4
                new_w_point = wind_new_points(index_w,:);
                if new_w_point(1)<=M && new_w_point(2)<=N && new_w_point(1)>0 && new_w_point(2)>0 && map(new_w_point(1),new_w_point(2)) ~= TREE  

                    prob = get_prob_of_shooters(new_w_point);
                    
                    if new_w_point == pickup_loc
                         P(from,get_row(new_w_point,1),input) = P(from,get_row(new_w_point,1),input) + (0.25 * P_WIND)*(1-prob);
                    else
                        P(from,get_row(new_w_point,carry),input) = P(from,get_row(new_w_point,carry),input) + (0.25 * P_WIND)*(1-prob);
                    end
                    
                    if prob>0
                        P(from,base_row,input) = P(from,base_row,input) + (0.25* P_WIND)*prob;
                    end
                else
                    
                    P(from,base_row,input) = P(from,base_row,input) + 0.25 * P_WIND;
                end
            end  
        end
    end  


end
end
