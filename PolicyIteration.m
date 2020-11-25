function [ J_opt, u_opt_ind ] = PolicyIteration(P, G)
%POLICYITERATION Policy iteration
%   Solve a stochastic shortest path problem by Policy Iteration.
%
%   [J_opt, u_opt_ind] = PolicyIteration(P, G) computes the optimal cost and
%   the optimal control input for each state of the state space.
%
%   Input arguments:
%       P:
%           A (k x k x L)-matrix containing the transition probabilities
%           between all states in the state space for all control inputs.
%           The entry P(i, j, l) represents the transition probability
%           from state i to state j if control input l is applied.
%
%       G:
%           A (k x L)-matrix containing the stage costs of all states in
%           the state space for all control inputs. The entry G(i, l)
%           represents the cost if we are in state i and apply control
%           input l.
%
%   Output arguments:
%       J_opt:
%       	A (k x 1)-matrix containing the optimal cost-to-go for each
%       	element of the state space.
%
%       u_opt_ind:
%       	A (k x 1)-matrix containing the index of the optimal control
%       	input for each element of the state space. Mapping of the
%       	terminal state is arbitrary (for example: HOVER).

global K HOVER

%% Handle terminal state
% Do yo need to do something with the teminal state before starting policy
% iteration?
global TERMINAL_STATE_INDEX
% IMPORTANT: You can use the global variable TERMINAL_STATE_INDEX computed
% in the ComputeTerminalStateIndex.m file (see main.m)
J_opt = ones(K,1);
J_new = zeros(K,1);
u_opt_ind = ones(K,1)*5;
u_opt_ind(TERMINAL_STATE_INDEX)=HOVER;
finished = false;

counter = 0;
while ~finished    
    counter = counter + 1;
    for i=1:K  
        if i ~= TERMINAL_STATE_INDEX
            policy = u_opt_ind(i);
            J_new(i) =  G(i,policy) + J_opt'*squeeze(P(i,:,policy))' ;
        end
    end
    finished = max(abs(J_new-J_opt)) == 0;
    if finished
        counter
    end 
    J_opt = J_new;

    for h=1:K  
        if h ~= TERMINAL_STATE_INDEX
            if h == 473
                G(h,:) + J_opt'*squeeze(P(h,:,:))
            end
            [A, u_opt_ind(h)] = min( G(h,:) + J_opt'*squeeze(P(h,:,:)) );
        end
    end
    

end

%     TODO: should be normalize like in EX 2?

end
