function [ J_opt, u_opt_ind ] = LinearProgramming(P, G)
%LINEARPROGRAMMING Linear Programming
%   Solve a stochastic shortest path problem by Linear Programming.
%
%   [J_opt, u_opt_ind] = LinearProgramming(P, G) computes the optimal cost
%   and the optimal control input for each state of the state space.
%
%   Input arguments:
%       P:
%           A (K x K x L)-matrix containing the transition probabilities
%           between all states in the state space for all control inputs.
%           The entry P(i, j, l) represents the transition probability
%           from state i to state j if control input l is applied.
%
%       G:
%           A (K x L)-matrix containing the stage costs of all states in
%           the state space for all control inputs. The entry G(i, l)
%           represents the cost if we are in state i and apply control
%           input l.
%
%   Output arguments:
%       J_opt:
%       	A (K x 1)-matrix containing the optimal cost-to-go for each
%       	element of the state space.
%
%       u_opt_ind:
%       	A (K x 1)-matrix containing the index of the optimal control
%       	input for each element of the state space. Mapping of the
%       	terminal state is arbitrary (for example: HOVER).

global K HOVER

%% Handle terminal state
% Do yo need to do something with the teminal state before starting policy
% iteration ?
global TERMINAL_STATE_INDEX
% IMPORTANT: You can use the global variable TERMINAL_STATE_INDEX computed
% in the ComputeTerminalStateIndex.m file (see main.m)
J_opt = ones(K,1);
u_opt_ind = ones(K,1);

f = -ones(K-1,1);

b=G;
b(TERMINAL_STATE_INDEX,:)   = []; 
b = reshape(b,[(K-1)*5,1]);
infinite_stage_cost = b==inf;
b(infinite_stage_cost)=[];

P_new=P;
P_new(TERMINAL_STATE_INDEX,:,:)=[];
P_new = [P_new(:,:,1);P_new(:,:,2);P_new(:,:,3);P_new(:,:,4);P_new(:,:,5)];

I= eye(K);
I(TERMINAL_STATE_INDEX,:)=[];
I_new = [I(:,:);I(:,:);I(:,:);I(:,:);I(:,:)];

A = I_new-P_new;
A(infinite_stage_cost,:)=[];
A(:,TERMINAL_STATE_INDEX)=[];

J_opt([1:TERMINAL_STATE_INDEX-1,TERMINAL_STATE_INDEX+1:K]) = linprog(f,A,b);

for i=1:K
    temp=(G(i,:)+J_opt'*squeeze(P(i,:,:)))';
    x=find(temp==min(temp))
    u_opt_ind(i)=x(1);
end
    

end

