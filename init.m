rng(1); %(random seed for repeatability)
dt = 1e-1;
tspan = 0:dt:3000;

% Number of agents/obstacles to simulate:
num_agents = 4;

% Distance settings:
d0 = 1.5; % Desired distance from virtual leader
d_react_obsr = 1.5; %Reaction distance (in terms of obstacle radius)
drone_width = 2;
d_min_obsr = 1; %Minimum allowable distance to obstalce (in terms of obstacle radius)
% IF THE AGENT GOES BELOW THE MINIMUM DISTANCE, THE RUN FAILS.

% Agent limitations:
max_v = 10; %(Max velocity allowed)
max_u = 2; %(Max acceleration allowed)

% Virtaul leader (position and velocity):
vl = [-50 -50 5 5]; %(rx,ry,vx,vy)

%% Initialization:
% Memory allocation:
L = length(tspan);
u = zeros(2*num_agents,L);
r = zeros(2*num_agents,L);
v = zeros(2*num_agents,L);
vl_rv = zeros(4,L);

% Randomly place obstacles:
obs_radii = 20;
obs = [0 0, obs_radii];
d_react = d_react_obsr*obs(:,3); % react distance defined off of obstacle size
d_min = obs(:,3) + 1.5;

% Randomly place the agents:
r(:,1) = repmat(reshape(vl(1:2),[],1),num_agents,1) + [d0; 0; -d0; 0; 0; d0; 0; -d0];
v(:,1) = repmat(reshape(vl(3:4),[],1),num_agents,1) + .1*randn(2*num_agents,1);
vl_rv(:,1) = vl';