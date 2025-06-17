function [path, path_length] = RRT(start_state, goal_region, obstacles)
T = struct('state', start_state, 'parent', 0);
k = 0;

while k~=1
    x_rand = randi([1,100],1,2);
    T = extend_tree(x_rand,T,obstacles);
    x_n = T.state(end,:);
    if x_n(1)>=goal_region(1)
       k=1;
    end
end

Tree = T.state;
goal_index = length(Tree);
path = [];
while goal_index ~= 0
    path = [path; T.state(goal_index,:)];
    goal_index = T.parent(goal_index);
end
path = flipud(path); % Reverse the path to start from the start_state

% plot(path(:,1),path(:,2))

% path length
sum = 0;
for i = 2:length(path(:,1))
    sum = sum + sqrt((path(i,1)-path(i-1,1))^2 + (path(i,2)-path(i-1,2))^2);
end
path_length = sum;
end

%% Extend the tree
function T_new = extend_tree(x,T,obstacles)
e = 2;
if collision_check_point(x(1), x(2), obstacles) ==1
    T_new = T;
else
    [x_near,I] = nearest_neighbor(x,T.state);
    x_hat = (x-x_near)/norm(x-x_near); % Get the direction of the new node
    x_new = x_near + x_hat*e; % Grow the tree in the direction of the new node
    if collision_check_segment(x(1),x(2),x_new(1),x_new(2),obstacles) ==1
        T_new = T;
    else
        T_new.state = [T.state;x_new];
        T_new.parent = [T.parent;I];
    end
end
end

%% Find the nearest neighbor
function [x_near,I] = nearest_neighbor(x,T)
for i = 1:length(T(:,1))
    d(i) = norm(x-T(i,:)); % Find the distance of the new node to each node in the tree
end
[~,I] = min(d); % Find the node to which the new node is closest to
x_near = T(I,:);
end