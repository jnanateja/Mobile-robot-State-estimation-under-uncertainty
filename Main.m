clear all;
close all;
clc;
generate_obstacles;
num_trails = 100;
% Shortes path
short_path = [];
short_path_len = 1000;
Pk_short = [];

% Path with least uncertainity
least_uncertain_path = [];
least_uncertainity = 10000;
Pk_least_uncert = [];

% Max uncertainity
Max_uncertain_path = [];
Max_uncertainity = 10;
Pk_Max_uncert = [];


for i=1:num_trails
    [path, path_length] = RRT(start_state, goal_region, obstacles);
    [Pfinal,pk] = propagate_KF_path(path,obstacles);
    uncertainity = trace(Pfinal);

    if path_length < short_path_len
        short_path_len = path_length;
        short_path = path;
        Pk_short = pk;
    end

    if uncertainity < least_uncertainity 
        least_uncertainity = uncertainity;
        least_uncertain_path = path;
        Pk_least_uncert = pk;
    end

    if uncertainity > Max_uncertainity
        Max_uncertainity = uncertainity;
        Max_uncertain_path = path;
        Pk_Max_uncert = pk;
    end
end


% Shortest path
generate_obstacles;
for i = 1:length(Pk_short(1,1,:))
    ra(i) = sqrt(Pk_short(1,1,i));
    rb(i) = sqrt(Pk_short(2,2,i));
    ang(i) = atan(ra/rb);
end
x = short_path(:,1);
y = short_path(:,2);
C = [];
Nb = 300;
plot(x,y,'k')
title('Plot of the Shortest path')
ellipse(ra,rb,ang,x,y,C,Nb);


% Min uncertainity
generate_obstacles;
for i = 1:length(Pk_least_uncert(1,1,:))
    ra(i) = sqrt(Pk_least_uncert(1,1,i));
    rb(i) = sqrt(Pk_least_uncert(2,2,i));
    ang(i) = atan(ra/rb);
end
x = least_uncertain_path(:,1);
y = least_uncertain_path(:,2);
C = [];
Nb = 300;
plot(x,y,'k')
title('Plot of the minimum terminal uncertainity')
ellipse(ra,rb,ang,x,y,C,Nb);


% Max uncertainity
generate_obstacles;
for i = 1:length(Pk_Max_uncert(1,1,:))
    ra(i) = sqrt(Pk_Max_uncert(1,1,i));
    rb(i) = sqrt(Pk_Max_uncert(2,2,i));
    ang(i) = atan(ra/rb);
end
x = Max_uncertain_path(:,1);
y = Max_uncertain_path(:,2);
C = [];
Nb = 300;
plot(x,y,'k')
title('Plot of the maximum terminal uncertainity')
ellipse(ra,rb,ang,x,y,C,Nb);


