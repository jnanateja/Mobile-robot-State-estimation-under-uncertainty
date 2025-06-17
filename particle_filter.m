function p = particle_filter(path)
numParticles = 100;
x_0 = path(1,:);
particles = generate_particles(x_0, numParticles);
weights = ones(numParticles,1) / numParticles;


for i = 2:length(path(:,1))
    
    plot(particles(:,1), particles(:,2), '*');
    pause(0.1);

    particles = predict_particles(particles, path(i-1,:), path(i,:), numParticles);

    [particles,weights] = update_particles(particles, path(i,:), weights);

    particles = resample_particles(particles, weights);

end

plot(path(:,1), path(:,2))
end
function particles = generate_particles(x,numParticles)
particles = zeros(numParticles,2);
for i = 1:numParticles
    particles(i,:) = x + 10*randn(1,2);
end
end 


function particles = predict_particles(particles, x_p, x_c, num)
n = (x_c-x_p)/norm(x_c-x_p);
for t=1:num
    particles(t,:) = particles(t,:)+n*2;
end
end


function [particles,weights] = update_particles(particles, x, weights)
for t=1:length(particles(:,1))
    p_z_x = (1/sqrt(2*pi))*exp(-((x(1)-particles(t,1))^2)/2);
    p_z_y = (1/sqrt(2*pi))*exp(-((x(2)-particles(t,2))^2)/2);

    weights(t) = weights(t)*p_z_x*p_z_y;
end
weights = weights/sum(weights);
end

function particles = resample_particles(particles, weights)
    num_particles = size(particles, 1);
    resampled_particles = zeros(size(particles));
    cumulative_weights = cumsum(weights);
    random_numbers = rand(1, num_particles);

    for i = 1:num_particles
        idx = find(cumulative_weights >= random_numbers(i), 1, 'first');
        resampled_particles(i, :) = particles(idx, :);
    end
    particles = resampled_particles;
end