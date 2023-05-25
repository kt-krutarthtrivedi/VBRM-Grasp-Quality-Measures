%   VBRM - Grasp Quality Matrix,
%   Description: Based on "Grasp quality measures: review and performance" by 
%   Maximo A. Roa and Raul Suarez.
%   Author: Krutarth Ambarish Trivedi | ktrivedi@wpi.edu

clc;
clf;
close all;
clear;

%-------------- Locations -------------%
O = [3, 1.5];
C1 = [3, 0];
C2 = [0, 2];
C3 = [2, 3];
C4 = [4, 3];
C5_pos = [];

%------------- Frame angles --------------------%
theta_upper = deg2rad(180);
theta_lower = deg2rad(0);
theta_right = deg2rad(90);
theta_left = deg2rad(-90);

singular_min = [];
singular_max = [];
volume = [];
grasp_isotropy = [];

g = generate_grasp_matrix(O, C2, theta_left);
disp(g)

%------------ Right side edge of an object ------------%
C5 = [6, 0]; %starting at the lower right corner
C5_theta = theta_right;

while(C5(2) >= 0 && C5(2) <= 3)
    location_contact_points = [C1; C2; C3; C4; C5];
    angle_contact_points = [theta_lower; theta_left; theta_upper; theta_upper; C5_theta];
    grasp_matrix = generate_grasp_matrix(O, location_contact_points, angle_contact_points);

    s_min = minimum_singular_value(grasp_matrix);
    singular_min = [singular_min; s_min];
    s_max = maximum_singular_value(grasp_matrix);
    singular_max = [singular_max; s_max];
    v = volume_of_the_ellipsoid(grasp_matrix);
    volume = [volume; v];
    grasp_iso = grasp_isotropy_index(grasp_matrix);
    grasp_isotropy = [grasp_isotropy; grasp_iso];

    C5_pos = [C5_pos; C5];
    C5 = C5 + [0, 0.1];
end

%------------ Upper side edge ------------%
C5 = round(C5);
C5_theta = theta_upper;

while(C5(1) >= 0 && C5(1) <= 6)
    location_contact_points = [C1; C2; C3; C4; C5];
    angle_contact_points = [theta_lower; theta_left; theta_upper; theta_upper; C5_theta];
    grasp_matrix = generate_grasp_matrix(O, location_contact_points, angle_contact_points);

    s_min = minimum_singular_value(grasp_matrix);
    singular_min = [singular_min; s_min];
    s_max = maximum_singular_value(grasp_matrix);
    singular_max = [singular_max; s_max];
    v = volume_of_the_ellipsoid(grasp_matrix);
    volume = [volume; v];
    grasp_iso = grasp_isotropy_index(grasp_matrix);
    grasp_isotropy = [grasp_isotropy; grasp_iso];

    C5_pos = [C5_pos; C5];
    C5 = C5 + [-0.1, 0];
end

%------------ Left side edge ------------%
C5 = round(C5);
C5_theta = theta_left;

while(C5(2) >= 0 && C5(2) <= 3)
    location_contact_points = [C1; C2; C3; C4; C5];
    angle_contact_points = [theta_lower; theta_left; theta_upper; theta_upper; C5_theta];
    grasp_matrix = generate_grasp_matrix(O, location_contact_points, angle_contact_points);

    s_min = minimum_singular_value(grasp_matrix);
    singular_min = [singular_min; s_min];
    s_max = maximum_singular_value(grasp_matrix);
    singular_max = [singular_max; s_max];
    v = volume_of_the_ellipsoid(grasp_matrix);
    volume = [volume; v];
    grasp_iso = grasp_isotropy_index(grasp_matrix);
    grasp_isotropy = [grasp_isotropy; grasp_iso];

    C5_pos = [C5_pos; C5];
    C5 = C5 + [0, -0.1];
end
C5 = round(C5);

%------------ Bottom side edge ------------%
C5 = round(C5);
C5_theta = theta_lower;

while(C5(1) >= 0 && C5(1) <= 6)
    location_contact_points = [C1; C2; C3; C4; C5];
    angle_contact_points = [theta_lower; theta_left; theta_upper; theta_upper; C5_theta];
    grasp_matrix = generate_grasp_matrix(O, location_contact_points, angle_contact_points);

    s_min = minimum_singular_value(grasp_matrix);
    singular_min = [singular_min; s_min];
    s_max = maximum_singular_value(grasp_matrix);
    singular_max = [singular_max; s_max];
    v = volume_of_the_ellipsoid(grasp_matrix);
    volume = [volume; v];
    grasp_iso = grasp_isotropy_index(grasp_matrix);
    grasp_isotropy = [grasp_isotropy; grasp_iso];

    C5_pos = [C5_pos; C5];
    C5 = C5 + [0.1, 0];

end
C5 = round(C5);

%---------- Grasp Quality Matrix --------%

quality_matrix = [singular_min, volume, grasp_isotropy, singular_max];
disp(quality_matrix);

[p1, p2, p3] = run_quality_metrics(C5_pos, quality_matrix);

disp("Best Contact Point for 5 as per Q_MSV = ");
disp(p1);

disp("Best Contact Point for 5 as per Q_VEW = ");
disp(p2);

disp("Best Contact Point for 5 as per Q_GII = ");
disp(p3);

%------------ Plotting the data ---------------%

create_plot(C5_pos, location_contact_points, p1, quality_matrix(:,1), "Grasp Quality Value - MSV", "5th Vector Location - MSV");
create_plot(C5_pos, location_contact_points, p2, quality_matrix(:,2), "Grasp Quality Value - VEW", "5th Vector Location - VEW");
create_plot(C5_pos, location_contact_points, p3, quality_matrix(:,3), "Grasp Quality Value - GII", "5th Vector Location - GII");

%------------ User-Defined Functions -----------------------%

% Create a visualization
function create_plot(vector_locations, given_contact_locations, best_contact_location, Q, title1, title2)
    figure;
    plot3(vector_locations(:,1), vector_locations(:,2), Q);
    title(title1);
    figure;
    plot(vector_locations(:,1),vector_locations(:,2));
    xlim([-2,8]);
    ylim([-2,5]);
    hold on;
    scatter(best_contact_location(1), best_contact_location(2), "red");
    hold on;
    scatter(given_contact_locations([1:4],1), given_contact_locations([1:4],2), "magenta");
    title(title2);
    hold off;
end

% create a skew symmetric matrix from a 2D vector
function S = create_skew2d(vector)
    S =[-vector(2) vector(1)];
end

% calculate a grasp matrix for hard-contact between a gripper and a surface
function grasp = generate_grasp_matrix(object_center, contact_locations, contact_angles)
    hard_contact = [eye(2,2), zeros(2,1);
        zeros(1,2), 0];

    num_of_contacts = size(contact_locations, 1);
    grasp_prime = [];
    grasp = [];
    H = [];

    for itr = 1:num_of_contacts
        P = [eye(2), transpose(create_skew2d(contact_locations(itr,:)- object_center));
            zeros(1,2), 1];
        R = [cos(contact_angles(itr)), -sin(contact_angles(itr)), 0;
            sin(contact_angles(itr)), cos(contact_angles(itr)), 0;
            0,0,1];
        G_prime = pinv(R)*P;
        grasp_prime = [grasp_prime; G_prime];
        H = blkdiag(H, hard_contact);
    end
    grasp =   H * grasp_prime;
end

% refer to the research paper for more detail on this function
function Q_MSV_min = minimum_singular_value(G)
    singular_values = svd(G * transpose(G));
    non_zero_singular_values = round(singular_values, 4);

    msv_min = [];

    for i = 1: size(non_zero_singular_values)
        if(non_zero_singular_values(i) ~= 0)
            msv_min = [msv_min; non_zero_singular_values(i)];
        end
    end
    Q_MSV_min = min(msv_min);
end

% refer to the research paper for more detail on this function
function Q_MSV_max = maximum_singular_value(G)
    singular_values = svd(G * transpose(G));
    Q_MSV_max = max(singular_values);
end

% refer to the research paper for more detail on this function
function Q_VEW = volume_of_the_ellipsoid(G)
    Q_VEW = sqrt(det(transpose(G) * G));
end

% refer to the research paper for more detail on this function
function Q_GII = grasp_isotropy_index(G)
    Q_GII = minimum_singular_value(G)/maximum_singular_value(G);
end

% run a quality matrix and decide the best grasp
function [pos1, pos2, pos3] = run_quality_metrics(contact, Q)
    %Metric 1
    [M1, I1] = max(Q(:,1));
    pos1 = contact(I1,:);
    
    %Metric 2
    [M2, I2] = max(Q(:,2));
    pos2 = contact(I2,:);
    
    %Metric 3
    [M3, I3] = max(Q(:,3));
    pos3 = contact(I3,:);
end