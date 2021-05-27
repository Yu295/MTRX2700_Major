clear;
clc;

width = 70; % sufficient width for the person to pass through 
r = 0.1; % the length of the rotating arm of the PTU (cm) 
j = 1;	% initialise loop values for s case
k = 1;  % initialise loop values for m case

lidarData = readmatrix('lidarTest5.txt');    % read the sample lidar data

sizeData = size(lidarData);    % size of the sample data


for i = 1:sizeData(1)
    elevation(i) = deg2rad(lidarData(i,1));    % store the first column of data as elevation (radians)
    azimuth(i) = deg2rad(lidarData(i,3));      % store the second column of data as azimuth (radians)
    distance(i) = lidarData(i,4)/1000;     % store the thrid col of data as distance (cm)
    
    % calculate the cartesian coordinates
    x(i) = distance(i)*cos(elevation(i))*cos(azimuth(i)) + r*sin(azimuth(i));
    y(i) = r*cos(azimuth(i)) - distance(i)*cos(elevation(i))*sin(azimuth(i));
    z(i) = distance(i)*sin(elevation(i));
    
end

% plot the cartesian coordinates
scatter3(x,y,z,'filled');
