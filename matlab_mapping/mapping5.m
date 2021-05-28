clear;
clc;

width = 7.5; % sufficient width for person + rollator to pass through gap (cm)
r = 5;  % length of rotating arm of PTU (cm)
tolerance = 0;  % range of distance that should not be within gap values
n = 1;  % loop index for reading values
k = 1;  % loop index for base gap
m = 1;  % loop invalid gap index

lidarData = readmatrix('lidarTest4.txt');    % read the sample lidar data4
sizeData = size(lidarData);    %size of the sample data

for i=1:sizeData(1)
    
    % read the distance values and the ground values
    dist(i) = lidarData(i,4)/10;   % store the 4th column of data as distance readings (cm)
    ground(i) = lidarData(i,5)/10; % store the 5th col of data as estimated ground value
    
    %if the point is less than ground value and less than expected lidar range, take the point
    if  dist(i) < ground(i) && dist(i) < 273 
        elevation(n) = deg2rad(lidarData(i,1));     % take 1st col as actual elevation in rads
        intEl(n) = deg2rad(lidarData(i,2));     % take 2nd col as intended elevation in rads
        azimuth(n) = deg2rad(lidarData(i,3));   % take 3rd col as azimuth in rads
        distance(n) = dist(i);      % store 4th col of data as distance
        
        n = n+1;        % increase index
        
    end
    
end

% Calculate cartesian coordinates 
for i = 1: (n-1)
    
   x(i) = distance(i)*cos(elevation(i))*cos(azimuth(i)) + r*sin(azimuth(i));
   y(i) = r*cos(azimuth(i)) - distance(i)*cos(elevation(i))*sin(azimuth(i));
   z(i) = distance(i)*sin(elevation(i)); 
    
end

% figure setup

% plot the points on a scatter graph
map = scatter3(x,y,z,'filled');

% Axes labels and title
xlabel('x (cm)','FontSize',14);
ylabel('y (cm)','FontSize',14);
zlabel('z (cm)','FontSize',14);
title('Scanned Data', 'FontSize', 16);


for i = 1:n-2
   
    % Cartesian distance between points used to compare to spherical calcs
   cartDist(i) = sqrt((x(i+1)-x(i))^2 + (y(i+1)-y(i))^2);
    
   % using the spherical coordinates, find the actual distance between two
   % successive points
   % First define the side distances and angles used
   d1 = sqrt((distance(i+1)*cos(elevation(i+1)))^2 + r^2);
   d2 = sqrt((distance(i)*cos(elevation(i)))^2 +r^2);
   a1 = azimuth(i+1);
   a2 = azimuth(i);
   
   % ensure that d1 is always the longer distance
   if d1<d2
       d1 = d2;
       d2 = sqrt((distance(i+1)*cos(elevation(i+1)))^2 + r^2);
       a1 = a2;
       a2 = azimuth(i+1);
   end
       

   % Then, define the theta in between
   theta(i) = abs(a1 - a2 + acos(r/d1) - acos(r/d2));
 
   % using cosine rule find the distance between two points
   azDist(i) = sqrt(d2^2 + d1^2 - 2* d1 *d2 *cos(theta(i)));
   
   % find the front on distance between two points to decide whether user
   % can successfully navigate
   frontDist(i)= d2 * sqrt(2 - 2*cos(theta(i)));
   
   % define the base elevation as the first elevation in the intended el
   % array
   baseEl = intEl(1);
    
    % If scans are made at base elevation
    if intEl(i) == baseEl
        
        % if the front on gap is wide enough to pass through
        if frontDist(i) > width
            
            % save the gap index and the gap distance
            gapIndex(k) = i;
            baseGap(k) = frontDist(i);
            
            % save the angle range of the gap
            if a1>a2
                angleRange(k,1) = a2;
                angleRange(k,2) = a1;
            else
                angleRange(k,1) = a1;
                angleRange(k,2) = a2;
            end
            
            % save the distance of the gap
            distLow(k) = d2 - tolerance;
            distHigh(k) = d1 + tolerance;
            
            % increase loop index
            k = k + 1;
            
        end
        
    end
    
    
end

% cycle through other elevations
for i = k:(n-1)
    
   for j=1:(k-1)
       
      % Remove gap indexes where there is an obstacle in the same angle
      % range and distance range as base gaps
      if azimuth(i)>angleRange(j,1) && azimuth(i) < angleRange(j,2) && distance(i) > distLow(j) && distance(i) < distHigh(j)
          gapIndex(j) = 0;
      end
      
   end
    
end