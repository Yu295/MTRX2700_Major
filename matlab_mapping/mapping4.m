clear;
clc;

width = 20; % sufficient width for person + rollator to pass through gap 9cm)
r = 5;  % length of rotating arm of PTU (cm)

n = 1;  % initialise index value for loop to store spherical coords
j = 1;  % initialise index values for finding distance between two points
k = 1;  % initialise index values
j2 = 1;  % initalise the index value for s gap finding
k2 = 1;

lidarData = readmatrix('lidarTest4.txt');    % read the sample lidar data4
sizeData = size(lidarData);    %size of the sample data

for i = 1:sizeData(1)
    
    % eliminate the ground readings
    dist(i) = lidarData(i,4)/10;   % store the 4th column of data as distance readings (cm)
    ground(i) = lidarData(i,5)/10; % store the 5th col of data as estimated ground value
    
    if  dist(i) < ground(i) && dist(i) < 273  %if the point is less than ground value, take the point
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

% plot the cartesian coordinates
scatter3(x,y,z,'filled');

% find the distance between two points on the same elevation
for i= 1:n-2
    
   cs(i) = sqrt((x(i+1)-x(i))^2 + (y(i+1)-y(i))^2);   % store the distance between points
    
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
   as(i) = sqrt(d2^2 + d1^2 - 2* d1 *d2 *cos(theta(i)));
   
   % find the front on distance between two points to decide whether user
   % can successfully navigate
   m(i)= d2 * sqrt(2 - 2*cos(theta(i)));
   
   % if points are measured at the same elevation, count the distance as a
   % potential gap
    if intEl(i) == intEl(i+1)
        
        % Every elevation scanned
        elRange(k) = intEl(i);
        
        % using cartesian coordinates, find the actual distance between two
        % successive points and store in col
        % s(j,k) = sqrt((x(i+1)-x(i))^2 + (y(i+1)-y(i))^2);   % store the distance
       if m(i) > width
          
         % front on width is sufficient and user can move through this gap 
         m2(j) = m(i);  % Every distance that qualifies
         angle(j) = theta(i); % every theta for every distance
         
         xnew(j) = x(i);
         ynew(j) = y(i);
         znew(j) = z(i);
         % store the larger angle in the second col
         if a1 > a2
             
           
            angleRangeM(j,1) = a2; 
            angleRangeM(j,2) = a1;
            
         else
             angleRangeM(j,1) = a1;
             angleRangeM(j,2) = a2;
             
         end
            
            elM(j) = intEl(i);      % elevation of each angle range
            
             % to find the amount the angle for each gap is larger than minimum

            allowance(j) = theta(i) - 2*atan(width/(2*sqrt(d2^2 - (m2(j)/2)^2)));
            
            j = j+1;  % loop m case
          
       elseif as(i) > width
           
       end
        
    else
       
    k = k + 1;
          
    end
    
     
   
end

% find minimum angle for each elevation
% from within a gap, directly upwards, is there a minimum of width?
% if base angle supports 80cm, angle tolerance allows 5 cm either side for
% next angle test
 scatter( angleRangeM(:,1), elM,'filled')

 hold on
 scatter(angleRangeM(:,2), elM)
