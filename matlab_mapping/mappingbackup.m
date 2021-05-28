clear;
clc;

% DONT PLOT IF ITS THE GROUND THING AND LIMIT RANGE????
% Set actual width to look for
% Compare turn1 for side on angle with range of available front on angles
% to see if the path is clear
% How to give distance instructions (Physical instructions? Clicks as wheel turns? Handle grips to turn the clicks on). 
% Face forwards again after walking through obstacles
% weight angles so that they are valued closer to correct intended bearing
% manual override of intended bearing???

width = 70; % sufficient width for the person to pass through 
r = 10; % the length of the rotating arm of the PTU (cm) 
j = 1;	% initialise loop values for s case
k = 1;  % initialise loop values for m case

lidarData = readmatrix('lidarTest5.txt');    % read the sample lidar data

sizeData = size(lidarData);    % size of the sample data


for i = 1:sizeData(1)
    elevation(i) = deg2rad(lidarData(i,1));    % store the first column of data as elevation (radians)
    azimuth(i) = deg2rad(lidarData(i,3));      % store the second column of data as azimuth (radians)
    distance(i) = lidarData(i,4)/10;     % store the thrid col of data as distance (cm)
    
    % calculate the cartesian coordinates
    x(i) = distance(i)*cos(elevation(i))*cos(azimuth(i)) + r*sin(azimuth(i));
    y(i) = r*cos(azimuth(i)) - distance(i)*cos(elevation(i))*sin(azimuth(i));
    z(i) = distance(i)*sin(elevation(i));
    
end

% plot the cartesian coordinates
scatter3(x,y,z,'filled');

for i=1:(sizeData(1)-1)
   % using cartesian coordinates, find the actual distance between two
   % successive points
   s(i) = sqrt((x(i+1)-x(i))^2 + (y(i+1)-y(i))^2); 
   
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
     
   if m(i) > width
      % front on width is sufficient and user can move through this gap 
      mcase(k) = i; % store which loops satisfy m case
      
      turnAngleM(k) = (a1 + a2)/2;
      
      k = k+1;  % loop m case
      
   elseif s(i) > width
      % front on width insufficient but actual width sufficient 
      % calculate the distance and movements required to move through gap
      % head on
     
      % checking how many loops this occurs
     scase(j)= i;   % store which loops satisfy s case to check
     
     
     % from trig calcs
     sigma(j) = asin(d2 * sin(theta(i)) / as(i));
     c(j) = d1 * sin(sigma(j));
     a(j) = d1 * cos(sigma(j)) - 0.5 * s(i) ;
      
      % instructions
      if a1> a2
          turn1S(j) = a2 + sigma(j);   % angle from centre (positive and negative indicate direction)

          turn2S(j) = -90;    % turn right 90 deg instruction

      elseif a2>a1
          turn1S(j) = a2 - sigma(j);
          
          turn2S(j) = 90;   % turn left by 90 deg instruction
          
      end
      
      if turn1S(j) > 0
          
          turnAngleS(j) = turn1S(j) - atan(c(j)/a(j));  % net direction turned (used to measure how close the path is to the centre)
          
      elseif turn1S(j) < 0 
          
          turnAngleS(j) = turn1S(j) + atan(c(j)/a(j));
          
      end
      
      j = j+1;  % loop s case
   end
    
   
end

% Evaluate angles closest to centre
[angleS,indexS] = min(abs(turnAngleS));

[angleM,indexM] = min(abs(turnAngleM));

% If front on angle is close enough, take front on angle always (m)
% If front on angle is a large then consider if side on angle (s) is
% less than front on angle
if angleM < pi/12 || angleM < angleS  
    % angle chosen is the front on angle, set flag S to 0
    flagS = 0;
else
    % angle chosen is the side on angle, set flag S to 1
    flagS = 1;
end

% Issue instructions

if flagS == 0
    % if front on angle selected, issue instruction to move front on
    angleChosen = angleM;   % double check angle selected
    turn1 = rad2deg(turnAngleM(indexM));    % convert angle to degrees
else
    % if side on angle selected, issue series of instructions
    angleChosen = angleS; % double check angle selected 
    turn1 = turn1S(indexS); % convert angle to degrees 
    
    if turn1>0 
        % turn by this angle to the right
    else
        % turn by this angle to the left
    end
    
    % issue instruction to move forward by a
    move1 = a(indexS);
    turn2 = turn2S(indexS);
    move2 = c(indexS);
    
end
