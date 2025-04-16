clear;
clc;

%% read SBET
sbet_filename = 'sbet_Mission 1.out';
sbet = readSBET(sbet_filename);

%% llh = latitude longitude height
% Can use any point, for example:
% 1. first point of SBET
% 2. average of multiple SBET
% 3. known target coordinates
llh_0 = mean(sbet(:,2:4));
disp('== CRS origin ==')
disp(['Longitude: ' num2str(rad2deg(llh_0(1)), 15)]);
disp(['Latitude: ' num2str(rad2deg(llh_0(2)), 15)]);
disp(['Height: ' num2str(llh_0(3), 15)]);

%% LGF NED conversion
[sbet_ned, conversion_struct] = transform_sbet_to_ned_lgf(sbet, llh_0);

trajectory = [sbet_ned(:,2:4), sbet_ned(:,1)];

writeMatrixCsv('sbet_ned.txt', trajectory);