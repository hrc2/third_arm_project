clear all;
TIK = eye(4);

l1 = sym('l1');
l2 = sym('l2');
l3 = sym('l3');


alphas = [90 , 90 , 0 , 90 , 90, 0];
ais = [0, 0, 0 , 0, 0, l3];
di = [l1,0, sym('d3') , l2, 0, 0];
thetas = [sym('theta1'), sym('theta2'), 180, sym('theta4'), sym('theta5'), 0];

for i = 1:3
    T_new = [cosd(thetas(i)), -cosd(alphas(i))*sind(thetas(i)), sind(alphas(i))*sind(thetas(i)), ais(i)*cosd(thetas(i));
                     sind(thetas(i)), cosd(alphas(i))*cosd(thetas(i)), -sind(alphas(i))*cosd(thetas(i)), ais(i)*sind(thetas(i));
                     0, sind(alphas(i)), cosd(alphas(i)), di(i);
                     0, 0, 0, 1];
    TIK = TIK*T_new;
end

TIK2 = eye(4);
for i = 1:6
    T_new = [cosd(thetas(i)), -cosd(alphas(i))*sind(thetas(i)), sind(alphas(i))*sind(thetas(i)), ais(i)*cosd(thetas(i));
                     sind(thetas(i)), cosd(alphas(i))*cosd(thetas(i)), -sind(alphas(i))*cosd(thetas(i)), ais(i)*sind(thetas(i));
                     0, sind(alphas(i)), cosd(alphas(i)), di(i);
                     0, 0, 0, 1];
    TIK2 = TIK2*T_new;
end
% TIK2 = [sym('R1x'), sym('R2x'), sym('R3x'), sym('Px');
%         sym('R1y'), sym('R2y'), sym('R3y'), sym('Py');
%         sym('R1z'), sym('R2z'), sym('R3z'), sym('Pz');
%         0,             0,         0       , 1];
constraint = TIK\TIK2;


%Substitute values
lengths = [-0.08, 0.045, 0.135];
l1 = lengths(1);
l2 = lengths(2);
l3 = lengths(3);
d3 = 0.34; %needs to be between [0.33, 0.45]
ais_values = [0, 0, 0 , 0, 0, l3];
di_values = [l1,0, d3 , l2, 0, 0];
% thetas_values = [90, 45, 180, 0, 90, 0];
thetas_values = [45, 45, 180, 45, 45, 0];


T= vpa(subs(TIK2, [ais, di, thetas], [ais_values, di_values, thetas_values]))
IK(T)