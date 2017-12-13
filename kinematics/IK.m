% Function that takes in a transform matrix T and returns the Inverse
% Kinematic joint variable values

function joint_vars = IK(T)
%Link Lengths
lengths = [-0.08, 0.045, 0.135];

% Compute IK
t_pred = zeros(5,1);

for j = 1:size(t_pred,1)
    v = reshape(T, [16,1]);
    %nulls = find(abs(v)<eps);
    %v(nulls) = 0;
    t_pred(1) = thet1(v,lengths(3));
    [c4, t_pred(2)] = thet2(t_pred(1), v);
    
    t_pred(4) = thet4(t_pred(2), c4, v);
    %Recompute theta_1 to lie in correct range
    %t_pred(j,1) = recomp_thet1(c4, t_pred(j,2), v);
    
    t_pred(5) = thet5(t_pred(2), c4, v);
    
    t_pred(3) = thet3(t_pred(2), t_pred(4), t_pred(5), lengths, v);
end

% FK from IK predictions:
TIK = eye(4);

alphas = [pi/2 , pi/2 , 0 , pi/2 , pi/2];
ais = [0, l1, 0 , 0, l3];
di = [0,0, t_pred(3) , l2, 0];
thetas = t_pred;
thetas(3) = pi;

for i = 1:size(thetas,2)
    T_new = [cos(thetas(i)), -cos(alphas(i))*sin(thetas(i)), sin(alphas(i))*sin(thetas(i)), ais(i)*cos(thetas(i));
                     sin(thetas(i)), cos(alphas(i))*cos(thetas(i)), -sin(alphas(i))*cos(thetas(i)), ais(i)*sin(thetas(i));
                     0, sin(alphas(i)), cos(alphas(i)), di(i);
                     0, 0, 0, 1];
    TIK = TIK*T_new;
end

if rank(TIK)==4
    joint_vars = t_pred;
else
    msg = 'The Forward Kinematics Matrix is Singular. Cannot solve for joint angles.';
    error(msg);    
end

end


% Functions to Compute Inverse Kinematics terms based on analytical solution:
function t1 = thet1(v,l3)
r1y = v(2);
r1x = v(1);
py = v(14);
px = v(13);

%Range: [-pi,pi]
t1 = atan2(py - l3*r1y, px - l3*r1x);
end

function t1_re = recomp_thet1(c4, t2, v)
r2x = v(5);
r2y = v(6);
r2z = v(7);
A = [c4, cot(t2)*r2z; cot(t2)*r2z, -c4];
avec = A\[r2x; r2y];
t1_re = atan2(avec(1) , avec(2));
end

function [c4,t2] = thet2(t1, v)

r2x = v(5);
r2y = v(6);
r2z = v(7);

A = [sin(t1), cos(t1)*r2z; -cos(t1), sin(t1)*r2z];
avec = A\[r2x; r2y];

%cos(t4) is also found
c4 = avec(1);

%Range: [0,pi/2]
cand = atan2(1,avec(2));
if cand >0 && cand<pi/2
    t2 = cand;
elseif cand>pi/2 && cand<=pi
    t2 = -cand + pi;
elseif cand<-pi/2 && cand>-pi
    t2 = pi + cand;
else
    t2 = -cand;
end
end

function d3 = thet3(t2, t4, t5, l, v)
%Range: [0.33,0.45]

pz = v(15);
l1 = l(1);
l2 = l(2);
l3 = l(3);

d3 = (-pz + l1 - l3*cos(t2)*sin(t5) - l3*cos(t4)*cos(t5)*sin(t2) - l2*cos(t2))/cos(t2);
end

function t4 = thet4(t2, c4, v)
%Range: [-pi,pi]

r2z = v(7);
s4 = -r2z/sin(t2);

t4 = atan2(s4,c4);
end

function t5 = thet5(t2, c4, v)
%Range: [0,pi]
r1z = v(3);
r3z = v(11);

A = [-cos(t2), -c4*sin(t2); -c4*sin(t2), cos(t2)];
cvec = A\[r1z; r3z];

t5 = atan2(cvec(1), cvec(2));
end