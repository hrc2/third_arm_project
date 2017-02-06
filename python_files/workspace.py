
% Transform matrices:
T = eye(4);

%Trans = matlabFunction(T);

%limits for workspace volume:
thet1 = linspace(0,2*pi,10);
thet2 = linspace(0,pi/2+pi/6,10);
dist3 = linspace(280,430,10);
thet4 = linspace(-pi,pi,10);

%Plot points
plotpts = zeros(3,1)




%Using random sampling for faster plots:
N = 1000;
for i = 1:N
    t1 = rand2val(-pi,pi);
    %t1 = rand2val(-pi,pi/2);
    %t2 = rand2val(-pi/2,pi/6);
    t2 = rand2val(0,pi/2+pi/6);
    %t2 = pi/3;
    d3 = rand2val(280,430);
    %d3 = 280;
    t4 = rand2val(-pi,pi);
    %t4 = 0;

    a = [112,0,0,0];
    %alph = [pi/2,pi/2,0,0];
    alph = [pi/2,pi/2,0,0];
    %d = [0,0,d3,106];
    d = [0,0,d3,106+430-280];
    t = [t1,t2,pi/2,t4];

    T = eye(4);
    for h = 1:4
        T = T * [cos(t(h)), -cos(alph(h))*sin(t(h)), sin(alph(h))*sin(t(h)), a(h)*cos(t(h));
        sin(t(h)), cos(alph(h))*sin(t(h)), -sin(alph(h))*cos(t(h)), a(h)*sin(t(h));
        0, sin(alph(h)), cos(alph(h)), d(h);
        0, 0, 0, 1];
    end

    plotpts = [plotpts, T(1:3,4)];
end