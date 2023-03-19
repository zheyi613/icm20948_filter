device = serialport("COM6", 115200)
len = 1000;
mx = zeros(1,len);
my = zeros(1,len);
mz = zeros(1,len);

for ii = 1:len
    s = readline(device);
    data = strsplit(s,',');
    mx(ii) = str2double(data(1));
    my(ii) = str2double(data(2));
    mz(ii) = str2double(data(3));
end
%%
clear device
%%
x = mx';
y = my';
z = mz';
D = [x.*x y.*y z.*z x.*y x.*z y.*z x y z];
a = inv(D'*D)*D'*ones(size(x));
M = [a(1)   a(4)/2 a(5)/2;
     a(4)/2 a(2)   a(6)/2;
     a(5)/2 a(6)/2 a(3)  ];
center = -1/2*[a(7),a(8),a(9)]*inv(M);
SS = center*M*center'+1;
[U,V] = eig(M);
[~,n1] = max(abs(U(:,1)));
[~,n2] = max(abs(U(:,2)));
[~,n3] = max(abs(U(:,3)));
lambda(n1) = V(1,1);
lambda(n2) = V(2,2);
lambda(n3) = V(3,3);
scale_axis = [sqrt(SS/lambda(1)),sqrt(SS/lambda(2)),sqrt(SS/lambda(3))];
center = round(center);
scale_axis = round(scale_axis);

figure
plot3(x,y,z,'b.',center(1),center(2),center(3),'ro');
axis equal
xlabel('x');
ylabel('y');
zlabel('z');
